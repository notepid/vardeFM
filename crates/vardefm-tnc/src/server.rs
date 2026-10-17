//! TCP command and data ports.
//!
//! One client per port at a time; a new connection replaces the old one.
//! Input from the host arrives on [`HostPort::inbox`]; output is written with
//! [`HostPort::send_line`] and [`HostPort::send_data`].

use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

#[derive(Debug)]
pub enum HostIn {
    /// One command line (without the terminator).
    Line(String),
    /// Bytes written to the data port.
    Data(Vec<u8>),
    /// The command-port client went away.
    CmdClosed,
}

type Slot = Arc<Mutex<Option<TcpStream>>>;

pub struct HostPort {
    pub cmd_addr: SocketAddr,
    pub data_addr: SocketAddr,
    pub inbox: Receiver<HostIn>,
    cmd_out: Slot,
    data_out: Slot,
    stop: Arc<AtomicBool>,
}

impl HostPort {
    /// Listen on `host:cmd_port` and `host:data_port` (port 0 picks a free port).
    pub fn bind(host: &str, cmd_port: u16, data_port: u16) -> io::Result<Self> {
        let cmd = TcpListener::bind((host, cmd_port))?;
        let data = TcpListener::bind((host, data_port))?;
        let (tx, inbox) = channel();
        let stop = Arc::new(AtomicBool::new(false));
        let port = Self {
            cmd_addr: cmd.local_addr()?,
            data_addr: data.local_addr()?,
            inbox,
            cmd_out: Arc::default(),
            data_out: Arc::default(),
            stop: stop.clone(),
        };
        spawn_acceptor(cmd, port.cmd_out.clone(), tx.clone(), stop.clone(), true);
        spawn_acceptor(data, port.data_out.clone(), tx, stop, false);
        Ok(port)
    }

    /// Send one line to the command client (a CR is appended).
    pub fn send_line(&self, line: &str) {
        write_to(&self.cmd_out, format!("{line}\r").as_bytes());
    }

    pub fn send_data(&self, data: &[u8]) {
        write_to(&self.data_out, data);
    }

    pub fn shutdown(&self) {
        self.stop.store(true, Ordering::Relaxed);
        for slot in [&self.cmd_out, &self.data_out] {
            if let Some(s) = slot.lock().unwrap().take() {
                let _ = s.shutdown(std::net::Shutdown::Both);
            }
        }
    }
}

impl Drop for HostPort {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn write_to(slot: &Slot, bytes: &[u8]) {
    let mut guard = slot.lock().unwrap();
    if let Some(s) = guard.as_mut() {
        if s.write_all(bytes).is_err() {
            *guard = None;
        }
    }
}

fn spawn_acceptor(listener: TcpListener, slot: Slot, tx: Sender<HostIn>, stop: Arc<AtomicBool>, is_cmd: bool) {
    listener.set_nonblocking(true).expect("nonblocking listener");
    thread::spawn(move || {
        while !stop.load(Ordering::Relaxed) {
            match listener.accept() {
                Ok((stream, _)) => {
                    let _ = stream.set_nonblocking(false);
                    let _ = stream.set_nodelay(true);
                    let _ = stream.set_write_timeout(Some(Duration::from_secs(5)));
                    let Ok(reader) = stream.try_clone() else { continue };
                    if let Some(old) = slot.lock().unwrap().replace(stream) {
                        let _ = old.shutdown(std::net::Shutdown::Both);
                    }
                    let tx = tx.clone();
                    let slot = slot.clone();
                    thread::spawn(move || read_loop(reader, tx, is_cmd, slot));
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => thread::sleep(Duration::from_millis(20)),
                Err(_) => thread::sleep(Duration::from_millis(100)),
            }
        }
    });
}

fn read_loop(mut stream: TcpStream, tx: Sender<HostIn>, is_cmd: bool, slot: Slot) {
    let me = stream.peer_addr().ok();
    let mut buf = [0u8; 8192];
    let mut line = Vec::new();
    loop {
        let n = match stream.read(&mut buf) {
            Ok(0) | Err(_) => break,
            Ok(n) => n,
        };
        if !is_cmd {
            if tx.send(HostIn::Data(buf[..n].to_vec())).is_err() {
                return;
            }
            continue;
        }
        for &b in &buf[..n] {
            if b == b'\r' || b == b'\n' {
                if !line.is_empty() {
                    let s = String::from_utf8_lossy(&line).into_owned();
                    line.clear();
                    if tx.send(HostIn::Line(s)).is_err() {
                        return;
                    }
                }
            } else {
                line.push(b);
            }
        }
    }
    // Only report a close if this is still the current client (not one replaced by a newer connection).
    let mut guard = slot.lock().unwrap();
    if guard.as_ref().is_some_and(|s| s.peer_addr().ok() == me) {
        *guard = None;
        if is_cmd {
            let _ = tx.send(HostIn::CmdClosed);
        }
    }
}
