//! TCP command and data ports.
//!
//! One client per port at a time; a new connection replaces the old one.
//! Input from the host arrives on [`HostPort::inbox`] in one ordered stream;
//! output is written with [`HostPort::send_line`] and [`HostPort::send_data`].
//!
//! Ordering across the two sockets: a host that writes data and then sends
//! `DISCONNECT` expects that data to be sent. The two sockets are read by
//! different threads, so the command could overtake the data. To prevent that,
//! the command reader holds `DISCONNECT` back until the data reader has found
//! its socket empty at some point after the command arrived. Everything written
//! before the command is then already in the inbox, ahead of it.

use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

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

/// How often the data reader checks for an empty socket.
const DATA_POLL: Duration = Duration::from_millis(5);
/// Longest a DISCONNECT is held back waiting for the data reader.
const SYNC_LIMIT: Duration = Duration::from_millis(500);

/// Shared between the data reader and the command reader.
struct DataSync {
    epoch: Instant,
    /// Microseconds since `epoch` when the data reader last found its socket empty.
    empty_at: AtomicU64,
    /// A data client is connected.
    connected: AtomicBool,
}

impl DataSync {
    fn now_us(&self) -> u64 {
        self.epoch.elapsed().as_micros() as u64
    }

    /// Wait until the data reader has drained its socket after this call began.
    fn wait_drained(&self) {
        if !self.connected.load(Ordering::Acquire) {
            return;
        }
        let t = self.now_us();
        let start = Instant::now();
        while self.empty_at.load(Ordering::Acquire) <= t && start.elapsed() < SYNC_LIMIT {
            thread::sleep(Duration::from_millis(1));
        }
    }
}

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
        let sync = Arc::new(DataSync { epoch: Instant::now(), empty_at: AtomicU64::new(0), connected: AtomicBool::new(false) });
        let port = Self {
            cmd_addr: cmd.local_addr()?,
            data_addr: data.local_addr()?,
            inbox,
            cmd_out: Arc::default(),
            data_out: Arc::default(),
            stop: stop.clone(),
        };
        spawn_acceptor(cmd, port.cmd_out.clone(), tx.clone(), stop.clone(), sync.clone(), true);
        spawn_acceptor(data, port.data_out.clone(), tx, stop, sync, false);
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

fn spawn_acceptor(listener: TcpListener, slot: Slot, tx: Sender<HostIn>, stop: Arc<AtomicBool>, sync: Arc<DataSync>, is_cmd: bool) {
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
                    let (tx, slot, sync) = (tx.clone(), slot.clone(), sync.clone());
                    if is_cmd {
                        thread::spawn(move || cmd_loop(reader, tx, slot, sync));
                    } else {
                        sync.connected.store(true, Ordering::Release);
                        thread::spawn(move || data_loop(reader, tx, slot, sync));
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => thread::sleep(Duration::from_millis(20)),
                Err(_) => thread::sleep(Duration::from_millis(100)),
            }
        }
    });
}

/// True if this is still the current client; if so, clear the slot.
fn release(slot: &Slot, me: Option<SocketAddr>) -> bool {
    let mut guard = slot.lock().unwrap();
    let current = guard.as_ref().is_some_and(|s| s.peer_addr().ok() == me);
    if current {
        *guard = None;
    }
    current
}

fn data_loop(mut stream: TcpStream, tx: Sender<HostIn>, slot: Slot, sync: Arc<DataSync>) {
    let me = stream.peer_addr().ok();
    let _ = stream.set_read_timeout(Some(DATA_POLL));
    let mut buf = [0u8; 8192];
    loop {
        match stream.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => {
                if tx.send(HostIn::Data(buf[..n].to_vec())).is_err() {
                    return;
                }
            }
            Err(e) if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) => {
                // Socket empty: everything written so far is in the inbox.
                sync.empty_at.store(sync.now_us(), Ordering::Release);
            }
            Err(_) => break,
        }
    }
    if release(&slot, me) {
        sync.connected.store(false, Ordering::Release);
    }
}

fn cmd_loop(mut stream: TcpStream, tx: Sender<HostIn>, slot: Slot, sync: Arc<DataSync>) {
    let me = stream.peer_addr().ok();
    let mut buf = [0u8; 8192];
    let mut line = Vec::new();
    loop {
        let n = match stream.read(&mut buf) {
            Ok(0) | Err(_) => break,
            Ok(n) => n,
        };
        for &b in &buf[..n] {
            if b == b'\r' || b == b'\n' {
                if !line.is_empty() {
                    let s = String::from_utf8_lossy(&line).into_owned();
                    line.clear();
                    if s.trim().eq_ignore_ascii_case("DISCONNECT") {
                        sync.wait_drained();
                    }
                    if tx.send(HostIn::Line(s)).is_err() {
                        return;
                    }
                }
            } else {
                line.push(b);
            }
        }
    }
    if release(&slot, me) {
        let _ = tx.send(HostIn::CmdClosed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A host writes data and then DISCONNECT on the other socket. Every data byte
    /// must reach the inbox before the DISCONNECT line.
    #[test]
    fn disconnect_never_overtakes_earlier_data() {
        let port = HostPort::bind("127.0.0.1", 0, 0).unwrap();
        let mut cmd = TcpStream::connect(port.cmd_addr).unwrap();
        let mut data = TcpStream::connect(port.data_addr).unwrap();
        thread::sleep(Duration::from_millis(100));
        let block = vec![0x5a_u8; 1 << 20];
        for round in 0..5 {
            data.write_all(&block).unwrap();
            cmd.write_all(b"DISCONNECT\r").unwrap();
            let mut got = 0;
            loop {
                match port.inbox.recv_timeout(Duration::from_secs(5)).expect("inbox stalled") {
                    HostIn::Data(d) => got += d.len(),
                    HostIn::Line(l) => {
                        assert_eq!(l, "DISCONNECT");
                        assert_eq!(got, block.len(), "round {round}: DISCONNECT overtook data");
                        break;
                    }
                    HostIn::CmdClosed => panic!("command port closed"),
                }
            }
        }
    }
}
