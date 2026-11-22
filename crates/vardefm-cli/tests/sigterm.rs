//! A stop request from the system must release PTT. SIGTERM is what `systemctl stop`,
//! `kill` and a system shutdown send.
#![cfg(unix)]

use std::io::{BufRead, BufReader, Write};
use std::net::TcpListener;
use std::process::{Child, Command, Stdio};
use std::sync::mpsc;
use std::time::Duration;

const WAIT: Duration = Duration::from_secs(10);

/// Kills the program if the test fails before it has exited.
struct Running(Child);

impl Drop for Running {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

#[test]
fn sigterm_releases_ptt() {
    // Stands in for Hamlib rigctld: accepts every command and passes it to the test.
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let (tx, commands) = mpsc::channel();
    std::thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        let mut reply = stream.try_clone().unwrap();
        for line in BufReader::new(stream).lines() {
            let Ok(line) = line else { break };
            if reply.write_all(b"RPRT 0\n").is_err() || tx.send(line).is_err() {
                break;
            }
        }
    });

    let mut vardefm = Running(
        Command::new(env!("CARGO_BIN_EXE_vardefm"))
            .args(["ptt-test", "--ptt", &format!("rigctld:127.0.0.1:{port}"), "--seconds", "30"])
            .stdout(Stdio::null())
            .spawn()
            .unwrap(),
    );
    // Opening the PTT unkeys it; then ptt-test keys the radio.
    assert_eq!(commands.recv_timeout(WAIT).as_deref(), Ok("T 0"));
    assert_eq!(commands.recv_timeout(WAIT).as_deref(), Ok("T 1"));

    // `kill` is built into every POSIX shell, so this needs no separate kill program.
    let kill = format!("kill -TERM {}", vardefm.0.id());
    assert!(Command::new("sh").args(["-c", &kill]).status().unwrap().success());

    // Without a SIGTERM handler the program dies here and the connection just closes.
    assert_eq!(commands.recv_timeout(WAIT).as_deref(), Ok("T 0"), "PTT was not released on SIGTERM");
    assert!(vardefm.0.wait().unwrap().success(), "vardefm did not exit cleanly");
}
