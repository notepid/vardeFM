//! VardeFM station and radio engine without desktop I/O.
//!
//! - [`host`]: the host-compatible command protocol.
//! - [`station`]: one modem (link + modulator + receiver) with its host notifications.
//! - [`engine`]: one station stepped from the caller's audio and clock (`RadioEngine`).
//! - [`level`]: receive level meter and volume advice.
//!
//! The engine has no threads of its own and no wall clock. Time advances only when
//! the caller feeds samples through [`RadioEngine::step`] (or later,
//! [`RadioEngine::advance`]). Push-to-talk and sound-card I/O sit behind traits so
//! an embedding host can drive them from its own audio queue.

pub mod engine;
pub mod host;
pub mod level;
pub mod station;

pub use engine::{AudioSink, EngineConfig, HostIn, Ptt, RadioEngine, Report, LEVEL_WINDOW_S, RATE};
pub use station::{HostOut, Station, TxStart};

macro_rules! version {
    () => {
        concat!("VardeFM ", env!("CARGO_PKG_VERSION"))
    };
}

pub const VERSION: &str = version!();

/// [`VERSION`] as a C string, for the C ABI.
pub const VERSION_CSTR: &std::ffi::CStr =
    match std::ffi::CStr::from_bytes_with_nul(concat!(version!(), "\0").as_bytes()) {
        Ok(s) => s,
        Err(_) => panic!("version has a NUL byte"),
    };

/// Seed for the link's session ids and retry delays. Two stations with the same seed
/// draw the same ids, and the process id alone repeats between stations started from
/// one image.
pub fn random_seed() -> u32 {
    use std::hash::{BuildHasher, Hasher};
    // RandomState is keyed from the operating system's randomness.
    let mut h = std::collections::hash_map::RandomState::new().build_hasher();
    h.write_u32(std::process::id());
    if let Ok(t) = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH) {
        h.write_u128(t.as_nanos());
    }
    let x = h.finish();
    (x ^ (x >> 32)) as u32
}
