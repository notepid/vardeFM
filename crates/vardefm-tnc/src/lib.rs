//! VardeFM TNC: everything between the modem and a host application.
//!
//! - [`host`]: the VARA-compatible command protocol.
//! - [`server`]: TCP command and data ports.
//! - [`station`]: one modem (link + modulator + receiver) with its host notifications.
//! - [`simpair`]: two stations connected by a simulated FM radio link.
//! - [`radio`]: one station on a real radio (sound card + PTT).
//! - [`level`]: receive level meter and volume advice.
//!
//! The station and radio engine live in [`vardefm_engine`]; this crate adds the
//! TCP server, sound-card and PTT wiring, and the sim-pair virtual radio pair.

pub use vardefm_engine::{host, level, station, VERSION};

pub mod radio;
pub mod server;
pub mod simpair;
