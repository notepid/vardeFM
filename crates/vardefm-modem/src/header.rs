use vardefm_fec::crc::crc16;

/// Burst type.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Kind {
    /// Connection request; payload: one level-0 codeword with the call signs.
    ConnectReq = 0,
    /// Connection accepted; payload: one level-0 codeword with the call signs.
    ConnectAck = 1,
    /// Data and/or acknowledgements.
    #[default]
    Data = 2,
    /// Close the session (sent once all our data is acknowledged).
    Disc = 3,
    /// Session closed.
    DiscAck = 4,
}

impl Kind {
    fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::ConnectReq,
            1 => Self::ConnectAck,
            2 => Self::Data,
            3 => Self::Disc,
            4 => Self::DiscAck,
            _ => return None,
        })
    }
}

/// Burst header: 80 bits of fields, MSB first, followed by a CRC-16.
///
/// | bits | field |
/// |---|---|
/// | 3 | kind |
/// | 8 | session id |
/// | 3 | payload speed level |
/// | 5 | ncw: payload codewords (0-24) |
/// | 3 | req_level: level the sender asks the peer to use next |
/// | 3 | burst_id: sender's burst counter (mod 8) |
/// | 3 | ack_burst: burst_id of the last peer burst this one acknowledges |
/// | 24 | ack_offset: next byte offset expected from the peer (mod 2^24) |
/// | 24 | bitmap: bit i (LSB = codeword 0) set if codeword i of burst ack_burst decoded |
/// | 4 | flags: bit 0 = POLL (peer must answer even with nothing to send) |
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Header {
    pub kind: Kind,
    pub session: u8,
    pub level: u8,
    pub ncw: u8,
    pub req_level: u8,
    pub burst_id: u8,
    pub ack_burst: u8,
    pub ack_offset: u32,
    pub bitmap: u32,
    pub flags: u8,
}

pub const FLAG_POLL: u8 = 1;

const FIELDS: [usize; 10] = [3, 8, 3, 5, 3, 3, 3, 24, 24, 4];

impl Header {
    fn values(&self) -> [u64; 10] {
        [
            self.kind as u64,
            self.session as u64,
            self.level as u64,
            self.ncw as u64,
            self.req_level as u64,
            self.burst_id as u64,
            self.ack_burst as u64,
            self.ack_offset as u64,
            self.bitmap as u64,
            self.flags as u64,
        ]
    }

    /// 12 bytes: 10 bytes of fields + CRC-16 (big-endian).
    pub fn pack(&self) -> [u8; 12] {
        let mut v: u128 = 0;
        for (val, bits) in self.values().into_iter().zip(FIELDS) {
            v = (v << bits) | (val as u128 & ((1u128 << bits) - 1));
        }
        let mut out = [0u8; 12];
        out[..10].copy_from_slice(&v.to_be_bytes()[6..16]);
        let c = crc16(&out[..10]);
        out[10..].copy_from_slice(&c.to_be_bytes());
        out
    }

    pub fn unpack(bytes: &[u8]) -> Option<Self> {
        if bytes.len() != 12 || crc16(&bytes[..10]).to_be_bytes() != bytes[10..12] {
            return None;
        }
        let mut buf = [0u8; 16];
        buf[6..].copy_from_slice(&bytes[..10]);
        let mut v = u128::from_be_bytes(buf);
        let mut vals = [0u64; 10];
        for i in (0..10).rev() {
            vals[i] = (v & ((1u128 << FIELDS[i]) - 1)) as u64;
            v >>= FIELDS[i];
        }
        let h = Self {
            kind: Kind::from_u8(vals[0] as u8)?,
            session: vals[1] as u8,
            level: vals[2] as u8,
            ncw: vals[3] as u8,
            req_level: vals[4] as u8,
            burst_id: vals[5] as u8,
            ack_burst: vals[6] as u8,
            ack_offset: vals[7] as u32,
            bitmap: vals[8] as u32,
            flags: vals[9] as u8,
        };
        let max_level = crate::LEVELS.len() as u8;
        (h.level < max_level && h.req_level < max_level && h.ncw as usize <= crate::MAX_NCW).then_some(h)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_roundtrip() {
        let h = Header {
            kind: Kind::Data,
            session: 0xa7,
            level: 5,
            ncw: 17,
            req_level: 3,
            burst_id: 6,
            ack_burst: 1,
            ack_offset: 0x00ab_cdef,
            bitmap: 0x0012_3456,
            flags: FLAG_POLL,
        };
        let p = h.pack();
        assert_eq!(Header::unpack(&p), Some(h));
        let mut bad = p;
        bad[3] ^= 1;
        assert_eq!(Header::unpack(&bad), None);
    }
}
