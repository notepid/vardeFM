//! CRC-16/CCITT-FALSE and CRC-32 (IEEE 802.3), bitwise implementations.

/// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final XOR.
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
        }
    }
    crc
}

/// CRC-32 (IEEE 802.3, as used by zip/Ethernet): reflected poly 0xEDB88320,
/// init 0xFFFFFFFF, final XOR 0xFFFFFFFF.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xffff_ffff;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xedb8_8320 } else { crc >> 1 };
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_values() {
        assert_eq!(crc16(b"123456789"), 0x29b1);
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
    }
}
