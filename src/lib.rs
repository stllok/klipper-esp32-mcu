//! Klipper MCU wire protocol primitives (host-testable, `no_std`).
//! Spec: docs/SPEC.md §4. Reference: https://www.klipper3d.org/Protocol.html
#![no_std]

pub mod dict_defs;
pub mod io;
pub mod mcu;
pub mod motion;

pub const MESSAGE_MIN: usize = 5;
pub const MESSAGE_MAX: usize = 64;
pub const MESSAGE_HEADER_SIZE: usize = 2;
pub const MESSAGE_TRAILER_SIZE: usize = 3;
pub const MESSAGE_DEST: u8 = 0x10;
pub const MESSAGE_SEQ_MASK: u8 = 0x0f;
pub const MESSAGE_SYNC: u8 = 0x7e;

/// CRC16-CCITT exactly as Klipper `crc16_ccitt()` (init 0xffff, reflected).
pub fn crc16_ccitt(buf: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    for &b in buf {
        let mut d = b ^ (crc as u8);
        d ^= d << 4;
        let d = d as u16;
        crc = ((d << 8) | (crc >> 8)) ^ (d >> 4) ^ (d << 3);
    }
    crc
}

/// Append Klipper VLQ encoding of `v`; returns bytes written.
pub fn encode_vlq(out: &mut [u8], v: u32) -> usize {
    let sv = v as i32;
    let mut n = 0;
    // Range bit b = shift-2 checks whether a 7-bit group at `shift` is needed.
    for shift in [28, 21, 14, 7] {
        let b = shift - 2;
        if !(-(1 << b)..(3 << b)).contains(&sv) {
            out[n] = ((sv >> shift) & 0x7f) as u8 | 0x80;
            n += 1;
        }
    }
    out[n] = (sv & 0x7f) as u8;
    n + 1
}

/// Decode one VLQ from `buf`; returns (value, bytes consumed).
pub fn decode_vlq(buf: &[u8]) -> Option<(u32, usize)> {
    let first = *buf.first()?;
    let mut v = (first & 0x7f) as u32;
    if first & 0x60 == 0x60 {
        v |= !0x1f; // sign extend
    }
    let mut i = 0;
    let mut c = first;
    while c & 0x80 != 0 {
        i += 1;
        c = *buf.get(i)?;
        v = (v << 7) | (c & 0x7f) as u32;
    }
    Some((v, i + 1))
}

#[derive(Debug, PartialEq, Eq)]
pub enum Frame {
    /// Not enough bytes yet.
    Incomplete,
    /// Drop this many bytes and resync (bad len/crc/sync).
    Skip(usize),
    /// Valid block of `len` bytes; payload is `buf[2..len-3]`.
    Ok { len: usize, seq: u8 },
}

/// Inspect start of rx buffer for one message block (mirrors `command_find_block`).
pub fn find_block(buf: &[u8]) -> Frame {
    let Some(&len) = buf.first() else { return Frame::Incomplete };
    let len = len as usize;
    if buf[0] == MESSAGE_SYNC {
        return Frame::Skip(1);
    }
    if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&len) {
        return Frame::Skip(skip_to_sync(buf));
    }
    if buf.len() < len {
        return Frame::Incomplete;
    }
    let seq = buf[1];
    let crc = crc16_ccitt(&buf[..len - MESSAGE_TRAILER_SIZE]);
    let got = u16::from_be_bytes([buf[len - 3], buf[len - 2]]);
    if seq & !MESSAGE_SEQ_MASK != MESSAGE_DEST || crc != got || buf[len - 1] != MESSAGE_SYNC {
        return Frame::Skip(skip_to_sync(buf));
    }
    Frame::Ok { len, seq: seq & MESSAGE_SEQ_MASK }
}

fn skip_to_sync(buf: &[u8]) -> usize {
    buf.iter().position(|&b| b == MESSAGE_SYNC).map_or(buf.len(), |p| p + 1)
}

/// Wrap `payload` into a block with sequence `seq`; returns total length.
pub fn encode_block(out: &mut [u8], seq: u8, payload: &[u8]) -> usize {
    let len = payload.len() + MESSAGE_MIN;
    assert!(len <= MESSAGE_MAX && out.len() >= len);
    out[0] = len as u8;
    out[1] = MESSAGE_DEST | (seq & MESSAGE_SEQ_MASK);
    out[2..2 + payload.len()].copy_from_slice(payload);
    let crc = crc16_ccitt(&out[..len - 3]);
    out[len - 3..len - 1].copy_from_slice(&crc.to_be_bytes());
    out[len - 1] = MESSAGE_SYNC;
    len
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_check_value() {
        // CRC-16/MCRF4XX check value == Klipper crc16_ccitt
        assert_eq!(crc16_ccitt(b"123456789"), 0x6f91);
    }

    #[test]
    fn vlq_roundtrip_and_sizes() {
        let cases: [(i64, usize); 10] = [
            (0, 1), (-32, 1), (95, 1), (96, 2), (-4096, 2), (12287, 2),
            (1572863, 3), (201326591, 4), (-2147483648, 5), (201326592, 5),
        ];
        for (v, size) in cases {
            let mut b = [0u8; 5];
            let n = encode_vlq(&mut b, v as u32);
            assert_eq!(n, size, "size of {v}");
            assert_eq!(decode_vlq(&b[..n]), Some((v as u32, n)), "value {v}");
        }
    }

    #[test]
    fn block_roundtrip_and_resync() {
        let mut b = [0u8; 64];
        let n = encode_block(&mut b, 3, &[1, 2, 3]);
        assert_eq!(find_block(&b[..n]), Frame::Ok { len: n, seq: 3 });
        assert_eq!(find_block(&b[..n - 1]), Frame::Incomplete);
        b[3] ^= 0xff; // corrupt payload -> skip through trailing sync
        assert_eq!(find_block(&b[..n]), Frame::Skip(n));
        assert_eq!(find_block(&[MESSAGE_SYNC]), Frame::Skip(1));
    }
}
