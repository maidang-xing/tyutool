//! JieLi cryptographic algorithms & checksums.
//!
//! Implements:
//! 1. CRC8-Maxim (polynomial 0x31 / reversed 0x8C) used in serial bootloader sync.
//! 2. JieLi ENC / SFC stream cipher used for flash payload and filesystem metadata.

/// Calculate 8-bit CRC-Maxim (poly 0x31 / reversed 0x8C, init 0x00).
pub fn crc8_maxim(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            if (crc & 1) != 0 {
                crc = (crc >> 1) ^ 0x8C;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

/// Calculate CRC16-CCITT with custom initial CRC.
pub fn crc16_ccitt_init(data: &[u8], init_crc: u16) -> u16 {
    let mut crc = init_crc;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

const MENGLI_MAGIC: &[u8] = &[
    195, 207, 192, 232, 206, 210, 176, 174, 196, 227, 163, 172, 211, 241, 193, 214,
];

/// JieLi MengLi CrcDecode cipher for RAM loader upload.
pub fn jl_crc_cipher(buff: &mut [u8], key: u32) {
    let k_hi = ((key >> 16) & 0xFFFF) as u16;
    let k_lo = (key & 0xFFFF) as u16;
    let mut crc = crc16_ccitt_init(&k_hi.to_le_bytes(), k_lo);
    let mlen = MENGLI_MAGIC.len();
    for (i, b) in buff.iter_mut().enumerate() {
        let mi = i % mlen;
        crc = crc16_ccitt_init(&MENGLI_MAGIC[mi..mi + 1], crc);
        *b ^= (crc & 0xFF) as u8;
    }
}

/// JieLi ENC stream cipher.
pub fn jl_enc_cipher(buff: &mut [u8], key: u16) -> u16 {
    let mut k = key;
    for b in buff.iter_mut() {
        *b ^= (k & 0xFF) as u8;
        k = (k << 1) ^ if (k & 0x8000) != 0 { 0x1021 } else { 0 };
    }
    k
}

/// JieLi SFC block cipher (involutory XOR cipher).
pub fn jl_sfc_cipher(buff: &mut [u8], base: usize, key: u16, blocksize: usize) {
    if blocksize == 0 {
        return;
    }
    let mut i = 0;
    while i < buff.len() {
        let chunk_len = (buff.len() - i).min(blocksize);
        let chunk_key = key ^ (((i.wrapping_sub(base)) >> 2) as u16);
        jl_enc_cipher(&mut buff[i..i + chunk_len], chunk_key);
        i += blocksize;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc8_maxim_standard_vector() {
        assert_eq!(crc8_maxim(b"123456789"), 0xA1);
        assert_eq!(crc8_maxim(&[]), 0x00);
    }

    #[test]
    fn sfc_cipher_roundtrip() {
        let original = b"Hello World 1234567890!";
        let mut buf = original.to_vec();
        jl_sfc_cipher(&mut buf, 0, 0xFFFF, 32);
        let hex_str: String = buf.iter().map(|b| format!("{:02x}", b)).collect();
        assert_eq!(hex_str, "b7baf373701e2b9782adc747ff8f68a33a280b40e8b306");
        jl_sfc_cipher(&mut buf, 0, 0xFFFF, 32);
        assert_eq!(&buf, original);
    }

    #[test]
    fn mengli_crc_cipher_vector() {
        let original = b"Hello World 12345678901234567890";
        let mut buf = original.to_vec();
        jl_crc_cipher(&mut buf, 0xFFFFFFFF);
        assert_eq!(
            buf,
            vec![
                103, 193, 237, 160, 33, 225, 10, 133, 156, 206, 210, 134, 24, 184, 231, 195, 54,
                178, 220, 217, 246, 186, 187, 38, 142, 210, 232, 42, 231, 192, 127, 11
            ]
        );
    }
}
