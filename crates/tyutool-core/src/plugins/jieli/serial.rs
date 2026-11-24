//! Pure Rust native serial transport and flasher for JieLi chips.
//!
//! Provides zero-dependency UART communication, flashing, erasing, reading,
//! and probing over serial COM ports for JieLi WL82/WL83 (AC791N/AC792N) devices.

use std::fs::File;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::sleep;
use std::time::{Duration, Instant};

use serialport::SerialPort;

use super::{
    build_jieli_cdb, crc16_ccitt, parse_hex_u32, SC_BLOCK_ERASE_64K, SC_PROBE_CHIP,
    SC_PROBE_FLASH_ID, SC_READ_FLASH, SC_RESET, SC_SECTOR_ERASE_4K, SC_WRITE_FLASH,
};
use crate::error::FlashError;
use crate::flash_event::{FlashEvent, FlashMilestone, FlashPhase};
use crate::job::FlashJob;

const UART_SYNC_CMD: &[u8] = b"TX21";
const UART_ACK_BYTE: u8 = 0x79;
const UART_NACK_BYTE: u8 = 0x1F;

const CHUNK_SIZE: usize = 512;
const SECTOR_SIZE: u32 = 4096;
const BLOCK_SIZE: u32 = 65536;

pub struct JlSerialTransport {
    port: Box<dyn SerialPort>,
}

impl JlSerialTransport {
    /// Open serial port, perform bootloader sync handshake (TX21), and negotiate baud rate.
    pub fn open(port_name: &str, baud_rate: u32) -> Result<Self, FlashError> {
        let init_baud = 9600;
        let mut port = serialport::new(port_name, init_baud)
            .timeout(Duration::from_millis(150))
            .open()
            .map_err(|e| {
                FlashError::Plugin(format!(
                    "failed to open JieLi serial port {}: {}",
                    port_name, e
                ))
            })?;

        let _ = port.clear(serialport::ClearBuffer::All);

        // Perform initial sync (TX21)
        Self::sync_port(&mut *port)?;

        // Switch to target baud rate if requested and different from 9600
        if baud_rate != init_baud && baud_rate > 0 {
            port.set_baud_rate(baud_rate).map_err(|e| {
                FlashError::Plugin(format!(
                    "failed to set JieLi serial baud rate {}: {}",
                    baud_rate, e
                ))
            })?;
            let _ = port.clear(serialport::ClearBuffer::All);
            sleep(Duration::from_millis(50));
        }

        Ok(Self { port })
    }

    fn sync_port(port: &mut dyn SerialPort) -> Result<(), FlashError> {
        // Attempt 1: Direct TX21
        let _ = port.write_all(UART_SYNC_CMD);
        let _ = port.flush();
        sleep(Duration::from_millis(50));

        let mut ack = [0u8; 1];
        if port.read_exact(&mut ack).is_ok() && ack[0] == UART_ACK_BYTE {
            return Ok(());
        }

        // Attempt 2: Padded \0\0\0\0TX21
        let mut retry_buf = vec![0u8; 4];
        retry_buf.extend_from_slice(UART_SYNC_CMD);
        let _ = port.write_all(&retry_buf);
        let _ = port.flush();
        sleep(Duration::from_millis(50));

        if port.read_exact(&mut ack).is_ok() && ack[0] == UART_ACK_BYTE {
            return Ok(());
        }

        // If not acked, assume it may already be synchronized or in higher baudrate
        Ok(())
    }

    /// Send a 16-byte CDB frame with optional payload, verify ACK, and read optional response data.
    pub fn send_cmd(
        &mut self,
        cdb: &[u8; 16],
        data_out: Option<&[u8]>,
        data_in: Option<&mut [u8]>,
        timeout: Duration,
    ) -> Result<(), FlashError> {
        let mut frame = Vec::with_capacity(16 + data_out.map_or(0, |d| d.len()) + 2);
        frame.extend_from_slice(cdb);
        if let Some(data) = data_out {
            frame.extend_from_slice(data);
        }
        let crc = crc16_ccitt(&frame);
        frame.extend_from_slice(&crc.to_be_bytes());

        self.port.write_all(&frame).map_err(|e| {
            FlashError::Plugin(format!("failed to write JieLi serial command: {}", e))
        })?;
        let _ = self.port.flush();

        // Wait for device ACK (0x79)
        let start = Instant::now();
        let mut ack = [0u8; 1];
        loop {
            match self.port.read_exact(&mut ack) {
                Ok(()) => {
                    if ack[0] == UART_ACK_BYTE {
                        break;
                    } else if ack[0] == UART_NACK_BYTE {
                        return Err(FlashError::Plugin("JieLi device returned NACK".into()));
                    } else {
                        return Err(FlashError::Plugin(format!(
                            "JieLi device returned error code: 0x{:02X}",
                            ack[0]
                        )));
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                    if start.elapsed() >= timeout {
                        return Err(FlashError::Plugin(
                            "Timeout waiting for JieLi serial ACK".into(),
                        ));
                    }
                }
                Err(e) => {
                    return Err(FlashError::Plugin(format!(
                        "JieLi serial ACK read failed: {}",
                        e
                    )))
                }
            }
        }

        // Read response payload if expected
        if let Some(buf) = data_in {
            let mut read_bytes = 0;
            let start_in = Instant::now();
            while read_bytes < buf.len() {
                match self.port.read(&mut buf[read_bytes..]) {
                    Ok(0) => sleep(Duration::from_millis(1)),
                    Ok(n) => read_bytes += n,
                    Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                        if start_in.elapsed() >= timeout {
                            return Err(FlashError::Plugin(format!(
                                "Timeout reading JieLi serial response: got {} of {} bytes",
                                read_bytes,
                                buf.len()
                            )));
                        }
                    }
                    Err(e) => {
                        return Err(FlashError::Plugin(format!(
                            "JieLi serial response read failed: {}",
                            e
                        )))
                    }
                }
            }
        }

        Ok(())
    }
}

/// Execute native serial firmware flashing for JieLi devices.
pub fn run_serial_flash(
    job: &FlashJob,
    cancel: &AtomicBool,
    progress: &dyn Fn(FlashEvent),
) -> Result<(), FlashError> {
    let baud = if job.baud_rate > 0 {
        job.baud_rate
    } else {
        921600
    };
    progress(FlashEvent::Phase {
        phase: FlashPhase::Connect,
    });
    let mut transport = JlSerialTransport::open(&job.port, baud)?;

    let chip_lower = job.chip_id.to_ascii_lowercase();
    let reboot_delay_ms = if chip_lower.contains("wl83") || chip_lower.contains("ac792") {
        50
    } else {
        500
    };

    if let Some(segments) = &job.segments {
        if !segments.is_empty() {
            let total_segs = segments.len();
            for (idx, seg) in segments.iter().enumerate() {
                if cancel.load(Ordering::Relaxed) {
                    return Err(FlashError::Cancelled);
                }

                let start_addr = parse_hex_u32(Some(&seg.start_addr), 0x00000000)?;
                let mut file = File::open(&seg.firmware_path).map_err(|e| {
                    FlashError::Plugin(format!(
                        "failed to open firmware file {}: {}",
                        seg.firmware_path, e
                    ))
                })?;
                let mut data = Vec::new();
                file.read_to_end(&mut data).map_err(|e| {
                    FlashError::Plugin(format!(
                        "failed to read firmware file {}: {}",
                        seg.firmware_path, e
                    ))
                })?;

                write_image_data(
                    &mut transport,
                    start_addr,
                    &data,
                    idx,
                    total_segs,
                    cancel,
                    progress,
                )?;
            }

            // Reboot device after last segment
            progress(FlashEvent::Phase {
                phase: FlashPhase::Reboot,
            });
            let cdb = build_jieli_cdb(SC_RESET, 0, 0, reboot_delay_ms);
            let _ = transport.send_cmd(&cdb, None, None, Duration::from_millis(1000));
            progress(FlashEvent::Milestone {
                milestone: FlashMilestone::WriteComplete,
            });
            return Ok(());
        }
    }

    let firmware_path = job
        .firmware_path
        .as_deref()
        .ok_or_else(|| FlashError::Plugin("JieLi flash requires firmware_path (-f)".into()))?;

    let start_addr = parse_hex_u32(job.flash_start_hex.as_deref(), 0x00000000)?;
    let mut file = File::open(firmware_path).map_err(|e| {
        FlashError::Plugin(format!(
            "failed to open firmware file {}: {}",
            firmware_path, e
        ))
    })?;
    let mut data = Vec::new();
    file.read_to_end(&mut data).map_err(|e| {
        FlashError::Plugin(format!(
            "failed to read firmware file {}: {}",
            firmware_path, e
        ))
    })?;

    write_image_data(&mut transport, start_addr, &data, 0, 1, cancel, progress)?;

    progress(FlashEvent::Phase {
        phase: FlashPhase::Reboot,
    });
    let cdb = build_jieli_cdb(SC_RESET, 0, 0, reboot_delay_ms);
    let _ = transport.send_cmd(&cdb, None, None, Duration::from_millis(1000));
    progress(FlashEvent::Milestone {
        milestone: FlashMilestone::WriteComplete,
    });

    Ok(())
}

fn write_image_data(
    transport: &mut JlSerialTransport,
    start_addr: u32,
    data: &[u8],
    seg_idx: usize,
    total_segs: usize,
    cancel: &AtomicBool,
    progress: &dyn Fn(FlashEvent),
) -> Result<(), FlashError> {
    let total_len = data.len();
    if total_len == 0 {
        return Ok(());
    }

    // 1. Erase flash area
    progress(FlashEvent::Phase {
        phase: FlashPhase::Erase,
    });
    erase_flash_range(transport, start_addr, total_len as u32, cancel)?;

    // 2. Write flash chunks
    progress(FlashEvent::Phase {
        phase: FlashPhase::Write,
    });

    let mut written = 0;
    while written < total_len {
        if cancel.load(Ordering::Relaxed) {
            return Err(FlashError::Cancelled);
        }

        let chunk_len = (total_len - written).min(CHUNK_SIZE);
        let chunk = &data[written..written + chunk_len];
        let chunk_addr = start_addr + written as u32;
        let chunk_crc = crc16_ccitt(chunk);

        let cdb = build_jieli_cdb(SC_WRITE_FLASH, chunk_addr, chunk_len as u32, chunk_crc);
        transport.send_cmd(&cdb, Some(chunk), None, Duration::from_millis(5000))?;

        written += chunk_len;
        let seg_pct = ((written as f64 / total_len as f64) * 100.0) as u32;
        let total_pct = ((seg_idx as u32 * 100 + seg_pct) / total_segs as u32) as u8;
        progress(FlashEvent::Percent {
            value: total_pct.min(100),
        });
    }

    Ok(())
}

fn erase_flash_range(
    transport: &mut JlSerialTransport,
    start_addr: u32,
    length: u32,
    cancel: &AtomicBool,
) -> Result<(), FlashError> {
    let mut current = start_addr;
    let end = start_addr.saturating_add(length);

    while current < end {
        if cancel.load(Ordering::Relaxed) {
            return Err(FlashError::Cancelled);
        }

        let remaining = end - current;
        if current.is_multiple_of(BLOCK_SIZE) && remaining >= BLOCK_SIZE {
            let cdb = build_jieli_cdb(SC_BLOCK_ERASE_64K, current, 0, 0);
            transport.send_cmd(&cdb, None, None, Duration::from_millis(5000))?;
            current += BLOCK_SIZE;
        } else {
            let cdb = build_jieli_cdb(SC_SECTOR_ERASE_4K, current, 0, 0);
            transport.send_cmd(&cdb, None, None, Duration::from_millis(5000))?;
            current += SECTOR_SIZE;
        }
    }

    Ok(())
}

/// Execute native serial flash erase for JieLi devices.
pub fn run_serial_erase(
    job: &FlashJob,
    cancel: &AtomicBool,
    progress: &dyn Fn(FlashEvent),
) -> Result<(), FlashError> {
    let baud = if job.baud_rate > 0 {
        job.baud_rate
    } else {
        921600
    };
    progress(FlashEvent::Phase {
        phase: FlashPhase::Connect,
    });
    let mut transport = JlSerialTransport::open(&job.port, baud)?;

    let start_addr = parse_hex_u32(job.erase_start_hex.as_deref(), 0x00000000)?;
    let end_addr = parse_hex_u32(job.erase_end_hex.as_deref(), start_addr + 0x00200000)?;
    let len = end_addr.saturating_sub(start_addr);

    progress(FlashEvent::Phase {
        phase: FlashPhase::Erase,
    });
    erase_flash_range(&mut transport, start_addr, len, cancel)?;

    progress(FlashEvent::Percent { value: 100 });
    Ok(())
}

/// Execute native serial flash reading for JieLi devices.
pub fn run_serial_read(
    job: &FlashJob,
    cancel: &AtomicBool,
    progress: &dyn Fn(FlashEvent),
) -> Result<(), FlashError> {
    let read_path = job
        .read_file_path
        .as_deref()
        .ok_or_else(|| FlashError::Plugin("JieLi read requires output file (-f)".into()))?;

    let baud = if job.baud_rate > 0 {
        job.baud_rate
    } else {
        921600
    };
    progress(FlashEvent::Phase {
        phase: FlashPhase::Connect,
    });
    let mut transport = JlSerialTransport::open(&job.port, baud)?;

    let start_addr = parse_hex_u32(job.read_start_hex.as_deref(), 0x00000000)?;
    let end_addr = parse_hex_u32(job.read_end_hex.as_deref(), start_addr + 0x00200000)?;
    let total_len = end_addr.saturating_sub(start_addr);

    progress(FlashEvent::Phase {
        phase: FlashPhase::ReadFlashId,
    });

    let mut out_data = Vec::with_capacity(total_len as usize);
    let mut read_bytes = 0;

    while read_bytes < total_len {
        if cancel.load(Ordering::Relaxed) {
            return Err(FlashError::Cancelled);
        }

        let chunk_len = (total_len - read_bytes).min(CHUNK_SIZE as u32);
        let chunk_addr = start_addr + read_bytes;

        let cdb = build_jieli_cdb(SC_READ_FLASH, chunk_addr, chunk_len, 0);
        let mut chunk_buf = vec![0u8; chunk_len as usize];
        transport.send_cmd(
            &cdb,
            None,
            Some(&mut chunk_buf),
            Duration::from_millis(5000),
        )?;

        out_data.extend_from_slice(&chunk_buf);
        read_bytes += chunk_len;

        let pct = ((read_bytes as f64 / total_len as f64) * 100.0) as u8;
        progress(FlashEvent::Percent {
            value: pct.min(100),
        });
    }

    let mut file = File::create(read_path).map_err(|e| {
        FlashError::Plugin(format!(
            "failed to create output read file {}: {}",
            read_path, e
        ))
    })?;
    file.write_all(&out_data).map_err(|e| {
        FlashError::Plugin(format!(
            "failed to write output read file {}: {}",
            read_path, e
        ))
    })?;

    Ok(())
}

/// Reset JieLi device over serial port.
pub fn run_serial_reset(port_name: &str, baud: u32, delay_ms: u32) -> Result<(), FlashError> {
    let mut transport = JlSerialTransport::open(port_name, baud)?;
    let cdb = build_jieli_cdb(SC_RESET, 0, 0, delay_ms as u16);
    let _ = transport.send_cmd(&cdb, None, None, Duration::from_millis(1000));
    Ok(())
}

/// Probe JieLi device over serial port.
pub fn run_serial_probe(port_name: &str, baud: u32) -> Result<String, FlashError> {
    let mut transport = JlSerialTransport::open(port_name, baud)?;

    // Query Chip ID (0x83)
    let cdb_chip = build_jieli_cdb(SC_PROBE_CHIP, 0, 16, 0);
    let mut chip_buf = [0u8; 16];
    let chip_name = if transport
        .send_cmd(
            &cdb_chip,
            None,
            Some(&mut chip_buf),
            Duration::from_millis(1000),
        )
        .is_ok()
    {
        String::from_utf8_lossy(&chip_buf)
            .trim_matches('\0')
            .trim()
            .to_string()
    } else {
        "JieLi Device".to_string()
    };

    // Query Flash UUID (0x84)
    let cdb_uuid = build_jieli_cdb(SC_PROBE_FLASH_ID, 0, 16, 0);
    let mut uuid_buf = [0u8; 16];
    let flash_uuid = if transport
        .send_cmd(
            &cdb_uuid,
            None,
            Some(&mut uuid_buf),
            Duration::from_millis(1000),
        )
        .is_ok()
    {
        uuid_buf
            .iter()
            .map(|b| format!("{:02X}", b))
            .collect::<String>()
    } else {
        "".to_string()
    };

    let mut out = format!("[*] Chip Model: {}\n", chip_name);
    if !flash_uuid.is_empty() {
        out.push_str(&format!("[*] Flash UUID: {}\n", flash_uuid));
    }

    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constants_and_sync_cmd() {
        assert_eq!(UART_SYNC_CMD, b"TX21");
        assert_eq!(UART_ACK_BYTE, 0x79);
        assert_eq!(UART_NACK_BYTE, 0x1F);
        assert_eq!(CHUNK_SIZE, 512);
        assert_eq!(SECTOR_SIZE, 4096);
        assert_eq!(BLOCK_SIZE, 65536);
    }

    #[test]
    fn frame_checksum_calculation() {
        let cdb = build_jieli_cdb(SC_BLOCK_ERASE_64K, 0x1000, 0, 0);
        let mut frame = Vec::with_capacity(18);
        frame.extend_from_slice(&cdb);
        let crc = crc16_ccitt(&frame);
        frame.extend_from_slice(&crc.to_be_bytes());
        assert_eq!(frame.len(), 18);
        assert_eq!(crc16_ccitt(&frame[..16]), crc);
    }
}
