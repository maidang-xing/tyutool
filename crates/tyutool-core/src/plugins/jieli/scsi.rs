//! Pure Rust USB Mass Storage (SCSI Generic BOT) transport & flasher for JieLi chips.
//!
//! Directly communicates with JieLi devices via SCSI Command Descriptor Blocks (CDBs)
//! over OS SCSI interfaces (such as Linux `/dev/sg*` via `SG_IO` ioctl), providing
//! native flashing, erasing, reading, resetting, and probing without Python dependencies.

use std::fs::File;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::sleep;
use std::time::{Duration, Instant};

use super::{
    build_jieli_cdb, crc16_ccitt, parse_hex_u32, SC_BLOCK_ERASE_64K, SC_PROBE_CHIP,
    SC_PROBE_FLASH_ID, SC_READ_FLASH, SC_RESET, SC_SECTOR_ERASE_4K, SC_WRITE_FLASH,
};
use crate::error::FlashError;
use crate::flash_event::{FlashEvent, FlashMilestone, FlashPhase};
use crate::job::FlashJob;

const SCSI_CHUNK_SIZE: usize = 4096;
const SECTOR_SIZE: u32 = 4096;
const BLOCK_SIZE: u32 = 65536;

pub const SUBCMD_HARDWARE_CRC: u8 = 0x13;

#[cfg(target_os = "linux")]
const SG_IO: std::os::raw::c_ulong = 0x2285;
#[cfg(target_os = "linux")]
const SG_DXFER_NONE: std::os::raw::c_int = -1;
#[cfg(target_os = "linux")]
const SG_DXFER_TO_DEV: std::os::raw::c_int = -2;
#[cfg(target_os = "linux")]
const SG_DXFER_FROM_DEV: std::os::raw::c_int = -3;

#[cfg(target_os = "linux")]
#[repr(C)]
#[derive(Debug)]
pub struct SgIoHdr {
    pub interface_id: std::os::raw::c_int,
    pub dxfer_direction: std::os::raw::c_int,
    pub cmd_len: u8,
    pub mx_sb_len: u8,
    pub iovec_count: u16,
    pub dxfer_len: u32,
    pub dxferp: *mut std::ffi::c_void,
    pub cmdp: *mut u8,
    pub sbp: *mut u8,
    pub timeout: u32,
    pub flags: u32,
    pub pack_id: std::os::raw::c_int,
    pub usr_ptr: *mut std::ffi::c_void,
    pub status: u8,
    pub masked_status: u8,
    pub msg_status: u8,
    pub sb_len_wr: u8,
    pub host_status: u16,
    pub driver_status: u16,
    pub resid: std::os::raw::c_int,
    pub duration: u32,
    pub info: u32,
}

#[cfg(target_os = "linux")]
extern "C" {
    fn ioctl(fd: std::os::raw::c_int, request: std::os::raw::c_ulong, ...) -> std::os::raw::c_int;
}

/// USB Mass Storage SCSI transport.
pub struct JlScsiTransport {
    #[cfg(target_os = "linux")]
    file: File,
    #[allow(dead_code)]
    pub node_path: String,
    pub is_uboot: bool,
    pub product: String,
}

impl JlScsiTransport {
    /// Discover connected JieLi USB SCSI device and establish communication.
    pub fn open(wait_duration: Duration, cancel: &AtomicBool) -> Result<Self, FlashError> {
        #[cfg(target_os = "linux")]
        {
            use std::fs::OpenOptions;

            let mut start = Instant::now();
            let mut soft_jump_triggered = false;

            while start.elapsed() < wait_duration {
                if cancel.load(Ordering::Relaxed) {
                    return Err(FlashError::Cancelled);
                }

                // Scan /dev/sg0 .. /dev/sg31
                for i in 0..32 {
                    let sg_name = format!("sg{}", i);
                    let sysfs_path = format!("/sys/class/scsi_generic/{}/device", sg_name);

                    // Skip non-disk devices (e.g. CDROM type 5) to prevent hanging on empty optical drives
                    if let Ok(dev_type) = std::fs::read_to_string(format!("{}/type", sysfs_path)) {
                        if dev_type.trim() != "0" {
                            continue;
                        }
                    }

                    // Pre-check vendor/model from sysfs when available
                    let sys_vendor = std::fs::read_to_string(format!("{}/vendor", sysfs_path))
                        .unwrap_or_default();
                    let sys_model = std::fs::read_to_string(format!("{}/model", sysfs_path))
                        .unwrap_or_default();
                    if !sys_vendor.is_empty() || !sys_model.is_empty() {
                        let sys_ident = format!("{} {}", sys_vendor, sys_model).to_uppercase();
                        let is_jieli_sys = [
                            "JIELI", "UBOOT", "WL80", "WL82", "WL83", "AC79", "LOADER", "BR21",
                            "BR23",
                        ]
                        .iter()
                        .any(|k| sys_ident.contains(k));
                        if !is_jieli_sys {
                            continue;
                        }
                    }

                    let path = format!("/dev/{}", sg_name);
                    let mut file = match OpenOptions::new().read(true).write(true).open(&path) {
                        Ok(f) => f,
                        Err(e) => {
                            if !sys_vendor.is_empty() || !sys_model.is_empty() {
                                log::warn!(
                                    "Found JieLi device at {} ({}/{}) but cannot open: {}",
                                    path,
                                    sys_vendor.trim(),
                                    sys_model.trim(),
                                    e
                                );
                            }
                            continue;
                        }
                    };

                    // Send SCSI INQUIRY (CDB 0x12)
                    let mut inquiry_cdb = [0u8; 16];
                    inquiry_cdb[0] = 0x12;
                    inquiry_cdb[4] = 36;

                    let mut resp = [0u8; 36];
                    if let Ok(()) = Self::sg_io_transfer_raw(
                        &mut file,
                        &inquiry_cdb,
                        None,
                        Some(&mut resp),
                        Duration::from_millis(500),
                    ) {
                        let vendor = String::from_utf8_lossy(&resp[8..16]).trim().to_string();
                        let product = String::from_utf8_lossy(&resp[16..32]).trim().to_string();
                        let revision = String::from_utf8_lossy(&resp[32..36]).trim().to_string();
                        let ident = format!("{} {}", vendor, product).to_uppercase();

                        let is_jieli = [
                            "JIELI", "UBOOT", "WL80", "WL82", "WL83", "AC79", "LOADER", "BR21",
                            "BR23",
                        ]
                        .iter()
                        .any(|k| ident.contains(k));

                        if is_jieli {
                            let is_ram_loader = product.to_uppercase().contains("WL80UBOOT")
                                || product.to_uppercase().contains("LOADER")
                                || revision == "3.00";
                            let is_uboot = (product.to_uppercase().contains("UBOOT")
                                || is_ram_loader)
                                && !is_ram_loader;

                            if !is_uboot && !soft_jump_triggered {
                                log::info!(
                                    "Target {} is in application mode ({} {}). Sending SCSI soft-jump (0xFD)...",
                                    path, vendor, product
                                );
                                let mut jump_cdb = [0u8; 16];
                                jump_cdb[0] = 0xFD;
                                let _ = Self::sg_io_transfer_raw(
                                    &mut file,
                                    &jump_cdb,
                                    None,
                                    None,
                                    Duration::from_millis(500),
                                );
                                soft_jump_triggered = true;
                                sleep(Duration::from_millis(1500));
                                start = Instant::now();
                                break;
                            }

                            log::info!(
                                "Connected to JieLi USB SCSI device at {} ({} {})",
                                path,
                                vendor,
                                product
                            );
                            return Ok(Self {
                                file,
                                node_path: path,
                                is_uboot,
                                product,
                            });
                        }
                    }
                }

                sleep(Duration::from_millis(200));
            }

            Err(FlashError::Plugin(
                "No JieLi USB SCSI device found within timeout. If inside a virtual machine (like VMware), please ensure the USB device (VID 4C4A / 3654) is connected to this VM; or hold UPDATE and press RST to enter download mode."
                    .into(),
            ))
        }

        #[cfg(not(target_os = "linux"))]
        {
            let _ = (wait_duration, cancel);
            Err(FlashError::Plugin(
                "Native USB SCSI transport is currently supported on Linux. On other platforms please specify a serial COM port or use the Python flasher fallback."
                    .into(),
            ))
        }
    }

    /// Reconnect to JieLi device after USB bus reset triggered by RAM loader execution.
    pub fn reopen(
        &mut self,
        wait_duration: Duration,
        cancel: &AtomicBool,
    ) -> Result<(), FlashError> {
        #[cfg(target_os = "linux")]
        {
            let start = Instant::now();
            sleep(Duration::from_millis(3000));

            while start.elapsed() < wait_duration {
                if cancel.load(Ordering::Relaxed) {
                    return Err(FlashError::Cancelled);
                }

                for i in 0..32 {
                    let sg_name = format!("sg{}", i);
                    let sysfs_path = format!("/sys/class/scsi_generic/{}/device", sg_name);

                    if let Ok(dev_type) = std::fs::read_to_string(format!("{}/type", sysfs_path)) {
                        if dev_type.trim() != "0" {
                            continue;
                        }
                    }

                    let sys_vendor = std::fs::read_to_string(format!("{}/vendor", sysfs_path))
                        .unwrap_or_default();
                    let sys_model = std::fs::read_to_string(format!("{}/model", sysfs_path))
                        .unwrap_or_default();
                    if sys_model.to_uppercase().contains("UDISK") {
                        continue;
                    }
                    if !sys_vendor.is_empty() || !sys_model.is_empty() {
                        let sys_ident = format!("{} {}", sys_vendor, sys_model).to_uppercase();
                        let is_jieli_sys = [
                            "JIELI", "UBOOT", "WL80", "WL82", "WL83", "AC79", "LOADER", "BR21",
                            "BR23",
                        ]
                        .iter()
                        .any(|k| sys_ident.contains(k));
                        if !is_jieli_sys {
                            continue;
                        }
                    }

                    let path = format!("/dev/{}", sg_name);
                    let mut file = match std::fs::OpenOptions::new()
                        .read(true)
                        .write(true)
                        .open(&path)
                    {
                        Ok(f) => f,
                        Err(e) => {
                            if !sys_vendor.is_empty() || !sys_model.is_empty() {
                                log::warn!(
                                    "Found JieLi device at {} ({}/{}) but cannot open: {}",
                                    path,
                                    sys_vendor.trim(),
                                    sys_model.trim(),
                                    e
                                );
                            }
                            continue;
                        }
                    };

                    let mut inquiry_cdb = [0u8; 16];
                    inquiry_cdb[0] = 0x12;
                    inquiry_cdb[4] = 36;

                    let mut resp = [0u8; 36];
                    if let Ok(()) = Self::sg_io_transfer_raw(
                        &mut file,
                        &inquiry_cdb,
                        None,
                        Some(&mut resp),
                        Duration::from_millis(500),
                    ) {
                        let vendor = String::from_utf8_lossy(&resp[8..16]).trim().to_string();
                        let product = String::from_utf8_lossy(&resp[16..32]).trim().to_string();
                        let revision = String::from_utf8_lossy(&resp[32..36]).trim().to_string();
                        let ident = format!("{} {}", vendor, product).to_uppercase();

                        let is_jieli = [
                            "JIELI", "UBOOT", "WL80", "WL82", "WL83", "AC79", "LOADER", "BR21",
                            "BR23",
                        ]
                        .iter()
                        .any(|k| ident.contains(k));

                        if is_jieli {
                            if product.to_uppercase().contains("UDISK") {
                                continue;
                            }
                            // Mask ROM is revision 1.00; RAM Loader is revision 3.00 (WL80UBOOT).
                            // Wait for the RAM Loader to re-enumerate; ignore the old Mask ROM device before it detaches.
                            let is_ram_loader = revision == "3.00"
                                || product.to_uppercase().contains("WL80")
                                || product.to_uppercase().contains("LOADER");
                            if !is_ram_loader {
                                continue;
                            }

                            log::info!(
                                "Reconnected to JieLi USB SCSI device at {} ({} {}, rev {})",
                                path,
                                vendor,
                                product,
                                revision
                            );
                            self.file = file;
                            self.node_path = path;
                            self.product = product;
                            self.is_uboot = false;
                            return Ok(());
                        }
                    }
                }

                sleep(Duration::from_millis(200));
            }

            Err(FlashError::Plugin(
                "Timeout waiting for JieLi device to re-enumerate after RAM loader execution"
                    .into(),
            ))
        }

        #[cfg(not(target_os = "linux"))]
        {
            let _ = (wait_duration, cancel);
            Ok(())
        }
    }

    #[cfg(target_os = "linux")]
    fn sg_io_transfer_raw(
        file: &mut File,
        cdb: &[u8; 16],
        data_out: Option<&[u8]>,
        mut data_in: Option<&mut [u8]>,
        timeout: Duration,
    ) -> Result<(), FlashError> {
        use std::os::unix::io::AsRawFd;

        let mut sense_buffer = [0u8; 32];
        let mut hdr: SgIoHdr = unsafe { std::mem::zeroed() };
        hdr.interface_id = b'S' as std::os::raw::c_int;
        hdr.cmd_len = 16;
        hdr.cmdp = cdb.as_ptr() as *mut u8;
        hdr.mx_sb_len = sense_buffer.len() as u8;
        hdr.sbp = sense_buffer.as_mut_ptr();
        hdr.timeout = timeout.as_millis().min(u32::MAX as u128) as u32;

        if let Some(out) = data_out {
            hdr.dxfer_direction = SG_DXFER_TO_DEV;
            hdr.dxfer_len = out.len() as u32;
            hdr.dxferp = out.as_ptr() as *mut std::ffi::c_void;
        } else if let Some(ref mut inp) = data_in {
            hdr.dxfer_direction = SG_DXFER_FROM_DEV;
            hdr.dxfer_len = inp.len() as u32;
            hdr.dxferp = inp.as_mut_ptr() as *mut std::ffi::c_void;
        } else {
            hdr.dxfer_direction = SG_DXFER_NONE;
            hdr.dxfer_len = 0;
            hdr.dxferp = std::ptr::null_mut();
        }

        let ret = unsafe { ioctl(file.as_raw_fd(), SG_IO, &mut hdr) };
        if ret < 0 {
            return Err(FlashError::Plugin(format!(
                "SG_IO ioctl failed: {}",
                std::io::Error::last_os_error()
            )));
        }

        if hdr.status != 0 || hdr.host_status != 0 {
            return Err(FlashError::Plugin(format!(
                "JieLi SCSI command 0x{:02X} failed: status=0x{:02X}, host_status=0x{:04X}",
                cdb[1], hdr.status, hdr.host_status
            )));
        }

        Ok(())
    }

    /// Send a 16-byte SCSI CDB frame with optional payload and receive response.
    pub fn send_cmd(
        &mut self,
        cdb: &[u8; 16],
        data_out: Option<&[u8]>,
        data_in: Option<&mut [u8]>,
        timeout: Duration,
    ) -> Result<(), FlashError> {
        #[cfg(target_os = "linux")]
        {
            Self::sg_io_transfer_raw(&mut self.file, cdb, data_out, data_in, timeout)
        }

        #[cfg(not(target_os = "linux"))]
        {
            let _ = (cdb, data_out, data_in, timeout);
            Err(FlashError::Plugin(
                "Native USB SCSI transport is only supported on Linux.".into(),
            ))
        }
    }
}

fn locate_ram_loader(chip_name: &str) -> Option<Vec<u8>> {
    let loader_name = if chip_name.to_ascii_lowercase().contains("83")
        || chip_name.to_ascii_lowercase().contains("792")
    {
        "wl83loader.bin"
    } else {
        "wl82loader.bin"
    };

    // 1. Check TYUTOOL_RAM_LOADER_DIR
    if let Ok(dir) = std::env::var("TYUTOOL_RAM_LOADER_DIR") {
        let p = std::path::Path::new(&dir).join(loader_name);
        if let Ok(bytes) = std::fs::read(&p) {
            return Some(bytes);
        }
    }

    // 2. Check TuyaOpen-JieLi tools path
    let search_roots = [
        ".",
        "platform/JIELI",
        "platform/jieli",
        "../platform/JIELI",
        "../platform/jieli",
        "../../platform/JIELI",
        "../../platform/jieli",
        "platforms/JIELI",
        "platforms/jieli",
        "../platforms/JIELI",
        "../platforms/jieli",
        "TuyaOpen-JieLi",
        "../TuyaOpen-JieLi",
        "../../TuyaOpen-JieLi",
        "/home/share/samba/ePanda/TuyaOpen-JieLi",
    ];
    for root in &search_roots {
        let p = std::path::Path::new(root)
            .join("tools")
            .join("jieli_flasher")
            .join("loaders")
            .join(loader_name);
        if let Ok(bytes) = std::fs::read(&p) {
            return Some(bytes);
        }
    }

    None
}

fn build_16bit_cdb(subcmd: u16, addr: u32, length: u16, crc: u16) -> [u8; 16] {
    let mut cdb = [0xFFu8; 16];
    cdb[0] = ((subcmd >> 8) & 0xFF) as u8;
    cdb[1] = (subcmd & 0xFF) as u8;
    cdb[2] = ((addr >> 24) & 0xFF) as u8;
    cdb[3] = ((addr >> 16) & 0xFF) as u8;
    cdb[4] = ((addr >> 8) & 0xFF) as u8;
    cdb[5] = (addr & 0xFF) as u8;
    cdb[6] = ((length >> 8) & 0xFF) as u8;
    cdb[7] = (length & 0xFF) as u8;
    if subcmd == 0xFB06 || subcmd == 0xFB04 {
        cdb[8] = 0x00;
        cdb[9] = (crc & 0xFF) as u8;
        cdb[10] = ((crc >> 8) & 0xFF) as u8;
    }
    cdb
}

fn stage_ram_loader_if_needed(
    transport: &mut JlScsiTransport,
    chip_id: &str,
) -> Result<(), FlashError> {
    if !transport.is_uboot {
        return Ok(());
    }

    let chip_lower = chip_id.to_ascii_lowercase();
    let is_wl83 = chip_lower.contains("83")
        || chip_lower.contains("792")
        || transport.product.contains("83")
        || transport.product.contains("792");

    let (target_addr, mengli) = if is_wl83 {
        (0x103000u32, false)
    } else {
        // wl82loader.bin is already MengLi-ciphered on disk; Mask ROM decrypts it in hardware.
        (0x1C02000u32, false)
    };

    let loader_bytes = match locate_ram_loader(if is_wl83 { "wl83" } else { "wl82" }) {
        Some(b) => b,
        None => {
            log::warn!("Could not locate RAM loader binary for staging");
            return Ok(());
        }
    };

    log::info!(
        "Target is in Mask ROM mode ({}). Staging RAM loader ({} bytes) at 0x{:08X}...",
        transport.product,
        loader_bytes.len(),
        target_addr
    );

    let block_size = 512usize;
    let mut offset = 0usize;

    while offset < loader_bytes.len() {
        let chunk_len = (loader_bytes.len() - offset).min(block_size);
        let mut block = loader_bytes[offset..offset + chunk_len].to_vec();

        if mengli {
            super::crypto::jl_crc_cipher(&mut block, 0xFFFFFFFF);
        }

        let addr = target_addr + offset as u32;
        let block_crc = crc16_ccitt(&block);
        let cdb = build_16bit_cdb(0xFB06, addr, chunk_len as u16, block_crc);

        transport.send_cmd(&cdb, Some(&block), None, Duration::from_millis(3000))?;
        offset += chunk_len;
    }

    log::info!("Executing RAM loader at 0x{:08X}...", target_addr);
    let cdb_jump = build_16bit_cdb(0xFB08, target_addr, 1, 0);
    let mut jump_resp = [0u8; 16];
    let _ = transport.send_cmd(
        &cdb_jump,
        None,
        Some(&mut jump_resp),
        Duration::from_millis(3000),
    );

    log::info!("Waiting for RAM loader to re-enumerate on USB...");
    let cancel_reopen = AtomicBool::new(false);
    transport.reopen(Duration::from_secs(30), &cancel_reopen)?;
    log::info!("RAM loader executed successfully and reconnected, flash operations ready.");

    Ok(())
}

/// Execute native USB SCSI firmware flashing for JieLi devices.
pub fn run_scsi_flash(
    job: &FlashJob,
    cancel: &AtomicBool,
    progress: &dyn Fn(FlashEvent),
) -> Result<(), FlashError> {
    progress(FlashEvent::Phase {
        phase: FlashPhase::Connect,
    });
    let mut transport = JlScsiTransport::open(Duration::from_secs(60), cancel)?;
    stage_ram_loader_if_needed(&mut transport, &job.chip_id)?;

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
    transport: &mut JlScsiTransport,
    start_addr: u32,
    data: &[u8],
    seg_idx: usize,
    total_segs: usize,
    cancel: &AtomicBool,
    progress: &dyn Fn(FlashEvent),
) -> Result<(), FlashError> {
    let total_len = data.len() as u32;
    if total_len == 0 {
        return Ok(());
    }

    // 1. Erase Phase
    progress(FlashEvent::Phase {
        phase: FlashPhase::Erase,
    });
    erase_range(transport, start_addr, total_len, cancel)?;

    // 2. Write Phase
    progress(FlashEvent::Phase {
        phase: FlashPhase::Write,
    });

    let mut offset = 0usize;
    let mut last_pct = 255u8;

    while offset < data.len() {
        if cancel.load(Ordering::Relaxed) {
            return Err(FlashError::Cancelled);
        }

        let chunk_len = (data.len() - offset).min(SCSI_CHUNK_SIZE);
        let chunk = &data[offset..offset + chunk_len];
        let current_addr = start_addr + offset as u32;

        let crc = crc16_ccitt(chunk);
        let cdb = build_jieli_cdb(SC_WRITE_FLASH, current_addr, chunk_len as u32, crc);

        if transport
            .send_cmd(&cdb, Some(chunk), None, Duration::from_millis(5000))
            .is_err()
        {
            let cdb_fb = build_16bit_cdb(0xFB04, current_addr, chunk_len as u16, crc);
            transport.send_cmd(&cdb_fb, Some(chunk), None, Duration::from_millis(5000))?;
        }

        offset += chunk_len;

        let seg_pct = ((offset as f32 / data.len() as f32) * 100.0) as u8;
        let overall_pct = ((seg_idx as u32 * 100 + seg_pct as u32) / total_segs as u32) as u8;

        if overall_pct != last_pct {
            last_pct = overall_pct;
            progress(FlashEvent::Percent {
                value: overall_pct.min(100),
            });
        }
    }

    // 3. Verify Phase
    progress(FlashEvent::Phase {
        phase: FlashPhase::Verify,
    });

    let expected_crc = crc16_ccitt(data);
    let cdb_crc = build_jieli_cdb(SUBCMD_HARDWARE_CRC, start_addr, total_len, 0);
    let mut crc_buf = [0u8; 16];

    let verified = if transport
        .send_cmd(
            &cdb_crc,
            None,
            Some(&mut crc_buf),
            Duration::from_millis(5000),
        )
        .is_ok()
    {
        let chip_crc = u16::from_be_bytes([crc_buf[0], crc_buf[1]]);
        chip_crc == expected_crc
    } else {
        // Fallback read verify
        let mut read_offset = 0usize;
        let mut match_ok = true;
        while read_offset < data.len() {
            if cancel.load(Ordering::Relaxed) {
                return Err(FlashError::Cancelled);
            }
            let read_len = (data.len() - read_offset).min(SCSI_CHUNK_SIZE);
            let cdb_read = build_jieli_cdb(
                SC_READ_FLASH,
                start_addr + read_offset as u32,
                read_len as u32,
                0,
            );
            let mut read_buf = vec![0u8; read_len];
            transport.send_cmd(
                &cdb_read,
                None,
                Some(&mut read_buf),
                Duration::from_millis(5000),
            )?;
            if read_buf[..read_len] != data[read_offset..read_offset + read_len] {
                match_ok = false;
                break;
            }
            read_offset += read_len;
        }
        match_ok
    };

    if !verified {
        return Err(FlashError::Plugin("JieLi flash verification failed".into()));
    }

    Ok(())
}

fn erase_range(
    transport: &mut JlScsiTransport,
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
            let mut resp = [0u8; 16];
            if transport
                .send_cmd(&cdb, None, None, Duration::from_millis(5000))
                .is_err()
                && transport
                    .send_cmd(&cdb, None, Some(&mut resp), Duration::from_millis(5000))
                    .is_err()
            {
                let cdb_fb = build_16bit_cdb(0xFB00, current, 0, 0);
                if transport
                    .send_cmd(&cdb_fb, None, Some(&mut resp), Duration::from_millis(5000))
                    .is_err()
                {
                    let _ = transport.send_cmd(&cdb_fb, None, None, Duration::from_millis(5000));
                }
            }
            current += BLOCK_SIZE;
        } else {
            let cdb = build_jieli_cdb(SC_SECTOR_ERASE_4K, current, 0, 0);
            let mut resp = [0u8; 16];
            if transport
                .send_cmd(&cdb, None, None, Duration::from_millis(5000))
                .is_err()
                && transport
                    .send_cmd(&cdb, None, Some(&mut resp), Duration::from_millis(5000))
                    .is_err()
            {
                let cdb_fb = build_16bit_cdb(0xFB01, current, 0, 0);
                if transport
                    .send_cmd(&cdb_fb, None, Some(&mut resp), Duration::from_millis(5000))
                    .is_err()
                {
                    let _ = transport.send_cmd(&cdb_fb, None, None, Duration::from_millis(5000));
                }
            }
            current += SECTOR_SIZE;
        }
    }

    Ok(())
}

/// Execute native USB SCSI flash range erase for JieLi devices.
pub fn run_scsi_erase(
    job: &FlashJob,
    cancel: &AtomicBool,
    progress: &dyn Fn(FlashEvent),
) -> Result<(), FlashError> {
    progress(FlashEvent::Phase {
        phase: FlashPhase::Connect,
    });
    let mut transport = JlScsiTransport::open(Duration::from_secs(60), cancel)?;
    stage_ram_loader_if_needed(&mut transport, &job.chip_id)?;

    let start_addr = parse_hex_u32(job.erase_start_hex.as_deref(), 0x00000000)?;
    let end_addr = parse_hex_u32(job.erase_end_hex.as_deref(), start_addr + 0x00200000)?;
    let len = end_addr.saturating_sub(start_addr);

    progress(FlashEvent::Phase {
        phase: FlashPhase::Erase,
    });
    erase_range(&mut transport, start_addr, len, cancel)?;

    progress(FlashEvent::Milestone {
        milestone: FlashMilestone::EraseComplete,
    });
    Ok(())
}

/// Execute native USB SCSI flash read for JieLi devices.
pub fn run_scsi_read(
    job: &FlashJob,
    cancel: &AtomicBool,
    progress: &dyn Fn(FlashEvent),
) -> Result<(), FlashError> {
    let read_path = job
        .read_file_path
        .as_deref()
        .ok_or_else(|| FlashError::Plugin("JieLi read requires output file (-f)".into()))?;

    progress(FlashEvent::Phase {
        phase: FlashPhase::Connect,
    });
    let mut transport = JlScsiTransport::open(Duration::from_secs(60), cancel)?;
    stage_ram_loader_if_needed(&mut transport, &job.chip_id)?;

    let start_addr = parse_hex_u32(job.read_start_hex.as_deref(), 0x00000000)?;
    let end_addr = parse_hex_u32(job.read_end_hex.as_deref(), start_addr + 0x00200000)?;
    let total_len = end_addr.saturating_sub(start_addr);

    progress(FlashEvent::Phase {
        phase: FlashPhase::ReadFlashId,
    });

    let mut out_file = File::create(read_path).map_err(|e| {
        FlashError::Plugin(format!(
            "failed to create read output file {}: {}",
            read_path, e
        ))
    })?;

    let mut offset = 0u32;
    let mut buf = vec![0u8; SCSI_CHUNK_SIZE];

    while offset < total_len {
        if cancel.load(Ordering::Relaxed) {
            return Err(FlashError::Cancelled);
        }

        let chunk_len = (total_len - offset).min(SCSI_CHUNK_SIZE as u32);
        let cdb = build_jieli_cdb(SC_READ_FLASH, start_addr + offset, chunk_len, 0);

        transport.send_cmd(
            &cdb,
            None,
            Some(&mut buf[..chunk_len as usize]),
            Duration::from_millis(5000),
        )?;

        out_file
            .write_all(&buf[..chunk_len as usize])
            .map_err(|e| FlashError::Plugin(format!("failed to write read output file: {}", e)))?;

        offset += chunk_len;
        let pct = ((offset as f32 / total_len as f32) * 100.0) as u8;
        progress(FlashEvent::Percent { value: pct });
    }

    Ok(())
}

/// Execute native USB SCSI soft reset for JieLi devices.
pub fn run_scsi_reset(delay_ms: u32) -> Result<(), FlashError> {
    let cancel = AtomicBool::new(false);
    let mut transport = JlScsiTransport::open(Duration::from_secs(60), &cancel)?;
    let cdb = build_jieli_cdb(SC_RESET, 0, 0, delay_ms as u16);
    let _ = transport.send_cmd(&cdb, None, None, Duration::from_millis(1000));
    Ok(())
}

/// Query JieLi chip model and Flash UUID over native USB SCSI.
pub fn run_scsi_probe() -> Result<String, FlashError> {
    let cancel = AtomicBool::new(false);
    let mut transport = JlScsiTransport::open(Duration::from_secs(60), &cancel)?;

    let chip_name = if transport.product.is_empty() || transport.product.contains("Device") {
        let cdb_chip = build_jieli_cdb(SC_PROBE_CHIP, 0, 16, 0);
        let mut chip_buf = [0u8; 16];
        if transport
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
            transport.product.clone()
        }
    } else {
        transport.product.clone()
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
    fn constants_and_scsi_chunks() {
        assert_eq!(SCSI_CHUNK_SIZE, 4096);
        assert_eq!(SECTOR_SIZE, 4096);
        assert_eq!(BLOCK_SIZE, 65536);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn sg_io_hdr_size_matches_c_abi() {
        assert_eq!(std::mem::size_of::<SgIoHdr>(), 88);
    }
}
