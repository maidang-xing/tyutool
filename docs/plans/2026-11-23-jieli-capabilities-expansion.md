# 杰理平台 (WL82 / WL83) tyutool 能力补全计划

**Goal:** 补齐杰理芯片在 tyutool 中的关键通用能力，包括独立免串口 USB 软复位、芯片与 Flash 探测识别 (`tyutool probe`)、串口模式烧录与芯片参数自适应、GUI/Core 多段固件 (Multi-segment) 写入以及 TuyaOpen 产测授权时序支持。

**Architecture:**
1. 底层 flasher (`TuyaOpen-JieLi/tools/jieli_flasher`): 增加独立 `--reset` 命令与参数透传。
2. 核心插件 (`tyutool-core::plugins::jieli`):
   - 支持根据端口自动在 USB 与 Serial UART 模式间切换；
   - 根据芯片别名 (`wl82` vs `wl83`) 自动下发正确的 boot 地址与 reboot 延时；
   - 增加 `run_reset` 与 `run_probe` 导出函数；
   - 支持 `job.segments` 多段固件连续烧录与进度映射；
   - 解除 `FlashMode::Authorize` 限制并在 `authorize.rs` 补充 JieLi 时序。
3. 命令行 (`tyutool-cli`):
   - `tyutool reset`: 对 JieLi 免除 `--port` 强制校验，默认执行 USB 软复位；
   - `tyutool probe` / `tyutool info`: 增加设备探测命令，支持免串口探测 JieLi 芯片与 Flash 信息。

**Tech Stack:** Rust 2021 (`tyutool-core`, `tyutool-cli`), Python 3 (`jieli_flasher`).

---

## 任务拆分与执行步骤

### Task 1: 独立 USB 软复位命令 (`tyutool reset -d jieli`)

**Files:**
- Modify: `/home/share/samba/ePanda/TuyaOpen-JieLi/tools/jieli_flasher/cli.py`
- Modify: `/home/share/samba/tyopen/tyutool/crates/tyutool-core/src/plugins/jieli/mod.rs`
- Modify: `/home/share/samba/tyopen/tyutool/crates/tyutool-cli/src/main.rs`
- Test: `/home/share/samba/tyopen/tyutool/crates/tyutool-cli/src/main.rs`

- [x] **Step 1.1**: 在 `jieli_flasher/cli.py` 增加 `--reset` 命令行参数支持并执行 `flasher.reset()`.
- [x] **Step 1.2**: 在 `tyutool-core` 的 `jieli` 插件中实现 `pub fn run_reset(port: Option<&str>, delay_ms: Option<u32>) -> Result<(), FlashError>`.
- [x] **Step 1.3**: 在 `tyutool-cli` 的 `Commands::Reset` 中为 `JIELI` 芯片添加免串口支持（未传 port 或 port 为 auto 时直接调用 `run_reset`）.
- [x] **Step 1.4**: 在 `tyutool-cli` 补充单元测试 `cli_jieli_reset_does_not_require_port_flag`.

---

### Task 2: 芯片与 Flash 探测识别 (`tyutool probe` / `tyutool info`)

**Files:**
- Modify: `/home/share/samba/tyopen/tyutool/crates/tyutool-core/src/plugins/jieli/mod.rs`
- Modify: `/home/share/samba/tyopen/tyutool/crates/tyutool-cli/src/main.rs`
- Test: `/home/share/samba/tyopen/tyutool/crates/tyutool-cli/src/main.rs`

- [x] **Step 2.1**: 在 `tyutool-core` 的 `jieli` 插件实现 `pub fn run_probe(port: Option<&str>, baud: Option<u32>) -> Result<String, FlashError>`.
- [x] **Step 2.2**: 在 `tyutool-cli` 的 `Commands` 枚举中增加 `Probe`（别名 `info`）命令.
- [x] **Step 2.3**: 在 `tyutool-cli` 的 `main` 中处理 `Commands::Probe`，对 `JIELI` 设备执行 `run_probe` 并输出信息.
- [x] **Step 2.4**: 补充 `tyutool probe` 的 CLI 参数解析测试.

---

### Task 3: 开放串口模式烧录与芯片参数自适应

**Files:**
- Modify: `/home/share/samba/tyopen/tyutool/crates/tyutool-core/src/plugins/jieli/mod.rs`
- Test: `/home/share/samba/tyopen/tyutool/crates/tyutool-core/src/plugins/jieli/mod.rs`

- [x] **Step 3.1**: 在 `plugins/jieli/mod.rs` 实现通道自适应逻辑：若 `port` 为有效串口，透传 `-m serial -p <port> -b <baud>`；若为空或 `"auto"`，透传 `-m usb`.
- [x] **Step 3.2**: 识别 `job.chip_id`，对 WL83 自动传递 `-dev wl83 -boot 0x103000 -reboot 50`，对 WL82 传递 `-dev wl82 -boot 0x1c02000 -reboot 500`.
- [x] **Step 3.3**: 补充参数构建单元测试，验证串口模式与芯片参数生成正确.

---

### Task 4: 多段固件（Multi-segment）在 core 的支持

**Files:**
- Modify: `/home/share/samba/tyopen/tyutool/crates/tyutool-core/src/plugins/jieli/mod.rs`
- Test: `/home/share/samba/tyopen/tyutool/crates/tyutool-core/src/plugins/jieli/mod.rs`

- [x] **Step 4.1**: 在 `run_flash` 中解析 `job.segments`，当存在多段固件时循环分段写入，前 N-1 段携带 `--no-reboot`，最后一段执行复位.
- [x] **Step 4.2**: 对多段进度百分比进行加权合成 `(seg_idx * 100 + pct) / total_segs`.
- [x] **Step 4.3**: 补充多段固件调用的单元测试.

---

### Task 5: Tuya 产测授权（`FlashMode::Authorize`）支持与时序配置

**Files:**
- Modify: `/home/share/samba/tyopen/tyutool/crates/tyutool-core/src/plugins/jieli/mod.rs`
- Modify: `/home/share/samba/tyopen/tyutool/crates/tyutool-core/src/authorize.rs`
- Test: `/home/share/samba/tyopen/tyutool/crates/tyutool-core/src/authorize.rs`

- [x] **Step 5.1**: 在 `authorize.rs` 的 `CHIP_TIMING` 中增加 `JIELI`, `WL82`, `WL83`, `AC791N`, `AC792N` 的授权握手时序配置.
- [x] **Step 5.2**: 在 `plugins/jieli/mod.rs` 中支持 `FlashMode::Authorize`（委托给通用 `run_authorize` 统一处理）.
- [x] **Step 5.3**: 补充 `AuthTiming::for_chip` 对杰理芯片的测试用例.

---

### Task 6: 纯 Rust 原生串口底层驱动与算法内置（零 Python 依赖）

**Files:**
- Create: `/home/share/samba/tyopen/tyutool/crates/tyutool-core/src/plugins/jieli/crypto.rs`
- Create: `/home/share/samba/tyopen/tyutool/crates/tyutool-core/src/plugins/jieli/serial.rs`
- Modify: `/home/share/samba/tyopen/tyutool/crates/tyutool-core/src/plugins/jieli/mod.rs`

- [x] **Step 6.1**: 实现杰理专用加解密与校验算法（`CRC8-Maxim`、`JL_ENC` 流加密、`JL_SFC` 块加密算法），并通过双向回环单测。
- [x] **Step 6.2**: 实现原生串口传输驱动 `JlSerialTransport`（`TX21` 同步握手、自动波特率协商、SCSI CDB 16 字节帧封装与 CRC16-CCITT 校验）。
- [x] **Step 6.3**: 实现纯 Rust 原生串口固件写入（512 字节分片与自动扇区擦除）、擦除（64KB/4KB 自适应）、读取、复位和芯片探测。
- [x] **Step 6.4**: 在 `plugins/jieli/mod.rs` 中，当指定具体串口时完全转入纯 Rust 原生执行，解除对 Python 环境的依赖，为多路治具产测批量授权提供强保证。

---

### Task 7: USB SCSI BOT 原生底层驱动移植（USB 免串口模式脱离 Python）

**Files:**
- Create: `/home/share/samba/tyopen/tyutool/crates/tyutool-core/src/plugins/jieli/scsi.rs`
- Modify: `/home/share/samba/tyopen/tyutool/crates/tyutool-core/src/plugins/jieli/crypto.rs`
- Modify: `/home/share/samba/tyopen/tyutool/crates/tyutool-core/src/plugins/jieli/mod.rs`

- [x] **Step 7.1**: 在 `crypto.rs` 实现 `jl_crc_cipher` (MengLi CrcDecode 算法，用于 RAM Loader 上传)。
- [x] **Step 7.2**: 在 `scsi.rs` 实现纯 Rust `JlScsiTransport`（通过 Linux `SG_IO` ioctl 驱动 `/dev/sg*` 设备节点，支持免 root 读写与自动 SCSI soft-jump 切入 UBOOT 模式）。
- [x] **Step 7.3**: 实现原生 USB SCSI 固件烧录（4KB 分片传输、64KB/4KB 自适应擦除）、读取、硬件 CRC16 校验、设备软复位与芯片探测。
- [x] **Step 7.4**: 在 `plugins/jieli/mod.rs` 中将 USB 免串口直连模式接入原生 `scsi::*` 执行流，使得杰理芯片在 USB 模式与串口模式下**均全面脱离 Python 依赖**。

---

## 验证与验收

运行官方 CI 门禁检查：
1. `cargo fmt --all --check`
2. `cargo clippy -p tyutool-core -p tyutool-cli -p tyutool-serve --all-targets -- -D warnings`
3. `cargo test -p tyutool-core -p tyutool-cli -p tyutool-serve`
4. `cargo test -p tyutool-core --features ts-rs && git diff --exit-code -- src/bindings/`
5. `pnpm run lint && pnpm run typecheck:scripts && pnpm run test:coverage && pnpm run build`
