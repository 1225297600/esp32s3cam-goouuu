# Goouuu ESP32-S3-CAM Web 控制与视频流平台

基于 Rust + Embassy 异步框架开发的 Goouuu ESP32-S3-CAM 边缘视觉控制平台。通过网页界面控制硬件、采集图像、传输视频流，支持 Wi-Fi、HTTP、摄像头、SD 卡、WS2812 等外设。

> **适配说明**：本文档参照 ESP32-S3-RS485-CAN 的文档结构编写，内容已针对 **Goouuu ESP32-S3-CAM（N16R8）** 的硬件特性重新整理。Goouuu-CAM 是视觉优先的开发板，原生不包含隔离 CAN / RS485 收发器；如需总线通信，需通过扩展接口外接模块。


## 硬件平台

- **主控**：ESP32-S3-WROOM-1 N16R8（Xtensa LX7 双核 @ 240MHz）
- **存储**：16MB Flash / 8MB PSRAM（N16R8 配置，适合图像处理与视频流）
- **无线**：2.4GHz Wi-Fi 4（802.11 b/g/n）+ Bluetooth 5 (LE)
- **摄像头**：24Pin DVP 接口，支持 OV2640 / OV5640（2MP / 5MP 可选）
- **存储扩展**：MicroSD 卡槽（SD_CMD / SD_CLK / SD_DATA 单线模式）
- **板载外设**：WS2812 RGB LED（GPIO48）、用户 LED（GPIO43）、BOOT 按键（GPIO0）
- **USB**：双 Type-C 接口（原生 USB + CH340 串口），支持 USB-JTAG 烧录与调试
- **GPIO 扩展**：42Pin 排针引出，部分 GPIO 与摄像头 / SD 卡复用
- **参考项目**：
  - [zhuhai-esp/ESP32-S3-Goouuu-Cam](https://github.com/zhuhai-esp/ESP32-S3-Goouuu-Cam)（RTSP / WebServer / WS2812 示例）
  - [profharris/GOOUUU_ESP32-S3-CAM](https://github.com/profharris/GOOUUU_ESP32-S3-CAM)（引脚定义参考）
  - [果云 GOOOUU ESP32-S3-CAM 引脚图](https://www.16302.com)


## 功能列表

### 已规划功能

| 功能 | 说明 | 状态 |
|------|------|------|
| **Wi-Fi** | STA 模式连接路由器，DHCP 获取 IP | ✅ 已实现 |
| **Web 服务器** | 基于 picoserve，提供 HTTP 接口与网页控制 | ✅ 已实现 |
| **摄像头采集** | OV2640 JPEG 单帧拍摄、参数调节 | ✅ 已实现 |
| **视频流** | MJPEG 视频流推送至浏览器（multipart/x-mixed-replace） | 🚧 开发中 |
| **SD 卡存储** | MicroSD（1-bit SDMMC）挂载、照片保存、文件列表与下载 | 🚧 开发中 |
| **WS2812 RGB** | 网页控制板载 WS2812（GPIO48）颜色 | ✅ 已实现 |
| **BLE** | 蓝牙低功耗通信，可用于配网或参数同步 | 🚧 开发中 |

> 说明：本板为视觉优先开发板，**不含**板载 CAN / RS485 / RTC 收发器，也**无独立用户 LED**（IO43 为 U0TXD 串口/日志引脚，不可复用为 LED）。板载可控 LED 仅 WS2812（GPIO48）。

### 待定功能

- Modbus RTU / Modbus TCP 协议支持
- MQTT 上报图像与传感器数据
- OTA 在线升级
- 边缘 AI 推理（人员检测、目标识别）
- 多设备组网与视频汇聚


## 引脚分配与功能

以下引脚定义综合自 Goouuu-CAM N16R8 的典型配置及社区文档。

| IO | 功能 | 说明 |
|----|------|------|
| 0 | BOOT | 启动模式选择按键（低电平进入下载模式） |
| 4 | CAM_SIOD | 摄像头 I²C 数据（SDA） |
| 5 | CAM_SIOC | 摄像头 I²C 时钟（SCL） |
| 6 | CAM_VSYNC | 摄像头场同步 |
| 7 | CAM_HREF | 摄像头行同步 |
| 8 | CAM_Y4 / D2 | 摄像头数据位 |
| 9 | CAM_Y3 / D1 | 摄像头数据位 |
| 10 | CAM_Y5 / D3 | 摄像头数据位 |
| 11 | CAM_Y2 / D0 | 摄像头数据位 |
| 12 | CAM_Y6 / D4 | 摄像头数据位 |
| 13 | CAM_PCLK | 摄像头像素时钟 |
| 15 | CAM_XCLK | 摄像头主时钟（20MHz / 10MHz） |
| 16 | CAM_Y9 / D7 | 摄像头数据位 |
| 17 | CAM_Y8 / D6 | 摄像头数据位 |
| 18 | CAM_Y7 / D5 | 摄像头数据位 |
| 19 | USB D- | 原生 USB（与摄像头共用时需注意冲突） |
| 20 | USB D+ | 原生 USB（与摄像头共用时需注意冲突） |
| 21 | LCD BLK | SPI LCD 背光控制（无独立用户 LED） |
| 38 | SD_CMD | MicroSD 命令线（SDMMC CMD） |
| 39 | SD_CLK | MicroSD 时钟线（SDMMC CLK） |
| 40 | SD_DATA | MicroSD 数据线（SDMMC DATA0，1-bit 模式） |
| 43 | U0TXD | 串口调试 TX（CH343 控制台 + defmt 日志，不可复用为 LED） |
| 44 | U0RXD | 串口调试 RX |
| 48 | WS2812 | 板载 RGB LED |

> **冲突注意**：GPIO19/20 为原生 USB 引脚，当摄像头排线或扩展板占用时不可用于 USB；摄像头 DVP 接口占用 GPIO4–18 中的大部分引脚，引出至排针的“空闲 GPIO”以实际板型丝印为准。


## MicroSD 卡

板载 MicroSD 卡槽位于开发板背面，通过 **SDMMC 外设的 1-bit 模式**（非 SPI、非 4-bit）驱动，占用引脚如下：

| 信号 | GPIO | SDMMC 功能 |
|------|------|-----------|
| SD_CMD | IO38 | CMD（命令线） |
| SD_CLK | IO39 | CLK（时钟线） |
| SD_DATA | IO40 | DATA0（单数据线） |

- **总线模式**：1-bit（CMD / CLK / DATA0 各一根），初始化时必须按 1-bit 配置，否则挂载失败。
- **文件系统**：规划使用 FAT（`esp-hal::sdmmc` + FAT 文件系统组件），用于照片保存/读取。
- **供电**：卡与摄像头/Wi-Fi/PSRAM 同时工作时峰值电流高，务必 5V Type-C 供电。
- **状态查询**：`/api/status` 的 `sd` 字段返回 SD 卡检测/挂载状态；文件列表与下载见 `/api/sd/list`、`/api/sd/download`（规划中）。


## 项目结构

以下结构参照原文风格设计，适配 Goouuu-CAM 的视觉与网络功能。

```text
software/
├── Cargo.toml
├── .cargo/
│   ├── config.toml              # 编译/烧录配置（runner 指向 /dev/ttyUSB0）
│   └── esp-config.toml          # 运行时配置（DEFMT_LOG 等）
├── build.rs
└── src/
    ├── bin/
    │   └── main.rs              # 入口：外设初始化 + Embassy 任务启动
    ├── drivers/
    │   └── ov2640.rs            # OV2640 传感器驱动（SCCB 寄存器配置）
    └── app/
        ├── mod.rs
        ├── task_net.rs          # Wi-Fi STA + DHCP + embassy-net 网络栈
        ├── task_web.rs          # HTTP 服务器（picoserve）
        ├── routes.rs            # 路由：/、/api/status、/api/capture、/api/ws2812 等
        ├── task_camera.rs       # 摄像头 DVP 初始化与 JPEG 采集
        └── task_ws2812.rs       # WS2812 RGB 控制（RMT，GPIO48）
```


## 依赖

以下为参考依赖清单，实际版本请根据 esp-hal / embassy 的当前发布调整。

```toml
[dependencies]
esp-hal = { version = "~1.2.2", features = ["defmt", "esp32s3", "unstable"] }
esp-radio = { version = "1.0.0-beta.1", features = ["ble", "coex", "defmt", "esp-alloc", "esp32s3", "unstable", "wifi"] }
esp-rtos = { version = "0.4.0", features = ["defmt", "embassy", "esp-alloc", "esp-radio", "esp32s3"] }
esp-alloc = { version = "0.11.0", features = ["defmt", "esp32s3"] }
esp-println = { version = "0.18.0", features = ["defmt-espflash", "esp32s3"] }
esp-bootloader-esp-idf = { version = "0.6.0", features = ["defmt", "esp32s3"] }

embassy-executor = { version = "0.10.0", features = ["defmt"] }
embassy-time = { version = "0.5.0", features = ["defmt"] }
embassy-net = { version = "0.9.1", features = ["defmt", "dhcpv4", "medium-ethernet", "tcp", "udp"] }
embassy-sync = { version = "0.7", features = ["defmt"] }

picoserve = { version = "0.18", features = ["embassy"] }
static_cell = "2.1.1"

# 摄像头与图像（需根据 esp-hal 摄像头驱动生态选用）
# esp-camera = { git = "...", features = ["esp32s3"] }  # 示例占位

# BLE
bt-hci = "0.9.0"
trouble-host = { version = "0.7.0", features = ["gatt"] }

# 嵌入式 IO 抽象
embedded-io = { version = "0.7.1", features = ["defmt"] }
embedded-io-async = { version = "0.7.0", features = ["defmt"] }
```


## 构建与烧录

### 环境准备

```bash
cargo install espflash
rustup target add xtensa-esp32s3-none-elf
```

### 配置烧录串口

在 `.cargo/config.toml` 中指定串口（Goouuu-CAM 通常通过 CH340 串口或原生 USB 识别）：

```toml
[target.'cfg(all(target_arch = "xtensa", target_os = "none"))']
runner = "espflash flash --monitor --port /dev/ttyUSB0"
```

> 若使用原生 USB-JTAG，端口可能为 `/dev/ttyACM0`；若使用板载 CH340，通常为 `/dev/ttyUSB0`。

### 编译运行

```bash
cargo run
```

临时指定串口：

```bash
cargo run -- --port /dev/ttyACM0
```


## 使用说明

### 1. 配置 Wi-Fi

编辑 `src/app/task_net.rs`：

```rust
const WIFI_NAME: &str = "你的WiFi名称";
const WIFI_PASSWORD: &str = "你的WiFi密码";
```

### 2. 获取设备 IP

烧录后串口会打印 DHCP 分配的 IP 地址：

```text
[INFO ] [net] IPv4 address: 192.168.x.x/24
```

### 3. 访问 Web 界面

在同一局域网内浏览器访问 `http://<设备IP>/`，可看到控制面板首页。

### 4. Web API（设计参考）

| 路径 | 方法 | 说明 |
|------|------|------|
| `/` | GET | 控制面板首页 |
| `/api/status` | GET | 设备状态 JSON（IP、内存、摄像头、SD 卡） |
| `/api/capture` | GET | 获取最新一帧 JPEG 照片 |
| `/stream` | GET | MJPEG 视频流（multipart，规划中） |
| `/api/camera/params` | GET / POST | 读取 / 设置分辨率、JPEG 质量 |
| `/api/ws2812/:r/:g/:b` | POST | 设置板载 WS2812 颜色 |
| `/api/sd/list` | GET | SD 卡文件列表（规划中） |
| `/api/sd/download?file=` | GET | 下载 SD 卡文件（规划中） |
| `/api/ble/status` | GET | BLE 连接状态（待实现） |


## 硬件注意事项

1. **摄像头与 USB 冲突**：Goouuu-CAM 的摄像头 DVP 接口占用大量 GPIO，部分板型会将 GPIO19/20 用于摄像头相关信号，导致原生 USB 不可用。烧录时优先使用 CH340 串口口；如需原生 USB，请确认摄像头排线未占用相应引脚。

2. **PSRAM 使用**：图像帧缓冲（frame buffer）应优先分配在 PSRAM 中。`esp-alloc` 需正确配置 PSRAM 区域，否则在高分辨率下可能因内部 SRAM 不足而采集失败。

3. **电源**：摄像头 + Wi-Fi + PSRAM 同时工作时峰值电流较高，建议通过 5V Type-C 供电，避免仅靠调试器供电导致复位。

4. **SD 卡模式**：Goouuu-CAM 的 MicroSD 为 **1-bit SDMMC 模式**（CMD=IO38 / CLK=IO39 / DATA0=IO40），不是 SPI、也不是 4-bit 模式。文件系统初始化时需按 1-bit 配置，否则挂载失败。

5. **WS2812 电平**：GPIO48 输出 3.3V 逻辑电平驱动 WS2812，部分灯珠对 3.3V 高电平阈值较敏感；如出现颜色异常，可外接电平转换或上拉。

6. **无板载总线/用户 LED**：Goouuu-CAM 没有板载 CAN / RS485 / RTC 收发器，也无独立用户 LED（IO43=U0TXD 为串口/日志引脚）。如需工业总线，请自行通过排针外接模块并注意电平/隔离；本项目不再内置相关代码。

7. **摄像头时钟**：OV2640 常用 XCLK 为 20MHz，部分场景可降至 10MHz 以提高稳定性；OV5640 对时钟和电源要求更高，高分辨率下需确保 PSRAM 带宽充足。


## 开发状态

- ✅ Wi-Fi STA 连接
- ✅ DHCP 获取 IP
- ✅ HTTP 服务器（picoserve）
- ✅ 摄像头 OV2640 初始化与 JPEG 采集（QQVGA 实测出图）
- ✅ WS2812 RGB 网页控制
- 🚧 MJPEG 视频流推送
- 🚧 SD 卡 1-bit SDMMC 挂载与读写
- 🚧 BLE 通信


## 许可证

MIT OR Apache-2.0