//! OV2640 DVP 摄像头传感器驱动（SCCB/I2C 寄存器配置）
//!
//! 寄存器表移植自 Zephyr 驱动的 ov2640.c（Apache-2.0），
//! 以 UXGA 1600x1200 为基础窗口，通过 DSP zoom 通道缩放至目标分辨率，
//! 输出 JPEG 压缩码流（适合 esp-hal `lcd_cam::cam::Camera` 的 8bit DVP 接收）。

use defmt::info;
use esp_hal::i2c::master::I2c;
use esp_hal::Async;

/// OV2640 SCCB 7 位地址（8 位写地址 0x60 >> 1 = 0x30）
pub const ADDR: u8 = 0x30;

// ---------------- DSP 寄存器组 (bank FF=0x00) ----------------
pub const REG_QS: u8 = 0x44; // JPEG 质量因子，越小质量越高
const REG_HSIZE: u8 = 0x51;
const REG_VSIZE: u8 = 0x52;
const REG_XOFFL: u8 = 0x53;
const REG_YOFFL: u8 = 0x54;
const REG_VHYX: u8 = 0x55;
const REG_TEST: u8 = 0x57;
const REG_ZMOW: u8 = 0x5A;
const REG_ZMOH: u8 = 0x5B;
const REG_ZMHH: u8 = 0x5C;
const REG_BPADDR: u8 = 0x7C;
const REG_BPDATA: u8 = 0x7D;
const REG_SIZEL: u8 = 0x8C;
const REG_HSIZE8: u8 = 0xC0;
const REG_VSIZE8: u8 = 0xC1;
#[allow(dead_code)]
const REG_CTRL1: u8 = 0xC3; // AWB 使能位
const REG_CTRLI: u8 = 0x50;
const REG_CTRL0: u8 = 0xC2;
const REG_CTRL2: u8 = 0x86;
const REG_CTRL3: u8 = 0x87;
const REG_RDVPSP: u8 = 0xD3;
const REG_R_BYPASS: u8 = 0x05;
const REG_IMAGE_MODE: u8 = 0xDA;
const REG_RESET: u8 = 0xE0;
const REG_MC_BIST: u8 = 0xF9;
const REG_BANK_SEL: u8 = 0xFF;

const BANK_SEL_DSP: u8 = 0x00;
const BANK_SEL_SENSOR: u8 = 0x01;

// ---------------- Sensor 寄存器组 (bank FF=0x01) ----------------
const REG_COM1: u8 = 0x03;
const REG_PID: u8 = 0x0A;
const REG_VER: u8 = 0x0B;
#[allow(dead_code)]
const REG_AEC: u8 = 0x10;
const REG_CLKRC: u8 = 0x11;
const REG_COM10: u8 = 0x15;
const REG_HSTART: u8 = 0x17;
const REG_HSTOP: u8 = 0x18;
const REG_VSTART: u8 = 0x19;
const REG_VSTOP: u8 = 0x1A;
const REG_AEW: u8 = 0x24;
const REG_AEB: u8 = 0x25;
const REG_ARCOM2: u8 = 0x34;
const REG_FLL: u8 = 0x46;
const REG_FLH: u8 = 0x47;
const REG_COM19: u8 = 0x48;
const REG_ZOOMS: u8 = 0x49;
const REG_BD50: u8 = 0x4F;
const REG_BD60: u8 = 0x50;
const REG_COM2: u8 = 0x09;
const REG_COM3: u8 = 0x0C;
const REG_COM7: u8 = 0x12;
const REG_COM8: u8 = 0x13;
const REG_COM9: u8 = 0x14;
const REG_REG04: u8 = 0x04;
const REG_REG32: u8 = 0x32;
const REG_VV: u8 = 0x26;
const REG_AWB: u8 = 0xC3; // 与 DSP CTRL1 同址（此表实际写在 sensor bank 的 0xC3 无需切 bank，保持与 Zephyr 一致的写法）

type Reg = (u8, u8);

/// DSP bank：公共初始化序列（Zephyr default_regs）
const DEFAULT_REGS: &[Reg] = &[
    (REG_BANK_SEL, BANK_SEL_DSP),
    (0x2C, 0xFF),
    (0x2E, 0xDF),
    (REG_BANK_SEL, BANK_SEL_SENSOR),
    (0x3C, 0x32),
    (REG_CLKRC, 0x80),
    (REG_COM2, 0x02), // 输出驱动 x2
    (REG_REG04, 0x28 | 0x08), // HREF 使能
    (REG_COM8, 0xC0 | 0x20 | 0x04 | 0x01), // BNDF + AGC + AEC 自动
    (REG_COM9, 0x08 | (0x02 << 5)), // AGC 上限 8x
    (REG_COM10, 0x00), // VSYNC 反相
    (0x2C, 0x0C),
    (0x33, 0x78),
    (0x3A, 0x33),
    (0x3B, 0xFB),
    (0x3E, 0x00),
    (0x43, 0x11),
    (0x16, 0x10),
    (0x39, 0x02),
    (0x35, 0x88),
    (0x22, 0x0A),
    (0x37, 0x40),
    (0x23, 0x00),
    (REG_ARCOM2, 0xA0),
    (0x06, 0x02),
    (0x06, 0x88),
    (0x07, 0xC0),
    (0x0D, 0xB7),
    (0x0E, 0x01),
    (0x4C, 0x00),
    (0x4A, 0x81),
    (0x21, 0x99),
    (REG_AEW, 0x40),
    (REG_AEB, 0x38),
    (REG_VV, 0x08 << 4 | 0x02), // AGC 阈值
    (REG_COM19, 0x00),
    (REG_ZOOMS, 0x00),
    (0x5C, 0x00),
    (0x63, 0x00),
    (REG_FLL, 0x00),
    (REG_FLH, 0x00),
    (REG_COM3, 0x38 | 0x02), // 自动banding滤波
    (0x5D, 0x55),
    (0x5E, 0x7D),
    (0x5F, 0x7D),
    (0x60, 0x55),
    (0x61, 0x70), // HISTO_LOW
    (0x62, 0x80), // HISTO_HIGH
    (0x7C, 0x05),
    (0x20, 0x80),
    (0x28, 0x30),
    (0x6C, 0x00),
    (0x6D, 0x80),
    (0x6E, 0x00),
    (0x70, 0x02),
    (0x71, 0x94),
    (0x73, 0xC1),
    (0x3D, 0x34),
    (0x5A, 0x57),
    (REG_BD50, 0xBB),
    (REG_BD60, 0x9C),
    (REG_BANK_SEL, BANK_SEL_DSP),
    (0xE5, 0x7F),
    (REG_MC_BIST, 0x80 | 0x40),
    (0x41, 0x24),
    (REG_RESET, 0x10 | 0x04), // 复位 JPEG 与 DVP
    (0x76, 0xFF),
    (0x33, 0xA0),
    (0x42, 0x20),
    (0x43, 0x18),
    (0x4C, 0x00),
    (REG_CTRL3, 0x80 | 0x40 | 0x10), // BPC + WPC
    (0x88, 0x3F),
    (0xD7, 0x03),
    (0xD9, 0x10),
    (REG_RDVPSP, 0x80 | 0x02),
    (0xC8, 0x08),
    (0xC9, 0x80),
    (REG_BPADDR, 0x00),
    (REG_BPDATA, 0x00),
    (REG_BPADDR, 0x03),
    (REG_BPDATA, 0x48),
    (REG_BPDATA, 0x48),
    (REG_BPADDR, 0x08),
    (REG_BPDATA, 0x20),
    (REG_BPDATA, 0x10),
    (REG_BPDATA, 0x0E),
    (0x90, 0x00),
    (0x91, 0x0E),
    (0x91, 0x1A),
    (0x91, 0x31),
    (0x91, 0x5A),
    (0x91, 0x69),
    (0x91, 0x75),
    (0x91, 0x7E),
    (0x91, 0x88),
    (0x91, 0x8F),
    (0x91, 0x96),
    (0x91, 0xA3),
    (0x91, 0xAF),
    (0x91, 0xC4),
    (0x91, 0xD7),
    (0x91, 0xE8),
    (0x91, 0x20),
    (0x92, 0x00),
    (0x93, 0x06),
    (0x93, 0xE3),
    (0x93, 0x03),
    (0x93, 0x03),
    (0x93, 0x00),
    (0x93, 0x02),
    (0x93, 0x00),
    (0x93, 0x00),
    (0x93, 0x00),
    (0x93, 0x00),
    (0x93, 0x00),
    (0x93, 0x00),
    (0x93, 0x00),
    (0x96, 0x00),
    (0x97, 0x08),
    (0x97, 0x19),
    (0x97, 0x02),
    (0x97, 0x0C),
    (0x97, 0x24),
    (0x97, 0x30),
    (0x97, 0x28),
    (0x97, 0x26),
    (0x97, 0x02),
    (0x97, 0x98),
    (0x97, 0x80),
    (0x97, 0x00),
    (0x97, 0x00),
    (0xA4, 0x00),
    (0xA8, 0x00),
    (0xC5, 0x11),
    (0xC6, 0x51),
    (0xBF, 0x80),
    (0xC7, 0x10),
    (0xB6, 0x66),
    (0xB8, 0xA5),
    (0xB7, 0x64),
    (0xB9, 0x7C),
    (0xB3, 0xAF),
    (0xB4, 0x97),
    (0xB5, 0xFF),
    (0xB0, 0xC5),
    (0xB1, 0x94),
    (0xB2, 0x0F),
    (0xC4, 0x5C),
    (0xA6, 0x00),
    (0xA7, 0x20),
    (0xA7, 0xD8),
    (0xA7, 0x1B),
    (0xA7, 0x31),
    (0xA7, 0x00),
    (0xA7, 0x18),
    (0xA7, 0x20),
    (0xA7, 0xD8),
    (0xA7, 0x19),
    (0xA7, 0x31),
    (0xA7, 0x00),
    (0xA7, 0x18),
    (0xA7, 0x20),
    (0xA7, 0xD8),
    (0xA7, 0x19),
    (0xA7, 0x31),
    (0xA7, 0x00),
    (0xA7, 0x18),
    (0x7F, 0x00),
    (0xE5, 0x1F),
    (0xE1, 0x77),
    (0xDD, 0x7F),
    (REG_CTRL0, 0x08 | 0x04 | 0x02), // YUV422 + YUV EN + RGB EN
    (0x00, 0x00),
];

/// Sensor + DSP bank：UXGA 1600x1200 基础窗口（Zephyr uxga_regs）
const UXGA_REGS: &[Reg] = &[
    (REG_BANK_SEL, BANK_SEL_SENSOR),
    (REG_COM7, 0x00), // UXGA
    (REG_COM1, 0x0F),
    (REG_REG32, 0x36),
    (REG_HSTART, 0x11),
    (REG_HSTOP, 0x75),
    (REG_VSTART, 0x01),
    (REG_VSTOP, 0x97),
    (0x3D, 0x34),
    (0x35, 0x88),
    (0x22, 0x0A),
    (0x37, 0x40),
    (REG_ARCOM2, 0xA0),
    (0x06, 0x02),
    (0x0D, 0xB7),
    (0x0E, 0x01),
    (0x42, 0x83),
    (REG_BANK_SEL, BANK_SEL_DSP),
    (REG_R_BYPASS, 0x01), // DSP bypass
    (REG_RESET, 0x04),    // 复位 DVP
    (REG_HSIZE8, (1600u16 >> 3) as u8),
    (REG_VSIZE8, (1200u16 >> 3) as u8),
    // {HSIZE[11], HSIZE[2:0], VSIZE[2:0]}: UXGA 固定值
    (REG_SIZEL, 0x00),
    (REG_XOFFL, 0x00),
    (REG_YOFFL, 0x00),
    (REG_HSIZE, ((1600u16 >> 2) & 0xFF) as u8),
    (REG_VSIZE, ((1200u16 >> 2) & 0xFF) as u8),
    (
        REG_VHYX,
        ((((1200u16 >> 3) & 0x80) | ((1600u16 >> 7) & 0x08)) as u8),
    ),
    (REG_TEST, ((1600u16 >> 4) & 0x80) as u8),
    (REG_CTRL2, 0x20 | 0x10 | 0x04 | 0x01 | 0x08), // DCW + SDE + UV_AVG + CMX + UV_ADJ
    (REG_CTRLI, 0x80 | 0x00),
    (REG_RDVPSP, 0x80 | 0x04),
    (REG_R_BYPASS, 0x00), // DSP 使能
    (REG_RESET, 0x00),
];

/// 输出分辨率（经 zoom 通道从 UXGA 缩放）
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum JpegMode {
    /// 160x120
    QQVGA,
    /// 320x240
    QVGA,
    /// 640x480
    VGA,
    /// 800x600
    SVGA,
    /// 1600x1200（原生 UXGA，无缩放）
    UXGA,
}

impl JpegMode {
    pub const fn size(self) -> (u16, u16) {
        match self {
            JpegMode::QQVGA => (160, 120),
            JpegMode::QVGA => (320, 240),
            JpegMode::VGA => (640, 480),
            JpegMode::SVGA => (800, 600),
            JpegMode::UXGA => (1600, 1200),
        }
    }

    pub const fn name(self) -> &'static str {
        match self {
            JpegMode::QQVGA => "QQVGA",
            JpegMode::QVGA => "QVGA",
            JpegMode::VGA => "VGA",
            JpegMode::SVGA => "SVGA",
            JpegMode::UXGA => "UXGA",
        }
    }

    /// (v_div, h_div, pclk_div) —— 与 Zephyr OV2640_ZOOM_CONFIG 一致
    const fn zoom_params(self) -> (u8, u8, u8) {
        match self {
            JpegMode::QQVGA => (3, 3, 8),
            JpegMode::QVGA => (2, 2, 4),
            JpegMode::VGA => (0, 0, 2),
            JpegMode::SVGA => (1, 1, 2),
            JpegMode::UXGA => (0, 0, 0),
        }
    }
}

/// 应用初始化配置（不含复位，复位需在上电后另行执行）
pub async fn apply_init(i2c: &mut I2c<'static, Async>, mode: JpegMode, qs: u8) -> Result<(), esp_hal::i2c::master::Error> {
    write_all(i2c, DEFAULT_REGS).await?;
    write_all(i2c, UXGA_REGS).await?;

    let (w, h) = mode.size();
    let (v_div, h_div, pclk_div) = mode.zoom_params();

    // 始终写入 zoom 配置（UXGA 为 1:1 全幅）。
    // 若跳过会残留上一次缩放寄存器，导致 UXGA 实际输出 CIF(352x288)。
    // R_DVP_SP：pclk_div==0 时用自动模式(0x80)，否则用分频值（与 Zephyr 一致）
    let rdvsp = if pclk_div == 0 { 0x80 } else { pclk_div };
    let zoom: &[Reg] = &[
        (REG_CTRLI, 0x80 | ((v_div & 0x07) << 3) | (h_div & 0x07)),
        (REG_ZMOW, (w >> 2) as u8),
        (REG_ZMOH, ((h >> 2) & 0xFF) as u8),
        (REG_ZMHH, (((w >> 10) & 0x03) as u8) | ((((h >> 10) & 0x01) as u8) << 2)),
        (REG_RDVPSP, rdvsp),
        (REG_RESET, 0x00),
    ];
    write_all(i2c, zoom).await?;

    // JPEG 输出 + 质量
    write_reg(i2c, REG_BANK_SEL, BANK_SEL_DSP).await?;
    write_reg(i2c, REG_IMAGE_MODE, 0x10).await?; // JPEG 使能
    write_reg(i2c, REG_QS, qs).await?;

    info!("[ov2640] jpeg mode set, qs={}", qs);
    Ok(())
}

/// 读取芯片 ID，OV2640 应返回 0x2642
pub async fn chip_id(i2c: &mut I2c<'static, Async>) -> Result<u16, esp_hal::i2c::master::Error> {
    write_reg(i2c, REG_BANK_SEL, BANK_SEL_SENSOR).await?;
    let pid = read_reg(i2c, REG_PID).await?;
    let ver = read_reg(i2c, REG_VER).await?;
    Ok(((pid as u16) << 8) | ver as u16)
}

/// 软复位：切到 sensor bank 后写 COM7[7]
pub async fn soft_reset(i2c: &mut I2c<'static, Async>) -> Result<(), esp_hal::i2c::master::Error> {
    write_reg(i2c, REG_BANK_SEL, BANK_SEL_SENSOR).await?;
    write_reg(i2c, REG_COM7, 0x80).await
}

/// 设置 JPEG 质量（10=最好，63=最差）
pub async fn set_quality(i2c: &mut I2c<'static, Async>, qs: u8) -> Result<(), esp_hal::i2c::master::Error> {
    write_reg(i2c, REG_BANK_SEL, BANK_SEL_DSP).await?;
    write_reg(i2c, REG_QS, qs).await
}

/// 切换输出分辨率
pub async fn set_mode(i2c: &mut I2c<'static, Async>, mode: JpegMode, qs: u8) -> Result<(), esp_hal::i2c::master::Error> {
    apply_init(i2c, mode, qs).await
}

pub const REG_QS_DEFAULT: u8 = 15;

/// 亮度等级 -2..+2（Zephyr brightness_regs）
pub async fn set_brightness(i2c: &mut I2c<'static, Async>, level: i8) -> Result<(), esp_hal::i2c::master::Error> {
    const TABLE: [[u8; 5]; 6] = [
        [REG_BPADDR, REG_BPDATA, REG_BPADDR, REG_BPDATA, REG_BPDATA],
        [0x00, 0x04, 0x09, 0x00, 0x00],
        [0x00, 0x04, 0x09, 0x10, 0x00],
        [0x00, 0x04, 0x09, 0x20, 0x00],
        [0x00, 0x04, 0x09, 0x30, 0x00],
        [0x00, 0x04, 0x09, 0x40, 0x00],
    ];
    apply_special(i2c, &TABLE, level).await
}

/// 对比度等级 -2..+2（Zephyr contrast_regs）
pub async fn set_contrast(i2c: &mut I2c<'static, Async>, level: i8) -> Result<(), esp_hal::i2c::master::Error> {
    const TABLE: [[u8; 7]; 6] = [
        [REG_BPADDR, REG_BPDATA, REG_BPADDR, REG_BPDATA, REG_BPDATA, REG_BPDATA, REG_BPDATA],
        [0x00, 0x04, 0x07, 0x20, 0x18, 0x34, 0x06],
        [0x00, 0x04, 0x07, 0x20, 0x1C, 0x2A, 0x06],
        [0x00, 0x04, 0x07, 0x20, 0x20, 0x20, 0x06],
        [0x00, 0x04, 0x07, 0x20, 0x24, 0x16, 0x06],
        [0x00, 0x04, 0x07, 0x20, 0x28, 0x0C, 0x06],
    ];
    apply_special(i2c, &TABLE, level).await
}

/// 饱和度等级 -2..+2（Zephyr saturation_regs）
pub async fn set_saturation(i2c: &mut I2c<'static, Async>, level: i8) -> Result<(), esp_hal::i2c::master::Error> {
    const TABLE: [[u8; 5]; 6] = [
        [REG_BPADDR, REG_BPDATA, REG_BPADDR, REG_BPDATA, REG_BPDATA],
        [0x00, 0x02, 0x03, 0x28, 0x28],
        [0x00, 0x02, 0x03, 0x38, 0x38],
        [0x00, 0x02, 0x03, 0x48, 0x48],
        [0x00, 0x02, 0x03, 0x58, 0x48],
        [0x00, 0x02, 0x03, 0x58, 0x48],
    ];
    apply_special(i2c, &TABLE, level).await
}

/// 特殊效果表写入：第一行为寄存器地址序列，后续行为等级 00..04（-2..+2）
async fn apply_special<const C: usize>(
    i2c: &mut I2c<'static, Async>,
    table: &[[u8; C]; 6],
    level: i8,
) -> Result<(), esp_hal::i2c::master::Error> {
    let idx = (level.clamp(-2, 2) + 2) as usize + 1;
    write_reg(i2c, REG_BANK_SEL, BANK_SEL_DSP).await?;
    let addrs = &table[0];
    let values = &table[idx];
    for i in 0..C {
        write_reg(i2c, addrs[i], values[i]).await?;
    }
    Ok(())
}

/// 设置特殊效果：0=无，1=负片，2=黑白，3=绿色，4=蓝色，5=红光
pub async fn set_special_effect(i2c: &mut I2c<'static, Async>, effect: u8) -> Result<(), esp_hal::i2c::master::Error> {
    write_reg(i2c, REG_BANK_SEL, BANK_SEL_DSP).await?;
    match effect {
        0 => {
            write_reg(i2c, REG_BPADDR, 0x00).await?;
            write_reg(i2c, REG_BPDATA, 0x00).await?;
            write_reg(i2c, REG_BPADDR, 0x01).await?;
            write_reg(i2c, REG_BPDATA, 0x00).await?;
            write_reg(i2c, REG_BPADDR, 0x02).await?;
            write_reg(i2c, REG_BPDATA, 0x00).await?;
        }
        1 => {
            write_reg(i2c, REG_BPADDR, 0x06).await?;
            write_reg(i2c, REG_BPDATA, 0x01).await?;
        }
        2 => {
            write_reg(i2c, REG_BPADDR, 0x06).await?;
            write_reg(i2c, REG_BPDATA, 0x80).await?;
        }
        _ => {}
    }
    Ok(())
}

/// 自动白平衡开关
pub async fn set_awb(i2c: &mut I2c<'static, Async>, enable: bool) -> Result<(), esp_hal::i2c::master::Error> {
    write_reg(i2c, REG_BANK_SEL, BANK_SEL_SENSOR).await?;
    let mut reg = read_reg(i2c, REG_AWB).await?;
    if enable {
        reg |= 0x08;
    } else {
        reg &= !0x08;
    }
    write_reg(i2c, REG_AWB, reg).await
}

/// 水平镜像
pub async fn set_hmirror(i2c: &mut I2c<'static, Async>, enable: bool) -> Result<(), esp_hal::i2c::master::Error> {
    write_reg(i2c, REG_BANK_SEL, BANK_SEL_SENSOR).await?;
    let mut reg = read_reg(i2c, REG_REG04).await?;
    if enable {
        reg |= 0x80;
    } else {
        reg &= !0x80;
    }
    write_reg(i2c, REG_REG04, reg).await
}

/// 垂直翻转
pub async fn set_vflip(i2c: &mut I2c<'static, Async>, enable: bool) -> Result<(), esp_hal::i2c::master::Error> {
    write_reg(i2c, REG_BANK_SEL, BANK_SEL_SENSOR).await?;
    let mut reg = read_reg(i2c, REG_REG04).await?;
    if enable {
        reg |= 0x40;
    } else {
        reg &= !0x40;
    }
    write_reg(i2c, REG_REG04, reg).await
}

/// 测试彩条开关
pub async fn set_colorbar(i2c: &mut I2c<'static, Async>, enable: bool) -> Result<(), esp_hal::i2c::master::Error> {
    write_reg(i2c, REG_BANK_SEL, BANK_SEL_SENSOR).await?;
    let mut reg = read_reg(i2c, REG_COM7).await?;
    if enable {
        reg |= 0x02;
    } else {
        reg &= !0x02;
    }
    write_reg(i2c, REG_COM7, reg).await
}

/// 帧率控制：写入 sensor REG5D/5E/5F/60（Zephyr set_frame_rate 简化版）
pub async fn set_aec_exposure(i2c: &mut I2c<'static, Async>, gain: u8, exposure: u16) -> Result<(), esp_hal::i2c::master::Error> {
    write_reg(i2c, REG_BANK_SEL, BANK_SEL_SENSOR).await?;
    write_reg(i2c, 0x00, (exposure & 0xFF) as u8).await?; // AEC low
    write_reg(i2c, 0x05, (exposure >> 8) as u8).await?; // AEC high
    write_reg(i2c, REG_AGC, gain).await?;
    Ok(())
}

const REG_AGC: u8 = 0x00; // sensor bank 0x00 = AGC（manual gain 时）

pub async fn write_reg(i2c: &mut I2c<'static, Async>, reg: u8, value: u8) -> Result<(), esp_hal::i2c::master::Error> {
    i2c.write_async(ADDR, &[reg, value]).await
}

pub async fn read_reg(i2c: &mut I2c<'static, Async>, reg: u8) -> Result<u8, esp_hal::i2c::master::Error> {
    let mut buf = [0u8; 1];
    i2c.write_read_async(ADDR, &[reg], &mut buf).await?;
    Ok(buf[0])
}

async fn write_all(i2c: &mut I2c<'static, Async>, regs: &[Reg]) -> Result<(), esp_hal::i2c::master::Error> {
    for &(reg, value) in regs {
        write_reg(i2c, reg, value).await?;
    }
    Ok(())
}
