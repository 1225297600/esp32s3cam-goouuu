// src/app/task_camera.rs
//
// DVP 摄像头采集任务：
// - 通过 I2C(SCCB) 初始化 OV2640（JPEG 输出）
// - 通过 lcd_cam::Camera + DMA 流缓冲持续接收字节流
// - 按 FFD8...FFD9 切分 JPEG 帧，发布最新一帧供 web 任务读取
use alloc::vec::Vec;
use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};
use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_time::{Instant, Timer};
use esp_hal::dma::DmaRxStreamBuf;
use esp_hal::gpio::Output;
use esp_hal::i2c::master::I2c;
use esp_hal::lcd_cam::cam::Camera;
use esp_hal::peripherals::RMT;
use esp_hal::Async;

use crate::app::task_ws2812::task_ws2812;
use crate::drivers::ov2640::{self, JpegMode};

/// 单帧最大发布字节数（与 web 端共享）
/// UXGA 高质量 JPEG 帧可达数十 KB，上限需足够大，否则会被丢弃
pub const MAX_FRAME_LEN: usize = 64 * 1024;

/// 最新 JPEG 帧 + 帧序号（web 端轮询序号检测新帧）
pub static FRAME: Mutex<CriticalSectionRawMutex, RefCell<FrameSlot>> =
    Mutex::new(RefCell::new(FrameSlot::new()));

/// 摄像头参数设置请求（quality, mode）
pub static CAMERA_SET_CHANNEL: Channel<CriticalSectionRawMutex, (u8, JpegMode), 2> = Channel::new();

/// 摄像头状态快照（供 /api/status）
pub static CAMERA_STATUS: Mutex<CriticalSectionRawMutex, RefCell<CameraStatus>> =
    Mutex::new(RefCell::new(CameraStatus::new()));

/// 运行秒数（由本任务维护，供 /api/status）
pub static UPTIME_SECS: AtomicU32 = AtomicU32::new(0);

pub struct FrameSlot {
    pub seq: u32,
    pub data: Option<Vec<u8>>,
}

impl FrameSlot {
    const fn new() -> Self {
        Self { seq: 0, data: None }
    }
}

#[derive(Clone, Copy)]
pub struct CameraStatus {
    pub present: bool,
    pub quality: u8,
    pub mode: JpegMode,
    pub frame_seq: u32,
    pub frame_len: u32,
    pub fps: u16,
    pub kbps: u16,
}

impl CameraStatus {
    const fn new() -> Self {
        Self {
            present: false,
            quality: 0,
            mode: JpegMode::QQVGA,
            frame_seq: 0,
            frame_len: 0,
            fps: 0,
            kbps: 0,
        }
    }
}

/// 取最新一帧 JPEG 的拷贝；seq=0 表示尚无有效帧
pub fn latest_frame() -> (u32, Option<Vec<u8>>) {
    FRAME.lock(|slot| {
        let s = slot.borrow();
        (s.seq, s.data.as_ref().cloned())
    })
}

/// 仅取当前帧序号（不拷贝数据，供推流轮询新帧）
pub fn frame_seq() -> u32 {
    FRAME.lock(|slot| slot.borrow().seq)
}

/// 发布设置请求
pub fn request_params(quality: u8, mode: JpegMode) -> Result<(), ()> {
    CAMERA_SET_CHANNEL.try_send((quality, mode)).map_err(|_| ())
}

/// 从数字解析分辨率模式（web API 用）
pub fn parse_mode(v: u8) -> Option<JpegMode> {
    match v {
        0 => Some(JpegMode::QQVGA),
        1 => Some(JpegMode::QVGA),
        2 => Some(JpegMode::VGA),
        3 => Some(JpegMode::SVGA),
        4 => Some(JpegMode::UXGA),
        _ => None,
    }
}

/// 分辨率模式编号（web API 用）
pub fn mode_number(mode: JpegMode) -> u8 {
    match mode {
        JpegMode::QQVGA => 0,
        JpegMode::QVGA => 1,
        JpegMode::VGA => 2,
        JpegMode::SVGA => 3,
        JpegMode::UXGA => 4,
    }
}

fn publish_status(present: bool, quality: u8, mode: JpegMode, len: usize, fps: u16, kbps: u16) {
    let seq = FRAME.lock(|slot| slot.borrow().seq);
    CAMERA_STATUS.lock(|s| {
        *s.borrow_mut() = CameraStatus {
            present,
            quality,
            mode,
            frame_seq: seq,
            frame_len: len as u32,
            fps,
            kbps,
        };
    });
}

/// 在一段 DMA 数据中扫描 JPEG 帧；完整帧写入全局 FRAME
///
/// 返回 (本次产出的帧数, 产出的总字节数)
fn feed(asm: &mut Vec<u8>, prev_tail: &mut Option<u8>, data: &[u8]) -> (u32, usize) {
    let mut frames = 0u32;
    let mut bytes = 0usize;

    for &b in data {
        if asm.is_empty() {
            // 等待 SOI (FFD8)
            if *prev_tail == Some(0xFF) && b == 0xD8 {
                asm.push(0xFF);
                asm.push(0xD8);
            }
            *prev_tail = Some(b);
            continue;
        }

        // 帧内又遇到 SOI：说明上一帧的 EOI 因 DMA 间隙丢失，
        // 丢弃这个残缺帧，从新的 SOI 重新开始（避免两帧粘连成花屏）
        if *prev_tail == Some(0xFF) && b == 0xD8 {
            asm.clear();
            asm.push(0xFF);
            asm.push(0xD8);
            *prev_tail = Some(b);
            continue;
        }

        asm.push(b);

        // 找到 EOI (FFD9) → 一帧结束
        if *prev_tail == Some(0xFF) && b == 0xD9 {
            if asm.len() <= MAX_FRAME_LEN {
                FRAME.lock(|slot| {
                    let mut s = slot.borrow_mut();
                    s.seq += 1;
                    s.data = Some(alloc::vec::Vec::from(asm.as_slice()));
                });
                frames += 1;
                bytes += asm.len();
            } else {
                error!("[cam] frame too large ({} B), dropped", asm.len());
            }
            asm.clear();
        } else if asm.len() >= MAX_FRAME_LEN {
            // 超长且未见 EOI：丢弃重来
            asm.clear();
        }

        *prev_tail = Some(b);
    }

    (frames, bytes)
}

#[embassy_executor::task]
pub async fn task_camera(
    mut i2c: I2c<'static, Async>,
    mut camera: Camera<'static>,
    mut dma_buf: DmaRxStreamBuf,
    rmt_peri: RMT<'static>,
    ws_pin: Output<'static>,
    spawner: Spawner,
) -> ! {
    // ---- 板载 LED 任务（spawn 失败必须可见，否则灯不亮无从排查）----
    match task_ws2812(rmt_peri, ws_pin) {
        Ok(fut) => {
            spawner.spawn(fut);
            info!("[cam] ws2812 task spawned");
        }
        Err(_) => error!("[cam] ws2812 future exceeds pool cell size"),
    }

    // ---- 传感器探测与初始化 ----
    let mut quality = ov2640::REG_QS_DEFAULT;
    let mut mode = JpegMode::QQVGA;
    let mut present = false;

    for _ in 0..3 {
        match ov2640::chip_id(&mut i2c).await {
            Ok(id) if (id & 0xff00) == 0x2600 => {
                info!("[cam] OV2640 detected, id=0x{:04X}", id);
                present = true;
                break;
            }
            Ok(id) => error!("[cam] unexpected sensor id=0x{:04X}", id),
            Err(e) => {
                error!("[cam] i2c probe failed: {:?}", e);
                Timer::after_millis(100).await;
            }
        }
    }

    if present {
        ov2640::soft_reset(&mut i2c).await.ok();
        Timer::after_millis(300).await;
        match ov2640::apply_init(&mut i2c, mode, quality).await {
            Ok(_) => info!("[cam] OV2640 initialized, JPEG mode={:?}", mode),
            Err(e) => {
                error!("[cam] init failed: {:?}", e);
                present = false;
            }
        }
    } else {
        error!("[cam] no OV2640 found, camera disabled");
    }

    // ---- 采集主循环 ----
    let start = Instant::now();
    let mut asm: Vec<u8> = Vec::with_capacity(MAX_FRAME_LEN + 8);
    let mut prev_tail: Option<u8> = None;
    let mut frame_len = 0usize;
    let mut win = Instant::now();
    let mut win_frames: u32 = 0;
    let mut win_bytes: usize = 0;
    let mut fps: u16 = 0;
    let mut kbps: u16 = 0;
    let mut pop_buf = [0u8; 4096];

    loop {
        UPTIME_SECS.store(start.elapsed().as_secs() as u32, Ordering::Relaxed);

        if !present {
            if let Ok((q, m)) = CAMERA_SET_CHANNEL.try_receive() {
                quality = q;
                mode = m;
            }
            publish_status(false, quality, mode, 0, 0, 0);
            Timer::after_millis(500).await;
            continue;
        }

        let mut transfer = match camera.receive(dma_buf) {
            Ok(t) => t,
            Err((e, cam, buf)) => {
                error!("[cam] dma start failed: {:?}", e);
                camera = cam;
                dma_buf = buf;
                Timer::after_millis(500).await;
                continue;
            }
        };

        loop {
            // 消费 DMA 已写入的数据；每轮最多排空 ~16KB，保证周期性让出单线程执行器
            // （否则高像素流下 available_bytes() 永不归零，while 死循环会饿死 web/net 等任务）
            let mut drained = 0usize;
            while drained < 16384 {
                if transfer.available_bytes() == 0 {
                    break;
                }
                let n = transfer.pop(&mut pop_buf);
                if n == 0 {
                    break;
                }
                drained += n;
                let (f, b) = feed(&mut asm, &mut prev_tail, &pop_buf[..n]);
                win_frames += f;
                win_bytes += b;
                if b > 0 {
                    FRAME.lock(|slot| {
                        if let Some(data) = &slot.borrow().data {
                            frame_len = data.len();
                        }
                    });
                }
            }

            // 环满 / 硬件停止：重启接收以重置视图；参数请求也在此处理
            let done = transfer.is_done();
            let param = CAMERA_SET_CHANNEL.try_receive().ok();

            if done || param.is_some() {
                let (cam, buf) = transfer.stop();
                camera = cam;
                dma_buf = buf;
                if let Some((q, m)) = param {
                    quality = q;
                    mode = m;
                    asm.clear();
                    prev_tail = None;
                    match ov2640::set_mode(&mut i2c, mode, quality).await {
                        Ok(_) => info!("[cam] params updated: mode={:?} qs={}", mode, quality),
                        Err(e) => error!("[cam] set_mode failed: {:?}", e),
                    }
                }
                break;
            }

            Timer::after_millis(1).await;
        }

        // 5 秒统计窗口
        if win.elapsed().as_millis() >= 5000 {
            fps = (win_frames / 5) as u16;
            kbps = (win_bytes / 625) as u16; // (bytes * 8 / 5) / 1000
            win = Instant::now();
            win_frames = 0;
            win_bytes = 0;
        }

        publish_status(true, quality, mode, frame_len, fps, kbps);
    }
}
