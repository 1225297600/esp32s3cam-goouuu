#![allow(linker_messages)]
#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::Config as I2cConfig;
use esp_hal::i2c::master::I2c;
use esp_hal::lcd_cam::cam::{Camera, Config as CamConfig, EofMode};
use esp_hal::lcd_cam::LcdCam;
use esp_hal::timer::timg::TimerGroup;
use esp_println as _;

use wx_esp32s3::app::task_camera::task_camera;
use wx_esp32s3::app::task_net::task_net;
use wx_esp32s3::app::task_sd;

#[panic_handler]
fn panic(panic_info: &core::panic::PanicInfo) -> ! {
    error!("{}", panic_info);
    loop {}
}

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

/// DMA 接收环形缓冲大小（字节）
const DMA_RX_SIZE: usize = 16 * 1024;
/// 单个 DMA 描述符块大小
const DMA_CHUNK_SIZE: usize = 2048;

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // ---- HAL 初始化 ----
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // ---- 初始化堆分配器 ----
    // reclaimed 区专用；普通 DRAM 堆取保守值，为共享的 main-task 执行器栈留出空间
    // （大图/帧缓冲优先从 PSRAM 堆分配）
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    esp_alloc::heap_allocator!(size: 24 * 1024);

    // ---- PSRAM（N16R8：8MB，Auto 探测八线/四线模式）----
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    info!("PSRAM allocator registered");

    // ---- 初始化 RTOS ----
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    info!("Embassy initialized!");

    // ---- 摄像头 SCCB (I2C0): CAM_SIOD=GPIO4, CAM_SIOC=GPIO5 ----
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(peripherals.GPIO4)
        .with_scl(peripherals.GPIO5)
        .into_async();

    // ---- DVP 并行接口 (LCD_CAM, 8bit, XCLK 10MHz) ----
    // GOOUUU ESP32-S3-CAM 引脚（等同 CAMERA_MODEL_ESP32S3_EYE）：
    //   XCLK=15  PCLK=13  VSYNC=6  HREF=7
    //   D0..D7 = Y2..Y9 = 11,9,8,10,12,18,17,16
    // XCLK 取 10MHz：排线较长时 20MHz 易采样出错导致花屏，降频换取信号余量
    let cam_cfg = CamConfig::default()
        .with_frequency(esp_hal::time::Rate::from_mhz(10))
        .with_eof_mode(EofMode::VsyncSignal);

    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
    let dma_rx_buf = esp_hal::dma_rx_stream_buffer!(DMA_RX_SIZE, DMA_CHUNK_SIZE);

    let camera = Camera::new(lcd_cam.cam, peripherals.DMA_CH0, cam_cfg)
        .unwrap()
        .with_master_clock(peripherals.GPIO15)
        .with_pixel_clock(peripherals.GPIO13)
        .with_vsync(peripherals.GPIO6)
        .with_h_enable(peripherals.GPIO7)
        .with_data0(peripherals.GPIO11)
        .with_data1(peripherals.GPIO9)
        .with_data2(peripherals.GPIO8)
        .with_data3(peripherals.GPIO10)
        .with_data4(peripherals.GPIO12)
        .with_data5(peripherals.GPIO18)
        .with_data6(peripherals.GPIO17)
        .with_data7(peripherals.GPIO16);

    // ---- 板载 WS2812（GPIO48） ----
    // 注意：GPIO43/44 是 U0TXD/U0RXD（CH343 串口 + defmt 日志），不可复用为 LED。
    let ws2812_led = Output::new(peripherals.GPIO48, Level::Low, OutputConfig::default());

    // ---- SD 卡（native SDMMC 1-bit：CMD=IO38 CLK=IO39 DATA0=IO40）----
    task_sd::try_mount(
        peripherals.SDHOST,
        peripherals.GPIO39,
        peripherals.GPIO38,
        peripherals.GPIO40,
    );

    // ---- 启动任务 ----
    spawner.spawn(task_net(spawner, peripherals.WIFI).unwrap());
    spawner
        .spawn(task_camera(
            i2c,
            camera,
            dma_rx_buf,
            peripherals.RMT,
            ws2812_led,
            spawner,
        ).unwrap());

    info!("All tasks spawned");

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
