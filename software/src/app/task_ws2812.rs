// src/app/task_ws2812.rs
//
// WS2812 RGB LED 控制任务（GPIO48，经 RMT 驱动）
//
// 上机实证参数与机制：
// - tick = 250ns（RMT 计数 8MHz + 通道 divider 1）；100ns 网格本板灯珠无反应。
// - T1H=1500/T1L=500，T0H=500/T0L=1500，bit 周期 2000ns —— 红绿蓝实测全部正确。
// - Blocking 通道 + 整帧关中断（critical_section）发送：
//   S3 每通道 RMT RAM 仅 48 码字，一帧 73 码字须分两批；async 续写依赖 RMT 中断，
//   被 WiFi 临界区延迟 >50µs 时灯珠提前锁存半帧，造成红色串蓝（品红）。
//   关中断阻塞发送根除批次间隙，实测解决。
use defmt::info;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use esp_hal::gpio::{Level, Output};
use esp_hal::peripherals::RMT;
use esp_hal::rmt::{Channel as RmtChannel, PulseCode, Rmt, Tx, TxChannelConfig, TxChannelCreator};
use esp_hal::time::Rate;

/// 1 颗板载 WS2812（GPIO48）
const NUM_LEDS: usize = 1;

pub static WS2812_CHANNEL: Channel<CriticalSectionRawMutex, RGB8, 4> = Channel::new();

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub struct RGB8(pub u8, pub u8, pub u8);

/// 实测有效脉宽（tick=250ns）
const ONE_HIGH: u16 = 6;
const ONE_LOW: u16 = 2;
const ZERO_HIGH: u16 = 2;
const ZERO_LOW: u16 = 6;

/// 颜色 → GRB 位流（WS2812 出厂字节序为 G,R,B）
fn color_to_codes(color: RGB8, out: &mut [PulseCode; NUM_LEDS * 24 + 1]) {
    let bytes = [color.1, color.0, color.2]; // G, R, B
    let one = PulseCode::new(Level::High, ONE_HIGH, Level::Low, ONE_LOW);
    let zero = PulseCode::new(Level::High, ZERO_HIGH, Level::Low, ZERO_LOW);
    let mut i = 0;
    for &b in &bytes {
        for bit in (0..8).rev() {
            out[i] = if b & (1 << bit) != 0 { one } else { zero };
            i += 1;
        }
    }
    out[NUM_LEDS * 24] = PulseCode::end_marker();
}

/// 阻塞发送一帧：transmit 消费通道，wait 自旋续写直至完成并归还
fn send_frame<'ch>(
    ch: RmtChannel<'ch, esp_hal::Blocking, Tx>,
    codes: &[PulseCode],
) -> RmtChannel<'ch, esp_hal::Blocking, Tx> {
    let tx = match ch.transmit(codes) {
        Ok(tx) => tx,
        Err((e, ch)) => {
            info!("[ws2812] start err {:?}", e);
            return ch;
        }
    };
    match tx.wait() {
        Ok(ch) => ch,
        Err((e, ch)) => {
            info!("[ws2812] wait err {:?}", e);
            ch
        }
    }
}

#[embassy_executor::task]
pub async fn task_ws2812(rmt_peri: RMT<'static>, led_pin: Output<'static>) -> ! {
    let rmt = Rmt::new(rmt_peri, Rate::from_hz(8_000_000)).unwrap();

    let tx_cfg = TxChannelConfig::default()
        .with_clk_divider(1)
        .with_idle_output_level(Level::Low)
        .with_idle_output(true);

    let mut channel = rmt
        .channel0
        .configure_tx(&tx_cfg)
        .unwrap()
        .with_pin(led_pin);

    let mut codes = [PulseCode::end_marker(); NUM_LEDS * 24 + 1];

    // 启动即发一帧全灭，清除上电/复位残留
    color_to_codes(RGB8(0, 0, 0), &mut codes);
    channel = critical_section::with(|_| send_frame(channel, &codes));

    info!("[ws2812] ready");

    loop {
        let color = WS2812_CHANNEL.receive().await;
        color_to_codes(color, &mut codes);
        // 整帧关中断，根除批次间隙导致的灯珠提前锁存
        channel = critical_section::with(|_| send_frame(channel, &codes));
    }
}
