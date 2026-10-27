// src/app/task_ws2812.rs
//
// WS2812 RGB LED 控制任务（GPIO48，经 RMT 驱动）
use defmt::info;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use esp_hal::gpio::{Level, Output};
use esp_hal::peripherals::RMT;
use esp_hal::rmt::{PulseCode, Rmt, TxChannelConfig, TxChannelCreator};
use esp_hal::time::Rate;

/// 1 颗板载 WS2812（GPIO48）
const NUM_LEDS: usize = 1;

pub static WS2812_CHANNEL: Channel<CriticalSectionRawMutex, RGB8, 4> = Channel::new();

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RGB8(pub u8, pub u8, pub u8);

/// RMT 时钟 8MHz → 1 tick = 125ns
/// WS2812 "1": 875ns 高 / 410ns 低 ≈ 7 / 3 tick
/// WS2812 "0": 375ns 高 / 875ns 低 ≈ 3 / 7 tick
fn bit1() -> PulseCode {
    PulseCode::new(Level::High, 7, Level::Low, 3)
}

fn bit0() -> PulseCode {
    PulseCode::new(Level::High, 3, Level::Low, 7)
}

fn color_to_codes(color: RGB8, out: &mut [PulseCode; NUM_LEDS * 24 + 1]) {
    let bytes = [color.1, color.0, color.2]; // WS2812 顺序：G, R, B
    let mut i = 0;
    for &b in &bytes {
        for bit in (0..8).rev() {
            out[i] = if b & (1 << bit) != 0 { bit1() } else { bit0() };
            i += 1;
        }
    }
    out[NUM_LEDS * 24] = PulseCode::end_marker();
}

#[embassy_executor::task]
pub async fn task_ws2812(rmt_peri: RMT<'static>, led_pin: Output<'static>) -> ! {
    let rmt = Rmt::new(rmt_peri, Rate::from_mhz(8))
        .unwrap()
        .into_async();

    let tx_cfg = TxChannelConfig::default()
        .with_clk_divider(0)
        .with_idle_output_level(Level::Low)
        .with_idle_output(true);

    let mut channel = rmt
        .channel0
        .configure_tx(&tx_cfg)
        .unwrap()
        .with_pin(led_pin);

    let mut codes = [PulseCode::end_marker(); NUM_LEDS * 24 + 1];
    let mut current = RGB8(0, 0, 0);

    color_to_codes(current, &mut codes);
    if channel.transmit(&codes).await.is_err() {
        // 重试一次（首次上电时序未稳时可能失败）
        let _ = channel.transmit(&codes).await;
    }

    info!("[ws2812] ready");

    loop {
        let color = WS2812_CHANNEL.receive().await;
        if color != current {
            current = color;
            color_to_codes(color, &mut codes);
            match channel.transmit(&codes).await {
                Ok(_) => {}
                Err(e) => info!("[ws2812] tx err {:?}", e),
            }
        }
    }
}
