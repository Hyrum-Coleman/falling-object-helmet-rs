use crate::{DetectionStatus, ALERT_TIME_MILLISECONDS, WATCH};
use embassy_futures::select::select;
use embassy_time::Timer;
use esp_hal::gpio::Output;
use esp_hal::spi::master::Spi;
use esp_hal::Async;
use smart_leds::{SmartLedsWrite, RGB8};
use ws2812_spi::Ws2812;

pub const NUM_LEDS: usize = 30;
const RED: RGB8 = RGB8::new(40, 0, 0);
const _YELLOW: RGB8 = RGB8::new(40, 40, 0);
const _GREEN: RGB8 = RGB8::new(0, 40, 0);
const _CYAN: RGB8 = RGB8::new(0, 40, 40);
const _BLUE: RGB8 = RGB8::new(0, 0, 40);
const _PURPLE: RGB8 = RGB8::new(40, 0, 40);

/// This task is a disaster. Literally copy pasted from an example
/// Can be updated to better suit our needs
#[embassy_executor::task]
pub async fn led_strip_alert_task(mut ws: Ws2812<Spi<'static, Async>>) {
    let color_leds = [RED; NUM_LEDS];
    let clear_led = [RGB8::default(); NUM_LEDS];
    let mut receiver = WATCH.receiver().unwrap();

    loop {
        match receiver.changed().await {
            DetectionStatus::ObjectDetected => {
                select(Timer::after_millis(ALERT_TIME_MILLISECONDS.into()), async {
                    for _i in 1..=8 {
                        ws.write(color_leds).unwrap();
                        Timer::after_millis(100).await;
                        ws.write(clear_led).unwrap();
                        Timer::after_millis(100).await;
                    }
                }).await;
            }
        }
    }
}

/// Toggles haptic motor
#[embassy_executor::task]
pub async fn haptic_task(mut haptic: Output<'static>) {
    let mut receiver = WATCH.dyn_receiver().unwrap();
    loop {
        let val = receiver.changed().await;

        match val {
            DetectionStatus::ObjectDetected => {
                haptic.set_high();
                Timer::after_millis(ALERT_TIME_MILLISECONDS.into()).await;
                haptic.set_low();
            }
            }
    }
}
