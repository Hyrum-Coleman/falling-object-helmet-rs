#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output},
};
use log::{info, warn};

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.2.2

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::logger::init_logger_from_env();

    let timer0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    info!("Embassy initialized!");

    let led = Output::new(peripherals.GPIO2, Level::Low);

    // TODO: Spawn some tasks
    let _ = spawner.spawn(led_task(led));

    loop {
        info!("{} ms | Hello world!", Instant::now().as_millis());
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}

#[embassy_executor::task]
async fn led_task(mut led: Output<'static>) {
    loop {
        let led_level = led.output_level();
        warn!("Setting LED to {:?}", !led_level);
        led.set_level(!led_level);
        Timer::after(Duration::from_millis(469)).await;
    }
}
