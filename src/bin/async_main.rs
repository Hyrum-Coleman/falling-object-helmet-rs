#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::i2c::master::{Config as i2cConfig, I2c};
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output},
    Async,
};
use log::{error, info};

#[allow(dead_code)]
mod mpu9250 {
    pub const ADDR: u8 = 0x69; // default addr

    macro_rules! mpuregs {
        ($($name:ident : $val:expr),* $(,)?) => {
            $(
                pub const $name: u8 = $val;
            )*

            pub fn regname(reg: u8) -> &'static str {
                match reg {
                    $(
                        $val => stringify!($name),
                    )*
                    _ => panic!("bad reg"),
                }
            }
        }
    }

    // These are correct for IOCON.BANK=0
    mpuregs! {
        ACCEL_XOUT_H: 0x3B,
        ACCEL_XOUT_L: 0x3C,
        ACCEL_YOUT_H: 0x3D,
        ACCEL_YOUT_L: 0x3E,
        ACCEL_ZOUT_H: 0x3F,
        ACCEL_ZOUT_L: 0x40,
        TEMP_OUT_H: 0x041,
        TEMP_OUT_L: 0x42,
        GYRO_XOUT_H: 0x43,
        GYRO_XOUT_L: 0x44,
        GYRO_YOUT_H: 0x45,
        GYRO_YOUT_L: 0x46,
        GYRO_ZOUT_H: 0x47,
        GYRO_ZOUT_L: 0x48,
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.2.2

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::logger::init_logger_from_env();

    let timer0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    info!("Embassy initialized!");
    let _sda = peripherals.GPIO21;
    let _scl = peripherals.GPIO22;

    let i2c = match I2c::new(peripherals.I2C1, i2cConfig::default()) {
        Ok(i2c) => i2c.into_async(),
        Err(err) => {
            error!("Error setting up i2c: {err}");
            Timer::after(Duration::from_secs(2)).await;
            return;
        }
    };

    let led = Output::new(peripherals.GPIO2, Level::Low);

    // TODO: Spawn some tasks
    let _ = spawner.spawn(led_task(led));
    let _ = spawner.spawn(read_mpu_data(i2c));

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
        info!("Setting LED to {:?}", !led_level);
        led.set_level(!led_level);
        Timer::after(Duration::from_millis(469)).await;
    }
}

#[embassy_executor::task]
async fn read_mpu_data(mut i2c: I2c<'static, Async>) {
    use mpu9250::*;
    let buffer: &mut [u8] = &mut [ACCEL_XOUT_L];
    loop {
        match i2c.read(ADDR, buffer).await {
            Ok(_) => (),
            Err(err) => {
                error!("Something went wrong reading the i2c device: {err}");
                Timer::after(Duration::from_secs(2)).await;
                continue;
            }
        };

        for item in &mut *buffer {
            info!("Reading: {item}");
        }
    }
}
