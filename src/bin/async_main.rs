#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::i2c::master::{Config as i2cConfig, I2c};
use esp_hal::ledc::channel::{Channel, ChannelIFace};
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{channel, timer, LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output},
    Async,
};
use fugit::Rate;
use log::{error, info};

#[allow(dead_code)]
mod mpu9250 {
    pub const ADDR: u8 = 0x68; // default addr

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

    let sda = peripherals.GPIO21;
    let scl = peripherals.GPIO22;

    let i2c = match I2c::new(peripherals.I2C1, i2cConfig::default()) {
        Ok(i2c) => i2c.with_sda(sda).with_scl(scl).into_async(),
        Err(err) => {
            error!("Error setting up i2c: {err}");
            Timer::after(Duration::from_secs(2)).await;
            return;
        }
    };

    let uart = match Uart::new(peripherals.UART1, UartConfig::default()) {
        Ok(uart) => uart.into_async(),
        Err(err) => {
            error!("Error setting up UART1: {err}");
            Timer::after(Duration::from_secs(2)).await;
            return;
        }
    };

    let builtin_led = Output::new(peripherals.GPIO2, Level::Low);

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::<u32, 1, 1>::kHz(24),
        })
        .unwrap();

    let mut channel0 = ledc.channel(channel::Number::Channel0, peripherals.GPIO4);
    channel0
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 10,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    // TODO: Spawn some tasks
    let _ = spawner.spawn(led_task(builtin_led));
    let _ = spawner.spawn(read_mpu_data(i2c));
    let _ = spawner.spawn(read_uart(uart));

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
    let buffer: &mut [u8; 2] = &mut [TEMP_OUT_H, TEMP_OUT_L];
    loop {
        match i2c.read(ADDR, buffer).await {
            Ok(_) => (),
            Err(err) => {
                error!("Something went wrong reading the i2c device: {err}");
                Timer::after(Duration::from_secs(2)).await;
                continue;
            }
        };

        let temp = u16::from_be_bytes(*buffer);
        info!("Temperature reading: {temp} C");
    }
}

#[embassy_executor::task]
async fn led_strip(led_channel: Channel<'static, LowSpeed>) {
    led_channel.start_duty_fade(0, 100, 1000).unwrap();
    while led_channel.is_duty_fade_running() {}
    led_channel.start_duty_fade(100, 0, 1000).unwrap();
    while led_channel.is_duty_fade_running() {}
}

#[embassy_executor::task]
async fn read_uart(mut uart: Uart<'static, Async>) {
    let uart_buffer: &mut [u8; 1] = &mut [0];

    loop {
        match uart.read_async(uart_buffer).await {
            Ok(_) => {}
            Err(err) => {
                error!("Error reading UART: {err}");
                continue;
            }
        };
        let velocity_reading = i8::from_be_bytes(*uart_buffer);

        info!("Reading UART Channel: {velocity_reading}");
    }
}
