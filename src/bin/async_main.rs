#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::i2c::master::{Config as i2cConfig, I2c};
use esp_hal::spi::master::{Config as spiConfig, Spi};
use esp_hal::spi::Mode;
use esp_hal::uart::{Config as UartConfig, Uart, UartRx};
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output},
    Async,
};
use fugit::HertzU32;
use log::{error, info, warn};
use smart_leds::{SmartLedsWrite, RGB8};
use ws2812_spi::Ws2812;

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

const NUM_LEDS: usize = 30;

const RED: RGB8 = RGB8::new(40, 0, 0);
const YELLOW: RGB8 = RGB8::new(40, 40, 0);
const GREEN: RGB8 = RGB8::new(0, 40, 0);
const CYAN: RGB8 = RGB8::new(0, 40, 40);
const BLUE: RGB8 = RGB8::new(0, 0, 40);
const PURPLE: RGB8 = RGB8::new(40, 0, 40);

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.2.2

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_println::logger::init_logger_from_env();

    let timer_group0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer_group0.timer0);

    info!("Embassy initialized!");

    let sda = peripherals.GPIO21;
    let scl = peripherals.GPIO22;

    let _i2c = match I2c::new(peripherals.I2C1, i2cConfig::default()) {
        Ok(i2c) => i2c.with_sda(sda).with_scl(scl).into_async(),
        Err(err) => {
            error!("Error setting up i2c: {err}");
            Timer::after(Duration::from_secs(2)).await;
            return;
        }
    };

    let uart_config = UartConfig::default().with_baudrate(19200);

    let (tx, mut rx) = match Uart::new(peripherals.UART2, uart_config) {
        Ok(uart) => uart
            .with_rx(peripherals.GPIO16)
            .with_tx(peripherals.GPIO17)
            .into_async()
            .split(),
        Err(err) => {
            error!("Error setting up UART1: {err}");
            Timer::after(Duration::from_secs(2)).await;
            return;
        }
    };

    let Ok(_len) = rx.write_async(&[0x49, 0x73]).await else {
        error!("Error writing to UART");
        Timer::after(Duration::from_secs(2)).await;
        return;
    };

    let led_pin4 = Output::new(peripherals.GPIO4, Level::Low);
    let spi = match Spi::new(
        peripherals.SPI2,
        spiConfig::default()
            .with_frequency(HertzU32::MHz(3))
            .with_mode(Mode::_0),
    ) {
        Ok(spi) => spi.with_mosi(led_pin4).into_async(),
        Err(err) => {
            error!("Error setting up SPI: {:?}", err);
            Timer::after(Duration::from_secs(2)).await;
            return;
        }
    };

    let mut ws = Ws2812::new(spi);
    let leds = [RGB8::default(); NUM_LEDS];
    ws.write(leds).unwrap();

    let haptic = Output::new(peripherals.GPIO5, Level::Low);

    // TODO: Spawn some tasks
    let _ = spawner.spawn(haptic_task(haptic));
    // let _ = spawner.spawn(read_mpu_data(i2c));
    let _ = spawner.spawn(read_uart(tx));
    let _ = spawner.spawn(led_strip_rainbow(ws));

    loop {
        info!("{} ms | Hello world!", Instant::now().as_millis());
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}

/// Toggles haptic motor
#[embassy_executor::task]
async fn haptic_task(mut haptic: Output<'static>) {
    loop {
        let haptic_level = haptic.output_level();
        info!("Setting LED to {:?}", !haptic_level);
        haptic.set_level(!haptic_level);
        Timer::after(Duration::from_millis(469)).await;
    }
}

/// Reads I2C bus, and reports readings
/// TODO: Figure out wtf is going on with I2C
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

/// Reads UART channel, then attemps to convert recieved bytes into an f32
/// TODO: UART is probably sending character bytes, which won't cleanly convert to an f32, attempt conversion to char array, then f32
#[embassy_executor::task]
async fn read_uart(mut uart: UartRx<'static, Async>) {
    let mut uart_buffer: [u8; 4] = [0u8; 4];

    loop {
        let Ok(_len) = uart.read_async(&mut uart_buffer).await else {
            error!("Error reading UART");
            continue;
        };

        warn!("UART Buffer: {:?}", uart_buffer);

        let velocity_reading = f32::from_be_bytes(uart_buffer);

        info!("Velocity Reading: {:?}", velocity_reading);
    }
}

/// This task is a disaster. Literally copy pasted from an example
/// Can be updated to better suit our needs
#[embassy_executor::task]
async fn led_strip_rainbow(mut ws: Ws2812<Spi<'static, Async>>) {
    let mut leds = [RGB8::default(); NUM_LEDS];
    let colors = [RED, GREEN, BLUE];
    let colors2 = [PURPLE, YELLOW, CYAN];

    let mut led_idx1 = 0;
    let mut led_idx2 = NUM_LEDS / 2;
    let mut color_iter1 = colors.into_iter().cycle();
    let mut color_iter2 = colors2.into_iter().cycle();
    let mut current_color1 = color_iter1.next().unwrap();
    let mut current_color2 = color_iter2.next().unwrap();

    loop {
        info!("writing index {led_idx1} and {led_idx2}");

        leds[led_idx1] = current_color1;
        leds[led_idx2] = current_color2;

        ws.write(leds).unwrap();

        Timer::after(Duration::from_micros(10)).await;
        led_idx1 = if led_idx1 >= NUM_LEDS - 1 {
            current_color1 = color_iter1.next().unwrap();
            0
        } else {
            led_idx1 + 1
        };
        led_idx2 = if led_idx2 >= NUM_LEDS - 1 {
            current_color2 = color_iter2.next().unwrap();
            0
        } else {
            led_idx2 + 1
        };
    }
}
