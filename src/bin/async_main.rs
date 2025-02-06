#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
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
use log::{error, info};
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

#[derive(Debug, Clone, Copy)]
enum DetectionStatus {
    ObjectDetected,
    Clear,
}

const NUM_LEDS: usize = 30;

const _RED: RGB8 = RGB8::new(40, 0, 0);
const _YELLOW: RGB8 = RGB8::new(40, 40, 0);
const _GREEN: RGB8 = RGB8::new(0, 40, 0);
const CYAN: RGB8 = RGB8::new(0, 40, 40);
const _BLUE: RGB8 = RGB8::new(0, 0, 40);
const _PURPLE: RGB8 = RGB8::new(40, 0, 40);

static WATCH: Watch<CriticalSectionRawMutex, DetectionStatus, 2> = Watch::new();

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

    let haptic = Output::new(peripherals.GPIO18, Level::Low);

    // TODO: Spawn some tasks
    let _ = spawner.spawn(haptic_task(haptic));
    // let _ = spawner.spawn(read_mpu_data(i2c));
    let _ = spawner.spawn(read_uart(tx));
    let _ = spawner.spawn(led_strip_alert_task(ws));

    loop {
        info!("{} ms | Hello world!", Instant::now().as_millis());
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}

/// Toggles haptic motor
#[embassy_executor::task]
async fn haptic_task(mut haptic: Output<'static>) {
    let mut receiver = WATCH.dyn_receiver().unwrap();
    loop {
        info!("waiting for watch on haptic");
        let val = receiver.changed().await;

        match val {
            DetectionStatus::ObjectDetected => haptic.set_high(),
            DetectionStatus::Clear => haptic.set_low(),
        }
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
async fn read_uart(mut _uart: UartRx<'static, Async>) {
    let mut _uart_buffer: [u8; 4] = [0u8; 4];
    let sender = WATCH.dyn_sender();

    loop {
        Timer::after_secs(10).await;

        info!("Set signal true");
        sender.send(DetectionStatus::ObjectDetected);
        Timer::after_secs(5).await;
        sender.send(DetectionStatus::Clear);
        info!("Set signal false");
        // let Ok(_len) = uart.read_async(&mut uart_buffer).await else {
        //     error!("Error reading UART");
        //     continue;
        // };

        // warn!("UART Buffer: {:?}", uart_buffer);

        // let velocity_reading = f32::from_be_bytes(uart_buffer);

        // info!("Velocity Reading: {:?}", velocity_reading);
    }
}

/// This task is a disaster. Literally copy pasted from an example
/// Can be updated to better suit our needs
#[embassy_executor::task]
async fn led_strip_alert_task(mut ws: Ws2812<Spi<'static, Async>>) {
    let color_leds = [CYAN; NUM_LEDS];
    let clear_led = [RGB8::default(); NUM_LEDS];
    let mut receiver = WATCH.receiver().unwrap();

    loop {
        let val = receiver.changed().await;

        match val {
            DetectionStatus::ObjectDetected => {
                loop {
                    ws.write(color_leds).unwrap();
                    match select(Timer::after_millis(100), receiver.changed()).await {
                        embassy_futures::select::Either::First(_) => (),
                        embassy_futures::select::Either::Second(_) => {
                            ws.write(clear_led).unwrap();
                            break;
                        },
                    };
                    ws.write(clear_led).unwrap();
                    match select(Timer::after_millis(100), receiver.changed()).await {
                        embassy_futures::select::Either::First(_) => (),
                        embassy_futures::select::Either::Second(_) => {
                            ws.write(clear_led).unwrap();
                            break;
                        },
                    };
                }
            }
            DetectionStatus::Clear => {
                info!("Turning off LEDS");

                ws.write(clear_led).unwrap();
            }
        }
    }
}
