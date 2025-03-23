#![no_std]
#![no_main]

mod alert;
mod imu;
mod uart;
#[cfg(feature = "wifi")]
mod wifi;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_backtrace as _;
use esp_hal::i2c::master::{Config as i2cConfig, I2c};
use esp_hal::spi::master::{Config as spiConfig, Spi};
use esp_hal::spi::Mode;
use esp_hal::uart::{Config as UartConfig, Uart};
use esp_hal::{
    clock::CpuClock,
    gpio::{Level, Output},
};
use fugit::HertzU32;
use log::{error, info};
use smart_leds::{SmartLedsWrite, RGB8};
use ws2812_spi::Ws2812;

use crate::alert::haptic_task;
use crate::uart::read_uart;
use alert::led_strip_alert_task;
use alert::NUM_LEDS;
#[cfg(not(feature = "wifi"))]
use embassy_time::Instant;
use esp_hal::peripherals::Peripherals;
use falling_object_helmet_rs::DetectionStatus;
#[cfg(feature = "wifi")]
use {
    embassy_futures::select::Either, embassy_net::IpListenEndpoint, embassy_sync::signal::Signal,
    falling_object_helmet_rs::SensorData,
};

static WATCH: Watch<CriticalSectionRawMutex, DetectionStatus, 2> = Watch::new();

#[cfg(feature = "wifi")]
static SENSOR_SIGNALS: Signal<CriticalSectionRawMutex, SensorData> = Signal::new();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.2.2

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals: Peripherals = esp_hal::init(config);

    esp_println::logger::init_logger_from_env();

    let timer_group0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer_group0.timer0);

    info!("Embassy initialized!");

    // wifi stuff

    #[cfg(feature = "wifi")]
    let mut ap_server_rx_buffer = [0; 1536];
    #[cfg(feature = "wifi")]
    let mut ap_server_tx_buffer = [0; 1536];
    #[cfg(feature = "wifi")]
    let mut sta_server_rx_buffer = [0; 1536];
    #[cfg(feature = "wifi")]
    let mut sta_server_tx_buffer = [0; 1536];

    #[cfg(feature = "wifi")]
    let (mut ap_server_socket, mut sta_server_socket) = wifi::wifi_init(
        spawner,
        peripherals.TIMG0,
        peripherals.RNG,
        peripherals.RADIO_CLK,
        peripherals.WIFI,
        &mut ap_server_rx_buffer,
        &mut ap_server_tx_buffer,
        &mut sta_server_rx_buffer,
        &mut sta_server_tx_buffer,
    )
    .await;

    // end of wifi stuff

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

    let uart_config = UartConfig::default()
        .with_baudrate(19200)
        .with_stop_bits(esp_hal::uart::StopBits::_1);

    let (tx, mut _rx) = match Uart::new(peripherals.UART2, uart_config) {
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
    let builtin_led = Output::new(peripherals.GPIO2, Level::Low);

    // TODO: Spawn some tasks
    #[cfg(feature = "wifi")]
    let _ = spawner.spawn(read_uart(tx, &SENSOR_SIGNALS, builtin_led));
    #[cfg(not(feature = "wifi"))]
    let _ = spawner.spawn(read_uart(tx, builtin_led));
    let _ = spawner.spawn(haptic_task(haptic));
    // let _ = spawner.spawn(read_mpu_data(i2c));
    let _ = spawner.spawn(led_strip_alert_task(ws));

    #[cfg(not(feature = "wifi"))]
    loop {
        info!("{} ms | Hello world!", Instant::now().as_millis());
        Timer::after_millis(1000).await;
    }

    #[cfg(feature = "wifi")]
    loop {
        // wait for someone to connect
        let either_socket = embassy_futures::select::select(
            ap_server_socket.accept(IpListenEndpoint {
                addr: None,
                port: 8080,
            }),
            sta_server_socket.accept(IpListenEndpoint {
                addr: None,
                port: 8080,
            }),
        )
        .await;

        let (r, server_socket) = match either_socket {
            Either::First(r) => (r, &mut ap_server_socket),
            Either::Second(r) => (r, &mut sta_server_socket),
        };

        if let Err(e) = r {
            error!("connect error: {:?}", e);
            continue;
        }

        info!("Client Connected...");

        // send sensor data to client
        loop {
            let sensor_data = SENSOR_SIGNALS.wait().await;

            //info!("{},{}", sensor_data.time, sensor_data.velocity);

            if !server_socket.can_send() {
                // client disconected
                info!("Client Disconnected...");
                break;
            }

            let Ok(_) = server_socket.write(&[0xA1]).await else {
                info!("Client Disconnected...");
                break;
            };

            let Ok(_) = server_socket
                .write(&sensor_data.velocity.to_le_bytes())
                .await
            else {
                info!("Client Disconnected...");
                break;
            };

            let Ok(_) = server_socket.write(&sensor_data.time.to_le_bytes()).await else {
                info!("Client Disconnected...");
                break;
            };

            //info!("Value Received: {}", vel);
        }
    }
}
