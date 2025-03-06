#![no_std]
#![no_main]

use core::ffi::CStr;

use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Timer};
use esp_alloc as _;
use esp_backtrace as _;
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
use esp_println;
use fugit::HertzU32;
use log::{error, info};
use smart_leds::{SmartLedsWrite, RGB8};
use ws2812_spi::Ws2812;

#[cfg(feature = "wifi")]
use {
    core::net::Ipv4Addr,
    embassy_futures::select::Either,
    embassy_net::{
        tcp::TcpSocket, IpListenEndpoint, Ipv4Cidr, Runner, StackResources, StaticConfigV4,
    },
    embassy_sync::signal::Signal,
    esp_hal::rng::Rng,
    esp_wifi::{
        init,
        wifi::{
            AccessPointConfiguration, ClientConfiguration, Configuration, WifiController,
            WifiDevice, WifiEvent, WifiState,
        },
        EspWifiController,
    },
};

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

#[derive(Clone)]
struct SensorData {
    velocity: f32,
    time: f32,
}

impl SensorData {
    fn new(velocity: f32, time: f32) -> Self {
        Self { velocity, time }
    }
}

const ALERT_TIME_MILLISECONDS: u16 = 1000;
const VELOCITY_THRESHOLD: f32 = 3.0;

const NUM_LEDS: usize = 30;

const RED: RGB8 = RGB8::new(40, 0, 0);
const _YELLOW: RGB8 = RGB8::new(40, 40, 0);
const _GREEN: RGB8 = RGB8::new(0, 40, 0);
const _CYAN: RGB8 = RGB8::new(0, 40, 40);
const _BLUE: RGB8 = RGB8::new(0, 0, 40);
const _PURPLE: RGB8 = RGB8::new(40, 0, 40);

static WATCH: Watch<CriticalSectionRawMutex, DetectionStatus, 2> = Watch::new();

#[cfg(feature = "wifi")]
static SENSOR_SIGNALS: Signal<CriticalSectionRawMutex, SensorData> = Signal::new();

#[cfg(feature = "wifi")]
const SSID: &str = env!("SSID");
#[cfg(feature = "wifi")]
const PASSWORD: &str = env!("PASSWORD");

#[cfg(feature = "wifi")]
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.2.2

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

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
    let (mut ap_server_socket, mut sta_server_socket) = {
        esp_alloc::heap_allocator!(72 * 1024);

        let timer_group1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
        let mut rng = Rng::new(peripherals.RNG);

        let esp_wifi_ctrl = &*mk_static!(
            EspWifiController<'static>,
            init(timer_group1.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap()
        );

        let (wifi_ap_device, wifi_sta_device, mut controller) =
            esp_wifi::wifi::new_ap_sta(&esp_wifi_ctrl, peripherals.WIFI).unwrap();

        let ap_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
            address: Ipv4Cidr::new(Ipv4Addr::new(192, 168, 2, 1), 24),
            gateway: Some(Ipv4Addr::new(192, 168, 2, 1)),
            dns_servers: Default::default(),
        });

        let sta_config = embassy_net::Config::dhcpv4(Default::default());

        let seed = (rng.random() as u64) << 32 | rng.random() as u64;

        // Init network stacks
        let (ap_stack, ap_runner) = embassy_net::new(
            wifi_ap_device,
            ap_config,
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            seed,
        );
        let (sta_stack, sta_runner) = embassy_net::new(
            wifi_sta_device,
            sta_config,
            mk_static!(StackResources<4>, StackResources::<4>::new()),
            seed,
        );

        let client_config = Configuration::Mixed(
            ClientConfiguration {
                ssid: SSID.try_into().unwrap(),
                password: PASSWORD.try_into().unwrap(),
                ..Default::default()
            },
            AccessPointConfiguration {
                ssid: "esp-wifi".try_into().unwrap(),
                ..Default::default()
            },
        );
        controller.set_configuration(&client_config).unwrap();

        spawner.spawn(connection(controller)).ok();
        spawner.spawn(net_task_ap(ap_runner)).ok();
        spawner.spawn(net_task_sta(sta_runner)).ok();

        let sta_address = loop {
            if let Some(config) = sta_stack.config_v4() {
                let address = config.address.address();
                info!("Got IP: {}", address);
                break address;
            }
            info!("Waiting for IP...");
            Timer::after(Duration::from_millis(500)).await;
        };
        loop {
            if ap_stack.is_link_up() {
                break;
            }
            Timer::after(Duration::from_millis(500)).await;
        }

        info!("Connect to the ap '{SSID}' and connect to {sta_address} port 8080");

        let mut ap_server_socket =
            TcpSocket::new(ap_stack, &mut ap_server_rx_buffer, &mut ap_server_tx_buffer);
        ap_server_socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

        let mut sta_server_socket = TcpSocket::new(
            sta_stack,
            &mut sta_server_rx_buffer,
            &mut sta_server_tx_buffer,
        );
        sta_server_socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));
        (ap_server_socket, sta_server_socket)
    };

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

    // let Ok(_len) = rx.write_async(&[0x4F, 0x54]).await else {
    //     error!("Error writing to UART");
    //     Timer::after(Duration::from_secs(2)).await;
    //     return;
    // }; // "OT", command to turn on time reading

    // let Ok(_len) = rx.write_async(&[0x49, 0x73]).await else {
    //     error!("Error writing to UART");
    //     Timer::after(Duration::from_secs(2)).await;
    //     return;
    // }; // command to turn on uart

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
    #[cfg(feature = "wifi")]
    let _ = spawner.spawn(read_uart(tx, &SENSOR_SIGNALS));
    #[cfg(not(feature = "wifi"))]
    let _ = spawner.spawn(read_uart(tx));
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

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.23.1/examples/src/bin
}

/// Toggles haptic motor
#[embassy_executor::task]
async fn haptic_task(mut haptic: Output<'static>) {
    let mut receiver = WATCH.dyn_receiver().unwrap();
    loop {
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
#[cfg(not(feature = "wifi"))]
#[embassy_executor::task]
async fn read_uart(mut uart: UartRx<'static, Async>) {
    let mut uart_buffer: [u8; 128] = [0u8; 128];
    let sender = WATCH.dyn_sender();

    loop {
        let Ok(_len) = uart.read_async(&mut uart_buffer).await else {
            error!("Error reading UART");
            continue;
        };

        // warn!("UART Buffer: {:?}", uart_buffer);

        let sensor_ascii = CStr::from_bytes_until_nul(&uart_buffer)
            .unwrap()
            .to_str()
            .unwrap();

        info!("{}", sensor_ascii);

        let mut variables: [f32; 2] = [0.0; 2];
        for (i, var) in sensor_ascii.trim().split(',').enumerate() {
            //info!("{}", var);
            variables[i] = match var.trim().parse::<f32>() {
                Ok(reading) => reading,
                Err(_err) => {
                    error!("Parse float error: {_err:?}:{var}");
                    continue;
                }
            };
        }

        let sensor_data = SensorData::new(variables[1], variables[0]);

        match sensor_data.velocity.total_cmp(&VELOCITY_THRESHOLD) {
            core::cmp::Ordering::Less => {
                sender.send(DetectionStatus::Clear);
            }
            core::cmp::Ordering::Equal => {
                sender.send(DetectionStatus::ObjectDetected);
                Timer::after_millis(ALERT_TIME_MILLISECONDS.into()).await;
            }
            core::cmp::Ordering::Greater => {
                sender.send(DetectionStatus::ObjectDetected);
                Timer::after_millis(ALERT_TIME_MILLISECONDS.into()).await;
            }
        };

        // info!("Velocity Reading: {:?}", velocity_reading);
    }
}

// wifi version of read_uart
#[cfg(feature = "wifi")]
#[embassy_executor::task]
async fn read_uart(
    mut uart: UartRx<'static, Async>,
    signals: &'static Signal<CriticalSectionRawMutex, SensorData>,
) {
    let mut uart_buffer: [u8; 128] = [0u8; 128];
    let sender = WATCH.dyn_sender();

    loop {
        let Ok(_len) = uart.read_async(&mut uart_buffer).await else {
            error!("Error reading UART");
            continue;
        };

        // warn!("UART Buffer: {:?}", uart_buffer);

        let sensor_ascii = CStr::from_bytes_until_nul(&uart_buffer)
            .unwrap()
            .to_str()
            .unwrap();

        info!("{}", sensor_ascii);

        let mut variables: [f32; 2] = [0.0; 2];
        for (i, var) in sensor_ascii.trim().split(',').enumerate() {
            //info!("{}", var);
            variables[i] = match var.trim().parse::<f32>() {
                Ok(reading) => reading,
                Err(_err) => {
                    error!("Parse float error: {_err:?}:{var}");
                    continue;
                }
            };
        }

        let sensor_data = SensorData::new(variables[1], variables[0]);

        signals.signal(sensor_data.clone());

        match sensor_data.velocity.total_cmp(&VELOCITY_THRESHOLD) {
            core::cmp::Ordering::Less => {
                sender.send(DetectionStatus::Clear);
            }
            core::cmp::Ordering::Equal => {
                sender.send(DetectionStatus::ObjectDetected);
                Timer::after_millis(ALERT_TIME_MILLISECONDS.into()).await;
            }
            core::cmp::Ordering::Greater => {
                sender.send(DetectionStatus::ObjectDetected);
                Timer::after_millis(ALERT_TIME_MILLISECONDS.into()).await;
            }
        };

        // info!("Velocity Reading: {:?}", velocity_reading);
    }
}

/// This task is a disaster. Literally copy pasted from an example
/// Can be updated to better suit our needs
#[embassy_executor::task]
async fn led_strip_alert_task(mut ws: Ws2812<Spi<'static, Async>>) {
    let color_leds = [RED; NUM_LEDS];
    let clear_led = [RGB8::default(); NUM_LEDS];
    let mut receiver = WATCH.receiver().unwrap();

    loop {
        let val = receiver.changed().await;

        match val {
            DetectionStatus::ObjectDetected => loop {
                ws.write(color_leds).unwrap();
                match select(Timer::after_millis(100), receiver.changed()).await {
                    embassy_futures::select::Either::First(_) => (),
                    embassy_futures::select::Either::Second(_) => {
                        ws.write(clear_led).unwrap();
                        break;
                    }
                };
                ws.write(clear_led).unwrap();
                match select(Timer::after_millis(100), receiver.changed()).await {
                    embassy_futures::select::Either::First(_) => (),
                    embassy_futures::select::Either::Second(_) => {
                        ws.write(clear_led).unwrap();
                        break;
                    }
                };
            },
            DetectionStatus::Clear => {
                ws.write(clear_led).unwrap();
            }
        }
    }
}

// from example
#[cfg(feature = "wifi")]
#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("start connection task");
    info!("Device capabilities: {:?}", controller.capabilities());

    info!("Starting wifi");
    controller.start_async().await.unwrap();
    info!("Wifi started!");

    loop {
        match esp_wifi::wifi::ap_state() {
            WifiState::ApStarted => {
                info!("About to connect...");

                match controller.connect_async().await {
                    Ok(_) => {
                        // wait until we're no longer connected
                        controller.wait_for_event(WifiEvent::StaDisconnected).await;
                        info!("STA disconnected");
                    }
                    Err(e) => {
                        error!("Failed to connect to wifi: {e:?}");
                        Timer::after(Duration::from_millis(5000)).await
                    }
                }
            }
            _ => return,
        }
    }
}

// from example
#[cfg(feature = "wifi")]
#[embassy_executor::task(pool_size = 2)]
async fn net_task_ap(
    mut runner: Runner<'static, WifiDevice<'static, esp_wifi::wifi::WifiApDevice>>,
) {
    runner.run().await
}

#[cfg(feature = "wifi")]
#[embassy_executor::task(pool_size = 2)]
async fn net_task_sta(
    mut runner: Runner<'static, WifiDevice<'static, esp_wifi::wifi::WifiStaDevice>>,
) {
    runner.run().await
}
