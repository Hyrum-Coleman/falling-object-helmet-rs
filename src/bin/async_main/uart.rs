use crate::{DetectionStatus, SensorData, ALERT_TIME_MILLISECONDS, VELOCITY_THRESHOLD, WATCH};
use core::ffi::CStr;

#[cfg(feature = "wifi")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(feature = "wifi")]
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use esp_hal::gpio::Output;
use esp_hal::uart::UartRx;
use esp_hal::Async;
use heapless::Vec;
use log::{error, info, warn};

/// Reads UART channel, then attemps to convert recieved bytes into an f32
/// TODO: UART is probably sending character bytes, which won't cleanly convert to an f32, attempt conversion to char array, then f32
#[cfg(not(feature = "wifi"))]
#[embassy_executor::task]
pub async fn read_uart(mut uart: UartRx<'static, Async>, mut builtin_led: Output<'static>) {
    let mut uart_buffer: [u8; 128] = [0u8; 128];
    let sender = WATCH.dyn_sender();

    loop {
        let Ok(_len) = uart.read_async(&mut uart_buffer).await else {
            error!("Error reading UART");
            continue;
        };

        let sensor_ascii = match CStr::from_bytes_until_nul(&uart_buffer) {
            Ok(valid_ascii) => valid_ascii,
            Err(e) => {
                warn!("UART Buffer: {:?}", uart_buffer);
                error!("{}", e);
                continue;
            }
        }
        .to_str()
        .unwrap();

        let variables: Vec<f32, 2> = sensor_ascii
            .trim()
            .split(',')
            .flat_map(|var| var.parse::<f32>())
            .collect();

        if variables.len() != 2 {
            error!("SHit's fucked: {:?}", variables);
            builtin_led.set_high();
            continue;
        } else {
            builtin_led.set_low();
        }

        uart_buffer = [0u8; 128];

        let sensor_data = SensorData::new(variables[1], variables[0]);

        info!(
            "Time: {} -- Velocity Reading: {}",
            sensor_data.time, sensor_data.velocity
        );

        match sensor_data.velocity >= VELOCITY_THRESHOLD {
            true => {
                sender.send(DetectionStatus::ObjectDetected);
                Timer::after_millis(ALERT_TIME_MILLISECONDS.into()).await;
            }
            false => {
                sender.send(DetectionStatus::Clear);
            }
        };
    }
}

// wifi version of read_uart
#[cfg(feature = "wifi")]
#[embassy_executor::task]
pub async fn read_uart(
    mut uart: UartRx<'static, Async>,
    signals: &'static Signal<CriticalSectionRawMutex, SensorData>,
    mut builtin_led: Output<'static>,
) {
    let mut uart_buffer: [u8; 128] = [0u8; 128];
    let sender = WATCH.dyn_sender();

    loop {
        let Ok(_len) = uart.read_async(&mut uart_buffer).await else {
            error!("Error reading UART");
            continue;
        };

        let sensor_ascii = match CStr::from_bytes_until_nul(&uart_buffer) {
            Ok(valid_ascii) => valid_ascii,
            Err(e) => {
                warn!("UART Buffer: {:?}", uart_buffer);
                error!("{}", e);
                continue;
            }
        }
        .to_str()
        .unwrap();

        let variables: Vec<f32, 2> = sensor_ascii
            .trim()
            .split(',')
            .flat_map(|var| var.parse::<f32>())
            .collect();

        if variables.len() != 2 {
            error!("SHit's fucked: {:?}", variables);
            builtin_led.set_high();
            continue;
        } else {
            builtin_led.set_low();
        }

        uart_buffer = [0u8; 128];

        let sensor_data = SensorData::new(variables[1], variables[0]);

        info!(
            "Time: {} -- Velocity Reading: {}",
            sensor_data.time, sensor_data.velocity
        );

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
