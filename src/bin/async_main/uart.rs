use crate::WATCH;
use core::ffi::CStr;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch::DynSender;
use esp_hal::gpio::Output;
use esp_hal::uart::UartRx;
use esp_hal::Async;
use falling_object_helmet_rs::{DetectionStatus, SensorData};
use heapless::Vec;
use log::{error, info, warn};

const VELOCITY_THRESHOLD: f32 = 1.0;

/// Handles shared UART logic for parsing SensorData and triggering alerts.
async fn process_uart_read(
    uart: &mut UartRx<'static, Async>,
    builtin_led: &mut Output<'static>,
) -> Option<SensorData> {
    let mut uart_buffer: [u8; 128] = [0u8; 128];

    let Ok(_len) = uart.read_async(&mut uart_buffer).await else {
        error!("Error reading UART");
        return None;
    };

    let sensor_ascii = match CStr::from_bytes_until_nul(&uart_buffer) {
        Ok(valid_ascii) => valid_ascii,
        Err(e) => {
            warn!("UART Buffer: {:?}", uart_buffer);
            error!("{}", e);
            return None;
        }
    }
    .to_str()
    .unwrap_or_default();

    let variables: Vec<f32, 2> = sensor_ascii
        .trim()
        .split(',')
        .flat_map(|var| var.parse::<f32>())
        .collect();

    if variables.len() != 2 {
        error!("Failed to parse two floats: {:?}", variables);
        builtin_led.set_high();
        return None;
    } else {
        builtin_led.set_low();
    }

    Some(SensorData::new(variables[1], variables[0]))
}

async fn send_alert(sensor_data: SensorData, sender: &DynSender<'static, DetectionStatus>) {
    match sensor_data.velocity.total_cmp(&VELOCITY_THRESHOLD) {
        core::cmp::Ordering::Less => (),
        _ => {
            sender.send(DetectionStatus::ObjectDetected);
        }
    }
}

#[cfg(not(feature = "wifi"))]
#[embassy_executor::task]
pub async fn read_uart(
    mut uart: UartRx<'static, Async>,
    signal: &'static Signal<CriticalSectionRawMutex, SensorData>,
    mut builtin_led: Output<'static>,
) {
    let sender = WATCH.dyn_sender();

    loop {
        if let Some(sensor_data) = process_uart_read(&mut uart, &mut builtin_led).await {
            signal.signal(sensor_data);

            send_alert(sensor_data, &sender).await;
        }
    }
}

#[cfg(feature = "wifi")]
#[embassy_executor::task]
pub async fn read_uart(
    mut uart: UartRx<'static, Async>,
    signal: &'static Signal<CriticalSectionRawMutex, SensorData>,
    mut builtin_led: Output<'static>,
) {
    let sender = WATCH.dyn_sender();

    loop {
        if let Some(sensor_data) = process_uart_read(&mut uart, &mut builtin_led).await {
            signal.signal(sensor_data.clone());

            send_alert(sensor_data, &sender).await;
        }
    }
}
