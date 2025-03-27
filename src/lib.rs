#![no_std]

#[cfg(feature="wifi")]
use esp_hal::peripherals::{RADIO_CLK, RNG, TIMG0, WIFI};

#[derive(Debug, Clone, Copy)]
pub enum DetectionStatus {
    ObjectDetected,
}

#[derive(Debug, Clone, Copy)]
pub struct SensorData {
    pub velocity: f32,
    pub time: f32,
}

impl SensorData {
    pub fn new(velocity: f32, time: f32) -> Self {
        Self { velocity, time }
    }
}

#[cfg(feature="wifi")]
pub struct WifiPeripherals {
    pub timg0: TIMG0,
    pub rng: RNG,
    pub radio_clk: RADIO_CLK,
    pub wifi: WIFI,
}
