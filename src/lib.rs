#![no_std]

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
