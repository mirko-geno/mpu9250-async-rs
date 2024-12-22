//! Motion detection configuration for the MPU-6050.
//!
//! This module provides types for configuring the hardware motion detection
//! features of the MPU-6050 sensor.

/// Motion detection configuration parameters
#[derive(Debug, Clone, Copy)]
pub struct MotionConfig {
    /// Motion detection threshold in mg (1LSB = 2mg)
    /// Range: 0-255 (0-510mg)
    pub threshold: u8,

    /// Number of consecutive samples that must exceed threshold
    /// Duration = (sample_rate / 1000) * duration
    /// Range: 0-255
    pub duration: u8,
}

impl Default for MotionConfig {
    fn default() -> Self {
        Self {
            // Default 40mg threshold (20 * 2mg)
            threshold: 20,
            // Default ~5ms at 1kHz sample rate
            duration: 5,
        }
    }
}

/// Motion detection status
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotionStatus {
    /// No motion detected
    Still,
    /// Motion detected
    Moving,
}

impl From<u8> for MotionStatus {
    fn from(value: u8) -> Self {
        if (value & 0x40) != 0 {
            MotionStatus::Moving
        } else {
            MotionStatus::Still
        }
    }
}
