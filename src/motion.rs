//! Motion detection configuration for the MPU-6050.
//!
//! The MPU-6050 provides hardware motion detection capabilities that can efficiently
//! detect movement without constant CPU monitoring. The detection works by:
//!
//! 1. Comparing consecutive accelerometer samples
//! 2. Triggering when the difference exceeds the configured threshold
//! 3. Only signaling after the threshold is exceeded for the specified duration
//!
//! Key concepts:
//! - Threshold (in mg): How much acceleration change triggers detection
//! - Duration (in samples): How many consecutive samples must exceed threshold
//! - High-pass filter: Removes gravity bias for better motion isolation
//! - All-axis detection: Monitors movement in any direction
//!
//! For best results:
//! - Use lower threshold (2-10mg) for subtle movement detection
//! - Use shorter duration (1-5ms) for quick response
//! - Enable the high-pass filter to remove gravity effects
//! - Configure all axes for complete motion coverage

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

/// Indicates if motion was detected based on the configured threshold and duration
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotionDetected(pub bool);

impl From<u8> for MotionDetected {
    fn from(value: u8) -> Self {
        // Bit 6 indicates motion was detected
        MotionDetected((value & 0x40) != 0)
    }
}
