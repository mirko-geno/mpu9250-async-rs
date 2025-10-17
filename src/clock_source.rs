//! MPU9250 Clock Source Configuration
//!
//! The MPU9250 can use different clock sources for timing:
//! - Internal oscillator (default, less accurate)
//! - Gyroscope reference (more stable, recommended)
//! - External crystals (highest accuracy)
//!
//! The clock source affects:
//! - Timing accuracy
//! - Power consumption
//! - Sensor startup time
//! - Temperature sensitivity

/// Available clock sources for the MPU9250.
///
/// Clock source selection considerations:
/// - Accuracy requirements
/// - Power consumption needs
/// - Temperature variations
/// - External component availability
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ClockSource {
    /// Internal 8MHz oscillator
    /// - Fastest startup
    /// - Less accurate
    /// - Higher temperature drift
    Internal = 0,

    /// X-axis gyroscope reference
    /// - Recommended for general use
    /// - Good stability
    /// - Low temperature drift
    Xgyro = 1,

    /// Y-axis gyroscope reference
    /// - Alternative to X-axis
    /// - Similar stability to X-axis
    Ygyro = 2,

    /// Z-axis gyroscope reference
    /// - Alternative to X/Y-axis
    /// - Similar stability to X/Y-axis
    Zgyro = 3,

    /// External 32.768kHz crystal
    /// - Highest accuracy
    /// - Requires external crystal
    /// - Common RTC frequency
    External32768 = 4,

    /// External 19.2MHz crystal
    /// - High accuracy
    /// - Requires external crystal
    /// - Typical system clock frequency
    External19200 = 5,

    /// Stops the clock
    /// - Lowest power consumption
    /// - Sensor stops operating
    /// - Must be restarted to resume
    Stop = 7,
}
