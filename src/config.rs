//! MPU6050 Digital Low-Pass Filter Configuration
//!
//! The digital low-pass filter (DLPF) reduces noise in sensor readings by:
//! - Filtering out high-frequency vibrations
//! - Smoothing rapid changes in sensor data
//! - Reducing aliasing effects
//!
//! Filter selection involves balancing:
//! - Noise reduction (higher filtering)
//! - Response time (lower filtering)
//! - Power consumption
//! - Output data rate

/// Digital low-pass filter configurations.
///
/// Each filter setting provides different:
/// - Bandwidth (Hz): How much high-frequency data passes through
/// - Delay (ms): How long it takes for changes to be reflected
/// - Noise reduction: How much sensor noise is filtered out
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum DigitalLowPassFilter {
    /// Minimal filtering
    /// - Fastest response
    /// - Highest bandwidth
    /// - Least noise reduction
    Filter0 = 0,

    /// Light filtering
    /// - Good balance for most uses
    /// - Quick response
    /// - Moderate noise reduction
    Filter1 = 1,

    /// Moderate filtering
    /// - Balanced response/filtering
    /// - Good for general motion
    Filter2 = 2,

    /// Strong filtering
    /// - Better noise reduction
    /// - Slower response
    /// - Good for stable readings
    Filter3 = 3,

    /// Very strong filtering
    /// - High noise reduction
    /// - Significant delay
    /// - Good for slow movements
    Filter4 = 4,

    /// Heavy filtering
    /// - Very high noise reduction
    /// - Long delay
    /// - Best for very slow changes
    Filter5 = 5,

    /// Maximum filtering
    /// - Maximum noise reduction
    /// - Longest delay
    /// - Use only for nearly static measurements
    Filter6 = 6,
}
