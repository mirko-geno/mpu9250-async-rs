//! MPU9250 Error Types
//!
//! This module defines the error types that can occur when:
//! - Initializing the sensor
//! - Communicating over I2C
//! - Validating sensor responses

use core::fmt::{Debug, Formatter};
use embedded_hal::i2c::I2c;

/// Error that occurs during sensor initialization.
///
/// Initialization can fail due to:
/// - I2C communication errors
/// - Wrong device connected (incorrect WHO_AM_I response)
/// - Power-on reset failure
/// - Configuration errors
///
/// Contains both the error and the I2C interface for error recovery.
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct InitError<I>
where
    I: I2c,
{
    pub i2c: I,
    pub error: Error<I>,
}

impl<I> Debug for InitError<I>
where
    I: I2c,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        self.error.fmt(f)
    }
}

/// Errors that can occur during normal sensor operation.
///
/// These errors typically happen when:
/// - Reading sensor data
/// - Writing configuration values
/// - Accessing the FIFO buffer
/// - Loading DMP firmware
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Error<I>
where
    I: I2c,
{
    /// Failed to write data to sensor
    /// Common causes:
    /// - I2C bus busy
    /// - Device not responding
    /// - Incorrect address
    WriteError(I::Error),

    /// Failed to read data from sensor
    /// Common causes:
    /// - I2C bus busy
    /// - Device not responding
    /// - Invalid register address
    WriteReadError(I::Error),

    /// Connected device is not an MPU9250
    /// Common causes:
    /// - Wrong device connected
    /// - Incorrect I2C address
    /// - Device malfunction
    WrongDevice,
}

impl<I> Debug for Error<I>
where
    I: I2c,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::result::Result<(), core::fmt::Error> {
        match self {
            Self::WriteReadError(e) => f.debug_tuple("WriteReadError").field(e).finish(),
            Self::WriteError(e) => f.debug_tuple("WriteError").field(e).finish(),
            Self::WrongDevice => f.write_str("WrongDevice"),
        }
    }
}
