//! MPU6050 I2C Address Configuration
//!
//! The MPU6050 uses I2C for communication and can be configured with
//! one of two 7-bit addresses:
//! - 0x68 (default, AD0 pin low)
//! - 0x69 (alternate, AD0 pin high)
//!
//! This allows two MPU6050 devices to be used on the same I2C bus.
//! The address is selected by connecting the AD0 pin to either:
//! - GND for address 0x68
//! - VCC for address 0x69

/// Represents an MPU6050 I2C address.
///
/// The address is determined by the AD0 pin:
/// - When AD0 is connected to GND: 0x68 (default)
/// - When AD0 is connected to VCC: 0x69
///
/// Note: These are 7-bit addresses. Some I2C implementations may
/// require left-shifting by 1 to create the 8-bit address.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Address(pub u8);

impl Default for Address {
    /// Returns the default I2C address (0x68).
    ///
    /// This is the address used when:
    /// - AD0 pin is connected to GND
    /// - AD0 pin is left floating (has internal pulldown)
    fn default() -> Self {
        Self(0x68)
    }
}

impl From<Address> for u8 {
    /// Converts the address wrapper to raw u8 value.
    /// Used internally for I2C communication.
    fn from(addr: Address) -> Self {
        addr.0
    }
}

impl From<u8> for Address {
    /// Creates an address from raw u8 value.
    /// Typically used with either 0x68 or 0x69.
    fn from(addr: u8) -> Self {
        Self(addr)
    }
}
