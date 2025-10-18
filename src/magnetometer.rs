//! Magnetometer Data Processing
//!
//! The MPU9250's magnetometer measures direction of a magnetic field at a point in space along three axes:
//! - X
//! - Y
//! - Z
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "postcard-experimental", derive(postcard::experimental::max_size::MaxSize))]
pub struct Mag {
    pub(crate) x: i16,
    pub(crate) y: i16,
    pub(crate) z: i16,
}

impl Mag {
    pub const fn new(x: i16, y:i16, z:i16) -> Self {
        Self { x, y, z }
    }

    /// Converts raw sensor bytes into magnetometer values.
    ///
    /// The MPU9250 provides 14 or 16 bit resolution magnetometer values,
    /// this library implements the 16 bit representation as:
    /// - 2 bytes per axis in Little-endian byte order
    /// - Signed integers
    pub const fn from_bytes(data: [u8; 6]) -> Self {
        let x = [data[0], data[1]];
        let y = [data[2], data[3]];
        let z = [data[4], data[5]];
        Self {
            x: i16::from_le_bytes(x),
            y: i16::from_le_bytes(y),
            z: i16::from_le_bytes(z),
        }
    }

    pub const fn to_bytes(&self) -> [u8; 6] {
        let x = self.x.to_le_bytes();
        let y = self.y.to_le_bytes();
        let z = self.z.to_le_bytes();
        [x[0], x[1], y[0], y[1], z[0], z[1]]
    }
}