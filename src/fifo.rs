/// Configuration for the MPU9250's First-In-First-Out (FIFO) buffer.
///
/// The FIFO buffer is a hardware queue that stores sensor readings, providing several benefits:
/// - Reduces I2C bus traffic by allowing batch reading of multiple samples
/// - Ensures no data is lost even if the host processor is temporarily busy
/// - Enables precise timing between samples
/// - Helps synchronize data from different sensors
///
/// Fields:
/// - `temp`: Enable temperature sensor data in FIFO
/// - `xg`, `yg`, `zg`: Enable individual gyroscope axis data (X, Y, Z)
/// - `accel`: Enable accelerometer data (all axes)
/// - `slv0`, `slv1`, `slv2`: Enable auxiliary I2C slave device data
///
/// When enabled, each sensor's data will be written to the FIFO buffer
/// at the configured sample rate. The buffer can hold up to 1024 bytes
/// of sensor data.
#[derive(Debug, Default, Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Fifo {
    pub temp: bool,
    pub xg: bool,
    pub yg: bool,
    pub zg: bool,
    pub accel: bool,
    pub slv2: bool,
    pub slv1: bool,
    pub slv0: bool,
}

impl Fifo {
    /// Creates a new FIFO configuration with all sensors disabled.
    ///
    /// This is the safest default state as it:
    /// - Prevents unwanted data accumulation
    /// - Minimizes power consumption
    /// - Allows selective enabling of only needed sensors
    pub fn all_disabled() -> Self {
        Self::default()
    }

    /// Converts a register byte into a FIFO configuration.
    ///
    /// Bit mapping:
    /// - Bit 7: Temperature
    /// - Bit 6-4: Gyroscope (X,Y,Z)
    /// - Bit 3: Accelerometer
    /// - Bit 2-0: I2C Slaves
    pub(crate) fn from_byte(byte: u8) -> Self {
        Self {
            temp: (byte & 0b1000_0000) != 0,
            xg: (byte & 0b0100_0000) != 0,
            yg: (byte & 0b0010_0000) != 0,
            zg: (byte & 0b0001_0000) != 0,
            accel: (byte & 0b0000_1000) != 0,
            slv2: (byte & 0b0000_0100) != 0,
            slv1: (byte & 0b0000_0010) != 0,
            slv0: (byte & 0b0000_0001) != 0,
        }
    }

    pub(crate) fn to_byte(self) -> u8 {
        let mut byte = 0;
        if self.temp {
            byte |= 1 << 7;
        }
        if self.xg {
            byte |= 1 << 6;
        }
        if self.yg {
            byte |= 1 << 5;
        }
        if self.zg {
            byte |= 1 << 4;
        }
        if self.accel {
            byte |= 1 << 3;
        }
        if self.slv2 {
            byte |= 1 << 2;
        }
        if self.slv1 {
            byte |= 1 << 1;
        }
        if self.slv0 {
            byte |= 1 << 0;
        }

        byte
    }
}
