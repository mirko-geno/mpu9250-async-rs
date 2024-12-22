//! MPU6050 Register Map and Descriptions
//!
//! The MPU6050 is controlled through its registers, which are organized into groups:
//! - Configuration registers: Control sensor behavior and features
//! - Data registers: Store sensor measurements
//! - Calibration registers: Store offset values
//! - DMP registers: Control the Digital Motion Processor
//! - FIFO registers: Manage the FIFO buffer
//!
//! Note: Some hardware features like motion detection are controlled through
//! registers not currently exposed in this enum (MOT_THR, MOT_DUR, etc.).
//! These could be added to enable hardware-based motion detection.

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Register {
    /// Configuration register (0x1A)
    /// Controls the digital low pass filter and external sync
    Config = 0x1A,

    /// Power Management 1 register (0x6B)
    /// Controls device power state, clock source, and reset
    PwrMgmt1 = 0x6B,

    /// Sample Rate Divider register (0x19)
    /// Sets the sample rate by dividing the gyroscope output
    SmpRtDiv = 0x19,

    // Accelerometer Calibration Registers
    /// High byte of X-axis accelerometer offset
    AccelOffsetX_H = 0x06,
    /// Low byte of X-axis accelerometer offset
    AccelOffsetX_L = 0x07,
    /// High byte of Y-axis accelerometer offset
    AccelOffsetY_H = 0x08,
    /// Low byte of Y-axis accelerometer offset
    AccelOffsetY_L = 0x09,
    /// High byte of Z-axis accelerometer offset
    AccelOffsetZ_H = 0x0A,
    /// Low byte of Z-axis accelerometer offset
    AccelOffsetZ_L = 0x0B,

    // Gyroscope Calibration Registers
    /// High byte of X-axis gyroscope offset
    GyroOffsetX_H = 0x13,
    /// Low byte of X-axis gyroscope offset
    GyroOffsetX_L = 0x14,
    /// High byte of Y-axis gyroscope offset
    GyroOffsetY_H = 0x15,
    /// Low byte of Y-axis gyroscope offset
    GyroOffsetY_L = 0x16,
    /// High byte of Z-axis gyroscope offset
    GyroOffsetZ_H = 0x17,
    /// Low byte of Z-axis gyroscope offset
    GyroOffsetZ_L = 0x18,

    // Accelerometer Data Registers
    /// High byte of X-axis acceleration
    AccelX_H = 0x3B,
    /// Low byte of X-axis acceleration
    AccelX_L = 0x3C,
    /// High byte of Y-axis acceleration
    AccelY_H = 0x3D,
    /// Low byte of Y-axis acceleration
    AccelY_L = 0x3E,
    /// High byte of Z-axis acceleration
    AccelZ_H = 0x3F,
    /// Low byte of Z-axis acceleration
    AccelZ_L = 0x40,

    // Temperature Data Registers
    /// High byte of temperature reading
    TempOut_H = 0x41,
    /// Low byte of temperature reading
    TempOut_L = 0x42,

    /// Accelerometer Configuration register (0x1C)
    /// Controls full-scale range and high pass filter
    AccelConfig = 0x1C,

    // Gyroscope Data Registers
    /// High byte of X-axis angular rate
    GyroX_H = 0x43,
    /// Low byte of X-axis angular rate
    GyroX_L = 0x44,
    /// High byte of Y-axis angular rate
    GyroY_H = 0x45,
    /// Low byte of Y-axis angular rate
    GyroY_L = 0x46,
    /// High byte of Z-axis angular rate
    GyroZ_H = 0x47,
    /// Low byte of Z-axis angular rate
    GyroZ_L = 0x48,

    /// Gyroscope Configuration register (0x1B)
    /// Controls full-scale range
    GyroConfig = 0x1B,

    /// User Control register (0x6A)
    /// Controls FIFO and I2C configuration
    UserCtrl = 0x6A,

    /// Interrupt Enable register (0x38)
    /// Controls which interrupts are enabled
    IntEnable = 0x38,

    /// Interrupt Status register (0x3A)
    /// Indicates which interrupts have been triggered
    IntStatus = 0x3A,

    // FIFO Registers
    /// FIFO Enable register (0x23)
    /// Controls which sensor data goes to FIFO
    FifoEn = 0x23,
    /// High byte of FIFO byte count
    FifoCount_H = 0x72,
    /// Low byte of FIFO byte count
    FifoCount_L = 0x73,
    /// FIFO Read Write register
    FifoRw = 0x74,

    // DMP Registers
    /// DMP Bank Select
    BankSel = 0x6D,
    /// DMP Memory Start Address
    MemStartAddr = 0x6E,
    /// DMP Memory Read Write
    MemRw = 0x6F,
    /// DMP Program Start Address
    PrgmStart = 0x70,
    /// DMP Configuration
    DmpConfig = 0x71,

    // Motion Detection Registers
    /// Motion Detection Threshold (1LSB = 2mg)
    MotionThreshold = 0x1F,
    /// Motion Detection Duration Counter
    MotionDuration = 0x20,
    /// Motion Detection Control
    MotionDetectCtrl = 0x69,
    /// Motion Detection Status
    MotionDetectStatus = 0x61,
}
