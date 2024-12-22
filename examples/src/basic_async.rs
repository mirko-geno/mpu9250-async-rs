//! Embassy Async MPU6050-DMP Example
//!
//! This example demonstrates using the MPU6050 sensor with Embassy's async runtime on a Raspberry Pi Pico.
//! It shows how to:
//! - Initialize the sensor with async I2C
//! - Load and initialize the DMP firmware
//! - Perform sensor calibration
//! - Continuously read accelerometer, gyroscope and temperature data
//!
//! Hardware Setup:
//! - Connect MPU6050 to Raspberry Pi Pico:
//!   - SDA -> GP14
//!   - SCL -> GP15
//!   - VCC -> 3.3V
//!   - GND -> GND

#![no_std]
#![no_main]

use defmt::{info, Debug2Format};
use embassy_executor::Spawner;
use embassy_rp::{block::ImageDef, config::Config, i2c::InterruptHandler};
use embassy_time::{Delay, Timer};
use {defmt_rtt as _, panic_probe as _};

// mpu6050-dmp
use mpu6050_dmp::sensor_async::Mpu6050;
use mpu6050_dmp::{address::Address, calibration::CalibrationParameters};

embassy_rp::bind_interrupts!(struct Irqs {
    I2C1_IRQ => InterruptHandler<embassy_rp::peripherals::I2C1>;
});

/// Firmware image type for bootloader
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

/// Firmware entry point
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Config::default());

    // Initialize the MPU6050 sensor using I2C1
    // GP14 (SDA) and GP15 (SCL) are used for I2C communication
    // The MPU6050 operates as an I2C slave device with a default address of 0x68
    let sda = p.PIN_14;
    let scl = p.PIN_15;
    let config = embassy_rp::i2c::Config::default();
    let bus = embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, config);
    let mut sensor = Mpu6050::new(bus, Address::default()).await.unwrap();
    let mut delay = Delay;
    info!("MPU6050-DMP Sensor Initialized");

    // Initialize the Digital Motion Processor (DMP) firmware
    // The DMP provides enhanced motion processing capabilities and sensor fusion
    // This step resets the DMP to its default state and loads the firmware
    info!("Initializing DMP Firmware");
    sensor.initialize_dmp(&mut delay).await.unwrap();
    info!("DMP Firmware Initialized");

    // Read raw accelerometer data (uncalibrated)
    // The accelerometer measures linear acceleration in three axes (X, Y, Z)
    // Values will be imprecise until calibration is performed
    let accel_data = sensor.accel().await.unwrap();
    info!("Accelerometer Data: {:?}", Debug2Format(&accel_data));

    // Read raw gyroscope data (uncalibrated)
    // The gyroscope measures angular velocity in three axes (X, Y, Z)
    // Values will have drift and bias until calibration is performed
    let gyro_data = sensor.gyro().await.unwrap();
    info!("Gyroscope Data: {:?}", Debug2Format(&gyro_data));

    // Configure sensor calibration parameters
    // AccelFullScale options: G2, G4, G8, G16 (higher means larger range, lower precision)
    // GyroFullScale options: Deg250, Deg500, Deg1000, Deg2000 (degrees/second range)
    // ReferenceGravity: XN, XP, YN, YP, ZN, ZP (axis and direction of gravity during calibration)
    let calibration_params = CalibrationParameters::new(
        mpu6050_dmp::accel::AccelFullScale::G2,
        mpu6050_dmp::gyro::GyroFullScale::Deg2000,
        mpu6050_dmp::calibration::ReferenceGravity::ZN,
    );

    info!("Calibrating Sensor");
    sensor
        .calibrate(&mut delay, &calibration_params)
        .await
        .unwrap();
    info!("Sensor Calibrated");

    // Read the accelerometer data from the mpu6050-dmp sensor again after calibration
    let accel_data = sensor.accel().await.unwrap();
    info!("Accelerometer Data: {:?}", Debug2Format(&accel_data));

    // Read the gyroscope data from the mpu6050-dmp sensor again after calibration
    let gyro_data = sensor.gyro().await.unwrap();
    info!("Gyroscope Data: {:?}", Debug2Format(&gyro_data));

    // Main loop: Read sensor data every second
    // - Accelerometer: returns g-force per axis, including gravity
    // - Gyroscope: returns rotational velocity in degrees/second
    // - Temperature: returns degrees Celsius
    loop {
        let (accel, gyro, temp) = (
            sensor.accel().await.unwrap(),
            sensor.gyro().await.unwrap(),
            sensor.temperature().await.unwrap().celsius(),
        );
        info!(
            "Sensors - Accel: {:?}, Gyro: {:?}, Temp: {:?}°C",
            Debug2Format(&accel),
            Debug2Format(&gyro),
            Debug2Format(&temp)
        );
        Timer::after_millis(1000).await;
    }
}
