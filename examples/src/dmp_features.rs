//! MPU6050-DMP Sample Rate Example
//!
//! This example demonstrates configuring and using the MPU6050's sample rate features:
//! - Initializing the Digital Motion Processor (DMP)
//! - Configuring custom sample rate for motion data
//! - Reading combined accelerometer and gyroscope data (6-axis)
//! - High-frequency data sampling (100Hz)
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
use mpu6050_dmp::{address::Address, calibration::CalibrationParameters, sensor_async::Mpu6050};

embassy_rp::bind_interrupts!(struct Irqs {
    I2C1_IRQ => InterruptHandler<embassy_rp::peripherals::I2C1>;
});

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Config::default());

    // Initialize I2C and sensor
    let sda = p.PIN_14;
    let scl = p.PIN_15;
    let config = embassy_rp::i2c::Config::default();
    let bus = embassy_rp::i2c::I2c::new_async(p.I2C1, scl, sda, Irqs, config);
    let mut sensor = Mpu6050::new(bus, Address::default()).await.unwrap();
    let mut delay = Delay;

    info!("MPU6050-DMP Sensor Initialized");

    // Initialize DMP with advanced features
    info!("Initializing DMP");
    sensor.initialize_dmp(&mut delay).await.unwrap();

    // Configure calibration parameters
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

    // Configure sample rate
    // Higher rates for fast movements (up to 1000Hz)
    // Lower rates for slow movements and power saving
    sensor.set_sample_rate_divider(9).await.unwrap(); // 100Hz (1000Hz / (1 + 9))
    info!("Sample rate configured");

    // Main loop demonstrating DMP features
    loop {
        // Read 6-axis motion data
        let (accel, gyro) = sensor.motion6().await.unwrap();
        info!(
            "Accel: {:?}, Gyro: {:?}",
            Debug2Format(&accel),
            Debug2Format(&gyro)
        );

        Timer::after_millis(10).await;
    }
}
