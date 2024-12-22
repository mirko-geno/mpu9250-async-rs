//! MPU6050-DMP Motion Detection Example
//!
//! This example demonstrates hardware motion detection using the MPU6050's built-in features:
//! - Initializing and calibrating the sensor
//! - Configuring hardware motion detection thresholds
//! - Using interrupts to detect movement changes
//! - Asynchronously waiting for movement state changes
//!
//! Hardware Setup:
//! - Connect MPU6050 to Raspberry Pi Pico:
//!   - SDA -> GP14
//!   - SCL -> GP15
//!   - VCC -> 3.3V
//!   - GND -> GND

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::{block::ImageDef, config::Config, i2c::InterruptHandler};
use embassy_time::Delay;
use {defmt_rtt as _, panic_probe as _};

// mpu6050-dmp
use mpu6050_dmp::{
    address::Address,
    calibration::CalibrationParameters,
    motion::{MotionConfig, MotionStatus},
    sensor_async::Mpu6050,
};

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

    // Initialize DMP
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

    // Configure motion detection
    let motion_config = MotionConfig {
        threshold: 1, // 40mg threshold
        duration: 10, // ~5ms at 1kHz sample rate
    };
    sensor
        .configure_motion_detection(&motion_config)
        .await
        .unwrap();
    sensor.enable_motion_interrupt().await.unwrap();

    info!("Starting movement detection");

    // Main loop monitoring movement changes
    loop {
        match sensor.wait_for_motion_change(&mut delay).await.unwrap() {
            MotionStatus::Moving => {
                info!("Movement detected!");
                // Get current acceleration for debugging
                let accel = sensor.accel().await.unwrap();
                info!(
                    "Current acceleration: x={}, y={}, z={}",
                    accel.x(),
                    accel.y(),
                    accel.z()
                );
            }
            MotionStatus::Still => {
                info!("Device is now stationary");
            }
        }
    }
}
