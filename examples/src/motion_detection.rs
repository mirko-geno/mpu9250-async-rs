//! MPU6050-DMP Motion Detection Example
//!
//! This example demonstrates software motion detection using the MPU6050's accelerometer:
//! - Initializing and calibrating the sensor
//! - Reading accelerometer data
//! - Detecting motion by monitoring acceleration changes
//! - Detecting periods of stillness
//!
//! Note: This example uses software motion detection since the hardware
//! motion detection registers are not currently exposed in the driver.
//! Consider contributing to the driver to add support for hardware motion detection.
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
use libm::sqrt;
use num_traits::float::FloatCore;
use {defmt_rtt as _, panic_probe as _};

// mpu6050-dmp
use mpu6050_dmp::{address::Address, calibration::CalibrationParameters, sensor_async::Mpu6050};

embassy_rp::bind_interrupts!(struct Irqs {
    I2C1_IRQ => InterruptHandler<embassy_rp::peripherals::I2C1>;
});

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

// Motion detection parameters
const MOTION_THRESHOLD: f64 = 0.2; // g-force threshold for motion detection
const STILLNESS_THRESHOLD: f64 = 0.05; // g-force threshold for stillness detection
const STILLNESS_DURATION: u32 = 10; // number of consecutive readings below threshold for stillness

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

    info!("Starting motion detection");

    // Variables for motion detection
    let mut last_accel = sensor.accel().await.unwrap();
    let mut stillness_counter = 0;
    let mut in_motion = false;

    // Main loop monitoring motion
    loop {
        let accel = sensor.accel().await.unwrap();

        // Get scaled acceleration values in g-force units
        let accel_g = accel.scaled(mpu6050_dmp::accel::AccelFullScale::G2);
        let last_accel_g = last_accel.scaled(mpu6050_dmp::accel::AccelFullScale::G2);

        // Calculate acceleration change in g-force
        let delta_x = (accel_g.x() - last_accel_g.x()).abs();
        let delta_y = (accel_g.y() - last_accel_g.y()).abs();
        let delta_z = (accel_g.z() - last_accel_g.z()).abs();

        // Total acceleration change
        let total_delta = sqrt((delta_x * delta_x + delta_y * delta_y + delta_z * delta_z) as f64);

        // Motion detection
        if total_delta > MOTION_THRESHOLD as f64 {
            if !in_motion {
                info!("Motion detected!");
                info!("Acceleration: {:?}", Debug2Format(&accel));
                in_motion = true;
            }
            stillness_counter = 0;
        } else if total_delta < STILLNESS_THRESHOLD as f64 {
            stillness_counter += 1;
            if stillness_counter >= STILLNESS_DURATION && in_motion {
                info!("Zero motion (standstill) detected");
                in_motion = false;
            }
        } else {
            stillness_counter = 0;
        }

        last_accel = accel;
        Timer::after_millis(50).await;
    }
}
