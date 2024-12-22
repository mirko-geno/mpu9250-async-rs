//! MPU6050 Motion Detection Example
//!
//! This example demonstrates how to use the MPU6050's hardware motion detection feature
//! to efficiently detect movement without constant CPU monitoring. The implementation:
//!
//! 1. Configures the sensor for maximum sensitivity:
//!    - Sets minimum threshold (2mg) to detect subtle movements
//!    - Uses fastest response time (1ms) for immediate detection
//!    - Enables detection on all axes for complete coverage
//!    - Configures high-pass filter to remove gravity bias
//!
//! 2. Optimizes sensor settings:
//!    - Uses most sensitive accelerometer range (±2g)
//!    - Sets maximum sample rate (1kHz)
//!    - Configures DLPF for best motion detection
//!    - Performs proper sensor calibration
//!
//! 3. Monitors motion events:
//!    - Uses interrupts for efficient detection
//!    - Provides detailed motion data when triggered
//!    - Shows both acceleration and rotation values
//!    - Logs motion detection status for debugging
//!
//! The configuration used here prioritizes maximum sensitivity and fast response.
//! For different use cases:
//! - Increase threshold to ignore minor movements
//! - Increase duration to require sustained motion
//! - Adjust DLPF settings to filter more noise
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
    address::Address, calibration::CalibrationParameters, motion::MotionConfig,
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

    // Configure sensor settings
    sensor
        .set_clock_source(mpu6050_dmp::clock_source::ClockSource::Xgyro)
        .await
        .unwrap();

    // Set accelerometer full scale to most sensitive range
    sensor
        .set_accel_full_scale(mpu6050_dmp::accel::AccelFullScale::G2)
        .await
        .unwrap();

    // Configure DLPF for maximum sensitivity
    sensor
        .set_digital_lowpass_filter(mpu6050_dmp::config::DigitalLowPassFilter::Filter6)
        .await
        .unwrap();

    // Set sample rate to 1kHz (1ms period)
    sensor.set_sample_rate_divider(0).await.unwrap();

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

    // Configure motion detection with maximum sensitivity
    let motion_config = MotionConfig {
        threshold: 1, // 2mg threshold (minimum possible)
        duration: 1,  // 1ms at 1kHz sample rate (fastest response)
    };
    sensor
        .configure_motion_detection(&motion_config)
        .await
        .unwrap();
    sensor.enable_motion_interrupt().await.unwrap();

    info!("Starting motion detection");

    // Main loop monitoring motion detection events
    loop {
        // Wait for motion detection events
        if sensor.wait_for_motion(&mut delay).await.unwrap().0 {
            // Get motion status and current sensor data
            let motion = sensor.check_motion().await.unwrap();
            let (accel, gyro) = sensor.motion6().await.unwrap();

            info!("Motion detected! Status: {}", motion.0);
            info!(
                "Accel: x={}, y={}, z={}",
                accel.x() as i32,
                accel.y() as i32,
                accel.z() as i32
            );
            info!(
                "Gyro: x={}, y={}, z={}",
                gyro.x() as i32,
                gyro.y() as i32,
                gyro.z() as i32
            );
        }
    }
}
