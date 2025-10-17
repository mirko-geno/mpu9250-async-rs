//! Blocking Calibration Implementation
//!
//! This module provides synchronous (blocking) versions of the calibration routines:
//! - Collecting sensor readings with blocking I/O
//! - Computing calibration offsets
//! - Adjusting sensor calibration with blocking I2C operations
//!
//! The calibration process runs in a loop until all axes are within
//! their specified thresholds, using standard blocking I/O operations.

use crate::{
    accel::{Accel, AccelFullScale},
    calibration::{
        CalibrationActions, CalibrationParameters, MeanAccumulator, ReferenceGravity, DELAY_MS,
        ITERATIONS, WARMUP_ITERATIONS,
    },
    error::Error,
    gyro::Gyro,
    sensor::Mpu9250,
};
use embedded_hal::{delay::DelayNs, i2c::I2c};

/// Perform a single loop computing the means of the readings
pub fn collect_mean_values<I>(
    mpu: &mut Mpu9250<I>,
    delay: &mut impl DelayNs,
    accel_scale: AccelFullScale,
    gravity: ReferenceGravity,
) -> Result<(Accel, Gyro), Error<I>>
where
    I: I2c,
{
    let mut accumulator = MeanAccumulator::new(accel_scale, gravity);

    for _ in 0..WARMUP_ITERATIONS {
        _ = mpu.accel()?;
        _ = mpu.gyro()?;
        delay.delay_ms(DELAY_MS);
    }

    for _ in 0..ITERATIONS {
        let accel = mpu.accel()?;
        let gyro = mpu.gyro()?;
        accumulator.add(&accel, &gyro);
        delay.delay_ms(DELAY_MS);
    }

    Ok(accumulator.means())
}

/// A single, full-fledged calibration loop (it also alters the device offset)
pub fn calibration_loop<I>(
    mpu: &mut Mpu9250<I>,
    delay: &mut impl DelayNs,
    parameters: &CalibrationParameters,
    actions: CalibrationActions,
) -> Result<(CalibrationActions, Accel, Gyro), Error<I>>
where
    I: I2c,
{
    let mut actions = actions;

    mpu.set_accel_full_scale(parameters.accel_scale)?;
    mpu.set_gyro_full_scale(parameters.gyro_scale)?;
    let mut accel_offset = mpu.get_accel_calibration()?;
    let mut gyro_offset = mpu.get_gyro_calibration()?;
    let (accel_mean, gyro_mean) =
        collect_mean_values(mpu, delay, parameters.accel_scale, parameters.gravity)?;

    if parameters.accel_threshold.is_value_within(accel_mean.x()) {
        actions = actions.with_accel_x(false);
    }
    if parameters.accel_threshold.is_value_within(accel_mean.y()) {
        actions = actions.with_accel_y(false);
    }
    if parameters.accel_threshold.is_value_within(accel_mean.z()) {
        actions = actions.with_accel_z(false);
    }
    if parameters.gyro_threshold.is_value_within(gyro_mean.x()) {
        actions = actions.with_gyro_x(false);
    }
    if parameters.gyro_threshold.is_value_within(gyro_mean.y()) {
        actions = actions.with_gyro_y(false);
    }
    if parameters.gyro_threshold.is_value_within(gyro_mean.z()) {
        actions = actions.with_gyro_z(false);
    }

    if actions.accel_x() {
        accel_offset.x = parameters
            .accel_threshold
            .next_offset(accel_mean.x(), accel_offset.x());
    }
    if actions.accel_y() {
        accel_offset.y = parameters
            .accel_threshold
            .next_offset(accel_mean.y(), accel_offset.y());
    }
    if actions.accel_z() {
        accel_offset.z = parameters
            .accel_threshold
            .next_offset(accel_mean.z(), accel_offset.z());
    }
    if actions.gyro_x() {
        gyro_offset.x = parameters
            .gyro_threshold
            .next_offset(gyro_mean.x(), gyro_offset.x());
    }
    if actions.gyro_y() {
        gyro_offset.y = parameters
            .gyro_threshold
            .next_offset(gyro_mean.y(), gyro_offset.y());
    }
    if actions.gyro_z() {
        gyro_offset.z = parameters
            .gyro_threshold
            .next_offset(gyro_mean.z(), gyro_offset.z());
    }

    if !actions.is_empty() {
        mpu.set_accel_calibration(&accel_offset)?;
        mpu.set_gyro_calibration(&gyro_offset)?;
    }

    Ok((actions, accel_mean, gyro_mean))
}

/// Repeatedly perform calibration loops until the errors are within the given thresholds
pub fn calibrate<I>(
    mpu: &mut Mpu9250<I>,
    delay: &mut impl DelayNs,
    parameters: &CalibrationParameters,
) -> Result<(Accel, Gyro), Error<I>>
where
    I: I2c,
{
    let mut actions = CalibrationActions::all();
    while !actions.is_empty() {
        (actions, _, _) = calibration_loop(mpu, delay, parameters, actions)?;
    }
    Ok((mpu.get_accel_calibration()?, mpu.get_gyro_calibration()?))
}
