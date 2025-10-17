# `mpu9250-async`

## Fork of [mpu6050-dmp-rs](https://github.com/barafael/mpu6050-dmp-rs)
## Disclaimer:
I forked the original repo because I needed to use mpu9250 instead of mpu6050 for a school project and wanted to add the missing magnetometer readings through I2C. I'm not sure if I'll mantain this repo or add Master I2C protocol to enable DMP.

## Examples
The `examples` directory contains several examples demonstrating different features (For now only examples from the original repo):

- **Basic Async**: Basic sensor initialization and data reading
- **FIFO Buffer**: FIFO buffer operations and data processing
- **Quaternion**: 3D orientation tracking using quaternions
- **Motion Detection**: Hardware motion detection with configurable sensitivity

See the [examples README](examples/README.md) for detailed information.

## On-Chip DMP 'Digital Motion Processor'
This driver can load only the appropriate firmware for accelerometer gyroscope quaternion-based on-chip DMP processing.
## Setup

### i<sup>2</sup>c

Initialize your i<sup>2</sup>c bus:

```rust
let gpiob = dp.GPIOB.split();
let scl = gpiob
    .pb8
    .into_alternate()
    .internal_pull_up(true)
    .set_open_drain();
let sda = gpiob
    .pb9
    .into_alternate()
    .internal_pull_up(true)
    .set_open_drain();
let i2c = dp.I2C1.i2c((scl, sda), 400.kHz(), &clocks);
```

### MPU driver

```rust
let sensor = Mpu9250::new(i2c, Address::default()).unwrap();
```

### Temperature Measurement

The MPU-9250 includes an on-chip temperature sensor. Temperature readings are available through the `temperature()` method when the `temperature` feature is enabled (enabled by default):

```rust
// Get temperature reading
let temp = sensor.temperature().unwrap();
println!("Temperature: {}°C", temp.celsius());
```

Setting up the DMP requires temporary exclusive access to a blocking delay implementation.
The `initialize_dmp(&mut delay)` method is provided to set up reasonable configurations and load the DMP firmware into the processor.

```rust
sensor.initialize_dmp(&mut delay).unwrap();
```

If using the advanced on-chip DMP logic, the FIFO will contain 28-byte packets of quaternion and other data.

The first 16 bytes are quaternions, which can be constructed using the `Quaternion` class.

```rust
let len = sensor.get_fifo_count().unwrap();
if len >= 28 {
    let buf = sensor.read_fifo(&mut buf).unwrap();
    let q = Quaternion::from_bytes(&buf[..16]).unwrap().normalize();
    let ypr = YawPitchRoll::from(quat);
    rprintln!("{:?}", ypr);
}
```
