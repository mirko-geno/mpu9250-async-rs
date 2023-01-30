use core::fmt::Debug;
use core::fmt::Formatter;
use embedded_hal::blocking::i2c::{Write, WriteRead};

/// Error during initialization of sensor. Wraps [`Error`].
pub struct InitError<I2c>
where
    I2c: WriteRead + Write,
    <I2c as WriteRead>::Error: Debug,
    <I2c as Write>::Error: Debug,
{
    pub i2c: I2c,
    pub error: Error<I2c>,
}

impl<I2c> Debug for InitError<I2c>
where
    I2c: WriteRead + Write,
    <I2c as WriteRead>::Error: Debug,
    <I2c as Write>::Error: Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        self.error.fmt(f)
    }
}

/// Error for sensor operations.
pub enum Error<I2c>
where
    I2c: WriteRead + Write,
    <I2c as WriteRead>::Error: Debug,
    <I2c as Write>::Error: Debug,
{
    WriteError(<I2c as Write>::Error),
    WriteReadError(<I2c as WriteRead>::Error),
    WrongDevice,
}

impl<I2c> Debug for Error<I2c>
where
    I2c: WriteRead + Write,
    <I2c as WriteRead>::Error: Debug,
    <I2c as Write>::Error: Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::result::Result<(), core::fmt::Error> {
        match self {
            Self::WriteReadError(e) => f.debug_tuple("WriteReadError").field(e).finish(),
            Self::WriteError(e) => f.debug_tuple("WriteError").field(e).finish(),
            Self::WrongDevice => f.write_str("WrongDevice"),
        }
    }
}
