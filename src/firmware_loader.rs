//! DMP Firmware Loader for MPU6050
//!
//! The Digital Motion Processor (DMP) is a programmable processor inside the MPU6050 that:
//! - Offloads complex motion processing algorithms from the main processor
//! - Provides more accurate sensor fusion and motion tracking
//! - Reduces power consumption by handling processing on-chip
//! - Enables advanced features like quaternion calculations and gesture detection
//!
//! This module handles loading the DMP firmware into the sensor's program memory.
//! The process involves:
//! 1. Writing the firmware data in chunks to the sensor's memory banks
//! 2. Booting the firmware to start DMP operation
//! 3. Configuring DMP features as needed

use embedded_hal::i2c::I2c;

use crate::dmp_firmware::FIRMWARE;
use crate::error::Error;
use crate::registers::Register;
use crate::sensor::Mpu6050;

/// Size of each memory bank in the DMP program memory
const BANK_SIZE: usize = 256;

/// Maximum size of data that can be written in one I2C transaction
const CHUNK_SIZE: usize = 16;

impl<I> Mpu6050<I>
where
    I: I2c,
{
    /// Loads the DMP firmware into the sensor's program memory.
    ///
    /// This writes the complete firmware image in chunks:
    /// 1. Divides firmware into BANK_SIZE (256 byte) sections
    /// 2. Further divides each bank into CHUNK_SIZE (16 byte) pieces
    /// 3. Writes each chunk using I2C while managing memory addressing
    pub fn load_firmware(&mut self) -> Result<(), Error<I>> {
        self.write_memory(&FIRMWARE)
    }

    /// Starts execution of the loaded DMP firmware.
    ///
    /// This must be called after load_firmware() to:
    /// 1. Set the program start address (0x0400)
    /// 2. Enable DMP execution
    /// 3. Begin motion processing
    pub fn boot_firmware(&mut self) -> Result<(), Error<I>> {
        self.write(&[Register::PrgmStart as u8, 0x04, 0x00])
    }

    /// Writes firmware data to the DMP program memory.
    ///
    /// The DMP memory is organized into banks of 256 bytes each.
    /// This function:
    /// 1. Splits the firmware into bank-sized chunks
    /// 2. Writes each bank sequentially
    /// 3. Manages bank selection and addressing
    fn write_memory(&mut self, data: &[u8]) -> Result<(), Error<I>> {
        for (bank, chunk) in data.chunks(BANK_SIZE).enumerate() {
            self.write_bank(bank as u8, chunk)?;
        }
        Ok(())
    }

    /// Writes a 256-byte bank of firmware data.
    ///
    /// Due to I2C limitations, each bank is written in 16-byte chunks:
    /// 1. Selects the target memory bank
    /// 2. Sets the starting address within the bank
    /// 3. Writes the data in CHUNK_SIZE pieces
    /// 4. Adds necessary protocol bytes to each chunk
    fn write_bank(&mut self, bank: u8, data: &[u8]) -> Result<(), Error<I>> {
        self.set_bank(bank)?;

        for (i, chunk) in data.chunks(CHUNK_SIZE).enumerate() {
            let mut prolog_and_chunk: [u8; CHUNK_SIZE + 1] = [0; CHUNK_SIZE + 1];
            prolog_and_chunk[0] = Register::MemRw as u8;
            for (i, b) in chunk.iter().enumerate() {
                prolog_and_chunk[i + 1] = *b;
            }
            self.set_memory_start_address((i * CHUNK_SIZE) as u8)?;
            self.write(&prolog_and_chunk)?;
        }

        //log::info!("write {}", data.len());
        Ok(())
    }

    fn set_bank(&mut self, bank: u8) -> Result<(), Error<I>> {
        //log::info!("set bank={}", bank);
        self.write_register(Register::BankSel, bank)
    }

    fn set_memory_start_address(&mut self, addr: u8) -> Result<(), Error<I>> {
        //log::info!("set mem={}", addr);
        self.write_register(Register::MemStartAddr, addr)
    }
}
