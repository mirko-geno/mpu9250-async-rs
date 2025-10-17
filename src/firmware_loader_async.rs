use embedded_hal_async::i2c::I2c;

use crate::dmp_firmware::FIRMWARE;
use crate::error_async::Error;
use crate::registers::Register;
use crate::sensor_async::Mpu9250;

const BANK_SIZE: usize = 256;
const CHUNK_SIZE: usize = 16;

impl<I> Mpu9250<I>
where
    I: I2c,
{
    pub async fn load_firmware(&mut self) -> Result<(), Error<I>> {
        //log::info!("loading firmware");
        self.write_memory(&FIRMWARE).await
        //self.boot_firmware()
    }

    pub async fn boot_firmware(&mut self) -> Result<(), Error<I>> {
        self.write(&[Register::PrgmStart as u8, 0x04, 0x00]).await
    }

    async fn write_memory(&mut self, data: &[u8]) -> Result<(), Error<I>> {
        for (bank, chunk) in data.chunks(BANK_SIZE).enumerate() {
            self.write_bank(bank as u8, chunk).await?;
        }
        Ok(())
    }

    async fn write_bank(&mut self, bank: u8, data: &[u8]) -> Result<(), Error<I>> {
        self.set_bank(bank).await?;

        for (i, chunk) in data.chunks(CHUNK_SIZE).enumerate() {
            let mut prolog_and_chunk: [u8; CHUNK_SIZE + 1] = [0; CHUNK_SIZE + 1];
            prolog_and_chunk[0] = Register::MemRw as u8;
            for (i, b) in chunk.iter().enumerate() {
                prolog_and_chunk[i + 1] = *b;
            }
            self.set_memory_start_address((i * CHUNK_SIZE) as u8)
                .await?;
            self.write(&prolog_and_chunk).await?;
        }

        //log::info!("write {}", data.len());
        Ok(())
    }

    async fn set_bank(&mut self, bank: u8) -> Result<(), Error<I>> {
        //log::info!("set bank={}", bank);
        self.write_register(Register::BankSel, bank).await
    }

    async fn set_memory_start_address(&mut self, addr: u8) -> Result<(), Error<I>> {
        //log::info!("set mem={}", addr);
        self.write_register(Register::MemStartAddr, addr).await
    }
}
