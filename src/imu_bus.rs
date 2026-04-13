#![allow(unused)]
//use embassy_rp::i2c::{I2c, Async};
#[cfg(feature = "rp2040")]
use embassy_rp::gpio::Output;
#[cfg(all(feature = "rp2040", feature = "spi"))]
use embassy_rp::gpio::Pin;
#[cfg(all(feature = "rp2040", feature = "i2c"))]
use embassy_rp::i2c::{Async, I2c, Instance};
#[cfg(all(feature = "rp2040", feature = "spi"))]
use embassy_rp::spi::{Async, Instance, Spi};
//use embedded_hal::i2c;
use embedded_hal_async::i2c;

use core::convert::Infallible;
use core::future::Future;

pub trait ImuBus {
    type Error;

    #[allow(async_fn_in_trait)]
    async fn bus_write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error>;
    fn read_register(&mut self, address: u8, reg: u8) -> impl core::future::Future<Output = Result<u8, Self::Error>>;
    fn read_registers(
        &mut self,
        address: u8,
        reg: u8,
        data: &mut [u8],
    ) -> impl core::future::Future<Output = Result<(), Self::Error>>;

    fn write_register(
        &mut self,
        address: u8,
        reg: u8,
        data: u8,
    ) -> impl core::future::Future<Output = Result<(), Self::Error>>;
    fn write_registers(
        &mut self,
        address: u8,
        reg: u8,
        data: &[u8],
    ) -> impl core::future::Future<Output = Result<(), Self::Error>>;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MockImuBus {
    // Standard IMU register maps are usually 128 or 256 bytes
    pub registers: [u8; 256],
}

impl Default for MockImuBus {
    fn default() -> Self {
        Self::new()
    }
}
impl MockImuBus {
    pub const fn new() -> Self {
        Self { registers: [0u8; 256] }
    }
}

impl ImuBus for MockImuBus {
    type Error = core::convert::Infallible;

    async fn bus_write_read(&mut self, _address: u8, _write: &[u8], _read: &mut [u8]) -> Result<(), Self::Error> {
        Ok(())
    }
    async fn read_register(&mut self, _address: u8, reg: u8) -> Result<u8, Self::Error> {
        Ok(self.registers[reg as usize])
    }

    async fn read_registers(&mut self, _address: u8, reg: u8, data: &mut [u8]) -> Result<(), Self::Error> {
        let start = reg as usize;
        let end = start + data.len();

        // Copy slice from internal memory to the output buffer
        data.copy_from_slice(&self.registers[start..end]);
        Ok(())
    }

    async fn write_register(&mut self, _address: u8, reg: u8, data: u8) -> Result<(), Self::Error> {
        self.registers[reg as usize] = data;
        Ok(())
    }

    async fn write_registers(&mut self, _address: u8, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        let start = reg as usize;
        let end = start + data.len();

        self.registers[start..end].copy_from_slice(data);
        Ok(())
    }
}

#[cfg(all(feature = "rp2040", feature = "i2c"))]
impl<T: Instance> ImuBus for I2c<'_, T, Async> {
    type Error = embassy_rp::i2c::Error;
    async fn bus_write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        // On the Pico, I2C write_read is natively async.
        // We just delegate the call and .await the result.
        self.write_read_async(address, write.iter().copied(), read).await

        // Explicitly call the method on the I2c struct itself
        // instead of let the compiler guess (and pick the trait method)
        //I2c::<'d, T, Async>::write_read(self, address, write, read).await
    }

    async fn read_register(&mut self, address: u8, reg: u8) -> Result<u8, Self::Error> {
        let mut buf = [0u8; 1];
        // Write the register address, read back 1 byte
        self.bus_write_read(address, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    async fn read_registers(&mut self, address: u8, reg: u8, data: &mut [u8]) -> Result<(), Self::Error> {
        // Write the starting register address, read back 'data.len()' bytes
        // The MPU6050 automatically increments the register pointer internally
        self.bus_write_read(address, &[reg], data).await
    }

    async fn write_register(&mut self, address: u8, reg: u8, data: u8) -> Result<(), Self::Error> {
        // To write, we send [register, value] and expect 0 bytes back
        self.bus_write_read(address, &[reg, data], &mut []).await
    }

    async fn write_registers(&mut self, address: u8, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        // This is trickier: I2C writes usually need the register and data in one contiguous stream.
        // For no_std, we can use a small local buffer or a loop if the bus supports it.
        // For a simple MPU6050 config, usually we only write 1-2 bytes at a time.

        // Example for up to 4 bytes of data:
        let mut buf = [0u8; 5];
        buf[0] = reg;
        let len = data.len().min(4);
        buf[1..=len].copy_from_slice(&data[..len]);

        self.bus_write_read(address, &buf[..=len], &mut []).await
    }
}

/*#[cfg(all(feature = "rp2040", feature = "spi"))]
pub struct SpiBusWrapper<'d, T: Instance, CS: Pin> {
    pub spi: Spi<'d, T, Async>,
    // Remove CS from the angle brackets here
    pub cs: Output<'d>,
    // This _marker ensures the compiler knows CS is "used"
    _pcs: core::marker::PhantomData<CS>,
}

#[cfg(all(feature = "rp2040", feature = "spi"))]
impl<'d, T: Instance, CS: embassy_rp::gpio::Pin> SpiBusWrapper<'d, T, CS> {
    async fn write_burst(&mut self, reg: u8, data: &[u8]) -> Result<(), embassy_rp::spi::Error> {
        self.cs.set_low();

        // 1. Write register (ensure Write bit is 0)
        let mut dummy = [0u8; 1];
        self.spi.transfer(&mut dummy, &[reg & 0x7F]).await?;

        // 2. Write all data bytes in the same session
        // We need a dummy buffer for the SPI read-back
        // (For small writes, a stack array is fine; for large, use a loop)
        for chunk in data.chunks(16) {
            let mut d_buf = [0u8; 16];
            self.spi.transfer(&mut d_buf[..chunk.len()], chunk).await?;
        }

        self.cs.set_high();
        Ok(())
    }
}
*/
#[cfg(all(feature = "rp2040", feature = "spi"))]
pub struct SpiBusWrapper<'d, T: Instance> {
    pub spi: Spi<'d, T, Async>,
    pub cs: Output<'d>,
}

#[cfg(all(feature = "rp2040", feature = "spi"))]
impl<'d, T: Instance> ImuBus for SpiBusWrapper<'d, T> {
    //impl<'d, T: Instance, CS: embassy_rp::gpio::Pin> ImuBus for SpiBusWrapper<'d, T, CS> {
    type Error = embassy_rp::spi::Error;

    async fn bus_write_read(&mut self, _addr: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.cs.set_low(); // Pull CS low to start transaction

        // Use transfer for full-duplex SPI communication
        let res = self.spi.transfer(read, write).await;

        self.cs.set_high(); // Pull CS high to end transaction
        res
    }

    async fn read_register(&mut self, address: u8, reg: u8) -> Result<u8, Self::Error> {
        let mut buf = [0u8; 1];

        // If SPI is enabled, we need to set the Read Bit (0x80)
        #[cfg(feature = "spi")]
        let reg = reg | 0x80;

        self.bus_write_read(address, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    async fn read_registers(&mut self, address: u8, reg: u8, data: &mut [u8]) -> Result<(), Self::Error> {
        // Write the starting register address, read back 'data.len()' bytes
        // The MPU6050 automatically increments the register pointer internally
        self.bus_write_read(address, &[reg], data).await
    }

    async fn write_register(&mut self, address: u8, reg: u8, data: u8) -> Result<(), Self::Error> {
        // To write, we send [register, value] and expect 0 bytes back
        self.bus_write_read(address, &[reg, data], &mut []).await
    }
    async fn write_registers(&mut self, address: u8, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        // For SPI Write, MSB of register must be 0 (reg & 0x7F)
        // We send [reg, data[0], data[1], ...]

        // Step 1: Send the register address
        // We use a dummy read buffer of the same size
        self.bus_write_read(0, &[reg & 0x7F], &mut [0u8; 1]).await?;

        // Step 2: Send the data bytes
        // In SPI, as long as CS stays low, the MPU increments the internal register pointer
        //self.bus_write_read(0, data, &mut data).await?;

        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum SetupError<E> {
    /// An error occurred with the I2C/SPI bus during setup.
    Bus(E),
    /// An incorrect 'Who Am I' value was returned from the IMU.
    ImuWhoAmI(u8),
}

impl<E> From<E> for SetupError<E> {
    fn from(error: E) -> Self {
        SetupError::Bus(error)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[allow(unused)]
    fn is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MockImuBus>();
    }

    #[test]
    fn test_imu_mock() {
        let mut bus = MockImuBus::new();
        let write_data = [0xAA, 0xBB];

        // In a test environment, you'd "await" these
        pollster::block_on(bus.write_registers(0, 0x10, &write_data)).unwrap();

        let mut read_data = [0u8; 2];
        pollster::block_on(bus.read_registers(0, 0x10, &mut read_data)).unwrap();

        assert_eq!(read_data, [0xAA, 0xBB]);
    }
}
