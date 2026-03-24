#![allow(unused)]
use core::convert::Infallible;
use core::future::Future;

pub trait ImuBus {
    type Error;

    fn read_register(&mut self, reg: u8) -> impl core::future::Future<Output = Result<u8, Self::Error>>;
    fn read_registers(
        &mut self,
        reg: u8,
        data: &mut [u8],
    ) -> impl core::future::Future<Output = Result<(), Self::Error>>;

    fn write_register(&mut self, reg: u8, data: u8) -> impl core::future::Future<Output = Result<(), Self::Error>>;
    fn write_registers(&mut self, reg: u8, data: &[u8]) -> impl core::future::Future<Output = Result<(), Self::Error>>;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MockImuBus {
    // Standard IMU register maps are usually 128 or 256 bytes
    pub registers: [u8; 128],
}

impl Default for MockImuBus {
    fn default() -> Self {
        Self::new()
    }
}
impl MockImuBus {
    pub const fn new() -> Self {
        Self { registers: [0u8; 128] }
    }
}

impl ImuBus for MockImuBus {
    type Error = core::convert::Infallible;

    async fn read_register(&mut self, reg: u8) -> Result<u8, Self::Error> {
        if reg as usize <= self.registers.len() {
            return Ok(self.registers[reg as usize]);
        }
        Ok(0)
    }
    async fn read_registers(&mut self, reg: u8, data: &mut [u8]) -> Result<(), Self::Error> {
        let start = reg as usize;
        let end = start + data.len();

        // Copy slice from internal memory to the output buffer
        if end <= self.registers.len() {
            data.copy_from_slice(&self.registers[start..end]);
        }
        Ok(())
    }

    async fn write_register(&mut self, reg: u8, data: u8) -> Result<(), Self::Error> {
        let start = reg as usize;

        if reg as usize <= self.registers.len() {
            self.registers[reg as usize] = data;
        }
        Ok(())
    }
    async fn write_registers(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        let start = reg as usize;
        let end = start + data.len();

        if end <= self.registers.len() {
            self.registers[start..end].copy_from_slice(data);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum SetupError<E> {
    /// An error occurred with the I2C/SPI bus during setup
    Bus(E),
    /// An incorrect 'Who Am I' value was returned from the IMU
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
        pollster::block_on(bus.write_registers(0x10, &write_data)).unwrap();

        let mut read_data = [0u8; 2];
        pollster::block_on(bus.read_registers(0x10, &mut read_data)).unwrap();

        assert_eq!(read_data, [0xAA, 0xBB]);
    }
}
