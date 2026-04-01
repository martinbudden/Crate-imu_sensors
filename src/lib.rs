#![doc = include_str!("../README.md")]
#![no_std]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::panic)]
#![deny(unused_must_use)]

#[cfg(all(feature = "i2c", feature = "spi"))]
compile_error!("Features 'i2c' and 'spi' are mutually exclusive and cannot be enabled together.");

mod axes;
mod i2c;
mod imu;
mod imu_bus;
mod lsm6ds;
mod mpu6050;
mod mpu6886;
mod spi;

pub use axes::ImuAxesOrder;
pub use i2c::I2cInterface;
pub use imu::{AccScale, Imu, ImuCommon, ImuConfig, ImuReading, ImuReadingf32, ImuReadingf64};
pub use imu_bus::{ImuBus, MockImuBus, SetupError};
pub use lsm6ds::Lsm6ds;
pub use mpu6050::Mpu6050;
pub use mpu6886::Mpu6886;
pub use spi::SpiInterface;

/*
In main:
let cs_pin = Output::new(p.PIN_1.into(), Level::High); // .into() converts to AnyPin
let wrapper = SpiBusWrapper { spi, cs: cs_pin };
*/
