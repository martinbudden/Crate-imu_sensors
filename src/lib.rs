#![no_std]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::panic)]
#![deny(unused_must_use)]

mod axes;
mod imu;
mod lsm6ds;
mod mpu6050;
mod mpu6886;

pub use axes::ImuAxesOrder;
pub use imu::{AccScale, I2cInterface, Imu, ImuBus, ImuConfig, ImuReading, ImuState, SetupError};
