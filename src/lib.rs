#![doc = include_str!("../README.md")]
#![no_std]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::panic)]
//#![deny(missing_docs)]
#![deny(
    missing_copy_implementations,
    missing_debug_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unused_must_use,
    unused_extern_crates,
    unused_import_braces,
    unused_qualifications,
    unused_results
)]
#![warn(unused_results)]
#![warn(clippy::pedantic)]
#![warn(clippy::doc_paragraphs_missing_punctuation)]

#[cfg(all(feature = "i2c", feature = "spi"))]
compile_error!("Features 'i2c' and 'spi' are mutually exclusive and cannot be enabled together.");

mod axes;
mod bmi270;
mod i2c;
mod imu;
mod imu426xx;
mod imu_bus;
mod imu_mock;
mod lsm6ds;
mod mpu6050;
mod mpu6886;
mod qmi8658a;
mod spi;

pub use axes::ImuAxesOrder;
pub use bmi270::Bmi270;
pub use imu::{Imu, ImuAccScale, ImuCommon, ImuConfig, ImuGyroScale};
pub use imu_bus::{ImuBus, MockImuBus, SetupError};
pub use imu_mock::ImuMock;
pub use imu426xx::Imu426xx;
pub use lsm6ds::Lsm6ds;
pub use mpu6050::Mpu6050;
pub use mpu6886::Mpu6886;
pub use qmi8658a::Qmi8658a;
