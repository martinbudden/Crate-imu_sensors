use serde::{Deserialize, Serialize};

use crate::{ImuAxesOrder, ImuBus};
use vqm::{Vector3d, Vector3df32};

// Shared data members
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuCommon {
    pub acc_offset: Vector3df32,
    pub gyro_offset: Vector3df32,
    pub gyro_scale_rps: f32,
    pub gyro_scale_dps: f32,
    pub acc_scale: f32,
    pub gyro_sample_rate_hz: u32,
    pub acc_sample_rate_hz: u32,
    pub axis_order: ImuAxesOrder,
}

impl ImuCommon {
    pub const GYRO_FULL_SCALE_MAX: u8 = 0;
    pub const GYRO_FULL_SCALE_125_DPS: u8 = 1;
    pub const GYRO_FULL_SCALE_250_DPS: u8 = 2;
    pub const GYRO_FULL_SCALE_500_DPS: u8 = 3;
    pub const GYRO_FULL_SCALE_1000_DPS: u8 = 4;
    pub const GYRO_FULL_SCALE_2000_DPS: u8 = 5;
    pub const GYRO_FULL_SCALE_4000_DPS: u8 = 6;

    pub const ACC_FULL_SCALE_MAX: u8 = 0;
    pub const ACC_FULL_SCALE_1G: u8 = 1;
    pub const ACC_FULL_SCALE_2G: u8 = 2;
    pub const ACC_FULL_SCALE_4G: u8 = 3;
    pub const ACC_FULL_SCALE_8G: u8 = 4;
    pub const ACC_FULL_SCALE_16G: u8 = 5;
    pub const ACC_FULL_SCALE_32G: u8 = 6;

    pub fn new(axis_order: ImuAxesOrder) -> Self {
        const GYRO_2000DPS_RES: f32 = 2000.0 / 32768.0;
        const ACC_8G_RES: f32 = 8.0 / 32768.0;
        Self {
            acc_offset: Vector3df32::default(),
            gyro_offset: Vector3df32::default(),
            gyro_scale_dps: GYRO_2000DPS_RES,
            gyro_scale_rps: GYRO_2000DPS_RES.to_radians(),
            acc_scale: ACC_8G_RES,
            gyro_sample_rate_hz: 1000,
            acc_sample_rate_hz: 1000,
            axis_order,
        }
    }
}

impl Default for ImuCommon {
    fn default() -> Self {
        Self::new(ImuAxesOrder::XPOS_YPOS_ZPOS)
    }
}

// Imu configuration, set on construction and read-only thereafter
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
pub struct ImuConfig {
    pub gyro_id_msp: u16,
    pub acc_id_msp: u16,
    pub device_id: u8, // 8-bit id assigned by IMU manufacturer
    pub address: u8,
    pub axis_order: u8,
    pub flags: u8, // Flags for describing IMU characteristics
}

impl Default for ImuConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl ImuConfig {
    // Betaflight compatible acc and gyro ids
    // Used for reporting gyro and acc type back to MSP (MultiWii Serial Protocol)
    pub const MSP_ACC_ID_DEFAULT: u16 = 0;
    pub const MSP_ACC_ID_NONE: u16 = 1;
    pub const MSP_ACC_ID_MPU6050: u16 = 2;
    pub const MSP_ACC_ID_MPU6000: u16 = 3;
    pub const MSP_ACC_ID_ICM42605: u16 = 11;
    pub const MSP_ACC_ID_ICM42688P: u16 = 12;
    pub const MSP_ACC_ID_LSM6DS: u16 = 19;
    pub const MSP_ACC_ID_VIRTUAL: u16 = 21;

    pub const MSP_GYRO_ID_NONE: u16 = 0;
    pub const MSP_GYRO_ID_DEFAULT: u16 = 1;
    pub const MSP_GYRO_ID_MPU6050: u16 = 2;
    pub const MSP_GYRO_ID_MPU6000: u16 = 4;
    pub const MSP_GYRO_ID_ICM42605: u16 = 12;
    pub const MSP_GYRO_ID_ICM42688P: u16 = 13;
    pub const MSP_GYRO_ID_LSM6DS: u16 = 18;
    pub const MSP_GYRO_ID_VIRTUAL: u16 = 20;

    fn new() -> Self {
        Self {
            gyro_id_msp: 0,
            acc_id_msp: 0,
            device_id: 0, // 8-bit id assigned by IMU manufacturer
            address: 0,
            axis_order: 0,
            flags: 0, // Flags for describing IMU characteristics
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuReading<T> {
    pub acc: Vector3d<T>,
    pub gyro_rps: Vector3d<T>,
}

pub type ImuReadingf32 = ImuReading<f32>;
pub type ImuReadingf64 = ImuReading<f64>;

#[repr(u8)]
pub enum AccScale {
    Fs2g = 0x00,
    Fs4g = 0x01,
    Fs8g = 0x02,
    Fs16g = 0x03,
}

// Imu trait uses Bus as an associated type.
pub trait Imu {
    type Bus: ImuBus;
    // This forces the IMU error to be the same as the Bus error
    type Error: From<<Self::Bus as ImuBus>::Error>;

    const TARGET_OUTPUT_DATA_RATE_MAX: u8 = 0;

    fn bus(&mut self) -> &mut Self::Bus;
    fn common(&self) -> &ImuCommon;
    fn common_mut(&mut self) -> &mut ImuCommon;
    fn config(&self) -> &ImuConfig;

    #[allow(async_fn_in_trait)]
    async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error>;

    #[allow(async_fn_in_trait)]
    async fn read_acc(&mut self) -> Result<Vector3df32, Self::Error>;

    #[allow(async_fn_in_trait)]
    async fn read_gyro_rps(&mut self) -> Result<Vector3df32, Self::Error>;

    #[allow(async_fn_in_trait)]
    async fn read_acc_gyro_rps(&mut self) -> Result<ImuReadingf32, Self::Error>;

    fn gyro_offset(&self) -> Vector3df32 {
        self.common().gyro_offset
    }
    fn set_gyro_offset(&mut self, gyro_offset: Vector3df32) {
        self.common_mut().gyro_offset = gyro_offset;
    }
    fn acc_offset(&self) -> Vector3df32 {
        self.common().acc_offset
    }
    fn set_acc_offset(&mut self, acc_offset: Vector3df32) {
        self.common_mut().acc_offset = acc_offset;
    }
    fn gyro_offset_mapped(&self) -> Vector3df32 {
        self.common().axis_order.map_vector(&self.common().gyro_offset)
    }
    fn set_gyro_offset_mapped(&mut self, gyro_offset: Vector3df32) {
        let gyro_offset_mapped = self.common().axis_order.axes_order_inverse().map_vector(&gyro_offset);
        self.set_gyro_offset(gyro_offset_mapped);
    }
    fn acc_offset_mapped(&self) -> Vector3df32 {
        self.common().axis_order.map_vector(&self.common().acc_offset)
    }
    fn set_acc_offset_mapped(&mut self, acc_offset: Vector3df32) {
        let acc_offset_mapped = self.common().axis_order.axes_order_inverse().map_vector(&acc_offset);
        self.set_gyro_offset(acc_offset_mapped);
    }
}
