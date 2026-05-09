#![allow(unused)]
use core::f32::consts::PI;
use vqm::{Vector3d, Vector3df32, Vector3di16};

use crate::{Imu, ImuAxesOrder, ImuBus, ImuCommon, ImuConfig};

const REG_ACC_XL: u8 = 0x20;
const REG_ACC_XH: u8 = 0x21;
const REG_ACC_YL: u8 = 0x22;
const REG_ACC_YH: u8 = 0x23;
const REG_ACC_ZL: u8 = 0x24;
const REG_ACC_ZH: u8 = 0x25;
const REG_GYRO_XL: u8 = 0x26;
const REG_GYRO_XH: u8 = 0x27;
const REG_GYRO_YL: u8 = 0x28;
const REG_GYRO_YH: u8 = 0x29;
const REG_GYRO_ZL: u8 = 0x2A;
const REG_GYRO_ZH: u8 = 0x2B;

pub struct ImuMock<B: ImuBus> {
    pub bus: B,
    pub common: ImuCommon,
    pub config: ImuConfig,
}

impl<B: ImuBus> Imu for ImuMock<B> {
    type Bus = B;
    type Error = <B as ImuBus>::Error;

    fn bus(&mut self) -> &mut Self::Bus {
        &mut self.bus
    }

    fn common(&self) -> &ImuCommon {
        &self.common
    }

    fn common_mut(&mut self) -> &mut ImuCommon {
        &mut self.common
    }

    fn config(&self) -> &ImuConfig {
        &self.config
    }

    async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.bus.bus_write_read(address, write, read).await
    }

    async fn read_acc(&mut self) -> Result<Vector3df32, Self::Error>
    where
        <B as ImuBus>::Error: From<<B as ImuBus>::Error>,
    {
        let mut buf = [0u8; 6];
        self.bus().read_registers(0, REG_ACC_XL, &mut buf).await;
        Ok(self.map_acc(buf, self.common.axis_order))
    }

    async fn read_gyro_rps(&mut self) -> Result<Vector3df32, Self::Error>
    where
        <B as ImuBus>::Error: From<<B as ImuBus>::Error>,
    {
        let mut buf = [0u8; 6];
        self.bus().read_registers(0, REG_GYRO_XL, &mut buf).await;
        Ok(self.map_gyro_rps(buf, self.common.axis_order))
    }

    async fn read_gyro_dps(&mut self) -> Result<Vector3df32, Self::Error>
    where
        <B as ImuBus>::Error: From<<B as ImuBus>::Error>,
    {
        let mut buf = [0u8; 6];
        self.bus().read_registers(0, REG_GYRO_XL, &mut buf).await;
        Ok(self.map_gyro_dps(buf, self.common.axis_order))
    }

    async fn read_acc_gyro_rps(&mut self) -> Result<(Vector3df32, Vector3df32), Self::Error>
    where
        <B as ImuBus>::Error: From<<B as ImuBus>::Error>,
    {
        let mut buf = [0u8; 12];
        self.bus().read_registers(0, REG_ACC_XL, &mut buf).await;
        Ok(self.map_acc_gyro_rps(buf, self.common.axis_order))
    }
}

impl<B: ImuBus> ImuMock<B> {
    pub fn new(bus: B, axis_order: ImuAxesOrder) -> Self {
        Self {
            bus,
            common: ImuCommon::new(axis_order),
            config: ImuConfig {
                gyro_id_msp: ImuConfig::MSP_ACC_ID_DEFAULT,
                acc_id_msp: ImuConfig::MSP_ACC_ID_DEFAULT,
                axis_order: axis_order.into(),
                device_id: 0,
                address: 0,
                flags: 0,
            },
        }
    }

    pub async fn set_acc(&mut self, acc: Vector3df32) {
        let acc_unscaled = (acc + self.common.acc_offset) / self.common.acc_scale;
        let acc_i16: Vector3di16 = acc_unscaled.into();
        let x = acc_i16.x.to_le_bytes();
        let y = acc_i16.y.to_le_bytes();
        let z = acc_i16.z.to_le_bytes();
        let data = [x[0], x[1], y[0], y[1], z[0], z[1]];
        self.bus().write_registers(0, REG_ACC_XL, &data).await;
    }

    pub async fn set_gyro_dps(&mut self, gyro_dps: Vector3df32) {
        let gyro_dps_unscaled = (gyro_dps + self.common.gyro_offset_dps) / self.common.gyro_scale_dps;
        let gyro_dps_i16: Vector3di16 = gyro_dps_unscaled.into();
        let x = gyro_dps_i16.x.to_le_bytes();
        let y = gyro_dps_i16.y.to_le_bytes();
        let z = gyro_dps_i16.z.to_le_bytes();
        let data = [x[0], x[1], y[0], y[1], z[0], z[1]];
        self.bus().write_registers(0, REG_GYRO_XL, &data).await;
    }

    /// # Errors
    pub async fn read_register(&mut self, reg: u8) -> Result<u8, B::Error> {
        self.bus.read_register(self.config.address, reg).await
    }

    /// # Errors
    pub async fn init(
        &mut self,
        _target_output_data_rate_hz: u32,
        gyro_sensitivity: u8,
        acc_sensitivity: u8,
    ) -> Result<(u32, u32), B::Error> {
        self.bus.write_register(0, 0, 0).await?;
        match acc_sensitivity {
            ImuCommon::ACC_FULL_SCALE_2G => {
                self.common.acc_scale = 2.0 / 32768.0;
            }
            ImuCommon::ACC_FULL_SCALE_4G => {
                self.common.acc_scale = 4.0 / 32768.0;
            }
            ImuCommon::ACC_FULL_SCALE_8G => {
                self.common.acc_scale = 8.0 / 32768.0;
            }
            _ => {
                // default includes  ImuCommon::ACC_FULL_SCALE_16G
                self.common.acc_scale = 16.0 / 32768.0;
            }
        }
        match gyro_sensitivity {
            ImuCommon::GYRO_FULL_SCALE_125_DPS => {
                self.common.gyro_scale_dps = 125.0 / 32768.0;
            }
            ImuCommon::GYRO_FULL_SCALE_250_DPS => {
                self.common.gyro_scale_dps = 250.0 / 32768.0;
            }
            ImuCommon::GYRO_FULL_SCALE_500_DPS => {
                self.common.gyro_scale_dps = 500.0 / 32768.0;
            }
            ImuCommon::GYRO_FULL_SCALE_1000_DPS => {
                self.common.gyro_scale_dps = 1000.0 / 32768.0;
            }
            _ => {
                // default includes ImuCommon::GYRO_FULL_SCALE_2000_DPS
                self.common.gyro_scale_dps = 2000.0 / 32768.0;
            }
        }
        self.common.gyro_offset_rps = self.common.gyro_offset_dps.to_radians();

        Ok((0, 0))
    }

    pub fn map_acc(&self, buf: [u8; 6], axis_order: ImuAxesOrder) -> Vector3df32 {
        let acc16 = Vector3di16 {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        };
        let acc = Vector3df32::from(acc16) * self.common.acc_scale - self.common.acc_offset;
        ImuAxesOrder::map_vector(axis_order, &acc)
    }

    pub fn map_gyro_rps(&self, buf: [u8; 6], axis_order: ImuAxesOrder) -> Vector3df32 {
        let gyro16 = Vector3di16 {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        };
        let gyro_rps = Vector3df32::from(gyro16) * self.common.gyro_scale_rps - self.common.gyro_offset_rps;
        ImuAxesOrder::map_vector(axis_order, &gyro_rps)
    }

    pub fn map_gyro_dps(&self, buf: [u8; 6], axis_order: ImuAxesOrder) -> Vector3df32 {
        let gyro16 = Vector3di16 {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        };
        let gyro_dps = Vector3df32::from(gyro16) * self.common.gyro_scale_dps - self.common.gyro_offset_dps;
        ImuAxesOrder::map_vector(axis_order, &gyro_dps)
    }

    pub fn map_acc_gyro_rps(&self, buf: [u8; 12], axis_order: ImuAxesOrder) -> (Vector3df32, Vector3df32) {
        let acc16 = Vector3di16 {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        };
        let gyro16 = Vector3di16 {
            x: i16::from_le_bytes([buf[6], buf[7]]),
            y: i16::from_le_bytes([buf[8], buf[9]]),
            z: i16::from_le_bytes([buf[10], buf[11]]),
        };

        let acc = Vector3df32::from(acc16) * self.common.acc_scale - self.common.acc_offset;
        let gyro_rps = Vector3df32::from(gyro16) * self.common.gyro_scale_rps - self.common.gyro_offset_rps;

        ImuAxesOrder::map_acc_gyro_rps(axis_order, acc, gyro_rps)
    }
}

#[cfg(test)]
mod tests {
    // we can do float comparisons because all floats have been converted from i16s, and so can be represented exactly.
    #![allow(clippy::float_cmp)]

    use super::*;
    use crate::{ImuAxesOrder, MockImuBus};

    fn is_normal<T: Sized + Send + Sync + Unpin>() {}

    #[test]
    fn normal_types() {
        is_normal::<ImuMock<MockImuBus>>();
    }
    #[test]
    fn imu_init() {
        let imu_bus = MockImuBus::new();
        let mut imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);

        let result = pollster::block_on(imu.init(8000, ImuCommon::GYRO_FULL_SCALE_MAX, ImuCommon::ACC_FULL_SCALE_MAX));
        let (gyro_register_value, acc_register_value) = result.unwrap();

        assert_eq!(0, gyro_register_value);
        assert_eq!(0, acc_register_value);
        //assert_eq!(2000.0 / 32768.0, state.gyro_scale_dps);
        //assert_eq!(16.0 / 32768.0, state.acc_scale);
        //assert_eq!(6664, state.gyro_sample_rate_hz);
        //assert_eq!(6664, state.acc_sample_rate_hz);
    }
    #[test]
    fn acc_buf() {
        let imu_bus = MockImuBus::new();
        let mut imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);

        // TODO: sit down and work out some useful test data for this
        let data: [u8; 6] = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        let acc = imu.map_acc(data, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32 { x: 0.0, y: 0.0, z: 0.0 }, acc);
    }
    #[test]
    fn gyro_buf() {
        let imu_bus = MockImuBus::new();
        let mut imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);

        // TODO: sit down and work out some useful test data for this
        let data: [u8; 6] = [0x10, 0x00, 0x00, 0x01, 0x00, 0x7f];
        let gyro_dps = imu.map_gyro_dps(data, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32 { x: 0.976_562_5, y: 15.625, z: 1984.375 }, gyro_dps);

        let data: [u8; 6] = [0x01, 0x00, 0x80, 0x00, 0xff, 0x7f];
        let gyro_dps = imu.map_gyro_dps(data, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32 { x: 0.061_035_156, y: 7.8125, z: 1999.939 }, gyro_dps);
    }
    #[test]
    fn scale_acc() {
        let imu_bus = MockImuBus::new();
        let mut imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);

        let acc_scale = imu.acc_scale() * 32768.0;
        assert_eq!(8.0, acc_scale);

        let acc = Vector3df32::new(0.5, 2.0, 1.0);
        pollster::block_on(imu.set_acc(acc));

        let mut buf = [0u8; 6];
        let result = pollster::block_on(imu.bus().read_registers(0, REG_ACC_XL, &mut buf));
        assert_eq!([0x00, 0x08, 0x00, 0x20, 0x00, 0x10], buf);

        let a = imu.map_acc(buf, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32::new(0.5, 2.0, 1.0), a);

    }
    #[test]
    fn read_acc() {
        let imu_bus = MockImuBus::new();
        let mut imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);

        let acc = Vector3df32::new(0.5, 2.0, 1.0);
        pollster::block_on(imu.set_acc(acc));
        let result = pollster::block_on(imu.read_acc());
        let (a) = result.unwrap();
        assert_eq!(Vector3df32::new(0.5, 2.0, 1.0), a);
    }
    #[test]
    fn scale_gyro() {
        let imu_bus = MockImuBus::new();
        let mut imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);
        let gyro_scale_dps = imu.gyro_scale_dps() * 32768.0;
        assert_eq!(2000.0, gyro_scale_dps);

        let mut buf = [0u8; 6];

        let gyro_dps = Vector3df32::new(125.0, 1000.0, 1750.0);
        pollster::block_on(imu.set_gyro_dps(gyro_dps));
        let result = pollster::block_on(imu.bus().read_registers(0, REG_GYRO_XL, &mut buf));
        assert_eq!([0x00, 0x08, 0x00, 0x40, 0x00, 0x70], buf);
        let g = imu.map_gyro_dps(buf, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32 { x: 125.0, y: 1000.0, z: 1750.0 }, gyro_dps);

        let gyro_dps = Vector3df32::new(500.0, 1000.0, 2000.0);
        pollster::block_on(imu.set_gyro_dps(gyro_dps));
        let result = pollster::block_on(imu.bus().read_registers(0, REG_GYRO_XL, &mut buf));
        assert_eq!([0x00, 0x20, 0x00, 0x40, 0xFF, 0x7F], buf);
        let g = imu.map_gyro_dps(buf, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32 { x: 500.0, y: 1000.0, z: 1999.939 }, g);

        let gyro_dps = Vector3df32::new(2000.0, 4000.0, 10_000.0);
        pollster::block_on(imu.set_gyro_dps(gyro_dps));
        let result = pollster::block_on(imu.bus().read_registers(0, REG_GYRO_XL, &mut buf));
        assert_eq!([0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F], buf);
        let g = imu.map_gyro_dps(buf, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32 { x: 1999.939, y: 1999.939, z: 1999.939 }, g);

        let gyro_dps = Vector3df32::new(-2000.0, -4000.0, -10_000.0);
        pollster::block_on(imu.set_gyro_dps(gyro_dps));
        let result = pollster::block_on(imu.bus().read_registers(0, REG_GYRO_XL, &mut buf));
        assert_eq!([0x00, 0x80, 0x00, 0x80, 0x00, 0x80], buf);
        let g = imu.map_gyro_dps(buf, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32 { x: -2000.0, y: -2000.0, z: -2000.0 }, g);
    }
    #[test]
    fn read_gyro() {
        let imu_bus = MockImuBus::new();
        let mut imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);
        let gyro_scale_dps = imu.gyro_scale_dps() * 32768.0;
        assert_eq!(2000.0, gyro_scale_dps);

        let mut buf = [0u8; 6];

        let gyro_dps = Vector3df32::new(125.0, 1000.0, 1750.0);
        pollster::block_on(imu.set_gyro_dps(gyro_dps));
        let result = pollster::block_on(imu.read_gyro_dps());
        let (g) = result.unwrap();
        assert_eq!(Vector3df32 { x: 125.0, y: 1000.0, z: 1750.0 }, g);

        let gyro_dps = Vector3df32::new(500.0, 1000.0, 2000.0);
        pollster::block_on(imu.set_gyro_dps(gyro_dps));
        let result = pollster::block_on(imu.read_gyro_dps());
        let (g) = result.unwrap();
        assert_eq!(Vector3df32 { x: 500.0, y: 1000.0, z: 1999.939 }, g);

        let gyro_dps = Vector3df32::new(2000.0, 4000.0, 10_000.0);
        pollster::block_on(imu.set_gyro_dps(gyro_dps));
        let result = pollster::block_on(imu.read_gyro_dps());
        let (g) = result.unwrap();
        assert_eq!(Vector3df32 { x: 1999.939, y: 1999.939, z: 1999.939 }, g);

        let gyro_dps = Vector3df32::new(-2000.0, -4000.0, -10_000.0);
        pollster::block_on(imu.set_gyro_dps(gyro_dps));
        let result = pollster::block_on(imu.read_gyro_dps());
        let (g) = result.unwrap();
        assert_eq!(Vector3df32 { x: -2000.0, y: -2000.0, z: -2000.0 }, g);
    }
}
