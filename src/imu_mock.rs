use vqm::Vector3df32;

use crate::{
    Imu, ImuAxesOrder, ImuBus, ImuCommon, ImuConfig,
    imu::{ImuAccScale, ImuGyroScale},
};

const REG_ACC_XL: u8 = 0x20;
const _REG_ACC_XH: u8 = 0x21;
const _REG_ACC_YL: u8 = 0x22;
const _REG_ACC_YH: u8 = 0x23;
const _REG_ACC_ZL: u8 = 0x24;
const _REG_ACC_ZH: u8 = 0x25;
const REG_GYRO_XL: u8 = 0x26;
const _REG_GYRO_XH: u8 = 0x27;
const _REG_GYRO_YL: u8 = 0x28;
const _REG_GYRO_YH: u8 = 0x29;
const _REG_GYRO_ZL: u8 = 0x2A;
const _REG_GYRO_ZH: u8 = 0x2B;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[allow(missing_docs)]
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

    async fn write_read(&mut self, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.bus.bus_write_read(self.config.address, write, read).await
    }

    async fn read_acc(&mut self) -> Result<Vector3df32, Self::Error> {
        let mut buf = [0u8; 6];
        #[allow(clippy::expect_used)]
        self.bus().read_registers(0, REG_ACC_XL, &mut buf).await.expect("read_resisters cannot fail for ImuMock");
        Ok(self.map_acc(buf, self.common.axis_order))
    }

    async fn read_gyro(&mut self) -> Result<Vector3df32, Self::Error> {
        let mut buf = [0u8; 6];
        #[allow(clippy::expect_used)]
        self.bus().read_registers(0, REG_GYRO_XL, &mut buf).await.expect("read_resisters cannot fail for ImuMock");
        Ok(self.map_gyro(buf, self.common.axis_order))
    }

    async fn read_acc_gyro(&mut self) -> Result<(Vector3df32, Vector3df32), Self::Error> {
        let mut buf = [0u8; 12];
        #[allow(clippy::expect_used)]
        self.bus().read_registers(0, REG_ACC_XL, &mut buf).await.expect("read_resisters cannot fail for ImuMock");
        Ok(self.map_acc_gyro(buf, self.common.axis_order))
    }
}

impl<B: ImuBus> ImuMock<B> {
    /// Constructor.
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

    /// # Panics
    pub async fn set_acc(&mut self, acc: Vector3df32) {
        let acc_unscaled =
            ((acc + self.common.acc_offset) / self.common.acc_scale).clamp(f32::from(i16::MIN), f32::from(i16::MAX));
        #[allow(clippy::cast_possible_truncation)]
        let x_i16 = acc_unscaled.x as i16;
        #[allow(clippy::cast_possible_truncation)]
        let y_i16 = acc_unscaled.y as i16;
        #[allow(clippy::cast_possible_truncation)]
        let z_i16 = acc_unscaled.z as i16;

        let x = x_i16.to_le_bytes();
        let y = y_i16.to_le_bytes();
        let z = z_i16.to_le_bytes();
        let data = [x[0], x[1], y[0], y[1], z[0], z[1]];
        #[allow(clippy::expect_used)]
        self.bus().write_registers(0, REG_ACC_XL, &data).await.expect("write_resisters cannot fail for ImuMock");
    }

    /// # Panics
    pub async fn set_gyro(&mut self, gyro: Vector3df32) {
        let gyro_unscaled =
            ((gyro + self.common.gyro_offset) / self.common.gyro_scale).clamp(f32::from(i16::MIN), f32::from(i16::MAX));

        #[allow(clippy::cast_possible_truncation)]
        let x_i16 = gyro_unscaled.x as i16;
        #[allow(clippy::cast_possible_truncation)]
        let y_i16 = gyro_unscaled.y as i16;
        #[allow(clippy::cast_possible_truncation)]
        let z_i16 = gyro_unscaled.z as i16;

        let x = x_i16.to_le_bytes();
        let y = y_i16.to_le_bytes();
        let z = z_i16.to_le_bytes();
        let data = [x[0], x[1], y[0], y[1], z[0], z[1]];
        #[allow(clippy::expect_used)]
        self.bus().write_registers(0, REG_GYRO_XL, &data).await.expect("write_resisters cannot fail for ImuMock");
    }

    /// # Errors
    pub async fn read_register(&mut self, reg: u8) -> Result<u8, B::Error> {
        self.bus.read_register(self.config.address, reg).await
    }

    /// # Errors
    pub async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: u8,
        gyro_scale: ImuGyroScale,
        acc_sensitivity: u8,
        acc_scale: ImuAccScale,
    ) -> Result<(u32, u32), B::Error> {
        self.bus.write_register(0, 0, 0).await?;

        self.calculate_acc_scale(acc_scale, acc_sensitivity);

        self.calculate_gyro_scale_and_odr(gyro_sensitivity, gyro_scale, target_output_data_rate_hz);

        Ok((0, 0))
    }

    pub fn calculate_gyro_scale_and_odr(
        &mut self,
        gyro_sensitivity: u8,
        gyro_scale: ImuGyroScale,
        target_output_data_rate_hz: u32,
    ) {
        let scale_dps = match gyro_sensitivity {
            ImuCommon::GYRO_FULL_SCALE_125_DPS => 125.0,
            ImuCommon::GYRO_FULL_SCALE_250_DPS => 250.0,
            ImuCommon::GYRO_FULL_SCALE_500_DPS => 500.0,
            ImuCommon::GYRO_FULL_SCALE_1000_DPS => 1000.0,
            _ => 2000.0,
        };
        self.common.gyro_scale = if gyro_scale == ImuGyroScale::Dps { scale_dps } else { scale_dps.to_radians() };

        self.common.gyro_sample_rate_hz = target_output_data_rate_hz;
    }

    pub fn calculate_acc_scale(&mut self, acc_scale: ImuAccScale, acc_sensitivity: u8) {
        let scale = match acc_sensitivity {
            ImuCommon::ACC_FULL_SCALE_2G => 2.0 / 32768.0,
            ImuCommon::ACC_FULL_SCALE_4G => 4.0 / 32768.0,
            ImuCommon::ACC_FULL_SCALE_8G => 8.0 / 32768.0,
            _ => 16.0 / 32768.0,
        };
        self.common.acc_scale = if acc_scale == ImuAccScale::G { scale } else { scale * ImuCommon::G0 };
    }

    #[inline]
    pub fn map_acc(&self, buf: [u8; 6], axis_order: ImuAxesOrder) -> Vector3df32 {
        let acc = Vector3df32::from_le_bytes_6(buf) * self.common.acc_scale - self.common.acc_offset;
        ImuAxesOrder::map_vector(axis_order, acc)
    }

    #[inline]
    pub fn map_gyro(&self, buf: [u8; 6], axis_order: ImuAxesOrder) -> Vector3df32 {
        let gyro_dps = Vector3df32::from_le_bytes_6(buf) * self.common.gyro_scale - self.common.gyro_offset;
        ImuAxesOrder::map_vector(axis_order, gyro_dps)
    }

    #[inline]
    pub fn map_acc_gyro(&self, buf: [u8; 12], axis_order: ImuAxesOrder) -> (Vector3df32, Vector3df32) {
        let acc_buf = [buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]];
        let gyro_buf = [buf[6], buf[7], buf[8], buf[9], buf[10], buf[11]];

        let acc = Vector3df32::from_le_bytes_6(acc_buf) * self.common.acc_scale - self.common.acc_offset;
        let gyro = Vector3df32::from_le_bytes_6(gyro_buf) * self.common.gyro_scale - self.common.gyro_offset;

        ImuAxesOrder::map_acc_gyro(axis_order, acc, gyro)
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

        let result = pollster::block_on(imu.init(
            8000,
            ImuCommon::GYRO_FULL_SCALE_MAX,
            ImuGyroScale::Dps,
            ImuCommon::ACC_FULL_SCALE_MAX,
            ImuAccScale::G,
        ));
        let (gyro_register_value, acc_register_value) = result.unwrap();

        assert_eq!(0, gyro_register_value);
        assert_eq!(0, acc_register_value);
        //assert_eq!(2000.0 / 32768.0, state.gyro_scale);
        //assert_eq!(16.0 / 32768.0, state.acc_scale);
        //assert_eq!(6664, state.gyro_sample_rate_hz);
        //assert_eq!(6664, state.acc_sample_rate_hz);
    }
    #[test]
    fn acc_buf() {
        let imu_bus = MockImuBus::new();
        let imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);

        // TODO: sit down and work out some useful test data for this
        let data: [u8; 6] = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        let acc = imu.map_acc(data, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32 { x: 0.0, y: 0.0, z: 0.0 }, acc);
    }
    #[test]
    fn gyro_buf() {
        let imu_bus = MockImuBus::new();
        let imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);

        // TODO: sit down and work out some useful test data for this
        let data: [u8; 6] = [0x10, 0x00, 0x00, 0x01, 0x00, 0x7f];
        let gyro_dps = imu.map_gyro(data, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32 { x: 0.976_562_5, y: 15.625, z: 1984.375 }, gyro_dps);

        let data: [u8; 6] = [0x01, 0x00, 0x80, 0x00, 0xff, 0x7f];
        let gyro_dps = imu.map_gyro(data, ImuAxesOrder::XPOS_YPOS_ZPOS);
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
        let _result = pollster::block_on(imu.bus().read_registers(0, REG_ACC_XL, &mut buf));
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
        let a = result.unwrap();
        assert_eq!(Vector3df32::new(0.5, 2.0, 1.0), a);
    }
    #[test]
    fn scale_gyro() {
        let imu_bus = MockImuBus::new();
        let mut imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);
        let gyro_scale = imu.gyro_scale() * 32768.0;
        assert_eq!(2000.0, gyro_scale);

        let mut buf = [0u8; 6];

        let gyro_dps = Vector3df32::new(125.0, 1000.0, 1750.0);
        pollster::block_on(imu.set_gyro(gyro_dps));
        let _result = pollster::block_on(imu.bus().read_registers(0, REG_GYRO_XL, &mut buf));
        assert_eq!([0x00, 0x08, 0x00, 0x40, 0x00, 0x70], buf);
        let g = imu.map_gyro(buf, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32 { x: 125.0, y: 1000.0, z: 1750.0 }, g);

        let gyro_dps = Vector3df32::new(500.0, 1000.0, 2000.0);
        pollster::block_on(imu.set_gyro(gyro_dps));
        let _result = pollster::block_on(imu.bus().read_registers(0, REG_GYRO_XL, &mut buf));
        assert_eq!([0x00, 0x20, 0x00, 0x40, 0xFF, 0x7F], buf);
        let g = imu.map_gyro(buf, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32 { x: 500.0, y: 1000.0, z: 1999.939 }, g);

        let gyro_dps = Vector3df32::new(2000.0, 4000.0, 10_000.0);
        pollster::block_on(imu.set_gyro(gyro_dps));
        let _result = pollster::block_on(imu.bus().read_registers(0, REG_GYRO_XL, &mut buf));
        assert_eq!([0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F], buf);
        let g = imu.map_gyro(buf, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32 { x: 1999.939, y: 1999.939, z: 1999.939 }, g);

        let gyro_dps = Vector3df32::new(-2000.0, -4000.0, -10_000.0);
        pollster::block_on(imu.set_gyro(gyro_dps));
        let _result = pollster::block_on(imu.bus().read_registers(0, REG_GYRO_XL, &mut buf));
        assert_eq!([0x00, 0x80, 0x00, 0x80, 0x00, 0x80], buf);
        let g = imu.map_gyro(buf, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32 { x: -2000.0, y: -2000.0, z: -2000.0 }, g);
    }
    #[test]
    fn read_gyro() {
        let imu_bus = MockImuBus::new();
        let mut imu: ImuMock<MockImuBus> = ImuMock::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);
        let gyro_scale = imu.gyro_scale() * 32768.0;
        assert_eq!(2000.0, gyro_scale);

        let gyro_dps = Vector3df32::new(125.0, 1000.0, 1750.0);
        pollster::block_on(imu.set_gyro(gyro_dps));
        let result = pollster::block_on(imu.read_gyro());
        let g = result.unwrap();
        assert_eq!(Vector3df32 { x: 125.0, y: 1000.0, z: 1750.0 }, g);

        let gyro_dps = Vector3df32::new(500.0, 1000.0, 2000.0);
        pollster::block_on(imu.set_gyro(gyro_dps));
        let result = pollster::block_on(imu.read_gyro());
        let g = result.unwrap();
        assert_eq!(Vector3df32 { x: 500.0, y: 1000.0, z: 1999.939 }, g);

        let gyro_dps = Vector3df32::new(2000.0, 4000.0, 10_000.0);
        pollster::block_on(imu.set_gyro(gyro_dps));
        let result = pollster::block_on(imu.read_gyro());
        let g = result.unwrap();
        assert_eq!(Vector3df32 { x: 1999.939, y: 1999.939, z: 1999.939 }, g);

        let gyro_dps = Vector3df32::new(-2000.0, -4000.0, -10_000.0);
        pollster::block_on(imu.set_gyro(gyro_dps));
        let result = pollster::block_on(imu.read_gyro());
        let g = result.unwrap();
        assert_eq!(Vector3df32 { x: -2000.0, y: -2000.0, z: -2000.0 }, g);
    }
}
