use vqm::{Vector3df32, Vector3di16};

use crate::{Imu, ImuAxesOrder, ImuBus, ImuCommon, ImuConfig, ImuReadingf32};

const I2C_ADDRESS: u8 = 0x68;

// **** IMU Registers and associated bitflags ****
const _REG_XG_OFFS_TC_H: u8 = 0x04;
const _REG_XG_OFFS_TC_L: u8 = 0x05;
const _REG_YG_OFFS_TC_H: u8 = 0x07;
const _REG_YG_OFFS_TC_L: u8 = 0x08;
const _REG_ZG_OFFS_TC_H: u8 = 0x0A;
const _REG_ZG_OFFS_TC_L: u8 = 0x0B;

const _REG_SELF_TEST_X_ACCEL: u8 = 0x0D;
const _REG_SELF_TEST_Y_ACCEL: u8 = 0x0E;
const _REG_SELF_TEST_Z_ACCEL: u8 = 0x0F;

const _REG_XG_OFFS_USRH: u8 = 0x13;
const _REG_XG_OFFS_USRL: u8 = 0x14;
const _REG_YG_OFFS_USRH: u8 = 0x15;
const _REG_YG_OFFS_USRL: u8 = 0x16;
const _REG_ZG_OFFS_USRH: u8 = 0x17;
const _REG_ZG_OFFS_USRL: u8 = 0x18;

const REG_SAMPLE_RATE_DIVIDER: u8 = 0x19;
const _DIVIDE_BY_1: u8 = 0x00;
const DIVIDE_BY_2: u8 = 0x01;

const REG_CONFIG: u8 = 0x1A;
const DLPF_CFG_1: u8 = 0x01;
const _DLPF_CFG_7: u8 = 0x07;

const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_ACCEL_CONFIG: u8 = 0x1C;
const REG_ACCEL_CONFIG2: u8 = 0x1D;

const REG_FIFO_ENABLE: u8 = 0x23;
const _GYRO_FIFO_EN: u8 = 0b000_01000;
const _ACC_FIFO_EN: u8 = 0b0000_0100;

const REG_INT_PIN_CFG: u8 = 0x37;
const REG_INT_ENABLE: u8 = 0x38;
const _FIFO_WM_INT_STATUS: u8 = 0x39;

const REG_ACCEL_XOUT_H: u8 = 0x3B;
const _REG_ACCEL_XOUT_L: u8 = 0x3C;
const _REG_ACCEL_YOUT_H: u8 = 0x3D;
const _REG_ACCEL_YOUT_L: u8 = 0x3E;
const _REG_ACCEL_ZOUT_H: u8 = 0x3F;
const _REG_ACCEL_ZOUT_L: u8 = 0x40;

const _REG_TEMP_OUT_H: u8 = 0x41;
const _REG_TEMP_OUT_L: u8 = 0x42;

const REG_GYRO_XOUT_H: u8 = 0x43;
const _REG_GYRO_XOUT_L: u8 = 0x44;
const _REG_GYRO_YOUT_H: u8 = 0x45;
const _REG_GYRO_YOUT_L: u8 = 0x46;
const _REG_GYRO_ZOUT_H: u8 = 0x47;
const _REG_GYRO_ZOUT_L: u8 = 0x48;

const _REG_FIFO_WM_TH1: u8 = 0x60;
const _REG_FIFO_WM_TH2: u8 = 0x61;

const _REG_SIGNAL_PATH_RESET: u8 = 0x68;
const _REG_ACCEL_INTEL_CTRL: u8 = 0x69;
const REG_USER_CTRL: u8 = 0x6A;
const REG_PWR_MGMT_1: u8 = 0x6B;
const _REG_PWR_MGMT_2: u8 = 0x6C;

const _REG_FIFO_COUNT_H: u8 = 0x72;
const _REG_FIFO_COUNT_L: u8 = 0x73;
const _REG_FIFO_R_W: u8 = 0x74;

const REG_WHO_AM_I: u8 = 0x75;

const _REG_XA_OFFSET_H: u8 = 0x77;
const _REG_XA_OFFSET_L: u8 = 0x78;
const _REG_YA_OFFSET_H: u8 = 0x7A;
const _REG_YA_OFFSET_L: u8 = 0x7B;
const _REG_ZA_OFFSET_H: u8 = 0x7D;
const _REG_ZA_OFFSET_L: u8 = 0x7E;
// **** IMU Registers and associated bitflags ****

pub struct Mpu6886<B: ImuBus> {
    pub bus: B,
    pub common: ImuCommon,
    pub config: ImuConfig,
}

impl<B: ImuBus> Imu for Mpu6886<B> {
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
        // On the Pico, I2C write_read is natively async.
        // We just delegate the call and .await the result.
        self.bus.bus_write_read(address, write, read).await
    }

    async fn read_acc(&mut self) -> Result<Vector3df32, Self::Error>
    where
        <B as ImuBus>::Error: From<<B as ImuBus>::Error>,
    {
        let mut buf = [0u8; 6];
        self.write_read(I2C_ADDRESS, &[REG_ACCEL_XOUT_H], &mut buf).await?;
        Ok(self.map_gyro_rps(buf, self.common.axis_order))
    }

    async fn read_gyro_rps(&mut self) -> Result<Vector3df32, Self::Error>
    where
        <B as ImuBus>::Error: From<<B as ImuBus>::Error>,
    {
        let mut buf = [0u8; 6];
        self.write_read(I2C_ADDRESS, &[REG_GYRO_XOUT_H], &mut buf).await?;
        //self.bus().read_registers(self.config.address, REG_GYRO_XOUT_H, &mut buf).await;
        Ok(self.map_gyro_rps(buf, self.common.axis_order))
    }

    async fn read_acc_gyro_rps(&mut self) -> Result<ImuReadingf32, Self::Error>
    where
        <B as ImuBus>::Error: From<<B as ImuBus>::Error>,
    {
        let mut buf = [0u8; 12];
        self.write_read(I2C_ADDRESS, &[REG_GYRO_XOUT_H], &mut buf).await?;
        Ok(self.map_acc_gyro_rps(buf, self.common.axis_order))
    }
}

fn delay_ms(_delay: u32) {}

impl<B: ImuBus> Mpu6886<B> {
    const DEVICE_ID: u8 = 0;

    pub fn new(bus: B, axis_order: ImuAxesOrder) -> Self {
        Self {
            bus,
            common: ImuCommon::new(axis_order),
            config: ImuConfig {
                gyro_id_msp: ImuConfig::MSP_ACC_ID_DEFAULT,
                acc_id_msp: ImuConfig::MSP_ACC_ID_DEFAULT,
                axis_order: axis_order.into(),
                device_id: Self::DEVICE_ID,
                address: I2C_ADDRESS,
                flags: 0,
            },
        }
    }

    /// # Errors
    pub async fn read_register(&mut self, reg: u8) -> Result<u8, B::Error> {
        self.bus.read_register(self.config.address, reg).await
    }

    /// # Errors
    pub async fn init(
        &mut self,
        _target_output_data_rate_hz: u32,
        _gyro_sensitivity: u8,
        _acc_sensitivity: u8,
    ) -> Result<(u32, u32), B::Error> {
        let _chip_id = self.bus.read_register(self.config.address, REG_WHO_AM_I).await;
        delay_ms(1);

        self.bus.write_register(self.config.address, REG_PWR_MGMT_1, 0).await?; // clear the power management register
        delay_ms(10);

        {
            const DEVICE_RESET: u8 = 0x01u8 << 7;
            self.bus.write_register(self.config.address, REG_PWR_MGMT_1, DEVICE_RESET).await?; // reset the device
            delay_ms(10);
        }

        {
            const CLKSEL_1: u8 = 0x01;
            self.bus.write_register(self.config.address, REG_PWR_MGMT_1, CLKSEL_1).await?; // CLKSEL must be set to 001 to achieve full gyroscope performance.
            delay_ms(10);
        }

        // Gyro scale is fixed at 2000DPS, the maximum supported.
        //enum gyro_scale_e { GFS_250DPS = 0, GFS_500DPS = 1, GFS_1000DPS = 2, GFS_2000DPS = 3 };
        {
            const GFS_2000DPS: u8 = 3;
            const GYRO_FCHOICE_B: u8 = 0x00; // enables gyro update rate and filter configuration using REG_CONFIG
            self.bus.write_register(self.config.address, REG_GYRO_CONFIG, (GFS_2000DPS << 3) | GYRO_FCHOICE_B).await?;
            self.common.gyro_scale_dps = 2000.0 / 32768.0;
            self.common.gyro_scale_rps = self.common.gyro_scale_rps.to_radians();
            delay_ms(1);
        }

        // Accelerometer scale is fixed at 8G, the maximum supported.
        //enum acc_scale_e { AFS_2G = 0, AFS_4G = 1, AFS_8G = 2, AFS_16G = 3 };
        {
            const AFS_8G: u8 = 2;
            self.bus.write_register(self.config.address, REG_ACCEL_CONFIG, AFS_8G << 3).await?;
            self.common.acc_scale = 8.0 / 32768.0;
            delay_ms(1);
        }

        {
            const ACC_FCHOICE_B: u8 = 0x00; // Filter:218.1 3-DB BW (Hz), least filtered 1kHz update variant
            self.bus.write_register(self.config.address, REG_ACCEL_CONFIG2, ACC_FCHOICE_B).await?;
            delay_ms(1);
        }

        {
            const FIFO_MODE_OVERWRITE: u8 = 0b0100_0000;
            self.bus.write_register(self.config.address, REG_CONFIG, DLPF_CFG_1 | FIFO_MODE_OVERWRITE).await?;
            delay_ms(1);
        }

        // M5Stack default divider is two, giving 500Hz output rate
        self.bus.write_register(self.config.address, REG_SAMPLE_RATE_DIVIDER, DIVIDE_BY_2).await?;
        delay_ms(1);
        self.common.gyro_sample_rate_hz = 500;
        self.common.acc_sample_rate_hz = 500;

        self.bus.write_register(self.config.address, REG_FIFO_ENABLE, 0x00).await?; // FIFO disabled
        delay_ms(1);

        // M5 Unified settings
        //self.bus.write_register(self.config.address, REG_INT_PIN_CFG, 0b1100_0000).await; // Active low, open drain 50us pulse width, clear on read
        self.bus.write_register(self.config.address, REG_INT_PIN_CFG, 0x22).await?;
        delay_ms(1);

        {
            const DATA_RDY_INT_EN: u8 = 0x01;
            self.bus.write_register(self.config.address, REG_INT_ENABLE, DATA_RDY_INT_EN).await?; // data ready interrupt enabled
            delay_ms(10);
        }

        self.bus.write_register(self.config.address, REG_USER_CTRL, 0x00).await?;

        //bus_semaphore_give(_bus_mutex);
        delay_ms(1);

        // return the gyro sample rate actually set
        Ok((0, 0))
    }

    // NOTE: Not sure if this is the right place to put this code, but it "wanted" to go here.
    // It just kept floating upward until it reached this point.
    // And it makes it easily accessible from test code.
    pub fn map_acc(&mut self, buf: [u8; 6], axis_order: ImuAxesOrder) -> Vector3df32 {
        let acc16 = Vector3di16 {
            x: i16::from_be_bytes([buf[0], buf[1]]),
            y: i16::from_be_bytes([buf[2], buf[3]]),
            z: i16::from_be_bytes([buf[4], buf[5]]),
        };
        let acc = Vector3df32::from(acc16) * self.common.acc_scale - self.common.acc_offset;
        ImuAxesOrder::map_vector(axis_order, &acc)
    }

    pub fn map_gyro_rps(&mut self, buf: [u8; 6], axis_order: ImuAxesOrder) -> Vector3df32 {
        let gyro16 = Vector3di16 {
            x: i16::from_be_bytes([buf[0], buf[1]]),
            y: i16::from_be_bytes([buf[2], buf[3]]),
            z: i16::from_be_bytes([buf[4], buf[5]]),
        };
        let gyro_rps = Vector3df32::from(gyro16) * self.common.gyro_scale_rps - self.common.gyro_offset;
        ImuAxesOrder::map_vector(axis_order, &gyro_rps)
    }

    pub fn map_acc_gyro_rps(&mut self, buf: [u8; 12], axis_order: ImuAxesOrder) -> ImuReadingf32 {
        let gyro16 = Vector3di16 {
            x: i16::from_be_bytes([buf[0], buf[1]]),
            y: i16::from_be_bytes([buf[2], buf[3]]),
            z: i16::from_be_bytes([buf[4], buf[5]]),
        };
        let acc16 = Vector3di16 {
            x: i16::from_be_bytes([buf[6], buf[7]]),
            y: i16::from_be_bytes([buf[8], buf[9]]),
            z: i16::from_be_bytes([buf[10], buf[11]]),
        };

        let imu_reading = ImuReadingf32 {
            acc: Vector3df32::from(acc16) * self.common.acc_scale - self.common.acc_offset,
            gyro_rps: Vector3df32::from(gyro16) * self.common.gyro_scale_rps - self.common.gyro_offset,
        };

        ImuAxesOrder::map_reading(axis_order, &imu_reading)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ImuAxesOrder, MockImuBus};

    fn is_normal<T: Sized + Send + Sync + Unpin>() {}

    #[test]
    fn normal_types() {
        is_normal::<Mpu6886<MockImuBus>>();
    }
    #[test]
    fn imu_init() {
        let imu_bus = MockImuBus::new();
        let mut imu: Mpu6886<MockImuBus> = Mpu6886::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);

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
    fn map_acc() {
        let imu_bus = MockImuBus::new();
        let mut imu: Mpu6886<MockImuBus> = Mpu6886::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);

        // TODO: sit down and work out some useful test data for this
        let data: [u8; 6] = [0, 0, 0, 0, 0, 0];
        let acc = imu.map_acc(data, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32 { x: 0.0, y: 0.0, z: 0.0 }, acc);
    }
}
