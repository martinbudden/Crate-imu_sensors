#![allow(unused)]

use vector_quaternion_matrix::{Vector3df32, Vector3di16};

use crate::{I2cInterface, Imu, ImuAxesOrder, ImuBus, ImuCommon, ImuConfig, ImuReadingf32, MockImuBus};

use embedded_hal_async::i2c::{self, I2c};

// **** IMU Registers and associated bitflags ****
const REG_SAMPLE_RATE_DIVIDER: u8 = 0x19;
const REG_CONFIG: u8 = 0x1A;
const DLPF_CFG_260_HZ: u8 = 0b00000000; // FS = 8kHz only
const DLPF_CFG_184_HZ: u8 = 0b00000001;
const DLPF_CFG_94_HZ: u8 = 0b00000010;
const DLPF_CFG_44_HZ: u8 = 0b00000011;
const DLPF_CFG_21_HZ: u8 = 0b00000100;
const DLPF_CFG_10_HZ: u8 = 0b00000101;
const DLPF_CFG_5_HZ: u8 = 0b00000110;

const REG_GYRO_CONFIG: u8 = 0x1B;
const GYRO_RANGE_250_DPS: u8 = 0b00000000;
const GYRO_RANGE_500_DPS: u8 = 0b00001000;
const GYRO_RANGE_1000_DPS: u8 = 0b00010000;
const GYRO_RANGE_2000_DPS: u8 = 0b00011000;

const REG_ACCEL_CONFIG: u8 = 0x1C;
const ACCEL_RANGE_2G: u8 = 0b0000_0000;
const ACCEL_RANGE_4G: u8 = 0b0000_1000;
const ACCEL_RANGE_8G: u8 = 0b0001_0000;
const ACCEL_RANGE_16G: u8 = 0b0001_1000;

const REG_INT_PIN_CONFIG: u8 = 0x37;
const INT_LEVEL_ACTIVE_LOW: u8 = 0b10000000;
const INT_LEVEL_ACTIVE_HIGH: u8 = 0;
const INT_OPEN_DRAIN: u8 = 0b01000000;
const INT_PUSH_PULL: u8 = 0;
const INT_ENABLE_LATCHED: u8 = 0b00100000;
const INT_ENABLE_PULSE: u8 = 0;
const INT_CLEAR_READ_ANY: u8 = 0b00010000; // cleared on any read
const INT_CLEAR_READ_STATUS: u8 = 0; // cleared only by reading REG_INT_STATUS
const FSYNCH_ACTIVE_LOW: u8 = 0b00001000; // interrupt on FSYNCH pin active high
const FSYNCH_ACTIVE_HIGH: u8 = 0;
const FSYNCH_INT_ENABLE: u8 = 0b00000100; // enable interrupt on FSYNCH pin
const FSYNCH_INT_DISABLE: u8 = 0;

const REG_INT_ENABLE: u8 = 0x38;
const DATA_READY_ENABLE: u8 = 0b00000001;

const REG_INT_STATUS: u8 = 0x3A;

const REG_ACCEL_XOUT_H: u8 = 0x3B;
const REG_ACCEL_XOUT_L: u8 = 0x3C;
const REG_ACCEL_YOUT_H: u8 = 0x3D;
const REG_ACCEL_YOUT_L: u8 = 0x3E;
const REG_ACCEL_ZOUT_H: u8 = 0x3F;
const REG_ACCEL_ZOUT_L: u8 = 0x40;

const REG_TEMP_OUT_H: u8 = 0x41;
const REG_TEMP_OUT_L: u8 = 0x42;

const REG_GYRO_XOUT_H: u8 = 0x43;
const REG_GYRO_XOUT_L: u8 = 0x44;
const REG_GYRO_YOUT_H: u8 = 0x45;
const REG_GYRO_YOUT_L: u8 = 0x46;
const REG_GYRO_ZOUT_H: u8 = 0x47;
const REG_GYRO_ZOUT_L: u8 = 0x48;

const REG_USER_CTRL: u8 = 0x6A;
const I2C_INTERFACE_DISABLED: u8 = 0b0001_0000;

const REG_PWR_MGMT_1: u8 = 0x6B;
const CLKSEL_INTERNAL_8_MHZ: u8 = 0x00;
const CLKSEL_PLL_X_AXIS_GYRO: u8 = 0x01;
const CLKSEL_PLL_Y_AXIS_GYRO: u8 = 0x02;
const CLKSEL_PLL_Z_AXIS_GYRO: u8 = 0x03;
const CLKSEL_EXTERNAL_32768_HZ: u8 = 0x04;
const CLKSEL_EXTERNAL_19P2_MHZ: u8 = 0x05;

const REG_PWR_MGMT_2: u8 = 0x6C;

const REG_WHO_AM_I: u8 = 0x75;
// **** IMU Registers and associated bitflags ****

/// MPU6000 is SPI variant of MPU6050
/// MPU6000 and MPU6050 are Big Endian
pub struct Mpu6050<B: ImuBus> {
    pub bus: B,
    pub common: ImuCommon,
    pub config: ImuConfig,
}

fn delay_ms(_delay: u32) {}

impl<B: ImuBus> Mpu6050<B> {
    const DEVICE_ID: u8 = 0x68;

    pub fn new(bus: B, axis_order: ImuAxesOrder) -> Self {
        Self {
            bus,
            common: ImuCommon::default(),
            config: ImuConfig {
                gyro_id_msp: ImuConfig::MSP_GYRO_ID_MPU6050,
                acc_id_msp: ImuConfig::MSP_ACC_ID_MPU6050,
                axis_order,
                device_id: Self::DEVICE_ID,
                flags: 0,
            },
        }
    }

    async fn read_register(&mut self, reg: u8) -> Result<u8, B::Error> {
        self.bus.read_register(reg).await
    }

    pub async fn init(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: u8,
        acc_sensitivity: u8,
    ) -> Result<(u8, u8), B::Error> {
        let gyro_sample_rate_divider = if target_output_data_rate_hz == 0 || target_output_data_rate_hz > 4000 {
            0 // div by 1, ie 8kHz
        } else if target_output_data_rate_hz > 2000 {
            1 // div by 2
        } else if target_output_data_rate_hz > 1000 {
            3 // div by 4
        } else if target_output_data_rate_hz > 500 {
            7 // div by 8
        } else if target_output_data_rate_hz > 250 {
            15
        } else {
            31
        };

        // report the value that was actually set
        self.common.gyro_sample_rate_hz = if gyro_sample_rate_divider == 0 {
            8000
        } else if gyro_sample_rate_divider == 1 {
            4000
        } else if gyro_sample_rate_divider == 3 {
            2000
        } else if gyro_sample_rate_divider == 7 {
            1000
        } else if gyro_sample_rate_divider == 15 {
            500
        } else {
            125
        };
        let gyro_range = match gyro_sensitivity {
            ImuCommon::GYRO_FULL_SCALE_125_DPS | ImuCommon::GYRO_FULL_SCALE_250_DPS => {
                self.common.gyro_scale_dps = 250.0 / 32768.0;
                GYRO_RANGE_250_DPS
            }
            ImuCommon::GYRO_FULL_SCALE_500_DPS => {
                self.common.gyro_scale_dps = 500.0 / 32768.0;
                GYRO_RANGE_500_DPS
            }
            ImuCommon::GYRO_FULL_SCALE_1000_DPS => {
                self.common.gyro_scale_dps = 1000.0 / 32768.0;
                GYRO_RANGE_1000_DPS
            }
            _ => {
                // default includes ImuCommon::GYRO_FULL_SCALE_2000_DPS
                self.common.gyro_scale_dps = 2000.0 / 32768.0;
                GYRO_RANGE_2000_DPS
            }
        };

        self.common.acc_sample_rate_hz = 1000;
        let acc_range = match acc_sensitivity {
            ImuCommon::ACC_FULL_SCALE_2G => {
                self.common.acc_scale = 2.0 / 32768.0;
                ACCEL_RANGE_2G
            }
            ImuCommon::ACC_FULL_SCALE_4G => {
                self.common.acc_scale = 4.0 / 32768.0;
                ACCEL_RANGE_4G
            }
            ImuCommon::ACC_FULL_SCALE_8G => {
                self.common.acc_scale = 8.0 / 32768.0;
                ACCEL_RANGE_8G
            }
            _ => {
                self.common.acc_scale = 16.0 / 32768.0;
                ACCEL_RANGE_16G
            }
        };
        Ok((gyro_range, acc_range))
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

    pub fn map_acc_gyro_rps(&mut self, buf: [u8; 14], axis_order: ImuAxesOrder) -> ImuReadingf32 {
        let acc16 = Vector3di16 {
            x: i16::from_be_bytes([buf[0], buf[1]]),
            y: i16::from_be_bytes([buf[2], buf[3]]),
            z: i16::from_be_bytes([buf[4], buf[5]]),
        };
        let _temperature = i16::from_be_bytes([buf[6], buf[7]]);
        let gyro16 = Vector3di16 {
            x: i16::from_be_bytes([buf[8], buf[9]]),
            y: i16::from_be_bytes([buf[10], buf[11]]),
            z: i16::from_be_bytes([buf[12], buf[13]]),
        };
        let imu_reading = ImuReadingf32 {
            acc: Vector3df32::from(acc16) * self.common.acc_scale - self.common.acc_offset,
            gyro_rps: Vector3df32::from(gyro16) * self.common.gyro_scale_rps - self.common.gyro_offset,
        };
        // NOTE: this begs to be chained, but perhaps for another day
        ImuAxesOrder::map_reading(axis_order, &imu_reading)
    }
    /*pub async fn init(&mut self) -> Result<(), Error> {
        let id = self.common.bus.read_register(Self::REG_WHO_AM_I).await?;
        if id != Self::DEVICE_ID {
            return Err(Error::WrongDevice)
        }
        Ok(())
    }
    pub fn set_acc_scale(&mut self, scale: AccScale) {
        self.common.bus.write_register(Self::REG_ACCEL_CONFIG, scale as u8).ok();
    }*/
}

impl<B: ImuBus> Imu for Mpu6050<B> {
    type Bus = B;
    //type Error = I2C::Error;

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

    //async fn read_acc(&mut self) -> impl core::future::Future<Output = Result<(),Self::Error> > {
    fn read_acc(&mut self) -> Vector3df32 {
        let buf = [0u8; 6];
        //self.bus().read_registers(Self::REG_ACCEL_XOUT_H, &mut buf).await;
        //let addr =  0x68;
        //let reg =  REG_ACCEL_XOUT_H;
        //self.bus().blocking_write_read(addr, &[reg], &mut buf);
        //self.bus.blocking_write_read(addr, &[reg], data);
        self.map_acc(buf, self.config.axis_order)
    }

    //async fn read_gyro_rps(&mut self) -> impl core::future::Future<Output = Result<(),Self::Error>> {
    fn read_gyro_rps(&mut self) -> Vector3df32 {
        let buf = [0u8; 6];
        //self.bus().read_registers(REG_GYRO_XOUT_H, &mut buf).await;
        self.map_gyro_rps(buf, self.config.axis_order)
    }

    //fn read_acc_gyro_rps(&mut self) -> impl core::future::Future<Output = Result<(),Self::Error>> {
    fn read_acc_gyro_rps(&mut self) -> ImuReadingf32 {
        let buf = [0u8; 14];
        //self.bus().read_registers(REG_ACCEL_XOUT_H, &mut buf).await;
        self.map_acc_gyro_rps(buf, self.config.axis_order)
    }
}

/*
impl<I2C, E> ImuBus for I2cInterface<I2C>
where
    I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E>,
{
    type Error = E;

    async fn read_register(&mut self, reg: u8) -> Result<u8, Self::Error> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(self.address, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    async fn read_registers(&mut self, reg: u8, data: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, &[reg], data).await
    }

    async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.i2c.write(self.address, &[reg, value]).await
    }

}
*/

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{ImuAxesOrder, MockImuBus};

    fn is_normal<T: Sized + Send + Sync + Unpin>() {}

    #[test]
    fn normal_types() {
        is_normal::<Mpu6050<MockImuBus>>();
    }
    #[test]
    fn imu_state_init_mpu6050() {
        let mut imu_bus = MockImuBus::new();
        let mut imu: Mpu6050<MockImuBus> = Mpu6050::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);

        let result = pollster::block_on(imu.init(8000, ImuCommon::GYRO_FULL_SCALE_MAX, ImuCommon::ACC_FULL_SCALE_MAX));
        let (gyro_register_value, acc_register_value) = result.unwrap();

        assert_eq!(24, gyro_register_value);
        assert_eq!(24, acc_register_value);
        assert_eq!(2000.0 / 32768.0, imu.common.gyro_scale_dps);
        assert_eq!(16.0 / 32768.0, imu.common.acc_scale);
        assert_eq!(8000, imu.common.gyro_sample_rate_hz);
        assert_eq!(1000, imu.common.acc_sample_rate_hz);
    }
    #[test]
    fn map_mpu6050_acc() {
        let mut imu_bus = MockImuBus::new();
        let mut imu: Mpu6050<MockImuBus> = Mpu6050::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);

        let _result = pollster::block_on(imu.init(8000, ImuCommon::GYRO_FULL_SCALE_MAX, ImuCommon::ACC_FULL_SCALE_MAX));

        // TODO: sit down and work out some useful test data for this
        //let data: [u8; 6] = [0, 0, 0, 0, 0, 0];
        //let acc = common.map_mpu6050_acc(data, ImuAxesOrder::XPOS_YPOS_ZPOS);
        //assert_eq!(Vector3df32 { x: 0.0, y: 0.0, z: 0.0 }, acc);
    }
}
