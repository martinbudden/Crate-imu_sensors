use embassy_time::{Duration, Timer};
use vqm::Vector3df32;

use crate::{
    Imu, ImuAxesOrder, ImuBus, ImuCommon, ImuConfig,
    imu::{ImuAccScale, ImuGyroScale},
};

const I2C_ADDRESS: u8 = 0x68;
const _I2C_ADDRESS_ALTERNATIVE: u8 = 0x69;

// **** IMU Registers and associated bitflags ****
const _REG_SAMPLE_RATE_DIVIDER: u8 = 0x19;
const _REG_CONFIG: u8 = 0x1A;
const _DLPF_CFG_260_HZ: u8 = 0b0000_0000; // FS = 8kHz only
const _DLPF_CFG_184_HZ: u8 = 0b0000_0001;
const _DLPF_CFG_94_HZ: u8 = 0b0000_0010;
const _DLPF_CFG_44_HZ: u8 = 0b0000_0011;
const _DLPF_CFG_21_HZ: u8 = 0b0000_0100;
const _DLPF_CFG_10_HZ: u8 = 0b0000_0101;
const _DLPF_CFG_5_HZ: u8 = 0b0000_0110;

const _REG_GYRO_CONFIG: u8 = 0x1B;
const GYRO_RANGE_250_DPS: u8 = 0b0000_0000;
const GYRO_RANGE_500_DPS: u8 = 0b0000_1000;
const GYRO_RANGE_1000_DPS: u8 = 0b000_10000;
const GYRO_RANGE_2000_DPS: u8 = 0b0001_1000;

const _REG_ACCEL_CONFIG: u8 = 0x1C;
const ACCEL_RANGE_2G: u8 = 0b0000_0000;
const ACCEL_RANGE_4G: u8 = 0b0000_1000;
const ACCEL_RANGE_8G: u8 = 0b0001_0000;
const ACCEL_RANGE_16G: u8 = 0b0001_1000;

const REG_INT_PIN_CONFIG: u8 = 0x37;
const _INT_LEVEL_ACTIVE_LOW: u8 = 0b1000_0000;
const INT_LEVEL_ACTIVE_HIGH: u8 = 0;
const _INT_OPEN_DRAIN: u8 = 0b0100_0000;
const INT_PUSH_PULL: u8 = 0;
const _INT_ENABLE_LATCHED: u8 = 0b0010_0000;
const INT_ENABLE_PULSE: u8 = 0;
const INT_CLEAR_READ_ANY: u8 = 0b0001_0000; // cleared on any read
const _INT_CLEAR_READ_STATUS: u8 = 0; // cleared only by reading REG_INT_STATUS
const _FSYNCH_ACTIVE_LOW: u8 = 0b0000_1000; // interrupt on FSYNCH pin active high
const _FSYNCH_ACTIVE_HIGH: u8 = 0;
const _FSYNCH_INT_ENABLE: u8 = 0b0000_0100; // enable interrupt on FSYNCH pin
const FSYNCH_INT_DISABLE: u8 = 0;

const REG_INT_ENABLE: u8 = 0x38;
const DATA_READY_ENABLE: u8 = 0b0000_0001;

const _REG_INT_STATUS: u8 = 0x3A;

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

const _REG_USER_CTRL: u8 = 0x6A;
const _I2C_INTERFACE_DISABLED: u8 = 0b0001_0000;

const REG_PWR_MGMT_1: u8 = 0x6B;
const _CLKSEL_INTERNAL_8_MHZ: u8 = 0x00;
const _CLKSEL_PLL_X_AXIS_GYRO: u8 = 0x01;
const _CLKSEL_PLL_Y_AXIS_GYRO: u8 = 0x02;
const CLKSEL_PLL_Z_AXIS_GYRO: u8 = 0x03;
const _CLKSEL_EXTERNAL_32768_HZ: u8 = 0x04;
const _CLKSEL_EXTERNAL_19P2_MHZ: u8 = 0x05;

const REG_PWR_MGMT_2: u8 = 0x6C;

const _REG_WHO_AM_I: u8 = 0x75;
// **** IMU Registers and associated bitflags ****

/// MPU6000 is SPI variant of MPU6050
/// MPU6000 and MPU6050 are Big Endian.
///
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Mpu6050<B: ImuBus> {
    pub bus: B,
    pub common: ImuCommon,
    pub config: ImuConfig,
}

impl<B: ImuBus> Imu for Mpu6050<B> {
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
        // On the Pico, I2C write_read is natively async.
        // We just delegate the call and .await the result.
        self.bus.bus_write_read(I2C_ADDRESS, write, read).await
    }

    async fn read_acc(&mut self) -> Result<Vector3df32, Self::Error>
    where
        <B as ImuBus>::Error: From<<B as ImuBus>::Error>,
    {
        let mut buf = [0u8; 6];
        self.write_read(&[REG_ACCEL_XOUT_H], &mut buf).await?;
        Ok(self.map_acc(buf))
    }

    async fn read_gyro(&mut self) -> Result<Vector3df32, Self::Error>
    where
        <B as ImuBus>::Error: From<<B as ImuBus>::Error>,
    {
        let mut buf = [0u8; 6];
        self.write_read(&[REG_GYRO_XOUT_H], &mut buf).await?;
        //self.bus().read_registers(self.config.address, REG_GYRO_XOUT_H, &mut buf).await;
        Ok(self.map_gyro(buf))
    }

    async fn read_acc_gyro(&mut self) -> Result<(Vector3df32, Vector3df32), Self::Error>
    where
        <B as ImuBus>::Error: From<<B as ImuBus>::Error>,
    {
        let mut buf = [0u8; 14];
        self.write_read(&[REG_GYRO_XOUT_H], &mut buf).await?;
        let acc_buf = [buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]];
        let _temperature = i16::from_be_bytes([buf[6], buf[7]]);
        let gyro_buf = [buf[8], buf[9], buf[10], buf[11], buf[12], buf[13]];

        let acc = Vector3df32::from_be_bytes_6(acc_buf) * self.common.acc_scale - self.common.acc_offset;
        let gyro = Vector3df32::from_be_bytes_6(gyro_buf) * self.common.gyro_scale - self.common.gyro_offset;

        Ok(ImuAxesOrder::map_acc_gyro(self.common.axis_order, acc, gyro))
    }

    #[inline]
    fn map_acc(&self, buf: [u8; 6]) -> Vector3df32 {
        let acc = Vector3df32::from_be_bytes_6(buf) * self.common.acc_scale - self.common.acc_offset;
        ImuAxesOrder::map_vector(self.common.axis_order, acc)
    }

    #[inline]
    fn map_gyro(&self, buf: [u8; 6]) -> Vector3df32 {
        let gyro = Vector3df32::from_be_bytes_6(buf) * self.common.gyro_scale - self.common.gyro_offset;
        ImuAxesOrder::map_vector(self.common.axis_order, gyro)
    }

    #[inline]
    fn map_acc_gyro(&self, _buf: [u8; 12]) -> (Vector3df32, Vector3df32) {
        (Vector3df32::default(), Vector3df32::default())
    }
    #[inline]
    fn map_acc_gyro_slice(&self, slice: &[u8]) -> (Vector3df32, Vector3df32) {
        let acc_slice = &slice[0..6];
        // let temperature_slice = &slice[6..8];
        let gyro_slice = &slice[8..14];

        let acc = Vector3df32::from_be_slice_6(acc_slice) * self.common.acc_scale - self.common.acc_offset;
        let gyro = Vector3df32::from_be_slice_6(gyro_slice) * self.common.gyro_scale - self.common.gyro_offset;

        ImuAxesOrder::map_acc_gyro(self.common.axis_order, acc, gyro)
    }
}

async fn delay_ms(delay: u32) {
    Timer::after(Duration::from_millis(delay.into())).await;
}

impl<B: ImuBus> Mpu6050<B> {
    const DEVICE_ID: u8 = 0x68;

    /// Constructor.
    pub fn new(bus: B, axis_order: ImuAxesOrder) -> Self {
        Self {
            bus,
            common: ImuCommon::new(axis_order),
            config: ImuConfig {
                gyro_id_msp: ImuConfig::MSP_GYRO_ID_MPU6050,
                acc_id_msp: ImuConfig::MSP_ACC_ID_MPU6050,
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
        target_output_data_rate_hz: u32,
        gyro_sensitivity: u8,
        gyro_scale: ImuGyroScale,
        acc_sensitivity: u8,
        acc_scale: ImuAccScale,
    ) -> Result<(u32, u32), B::Error> {
        // clock source: PLL with Z axis gyro reference
        self.bus.write_register(self.config.address, REG_PWR_MGMT_1, CLKSEL_PLL_Z_AXIS_GYRO).await?;
        delay_ms(15).await;
        self.bus.write_register(self.config.address, REG_PWR_MGMT_2, 0x00).await?;
        delay_ms(15).await;

        /*let id= self.common.bus.read_register(Self::REG_WHO_AM_I).await?;
        if id != Self::DEVICE_ID {
            return Err(Error::WrongDevice)
        }
        Ok(())*/

        // Configure interrupts
        self.bus
            .write_register(
                self.config.address,
                REG_INT_PIN_CONFIG,
                INT_LEVEL_ACTIVE_HIGH | INT_PUSH_PULL | INT_ENABLE_PULSE | INT_CLEAR_READ_ANY | FSYNCH_INT_DISABLE,
            )
            .await?;
        delay_ms(15).await;
        self.bus.write_register(self.config.address, REG_INT_ENABLE, DATA_READY_ENABLE).await?;
        delay_ms(15).await;

        // TODO: write _gyro_sample_rate_divider to appropriate register.
        let _gyro_register_value =
            self.calculate_gyro_scale_and_odr(gyro_sensitivity, gyro_scale, target_output_data_rate_hz);
        //self.bus.write_register(self.config.address, REG_, gyro_register_value).await?;

        let _acc_register_value =
            self.calculate_acc_scale_and_odr(acc_sensitivity, acc_scale, target_output_data_rate_hz);
        //self.bus.write_register(self.config.address, REG_, acc_register_value).await?;

        // return the gyro and acc sample rates actually set
        Ok((self.common.gyro_sample_rate_hz, self.common.acc_sample_rate_hz))
    }

    pub fn calculate_gyro_scale_and_odr(
        &mut self,
        gyro_sensitivity: u8,
        gyro_scale: ImuGyroScale,
        target_output_data_rate_hz: u32,
    ) -> u8 {
        let (scale_dps, gyro_register_value) = match gyro_sensitivity {
            // full scale 125 not supported so use 250 instead.
            ImuCommon::GYRO_FULL_SCALE_125_DPS | ImuCommon::GYRO_FULL_SCALE_250_DPS => {
                (250.0 / 32768.0, GYRO_RANGE_250_DPS)
            }
            ImuCommon::GYRO_FULL_SCALE_500_DPS => (500.0 / 32768.0, GYRO_RANGE_500_DPS),
            ImuCommon::GYRO_FULL_SCALE_1000_DPS => (1000.0 / 32768.0, GYRO_RANGE_1000_DPS),
            _ => (2000.0 / 32768.0, GYRO_RANGE_2000_DPS),
        };
        self.common.gyro_scale = if gyro_scale == ImuGyroScale::Dps { scale_dps } else { scale_dps.to_radians() };

        let (gyro_sample_rate_hz, gyro_odr) = match target_output_data_rate_hz {
            2001..=4000 => (4000, 1), // div by 2
            1001..=2000 => (2000, 3), // div by 4
            501..=1000 => (1000, 7),  // div by 8
            251..=500 => (500, 15),
            1..=250 => (250, 31), // assuming 8000 / (31 + 1) = 250
            _ => (8000, 0),       // div by 1
        };
        self.common.gyro_sample_rate_hz = gyro_sample_rate_hz;

        gyro_register_value | gyro_odr
    }

    pub fn calculate_acc_scale_and_odr(
        &mut self,
        acc_sensitivity: u8,
        acc_scale: ImuAccScale,
        target_output_data_rate_hz: u32,
    ) -> u8 {
        self.common.acc_sample_rate_hz = 1000;
        let (scale, acc_register_value) = match acc_sensitivity {
            ImuCommon::ACC_FULL_SCALE_2G => (2.0 / 32768.0, ACCEL_RANGE_2G),
            ImuCommon::ACC_FULL_SCALE_4G => (4.0 / 32768.0, ACCEL_RANGE_4G),
            ImuCommon::ACC_FULL_SCALE_8G => (8.0 / 32768.0, ACCEL_RANGE_8G),
            _ => (16.0 / 32768.0, ACCEL_RANGE_16G),
        };
        self.common.acc_scale = if acc_scale == ImuAccScale::G { scale } else { scale * ImuCommon::G0 };

        // TODO: acc sample rate tops out at 1000, need to check the divisors
        let (acc_sample_rate_hz, acc_odr) = match target_output_data_rate_hz {
            251..=500 => (500, 0),
            1..=250 => (250, 0),
            _ => (1000, 0),
        };
        self.common.acc_sample_rate_hz = acc_sample_rate_hz;

        acc_register_value | acc_odr
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
    // we can do float comparisons because all floats have been converted from i16s, and so can be represented exactly.
    #![allow(clippy::float_cmp)]
    use super::*;
    use crate::{ImuAxesOrder, MockImuBus};

    fn is_normal<T: Sized + Send + Sync + Unpin>() {}

    #[test]
    fn normal_types() {
        is_normal::<Mpu6050<MockImuBus>>();
    }
    #[test]
    fn imu_init() {
        let imu_bus = MockImuBus::new();
        let mut imu: Mpu6050<MockImuBus> = Mpu6050::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);

        let result = pollster::block_on(imu.init(
            8000,
            ImuCommon::GYRO_FULL_SCALE_MAX,
            ImuGyroScale::Dps,
            ImuCommon::ACC_FULL_SCALE_MAX,
            ImuAccScale::G,
        ));
        let (gyro_sample_rate_hz, acc_sample_rate_hz) = result.unwrap();
        /*let Ok((gyro_sample_rate_hz, acc_sample_rate_hz)) = result else {
            panic!("Result unwrap error");
        };*/
        assert_eq!(8000, gyro_sample_rate_hz);
        assert_eq!(1000, acc_sample_rate_hz);

        assert_eq!(2000.0 / 32768.0, imu.common.gyro_scale);
        assert_eq!(16.0 / 32768.0, imu.common.acc_scale);
        assert_eq!(8000, imu.common.gyro_sample_rate_hz);
        assert_eq!(1000, imu.common.acc_sample_rate_hz);
    }
    #[test]
    fn map_acc() {
        let imu_bus = MockImuBus::new();
        let mut imu: Mpu6050<MockImuBus> = Mpu6050::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);

        let _result = pollster::block_on(imu.init(
            8000,
            ImuCommon::GYRO_FULL_SCALE_MAX,
            ImuGyroScale::Dps,
            ImuCommon::ACC_FULL_SCALE_MAX,
            ImuAccScale::G,
        ));

        // TODO: sit down and work out some useful test data for this
        let data: [u8; 6] = [0, 0, 0, 0, 0, 0];
        let acc = imu.map_acc(data);
        assert_eq!(Vector3df32 { x: 0.0, y: 0.0, z: 0.0 }, acc);
    }
}
