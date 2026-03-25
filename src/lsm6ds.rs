#![allow(unused)]

use vector_quaternion_matrix::{Vector3df32, Vector3di16};

use crate::{ImuAxesOrder, ImuBus, ImuCommon, ImuConfig, ImuReadingf32};

use cfg_if::cfg_if;

// **** IMU Registers and associated bitflags ****
const _REG_RESERVED_00: u8 = 0x00;
const _REG_FUNC_CFG_ACCESS: u8 = 0x01;
const _REG_RESERVED_03: u8 = 0x03;

cfg_if! {
if #[cfg(feature = "USE_IMU_LSMDS63TR_C")] {
const REG_RESERVED_02: u8 = 0x02;
const REG_SENSOR_SYNC_TIME_FRAME: u8 = 0x04;
const REG_SENSOR_SYNC_RES_RATIO: u8 = 0x05;
const REG_FIFO_CTRL1: u8 = 0x06;
const REG_FIFO_CTRL2: u8 = 0x07;
const REG_FIFO_CTRL3: u8 = 0x08;
const REG_FIFO_CTRL4: u8 = 0x09;
const REG_FIFO_CTRL5: u8 = 0x0A;
const REG_DRDY_PULSE_CFG_G: u8 = 0x0B;
const REG_RESERVED_0C: u8 = 0x0C;
const REG_MASTER_CONFIG: u8 = 0x1A;

} else if #[cfg(feature = "USE_IMU_ISM330DHCX")] {

const REG_PIN_CTRL: u8 = 0x02;
const REG_RESERVED_04: u8 = 0x04;
const REG_RESERVED_05: u8 = 0x05;
const REG_RESERVED_06: u8 = 0x06;
const REG_FIFO_CTRL1: u8 = 0x07;
const REG_FIFO_CTRL2: u8 = 0x08;
const REG_FIFO_CTRL3: u8 = 0x09;
const REG_FIFO_CTRL4: u8 = 0x0A;
const REG_COUNTER_BDR_REG1: u8 = 0x0B;
const REG_COUNTER_BDR_REG2: u8 = 0x0C;
const REG_ALL_INT_SRC: u8 = 0x1A;

} else if #[cfg(feature = "USE_IMU_LSM6DSOX")] {
const REG_PIN_CTRL: u8 = 0x02;
const REG_S4S_TPH_L: u8 = 0x04;
const REG_S4S_TPH_H: u8 = 0x05;
const REG_S4S_RR: u8 = 0x06;
const REG_FIFO_CTRL1: u8 = 0x07;
const REG_FIFO_CTRL2: u8 = 0x08;
const REG_FIFO_CTRL3: u8 = 0x09;
const REG_FIFO_CTRL4: u8 = 0x0A;
const REG_COUNTER_BDR_REG1: u8 = 0x0B;
const REG_COUNTER_BDR_REG2: u8 = 0x0C;
const REG_ALL_INT_SRC: u8 = 0x1A;
}
}

const REG_DATA_READY_PULSE_CONFIG: u8 = 0x0B;
const DATA_READY_PULSED: u8 = 0b10000000;
const REG_INT1_CTRL: u8 = 0x0D;
const INT1_DRDY_G: u8 = 0b00000010;
const REG_INT2_CTRL: u8 = 0x0E;
const INT2_DRDY_G: u8 = 0b00000010;
const _REG_WHO_AM_I: u8 = 0x0F;
const _REG_WHO_AM_I_RESPONSE_LSM6DS3TR_C: u8 = 0x6A;
const _REG_WHO_AM_I_RESPONSE_ISM330DHCX: u8 = 0x6B;
const _REG_WHO_AM_I_RESPONSE_LSM6DSOX: u8 = 0x6C;
const _REG_CTRL1_XL: u8 = 0x10;
const ACC_RANGE_2G: u8 = 0b0000;
const ACC_RANGE_4G: u8 = 0b1000;
const ACC_RANGE_8G: u8 = 0b1100;
const ACC_RANGE_16G: u8 = 0b0100;
const ACC_ODR_12P5_HZ: u8 = 0b00010000;
const ACC_ODR_26_HZ: u8 = 0b00100000;
const ACC_ODR_52_HZ: u8 = 0b00110000;
const ACC_ODR_104_HZ: u8 = 0b01000000;
const ACC_ODR_208_HZ: u8 = 0b01010000; // corrected: was 0b010100000 (9 bits)
const ACC_ODR_416_HZ: u8 = 0b01100000;
const ACC_ODR_833_HZ: u8 = 0b01110000;
const ACC_ODR_1666_HZ: u8 = 0b10000000;
const ACC_ODR_3332_HZ: u8 = 0b10010000;
const ACC_ODR_6664_HZ: u8 = 0b10100000;
const REG_CTRL2_G: u8 = 0x11;
const GYRO_RANGE_125_DPS: u8 = 0b0010;
const _GYRO_RANGE_245_DPS: u8 = 0b0000; // LSM6DS3TR_C
const _GYRO_RANGE_250_DPS: u8 = 0b0000; // ISM330DHCX, LSM6DSOX
const GYRO_RANGE_500_DPS: u8 = 0b0100;
const GYRO_RANGE_1000_DPS: u8 = 0b1000;
const GYRO_RANGE_2000_DPS: u8 = 0b1100;
const GYRO_ODR_12P5_HZ: u8 = 0b00010000;
const GYRO_ODR_26_HZ: u8 = 0b00100000;
const GYRO_ODR_52_HZ: u8 = 0b00110000;
const GYRO_ODR_104_HZ: u8 = 0b01000000;
const GYRO_ODR_208_HZ: u8 = 0b01010000; // corrected: was 0b010100000 (9 bits)
const GYRO_ODR_416_HZ: u8 = 0b01100000;
const GYRO_ODR_833_HZ: u8 = 0b01110000;
const GYRO_ODR_1666_HZ: u8 = 0b10000000;
const GYRO_ODR_3332_HZ: u8 = 0b10010000;
const GYRO_ODR_6664_HZ: u8 = 0b10100000;
const REG_CTRL3_C: u8 = 0x12;
const BDU: u8 = 0b01000000;
const IF_INC: u8 = 0b00000100;
const SW_RESET: u8 = 0b00000001;
const _REG_CTRL4_C: u8 = 0x13;
const _I2C_DISABLE: u8 = 0b00000100;
const _LPF1_SEL_G: u8 = 0b00000010;
const _REG_CTRL5_C: u8 = 0x14;
const _REG_CTRL6_C: u8 = 0x15;
const _XL_HM_MODE_DISABLE: u8 = 0b00010000;
const _LPF1_MEDIUM_HI: u8 = 0x00;
const _LPF1_MEDIUM_LO: u8 = 0x01;
const _LPF1_LO: u8 = 0x02;
const _LPF1_HI: u8 = 0x03;
const _REG_CTRL7_G: u8 = 0x16;
const _REG_CTRL8_XL: u8 = 0x17;
const _REG_CTRL9_XL: u8 = 0x18;
const _REG_CTRL10_C: u8 = 0x19;
const _REG_WAKE_UP_SRC: u8 = 0x1B;
const _REG_TAP_SRC: u8 = 0x1C;
const _REG_D6D_SRC: u8 = 0x1D;
const _REG_STATUS_REG: u8 = 0x1E;
const _REG_RESERVED_1F: u8 = 0x1F;
const _REG_OUT_TEMP_L: u8 = 0x20;
const _REG_OUT_TEMP_H: u8 = 0x22;
const _REG_OUTX_L_G: u8 = 0x22;
const _REG_OUTX_H_G: u8 = 0x23;
const _REG_OUTY_L_G: u8 = 0x24;
const _REG_OUTY_H_G: u8 = 0x25;
const _REG_OUTZ_L_G: u8 = 0x26;
const _REG_OUTZ_H_G: u8 = 0x27;
const _REG_OUTX_L_ACC: u8 = 0x28;
const _REG_OUTX_H_ACC: u8 = 0x29;
const _REG_OUTY_L_ACC: u8 = 0x2A;
const _REG_OUTY_H_ACC: u8 = 0x2B;
const _REG_OUTZ_L_ACC: u8 = 0x2C;
const _REG_OUTZ_H_ACC: u8 = 0x2D;
// **** IMU Registers and associated bitflags ****

pub struct Lsm6ds<B: ImuBus> {
    pub bus: B,
    pub common: ImuCommon,
    pub config: ImuConfig,
}

fn delay_ms(_delay: u32) {}

impl<B: ImuBus> Lsm6ds<B> {
    const DEVICE_ID: u8 = 0x68;

    pub fn new(bus: B, axis_order: ImuAxesOrder) -> Self {
        Self {
            bus,
            common: ImuCommon::default(),
            config: ImuConfig {
                gyro_id_msp: ImuConfig::MSP_GYRO_ID_LSM6DS,
                acc_id_msp: ImuConfig::MSP_ACC_ID_LSM6DS,
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
        //if (chip_id != REG_WHO_AM_I_RESPONSE_LSM6DS3TR_C && chip_id != REG_WHO_AM_I_RESPONSE_ISM330DHCX && chip_id != REG_WHO_AM_I_RESPONSE_LSM6DSOX) {
        // software reset
        self.bus.write_register(REG_CTRL3_C, SW_RESET).await?;

        // set data ready pulsed
        self.bus.write_register(REG_DATA_READY_PULSE_CONFIG, DATA_READY_PULSED).await?;
        delay_ms(1);

        // Interrupt pins are by default forced to ground, so active high
        self.bus.write_register(REG_INT1_CTRL, INT1_DRDY_G).await?; // Enable gyro data ready on INT1 pin
        delay_ms(1);

        self.bus.write_register(REG_INT2_CTRL, INT2_DRDY_G).await?; // Enable gyro data ready on INT2 pin
        delay_ms(1);

        self.bus.write_register(REG_CTRL3_C, BDU | IF_INC).await?; // Block Data Update and automatically increment registers when read via serial interface (I2C or SPI)
        delay_ms(1);

        let gyro_odr = if target_output_data_rate_hz == 0 || target_output_data_rate_hz > 3332 {
            GYRO_ODR_6664_HZ
        } else if target_output_data_rate_hz > 1666 {
            GYRO_ODR_3332_HZ
        } else if target_output_data_rate_hz > 833 {
            GYRO_ODR_1666_HZ
        } else if target_output_data_rate_hz > 416 {
            GYRO_ODR_833_HZ
        } else if target_output_data_rate_hz > 208 {
            GYRO_ODR_416_HZ
        } else if target_output_data_rate_hz > 104 {
            GYRO_ODR_208_HZ
        } else if target_output_data_rate_hz > 52 {
            GYRO_ODR_104_HZ
        } else if target_output_data_rate_hz > 26 {
            GYRO_ODR_52_HZ
        } else if target_output_data_rate_hz > 13 {
            GYRO_ODR_26_HZ
        } else {
            GYRO_ODR_12P5_HZ
        };

        self.common.gyro_sample_rate_hz = match gyro_odr {
            GYRO_ODR_6664_HZ => 6664,
            GYRO_ODR_3332_HZ => 3332,
            GYRO_ODR_1666_HZ => 1666,
            GYRO_ODR_833_HZ => 833,
            GYRO_ODR_416_HZ => 416,
            GYRO_ODR_208_HZ => 208,
            GYRO_ODR_104_HZ => 104,
            GYRO_ODR_52_HZ => 52,
            GYRO_ODR_26_HZ => 26,
            _ => 12,
        };

        let gyro_register_value: u8;
        match gyro_sensitivity {
            ImuCommon::GYRO_FULL_SCALE_125_DPS | ImuCommon::GYRO_FULL_SCALE_250_DPS => {
                self.common.gyro_scale_dps = 245.0 / 32768.0;
                gyro_register_value = GYRO_RANGE_125_DPS | gyro_odr;
            }
            ImuCommon::GYRO_FULL_SCALE_500_DPS => {
                self.common.gyro_scale_dps = 500.0 / 32768.0;
                gyro_register_value = GYRO_RANGE_500_DPS | gyro_odr;
            }
            ImuCommon::GYRO_FULL_SCALE_1000_DPS => {
                self.common.gyro_scale_dps = 1000.0 / 32768.0;
                gyro_register_value = GYRO_RANGE_1000_DPS | gyro_odr;
            }
            _ => {
                // default includes ImuCommon::GYRO_FULL_SCALE_2000_DPS
                self.common.gyro_scale_dps = 2000.0 / 32768.0;
                gyro_register_value = GYRO_RANGE_2000_DPS | gyro_odr;
            }
        }
        self.bus.write_register(REG_CTRL2_G, gyro_register_value).await?;

        let acc_odr = if target_output_data_rate_hz == 0 || target_output_data_rate_hz > 3332 {
            ACC_ODR_6664_HZ
        } else if target_output_data_rate_hz > 1666 {
            ACC_ODR_3332_HZ
        } else if target_output_data_rate_hz > 833 {
            ACC_ODR_1666_HZ
        } else if target_output_data_rate_hz > 416 {
            ACC_ODR_833_HZ
        } else if target_output_data_rate_hz > 208 {
            ACC_ODR_416_HZ
        } else if target_output_data_rate_hz > 104 {
            ACC_ODR_208_HZ
        } else if target_output_data_rate_hz > 52 {
            ACC_ODR_104_HZ
        } else if target_output_data_rate_hz > 26 {
            ACC_ODR_52_HZ
        } else if target_output_data_rate_hz > 13 {
            ACC_ODR_26_HZ
        } else {
            ACC_ODR_12P5_HZ
        };

        self.common.acc_sample_rate_hz = match acc_odr {
            ACC_ODR_6664_HZ => 6664,
            ACC_ODR_3332_HZ => 3332,
            ACC_ODR_1666_HZ => 1666,
            ACC_ODR_833_HZ => 833,
            ACC_ODR_416_HZ => 416,
            ACC_ODR_208_HZ => 208,
            ACC_ODR_104_HZ => 104,
            ACC_ODR_52_HZ => 52,
            ACC_ODR_26_HZ => 26,
            _ => 12,
        };
        let acc_register_value: u8;
        match acc_sensitivity {
            ImuCommon::ACC_FULL_SCALE_2G => {
                self.common.acc_scale = 2.0 / 32768.0;
                acc_register_value = ACC_RANGE_2G | acc_odr;
            }
            ImuCommon::ACC_FULL_SCALE_4G => {
                self.common.acc_scale = 4.0 / 32768.0;
                acc_register_value = ACC_RANGE_4G | acc_odr;
            }
            ImuCommon::ACC_FULL_SCALE_8G => {
                self.common.acc_scale = 8.0 / 32768.0;
                acc_register_value = ACC_RANGE_8G | acc_odr;
            }
            _ => {
                // default includes  ImuCommon::ACC_FULL_SCALE_16G
                self.common.acc_scale = 16.0 / 32768.0;
                acc_register_value = ACC_RANGE_16G | acc_odr;
            }
        }
        Ok((gyro_register_value, acc_register_value))
    }

    pub fn map_acc(&mut self, buf: [u8; 6], axis_order: ImuAxesOrder) -> Vector3df32 {
        let acc16 = Vector3di16 {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        };
        let acc = Vector3df32::from(acc16) * self.common.acc_scale - self.common.acc_offset;
        ImuAxesOrder::map_vector(axis_order, &acc)
    }

    pub fn map_gyro_rps(&mut self, buf: [u8; 6], axis_order: ImuAxesOrder) -> Vector3df32 {
        let gyro16 = Vector3di16 {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        };
        let gyro_rps = Vector3df32::from(gyro16) * self.common.gyro_scale_rps - self.common.gyro_offset;
        ImuAxesOrder::map_vector(axis_order, &gyro_rps)
    }

    pub fn map_acc_gyro_rps(&mut self, buf: [u8; 12], axis_order: ImuAxesOrder) -> ImuReadingf32 {
        let acc16 = Vector3di16 {
            x: i16::from_le_bytes([buf[6], buf[7]]),
            y: i16::from_le_bytes([buf[8], buf[9]]),
            z: i16::from_le_bytes([buf[10], buf[11]]),
        };
        let gyro16 = Vector3di16 {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
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
        is_normal::<Lsm6ds<MockImuBus>>();
    }
    #[test]
    fn imu_init() {
        let mut imu_bus = MockImuBus::new();
        assert_eq!(0, imu_bus.registers[REG_CTRL3_C as usize]);
        imu_bus.registers[REG_CTRL3_C as usize] = 4;
        let mut imu: Lsm6ds<MockImuBus> = Lsm6ds::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);

        let result = pollster::block_on(imu.init(8000, ImuCommon::GYRO_FULL_SCALE_MAX, ImuCommon::ACC_FULL_SCALE_MAX));
        let (gyro_register_value, acc_register_value) = result.unwrap();

        assert_eq!(172, gyro_register_value);
        assert_eq!(164, acc_register_value);

        let reg = pollster::block_on(imu.read_register(REG_CTRL3_C));
        assert_eq!(BDU | IF_INC, reg.unwrap());

        let reg = pollster::block_on(imu.read_register(REG_DATA_READY_PULSE_CONFIG));
        assert_eq!(DATA_READY_PULSED, reg.unwrap());

        assert_eq!(2000.0 / 32768.0, imu.common.gyro_scale_dps);
        assert_eq!(16.0 / 32768.0, imu.common.acc_scale);
        assert_eq!(6664, imu.common.gyro_sample_rate_hz);
        assert_eq!(6664, imu.common.acc_sample_rate_hz);
    }
    #[test]
    fn map_acc() {
        let imu_bus = MockImuBus::new();

        let mut imu: Lsm6ds<MockImuBus> = Lsm6ds::new(imu_bus, ImuAxesOrder::XPOS_YPOS_ZPOS);

        let data: [u8; 6] = [0, 0, 0, 0, 0, 0];
        let acc = imu.map_acc(data, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32 { x: 0.0, y: 0.0, z: 0.0 }, acc);
    }
}
