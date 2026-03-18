#![allow(unused)]

use vector_quaternion_matrix::{Vector3df32, Vector3di16};

use crate::{AccScale, I2cInterface, Imu, ImuAxesOrder, ImuBus, ImuConfig, ImuReading, ImuState, SetupError};

use cfg_if::cfg_if;

// IMU Registers and associated bitflags
const REG_RESERVED_00: u8 = 0x00;
const REG_FUNC_CFG_ACCESS: u8 = 0x01;
const REG_RESERVED_03: u8 = 0x03;

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
const REG_WHO_AM_I: u8 = 0x0F;
const REG_WHO_AM_I_RESPONSE_LSM6DS3TR_C: u8 = 0x6A;
const REG_WHO_AM_I_RESPONSE_ISM330DHCX: u8 = 0x6B;
const REG_WHO_AM_I_RESPONSE_LSM6DSOX: u8 = 0x6C;
const REG_CTRL1_XL: u8 = 0x10;
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
const GYRO_RANGE_245_DPS: u8 = 0b0000; // LSM6DS3TR_C
const GYRO_RANGE_250_DPS: u8 = 0b0000; // ISM330DHCX, LSM6DSOX
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
const REG_CTRL4_C: u8 = 0x13;
const I2C_DISABLE: u8 = 0b00000100;
const LPF1_SEL_G: u8 = 0b00000010;
const REG_CTRL5_C: u8 = 0x14;
const REG_CTRL6_C: u8 = 0x15;
const XL_HM_MODE_DISABLE: u8 = 0b00010000;
const LPF1_MEDIUM_HI: u8 = 0x00;
const LPF1_MEDIUM_LO: u8 = 0x01;
const LPF1_LO: u8 = 0x02;
const LPF1_HI: u8 = 0x03;
const REG_CTRL7_G: u8 = 0x16;
const REG_CTRL8_XL: u8 = 0x17;
const REG_CTRL9_XL: u8 = 0x18;
const REG_CTRL10_C: u8 = 0x19;
const REG_WAKE_UP_SRC: u8 = 0x1B;
const REG_TAP_SRC: u8 = 0x1C;
const REG_D6D_SRC: u8 = 0x1D;
const REG_STATUS_REG: u8 = 0x1E;
const REG_RESERVED_1F: u8 = 0x1F;
const REG_OUT_TEMP_L: u8 = 0x20;
const REG_OUT_TEMP_H: u8 = 0x22;
const REG_OUTX_L_G: u8 = 0x22;
const REG_OUTX_H_G: u8 = 0x23;
const REG_OUTY_L_G: u8 = 0x24;
const REG_OUTY_H_G: u8 = 0x25;
const REG_OUTZ_L_G: u8 = 0x26;
const REG_OUTZ_H_G: u8 = 0x27;
const REG_OUTX_L_ACC: u8 = 0x28;
const REG_OUTX_H_ACC: u8 = 0x29;
const REG_OUTY_L_ACC: u8 = 0x2A;
const REG_OUTY_H_ACC: u8 = 0x2B;
const REG_OUTZ_L_ACC: u8 = 0x2C;
const REG_OUTZ_H_ACC: u8 = 0x2D;

impl ImuState {
    pub fn init_lsm6ds(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: u8,
        acc_sensitivity: u8,
    ) -> (u8, u8) {
        let gyro_odr = if target_output_data_rate_hz == 0 {
            GYRO_ODR_6664_HZ
        } else if target_output_data_rate_hz > 3332 {
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

        self.gyro_sample_rate_hz = match gyro_odr {
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
            ImuState::GYRO_FULL_SCALE_125_DPS | ImuState::GYRO_FULL_SCALE_250_DPS => {
                //self._bus.write_register(REG_CTRL2_G, GYRO_RANGE_125_DPS | gyro_odr).await;
                gyro_register_value = GYRO_RANGE_125_DPS | gyro_odr;
                self.gyro_scale_dps = 245.0 / 32768.0;
            }
            ImuState::GYRO_FULL_SCALE_500_DPS => {
                //self._bus.write_register(REG_CTRL2_G, GYRO_RANGE_500_DPS | gyro_odr).await;
                gyro_register_value = GYRO_RANGE_500_DPS | gyro_odr;
                self.gyro_scale_dps = 500.0 / 32768.0;
            }
            ImuState::GYRO_FULL_SCALE_1000_DPS => {
                //self._bus.write_register(REG_CTRL2_G, GYRO_RANGE_1000_DPS | gyro_odr).await;
                gyro_register_value = GYRO_RANGE_1000_DPS | gyro_odr;
                self.gyro_scale_dps = 1000.0 / 32768.0;
            }
            _ | ImuState::GYRO_FULL_SCALE_2000_DPS => {
                //self._bus.write_register(REG_CTRL2_G, GYRO_RANGE_2000_DPS | gyro_odr).await;
                gyro_register_value = GYRO_RANGE_2000_DPS | gyro_odr;
                self.gyro_scale_dps = 2000.0 / 32768.0;
            }
        }

        let acc_odr = if target_output_data_rate_hz == 0 {
            ACC_ODR_6664_HZ
        } else if target_output_data_rate_hz > 3332 {
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

        self.acc_sample_rate_hz = match acc_odr {
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
            ImuState::ACC_FULL_SCALE_2G => {
                acc_register_value = ACC_RANGE_2G | acc_odr;
                self.acc_scale = 2.0 / 32768.0;
            }
            ImuState::ACC_FULL_SCALE_4G => {
                acc_register_value = ACC_RANGE_4G | acc_odr;
                self.acc_scale = 4.0 / 32768.0;
            }
            ImuState::ACC_FULL_SCALE_8G => {
                acc_register_value = ACC_RANGE_8G | acc_odr;
                self.acc_scale = 8.0 / 32768.0;
            }
            _ | ImuState::ACC_FULL_SCALE_16G => {
                acc_register_value = ACC_RANGE_16G | acc_odr;
                self.acc_scale = 16.0 / 32768.0;
            }
        }
        (gyro_register_value, acc_register_value)
    }
    fn map_lsm6ds_acc(&mut self, buf: [u8; 6], axis_order: ImuAxesOrder) -> Vector3df32 {
        let acc16 = Vector3di16 {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        };
        let acc = Vector3df32::from(acc16) * self.acc_scale - self.acc_offset;
        let acc = ImuAxesOrder::map_vector(axis_order, &acc);
        acc
    }
    fn map_lsm6ds_gyro_rps(&mut self, buf: [u8; 6], axis_order: ImuAxesOrder) -> Vector3df32 {
        let gyro16 = Vector3di16 {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        };
        let gyro_rps = Vector3df32::from(gyro16) * self.gyro_scale_rps - self.gyro_offset;
        ImuAxesOrder::map_vector(axis_order, &gyro_rps)
    }
    fn map_lsm6ds_acc_gyro_rps(&mut self, buf: [u8; 12], axis_order: ImuAxesOrder) -> ImuReading {
        let gyro16 = Vector3di16 {
            x: i16::from_le_bytes([buf[0], buf[1]]),
            y: i16::from_le_bytes([buf[2], buf[3]]),
            z: i16::from_le_bytes([buf[4], buf[5]]),
        };
        let acc16 = Vector3di16 {
            x: i16::from_le_bytes([buf[6], buf[7]]),
            y: i16::from_le_bytes([buf[8], buf[9]]),
            z: i16::from_le_bytes([buf[10], buf[11]]),
        };
        let imu_reading = ImuReading {
            acc: Vector3df32::from(acc16) * self.acc_scale - self.acc_offset,
            gyro_rps: Vector3df32::from(gyro16) * self.gyro_scale_rps - self.gyro_offset,
        };
        ImuAxesOrder::map_reading(axis_order, &imu_reading)
    }
}

pub struct Lsm6ds<B: ImuBus> {
    pub bus: B,
    pub state: ImuState,
    pub config: ImuConfig,
}

//if (chip_id != REG_WHO_AM_I_RESPONSE_LSM6DS3TR_C && chip_id != REG_WHO_AM_I_RESPONSE_ISM330DHCX && chip_id != REG_WHO_AM_I_RESPONSE_LSM6DSOX) {

impl<B: ImuBus> Lsm6ds<B> {
    const DEVICE_ID: u8 = 0x68;

    pub fn new(bus: B, axis_order: ImuAxesOrder) -> Self {
        Self {
            bus: bus,
            state: ImuState::default(),
            config: ImuConfig {
                gyro_id_msp: ImuConfig::MSP_GYRO_ID_LSM6DS,
                acc_id_msp: ImuConfig::MSP_ACC_ID_LSM6DS,
                axis_order: axis_order,
                device_id: Self::DEVICE_ID,
                flags: 0,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn is_normal<T: Sized + Send + Sync + Unpin>() {}

    #[test]
    fn normal_types() {
        //is_normal::<Vector3d<f32>>();
    }
    #[test]
    fn imu_state_init_lsm6ds() {
        let mut state: ImuState = ImuState::default();
        let (gyro_register_value, acc_register_value) =
            state.init_lsm6ds(8000, ImuState::GYRO_FULL_SCALE_MAX, ImuState::ACC_FULL_SCALE_MAX);

        assert_eq!(172, gyro_register_value);
        assert_eq!(164, acc_register_value);
        assert_eq!(2000.0 / 32768.0, state.gyro_scale_dps);
        assert_eq!(16.0 / 32768.0, state.acc_scale);
        assert_eq!(6664, state.gyro_sample_rate_hz);
        assert_eq!(6664, state.acc_sample_rate_hz);
    }
    #[test]
    fn map_lsm6ds_acc() {
        let mut state: ImuState = ImuState::default();
        let (gyro_register_value, acc_register_value) =
            state.init_mpu6050(8000, ImuState::GYRO_FULL_SCALE_MAX, ImuState::ACC_FULL_SCALE_MAX);

        // TODO: sit down and work out some useful test data for this
        let data: [u8; 6] = [0, 0, 0, 0, 0, 0];
        let acc = state.map_lsm6ds_acc(data, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32 { x: 0.0, y: 0.0, z: 0.0 }, acc);
    }
}
