#![allow(unused)]

use vector_quaternion_matrix::{Vector3df32, Vector3di16};

use crate::{ImuAxesOrder, ImuReading, ImuState};

const REG_XG_OFFS_TC_H: u8 = 0x04;
const REG_XG_OFFS_TC_L: u8 = 0x05;
const REG_YG_OFFS_TC_H: u8 = 0x07;
const REG_YG_OFFS_TC_L: u8 = 0x08;
const REG_ZG_OFFS_TC_H: u8 = 0x0A;
const REG_ZG_OFFS_TC_L: u8 = 0x0B;

const REG_SELF_TEST_X_ACCEL: u8 = 0x0D;
const REG_SELF_TEST_Y_ACCEL: u8 = 0x0E;
const REG_SELF_TEST_Z_ACCEL: u8 = 0x0F;

const REG_XG_OFFS_USRH: u8 = 0x13;
const REG_XG_OFFS_USRL: u8 = 0x14;
const REG_YG_OFFS_USRH: u8 = 0x15;
const REG_YG_OFFS_USRL: u8 = 0x16;
const REG_ZG_OFFS_USRH: u8 = 0x17;
const REG_ZG_OFFS_USRL: u8 = 0x18;

const REG_SAMPLE_RATE_DIVIDER: u8 = 0x19;
const DIVIDE_BY_1: u8 = 0x00;
const DIVIDE_BY_2: u8 = 0x01;

const REG_CONFIG: u8 = 0x1A;
const DLPF_CFG_1: u8 = 0x01;
const DLPF_CFG_7: u8 = 0x07;

const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_ACCEL_CONFIG: u8 = 0x1C;
const REG_ACCEL_CONFIG2: u8 = 0x1D;

const REG_FIFO_ENABLE: u8 = 0x23;
const GYRO_FIFO_EN: u8 = 0b00001000;
const ACC_FIFO_EN: u8 = 0b00000100;

const REG_INT_PIN_CFG: u8 = 0x37;
const REG_INT_ENABLE: u8 = 0x38;
const FIFO_WM_INT_STATUS: u8 = 0x39;

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

const REG_FIFO_WM_TH1: u8 = 0x60;
const REG_FIFO_WM_TH2: u8 = 0x61;

const REG_SIGNAL_PATH_RESET: u8 = 0x68;
const REG_ACCEL_INTEL_CTRL: u8 = 0x69;
const REG_USER_CTRL: u8 = 0x6A;
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_PWR_MGMT_2: u8 = 0x6C;

const REG_FIFO_COUNT_H: u8 = 0x72;
const REG_FIFO_COUNT_L: u8 = 0x73;
const REG_FIFO_R_W: u8 = 0x74;

const REG_WHO_AM_I: u8 = 0x75;

const REG_XA_OFFSET_H: u8 = 0x77;
const REG_XA_OFFSET_L: u8 = 0x78;
const REG_YA_OFFSET_H: u8 = 0x7A;
const REG_YA_OFFSET_L: u8 = 0x7B;
const REG_ZA_OFFSET_H: u8 = 0x7D;
const REG_ZA_OFFSET_L: u8 = 0x7E;

impl ImuState {
    pub fn init_mpu6886(
        &mut self,
        target_output_data_rate_hz: u32,
        gyro_sensitivity: u8,
        acc_sensitivity: u8,
    ) -> (u8, u8) {
        (0, 0)
    }
    fn map_mpu6886_acc(&mut self, buf: [u8; 6], axis_order: ImuAxesOrder) -> Vector3df32 {
        let acc16 = Vector3di16 {
            x: i16::from_be_bytes([buf[0], buf[1]]),
            y: i16::from_be_bytes([buf[2], buf[3]]),
            z: i16::from_be_bytes([buf[4], buf[5]]),
        };
        let acc = Vector3df32::from(acc16) * self.acc_scale - self.acc_offset;
        ImuAxesOrder::map_vector(axis_order, &acc)
    }
    fn map_mpu6886_gyro_rps(&mut self, buf: [u8; 6], axis_order: ImuAxesOrder) -> Vector3df32 {
        let gyro16 = Vector3di16 {
            x: i16::from_be_bytes([buf[0], buf[1]]),
            y: i16::from_be_bytes([buf[2], buf[3]]),
            z: i16::from_be_bytes([buf[4], buf[5]]),
        };
        let gyro_rps = Vector3df32::from(gyro16) * self.gyro_scale_rps - self.gyro_offset;
        ImuAxesOrder::map_vector(axis_order, &gyro_rps)
    }
    fn map_mpu6886_acc_gyro_rps(&mut self, buf: [u8; 12], axis_order: ImuAxesOrder) -> ImuReading {
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
        let imu_reading = ImuReading {
            acc: Vector3df32::from(acc16) * self.acc_scale - self.acc_offset,
            gyro_rps: Vector3df32::from(gyro16) * self.gyro_scale_rps - self.gyro_offset,
        };
        ImuAxesOrder::map_reading(axis_order, &imu_reading)
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
    fn imu_state_init_mpu6886() {
        let mut state: ImuState = ImuState::default();
        let (gyro_register_value, acc_register_value) =
            state.init_mpu6886(8000, ImuState::GYRO_FULL_SCALE_MAX, ImuState::ACC_FULL_SCALE_MAX);

        assert_eq!(0, gyro_register_value);
        assert_eq!(0, acc_register_value);
        //assert_eq!(2000.0 / 32768.0, state.gyro_scale_dps);
        //assert_eq!(16.0 / 32768.0, state.acc_scale);
        //assert_eq!(6664, state.gyro_sample_rate_hz);
        //assert_eq!(6664, state.acc_sample_rate_hz);
    }
    #[test]
    fn map_lsm6ds_acc() {
        let mut state: ImuState = ImuState::default();
        let (gyro_register_value, acc_register_value) =
            state.init_mpu6886(8000, ImuState::GYRO_FULL_SCALE_MAX, ImuState::ACC_FULL_SCALE_MAX);

        // TODO: sit down and work out some useful test data for this
        let data: [u8; 6] = [0, 0, 0, 0, 0, 0];
        let acc = state.map_mpu6886_acc(data, ImuAxesOrder::XPOS_YPOS_ZPOS);
        assert_eq!(Vector3df32 { x: 0.0, y: 0.0, z: 0.0 }, acc);
    }
}
