use crate::ImuReading;
use vector_quaternion_matrix::Vector3df32;

use cfg_if::cfg_if;
use strum_macros::EnumIter;

#[allow(non_camel_case_types)]
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, EnumIter)]
pub enum ImuAxesOrder {
    XPOS_YPOS_ZPOS = 0,
    YPOS_XNEG_ZPOS = 1, // rotate  90 degrees anticlockwise
    XNEG_YNEG_ZPOS = 2, // rotate 180 degrees
    YNEG_XPOS_ZPOS = 3, // rotate 270 degrees anticlockwise
    XPOS_YNEG_ZNEG = 4,
    YPOS_XPOS_ZNEG = 5,
    XNEG_YPOS_ZNEG = 6,
    YNEG_XNEG_ZNEG = 7,

    ZPOS_YNEG_XPOS = 8,
    YPOS_ZPOS_XPOS = 9,
    ZNEG_YPOS_XPOS = 10,
    YNEG_ZNEG_XPOS = 11,

    ZPOS_YPOS_XNEG = 12,
    YPOS_ZNEG_XNEG = 13,
    ZNEG_YNEG_XNEG = 14,
    YNEG_ZPOS_XNEG = 15,

    ZPOS_XPOS_YPOS = 16,
    XNEG_ZPOS_YPOS = 17,
    ZNEG_XNEG_YPOS = 18,
    XPOS_ZNEG_YPOS = 19,

    ZPOS_XNEG_YNEG = 20,
    XNEG_ZNEG_YNEG = 21,
    ZNEG_XPOS_YNEG = 22,
    XPOS_ZPOS_YNEG = 23,

    XPOS_YPOS_ZPOS_45 = 24, // rotate  45 degrees anticlockwise
    YPOS_XNEG_ZPOS_45 = 25, // rotate 135 degrees anticlockwise
    XNEG_YNEG_ZPOS_45 = 26, // rotate 225 degrees anticlockwise
    YNEG_XPOS_ZPOS_45 = 27, // rotate 315 degrees anticlockwise

                            //XPOS_YPOS_ZPOS_135 = YPOS_XNEG_ZPOS_45,
                            //XPOS_YPOS_ZPOS_225 = XNEG_YNEG_ZPOS_45,
                            //XPOS_YPOS_ZPOS_315 = YNEG_XPOS_ZPOS_45,
}

impl ImuAxesOrder {
    pub fn map_vector(self, data: &Vector3df32) -> Vector3df32 {
        const SIN45F: f32 = 0.707_106_77_f32;
        const COS45F: f32 = 0.707_106_77_f32;

        match self {
            ImuAxesOrder::XPOS_YPOS_ZPOS => *data,
            ImuAxesOrder::YPOS_XNEG_ZPOS => Vector3df32 { x: data.y, y: -data.x, z: data.z },
            ImuAxesOrder::XNEG_YNEG_ZPOS => Vector3df32 { x: -data.x, y: -data.y, z: data.z },
            ImuAxesOrder::YNEG_XPOS_ZPOS => Vector3df32 { x: -data.y, y: data.x, z: data.z },
            ImuAxesOrder::XPOS_YNEG_ZNEG => Vector3df32 { x: data.x, y: -data.y, z: -data.z },
            ImuAxesOrder::YPOS_XPOS_ZNEG => Vector3df32 { x: data.y, y: data.x, z: -data.z },
            ImuAxesOrder::XNEG_YPOS_ZNEG => Vector3df32 { x: -data.x, y: data.y, z: -data.z },
            ImuAxesOrder::YNEG_XNEG_ZNEG => Vector3df32 { x: -data.y, y: -data.x, z: -data.z },
            ImuAxesOrder::ZPOS_YNEG_XPOS => Vector3df32 { x: data.z, y: -data.y, z: data.x },
            ImuAxesOrder::YPOS_ZPOS_XPOS => Vector3df32 { x: data.y, y: data.z, z: data.x },
            ImuAxesOrder::ZNEG_YPOS_XPOS => Vector3df32 { x: -data.z, y: data.y, z: data.x },
            ImuAxesOrder::YNEG_ZNEG_XPOS => Vector3df32 { x: -data.y, y: -data.z, z: data.x },
            ImuAxesOrder::ZPOS_YPOS_XNEG => Vector3df32 { x: data.z, y: data.y, z: -data.x },
            ImuAxesOrder::YPOS_ZNEG_XNEG => Vector3df32 { x: data.y, y: -data.z, z: -data.x },
            ImuAxesOrder::ZNEG_YNEG_XNEG => Vector3df32 { x: -data.z, y: -data.y, z: -data.x },
            ImuAxesOrder::YNEG_ZPOS_XNEG => Vector3df32 { x: -data.y, y: data.z, z: -data.x },
            ImuAxesOrder::ZPOS_XPOS_YPOS => Vector3df32 { x: data.z, y: data.x, z: data.y },
            ImuAxesOrder::XNEG_ZPOS_YPOS => Vector3df32 { x: -data.x, y: data.z, z: data.y },
            ImuAxesOrder::ZNEG_XNEG_YPOS => Vector3df32 { x: -data.z, y: -data.x, z: data.y },
            ImuAxesOrder::XPOS_ZNEG_YPOS => Vector3df32 { x: data.x, y: -data.z, z: data.y },
            ImuAxesOrder::ZPOS_XNEG_YNEG => Vector3df32 { x: data.z, y: -data.x, z: -data.y },
            ImuAxesOrder::XNEG_ZNEG_YNEG => Vector3df32 { x: -data.x, y: -data.z, z: -data.y },
            ImuAxesOrder::ZNEG_XPOS_YNEG => Vector3df32 { x: -data.z, y: data.x, z: -data.y },
            ImuAxesOrder::XPOS_ZPOS_YNEG => Vector3df32 { x: data.x, y: data.z, z: -data.y },
            ImuAxesOrder::XPOS_YPOS_ZPOS_45 => {
                Vector3df32 { x: data.x * COS45F + data.y * SIN45F, y: -data.x * SIN45F + data.y * COS45F, z: data.z }
            }
            ImuAxesOrder::YPOS_XNEG_ZPOS_45 => {
                const SIN135F: f32 = SIN45F;
                const COS135F: f32 = -COS45F;
                Vector3df32 {
                    x: data.x * COS135F + data.y * SIN135F,
                    y: -data.x * SIN135F + data.y * COS135F,
                    z: data.z,
                }
            }
            ImuAxesOrder::XNEG_YNEG_ZPOS_45 => {
                const SIN225F: f32 = -SIN45F;
                const COS225F: f32 = -COS45F;
                Vector3df32 {
                    x: data.x * COS225F + data.y * SIN225F,
                    y: -data.x * SIN225F + data.y * COS225F,
                    z: data.z,
                }
            }
            ImuAxesOrder::YNEG_XPOS_ZPOS_45 => {
                const SIN315F: f32 = -SIN45F;
                const COS315F: f32 = COS45F;
                Vector3df32 {
                    x: data.x * COS315F + data.y * SIN315F,
                    y: -data.x * SIN315F + data.y * COS315F,
                    z: data.z,
                }
            }
        }
    }
    pub fn map_reading(self, data: &ImuReading) -> ImuReading {
        // use a feature flag to hardcode the mapping, so that the match statement can be bypassed for optimal performance.
        cfg_if! {
        if #[cfg(feature = "LIBRARY_SENSORS_IMU_FIXED_AXES_XPOS_YPOS_ZPOS")] {
            *data
        } else if #[cfg(feature = "LIBRARY_SENSORS_IMU_FIXED_AXES_YNEG_XPOS_ZPOS")] {
            ImuReading {
                acc: Vector3df32 {
                    x: -data.acc.y,
                    y: data.acc.x,
                    z: data.acc.z,
                },
                gyro_rps: Vector3df32 {
                    x: -data.gyro_rps.y,
                    y: data.gyro_rps.x,
                    z: data.gyro_rps.z,
                },
            }
        } else {
            const SIN45F: f32 = core::f32::consts::FRAC_1_SQRT_2;
            const COS45F: f32 = core::f32::consts::FRAC_1_SQRT_2;
            const SIN135F: f32 = SIN45F;
            const COS135F: f32 = -COS45F;
            const SIN225F: f32 = -SIN45F;
            const COS225F: f32 = -COS45F;
            const SIN315F: f32 = -SIN45F;
            const COS315F: f32 = COS45F;

            match self {
                ImuAxesOrder::XPOS_YPOS_ZPOS => *data,
                ImuAxesOrder::YPOS_XNEG_ZPOS => ImuReading {
                    acc: Vector3df32 {
                        x: data.acc.y,
                        y: -data.acc.x,
                        z: data.acc.z,
                    },
                    gyro_rps: Vector3df32 {
                        x: data.gyro_rps.y,
                        y: -data.gyro_rps.x,
                        z: data.gyro_rps.z,
                    },
                },
                ImuAxesOrder::XNEG_YNEG_ZPOS => ImuReading {
                    acc: Vector3df32 {
                        x: -data.acc.x,
                        y: -data.acc.y,
                        z: data.acc.z,
                    },
                    gyro_rps: Vector3df32 {
                        x: -data.gyro_rps.x,
                        y: -data.gyro_rps.y,
                        z: data.gyro_rps.z,
                    },
                },
                ImuAxesOrder::YNEG_XPOS_ZPOS => ImuReading {
                    acc: Vector3df32 {
                        x: -data.acc.y,
                        y: data.acc.x,
                        z: data.acc.z,
                    },
                    gyro_rps: Vector3df32 {
                        x: -data.gyro_rps.y,
                        y: data.gyro_rps.x,
                        z: data.gyro_rps.z,
                    },
                },
                ImuAxesOrder::XPOS_YNEG_ZNEG => ImuReading {
                    acc: Vector3df32 {
                        x: data.acc.x,
                        y: -data.acc.y,
                        z: -data.acc.z,
                    },
                    gyro_rps: Vector3df32 {
                        x: data.gyro_rps.x,
                        y: -data.gyro_rps.y,
                        z: -data.gyro_rps.z,
                    },
                },
                ImuAxesOrder::YPOS_XPOS_ZNEG => ImuReading {
                    acc: Vector3df32 {
                        x: data.acc.y,
                        y: data.acc.x,
                        z: -data.acc.z,
                    },
                    gyro_rps: Vector3df32 {
                        x: data.gyro_rps.y,
                        y: data.gyro_rps.x,
                        z: -data.gyro_rps.z,
                    },
                },
                ImuAxesOrder::XNEG_YPOS_ZNEG => ImuReading {
                    acc: Vector3df32 {
                        x: -data.acc.x,
                        y: data.acc.y,
                        z: -data.acc.z,
                    },
                    gyro_rps: Vector3df32 {
                        x: -data.gyro_rps.x,
                        y: data.gyro_rps.y,
                        z: -data.gyro_rps.z,
                    },
                },
                ImuAxesOrder::YNEG_XNEG_ZNEG => ImuReading {
                    acc: Vector3df32 {
                        x: -data.acc.y,
                        y: -data.acc.x,
                        z: -data.acc.z,
                    },
                    gyro_rps: Vector3df32 {
                        x: -data.gyro_rps.y,
                        y: -data.gyro_rps.x,
                        z: -data.gyro_rps.z,
                    },
                },
                ImuAxesOrder::ZPOS_YNEG_XPOS => ImuReading {
                    acc: Vector3df32 {
                        x: data.acc.z,
                        y: -data.acc.y,
                        z: data.acc.x,
                    },
                    gyro_rps: Vector3df32 {
                        x: data.gyro_rps.z,
                        y: -data.gyro_rps.y,
                        z: data.gyro_rps.x,
                    },
                },
                ImuAxesOrder::YPOS_ZPOS_XPOS => ImuReading {
                    acc: Vector3df32 {
                        x: data.acc.y,
                        y: data.acc.z,
                        z: data.acc.x,
                    },
                    gyro_rps: Vector3df32 {
                        x: data.gyro_rps.y,
                        y: data.gyro_rps.z,
                        z: data.gyro_rps.x,
                    },
                },
                ImuAxesOrder::ZNEG_YPOS_XPOS => ImuReading {
                    acc: Vector3df32 {
                        x: -data.acc.z,
                        y: data.acc.y,
                        z: data.acc.x,
                    },
                    gyro_rps: Vector3df32 {
                        x: -data.gyro_rps.z,
                        y: data.gyro_rps.y,
                        z: data.gyro_rps.x,
                    },
                },
                ImuAxesOrder::YNEG_ZNEG_XPOS => ImuReading {
                    acc: Vector3df32 {
                        x: -data.acc.y,
                        y: -data.acc.z,
                        z: data.acc.x,
                    },
                    gyro_rps: Vector3df32 {
                        x: -data.gyro_rps.y,
                        y: -data.gyro_rps.z,
                        z: data.gyro_rps.x,
                    },
                },
                ImuAxesOrder::ZPOS_YPOS_XNEG => ImuReading {
                    acc: Vector3df32 {
                        x: data.acc.z,
                        y: data.acc.y,
                        z: -data.acc.x,
                    },
                    gyro_rps: Vector3df32 {
                        x: data.gyro_rps.z,
                        y: data.gyro_rps.y,
                        z: -data.gyro_rps.x,
                    },
                },
                ImuAxesOrder::YPOS_ZNEG_XNEG => ImuReading {
                    acc: Vector3df32 {
                        x: data.acc.y,
                        y: -data.acc.z,
                        z: -data.acc.x,
                    },
                    gyro_rps: Vector3df32 {
                        x: data.gyro_rps.y,
                        y: -data.gyro_rps.z,
                        z: -data.gyro_rps.x,
                    },
                },
                ImuAxesOrder::ZNEG_YNEG_XNEG => ImuReading {
                    acc: Vector3df32 {
                        x: -data.acc.z,
                        y: -data.acc.y,
                        z: -data.acc.x,
                    },
                    gyro_rps: Vector3df32 {
                        x: -data.gyro_rps.z,
                        y: -data.gyro_rps.y,
                        z: -data.gyro_rps.x,
                    },
                },
                ImuAxesOrder::YNEG_ZPOS_XNEG => ImuReading {
                    acc: Vector3df32 {
                        x: -data.acc.y,
                        y: data.acc.z,
                        z: -data.acc.x,
                    },
                    gyro_rps: Vector3df32 {
                        x: -data.gyro_rps.y,
                        y: data.gyro_rps.z,
                        z: -data.gyro_rps.x,
                    },
                },
                ImuAxesOrder::ZPOS_XPOS_YPOS => ImuReading {
                    acc: Vector3df32 {
                        x: data.acc.z,
                        y: data.acc.x,
                        z: data.acc.y,
                    },
                    gyro_rps: Vector3df32 {
                        x: data.gyro_rps.z,
                        y: data.gyro_rps.x,
                        z: data.gyro_rps.y,
                    },
                },
                ImuAxesOrder::XNEG_ZPOS_YPOS => ImuReading {
                    acc: Vector3df32 {
                        x: -data.acc.x,
                        y: data.acc.z,
                        z: data.acc.y,
                    },
                    gyro_rps: Vector3df32 {
                        x: -data.gyro_rps.x,
                        y: data.gyro_rps.z,
                        z: data.gyro_rps.y,
                    },
                },
                ImuAxesOrder::ZNEG_XNEG_YPOS => ImuReading {
                    acc: Vector3df32 {
                        x: -data.acc.z,
                        y: -data.acc.x,
                        z: data.acc.y,
                    },
                    gyro_rps: Vector3df32 {
                        x: -data.gyro_rps.z,
                        y: -data.gyro_rps.x,
                        z: data.gyro_rps.y,
                    },
                },
                ImuAxesOrder::XPOS_ZNEG_YPOS => ImuReading {
                    acc: Vector3df32 {
                        x: data.acc.x,
                        y: -data.acc.z,
                        z: data.acc.y,
                    },
                    gyro_rps: Vector3df32 {
                        x: data.gyro_rps.x,
                        y: -data.gyro_rps.z,
                        z: data.gyro_rps.y,
                    },
                },
                ImuAxesOrder::ZPOS_XNEG_YNEG => ImuReading {
                    acc: Vector3df32 {
                        x: data.acc.z,
                        y: -data.acc.x,
                        z: -data.acc.y,
                    },
                    gyro_rps: Vector3df32 {
                        x: data.gyro_rps.z,
                        y: -data.gyro_rps.x,
                        z: -data.gyro_rps.y,
                    },
                },
                ImuAxesOrder::XNEG_ZNEG_YNEG => ImuReading {
                    acc: Vector3df32 {
                        x: -data.acc.x,
                        y: -data.acc.z,
                        z: -data.acc.y,
                    },
                    gyro_rps: Vector3df32 {
                        x: -data.gyro_rps.x,
                        y: -data.gyro_rps.z,
                        z: -data.gyro_rps.y,
                    },
                },
                ImuAxesOrder::ZNEG_XPOS_YNEG => ImuReading {
                    acc: Vector3df32 {
                        x: -data.acc.z,
                        y: data.acc.x,
                        z: -data.acc.y,
                    },
                    gyro_rps: Vector3df32 {
                        x: -data.gyro_rps.z,
                        y: data.gyro_rps.x,
                        z: -data.gyro_rps.y,
                    },
                },
                ImuAxesOrder::XPOS_ZPOS_YNEG => ImuReading {
                    acc: Vector3df32 {
                        x: data.acc.x,
                        y: data.acc.z,
                        z: -data.acc.y,
                    },
                    gyro_rps: Vector3df32 {
                        x: data.gyro_rps.x,
                        y: data.gyro_rps.z,
                        z: -data.gyro_rps.y,
                    },
                },
                ImuAxesOrder::XPOS_YPOS_ZPOS_45 => ImuReading {
                    acc: Vector3df32 {
                        x: data.acc.x * COS45F + data.acc.y * SIN45F,
                        y: -data.acc.x * SIN45F + data.acc.y * COS45F,
                        z: data.acc.z,
                    },
                    gyro_rps: Vector3df32 {
                        x: data.gyro_rps.x * COS45F + data.gyro_rps.y * SIN45F,
                        y: -data.gyro_rps.x * SIN45F + data.gyro_rps.y * COS45F,
                        z: data.gyro_rps.z,
                    },
                },
                ImuAxesOrder::YPOS_XNEG_ZPOS_45 => ImuReading {
                    acc: Vector3df32 {
                        x: data.acc.x * COS135F + data.acc.y * SIN135F,
                        y: -data.acc.x * SIN135F + data.acc.y * COS135F,
                        z: data.acc.z,
                    },
                    gyro_rps: Vector3df32 {
                        x: data.gyro_rps.x * COS135F + data.gyro_rps.y * SIN135F,
                        y: -data.gyro_rps.x * SIN135F + data.gyro_rps.y * COS135F,
                        z: data.gyro_rps.z,
                    },
                },
                ImuAxesOrder::XNEG_YNEG_ZPOS_45 => ImuReading {
                    acc: Vector3df32 {
                        x: data.acc.x * COS225F + data.acc.y * SIN225F,
                        y: -data.acc.x * SIN225F + data.acc.y * COS225F,
                        z: data.acc.z,
                    },
                    gyro_rps: Vector3df32 {
                        x: data.gyro_rps.x * COS225F + data.gyro_rps.y * SIN225F,
                        y: -data.gyro_rps.x * SIN225F + data.gyro_rps.y * COS225F,
                        z: data.gyro_rps.z,
                    },
                },
                ImuAxesOrder::YNEG_XPOS_ZPOS_45 => ImuReading {
                    acc: Vector3df32 {
                        x: data.acc.x * COS315F + data.acc.y * SIN315F,
                        y: -data.acc.x * SIN315F + data.acc.y * COS315F,
                        z: data.acc.z,
                    },
                    gyro_rps: Vector3df32 {
                        x: data.gyro_rps.x * COS315F + data.gyro_rps.y * SIN315F,
                        y: -data.gyro_rps.x * SIN315F + data.gyro_rps.y * COS315F,
                        z: data.gyro_rps.z,
                    },
                },
            }
        }
        }
    }
    pub fn axes_order_inverse(self) -> Self {
        match self {
            ImuAxesOrder::YPOS_XNEG_ZPOS => ImuAxesOrder::YNEG_XPOS_ZPOS,
            ImuAxesOrder::YNEG_XPOS_ZPOS => ImuAxesOrder::YPOS_XNEG_ZPOS,
            ImuAxesOrder::YPOS_ZPOS_XPOS => ImuAxesOrder::ZPOS_XPOS_YPOS,
            ImuAxesOrder::ZNEG_YPOS_XPOS => ImuAxesOrder::ZPOS_YPOS_XNEG,
            ImuAxesOrder::YNEG_ZNEG_XPOS => ImuAxesOrder::ZPOS_XNEG_YNEG,
            ImuAxesOrder::ZPOS_YPOS_XNEG => ImuAxesOrder::ZNEG_YPOS_XPOS,
            ImuAxesOrder::YPOS_ZNEG_XNEG => ImuAxesOrder::ZNEG_XPOS_YNEG,
            ImuAxesOrder::YNEG_ZPOS_XNEG => ImuAxesOrder::ZNEG_XNEG_YPOS,
            ImuAxesOrder::ZPOS_XPOS_YPOS => ImuAxesOrder::YPOS_ZPOS_XPOS,
            ImuAxesOrder::ZNEG_XNEG_YPOS => ImuAxesOrder::YNEG_ZPOS_XNEG,
            ImuAxesOrder::XPOS_ZNEG_YPOS => ImuAxesOrder::XPOS_ZPOS_YNEG,
            ImuAxesOrder::ZPOS_XNEG_YNEG => ImuAxesOrder::YNEG_ZNEG_XPOS,
            ImuAxesOrder::ZNEG_XPOS_YNEG => ImuAxesOrder::YPOS_ZNEG_XNEG,
            ImuAxesOrder::XPOS_ZPOS_YNEG => ImuAxesOrder::XPOS_ZNEG_YPOS,
            ImuAxesOrder::XPOS_YPOS_ZPOS_45 => ImuAxesOrder::YNEG_XPOS_ZPOS_45, // 45 => 315
            ImuAxesOrder::YPOS_XNEG_ZPOS_45 => ImuAxesOrder::XNEG_YNEG_ZPOS_45, // 135 => 225
            ImuAxesOrder::XNEG_YNEG_ZPOS_45 => ImuAxesOrder::YPOS_XNEG_ZPOS_45, // 225 => 1355
            ImuAxesOrder::YNEG_XPOS_ZPOS_45 => ImuAxesOrder::XPOS_YPOS_ZPOS_45, // 315 => 45
            _ => self,                                                          // other axis orders are self-inverting
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ImuState;
    use strum::IntoEnumIterator;

    fn is_normal<T: Sized + Send + Sync + Unpin>() {}

    #[test]
    fn normal_types() {
        is_normal::<ImuReading>();
    }
    #[test]
    fn imu_state_default() {
        let state = ImuState::default();
        let z: Vector3df32 = Vector3df32::default();
        assert_eq!(state.acc_offset, z);
    }
    #[test]
    fn map_vector() {
        const INPUT: Vector3df32 = Vector3df32 { x: 2.0, y: 3.0, z: 5.0 };
        let output = ImuAxesOrder::map_vector(ImuAxesOrder::XPOS_YPOS_ZPOS, &INPUT);
        assert_eq!(Vector3df32 { x: 2.0, y: 3.0, z: 5.0 }, output);
        let output = ImuAxesOrder::map_vector(ImuAxesOrder::YPOS_XNEG_ZPOS, &INPUT);
        assert_eq!(Vector3df32 { x: 3.0, y: -2.0, z: 5.0 }, output);
    }
    #[test]
    fn axes_order_inverse() {
        let input = ImuAxesOrder::XPOS_YPOS_ZPOS;
        let output = ImuAxesOrder::axes_order_inverse(input);
        let output_inverse = ImuAxesOrder::axes_order_inverse(output);
        assert_eq!(input, output_inverse);
        for axis_order in ImuAxesOrder::iter() {
            let output = ImuAxesOrder::axes_order_inverse(axis_order);
            let output_inverse = ImuAxesOrder::axes_order_inverse(output);
            assert_eq!(axis_order, output_inverse);
        }
    }
}
