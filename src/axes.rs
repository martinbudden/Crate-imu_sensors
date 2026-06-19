//#![allow(clippy::return_self_not_must_use)]
use vqm::Vector3df32;

//use cfg_if::cfg_if;
use num_enum::{FromPrimitive, IntoPrimitive};
use strum::EnumIter;

#[allow(non_camel_case_types)]
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, EnumIter, FromPrimitive, IntoPrimitive)]
pub enum ImuAxesOrder {
    #[num_enum(default)]
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
    #[must_use]
    pub fn map_vector(self, v: Vector3df32) -> Vector3df32 {
        const SIN45F: f32 = 0.707_106_77_f32;
        const COS45F: f32 = 0.707_106_77_f32;

        match self {
            ImuAxesOrder::XPOS_YPOS_ZPOS => v,
            ImuAxesOrder::YPOS_XNEG_ZPOS => Vector3df32 { x: v.y, y: -v.x, z: v.z },
            ImuAxesOrder::XNEG_YNEG_ZPOS => Vector3df32 { x: -v.x, y: -v.y, z: v.z },
            ImuAxesOrder::YNEG_XPOS_ZPOS => Vector3df32 { x: -v.y, y: v.x, z: v.z },
            ImuAxesOrder::XPOS_YNEG_ZNEG => Vector3df32 { x: v.x, y: -v.y, z: -v.z },
            ImuAxesOrder::YPOS_XPOS_ZNEG => Vector3df32 { x: v.y, y: v.x, z: -v.z },
            ImuAxesOrder::XNEG_YPOS_ZNEG => Vector3df32 { x: -v.x, y: v.y, z: -v.z },
            ImuAxesOrder::YNEG_XNEG_ZNEG => Vector3df32 { x: -v.y, y: -v.x, z: -v.z },
            ImuAxesOrder::ZPOS_YNEG_XPOS => Vector3df32 { x: v.z, y: -v.y, z: v.x },
            ImuAxesOrder::YPOS_ZPOS_XPOS => Vector3df32 { x: v.y, y: v.z, z: v.x },
            ImuAxesOrder::ZNEG_YPOS_XPOS => Vector3df32 { x: -v.z, y: v.y, z: v.x },
            ImuAxesOrder::YNEG_ZNEG_XPOS => Vector3df32 { x: -v.y, y: -v.z, z: v.x },
            ImuAxesOrder::ZPOS_YPOS_XNEG => Vector3df32 { x: v.z, y: v.y, z: -v.x },
            ImuAxesOrder::YPOS_ZNEG_XNEG => Vector3df32 { x: v.y, y: -v.z, z: -v.x },
            ImuAxesOrder::ZNEG_YNEG_XNEG => Vector3df32 { x: -v.z, y: -v.y, z: -v.x },
            ImuAxesOrder::YNEG_ZPOS_XNEG => Vector3df32 { x: -v.y, y: v.z, z: -v.x },
            ImuAxesOrder::ZPOS_XPOS_YPOS => Vector3df32 { x: v.z, y: v.x, z: v.y },
            ImuAxesOrder::XNEG_ZPOS_YPOS => Vector3df32 { x: -v.x, y: v.z, z: v.y },
            ImuAxesOrder::ZNEG_XNEG_YPOS => Vector3df32 { x: -v.z, y: -v.x, z: v.y },
            ImuAxesOrder::XPOS_ZNEG_YPOS => Vector3df32 { x: v.x, y: -v.z, z: v.y },
            ImuAxesOrder::ZPOS_XNEG_YNEG => Vector3df32 { x: v.z, y: -v.x, z: -v.y },
            ImuAxesOrder::XNEG_ZNEG_YNEG => Vector3df32 { x: -v.x, y: -v.z, z: -v.y },
            ImuAxesOrder::ZNEG_XPOS_YNEG => Vector3df32 { x: -v.z, y: v.x, z: -v.y },
            ImuAxesOrder::XPOS_ZPOS_YNEG => Vector3df32 { x: v.x, y: v.z, z: -v.y },
            ImuAxesOrder::XPOS_YPOS_ZPOS_45 => {
                Vector3df32 { x: v.x * COS45F + v.y * SIN45F, y: -v.x * SIN45F + v.y * COS45F, z: v.z }
            }
            ImuAxesOrder::YPOS_XNEG_ZPOS_45 => {
                const SIN135F: f32 = SIN45F;
                const COS135F: f32 = -COS45F;
                Vector3df32 { x: v.x * COS135F + v.y * SIN135F, y: -v.x * SIN135F + v.y * COS135F, z: v.z }
            }
            ImuAxesOrder::XNEG_YNEG_ZPOS_45 => {
                const SIN225F: f32 = -SIN45F;
                const COS225F: f32 = -COS45F;
                Vector3df32 { x: v.x * COS225F + v.y * SIN225F, y: -v.x * SIN225F + v.y * COS225F, z: v.z }
            }
            ImuAxesOrder::YNEG_XPOS_ZPOS_45 => {
                const SIN315F: f32 = -SIN45F;
                const COS315F: f32 = COS45F;
                Vector3df32 { x: v.x * COS315F + v.y * SIN315F, y: -v.x * SIN315F + v.y * COS315F, z: v.z }
            }
        }
    }

    #[allow(clippy::too_many_lines)]
    #[must_use]
    pub fn map_acc_gyro(self, acc: Vector3df32, gyro: Vector3df32) -> (Vector3df32, Vector3df32) {
        // use a feature flag to hardcode the mapping, so that the match statement can be bypassed for optimal performance.
        /*cfg_if! {
        if #[cfg(feature = "axes_xpos_ypos_zpos")] {
            (acc, gyro)
        } else if #[cfg(feature = "axes_yneg_xpos_zpos")] {
            (Vector3df32 {
                    x: -acc.y,                     y: acc.x,                     z: acc.z,
                },Vector3df32 {
                    x: -gyro_rps.y,                     y: gyro_rps.x,                     z: gyro_rps.z,
                })
        } else {*/
        const SIN45F: f32 = core::f32::consts::FRAC_1_SQRT_2;
        const COS45F: f32 = core::f32::consts::FRAC_1_SQRT_2;
        const SIN135F: f32 = SIN45F;
        const COS135F: f32 = -COS45F;
        const SIN225F: f32 = -SIN45F;
        const COS225F: f32 = -COS45F;
        const SIN315F: f32 = -SIN45F;
        const COS315F: f32 = COS45F;

        match self {
            ImuAxesOrder::XPOS_YPOS_ZPOS => (acc, gyro),
            ImuAxesOrder::YPOS_XNEG_ZPOS => {
                (Vector3df32 { x: acc.y, y: -acc.x, z: acc.z }, Vector3df32 { x: gyro.y, y: -gyro.x, z: gyro.z })
            }
            ImuAxesOrder::XNEG_YNEG_ZPOS => {
                (Vector3df32 { x: -acc.x, y: -acc.y, z: acc.z }, Vector3df32 { x: -gyro.x, y: -gyro.y, z: gyro.z })
            }
            ImuAxesOrder::YNEG_XPOS_ZPOS => {
                (Vector3df32 { x: -acc.y, y: acc.x, z: acc.z }, Vector3df32 { x: -gyro.y, y: gyro.x, z: gyro.z })
            }
            ImuAxesOrder::XPOS_YNEG_ZNEG => {
                (Vector3df32 { x: acc.x, y: -acc.y, z: -acc.z }, Vector3df32 { x: gyro.x, y: -gyro.y, z: -gyro.z })
            }
            ImuAxesOrder::YPOS_XPOS_ZNEG => {
                (Vector3df32 { x: acc.y, y: acc.x, z: -acc.z }, Vector3df32 { x: gyro.y, y: gyro.x, z: -gyro.z })
            }
            ImuAxesOrder::XNEG_YPOS_ZNEG => {
                (Vector3df32 { x: -acc.x, y: acc.y, z: -acc.z }, Vector3df32 { x: -gyro.x, y: gyro.y, z: -gyro.z })
            }
            ImuAxesOrder::YNEG_XNEG_ZNEG => {
                (Vector3df32 { x: -acc.y, y: -acc.x, z: -acc.z }, Vector3df32 { x: -gyro.y, y: -gyro.x, z: -gyro.z })
            }
            ImuAxesOrder::ZPOS_YNEG_XPOS => {
                (Vector3df32 { x: acc.z, y: -acc.y, z: acc.x }, Vector3df32 { x: gyro.z, y: -gyro.y, z: gyro.x })
            }
            ImuAxesOrder::YPOS_ZPOS_XPOS => {
                (Vector3df32 { x: acc.y, y: acc.z, z: acc.x }, Vector3df32 { x: gyro.y, y: gyro.z, z: gyro.x })
            }
            ImuAxesOrder::ZNEG_YPOS_XPOS => {
                (Vector3df32 { x: -acc.z, y: acc.y, z: acc.x }, Vector3df32 { x: -gyro.z, y: gyro.y, z: gyro.x })
            }
            ImuAxesOrder::YNEG_ZNEG_XPOS => {
                (Vector3df32 { x: -acc.y, y: -acc.z, z: acc.x }, Vector3df32 { x: -gyro.y, y: -gyro.z, z: gyro.x })
            }
            ImuAxesOrder::ZPOS_YPOS_XNEG => {
                (Vector3df32 { x: acc.z, y: acc.y, z: -acc.x }, Vector3df32 { x: gyro.z, y: gyro.y, z: -gyro.x })
            }
            ImuAxesOrder::YPOS_ZNEG_XNEG => {
                (Vector3df32 { x: acc.y, y: -acc.z, z: -acc.x }, Vector3df32 { x: gyro.y, y: -gyro.z, z: -gyro.x })
            }
            ImuAxesOrder::ZNEG_YNEG_XNEG => {
                (Vector3df32 { x: -acc.z, y: -acc.y, z: -acc.x }, Vector3df32 { x: -gyro.z, y: -gyro.y, z: -gyro.x })
            }
            ImuAxesOrder::YNEG_ZPOS_XNEG => {
                (Vector3df32 { x: -acc.y, y: acc.z, z: -acc.x }, Vector3df32 { x: -gyro.y, y: gyro.z, z: -gyro.x })
            }
            ImuAxesOrder::ZPOS_XPOS_YPOS => {
                (Vector3df32 { x: acc.z, y: acc.x, z: acc.y }, Vector3df32 { x: gyro.z, y: gyro.x, z: gyro.y })
            }
            ImuAxesOrder::XNEG_ZPOS_YPOS => {
                (Vector3df32 { x: -acc.x, y: acc.z, z: acc.y }, Vector3df32 { x: -gyro.x, y: gyro.z, z: gyro.y })
            }
            ImuAxesOrder::ZNEG_XNEG_YPOS => {
                (Vector3df32 { x: -acc.z, y: -acc.x, z: acc.y }, Vector3df32 { x: -gyro.z, y: -gyro.x, z: gyro.y })
            }
            ImuAxesOrder::XPOS_ZNEG_YPOS => {
                (Vector3df32 { x: acc.x, y: -acc.z, z: acc.y }, Vector3df32 { x: gyro.x, y: -gyro.z, z: gyro.y })
            }
            ImuAxesOrder::ZPOS_XNEG_YNEG => {
                (Vector3df32 { x: acc.z, y: -acc.x, z: -acc.y }, Vector3df32 { x: gyro.z, y: -gyro.x, z: -gyro.y })
            }
            ImuAxesOrder::XNEG_ZNEG_YNEG => {
                (Vector3df32 { x: -acc.x, y: -acc.z, z: -acc.y }, Vector3df32 { x: -gyro.x, y: -gyro.z, z: -gyro.y })
            }
            ImuAxesOrder::ZNEG_XPOS_YNEG => {
                (Vector3df32 { x: -acc.z, y: acc.x, z: -acc.y }, Vector3df32 { x: -gyro.z, y: gyro.x, z: -gyro.y })
            }
            ImuAxesOrder::XPOS_ZPOS_YNEG => {
                (Vector3df32 { x: acc.x, y: acc.z, z: -acc.y }, Vector3df32 { x: gyro.x, y: gyro.z, z: -gyro.y })
            }
            ImuAxesOrder::XPOS_YPOS_ZPOS_45 => (
                Vector3df32 { x: acc.x * COS45F + acc.y * SIN45F, y: -acc.x * SIN45F + acc.y * COS45F, z: acc.z },
                Vector3df32 { x: gyro.x * COS45F + gyro.y * SIN45F, y: -gyro.x * SIN45F + gyro.y * COS45F, z: gyro.z },
            ),
            ImuAxesOrder::YPOS_XNEG_ZPOS_45 => (
                Vector3df32 { x: acc.x * COS135F + acc.y * SIN135F, y: -acc.x * SIN135F + acc.y * COS135F, z: acc.z },
                Vector3df32 {
                    x: gyro.x * COS135F + gyro.y * SIN135F,
                    y: -gyro.x * SIN135F + gyro.y * COS135F,
                    z: gyro.z,
                },
            ),
            ImuAxesOrder::XNEG_YNEG_ZPOS_45 => (
                Vector3df32 { x: acc.x * COS225F + acc.y * SIN225F, y: -acc.x * SIN225F + acc.y * COS225F, z: acc.z },
                Vector3df32 {
                    x: gyro.x * COS225F + gyro.y * SIN225F,
                    y: -gyro.x * SIN225F + gyro.y * COS225F,
                    z: gyro.z,
                },
            ),
            ImuAxesOrder::YNEG_XPOS_ZPOS_45 => (
                Vector3df32 { x: acc.x * COS315F + acc.y * SIN315F, y: -acc.x * SIN315F + acc.y * COS315F, z: acc.z },
                Vector3df32 {
                    x: gyro.x * COS315F + gyro.y * SIN315F,
                    y: -gyro.x * SIN315F + gyro.y * COS315F,
                    z: gyro.z,
                },
            ),
        }
    }
    //}
    //}
    #[must_use]
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
    use crate::ImuCommon;
    use strum::IntoEnumIterator;

    #[allow(unused)]
    fn is_normal<T: Sized + Send + Sync + Unpin>() {}

    #[test]
    fn normal_types() {}
    #[test]
    fn imu_state_default() {
        let state = ImuCommon::default();
        let z: Vector3df32 = Vector3df32::default();
        assert_eq!(state.acc_offset, z);
    }
    #[test]
    fn map_vector() {
        const INPUT: Vector3df32 = Vector3df32 { x: 2.0, y: 3.0, z: 5.0 };
        let output = ImuAxesOrder::map_vector(ImuAxesOrder::XPOS_YPOS_ZPOS, INPUT);
        assert_eq!(Vector3df32 { x: 2.0, y: 3.0, z: 5.0 }, output);
        let output = ImuAxesOrder::map_vector(ImuAxesOrder::YPOS_XNEG_ZPOS, INPUT);
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
