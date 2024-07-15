use super::*;

pub struct SliderConfigRegisters;
type SliderConfig<'a, D> = RegisterBlock<'a, SliderConfigRegisters, D>;

impl<D> Iqs323<D> {
    pub fn slider_config(&mut self) -> SliderConfig<D> {
        RegisterBlock {
            iqs323: self,
            phantom: PhantomData,
        }
    }
}

device_driver::implement_device!(
    impl<'a, D> SliderConfig<'a, D> {
        register SliderSetup {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x90;
            const SIZE_BITS: usize = 16;

            lower_calibration_value: u8 = 8..16,
            static_filter: u8 as strict enum SliderStaticFilter {
                Dynamic = "default",
                SlowStaticBeta,
            } = 6..7,
            slow_static_beta: u8 = 3..6,
            total_channels: u8 = 0..3,
        },
        register SliderBottomSpeed {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x91;
            const SIZE_BITS: usize = 16;

            bottom_speed: u8 = 8..16,
            upper_calibration_value: u8 = 0..8,
        },
        register SliderTopSpeed {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x92;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register SliderResolution {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x93;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register SliderEnableMask {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x94;
            const SIZE_BITS: usize = 16;

            ch2_enabled: bool = 2,
            ch1_enabled: bool = 1,
            ch0_enabled: bool = 0,
        },
        register DeltaLink0 {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x96;
            const SIZE_BITS: usize = 16;

            value: u16 as DeltaLinkChannel = 0..16,
        },
        register DeltaLink1 {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x96;
            const SIZE_BITS: usize = 16;

            value: u16 as DeltaLinkChannel = 0..16,
        },
        register DeltaLink2 {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x96;
            const SIZE_BITS: usize = 16;

            value: u16 as DeltaLinkChannel = 0..16,
        },
    }
);

cfg_if::cfg_if! {
    if #[cfg(feature = "movement-ui")] {
        device_driver::implement_device!(
            impl<'a, D> SliderConfig<'a, D> {
                register SliderEnableStatusPointer {
                    type RWType = ReadWrite;
                    const ADDRESS: u8 = 0x95;
                    const SIZE_BITS: usize = 16;
                    const RESET_VALUE: u16 = 0x552;

                    value: u16 = 0..16,
                },
            }
        );

        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, num_enum::IntoPrimitive, num_enum::TryFromPrimitive)]
        #[repr(u16)]
        pub enum DeltaLinkChannel {
            Disabled,
            Channel0 = 0x430,
            Channel1 = 0x474,
            Channel2 = 0x4b8,
        }
    } else {
        device_driver::implement_device!(
            impl<'a, D> SliderConfig<'a, D> {
                register SliderEnableStatusPointer {
                    type RWType = ReadWrite;
                    const ADDRESS: u8 = 0x95;
                    const SIZE_BITS: usize = 16;
                    const RESET_VALUE: u16 = 0x558;

                    value: u16 = 0..16,
                },
            }
        );

        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, num_enum::IntoPrimitive, num_enum::TryFromPrimitive)]
        #[repr(u16)]
        pub enum DeltaLinkChannel {
            Disabled,
            Channel0 = 0x430,
            Channel1 = 0x472,
            Channel2 = 0x4b4,
        }
    }
}
