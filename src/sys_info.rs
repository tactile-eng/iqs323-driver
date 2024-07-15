use super::*;

pub struct SysInfoRegisters;
type SysInfo<'a, D> = RegisterBlock<'a, SysInfoRegisters, D>;

impl<D> Iqs323<D> {
    pub fn sys_info(&mut self) -> SysInfo<D> {
        RegisterBlock {
            iqs323: self,
            phantom: PhantomData,
        }
    }
}

#[cfg(not(feature = "movement-ui"))]
cfg_if::cfg_if! {
    if #[cfg(feature = "movement-ui")] {
        device_driver::implement_device!(
            impl<D> Iqs323<D> {
                register Version {
                    type RWType = ReadOnly;
                    const ADDRESS: u8 = 0;
                    const SIZE_BITS: usize = 48;
                    const RESET_VALUE: [u8] = [0xb6, 0x05, 0x01, 0x00, 0x04, 0x00];

                    product_number: u16 = 0..16,
                    major_version: u16 = 16..32,
                    minor_version: u16 = 32..48,
                },
            }
        );
    } else {
        device_driver::implement_device!(
            impl<D> Iqs323<D> {
                register Version {
                    type RWType = ReadOnly;
                    const ADDRESS: u8 = 0;
                    const SIZE_BITS: usize = 48;
                    const RESET_VALUE: [u8] = [0x52, 0x04, 0x01, 0x00, 0x03, 0x00];

                    product_number: u16 = 0..16,
                    major_version: u16 = 16..32,
                    minor_version: u16 = 32..48,
                },
            }
        );
    }
}

device_driver::implement_device!(
    impl<'a, D> SysInfo<'a, D> {
        register SystemStatus {
            type RWType = ReadOnly;
            const ADDRESS: u8 = 0x10;
            const SIZE_BITS: usize = 16;

            prox_event: bool = 0,
            touch_event: bool = 1,
            slider_event: bool = 2,
            power_event: bool = 3,
            ati_event: bool = 4,
            ati_active: bool = 5,
            ati_error: bool = 6,
            reset_event: bool = 7,
            ch0_prox_event: bool = 8,
            ch0_touch_event: bool = 9,
            ch1_prox_event: bool = 10,
            ch1_touch_event: bool = 11,
            ch2_prox_event: bool = 12,
            ch2_touch_event: bool = 13,
            current_power_mode: u8 as strict enum PowerMode {
                Normal = "default",
                Low = 1,
                UltraLow = 2,
                Halt = 3,
            } = 14..16,
        },
        register Gestures {
            type RWType = ReadOnly;
            const ADDRESS: u8 = 0x11;
            const SIZE_BITS: usize = 16;

            tap: bool = 0,
            swipe_positive: bool = 1,
            swipe_negative: bool = 2,
            flick_positive: bool = 3,
            flick_negative: bool = 4,
            hold: bool = 5,
            event: bool = 6,
            busy: bool = 7,
        },
        register SliderPosition {
            type RWType = ReadOnly;
            const ADDRESS: u8 = 0x12;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch0FilteredCounts {
            type RWType = ReadOnly;
            const ADDRESS: u8 = 0x13;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch0Lta {
            type RWType = ReadOnly;
            const ADDRESS: u8 = 0x14;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch1FilteredCounts {
            type RWType = ReadOnly;
            const ADDRESS: u8 = 0x15;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch1Lta {
            type RWType = ReadOnly;
            const ADDRESS: u8 = 0x16;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch2FilteredCounts {
            type RWType = ReadOnly;
            const ADDRESS: u8 = 0x17;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch2Lta {
            type RWType = ReadOnly;
            const ADDRESS: u8 = 0x18;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
    }
);
