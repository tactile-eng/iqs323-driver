use super::*;

pub struct GeneralRegisters;
type General<'a, D> = RegisterBlock<'a, GeneralRegisters, D>;

impl<D> Iqs323<D> {
    pub fn general(&mut self) -> General<D> {
        RegisterBlock {
            iqs323: self,
            phantom: PhantomData,
        }
    }
}

device_driver::implement_device!(
    impl<'a, D> General<'a, D> {
        register OutAMask {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xd0;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register TransactionTimeout {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xd1;
            const SIZE_BITS: usize = 16;

            ms: u16 = 0..16,
        },
        register EventTimeouts {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xd2;
            const SIZE_BITS: usize = 16;

            touch: u8 = 8..16,
            prox: u8 = 0..8,
        },
    }
);

cfg_if::cfg_if! {
    if #[cfg(feature = "movement-ui")] {
        device_driver::implement_device!(
            impl<'a, D> General<'a, D> {
                register EventsEnable {
                    type RWType = ReadWrite;
                    type ByteOrder = LE;
                    const ADDRESS: u8 = 0xd3;
                    const SIZE_BITS: usize = 16;

                    ati_error: bool = 6,
                    ati_event: bool = 4,
                    power_event: bool = 3,
                    slider_event: bool = 2,
                    touch_event: bool = 1,
                    prox_event: bool = 0,
                },
                register MovementTimeout {
                    type RWType = ReadWrite;
                    type ByteOrder = LE;
                    const ADDRESS: u8 = 0xd4;
                    const SIZE_BITS: usize = 16;

                    value: u16 = 0..16,
                },
            }
        );
    } else {
        device_driver::implement_device!(
            impl<'a, D> General<'a, D> {
                register EventsEnable {
                    type RWType = ReadWrite;
                    type ByteOrder = LE;
                    const ADDRESS: u8 = 0xd3;
                    const SIZE_BITS: usize = 16;

                    activation_setting_threshold: u8 = 8..16,
                    ati_error: bool = 6,
                    ati_event: bool = 4,
                    power_event: bool = 3,
                    slider_event: bool = 2,
                    touch_event: bool = 1,
                    prox_event: bool = 0,
                },
                register ReleaseUiSettings {
                    type RWType = ReadWrite;
                    type ByteOrder = LE;
                    const ADDRESS: u8 = 0xd4;
                    const SIZE_BITS: usize = 16;

                    delta_snapshot_sample_delay: u8 = 8..16,
                    release_delta_percentage: u8 = 0..8,
                }
            }
        );
    }
}
