#![cfg(not(feature = "movement-ui"))]

use super::*;

pub struct ReleaseUiRegisters;
type ReleaseUi<'a, D> = RegisterBlock<'a, ReleaseUiRegisters, D>;

impl<D> Iqs323<D> {
    pub fn ui(&mut self) -> ReleaseUi<D> {
        RegisterBlock {
            iqs323: self,
            phantom: PhantomData,
        }
    }
}

device_driver::implement_device!(
    impl<'a, D> ReleaseUi<'a, D> {
        register Ch0ActivationLta {
            type RWType = ReadOnly;
            const ADDRESS: u8 = 0x20;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch1ActivationLta {
            type RWType = ReadOnly;
            const ADDRESS: u8 = 0x21;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch2ActivationLta {
            type RWType = ReadOnly;
            const ADDRESS: u8 = 0x22;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch0DeltaSnapshot {
            type RWType = ReadOnly;
            const ADDRESS: u8 = 0x23;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch1DeltaSnapshot {
            type RWType = ReadOnly;
            const ADDRESS: u8 = 0x24;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch2DeltaSnapshot {
            type RWType = ReadOnly;
            const ADDRESS: u8 = 0x25;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
    }
);
