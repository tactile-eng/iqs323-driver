#![cfg(not(feature = "movement-ui"))]

use super::*;

pub struct ReleaseUiRegisters;
type ReleaseUi<'a, D, P> = RegisterBlock<'a, ReleaseUiRegisters, D, P>;

impl<D, P> Iqs323<D, P> {
    pub fn ui(&mut self) -> ReleaseUi<D, P> {
        RegisterBlock {
            iqs323: self,
            phantom: PhantomData,
        }
    }
}

device_driver::implement_device!(
    impl<'a, D, P> ReleaseUi<'a, D, P> {
        register Ch0ActivationLta {
            type RWType = ReadOnly;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0x20;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch1ActivationLta {
            type RWType = ReadOnly;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0x21;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch2ActivationLta {
            type RWType = ReadOnly;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0x22;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch0DeltaSnapshot {
            type RWType = ReadOnly;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0x23;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch1DeltaSnapshot {
            type RWType = ReadOnly;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0x24;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch2DeltaSnapshot {
            type RWType = ReadOnly;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0x25;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
    }
);
