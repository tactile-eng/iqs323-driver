#![cfg(feature = "movement-ui")]

use super::*;

pub struct MovementUiRegisters;
type MovementUi<'a, D, P> = RegisterBlock<'a, MovementUiRegisters, D, P>;

impl<D, P> Iqs323<D, P> {
    pub fn ui(&mut self) -> MovementUi<D, P> {
        RegisterBlock {
            iqs323: self,
            phantom: PhantomData,
        }
    }
}

device_driver::implement_device!(
    impl<D, P> Iqs323<D, P> {
        register Ch0MovementLta {
            type RWType = ReadOnly;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0x20;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch1MovementLta {
            type RWType = ReadOnly;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0x21;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register Ch2MovementLta {
            type RWType = ReadOnly;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0x22;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register MovementStatus {
            type RWType = ReadOnly;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0x23;
            const SIZE_BITS: usize = 16;

            ch0_movement: bool = 0,
            ch1_movement: bool = 1,
            ch2_movement: bool = 2,
        },
    }
);
