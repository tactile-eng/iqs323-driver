#![cfg(feature = "movement-ui")]

use super::*;

pub struct MovementUiRegisters;
type MovementUi<'a, D> = RegisterBlock<'a, MovementUiRegisters, D>;

impl<D> Iqs323<D> {
    pub fn ui(&mut self) -> MovementUi<D> {
        RegisterBlock {
            iqs323: self,
            phantom: PhantomData,
        }
    }
}

device_driver::implement_device!(
    impl<D> Iqs323<D> {
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
