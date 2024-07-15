use super::*;

pub struct I2cRegisters;
type I2cSettings<'a, D> = RegisterBlock<'a, I2cRegisters, D>;

impl<D> Iqs323<D> {
    pub fn i2c(&mut self) -> I2cSettings<D> {
        RegisterBlock {
            iqs323: self,
            phantom: PhantomData,
        }
    }
}

device_driver::implement_device!(
    impl<'a, D> I2cSettings<'a, D> {
        register Settings {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xe0;
            const SIZE_BITS: usize = 16;

            rw_check_disabled: bool = 1,
            stop_bit_disabled: bool = 0,
        },
        register HardwareId {
            type RWType = ReadOnly;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xe1;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        }
    }
);
