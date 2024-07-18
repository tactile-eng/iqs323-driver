use super::*;

register_block!(
    /// I2C Settings
    I2cSettings
);

device_driver::implement_device!(
    impl<'a, D, P> I2cSettings<'a, D, P> {
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
