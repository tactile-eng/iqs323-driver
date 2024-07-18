use super::*;

register_block!(
    /// Filter Betas (read/write)
    FilterBetas
);

device_driver::implement_device!(
    impl<'a, D, P> FilterBetas<'a, D, P> {
        register Counts {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xb0;
            const SIZE_BITS: usize = 16;

            low_power: u8 = 8..16,
            normal_power: u8 = 0..8,
        },
        register Lta {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xb1;
            const SIZE_BITS: usize = 16;

            low_power: u8 = 8..16,
            normal_power: u8 = 0..8,
        },
        register LtaFast {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xb2;
            const SIZE_BITS: usize = 16;

            low_power: u8 = 8..16,
            normal_power: u8 = 0..8,
        },
        register FastFilterBand {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xb4;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        #[cfg(not(feature = "movement-ui"))]
        register ActivationLta {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xb3;
            const SIZE_BITS: usize = 16;

            low_power: u8 = 8..16,
            normal_power: u8 = 0..8,
        },
        #[cfg(feature = "movement-ui")]
        register MovementLta {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xb3;
            const SIZE_BITS: usize = 16;

            low_power: u8 = 8..16,
            normal_power: u8 = 0..8,
        },
    }
);
