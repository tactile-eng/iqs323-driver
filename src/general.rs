use super::*;

register_block!(
    /// General (read/write)
    General
);

device_driver::implement_device!(
    impl<'a, D, P> General<'a, D, P> {
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
        #[cfg(not(feature = "movement-ui"))]
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
        #[cfg(feature = "movement-ui")]
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
        #[cfg(not(feature = "movement-ui"))]
        register ReleaseUiSettings {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xd4;
            const SIZE_BITS: usize = 16;

            delta_snapshot_sample_delay: u8 = 8..16,
            release_delta_percentage: u8 = 0..8,
        },
        #[cfg(feature = "movement-ui")]
        register MovementTimeout {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xd4;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
    }
);
