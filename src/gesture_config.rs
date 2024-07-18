use super::*;

register_block!(
    /// Gesture Configuration (read/write)
    GestureConfig
);

device_driver::implement_device!(
    impl<'a, D, P> GestureConfig<'a, D, P> {
        register GestureEnable {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xa0;
            const SIZE_BITS: usize = 16;

            hold_enabled: bool = 3,
            flick_enabled: bool = 2,
            swipe_enabled: bool = 1,
            tap_enabled: bool = 0,
        },
        register MinimumGestureTime {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xa1;
            const SIZE_BITS: usize = 16;

            ms: u16 = 0..16,
        },
        register MaximumTapTime {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xa2;
            const SIZE_BITS: usize = 16;

            ms: u16 = 0..16,
        },
        register MaximumSwipeTime {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xa3;
            const SIZE_BITS: usize = 16;

            ms: u16 = 0..16,
        },
        register MinimumHoldTime {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xa4;
            const SIZE_BITS: usize = 16;

            ms: u16 = 0..16,
        },
        register MaximumTapDistance {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xa5;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register MinimumSwipeDistance {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xa6;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
    }
);
