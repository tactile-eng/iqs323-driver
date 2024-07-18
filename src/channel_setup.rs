use super::*;

register_block!(
    /// Channel Setup (read/write)
    ChannelSetup<BASE_ADDR>
);

device_driver::implement_device!(
    impl<'a, D, P, const BASE_ADDR: u8> ChannelSetup<'a, D, P, BASE_ADDR> {
        register Setup {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0x00;
            const SIZE_BITS: usize = 16;

            follower_event_mask: u8 = 8..16,
            reference_sensor_id: u8 = 4..8,
            channel_mode: u8 as enum ChannelMode {
                Independent,
                Follower,
                Reference,
            } = 0..4,
        },
        register ProxSettings {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0x01;
            const SIZE_BITS: usize = 16;

            debounce_exit: u8 = 12..16,
            debounce_enter: u8 = 8..12,
            threshold: u8 = 0..8,
        },
        register TouchSettings {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0x02;
            const SIZE_BITS: usize = 16;

            hysteresis: u8 = 12..16,
            threshold: u8 = 0..8,
        },
        register FollowerWeight {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0x03;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        #[cfg(feature = "movement-ui")]
        register MovementUiSettings {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0x04;
            const SIZE_BITS: usize = 16;

            debounce_exit: u8 = 12..16,
            debounce_enter: u8 = 8..12,
            threshold: u8 = 0..8,
        },
    }
);
