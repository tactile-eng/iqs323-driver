use super::*;

pub struct ChannelSetupRegisters;
type ChannelSetup<'a, D, const BASE_ADDR: u8> =
    RegisterBlock<'a, ChannelSetupRegisters, D, BASE_ADDR>;

impl<D> Iqs323<D> {
    pub fn channel_0_setup(&mut self) -> ChannelSetup<D, 0x60> {
        RegisterBlock {
            iqs323: self,
            phantom: PhantomData,
        }
    }

    pub fn channel_1_setup(&mut self) -> ChannelSetup<D, 0x70> {
        RegisterBlock {
            iqs323: self,
            phantom: PhantomData,
        }
    }

    pub fn channel_2_setup(&mut self) -> ChannelSetup<D, 0x80> {
        RegisterBlock {
            iqs323: self,
            phantom: PhantomData,
        }
    }
}

device_driver::implement_device!(
    impl<'a, D, const BASE_ADDR: u8> ChannelSetup<'a, D, BASE_ADDR> {
        register Setup {
            type RWType = ReadWrite;
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
            const ADDRESS: u8 = 0x01;
            const SIZE_BITS: usize = 16;

            debounce_exit: u8 = 12..16,
            debounce_enter: u8 = 8..12,
            threshold: u8 = 0..8,
        },
        register TouchSettings {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x02;
            const SIZE_BITS: usize = 16;

            hysteresis: u8 = 12..16,
            threshold: u8 = 0..8,
        },
        register FollowerWeight {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x03;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
    }
);

#[cfg(feature = "movement-ui")]
device_driver::implement_device!(
    impl<'a, D, const BASE_ADDR: u8> ChannelSetup<'a, D, BASE_ADDR> {
        register MovementUiSettings {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x04;
            const SIZE_BITS: usize = 16;

            debounce_exit: u8 = 12..16,
            debounce_enter: u8 = 8..12,
            threshold: u8 = 0..8,
        },
    }
);
