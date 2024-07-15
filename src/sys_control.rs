use super::*;

pub struct SysControlRegisters;
type SysControl<'a, D> = RegisterBlock<'a, SysControlRegisters, D>;

impl<D: I2c> Iqs323<D> {
    pub fn sys_control(&mut self) -> SysControl<D> {
        RegisterBlock {
            iqs323: self,
            phantom: PhantomData,
        }
    }
}

device_driver::implement_device!(
    impl<'a, D: I2c> SysControl<'a, D> {
        register Control {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0xc0;
            const SIZE_BITS: usize = 16;

            ch2_timeout_disabled: bool = 10,
            ch1_timeout_disabled: bool = 9,
            ch0_timeout_disabled: bool = 8,
            interface: u8 as strict enum InterfaceSelection {
                I2cStreaming = "default",
                I2cEvents,
            } = 7..8,
            power_mode: u8 as enum PowerMode {
                Normal,
                Low,
                UltraLow,
                Halt,
                Automatic,
                AutomaticNoUlp,
            } = 4..7,
            trigger_reseed: bool = 3,
            trigger_ati: bool = 2,
            trigger_soft_reset: bool = 1,
            ack_reset: bool = 0,
        },
        register NormalPowerReportRate {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0xc1;
            const SIZE_BITS: usize = 16;

            ms: u16 = 0..16,
        },
        register LowPowerReportRate {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0xc2;
            const SIZE_BITS: usize = 16;

            ms: u16 = 0..16,
        },
        register UltraLowPowerReportRate {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0xc3;
            const SIZE_BITS: usize = 16;

            ms: u16 = 0..16,
        },
        register HaltReportRate {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0xc4;
            const SIZE_BITS: usize = 16;

            ms: u16 = 0..16,
        },
        register PowerModeTimeout {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0xc5;
            const SIZE_BITS: usize = 16;

            ms: u16 = 0..16,
        },
    }
);
