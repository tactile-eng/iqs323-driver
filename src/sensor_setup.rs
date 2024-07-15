use super::*;

pub struct SensorSetupRegisters;
type SensorSetup<'a, D, const BASE_ADDR: u8> =
    RegisterBlock<'a, SensorSetupRegisters, D, BASE_ADDR>;

impl<D: I2c> Iqs323<D> {
    pub fn sensor_0_setup(&mut self) -> SensorSetup<D, 0x30> {
        RegisterBlock {
            iqs323: self,
            phantom: PhantomData,
        }
    }

    pub fn sensor_1_setup(&mut self) -> SensorSetup<D, 0x40> {
        RegisterBlock {
            iqs323: self,
            phantom: PhantomData,
        }
    }

    pub fn sensor_2_setup(&mut self) -> SensorSetup<D, 0x50> {
        RegisterBlock {
            iqs323: self,
            phantom: PhantomData,
        }
    }
}

device_driver::implement_device!(
    impl<'a, D: I2c, const BASE_ADDR: u8> SensorSetup<'a, D, BASE_ADDR> {
        register Setup {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x00;
            const SIZE_BITS: usize = 16;

            cal_cap_rx_selected: bool = 14,
            cal_cap_tx_selected: bool = 13,
            txa_enabled: bool = 12,
            ctx2_enabled: bool = 10,
            ctx1_enabled: bool = 9,
            ctx0_enabled: bool = 8,
            ui_enabled: bool = 6,
            fosc_tx_freq_14mhz: bool = 5,
            vbias_on_cx2: bool = 4,
            invert_channel_logic: bool = 3,
            dual_direction_thresholds: bool = 2,
            linearise_counts: bool = 1,
            channel_enabled: bool = 0,
        },
        register ConversionFrequency {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x01;
            const SIZE_BITS: usize = 16;

            period: u8 = 8..16,
            frac: u8 = 0..8,
        },
        register ProxControl {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x02;
            const SIZE_BITS: usize = 16;

            discharge_0v5_enabled: bool = 14,
            /// Note: On hardware rev IQS3ed, the cs_size field is specified to be bits 11-12; however bit 11 is read-only so this still works.
            cs_size: u8 as strict enum CsCapacitorSize {
                Cs40pF,
                Cs80pF = "default",
            } = 12..13,
            sh_bias_Select: u8 as strict enum SampleHoldBiasSelect {
                Bias2uA,
                Bias5uA,
                Bias7uA,
                Bias10uA = "default",
            } = 8..10,
            max_counts: u8 as strict enum MaxCounts {
                Max1023,
                Max2047,
                Max4095,
                Max16383 = "default",
            } = 6..8,
            pxs_mode: u8 as enum PxsMode {
                SelfCapacitance = 0x10,
                MutualCapacitance = 0x13,
                CurrentMeasurement = 0x1d,
                Inductive = 0x3d,
            } = 0..6,
        },
        register ProxInput {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x03;
            const SIZE_BITS: usize = 16;
            const RESET_VALUE: u16 = 0x0083;

            internal_reference_enabled: bool = 13,
            prox_engine_bias_current_enabled: bool = 12,
            calibration_capacitor_enabled: bool = 11,
            crx2_enabled: bool = 10,
            crx1_enabled: bool = 9,
            crx0_enabled: bool = 8,
            dead_time_enabled: bool = 6,
            auto_prox_cycle: u8 as strict enum AutoProxCycleCount {
                Cycles4 = "default",
                Cycles8,
                Cycles16,
                Cycles32,
            } = 2..4,
        },
        register PatternDefinitions {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x04;
            const SIZE_BITS: usize = 16;

            wav_pattern_1: u8 = 12..16,
            wav_pattern_0: u8 = 8..12,
            calibration_cap: u8 = 4..8,
            inactive_rxs: u8 as enum InactiveCxState {
                Floating = 0x00,
                BiasVolatage = 0x05,
                Vss = 0x0a,
                Vreg = 0x0f,
            } = 0..4,
        },
        register PatternSelectionAndEngineBiasCurrent {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x05;
            const SIZE_BITS: usize = 16;

            engine_bias_current: i8 = 12..16,
            engine_bias_current_trim: i8 = 8..12,
            wav_pattern_select: u8 = 0..8,
        },
        register AtiSetup {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x06;
            const SIZE_BITS: usize = 16;

            resolution_factor: u16 = 4..16,
            band: u8 as strict enum AtiBand {
                Small = "default",
                Large,
            } = 3..4,
            mode: u8 as enum AtiMode {
                Disabled = "default",
                CompensationOnly,
                FromCompensationDivider,
                FromFineFractionalDivider,
                Full = 4,
            } = 0..3,
        },
        register AtiBase {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x07;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
        register AtiMultipliers {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x08;
            const SIZE_BITS: usize = 16;

            fine_fractional_multiplier: u8 = 14..16,
            fine_fractional_divider: u8 = 9..14,
            coarse_fractional_multiplier: u8 = 5..9,
            coarse_fractional_divider: u8 = 0..5,
        },
        register Compensation {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0x09;
            const SIZE_BITS: usize = 16;

            divider: u8 = 11..16,
            value: u16 = 0..10,
        },
    }
);
