device_driver::create_device!(
    device_name: Registers,
    dsl: {
        config {
            type RegisterAddressType = u8;
            type DefaultByteOrder = LE;
        }

        #[cfg(feature = "movement-ui")]
        register Version {
            type Access = ReadOnly;
            const ADDRESS = 0;
            const SIZE_BITS = 48;
            const RESET_VALUE = [0xb6, 0x05, 0x01, 0x00, 0x04, 0x00];
            const ALLOW_ADDRESS_OVERLAP = true;

            product_number: uint = 0..16,
            major_version: uint = 16..32,
            minor_version: uint = 32..48,
        },
        #[cfg(not(feature = "movement-ui"))]
        register Version {
            type Access = ReadOnly;
            const ADDRESS = 0;
            const SIZE_BITS = 48;
            const RESET_VALUE = [0x52, 0x04, 0x01, 0x00, 0x03, 0x00];
            const ALLOW_ADDRESS_OVERLAP = true;

            product_number: uint = 0..16,
            major_version: uint = 16..32,
            minor_version: uint = 32..48,
        },
        block SysInfo {
            register SystemStatus {
                type Access = ReadOnly;
                const ADDRESS = 0x10;
                const SIZE_BITS = 16;

                prox_event: bool = 0,
                touch_event: bool = 1,
                slider_event: bool = 2,
                power_event: bool = 3,
                ati_event: bool = 4,
                ati_active: bool = 5,
                ati_error: bool = 6,
                reset_event: bool = 7,
                ch0_prox_event: bool = 8,
                ch0_touch_event: bool = 9,
                ch1_prox_event: bool = 10,
                ch1_touch_event: bool = 11,
                ch2_prox_event: bool = 12,
                ch2_touch_event: bool = 13,
                current_power_mode: uint as enum PowerModeStatus {
                    Normal = default,
                    Low = 1,
                    UltraLow = 2,
                    Halt = 3,
                } = 14..16,
            },
            register Gestures {
                type Access = ReadOnly;
                const ADDRESS = 0x11;
                const SIZE_BITS = 16;

                tap: bool = 0,
                swipe_positive: bool = 1,
                swipe_negative: bool = 2,
                flick_positive: bool = 3,
                flick_negative: bool = 4,
                hold: bool = 5,
                event: bool = 6,
                busy: bool = 7,
            },
            register SliderPosition {
                type Access = ReadOnly;
                const ADDRESS = 0x12;
                const SIZE_BITS = 16;

                value: uint = 0..16,
            },
            register Ch0FilteredCounts {
                type Access = ReadOnly;
                const ADDRESS = 0x13;
                const SIZE_BITS = 16;

                value: uint = 0..16,
            },
            register Ch0Lta {
                type Access = ReadOnly;
                const ADDRESS = 0x14;
                const SIZE_BITS = 16;

                value: uint = 0..16,
            },
            register Ch1FilteredCounts {
                type Access = ReadOnly;
                const ADDRESS = 0x15;
                const SIZE_BITS = 16;

                value: uint = 0..16,
            },
            register Ch1Lta {
                type Access = ReadOnly;
                const ADDRESS = 0x16;
                const SIZE_BITS = 16;

                value: uint = 0..16,
            },
            register Ch2FilteredCounts {
                type Access = ReadOnly;
                const ADDRESS = 0x17;
                const SIZE_BITS = 16;

                value: uint = 0..16,
            },
            register Ch2Lta {
                type Access = ReadOnly;
                const ADDRESS = 0x18;
                const SIZE_BITS = 16;

                value: uint = 0..16,
            },
        },
        #[cfg(not(feature = "movement-ui"))]
        block ReleaseUi {
            register Ch0ActivationLta {
                type Access = ReadOnly;
                const ADDRESS = 0x20;
                const SIZE_BITS = 16;
                const ALLOW_ADDRESS_OVERLAP = true;

                value: uint = 0..16,
            },
            register Ch1ActivationLta {
                type Access = ReadOnly;
                const ADDRESS = 0x21;
                const SIZE_BITS = 16;
                const ALLOW_ADDRESS_OVERLAP = true;

                value: uint = 0..16,
            },
            register Ch2ActivationLta {
                type Access = ReadOnly;
                const ADDRESS = 0x22;
                const SIZE_BITS = 16;
                const ALLOW_ADDRESS_OVERLAP = true;

                value: uint = 0..16,
            },
            register Ch0DeltaSnapshot {
                type Access = ReadOnly;
                const ADDRESS = 0x23;
                const SIZE_BITS = 16;
                const ALLOW_ADDRESS_OVERLAP = true;

                value: uint = 0..16,
            },
            register Ch1DeltaSnapshot {
                type Access = ReadOnly;
                const ADDRESS = 0x24;
                const SIZE_BITS = 16;
                const ALLOW_ADDRESS_OVERLAP = true;

                value: uint = 0..16,
            },
            register Ch2DeltaSnapshot {
                type Access = ReadOnly;
                const ADDRESS = 0x25;
                const SIZE_BITS = 16;
                const ALLOW_ADDRESS_OVERLAP = true;

                value: uint = 0..16,
            },
        },
        #[cfg(feature = "movement-ui")]
        block MovementUi {
            register Ch0MovementLta {
                type Access = ReadOnly;
                const ADDRESS = 0x20;
                const SIZE_BITS = 16;
                const ALLOW_ADDRESS_OVERLAP = true;

                value: uint = 0..16,
            },
            register Ch1MovementLta {
                type Access = ReadOnly;
                const ADDRESS = 0x21;
                const SIZE_BITS = 16;
                const ALLOW_ADDRESS_OVERLAP = true;

                value: uint = 0..16,
            },
            register Ch2MovementLta {
                type Access = ReadOnly;
                const ADDRESS = 0x22;
                const SIZE_BITS = 16;
                const ALLOW_ADDRESS_OVERLAP = true;

                value: uint = 0..16,
            },
            register MovementStatus {
                type Access = ReadOnly;
                const ADDRESS = 0x23;
                const SIZE_BITS = 16;
                const ALLOW_ADDRESS_OVERLAP = true;

                ch0_movement: bool = 0,
                ch1_movement: bool = 1,
                ch2_movement: bool = 2,
            },
        },
        block Sensor {
            const ADDRESS_OFFSET = 0x30;
            const REPEAT = { count: 3, stride: 0x10 };

            register SensorSetup {
                type Access = ReadWrite;
                const ADDRESS = 0x00;
                const SIZE_BITS = 16;

                cal_cap_rx_selected: bool = 14,
                cal_cap_tx_selected: bool = 13,
                txa_enabled: bool = 11,
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
                type Access = ReadWrite;
                const ADDRESS = 0x01;
                const SIZE_BITS = 16;
                const RESET_VALUE = 0x007f;

                period: uint = 8..16,
                frac: uint = 0..8,
            },
            register ProxControl {
                type Access = ReadWrite;
                const ADDRESS = 0x02;
                const SIZE_BITS = 16;

                discharge_0v5_enabled: bool = 14,
                /// Note: On hardware rev IQS3ed, the cs_size field is specified to be bits 11-12; however bit 11 is read-only so this still works.
                cs_size: uint as enum CsCapacitorSize {
                    Cs40pF,
                    Cs80pF = default,
                } = 12..13,
                sh_bias_Select: uint as enum SampleHoldBiasSelect {
                    Bias2uA,
                    Bias5uA,
                    Bias7uA,
                    Bias10uA = default,
                } = 8..10,
                max_counts: uint as enum MaxCounts {
                    Max1023,
                    Max2047,
                    Max4095,
                    Max16383 = default,
                } = 6..8,
                pxs_mode: uint as try enum PxsMode {
                    SelfCapacitance = 0x10,
                    MutualCapacitance = 0x13,
                    CurrentMeasurement = 0x1d,
                    Inductive = 0x3d,
                } = 0..6,
            },
            register ProxInput {
                type Access = ReadWrite;
                const ADDRESS = 0x03;
                const SIZE_BITS = 16;
                const RESET_VALUE = 0x0083;

                internal_reference_enabled: bool = 13,
                prox_engine_bias_current_enabled: bool = 12,
                calibration_capacitor_enabled: bool = 11,
                crx2_enabled: bool = 10,
                crx1_enabled: bool = 9,
                crx0_enabled: bool = 8,
                dead_time_enabled: bool = 6,
                auto_prox_cycle: uint as enum AutoProxCycleCount {
                    Cycles4 = default,
                    Cycles8,
                    Cycles16,
                    Cycles32,
                } = 2..4,
            },
            register PatternDefinitions {
                type Access = ReadWrite;
                const ADDRESS = 0x04;
                const SIZE_BITS = 16;

                wav_pattern_1: uint = 12..16,
                wav_pattern_0: uint = 8..12,
                calibration_cap: uint = 4..8,
                inactive_rxs: uint as try enum InactiveCxState {
                    Floating = 0x00,
                    BiasVolatage = 0x05,
                    Vss = 0x0a,
                    Vreg = 0x0f,
                } = 0..4,
            },
            register PatternSelectionAndEngineBiasCurrent {
                type Access = ReadWrite;
                const ADDRESS = 0x05;
                const SIZE_BITS = 16;

                engine_bias_current: int = 12..16,
                engine_bias_current_trim: int = 8..12,
                wav_pattern_select: uint = 0..8,
            },
            register AtiSetup {
                type Access = ReadWrite;
                const ADDRESS = 0x06;
                const SIZE_BITS = 16;

                resolution_factor: uint = 4..16,
                band: uint as enum AtiBand {
                    Small = default,
                    Large,
                } = 3..4,
                mode: uint as try enum AtiMode {
                    Disabled = default,
                    CompensationOnly,
                    FromCompensationDivider,
                    FromFineFractionalDivider,
                    Full = 4,
                } = 0..3,
            },
            register AtiBase {
                type Access = ReadWrite;
                const ADDRESS = 0x07;
                const SIZE_BITS = 16;

                value: uint = 0..16,
            },
            register AtiMultipliers {
                type Access = ReadWrite;
                const ADDRESS = 0x08;
                const SIZE_BITS = 16;

                fine_fractional_multiplier: uint = 14..16,
                fine_fractional_divider: uint = 9..14,
                coarse_fractional_multiplier: uint = 5..9,
                coarse_fractional_divider: uint = 0..5,
            },
            register Compensation {
                type Access = ReadWrite;
                const ADDRESS = 0x09;
                const SIZE_BITS = 16;

                divider: uint = 11..16,
                value: uint = 0..10,
            },
        },
        block Channel {
            const ADDRESS_OFFSET = 0x60;
            const REPEAT = { count: 3, stride: 0x10 };

            register ChannelSetup {
                type Access = ReadWrite;
                const ADDRESS = 0x00;
                const SIZE_BITS = 16;

                follower_event_mask: uint = 8..16,
                reference_sensor_id: uint = 4..8,
                channel_mode: uint as try enum ChannelMode {
                    Independent,
                    Follower,
                    Reference,
                } = 0..4,
            },
            register ProxSettings {
                type Access = ReadWrite;
                const ADDRESS = 0x01;
                const SIZE_BITS = 16;

                debounce_exit: uint = 12..16,
                debounce_enter: uint = 8..12,
                threshold: uint = 0..8,
            },
            register TouchSettings {
                type Access = ReadWrite;
                const ADDRESS = 0x02;
                const SIZE_BITS = 16;

                hysteresis: uint = 12..16,
                threshold: uint = 0..8,
            },
            register FollowerWeight {
                type Access = ReadWrite;
                const ADDRESS = 0x03;
                const SIZE_BITS = 16;

                value: uint = 0..16,
            },
            #[cfg(feature = "movement-ui")]
            register MovementUiSettings {
                type Access = ReadWrite;
                const ADDRESS = 0x04;
                const SIZE_BITS = 16;

                debounce_exit: uint = 12..16,
                debounce_enter: uint = 8..12,
                threshold: uint = 0..8,
            },
        },
        block SliderConfig {
            register SliderSetup {
                type Access = ReadWrite;
                const ADDRESS = 0x90;
                const SIZE_BITS = 16;

                lower_calibration_value: uint = 8..16,
                static_filter: uint as enum SliderStaticFilter {
                    Dynamic = default,
                    SlowStaticBeta,
                } = 6..7,
                slow_static_beta: uint = 3..6,
                total_channels: uint = 0..3,
            },
            register SliderBottomSpeed {
                type Access = ReadWrite;
                const ADDRESS = 0x91;
                const SIZE_BITS = 16;

                bottom_speed: uint = 8..16,
                upper_calibration_value: uint = 0..8,
            },
            register SliderTopSpeed {
                type Access = ReadWrite;
                const ADDRESS = 0x92;
                const SIZE_BITS = 16;

                value: uint = 0..16,
            },
            register SliderResolution {
                type Access = ReadWrite;
                const ADDRESS = 0x93;
                const SIZE_BITS = 16;

                value: uint = 0..16,
            },
            register SliderEnableMask {
                type Access = ReadWrite;
                const ADDRESS = 0x94;
                const SIZE_BITS = 16;

                ch2_enabled: bool = 2,
                ch1_enabled: bool = 1,
                ch0_enabled: bool = 0,
            },
            register DeltaLink0 {
                type Access = ReadWrite;
                const ADDRESS = 0x96;
                const SIZE_BITS = 16;

                value: uint as try DeltaLinkChannel = 0..16,
            },
            register DeltaLink1 {
                type Access = ReadWrite;
                const ADDRESS = 0x97;
                const SIZE_BITS = 16;

                value: uint as try DeltaLinkChannel = 0..16,
            },
            register DeltaLink2 {
                type Access = ReadWrite;
                const ADDRESS = 0x98;
                const SIZE_BITS = 16;

                value: uint as try DeltaLinkChannel = 0..16,
            },
            #[cfg(not(feature = "movement-ui"))]
            register SliderEnableStatusPointer {
                type Access = ReadWrite;
                const ADDRESS = 0x95;
                const SIZE_BITS = 16;
                const RESET_VALUE = 0x0558;
                const ALLOW_ADDRESS_OVERLAP = true;

                value: uint = 0..16,
            },
            #[cfg(feature = "movement-ui")]
            register SliderEnableStatusPointer {
                type Access = ReadWrite;
                const ADDRESS = 0x95;
                const SIZE_BITS = 16;
                const RESET_VALUE = 0x0552;
                const ALLOW_ADDRESS_OVERLAP = true;

                value: uint = 0..16,
            },
        },
        block GestureConfig {
            register GestureEnable {
                type Access = ReadWrite;
                const ADDRESS = 0xa0;
                const SIZE_BITS = 16;

                hold_enabled: bool = 3,
                flick_enabled: bool = 2,
                swipe_enabled: bool = 1,
                tap_enabled: bool = 0,
            },
            register MinimumGestureTime {
                type Access = ReadWrite;
                const ADDRESS = 0xa1;
                const SIZE_BITS = 16;

                ms: uint = 0..16,
            },
            register MaximumTapTime {
                type Access = ReadWrite;
                const ADDRESS = 0xa2;
                const SIZE_BITS = 16;

                ms: uint = 0..16,
            },
            register MaximumSwipeTime {
                type Access = ReadWrite;
                const ADDRESS = 0xa3;
                const SIZE_BITS = 16;

                ms: uint = 0..16,
            },
            register MinimumHoldTime {
                type Access = ReadWrite;
                const ADDRESS = 0xa4;
                const SIZE_BITS = 16;

                ms: uint = 0..16,
            },
            register MaximumTapDistance {
                type Access = ReadWrite;
                const ADDRESS = 0xa5;
                const SIZE_BITS = 16;

                value: uint = 0..16,
            },
            register MinimumSwipeDistance {
                type Access = ReadWrite;
                const ADDRESS = 0xa6;
                const SIZE_BITS = 16;

                value: uint = 0..16,
            },
        },
        block FilterBetas {
            register Counts {
                type Access = ReadWrite;
                const ADDRESS = 0xb0;
                const SIZE_BITS = 16;

                low_power: uint = 8..16,
                normal_power: uint = 0..8,
            },
            register Lta {
                type Access = ReadWrite;
                const ADDRESS = 0xb1;
                const SIZE_BITS = 16;

                low_power: uint = 8..16,
                normal_power: uint = 0..8,
            },
            register LtaFast {
                type Access = ReadWrite;
                const ADDRESS = 0xb2;
                const SIZE_BITS = 16;

                low_power: uint = 8..16,
                normal_power: uint = 0..8,
            },
            register FastFilterBand {
                type Access = ReadWrite;
                const ADDRESS = 0xb4;
                const SIZE_BITS = 16;

                value: uint = 0..16,
            },
            #[cfg(not(feature = "movement-ui"))]
            register ActivationLta {
                type Access = ReadWrite;
                const ADDRESS = 0xb3;
                const SIZE_BITS = 16;
                const ALLOW_ADDRESS_OVERLAP = true;

                low_power: uint = 8..16,
                normal_power: uint = 0..8,
            },
            #[cfg(feature = "movement-ui")]
            register MovementLta {
                type Access = ReadWrite;
                const ADDRESS = 0xb3;
                const SIZE_BITS = 16;
                const ALLOW_ADDRESS_OVERLAP = true;

                low_power: uint = 8..16,
                normal_power: uint = 0..8,
            },
        },
        block SysControl {
            register Control {
                type Access = ReadWrite;
                const ADDRESS = 0xc0;
                const SIZE_BITS = 16;

                ch2_timeout_disabled: bool = 10,
                ch1_timeout_disabled: bool = 9,
                ch0_timeout_disabled: bool = 8,
                interface: uint as enum InterfaceSelection {
                    I2cStreaming = default,
                    I2cEvents,
                } = 7..8,
                power_mode: uint as try enum PowerMode {
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
                type Access = ReadWrite;
                const ADDRESS = 0xc1;
                const SIZE_BITS = 16;

                ms: uint = 0..16,
            },
            register LowPowerReportRate {
                type Access = ReadWrite;
                const ADDRESS = 0xc2;
                const SIZE_BITS = 16;

                ms: uint = 0..16,
            },
            register UltraLowPowerReportRate {
                type Access = ReadWrite;
                const ADDRESS = 0xc3;
                const SIZE_BITS = 16;

                ms: uint = 0..16,
            },
            register HaltReportRate {
                type Access = ReadWrite;
                const ADDRESS = 0xc4;
                const SIZE_BITS = 16;

                ms: uint = 0..16,
            },
            register PowerModeTimeout {
                type Access = ReadWrite;
                const ADDRESS = 0xc5;
                const SIZE_BITS = 16;

                ms: uint = 0..16,
            },
        },
        block General {
            register OutAMask {
                type Access = ReadWrite;
                const ADDRESS = 0xd0;
                const SIZE_BITS = 16;

                value: uint = 0..16,
            },
            register TransactionTimeout {
                type Access = ReadWrite;
                const ADDRESS = 0xd1;
                const SIZE_BITS = 16;

                ms: uint = 0..16,
            },
            register EventTimeouts {
                type Access = ReadWrite;
                const ADDRESS = 0xd2;
                const SIZE_BITS = 16;

                touch: uint = 8..16,
                prox: uint = 0..8,
            },
            #[cfg(not(feature = "movement-ui"))]
            register EventsEnable {
                type Access = ReadWrite;
                const ADDRESS = 0xd3;
                const SIZE_BITS = 16;
                const ALLOW_ADDRESS_OVERLAP = true;

                activation_setting_threshold: uint = 8..16,
                ati_error: bool = 6,
                ati_event: bool = 4,
                power_event: bool = 3,
                slider_event: bool = 2,
                touch_event: bool = 1,
                prox_event: bool = 0,
            },
            #[cfg(feature = "movement-ui")]
            register EventsEnable {
                type Access = ReadWrite;
                const ADDRESS = 0xd3;
                const SIZE_BITS = 16;
                const ALLOW_ADDRESS_OVERLAP = true;

                ati_error: bool = 6,
                ati_event: bool = 4,
                power_event: bool = 3,
                slider_event: bool = 2,
                touch_event: bool = 1,
                prox_event: bool = 0,
            },
            #[cfg(not(feature = "movement-ui"))]
            register ReleaseUiSettings {
                type Access = ReadWrite;
                const ADDRESS = 0xd4;
                const SIZE_BITS = 16;
                const ALLOW_ADDRESS_OVERLAP = true;

                delta_snapshot_sample_delay: uint = 8..16,
                release_delta_percentage: uint = 0..8,
            },
            #[cfg(feature = "movement-ui")]
            register MovementTimeout {
                type Access = ReadWrite;
                const ADDRESS = 0xd4;
                const SIZE_BITS = 16;
                const ALLOW_ADDRESS_OVERLAP = true;

                value: uint = 0..16,
            },
        },
        block InterfaceSettings {
            register I2c {
                type Access = ReadWrite;
                const ADDRESS = 0xe0;
                const SIZE_BITS = 16;

                rw_check_disabled: bool = 1,
                stop_bit_disabled: bool = 0,
            },
            register HardwareId {
                type Access = ReadOnly;
                const ADDRESS = 0xe1;
                const SIZE_BITS = 16;

                value: uint = 0..16,
            },
        }
    }
);

cfg_if::cfg_if! {
    if #[cfg(feature = "movement-ui")] {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, num_enum::IntoPrimitive, num_enum::TryFromPrimitive)]
        #[repr(u16)]
        pub enum DeltaLinkChannel {
            Disabled,
            Channel0 = 0x430,
            Channel1 = 0x474,
            Channel2 = 0x4b8,
        }
    } else {
        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, num_enum::IntoPrimitive, num_enum::TryFromPrimitive)]
        #[repr(u16)]
        pub enum DeltaLinkChannel {
            Disabled,
            Channel0 = 0x430,
            Channel1 = 0x472,
            Channel2 = 0x4b4,
        }
    }
}
