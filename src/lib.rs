#![no_std]
// #![warn(missing_docs)]

//! An embedded async driver for the IQS323 capacitive/inductive sensing controller.

use array_concat::concat_arrays;
use bitvec::array::BitArray;
pub use device_driver::{bitvec, AddressableDevice, AsyncRegisterDevice, Register};
use embedded_hal::digital::{self, InputPin, OutputPin};
use embedded_hal::i2c::Operation;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::{self, I2c};

pub mod i2c_settings;

const ADDR: u8 = 0x44;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Error<D: i2c::Error, P: digital::Error> {
    Reset,
    I2cLockup,
    Io(P),
    I2c(D),
}

pub struct Iqs323<D, P> {
    i2c: D,
    rdy: P,
}

impl<D, P> Iqs323<D, P> {
    pub fn new(i2c: D, rdy: P) -> Self {
        Iqs323 { i2c, rdy }
    }

    pub fn mclr(&mut self) -> &mut P {
        &mut self.rdy
    }
}

impl<D: I2c, P: InputPin> Iqs323<D, P> {
    pub fn i2c_settings(&mut self) -> i2c_settings::I2cSettings<Self> {
        i2c_settings::I2cSettings::new(self, 0)
    }

    /// Send a communication request to the device to force a RDY window to open.
    pub async fn force_comms(&mut self) -> Result<(), Error<D::Error, P::Error>> {
        if self.rdy.is_high().map_err(Error::Io)? {
            self.i2c.write(ADDR, &[0xff]).await.map_err(Error::I2c)
        } else {
            Ok(())
        }
    }

    /// Terminate the communication window when [`stop_bit_disabled`](i2c_settings::settings::W::stop_bit_disabled) is set.
    ///
    /// Normally the communication windows is closed when any I²C transaction completes. However if the `stop_bit_disabled`
    /// bit is set, the device will ignore the I²C STOP and the communication window must be terminated by calling this method.
    pub async fn terminate_comms(&mut self) -> Result<(), Error<D::Error, P::Error>> {
        if self.rdy.is_low().map_err(Error::Io)? {
            self.i2c.write(ADDR, &[0xff]).await.map_err(Error::I2c)
        } else {
            Ok(())
        }
    }
}

impl<D: I2c, P: Wait> Iqs323<D, P> {
    pub async fn reset<T: DelayNs>(
        &mut self,
        delay: &mut T,
    ) -> Result<(), Error<D::Error, P::Error>> {
        while !self
            .sys_info()
            .system_status()
            .read_async()
            .await?
            .reset_event()
        {
            self.sys_control()
                .control()
                .write_async(|w| w.trigger_soft_reset(true))
                .await?;
            delay.delay_ms(100).await;
        }

        Ok(())
    }

    pub async fn ack_reset(&mut self) -> Result<(), Error<D::Error, P::Error>> {
        self.sys_control()
            .control()
            .modify_async(|w| w.ack_reset(true))
            .await
    }

    pub async fn setup(&mut self, setup: &AddressedSetup) -> Result<(), Error<D::Error, P::Error>> {
        self.rdy.wait_for_low().await.map_err(Error::Io)?;
        self.i2c
            .transaction(
                ADDR,
                &mut [
                    Operation::Write(&setup.sensors[0]),
                    Operation::Write(&setup.sensors[1]),
                    Operation::Write(&setup.sensors[2]),
                    Operation::Write(&setup.channels[0]),
                    Operation::Write(&setup.channels[1]),
                    Operation::Write(&setup.channels[2]),
                    Operation::Write(&setup.slider),
                    Operation::Write(&setup.gesture),
                    Operation::Write(&setup.filter_betas),
                    Operation::Write(&setup.system_control),
                    Operation::Write(&setup.general),
                    Operation::Write(&setup.i2c_settings),
                ],
            )
            .await
            .map_err(Error::I2c)?;
        self.rdy.wait_for_high().await.map_err(Error::Io)
    }

    pub async fn read_setup(&mut self) -> Result<Setup, Error<D::Error, P::Error>> {
        let mut setup = Setup::default();
        let (s0, s1, s2) = match &mut setup.sensors {
            [ref mut s0, ref mut s1, ref mut s2] => (s0, s1, s2),
        };
        let (c0, c1, c2) = match &mut setup.channels {
            [ref mut c0, ref mut c1, ref mut c2] => (c0, c1, c2),
        };

        self.rdy.wait_for_low().await.map_err(Error::Io)?;
        self.i2c
            .transaction(
                ADDR,
                &mut [
                    Operation::Write(&[0x30]),
                    Operation::Read(s0),
                    Operation::Write(&[0x40]),
                    Operation::Read(s1),
                    Operation::Write(&[0x50]),
                    Operation::Read(s2),
                    Operation::Write(&[0x60]),
                    Operation::Read(c0),
                    Operation::Write(&[0x70]),
                    Operation::Read(c1),
                    Operation::Write(&[0x80]),
                    Operation::Read(c2),
                    Operation::Write(&[0x90]),
                    Operation::Read(&mut setup.slider),
                    Operation::Write(&[0xa0]),
                    Operation::Read(&mut setup.gesture),
                    Operation::Write(&[0xb0]),
                    Operation::Read(&mut setup.filter_betas),
                    Operation::Write(&[0xc0]),
                    Operation::Read(&mut setup.system_control),
                    Operation::Write(&[0xd0]),
                    Operation::Read(&mut setup.general),
                    Operation::Write(&[0xe0]),
                    Operation::Read(&mut setup.i2c_settings),
                ],
            )
            .await
            .map_err(Error::I2c)?;
        self.rdy.wait_for_high().await.map_err(Error::Io)?;

        Ok(setup)
    }

    pub async fn read_sys_info(&mut self) -> Result<SysInfo, Error<D::Error, P::Error>> {
        let mut buf = [0; 19];
        self.rdy.wait_for_low().await.map_err(Error::Io)?;
        self.i2c
            .write_read(ADDR, &[0x10], &mut buf)
            .await
            .map_err(Error::I2c)?;
        self.rdy.wait_for_high().await.map_err(Error::Io)?;

        if buf[18] != 0xee {
            return Err(Error::I2cLockup);
        }

        let mut system_status = sys_info::SystemStatus::ZERO;
        system_status
            .bits_mut()
            .as_raw_mut_slice()
            .copy_from_slice(&buf[0..2]);
        let system_status = system_status.into_r();

        if system_status.reset_event() {
            return Err(Error::Reset);
        }

        let mut gestures = sys_info::Gestures::ZERO;
        gestures
            .bits_mut()
            .as_raw_mut_slice()
            .copy_from_slice(&buf[2..4]);
        let gestures = gestures.into_r();

        let mut slider_position = sys_info::SliderPosition::ZERO;
        slider_position
            .bits_mut()
            .as_raw_mut_slice()
            .copy_from_slice(&buf[4..6]);
        let slider_position = slider_position.into_r();

        let mut ch0_filtered_counts = sys_info::Ch0FilteredCounts::ZERO;
        ch0_filtered_counts
            .bits_mut()
            .as_raw_mut_slice()
            .copy_from_slice(&buf[6..8]);
        let ch0_filtered_counts = ch0_filtered_counts.into_r();

        let mut ch0_lta = sys_info::Ch0Lta::ZERO;
        ch0_lta
            .bits_mut()
            .as_raw_mut_slice()
            .copy_from_slice(&buf[8..10]);
        let ch0_lta = ch0_lta.into_r();

        let mut ch1_filtered_counts = sys_info::Ch1FilteredCounts::ZERO;
        ch1_filtered_counts
            .bits_mut()
            .as_raw_mut_slice()
            .copy_from_slice(&buf[10..12]);
        let ch1_filtered_counts = ch1_filtered_counts.into_r();

        let mut ch1_lta = sys_info::Ch1Lta::ZERO;
        ch1_lta
            .bits_mut()
            .as_raw_mut_slice()
            .copy_from_slice(&buf[12..14]);
        let ch1_lta = ch1_lta.into_r();

        let mut ch2_filtered_counts = sys_info::Ch2FilteredCounts::ZERO;
        ch2_filtered_counts
            .bits_mut()
            .as_raw_mut_slice()
            .copy_from_slice(&buf[14..16]);
        let ch2_filtered_counts = ch2_filtered_counts.into_r();

        let mut ch2_lta = sys_info::Ch2Lta::ZERO;
        ch2_lta
            .bits_mut()
            .as_raw_mut_slice()
            .copy_from_slice(&buf[16..18]);
        let ch2_lta = ch2_lta.into_r();

        Ok(SysInfo {
            system_status,
            gestures,
            slider_position,
            ch0_filtered_counts,
            ch0_lta,
            ch1_filtered_counts,
            ch1_lta,
            ch2_filtered_counts,
            ch2_lta,
        })
    }
}

impl<D: I2c, P: Wait + OutputPin> Iqs323<D, P> {
    pub async fn hard_reset<T: DelayNs>(&mut self, delay: &mut T) -> Result<(), P::Error> {
        self.rdy.wait_for_high().await?;
        self.rdy.set_low()?;
        delay.delay_us(1).await;
        self.rdy.set_high()?;
        delay.delay_ms(100).await;
        Ok(())
    }
}

impl<D, P> AddressableDevice for Iqs323<D, P> {
    type AddressType = u8;
}

impl<D: I2c, P: Wait> AsyncRegisterDevice for Iqs323<D, P> {
    type Error = Error<D::Error, P::Error>;

    async fn write_register<const SIZE_BYTES: usize>(
        &mut self,
        address: Self::AddressType,
        data: &BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error> {
        const { core::assert!(SIZE_BYTES == 2) };
        let buf = [address, data.as_raw_slice()[0], data.as_raw_slice()[1]];
        self.rdy.wait_for_low().await.map_err(Error::Io)?;
        self.i2c.write(ADDR, &buf).await.map_err(Error::I2c)?;
        self.rdy.wait_for_high().await.map_err(Error::Io)
    }

    async fn read_register<const SIZE_BYTES: usize>(
        &mut self,
        address: Self::AddressType,
        data: &mut BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error> {
        self.rdy.wait_for_low().await.map_err(Error::Io)?;
        self.i2c
            .write_read(ADDR, &[address], data.as_raw_mut_slice())
            .await
            .map_err(Error::I2c)?;
        self.rdy.wait_for_high().await.map_err(Error::Io)
    }
}

device_driver::implement_device! {
    impl<D, P> Iqs323<D, P> {
        #[cfg(feature = "movement-ui")]
        register Version {
            type RWType = ReadOnly;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0;
            const SIZE_BITS: usize = 48;
            const RESET_VALUE: [u8] = [0xb6, 0x05, 0x01, 0x00, 0x04, 0x00];

            product_number: u16 = 0..16,
            major_version: u16 = 16..32,
            minor_version: u16 = 32..48,
        },
        #[cfg(not(feature = "movement-ui"))]
        register Version {
            type RWType = ReadOnly;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0;
            const SIZE_BITS: usize = 48;
            const RESET_VALUE: [u8] = [0x52, 0x04, 0x01, 0x00, 0x03, 0x00];

            product_number: u16 = 0..16,
            major_version: u16 = 16..32,
            minor_version: u16 = 32..48,
        },
        block SysInfo {
            register SystemStatus {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x10;
                const SIZE_BITS: usize = 16;

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
                current_power_mode: u8 as strict enum PowerMode {
                    Normal = "default",
                    Low = 1,
                    UltraLow = 2,
                    Halt = 3,
                } = 14..16,
            },
            register Gestures {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x11;
                const SIZE_BITS: usize = 16;

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
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x12;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
            register Ch0FilteredCounts {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x13;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
            register Ch0Lta {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x14;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
            register Ch1FilteredCounts {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x15;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
            register Ch1Lta {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x16;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
            register Ch2FilteredCounts {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x17;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
            register Ch2Lta {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x18;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
        },
        #[cfg(not(feature = "movement-ui"))]
        block ReleaseUi {
            register Ch0ActivationLta {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x20;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
            register Ch1ActivationLta {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x21;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
            register Ch2ActivationLta {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x22;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
            register Ch0DeltaSnapshot {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x23;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
            register Ch1DeltaSnapshot {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x24;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
            register Ch2DeltaSnapshot {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x25;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
        },
        #[cfg(feature = "movement-ui")]
        block MovementUi {
            register Ch0MovementLta {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x20;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
            register Ch1MovementLta {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x21;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
            register Ch2MovementLta {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x22;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
            register MovementStatus {
                type RWType = ReadOnly;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x23;
                const SIZE_BITS: usize = 16;

                ch0_movement: bool = 0,
                ch1_movement: bool = 1,
                ch2_movement: bool = 2,
            },
        },
        block SensorSetup {
            const BASE_ADDRESS: u8 = 0x30;
            const REPEAT = { count: 3, stride: 0x10 };

            register Setup {
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x00;
                const SIZE_BITS: usize = 16;

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
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x01;
                const SIZE_BITS: usize = 16;

                period: u8 = 8..16,
                frac: u8 = 0..8,
            },
            register ProxControl {
                type RWType = ReadWrite;
                type ByteOrder = LE;
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
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x03;
                const SIZE_BITS: usize = 16;
                const RESET_VALUE: [u8] = [0x83, 0x00];

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
                type ByteOrder = LE;
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
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x05;
                const SIZE_BITS: usize = 16;

                engine_bias_current: i8 = 12..16,
                engine_bias_current_trim: i8 = 8..12,
                wav_pattern_select: u8 = 0..8,
            },
            register AtiSetup {
                type RWType = ReadWrite;
                type ByteOrder = LE;
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
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x07;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
            register AtiMultipliers {
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x08;
                const SIZE_BITS: usize = 16;

                fine_fractional_multiplier: u8 = 14..16,
                fine_fractional_divider: u8 = 9..14,
                coarse_fractional_multiplier: u8 = 5..9,
                coarse_fractional_divider: u8 = 0..5,
            },
            register Compensation {
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x09;
                const SIZE_BITS: usize = 16;

                divider: u8 = 11..16,
                value: u16 = 0..10,
            },
        },
        block ChannelSetup {
            const BASE_ADDRESS: u8 = 0x60;
            const REPEAT = { count: 3, stride: 0x10 };

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
        },
        block SliderConfig {
            register SliderSetup {
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x90;
                const SIZE_BITS: usize = 16;

                lower_calibration_value: u8 = 8..16,
                static_filter: u8 as strict enum SliderStaticFilter {
                    Dynamic = "default",
                    SlowStaticBeta,
                } = 6..7,
                slow_static_beta: u8 = 3..6,
                total_channels: u8 = 0..3,
            },
            register SliderBottomSpeed {
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x91;
                const SIZE_BITS: usize = 16;

                bottom_speed: u8 = 8..16,
                upper_calibration_value: u8 = 0..8,
            },
            register SliderTopSpeed {
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x92;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
            register SliderResolution {
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x93;
                const SIZE_BITS: usize = 16;

                value: u16 = 0..16,
            },
            register SliderEnableMask {
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x94;
                const SIZE_BITS: usize = 16;

                ch2_enabled: bool = 2,
                ch1_enabled: bool = 1,
                ch0_enabled: bool = 0,
            },
            register DeltaLink0 {
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x96;
                const SIZE_BITS: usize = 16;

                value: u16 as DeltaLinkChannel = 0..16,
            },
            register DeltaLink1 {
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x96;
                const SIZE_BITS: usize = 16;

                value: u16 as DeltaLinkChannel = 0..16,
            },
            register DeltaLink2 {
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x96;
                const SIZE_BITS: usize = 16;

                value: u16 as DeltaLinkChannel = 0..16,
            },
            #[cfg(not(feature = "movement-ui"))]
            register SliderEnableStatusPointer {
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x95;
                const SIZE_BITS: usize = 16;
                const RESET_VALUE: [u8] = [0x58, 0x05];

                value: u16 = 0..16,
            },
            #[cfg(feature = "movement-ui")]
            register SliderEnableStatusPointer {
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0x95;
                const SIZE_BITS: usize = 16;
                const RESET_VALUE: [u8] = [0x52, 0x05];

                value: u16 = 0..16,
            },
        },
        block GestureConfig {
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
        },
        block FilterBetas {
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
        },
        block SysControl {
            register Control {
                type RWType = ReadWrite;
                type ByteOrder = LE;
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
                type ByteOrder = LE;
                const ADDRESS: u8 = 0xc1;
                const SIZE_BITS: usize = 16;

                ms: u16 = 0..16,
            },
            register LowPowerReportRate {
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0xc2;
                const SIZE_BITS: usize = 16;

                ms: u16 = 0..16,
            },
            register UltraLowPowerReportRate {
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0xc3;
                const SIZE_BITS: usize = 16;

                ms: u16 = 0..16,
            },
            register HaltReportRate {
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0xc4;
                const SIZE_BITS: usize = 16;

                ms: u16 = 0..16,
            },
            register PowerModeTimeout {
                type RWType = ReadWrite;
                type ByteOrder = LE;
                const ADDRESS: u8 = 0xc5;
                const SIZE_BITS: usize = 16;

                ms: u16 = 0..16,
            },
        },
        block General {
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
        },
    }
}

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

#[derive(Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Setup {
    pub sensors: [[u8; 20]; 3],
    pub channels: [[u8; 8]; 3],
    pub slider: [u8; 18],
    pub gesture: [u8; 14],
    pub filter_betas: [u8; 10],
    pub system_control: [u8; 12],
    pub general: [u8; 10],
    pub i2c_settings: [u8; 1],
}

impl Setup {
    pub const fn addressed(self) -> AddressedSetup {
        AddressedSetup::from_setup(self)
    }
}

#[derive(Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct AddressedSetup {
    sensors: [[u8; 21]; 3],
    channels: [[u8; 9]; 3],
    slider: [u8; 19],
    gesture: [u8; 15],
    filter_betas: [u8; 11],
    system_control: [u8; 13],
    general: [u8; 11],
    i2c_settings: [u8; 2],
}

impl AddressedSetup {
    pub const fn from_setup(setup: Setup) -> Self {
        AddressedSetup {
            sensors: [
                concat_arrays!([0x30], setup.sensors[0]),
                concat_arrays!([0x40], setup.sensors[1]),
                concat_arrays!([0x50], setup.sensors[2]),
            ],
            channels: [
                concat_arrays!([0x60], setup.channels[0]),
                concat_arrays!([0x70], setup.channels[1]),
                concat_arrays!([0x80], setup.channels[2]),
            ],
            slider: concat_arrays!([0x90], setup.slider),
            gesture: concat_arrays!([0xa0], setup.gesture),
            filter_betas: concat_arrays!([0xb0], setup.filter_betas),
            system_control: concat_arrays!([0xc0], setup.system_control),
            general: concat_arrays!([0xd0], setup.general),
            i2c_settings: concat_arrays!([0xe0], setup.i2c_settings),
        }
    }
}

impl From<Setup> for AddressedSetup {
    fn from(value: Setup) -> Self {
        Self::from_setup(value)
    }
}

pub struct SysInfo {
    pub system_status: sys_info::system_status::R,
    pub gestures: sys_info::gestures::R,
    pub slider_position: sys_info::slider_position::R,
    pub ch0_filtered_counts: sys_info::ch_0_filtered_counts::R,
    pub ch0_lta: sys_info::ch_0_lta::R,
    pub ch1_filtered_counts: sys_info::ch_1_filtered_counts::R,
    pub ch1_lta: sys_info::ch_1_lta::R,
    pub ch2_filtered_counts: sys_info::ch_2_filtered_counts::R,
    pub ch2_lta: sys_info::ch_2_lta::R,
}
