#![no_std]
// #![warn(missing_docs)]

//! An embedded async driver for the IQS323 capacitive/inductive sensing controller.

use array_concat::concat_arrays;
use embedded_hal::digital::{self, InputPin, OutputPin};
use embedded_hal::i2c::Operation;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::{self, I2c};

pub mod regs;

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
        Self { i2c, rdy }
    }

    pub fn mclr(&mut self) -> &mut P {
        &mut self.rdy
    }
}

impl<D: I2c, P: InputPin> Iqs323<D, P> {
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
    pub fn regs(&mut self) -> regs::Registers<&mut Self> {
        regs::Registers::new(self)
    }

    pub async fn reset<T: DelayNs>(
        &mut self,
        delay: &mut T,
    ) -> Result<(), Error<D::Error, P::Error>> {
        let mut regs = self.regs();
        while !regs
            .sys_info()
            .system_status()
            .read_async()
            .await?
            .reset_event()
        {
            regs.sys_control()
                .control()
                .write_async(|reg| reg.set_trigger_soft_reset(true))
                .await?;
            delay.delay_ms(100).await;
        }

        Ok(())
    }

    pub async fn ack_reset(&mut self) -> Result<(), Error<D::Error, P::Error>> {
        self.regs()
            .sys_control()
            .control()
            .modify_async(|w| w.set_ack_reset(true))
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

        fn as_bytes_mut(slice: &mut [u16]) -> &mut [u8] {
            unsafe { core::slice::from_raw_parts_mut(slice.as_ptr() as *mut u8, 2 * slice.len()) }
        }

        let (s0, s1, s2) = match &mut setup.sensors {
            [s0, s1, s2] => (as_bytes_mut(s0), as_bytes_mut(s1), as_bytes_mut(s2)),
        };
        let (c0, c1, c2) = match &mut setup.channels {
            [c0, c1, c2] => (as_bytes_mut(c0), as_bytes_mut(c1), as_bytes_mut(c2)),
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
                    Operation::Read(as_bytes_mut(&mut setup.slider)),
                    Operation::Write(&[0xa0]),
                    Operation::Read(as_bytes_mut(&mut setup.gesture)),
                    Operation::Write(&[0xb0]),
                    Operation::Read(as_bytes_mut(&mut setup.filter_betas)),
                    Operation::Write(&[0xc0]),
                    Operation::Read(as_bytes_mut(&mut setup.system_control)),
                    Operation::Write(&[0xd0]),
                    Operation::Read(as_bytes_mut(&mut setup.general)),
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

        #[inline]
        fn as_array(slice: &[u8]) -> [u8; 2] {
            slice.try_into().unwrap()
        }

        let system_status = regs::field_sets::SystemStatus::from(as_array(&buf[0..2]));

        if system_status.reset_event() {
            return Err(Error::Reset);
        }

        let gestures = regs::field_sets::Gestures::from(as_array(&buf[2..4]));
        let slider_position = regs::field_sets::SliderPosition::from(as_array(&buf[4..6]));
        let ch0_filtered_counts = regs::field_sets::Ch0FilteredCounts::from(as_array(&buf[6..8]));
        let ch0_lta = regs::field_sets::Ch0Lta::from(as_array(&buf[8..10]));
        let ch1_filtered_counts = regs::field_sets::Ch1FilteredCounts::from(as_array(&buf[10..12]));
        let ch1_lta = regs::field_sets::Ch1Lta::from(as_array(&buf[12..14]));
        let ch2_filtered_counts = regs::field_sets::Ch2FilteredCounts::from(as_array(&buf[14..16]));
        let ch2_lta = regs::field_sets::Ch2Lta::from(as_array(&buf[16..18]));

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

impl<'a, D: I2c, P: Wait> device_driver::AsyncRegisterInterface for &'a mut Iqs323<D, P> {
    type AddressType = u8;
    type Error = Error<D::Error, P::Error>;

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        const MAX_LEN: usize = 3;
        assert!(data.len() < MAX_LEN);
        let mut buf = heapless::Vec::<u8, MAX_LEN>::new();
        let _ = buf.push(address);
        let _ = buf.extend_from_slice(data);
        self.rdy.wait_for_low().await.map_err(Error::Io)?;
        self.i2c.write(ADDR, &buf).await.map_err(Error::I2c)?;
        self.rdy.wait_for_high().await.map_err(Error::Io)
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.rdy.wait_for_low().await.map_err(Error::Io)?;
        self.i2c
            .write_read(ADDR, &[address], data)
            .await
            .map_err(Error::I2c)?;
        self.rdy.wait_for_high().await.map_err(Error::Io)
    }
}

#[derive(Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Setup {
    pub sensors: [[u16; 10]; 3],
    pub channels: [[u16; 4]; 3],
    pub slider: [u16; 9],
    pub gesture: [u16; 7],
    pub filter_betas: [u16; 5],
    pub system_control: [u16; 6],
    pub general: [u16; 5],
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
        const unsafe fn as_bytes<const N: usize, const M: usize>(src: [u16; N]) -> [u8; M] {
            const { assert!(M == 2 * N) }
            *(src.as_ptr() as *const [u8; M])
        }

        unsafe {
            AddressedSetup {
                sensors: [
                    concat_arrays!([0x30], as_bytes::<10, 20>(setup.sensors[0])),
                    concat_arrays!([0x40], as_bytes::<10, 20>(setup.sensors[1])),
                    concat_arrays!([0x50], as_bytes::<10, 20>(setup.sensors[2])),
                ],
                channels: [
                    concat_arrays!([0x60], as_bytes::<4, 8>(setup.channels[0])),
                    concat_arrays!([0x70], as_bytes::<4, 8>(setup.channels[1])),
                    concat_arrays!([0x80], as_bytes::<4, 8>(setup.channels[2])),
                ],
                slider: concat_arrays!([0x90], as_bytes::<9, 18>(setup.slider)),
                gesture: concat_arrays!([0xa0], as_bytes::<7, 14>(setup.gesture)),
                filter_betas: concat_arrays!([0xb0], as_bytes::<5, 10>(setup.filter_betas)),
                system_control: concat_arrays!([0xc0], as_bytes::<6, 12>(setup.system_control)),
                general: concat_arrays!([0xd0], as_bytes::<5, 10>(setup.general)),
                i2c_settings: concat_arrays!([0xe0], setup.i2c_settings),
            }
        }
    }
}

impl From<Setup> for AddressedSetup {
    fn from(value: Setup) -> Self {
        Self::from_setup(value)
    }
}

pub struct SysInfo {
    pub system_status: regs::field_sets::SystemStatus,
    pub gestures: regs::field_sets::Gestures,
    pub slider_position: regs::field_sets::SliderPosition,
    pub ch0_filtered_counts: regs::field_sets::Ch0FilteredCounts,
    pub ch0_lta: regs::field_sets::Ch0Lta,
    pub ch1_filtered_counts: regs::field_sets::Ch1FilteredCounts,
    pub ch1_lta: regs::field_sets::Ch1Lta,
    pub ch2_filtered_counts: regs::field_sets::Ch2FilteredCounts,
    pub ch2_lta: regs::field_sets::Ch2Lta,
}
