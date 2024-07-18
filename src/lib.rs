#![no_std]
// #![warn(missing_docs)]

//! An embedded async driver for the IQS323 capacitive/inductive sensing controller.

use bitvec::array::BitArray;
use device_driver::{bitvec, AddressableDevice, AsyncRegisterDevice};
use embedded_hal::digital::{self, InputPin};
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::{self, I2c};

pub mod channel_setup;
pub mod filter_betas;
pub mod general;
pub mod gesture_config;
pub mod i2c_settings;
pub mod movement_ui;
pub mod release_ui;
pub mod sensor_setup;
pub mod slider_config;
pub mod sys_control;
pub mod sys_info;

const ADDR: u8 = 0x44;

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Error<D: i2c::Error, P: digital::Error> {
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

cfg_if::cfg_if! {
    if #[cfg(feature = "movement-ui")] {
        device_driver::implement_device!(
            impl<D, P> Iqs323<D, P> {
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
            }
        );
    } else {
        device_driver::implement_device!(
            impl<D, P> Iqs323<D, P> {
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
            }
        );
    }
}

impl<D, P> Iqs323<D, P> {
    pub fn sys_info(&mut self) -> sys_info::SysInfo<D, P> {
        sys_info::SysInfo(self)
    }

    #[cfg(not(feature = "movement-ui"))]
    pub fn ui(&mut self) -> release_ui::ReleaseUi<D, P> {
        release_ui::ReleaseUi(self)
    }
    #[cfg(feature = "movement-ui")]
    pub fn ui(&mut self) -> movement_ui::MovementUi<D, P> {
        movement_ui::MovementUi(self)
    }

    pub fn sensor_0_setup(&mut self) -> sensor_setup::SensorSetup<D, P, 0x30> {
        sensor_setup::SensorSetup(self)
    }

    pub fn sensor_1_setup(&mut self) -> sensor_setup::SensorSetup<D, P, 0x40> {
        sensor_setup::SensorSetup(self)
    }

    pub fn sensor_2_setup(&mut self) -> sensor_setup::SensorSetup<D, P, 0x50> {
        sensor_setup::SensorSetup(self)
    }

    pub fn channel_0_setup(&mut self) -> channel_setup::ChannelSetup<D, P, 0x60> {
        channel_setup::ChannelSetup(self)
    }

    pub fn channel_1_setup(&mut self) -> channel_setup::ChannelSetup<D, P, 0x70> {
        channel_setup::ChannelSetup(self)
    }

    pub fn channel_2_setup(&mut self) -> channel_setup::ChannelSetup<D, P, 0x80> {
        channel_setup::ChannelSetup(self)
    }

    pub fn slider_config(&mut self) -> slider_config::SliderConfig<D, P> {
        slider_config::SliderConfig(self)
    }

    pub fn gesture_config(&mut self) -> gesture_config::GestureConfig<D, P> {
        gesture_config::GestureConfig(self)
    }

    pub fn filter_betas(&mut self) -> filter_betas::FilterBetas<D, P> {
        filter_betas::FilterBetas(self)
    }

    pub fn sys_control(&mut self) -> sys_control::SysControl<D, P> {
        sys_control::SysControl(self)
    }

    pub fn general(&mut self) -> general::General<D, P> {
        general::General(self)
    }

    pub fn i2c(&mut self) -> i2c_settings::I2cSettings<D, P> {
        i2c_settings::I2cSettings(self)
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
        self.i2c.write(ADDR, &buf).await.map_err(Error::I2c)
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
            .map_err(Error::I2c)
    }
}

macro_rules! register_block {
    ($(#[$meta:meta])* $name:ident) => {
        register_block!(TERM $(#[$meta])* $name, 0);
    };
    ($(#[$meta:meta])* $name:ident<$base_addr:ident>) => {
        register_block!(TERM $(#[$meta])* $name, $base_addr, $base_addr);
    };
    (TERM $(#[$meta:meta])* $name:ident, $base_addr:expr$(, $gen:ident)?) => {
        $(#[$meta])*
        pub struct $name<'a, D, P$(, const $gen: u8)?>(pub(crate) &'a mut Iqs323<D, P>);

        impl<'a, D, P$(, const $gen: u8)?> ::device_driver::RegisterBlock for $name<'a, D, P$(, $gen)?> {
            type Device = Iqs323<D, P>;

            const BASE_ADDR: u8 = $base_addr;

            fn dev(&mut self) -> &mut Self::Device {
                self.0
            }
        }
    };
}

pub(crate) use register_block;
