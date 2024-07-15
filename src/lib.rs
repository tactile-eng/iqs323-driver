#![no_std]
// #![warn(missing_docs)]

//! An embedded async driver for the IQS323 capacitive/inductive sensing controller.

use core::marker::PhantomData;

use bitvec::array::BitArray;
use device_driver::{bitvec, AsyncRegisterDevice};
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

impl<D: I2c, P: InputPin> Iqs323<D, P> {
    pub async fn force_comms(&mut self) -> Result<(), Error<D::Error, P::Error>> {
        if self.rdy.is_high().map_err(Error::Io)? {
            self.i2c.write(ADDR, &[0xff]).await.map_err(Error::I2c)
        } else {
            Ok(())
        }
    }

    pub async fn terminate_comms(&mut self) -> Result<(), Error<D::Error, P::Error>> {
        if self.rdy.is_low().map_err(Error::Io)? {
            self.i2c.write(ADDR, &[0xff]).await.map_err(Error::I2c)
        } else {
            Ok(())
        }
    }
}

impl<D: I2c, P: Wait> AsyncRegisterDevice for Iqs323<D, P> {
    type Error = Error<D::Error, P::Error>;

    type AddressType = u8;

    async fn write_register<R, const SIZE_BYTES: usize>(
        &mut self,
        data: &BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error>
    where
        R: device_driver::Register<SIZE_BYTES, AddressType = Self::AddressType>,
    {
        const { core::assert!(SIZE_BYTES == 2) };
        let buf = [R::ADDRESS, data.as_raw_slice()[0], data.as_raw_slice()[1]];
        self.rdy.wait_for_low().await.map_err(Error::Io)?;
        self.i2c.write(ADDR, &buf).await.map_err(Error::I2c)
    }

    async fn read_register<R, const SIZE_BYTES: usize>(
        &mut self,
        data: &mut BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error>
    where
        R: device_driver::Register<SIZE_BYTES, AddressType = Self::AddressType>,
    {
        self.rdy.wait_for_low().await.map_err(Error::Io)?;
        self.i2c
            .write_read(ADDR, &[R::ADDRESS], data.as_raw_mut_slice())
            .await
            .map_err(Error::I2c)
    }
}

pub struct RegisterBlock<'a, T, D, P, const BASE_ADDR: u8 = 0> {
    iqs323: &'a mut Iqs323<D, P>,
    phantom: PhantomData<T>,
}

impl<'a, T, D: I2c, P: Wait, const BASE_ADDR: u8> AsyncRegisterDevice
    for RegisterBlock<'a, T, D, P, BASE_ADDR>
{
    type Error = Error<D::Error, P::Error>;

    type AddressType = u8;

    async fn write_register<R, const SIZE_BYTES: usize>(
        &mut self,
        data: &BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error>
    where
        R: device_driver::Register<SIZE_BYTES, AddressType = Self::AddressType>,
    {
        const { core::assert!(SIZE_BYTES == 2) };
        let buf = [
            BASE_ADDR + R::ADDRESS,
            data.as_raw_slice()[0],
            data.as_raw_slice()[1],
        ];
        self.iqs323.rdy.wait_for_low().await.map_err(Error::Io)?;
        self.iqs323.i2c.write(ADDR, &buf).await.map_err(Error::I2c)
    }

    async fn read_register<R, const SIZE_BYTES: usize>(
        &mut self,
        data: &mut BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error>
    where
        R: device_driver::Register<SIZE_BYTES, AddressType = Self::AddressType>,
    {
        self.iqs323.rdy.wait_for_low().await.map_err(Error::Io)?;
        self.iqs323
            .i2c
            .write_read(ADDR, &[BASE_ADDR + R::ADDRESS], data.as_raw_mut_slice())
            .await
            .map_err(Error::I2c)
    }
}
