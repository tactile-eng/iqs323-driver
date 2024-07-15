#![no_std]
// #![warn(missing_docs)]

//! An embedded async driver for the IQS323 capacitive/inductive sensing controller.

use core::marker::PhantomData;

use bitvec::array::BitArray;
use device_driver::{bitvec, AsyncRegisterDevice, RegisterDevice};
use embedded_hal::i2c::{I2c, Operation};
use embedded_hal_async::i2c::I2c as AsyncI2c;

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

pub struct Iqs323<D> {
    i2c: D,
}

impl<D> Iqs323<D> {
    pub fn new(i2c: D) -> Self {
        Iqs323 { i2c }
    }
}

impl<D: I2c> RegisterDevice for Iqs323<D> {
    type Error = D::Error;

    type AddressType = u8;

    fn write_register<R, const SIZE_BYTES: usize>(
        &mut self,
        data: &BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error>
    where
        R: device_driver::Register<SIZE_BYTES, AddressType = Self::AddressType>,
    {
        self.i2c.transaction(
            ADDR,
            &mut [
                Operation::Write(&[R::ADDRESS]),
                Operation::Write(data.as_raw_slice()),
            ],
        )
    }

    fn read_register<R, const SIZE_BYTES: usize>(
        &mut self,
        data: &mut BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error>
    where
        R: device_driver::Register<SIZE_BYTES, AddressType = Self::AddressType>,
    {
        self.i2c.transaction(
            ADDR,
            &mut [
                Operation::Write(&[R::ADDRESS]),
                Operation::Read(data.as_raw_mut_slice()),
            ],
        )
    }
}

impl<D: AsyncI2c> AsyncRegisterDevice for Iqs323<D> {
    type Error = D::Error;

    type AddressType = u8;

    async fn write_register<R, const SIZE_BYTES: usize>(
        &mut self,
        data: &BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error>
    where
        R: device_driver::Register<SIZE_BYTES, AddressType = Self::AddressType>,
    {
        self.i2c
            .transaction(
                ADDR,
                &mut [
                    Operation::Write(&[R::ADDRESS]),
                    Operation::Write(data.as_raw_slice()),
                ],
            )
            .await
    }

    async fn read_register<R, const SIZE_BYTES: usize>(
        &mut self,
        data: &mut BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error>
    where
        R: device_driver::Register<SIZE_BYTES, AddressType = Self::AddressType>,
    {
        self.i2c
            .transaction(
                ADDR,
                &mut [
                    Operation::Write(&[R::ADDRESS]),
                    Operation::Read(data.as_raw_mut_slice()),
                ],
            )
            .await
    }
}

pub struct RegisterBlock<'a, T, D, const BASE_ADDR: u8 = 0> {
    iqs323: &'a mut Iqs323<D>,
    phantom: PhantomData<T>,
}

impl<'a, T, D: I2c, const BASE_ADDR: u8> RegisterDevice for RegisterBlock<'a, T, D, BASE_ADDR> {
    type Error = <Iqs323<D> as RegisterDevice>::Error;

    type AddressType = <Iqs323<D> as RegisterDevice>::AddressType;

    fn write_register<R, const SIZE_BYTES: usize>(
        &mut self,
        data: &BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error>
    where
        R: device_driver::Register<SIZE_BYTES, AddressType = Self::AddressType>,
    {
        self.iqs323.i2c.transaction(
            ADDR,
            &mut [
                Operation::Write(&[BASE_ADDR + R::ADDRESS]),
                Operation::Write(data.as_raw_slice()),
            ],
        )
    }

    fn read_register<R, const SIZE_BYTES: usize>(
        &mut self,
        data: &mut BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error>
    where
        R: device_driver::Register<SIZE_BYTES, AddressType = Self::AddressType>,
    {
        self.iqs323.i2c.transaction(
            ADDR,
            &mut [
                Operation::Write(&[BASE_ADDR + R::ADDRESS]),
                Operation::Read(data.as_raw_mut_slice()),
            ],
        )
    }
}

impl<'a, T, D: AsyncI2c, const BASE_ADDR: u8> AsyncRegisterDevice
    for RegisterBlock<'a, T, D, BASE_ADDR>
{
    type Error = <Iqs323<D> as AsyncRegisterDevice>::Error;

    type AddressType = <Iqs323<D> as AsyncRegisterDevice>::AddressType;

    async fn write_register<R, const SIZE_BYTES: usize>(
        &mut self,
        data: &BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error>
    where
        R: device_driver::Register<SIZE_BYTES, AddressType = Self::AddressType>,
    {
        self.iqs323
            .i2c
            .transaction(
                ADDR,
                &mut [
                    Operation::Write(&[BASE_ADDR + R::ADDRESS]),
                    Operation::Write(data.as_raw_slice()),
                ],
            )
            .await
    }

    async fn read_register<R, const SIZE_BYTES: usize>(
        &mut self,
        data: &mut BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error>
    where
        R: device_driver::Register<SIZE_BYTES, AddressType = Self::AddressType>,
    {
        self.iqs323
            .i2c
            .transaction(
                ADDR,
                &mut [
                    Operation::Write(&[BASE_ADDR + R::ADDRESS]),
                    Operation::Read(data.as_raw_mut_slice()),
                ],
            )
            .await
    }
}
