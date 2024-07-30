//! Implementation of the [I2cSettings] register block
use super::*;

pub struct I2cSettings<'block, D> {
    pub(super) device: &'block mut D,
    pub(super) base_address: u8,
}

impl<'block, D> I2cSettings<'block, D> {
    pub fn new(device: &'block mut D, base_address: u8) -> Self {
        Self {
            device,
            base_address,
        }
    }
}

impl<'block, D> ::device_driver::AddressableDevice for I2cSettings<'block, D> {
    type AddressType = u8;
}

impl<'block, D> ::device_driver::RegisterDevice for I2cSettings<'block, D>
where
    D: ::device_driver::RegisterDevice<AddressType = u8>,
{
    type Error = <D as ::device_driver::RegisterDevice>::Error;
    fn write_register<const SIZE_BYTES: usize>(
        &mut self,
        address: Self::AddressType,
        data: &BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error> {
        self.device
            .write_register(address + self.base_address, data)
    }
    fn read_register<const SIZE_BYTES: usize>(
        &mut self,
        address: Self::AddressType,
        data: &mut BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error> {
        self.device.read_register(address + self.base_address, data)
    }
}

impl<'block, D> ::device_driver::AsyncRegisterDevice for I2cSettings<'block, D>
where
    D: ::device_driver::AsyncRegisterDevice<AddressType = u8>,
{
    type Error = <D as ::device_driver::AsyncRegisterDevice>::Error;
    async fn write_register<const SIZE_BYTES: usize>(
        &mut self,
        address: Self::AddressType,
        data: &BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error> {
        self.device
            .write_register(address + self.base_address, data)
            .await
    }
    async fn read_register<const SIZE_BYTES: usize>(
        &mut self,
        address: Self::AddressType,
        data: &mut BitArray<[u8; SIZE_BYTES]>,
    ) -> Result<(), Self::Error> {
        self.device
            .read_register(address + self.base_address, data)
            .await
    }
}

device_driver::implement_device! {
    impl<'block, D> I2cSettings<'block, D> {
        register Settings {
            type RWType = ReadWrite;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xe0;
            const SIZE_BITS: usize = 16;

            rw_check_disabled: bool = 1,
            stop_bit_disabled: bool = 0,
        },
        register HardwareId {
            type RWType = ReadOnly;
            type ByteOrder = LE;
            const ADDRESS: u8 = 0xe1;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
    }
}
