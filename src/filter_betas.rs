use super::*;

pub struct FilterBetaRegisters;
type FilterBetas<'a, D> = RegisterBlock<'a, FilterBetaRegisters, D>;

impl<D> Iqs323<D> {
    pub fn filter_betas(&mut self) -> FilterBetas<D> {
        RegisterBlock {
            iqs323: self,
            phantom: PhantomData,
        }
    }
}

device_driver::implement_device!(
    impl<'a, D> FilterBetas<'a, D> {
        register Counts {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0xb0;
            const SIZE_BITS: usize = 16;

            low_power: u8 = 8..16,
            normal_power: u8 = 0..8,
        },
        register Lta {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0xb1;
            const SIZE_BITS: usize = 16;

            low_power: u8 = 8..16,
            normal_power: u8 = 0..8,
        },
        register LtaFast {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0xb2;
            const SIZE_BITS: usize = 16;

            low_power: u8 = 8..16,
            normal_power: u8 = 0..8,
        },
        register FastFilterBand {
            type RWType = ReadWrite;
            const ADDRESS: u8 = 0xb4;
            const SIZE_BITS: usize = 16;

            value: u16 = 0..16,
        },
    }
);

cfg_if::cfg_if! {
    if #[cfg(feature = "movement-ui")] {
        device_driver::implement_device!(
            impl<'a, D> FilterBetas<'a, D> {
                register MovementLta {
                    type RWType = ReadWrite;
                    const ADDRESS: u8 = 0xb3;
                    const SIZE_BITS: usize = 16;

                    low_power: u8 = 8..16,
                    normal_power: u8 = 0..8,
                },
            }
        );
    } else {
        device_driver::implement_device!(
            impl<'a, D> FilterBetas<'a, D> {
                register ActivationLta {
                    type RWType = ReadWrite;
                    const ADDRESS: u8 = 0xb3;
                    const SIZE_BITS: usize = 16;

                    low_power: u8 = 8..16,
                    normal_power: u8 = 0..8,
                },
            }
        );
    }
}
