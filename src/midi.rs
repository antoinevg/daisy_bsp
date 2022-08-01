#[cfg(any(feature = "alloc"))]
extern crate alloc;
#[cfg(any(feature = "alloc"))]
use alloc::boxed::Box;

use hal::prelude::*;
use hal::rcc;
pub use stm32h7xx_hal as hal;

use hal::pac;

mod log {
    pub use crate::loggit as warn;
}

// - Error --------------------------------------------------------------------

#[derive(Debug)]
pub enum Error {
    Usart,
}

// - Interface ----------------------------------------------------------------

#[repr(C)]
pub struct Interface<'a> {
    rx: hal::serial::Rx<pac::USART1>,
    tx: hal::serial::Tx<pac::USART1>,

    #[cfg(not(feature = "alloc"))]
    function_ptr: Option<fn(u8)>,
    #[cfg(any(feature = "alloc"))]
    closure: Option<Box<dyn FnMut(u8) + Send + 'a>>,

    _marker: core::marker::PhantomData<&'a ()>,
}

impl<'a> Interface<'a> {
    pub fn init(
        clocks: &rcc::CoreClocks,
        usart1_rec: rcc::rec::Usart1,
        pins: impl hal::serial::Pins<pac::USART1>,
    ) -> Result<Interface<'a>, Error> {
        let usart1 = unsafe { pac::Peripherals::steal().USART1 };

        let serial = usart1.serial(pins, 31_250.bps(), usart1_rec, clocks);
        match serial {
            Ok(mut serial) => {
                serial.listen(hal::serial::Event::Rxne);
                let (tx, rx) = serial.split();
                Ok(Self {
                    rx,
                    tx,

                    #[cfg(not(feature = "alloc"))]
                    function_ptr: None,
                    #[cfg(any(feature = "alloc"))]
                    closure: Option::None,

                    _marker: core::marker::PhantomData,
                })
            }
            Err(_e) => Err(Error::Usart),
        }
    }

    /// assign function pointer for interrupt callback and start interface
    #[cfg(not(feature = "alloc"))]
    pub fn spawn(mut self, function_ptr: fn(u8)) -> Result<Self, Error> {
        self.function_ptr = Some(function_ptr);
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::USART1);
        }
        Ok(self)
    }

    /// assign closure for interrupt callback and start interface
    #[cfg(any(feature = "alloc"))]
    pub fn spawn<F: FnMut(u8) + Send + 'a>(mut self, closure: F) -> Result<Self, Error> {
        self.closure = Some(Box::new(closure));
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::USART1);
        }
        Ok(self)
    }

    pub fn handle_interrupt_usart1(&mut self) -> Result<(), Error> {
        let usart1 = unsafe { &(*pac::USART1::ptr()) };
        let isr = usart1.isr.read();
        if isr.ore().bit_is_set() {
            usart1.icr.write(|w| w.orecf().clear());
            log::warn!("USART1::irq -> overrun error");
        } else if isr.pe().bit_is_set() {
            usart1.icr.write(|w| w.pecf().clear());
            log::warn!("USART1::irq -> parity error");
        } else if isr.fe().bit_is_set() {
            usart1.icr.write(|w| w.fecf().clear());
            log::warn!("USART1::irq -> frame error");
        } else if isr.nf().bit_is_set() {
            usart1.icr.write(|w| w.ncf().clear());
            log::warn!("USART1::irq -> noise error");
        }

        // invoke midi callback
        if isr.rxne().bit_is_set() {
            let byte = usart1.rdr.read().rdr().bits() as u8;
            self.invoke_callback(byte);
        }

        Ok(())
    }

    fn invoke_callback(&mut self, byte: u8) {
        #[cfg(not(feature = "alloc"))]
        if let Some(function_ptr) = self.function_ptr.as_mut() {
            function_ptr(byte);
        }

        #[cfg(any(feature = "alloc"))]
        if let Some(closure) = self.closure.as_mut() {
            closure(byte);
        }
    }
}
