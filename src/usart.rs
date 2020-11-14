use stm32h7xx_hal as hal;
use hal::prelude::*;
use hal::rcc;
use hal::gpio;

use hal::pac;
use pac::interrupt;

use alloc::prelude::v1::Box;


// - types --------------------------------------------------------------------

type Usart1Pins = (
    gpio::gpiob::PB14<gpio::Alternate<hal::gpio::AF4>>,  // tx
    gpio::gpiob::PB15<gpio::Alternate<hal::gpio::AF4>>,  // rx
);

#[repr(C)] pub struct OpaqueInterface { _private: [u8; 0] }


// - Interface ----------------------------------------------------------------

type Error = u32;

#[repr(C)]
pub struct Interface<'a> {
    rx: hal::serial::Rx<pac::USART1>,
    tx: hal::serial::Tx<pac::USART1>,
    closure: Option<Box<dyn FnMut(u8) + 'a>>,

    _marker: core::marker::PhantomData<&'a *const ()>,
}


impl<'a> Interface<'a> {
    pub fn init(clocks: &rcc::CoreClocks, rec_usart1: rcc::rec::Usart1, pins: Usart1Pins) -> Result<Interface<'a>, Error> {
        let usart1 = unsafe { pac::Peripherals::steal().USART1 };

        let serial = usart1.serial(pins, 31_250.bps(), rec_usart1, clocks);
        match serial {
            Ok(mut serial) => {
                serial.listen(hal::serial::Event::Rxne); // TODO Tx & errors too
                let (tx, rx) = serial.split();
                Ok(Self {
                    rx,
                    tx,
                    closure: None,
                    _marker: core::marker::PhantomData
                })
            },
            Err(_e) => Err(0), // TODO pass actual error up
        }
    }

    pub fn start<F: FnMut(u8) + 'a>(&mut self, closure: F) -> Result<&mut Self, Error> {
        self.closure = Some(Box::new(closure));

        let opaque_interface_ptr: *const OpaqueInterface = unsafe {
            core::mem::transmute::<*const Interface,
                                   *const OpaqueInterface>(self)
        };
        unsafe { INTERFACE_PTR = Some(opaque_interface_ptr); }

        unsafe { pac::NVIC::unmask(pac::Interrupt::USART1); }

        Ok(self)
    }
}


// - usart interrupt handler --------------------------------------------------

static mut INTERFACE_PTR: Option<*const OpaqueInterface> = None;

#[interrupt]
fn USART1() {
    let usart1  = unsafe { &(*pac::USART1::ptr()) };
    let isr = usart1.isr.read();
    if isr.ore().bit_is_set() {
        usart1.icr.write(|w| w.orecf().clear());
        //hprintln!("USART1::irq -> overrun error").unwrap();
    } else if isr.pe().bit_is_set() {
        usart1.icr.write(|w| w.pecf().clear());
        //hprintln!("USART1::irq -> parity error").unwrap();
    } else if isr.fe().bit_is_set() {
        usart1.icr.write(|w| w.fecf().clear());
        //hprintln!("USART1::irq -> frame error").unwrap();
    } else if isr.nf().bit_is_set() {
        usart1.icr.write(|w| w.ncf().clear());
        //hprintln!("USART1::irq -> noise error").unwrap();
    }

    if isr.rxne().bit_is_set() {
        let byte = usart1.rdr.read().rdr().bits() as u8;
        if let Some(interface_ptr) = unsafe { INTERFACE_PTR }  {
            let interface_ptr: *mut Interface = unsafe {
                core::mem::transmute::<*const OpaqueInterface,
                                       *mut Interface>(interface_ptr)
            };
            if let Some(closure) = unsafe { &mut (*interface_ptr).closure } {
                closure(byte);
            }
        }
    }
}
