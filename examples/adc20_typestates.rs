#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_mut)]

#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_rt::entry;

use stm32h7xx_hal as hal;
use hal::{prelude::*, pac, rcc, gpio};

// traits
use hal::rcc::ResetEnable;
//use hal::hal::digital::v2::OutputPin;


// - type states --------------------------------------------------------------

use core::marker::PhantomData;  // pub struct PhantomData<T>;

// Type states implemented as ZST's
pub struct Analog;
pub struct PushPull;
pub struct OpenDrain;
pub struct Floating;
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

pub struct PC0<MODE> {
    _mode: PhantomData<MODE>,
}
pub struct PC7<MODE> {
    _mode: PhantomData<MODE>,
}

pub struct Parts {
    /// Pin
    pub pc0: PC0<Analog>,
    pub pc7: PC7<Analog>,
}

pub trait GpioExt {
    type Parts;
    type Rec: rcc::ResetEnable;
    fn split_(self, prec: Self::Rec) -> Self::Parts;
}

// This is what pac::GPIOC looks like
pub struct GPIOC {
    _marker: PhantomData<*const ()>,
}
impl GPIOC {
    #[doc = r"Returns a pointer to the register block"]
    #[inline(always)]
    pub const fn ptr() -> *const pac::gpioa::RegisterBlock {
        0x5802_0800 as *const _
    }
}

impl GpioExt for pac::GPIOC {
    type Parts = Parts;
    type Rec = rcc::rec::Gpioc;
    fn split_(self, prec: rcc::rec::Gpioc) -> Parts {
        prec.enable().reset();
        Parts {
            pc0: PC0 { _mode: PhantomData },
            pc7: PC7 { _mode: PhantomData },
        }
    }
}

impl<MODE> PC7<MODE> {
    /// Configures the pin to operate as an push pull output pin
    pub fn into_push_pull_output(self) -> PC7<Output<PushPull>> {
        let offset = 2 * 7;
        unsafe {
            &(*GPIOC::ptr())
                .pupdr
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
            &(*GPIOC::ptr())
                .otyper
                .modify(|r, w| w.bits(r.bits() & !(0b1 << 7)));
            &(*GPIOC::ptr())
                .moder
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)))
        };
        PC7 { _mode: PhantomData }
    }
    /// Configures the pin to operate as an open drain output pin
    pub fn into_open_drain_output(self) -> PC7<Output<OpenDrain>> {
        /* ... */
        PC7 { _mode: PhantomData }
    }
    pub fn into_floating_input(self) -> PC7<Input<Floating>> {
        /* ... */
        PC7 { _mode: PhantomData }
    }
}

impl<MODE> PC7<Output<MODE>> {
    /// Set pin speed
    pub fn set_speed(self, speed: gpio::Speed) -> Self {
        let offset = 2 * 7;
        unsafe {
            &(*GPIOC::ptr()).ospeedr.modify(|r, w| {
                w.bits((r.bits() & !(0b11 << offset)) | ((speed as u32) << offset))
            })
        };
        self
    }
}

impl PC7<Output<OpenDrain>> {
    /// Enables / disables the internal pull up
    pub fn internal_pull_up(&mut self, on: bool) {
        let offset = 2 * 7;
        let value = if on { 0b01 } else { 0b00 };
        unsafe {
            &(*GPIOC::ptr())
                .pupdr
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)))
        };
    }
}

//use hal::hal::digital::v2::OutputPin; // embedded hal trait: OutputPin
pub trait OutputPin {
    type Error;
    fn set_low(&mut self) -> Result<(), Self::Error>;
    fn set_high(&mut self) -> Result<(), Self::Error>;
}

impl<MODE> OutputPin for PC7<Output<MODE>> {
    type Error = hal::Never;
    fn set_high(&mut self) -> Result<(), hal::Never> {
        unsafe { (*GPIOC::ptr()).bsrr.write(|w| w.bits(1 << 7)) }
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), hal::Never> {
        unsafe { (*GPIOC::ptr()).bsrr.write(|w| w.bits(1 << (7 + 16))) }
        Ok(())
    }
}

// Type states implemented as ZST's
/*pub struct Analog;
pub struct PushPull;
pub struct OpenDrain;

pub struct GpioPin;
pub struct Input;
pub struct Output;

impl GpioPin {
    pub fn into_output(self) -> Output {
        let offset = 2 * 0;
        let mode = 0b01;
        unsafe {
            &(*pac::GPIOC::ptr()).moder.modify(|r, w| {
                w.bits((r.bits() & !(0b11 << offset)) | (mode << offset))
            });
        }
        Output {}
    }

    pub fn into_input(self) -> Input {
        Input {}
    }
}

impl Output {
    pub fn set_speed(self, speed: gpio::Speed) -> Self {
        self
    }
}*/



// - main ---------------------------------------------------------------------

#[entry]
fn main() -> ! {
    use core::mem::size_of;

    assert!(size_of::<Output<OpenDrain>>() == 0);

    let dp = pac::Peripherals::take().unwrap();

    // - power & clocks -------------------------------------------------------

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.vos0(&dp.SYSCFG).freeze();
    let ccdr = dp.RCC.constrain()
        .use_hse(16.mhz())                                     // external crystal @ 16 MHz
        .pll1_strategy(rcc::PllConfigStrategy::Iterative)           // pll1 drives system clock
        .sys_ck(480.mhz())                                     // system clock @ 480 MHz
        .freeze(pwrcfg, &dp.SYSCFG);

    let gpioc = dp.GPIOC.split_(ccdr.peripheral.GPIOC);

    // - pins -----------------------------------------------------------------

    let mut pin = gpioc.pc7.into_push_pull_output()
                           .set_speed(gpio::Speed::High);

    // ---

    // 1. configure an open drain output pin

    /*let pin: PC7<Analog> = gpioc.pc7;
    let mut pin = pin.into_open_drain_output()
                     .set_speed(gpio::Speed::High);
    pin.internal_pull_up(true);*/

    // ---

    // 2. someone carelessly changes it to a push_pull output

/*    let pin: PC7<Analog> = gpioc.pc7;
    let pin = pin.into_push_pull_output()
                 .set_speed(gpio::Speed::High);
    pin.internal_pull_up(true); // <- only open drain outputs have an internal pull-up resistor
*/
    // COMPILE ERROR !!!

    // ---

    // 3. someone tries to reconfigure it in another place in the code

    /*let pin: PC7<Analog> = gpioc.pc7;
    let pin_output = pin.into_push_pull_output(); // <- consumes the pin
    let pin_input = pin.into_floating_input();*/

    // COMPILE ERROR !!!

    // ---

    //let pin2: PC7<Output<PushPull>> = pin1.into_push_pull_output();
    //let pin3: PC7<Output<PushPull>> = pin2.set_speed(gpio::Speed::High);

    //let mut pin2: PC7<Output<OpenDrain>> = pin1.into_open_drain_output();
    //let pin3: PC7<Output<OpenDrain>> = pin2.internal_pull_up(true);


    // - main loop ------------------------------------------------------------

    loop {
        //pin.set_high().unwrap();
        cortex_m::asm::delay(480_000_000);

        //pin.set_low().unwrap();
        cortex_m::asm::delay(480_000_000);
    }
}
