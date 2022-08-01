#![no_main]
#![no_std]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use cortex_m::asm;
use cortex_m_rt::entry;
use panic_semihosting as _;

use daisy::audio;
use daisy_bsp as daisy;

use daisy::hal;
use hal::gpio;
use hal::prelude::*;
use hal::rcc;

use daisy::pac;
use pac::interrupt;

// - static global state ------------------------------------------------------

static AUDIO_INTERFACE: Mutex<RefCell<Option<audio::Interface>>> = Mutex::new(RefCell::new(None));

// - entry point --------------------------------------------------------------

#[entry]
fn main() -> ! {
    // - power & clocks -------------------------------------------------------

    let dp: hal::pac::Peripherals = hal::pac::Peripherals::take().unwrap();
    let ccdr: hal::rcc::Ccdr =
        daisy::clocks::configure(dp.PWR.constrain(), dp.RCC.constrain(), &dp.SYSCFG);
    let sai1_rec = ccdr
        .peripheral
        .SAI1
        .kernel_clk_mux(rcc::rec::Sai1ClkSel::PLL3_P);

    // - configure pins -------------------------------------------------------

    let gpioc: gpio::gpioc::Parts = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let mut led_user: gpio::gpioc::PC7<gpio::Output<gpio::PushPull>> =
        gpioc.pc7.into_push_pull_output();

    let gpiob: gpio::gpiob::Parts = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioe: gpio::gpioe::Parts = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let ak4556_pins = (
        gpiob.pb11.into_push_pull_output(), // PDN
        gpioe.pe2.into_alternate(),         // MCLK_A
        gpioe.pe5.into_alternate(),         // SCK_A
        gpioe.pe4.into_alternate(),         // FS_A
        gpioe.pe6.into_alternate(),         // SD_A
        gpioe.pe3.into_alternate(),         // SD_B
    );

    // - start audio interface ------------------------------------------------

    let audio_interface =
        audio::Interface::init(&ccdr.clocks, sai1_rec, ak4556_pins, ccdr.peripheral.DMA1).unwrap();

    // handle callback with function pointer
    #[cfg(not(feature = "alloc"))]
    let audio_interface = {
        fn callback(_fs: f32, block: &mut audio::Block) {
            for frame in block {
                let (left, right) = *frame;
                *frame = (left, right);
            }
        }

        audio_interface.spawn(callback).unwrap()
    };

    // handle callback with closure (needs alloc)
    #[cfg(any(feature = "alloc"))]
    let audio_interface = audio_interface
        .spawn(|_fs, block| {
            for frame in block {
                let (left, right) = *frame;
                *frame = (left, right);
            }
        })
        .unwrap();

    // wrap audio interface in mutex so we can access it in the interrupt
    cortex_m::interrupt::free(|cs| {
        AUDIO_INTERFACE.borrow(cs).replace(Some(audio_interface));
    });

    // - main loop ------------------------------------------------------------

    let one_second = ccdr.clocks.sys_ck().raw();
    loop {
        led_user.set_high();
        asm::delay(one_second);

        led_user.set_low();
        asm::delay(one_second);
    }
}

// - interrupts ---------------------------------------------------------------

/// interrupt handler for: dma1, stream1
#[interrupt]
fn DMA1_STR1() {
    cortex_m::interrupt::free(|cs| {
        if let Some(audio_interface) = AUDIO_INTERFACE.borrow(cs).borrow_mut().as_mut() {
            match audio_interface.handle_interrupt_dma1_str1() {
                Ok(()) => (),
                Err(_e) => {
                    // handle any errors
                }
            };
        }
    });
}
