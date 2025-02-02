#![no_main]
#![no_std]

use core::cell::RefCell;
use core::ptr::addr_of_mut;

use cortex_m::asm;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use panic_semihosting as _;

use daisy_bsp::hal;
use hal::prelude::*;

use daisy_bsp::pac;
use pac::interrupt;

use daisy_bsp::audio;
use daisy_bsp::led::Led;
use daisy_bsp::loggit;

mod dsp;
use dsp::osc;

// - static global state ------------------------------------------------------

static AUDIO_INTERFACE: Mutex<RefCell<Option<audio::Interface>>> = Mutex::new(RefCell::new(None));

// - entry point --------------------------------------------------------------

#[entry]
fn main() -> ! {
    // - board setup ----------------------------------------------------------

    let board = daisy_bsp::Board::take().unwrap();

    let dp = pac::Peripherals::take().unwrap();

    let ccdr = board.freeze_clocks(dp.PWR.constrain(), dp.RCC.constrain(), &dp.SYSCFG);

    let pins = board.split_gpios(
        dp.GPIOA.split(ccdr.peripheral.GPIOA),
        dp.GPIOB.split(ccdr.peripheral.GPIOB),
        dp.GPIOC.split(ccdr.peripheral.GPIOC),
        dp.GPIOD.split(ccdr.peripheral.GPIOD),
        dp.GPIOE.split(ccdr.peripheral.GPIOE),
        dp.GPIOF.split(ccdr.peripheral.GPIOF),
        dp.GPIOG.split(ccdr.peripheral.GPIOG),
    );

    let mut led_user = daisy_bsp::led::UserLed::new(pins.LED_USER);

    let pins = (
        pins.AK4556.PDN.into_push_pull_output(),
        pins.AK4556.MCLK_A.into_alternate(),
        pins.AK4556.SCK_A.into_alternate(),
        pins.AK4556.FS_A.into_alternate(),
        pins.AK4556.SD_A.into_alternate(),
        pins.AK4556.SD_B.into_alternate(),
    );

    let sai1_prec = ccdr
        .peripheral
        .SAI1
        .kernel_clk_mux(hal::rcc::rec::Sai1ClkSel::PLL3_P);

    let audio_interface =
        audio::Interface::init(&ccdr.clocks, sai1_prec, pins, ccdr.peripheral.DMA1).unwrap();

    // - audio callback -------------------------------------------------------

    // handle callback with function pointer
    #[cfg(not(feature = "alloc"))]
    let audio_interface = {
        fn callback(fs: f32, block: &mut audio::Block) {
            static mut OSC_1: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);
            static mut OSC_2: osc::Wavetable = osc::Wavetable::new(osc::Shape::Saw);
            let osc_1: &'static mut osc::Wavetable = unsafe { &mut *addr_of_mut!(OSC_1) };
            let osc_2: &'static mut osc::Wavetable = unsafe { &mut *addr_of_mut!(OSC_2) };
            osc_1.dx = (1. / fs) * 110.00;
            osc_2.dx = (1. / fs) * 110.00;
            for frame in block {
                *frame = (osc_1.step(), osc_2.step());
            }
        }

        audio_interface.spawn(callback)
    };

    // handle callback with closure (needs alloc)
    #[cfg(any(feature = "alloc"))]
    let audio_interface = {
        let mut osc_1: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);
        let mut osc_2: osc::Wavetable = osc::Wavetable::new(osc::Shape::Saw);

        audio_interface.spawn(move |fs, block| {
            osc_1.dx = (1. / fs) * 110.00;
            osc_2.dx = (1. / fs) * 110.00;
            for frame in block {
                *frame = (osc_1.step(), osc_2.step());
            }
        })
    };

    let audio_interface = match audio_interface {
        Ok(audio_interface) => audio_interface,
        Err(e) => {
            loggit!("Failed to start audio interface: {:?}", e);
            loop {}
        }
    };

    cortex_m::interrupt::free(|cs| {
        AUDIO_INTERFACE.borrow(cs).replace(Some(audio_interface));
    });

    // - main loop ------------------------------------------------------------

    let one_second = ccdr.clocks.sys_ck().raw();

    loop {
        led_user.on();
        asm::delay(one_second);
        led_user.off();
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
                Err(e) => {
                    loggit!("Failed to handle interrupt: {:?}", e);
                }
            };
        }
    });
}
