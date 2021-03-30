#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_rt::entry;
use cortex_m::asm;

use daisy_bsp as daisy;
use daisy::hal::prelude::*;
use daisy::led::Led;

mod dsp;
use dsp::osc;


#[entry]
fn main() -> ! {

    // - board setup ----------------------------------------------------------

    let board = daisy::Board::take().unwrap();
    let dp = daisy::pac::Peripherals::take().unwrap();

    let ccdr = board.freeze_clocks(dp.PWR.constrain(),
                                   dp.RCC.constrain(),
                                   &dp.SYSCFG);

    let pins = board.split_gpios(dp.GPIOA.split(ccdr.peripheral.GPIOA),
                                 dp.GPIOB.split(ccdr.peripheral.GPIOB),
                                 dp.GPIOC.split(ccdr.peripheral.GPIOC),
                                 dp.GPIOD.split(ccdr.peripheral.GPIOD),
                                 dp.GPIOE.split(ccdr.peripheral.GPIOE),
                                 dp.GPIOF.split(ccdr.peripheral.GPIOF),
                                 dp.GPIOG.split(ccdr.peripheral.GPIOG));

    let mut led_user = daisy::led::LedUser::new(pins.LED_USER);

    let mut audio_interface = board.split_audio(&ccdr.clocks,
                                                ccdr.peripheral.SAI1,
                                                ccdr.peripheral.DMA1,
                                                pins.AK4556);


    // - audio callback -------------------------------------------------------

    let mut osc_1: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);
    let mut osc_2: osc::Wavetable = osc::Wavetable::new(osc::Shape::Saw);

    let fs = daisy::audio::FS.0 as f32;

    osc_1.dx = (1. / fs) * 110.00;
    osc_2.dx = (1. / fs) * 110.00;

    let _audio_interface = audio_interface.start(|_fs, block| {
        for frame in block {
            *frame = (osc_1.step(), osc_2.step());
        }
    });


    // - main loop ------------------------------------------------------------

    let one_second = ccdr.clocks.sys_ck().0;

    loop {
        led_user.on();
        asm::delay(one_second);
        led_user.off();
        asm::delay(one_second);
    }
}
