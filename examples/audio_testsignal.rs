#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_rt::entry;
use cortex_m::asm;

use daisy_bsp as daisy;
use daisy::led::Led;

mod dsp;
use dsp::osc;


#[entry]
fn main() -> ! {

    // - board setup ----------------------------------------------------------

    let board = daisy::Board::take().unwrap();

    let mut led_user = board.leds.USER;
    let mut audio_interface = board.SAI1;


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

    let one_second = board.clocks.sys_ck().0;

    loop {
        led_user.on();
        asm::delay(one_second);
        led_user.off();
        asm::delay(one_second);
    }
}
