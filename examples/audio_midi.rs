#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_rt::entry;

use daisy_bsp as daisy;
use daisy::hal::prelude::*;

mod dsp;
mod instrument;

use dsp::osc;
use instrument::midi;


#[entry]
fn main() -> ! {

    // - state ----------------------------------------------------------------

    use core::sync::atomic::AtomicU8;
    use core::sync::atomic::Ordering;
    let midi_note = AtomicU8::new(69);

    let mut osc: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);


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
                                 dp.GPIOG.split(ccdr.peripheral.GPIOG),
                                 dp.GPIOH.split(ccdr.peripheral.GPIOH),
                                 dp.GPIOI.split(ccdr.peripheral.GPIOI),
                                 dp.GPIOJ.split(ccdr.peripheral.GPIOJ),
                                 dp.GPIOK.split(ccdr.peripheral.GPIOK));

    let mut audio_interface = board.split_audio(&ccdr.clocks,
                                                ccdr.peripheral.SAI1,
                                                ccdr.peripheral.DMA1,
                                                pins.AK4556);

    let mut midi_interface = board.split_midi(&ccdr.clocks,
                                              ccdr.peripheral.USART1,
                                              (pins.SEED_PIN_13, pins.SEED_PIN_14));


    // - midi callback --------------------------------------------------------

    let mut midi_parser = midi::Parser::new();

    midi_interface.start(|byte| {
        midi_parser.rx(byte, |_channel, message| {
            if let midi::Message::NoteOn { note, velocity: _ } = message {
                midi_note.store(note, Ordering::Relaxed);
            }
        });
    }).unwrap();


    // - audio callback -------------------------------------------------------

    audio_interface.start(|fs, block| {
        let midi_note = midi_note.load(Ordering::Relaxed);
        let frequency = midi::to_hz(midi_note);
        osc.dx = (1. / fs) * frequency;
        osc.block2(block);
    }).unwrap();


    // - main loop ------------------------------------------------------------

    loop {
        cortex_m::asm::wfi();
    }
}
