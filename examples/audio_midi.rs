#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_rt::entry;

use daisy_bsp as daisy;

mod dsp;
use dsp::osc;

mod instrument;
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
    let mut audio_interface = board.SAI1;
    let mut midi_interface = board.USART1;


    // - usart1 interrupt -----------------------------------------------------

    let mut midi_parser = midi::Parser::new();

    midi_interface.start(|byte| {
        midi_parser.rx(byte, |_channel, message| {
            if let midi::Message::NoteOn { note, velocity: _ } = message {
                midi_note.store(note, Ordering::Relaxed);
            }
        });
    }).unwrap();


    // - dma1 interrupt -------------------------------------------------------

    audio_interface.start(|fs, block| {
        let midi_note = midi_note.load(Ordering::Relaxed);
        let frequency = midi::to_hz(midi_note);
        osc.dx = (1. / fs) * frequency;
        osc.block2(block);
    }).unwrap();

    loop {
        cortex_m::asm::wfi();
    }
}
