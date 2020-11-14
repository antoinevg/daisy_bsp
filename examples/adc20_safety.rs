#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_rt::entry;

use daisy_bsp as daisy;

use dsp::osc;
use instrument::midi;



#[entry]
fn main() -> ! {

    // - shared state ---------------------------------------------------------

    let mut osc: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);
    //let mut midi_note: u8 = 69;

    //use core::sync::atomic::AtomicU8;
    //use core::sync::atomic::Ordering; // Rust's memory orderings are the same as those of C++20.
    //let midi_note = AtomicU8::new(69);

    //use heapless::{ self, spsc };
    //let mut queue = spsc::Queue::<midi::Message, heapless::consts::U64>::new();
    //let (mut producer, mut consumer) = queue.split();

    use heapless::{ self, spsc };
    let mut queue = spsc::Queue::<midi::Message, heapless::consts::U64>::new();
    let (mut producer, mut consumer) = queue.split();


    // - board ----------------------------------------------------------------

    let board = daisy::Board::take().unwrap();
    let mut audio_interface = board.SAI1;
    let mut midi_interface = board.USART1;


    // - midi callback --------------------------------------------------------

    let mut midi_parser = midi::Parser::new();

    midi_interface.start(|byte| {
        midi_parser.rx(byte, |_channel, message| {
            /*if let midi::Message::NoteOn { note, velocity: _ } = message {
                //midi_note = note;
                midi_note.store(note, Ordering::Relaxed); // corresponds to memory_order_relaxed in C++20
            }*/
            producer.enqueue(message).unwrap()
        });
    }).unwrap();


    // - audio callback -------------------------------------------------------

    audio_interface.start(|fs, block| {
        //let midi_note = midi_note.load(Ordering::Relaxed);
        let message = consumer.dequeue();
        if let Some(midi::Message::NoteOn { note, velocity: _ }) = message {
            let frequency = dsp::midi_to_hz(note);
            osc.dx = (1. / fs) * frequency;
        }
        osc.block2(block);
    }).unwrap();

    loop {
    }
}
