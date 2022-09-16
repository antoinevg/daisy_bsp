#![no_main]
#![no_std]

use panic_semihosting as _;

extern crate alloc;
use alloc::sync::Arc;

use core::cell::RefCell;
use core::sync::atomic::{AtomicU8, Ordering};

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;

use daisy::loggit;
use daisy_bsp as daisy;

use daisy::pac;
use pac::interrupt;

mod dsp;
mod instrument;

// - static global state ------------------------------------------------------

static AUDIO_INTERFACE: Mutex<RefCell<Option<daisy::audio::Interface>>> =
    Mutex::new(RefCell::new(None));
static MIDI_INTERFACE: Mutex<RefCell<Option<daisy::midi::Interface>>> =
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // - state ----------------------------------------------------------------

    let midi_note = Arc::new(AtomicU8::new(69));

    let mut osc: dsp::osc::Wavetable = dsp::osc::Wavetable::new(dsp::osc::Shape::Sin);

    // - board setup ----------------------------------------------------------

    let board = daisy::Board::take().unwrap();
    let dp = daisy::pac::Peripherals::take().unwrap();

    let ccdr = daisy::board_freeze_clocks!(board, dp);
    let pins = daisy::board_split_gpios!(board, ccdr, dp);
    let audio_interface = daisy::board_split_audio!(ccdr, pins);
    let midi_interface = daisy::board_split_midi!(ccdr, pins);

    loggit!("Hello audio and midi!");

    // - midi callback --------------------------------------------------------

    let mut midi_parser = instrument::midi::Parser::new();
    let midi_note_clone = Arc::clone(&midi_note);

    let midi_interface = midi_interface
        .spawn(move |byte| {
            midi_parser.rx(byte, |_channel, message| {
                if let instrument::midi::Message::NoteOn { note, velocity: _ } = message {
                    loggit!("note_on: {:?}", note);
                    midi_note_clone.store(note, Ordering::Relaxed);
                }
            });
        })
        .unwrap();

    // - audio callback -------------------------------------------------------

    let audio_interface = audio_interface
        .spawn(move |fs, block| {
            let midi_note = midi_note.load(Ordering::Relaxed);
            let frequency = instrument::midi::to_hz(midi_note);
            osc.dx = (1. / fs) * frequency;
            osc.block2(block);
        })
        .unwrap();

    // - wrap audio & midi interfaces in a global mutex -----------------------

    cortex_m::interrupt::free(|cs| {
        AUDIO_INTERFACE.borrow(cs).replace(Some(audio_interface));
        MIDI_INTERFACE.borrow(cs).replace(Some(midi_interface));
    });

    // - main loop ------------------------------------------------------------

    loop {
        cortex_m::asm::wfi();
    }
}

// - interrupts ---------------------------------------------------------------

/// interrupt handler for: usart1
#[interrupt]
fn USART1() {
    cortex_m::interrupt::free(|cs| {
        if let Some(midi_interface) = MIDI_INTERFACE.borrow(cs).borrow_mut().as_mut() {
            match midi_interface.handle_interrupt_usart1() {
                Ok(()) => (),
                Err(e) => {
                    loggit!("Failed to handle interrupt USART1: {:?}", e);
                }
            };
        }
    });
}

/// interrupt handler for: dma1, stream1
#[interrupt]
fn DMA1_STR1() {
    cortex_m::interrupt::free(|cs| {
        if let Some(audio_interface) = AUDIO_INTERFACE.borrow(cs).borrow_mut().as_mut() {
            match audio_interface.handle_interrupt_dma1_str1() {
                Ok(()) => (),
                Err(e) => {
                    loggit!("Failed to handle interrupt DMA1_STR1: {:?}", e);
                }
            };
        }
    });
}
