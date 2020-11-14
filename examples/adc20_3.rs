#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_semihosting::hprintln;

use cortex_m_rt::entry;
use cortex_m::asm;

use heapless::{ self, spsc };

use daisy_bsp as daisy;
use daisy::led::Led;

use instrument::midi;
use instrument::synth;


#[entry]
fn main() -> ! {
    let board = daisy::Board::take().unwrap();


    // - spsc queue -----------------------------------------------------------

    let mut queue = spsc::Queue::<midi::Message, heapless::consts::U64>::new();
    let (mut producer, mut consumer) = queue.split();


    // - midi callback --------------------------------------------------------

    let mut midi_interface = board.USART1;
    let mut midi_parser = midi::Parser::new();

    let _midi_interface = midi_interface.start(|byte| {
        midi_parser.rx(byte, |_channel, message| {
            match producer.enqueue(message) {
                Ok(()) => (),
                Err(_) => hprintln!("midi buffer overflow").unwrap(),
            };
        });
    }).unwrap();


    // - audio callback -------------------------------------------------------

    let mut audio_interface = board.SAI1;

    let mut synth: synth::Synth = synth::Synth::new();
    synth.load_patch(2);  // 1=ep, 2=brass, 3=violin, 7=bass

    let _audio_interface = audio_interface.start(|fs, block| {
        loop {
            match consumer.dequeue() {
                Some(message) => synth.send(message),
                None => break,
            }
        }
        synth.block2(fs, block);
    }).unwrap();


    // - main loop ------------------------------------------------------------

    let mut led_user = board.leds.USER;
    let one_second = board.clocks.sys_ck().0;

    loop {
        led_user.on();
        asm::delay(one_second);
        led_user.off();
        asm::delay(one_second);
    }
}
