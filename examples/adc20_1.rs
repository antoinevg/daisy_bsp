#![no_main]
#![no_std]

use cortex_m_rt::entry;
use cortex_m::asm;

use daisy_bsp as daisy;
use daisy::led::Led;


#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    asm::bkpt();
    loop { }
}


#[entry]
fn main() -> ! {
    // - board setup ----------------------------------------------------------

    let board = daisy::Board::take().unwrap();
    // demo singleton: let board2 = daisy::Board::take().unwrap();

    let mut led_user = board.leds.USER;


    // - main loop ------------------------------------------------------------

    let one_second = board.clocks.sys_ck().0;

    loop {
        led_user.on();
        asm::delay(one_second);
        led_user.off();
        asm::delay(one_second);
    }
}
