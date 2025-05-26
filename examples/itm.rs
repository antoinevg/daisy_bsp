#![no_main]
#![no_std]

use cortex_m::asm;
use cortex_m_rt::entry;
use panic_itm as _;

use daisy_bsp::hal::prelude::*;
use daisy_bsp::led::Led;
use daisy_bsp::loggit;

#[entry]
fn main() -> ! {
    // - board setup ----------------------------------------------------------

    let board = daisy_bsp::Board::take().unwrap();
    let dp = daisy_bsp::pac::Peripherals::take().unwrap();

    let ccdr = board.freeze_clocks(dp.PWR.constrain(), dp.RCC.constrain(), &dp.SYSCFG);

    loggit!("Hello daisy::itm !");

    let pins = board.split_gpios(
        dp.GPIOA.split(ccdr.peripheral.GPIOA),
        dp.GPIOB.split(ccdr.peripheral.GPIOB),
        dp.GPIOC.split(ccdr.peripheral.GPIOC),
        dp.GPIOD.split(ccdr.peripheral.GPIOD),
        dp.GPIOE.split(ccdr.peripheral.GPIOE),
        dp.GPIOF.split(ccdr.peripheral.GPIOF),
        dp.GPIOG.split(ccdr.peripheral.GPIOG),
        dp.GPIOH.split(ccdr.peripheral.GPIOH),
        dp.GPIOI.split(ccdr.peripheral.GPIOI),
    );

    let mut user_led = daisy_bsp::led::UserLed::new(pins.LED_USER);

    // - main loop ------------------------------------------------------------

    let one_second = ccdr.clocks.sys_ck().raw();
    let mut counter = 0;

    loop {
        loggit!("ping: {}", counter);
        counter += 1;

        user_led.on();
        asm::delay(one_second);
        user_led.off();
        asm::delay(one_second);
    }
}
