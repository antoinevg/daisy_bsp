#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_rt::entry;

use daisy_bsp as daisy;
use daisy::hal::prelude::*;
use daisy::led::Led;
use daisy::loggit;


#[entry]
fn main() -> ! {
    // - board setup ----------------------------------------------------------

    let board = daisy::Board::take().unwrap();
    let dp = daisy::pac::Peripherals::take().unwrap();

    let ccdr = board.freeze_clocks(dp.PWR.constrain(),
                                   dp.RCC.constrain(),
                                   &dp.SYSCFG);

    loggit!("Hello daisy::itm !");

    let pins = board.split_gpios(dp.GPIOA.split(ccdr.peripheral.GPIOA),
                                 dp.GPIOB.split(ccdr.peripheral.GPIOB),
                                 dp.GPIOC.split(ccdr.peripheral.GPIOC),
                                 dp.GPIOD.split(ccdr.peripheral.GPIOD),
                                 dp.GPIOE.split(ccdr.peripheral.GPIOE),
                                 dp.GPIOF.split(ccdr.peripheral.GPIOF),
                                 dp.GPIOG.split(ccdr.peripheral.GPIOG));

    let mut led_user = daisy::led::UserLed::new(pins.LED_USER);


    // - main loop ------------------------------------------------------------

    let one_second = ccdr.clocks.sys_ck().raw();
    let mut counter = 0;

    loop {
        loggit!("ping: {}", counter);
        counter += 1;

        led_user.on();
        cortex_m::asm::delay(one_second);
        led_user.off();
        cortex_m::asm::delay(one_second);
    }
}
