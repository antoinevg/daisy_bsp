#![no_main]
#![no_std]

use cortex_m_rt::entry;
use panic_semihosting as _;

use daisy::hal::prelude::*;
use daisy::led::Led;
use daisy_bsp as daisy;

#[entry]
fn main() -> ! {
    // - board setup ----------------------------------------------------------

    let board = daisy::Board::take().unwrap();
    let dp = daisy::pac::Peripherals::take().unwrap();

    let ccdr = board.freeze_clocks(dp.PWR.constrain(), dp.RCC.constrain(), &dp.SYSCFG);

    let pins = board.split_gpios(
        dp.GPIOA.split(ccdr.peripheral.GPIOA),
        dp.GPIOB.split(ccdr.peripheral.GPIOB),
        dp.GPIOC.split(ccdr.peripheral.GPIOC),
        dp.GPIOD.split(ccdr.peripheral.GPIOD),
        dp.GPIOE.split(ccdr.peripheral.GPIOE),
        dp.GPIOF.split(ccdr.peripheral.GPIOF),
        dp.GPIOG.split(ccdr.peripheral.GPIOG),
    );

    let mut led_user = daisy::led::LedUser::new(pins.LED_USER);
    let mut flash =
        daisy::flash::Flash::new(&ccdr.clocks, dp.QUADSPI, ccdr.peripheral.QSPI, pins.FMC);

    // - test that what was written can be read back --------------------------

    const ADDRESS: u32 = 0x00;
    const SIZE: usize = 8000;

    // create the array
    let mut data: [u8; SIZE] = [0; SIZE];
    for (i, x) in data.iter_mut().enumerate() {
        *x = (i % 256) as u8;
    }

    // write it to memory
    flash.write(ADDRESS, &data);

    // read it back
    let mut buffer: [u8; SIZE] = [0; SIZE];
    flash.read(ADDRESS, &mut buffer);

    // and compare!
    if data == buffer {
        led_user.on();
    } else {
        led_user.off();
    }

    loop {
        cortex_m::asm::nop();
    }
}
