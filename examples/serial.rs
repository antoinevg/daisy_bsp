#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_rt::entry;

use daisy_bsp::hal;
use hal::{pac, prelude::*};
use hal::nb::block;

use core::fmt::Write;


#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(160.MHz()).freeze(pwrcfg, &dp.SYSCFG);

    // Acquire the GPIOB peripheral. This also enables the clock for
    // GPIOB in the RCC register.
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    let tx = gpiob.pb14.into_alternate();    // USART1 TX - GPIO29 - Pin 36 <= GPIOB 14
    let rx = gpiob.pb15.into_alternate();    // USART1 RX - GPIO30 - Pin 37 => GPIOB 15

    // Configure the serial peripheral.
    let serial = dp
        .USART1
        .serial((tx, rx), 38_400.bps(), ccdr.peripheral.USART1, &ccdr.clocks)
        .unwrap();

    let (mut tx, mut rx) = serial.split();

    // core::fmt::Write is implemented for tx.
    writeln!(tx, "Hello, world!").unwrap();

    loop {
        // Echo what is received on the serial link.
        let received = block!(rx.read()).unwrap();
        block!(tx.write(received)).ok();
    }
}
