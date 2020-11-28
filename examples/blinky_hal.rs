#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_rt::entry;
use cortex_m::asm;

use daisy_bsp::hal;
use hal::{pac, prelude::*};
use hal::rcc::PllConfigStrategy;
use hal::hal as embedded_hal;
use embedded_hal::digital::v2::OutputPin;


#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // - power & clocks -------------------------------------------------------

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.vos0(&dp.SYSCFG).freeze();
    let ccdr = dp.RCC.constrain()
        .use_hse(16.mhz())                                     // external crystal @ 16 MHz
        .pll1_strategy(PllConfigStrategy::Iterative)           // pll1 drives system clock
        .sys_ck(480.mhz())                                     // system clock @ 480 MHz
        .freeze(pwrcfg, &dp.SYSCFG);


    // - pins -----------------------------------------------------------------

    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let mut led_user = gpioc.pc7.into_push_pull_output();


    // - main loop ------------------------------------------------------------

    loop {
        loop {
            led_user.set_high().unwrap();
            asm::delay(480_000_000);

            led_user.set_low().unwrap();
            asm::delay(480_000_000);
        }
    }
}
