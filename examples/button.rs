#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_rt::entry;

use daisy_bsp::hal;
use hal::prelude::*;
use hal::rcc::PllConfigStrategy;

use hal::hal as embedded_hal;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

use hal::pac;


#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();


    // - power & clocks -------------------------------------------------------

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.vos0(&dp.SYSCFG).freeze();
    let ccdr = dp.RCC.constrain()
        .use_hse(16.mhz())                           // external crystal @ 16 MHz
        .pll1_strategy(PllConfigStrategy::Iterative) // pll1 drives system clock
        .sys_ck(480.mhz())                           // system clock @ 480 MHz
        .freeze(pwrcfg, &dp.SYSCFG);


    // - pins -----------------------------------------------------------------

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    let mut led_user = gpioc.pc7.into_push_pull_output();

    // DAISY_PIN_35 aka DAISY_GPIO_28 aka DAISY_SAI2_SCK aka DAISY_ADC_11
    let button = gpioa.pa2.into_pull_up_input();


    // - main loop ------------------------------------------------------------

    loop {
        if button.is_high().unwrap() {
            led_user.set_low().unwrap();
        } else {
            led_user.set_high().unwrap();
        }
    }
}
