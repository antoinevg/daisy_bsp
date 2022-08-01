//! Example of how to access the ADC peripheral directly via
//! stm32h7xx-hal without using the board support crate.

#![no_main]
#![no_std]

use panic_semihosting as _;

use cortex_m_rt::entry;
use cortex_m::asm;

use daisy_bsp::hal;
use hal::{pac, prelude::*};
use hal::rcc::PllConfigStrategy;
use hal::rcc::rec::AdcClkSel;
use hal::{adc, delay::Delay};


#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // - power & clocks -------------------------------------------------------

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.vos0(&dp.SYSCFG).freeze();
    let mut ccdr = dp.RCC.constrain()
        .use_hse(16.MHz())                                     // external crystal @ 16 MHz
        .pll1_strategy(PllConfigStrategy::Iterative)           // pll1 drives system clock
        .sys_ck(480.MHz())                                     // system clock @ 480 MHz
        .per_ck(4.MHz())                                       // adc1 clock @ 4 MHz
        .freeze(pwrcfg, &dp.SYSCFG);

    // switch adc_ker_ck_input multiplexer to per_ck
    ccdr.peripheral.kernel_adc_clk_mux(AdcClkSel::PER);

    // - adc ------------------------------------------------------------------

    let mut delay = Delay::new(cp.SYST, ccdr.clocks);
    let mut adc1 = adc::Adc::adc1(
        dp.ADC1,
        &mut delay,
        ccdr.peripheral.ADC12,
        &ccdr.clocks,
    ).enable();
    adc1.set_resolution(adc::Resolution::SIXTEENBIT);

    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let mut adc1_channel_4  = gpioc.pc4.into_analog(); // pot 1
    let mut adc1_channel_10 = gpioc.pc0.into_analog(); // pot 2

    // - led ------------------------------------------------------------------

    let mut led_user = gpioc.pc7.into_push_pull_output();

    // - main loop ------------------------------------------------------------

    let scale_factor = ccdr.clocks.sys_ck().raw() as f32 / 65_535.;

    loop {
        let pot_1: u32 = adc1.read(&mut adc1_channel_4).unwrap();
        let _pot_2: u32 = adc1.read(&mut adc1_channel_10).unwrap();

        let ticks = (pot_1 as f32 * scale_factor) as u32;

        led_user.set_high();
        asm::delay(ticks);

        led_user.set_low();
        asm::delay(ticks);
    }
}
