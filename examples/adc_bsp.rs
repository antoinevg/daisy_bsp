//! Example of how to directly access the ADC peripheral on the Daisy
//! Seed when using the Board Support Crate.

#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_rt::entry;

use daisy_bsp as daisy;

use cortex_m::asm;

use daisy_bsp::hal;
use hal::rcc::rec::AdcClkSel;
use hal::{adc, delay::Delay};

use hal::hal as embedded_hal;
use embedded_hal::digital::v2::OutputPin;

use hal::{pac, prelude::*};


#[entry]
fn main() -> ! {
    // - board setup ----------------------------------------------------------

    let mut board = daisy::Board::take().unwrap();

    // - clocks ---------------------------------------------------------------

    // switch adc_ker_ck_input multiplexer to per_ck
    board.peripheral.kernel_adc_clk_mux(AdcClkSel::PER);

    // - adc ------------------------------------------------------------------

    let cp = unsafe { cortex_m::Peripherals::steal() };
    let dp = unsafe { pac::Peripherals::steal() };

    let mut delay = Delay::new(cp.SYST, board.clocks);
    let mut adc1 = adc::Adc::adc1(
        dp.ADC1,
        &mut delay,
        board.peripheral.ADC12,
        &board.clocks,
    ).enable();
    adc1.set_resolution(adc::Resolution::SIXTEENBIT);

    let gpioc = dp.GPIOC.split(board.peripheral.GPIOC);
    let mut adc1_channel_4  = gpioc.pc4.into_analog(); // pot 1
    let mut adc1_channel_10 = gpioc.pc0.into_analog(); // pot 2

    // - led ------------------------------------------------------------------

    let mut led_user = gpioc.pc7.into_push_pull_output();

    // - main loop ------------------------------------------------------------

    loop {
        let pot_1: u32 = adc1.read(&mut adc1_channel_4).unwrap();
        let _pot_2: u32 = adc1.read(&mut adc1_channel_10).unwrap();

        let ticks = (pot_1 as f32 * (480_000_000. / 65_535.)) as u32;

        led_user.set_high().unwrap();
        asm::delay(ticks);

        led_user.set_low().unwrap();
        asm::delay(ticks);
    }
}
