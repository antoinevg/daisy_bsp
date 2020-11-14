#![no_main]
#![no_std]

use panic_semihosting as _;
use cortex_m_rt::entry;

use cortex_m::asm;

use daisy_bsp as daisy;
use daisy::hal;

use hal::prelude::*;
use hal::rcc;
use hal::gpio;
use hal::hal::digital::v2::OutputPin;


// = entry ====================================================================

#[entry]
fn main() -> ! {
    // - power & clocks -------------------------------------------------------

    let dp: hal::pac::Peripherals = hal::pac::Peripherals::take().unwrap(); // device-specific peripherals
    let ccdr: hal::rcc::Ccdr = daisy::configure_clocks(dp.PWR.constrain(),
                                                       dp.RCC.constrain(),
                                                       &dp.SYSCFG);
    let sai1_rec = ccdr.peripheral.SAI1.kernel_clk_mux(rcc::rec::Sai1ClkSel::PLL3_P);


    // - configure pins -------------------------------------------------------

    let gpioc: gpio::gpioc::Parts = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let mut led_user: gpio::gpioc::PC7<gpio::Output<gpio::PushPull>> = gpioc.pc7.into_push_pull_output();

    let gpiob: gpio::gpiob::Parts = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioe: gpio::gpioe::Parts = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let ak4556_pins = (
        gpiob.pb11.into_push_pull_output(), // PDN
        gpioe.pe2.into_alternate_af6(),     // MCLK_A
        gpioe.pe5.into_alternate_af6(),     // SCK_A
        gpioe.pe4.into_alternate_af6(),     // FS_A
        gpioe.pe6.into_alternate_af6(),     // SD_A
        gpioe.pe3.into_alternate_af6(),     // SD_B
    );


    // - start audio interface ------------------------------------------------

    let mut audio_interface = daisy::audio::Interface::init(&ccdr.clocks,
                                                            sai1_rec,
                                                            ak4556_pins,
                                                            ccdr.peripheral.DMA1).unwrap();

    let _audio_interface = audio_interface.start(|_fs, block| {
        for frame in block {
            let (left, right) = *frame;
            *frame = (left, right);
        }
    });


    // - main loop ------------------------------------------------------------

    let one_second = ccdr.clocks.sys_ck().0;
    loop {
        led_user.set_high().unwrap();
        asm::delay(one_second);

        led_user.set_low().unwrap();
        asm::delay(one_second);
    }
}
