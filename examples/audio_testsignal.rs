#![allow(unused_imports)]

#![no_main]
#![no_std]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use cortex_m::asm;
use cortex_m_rt::entry;
use panic_itm as _;

use daisy_bsp as daisy;

use daisy::hal;
use hal::prelude::*;

use daisy::pac;
use pac::interrupt;

//use daisy::audio;
use daisy::audio_wm8731 as audio;
use daisy::led::Led;
use daisy::loggit;

mod device;
use device::wm8731;

mod dsp;
use dsp::osc;

// - static global state ------------------------------------------------------

static AUDIO_INTERFACE: Mutex<RefCell<Option<audio::Interface>>> = Mutex::new(RefCell::new(None));

// - entry point --------------------------------------------------------------

#[entry]
fn main() -> ! {
    // - board setup ----------------------------------------------------------

    let board = daisy::Board::take().unwrap();

    let dp = pac::Peripherals::take().unwrap();

    let mut ccdr = board.freeze_clocks(dp.PWR.constrain(), dp.RCC.constrain(), &dp.SYSCFG);

    // configure SAI2 clock ???
    ccdr.peripheral.kernel_sai23_clk_mux(hal::rcc::rec::Sai23ClkSel::Pll3P);

    let pins = board.split_gpios(
        dp.GPIOA.split(ccdr.peripheral.GPIOA),
        dp.GPIOB.split(ccdr.peripheral.GPIOB),
        dp.GPIOC.split(ccdr.peripheral.GPIOC),
        dp.GPIOD.split(ccdr.peripheral.GPIOD),
        dp.GPIOE.split(ccdr.peripheral.GPIOE),
        dp.GPIOF.split(ccdr.peripheral.GPIOF),
        dp.GPIOG.split(ccdr.peripheral.GPIOG),
    );

    let mut led_user = daisy::led::UserLed::new(pins.LED_USER);

    // - ak4556 ---------------------------------------------------------------

    /*let pins = (
        pins.AK4556.PDN.into_push_pull_output(),
        pins.AK4556.MCLK_A.into_alternate(),
        pins.AK4556.SCK_A.into_alternate(),
        pins.AK4556.FS_A.into_alternate(),
        pins.AK4556.SD_A.into_alternate(),
        pins.AK4556.SD_B.into_alternate(),
    );

    let sai1_prec = ccdr
        .peripheral
        .SAI1
        .kernel_clk_mux(hal::rcc::rec::Sai1ClkSel::Pll3P);

    let audio_interface =
        audio::Interface::init(&ccdr.clocks, sai1_prec, pins, ccdr.peripheral.DMA1).unwrap();
     */

    // - wm8731 ---------------------------------------------------------------
    loggit!("\nstarting up...");
    use hal::gpio;
    pub type I2c1Pins = (
        gpio::gpiob::PB8<gpio::Alternate<4, gpio::OpenDrain>>,  // I2C1_SCL
        gpio::gpiob::PB9<gpio::Alternate<4, gpio::OpenDrain>>,  // I2C1_SDA
    );
    let i2c1_pins: I2c1Pins = (
        pins.SEED_PIN_11.into_alternate_open_drain(),       // I2C1 SCL - pb8
        pins.SEED_PIN_12.into_alternate_open_drain(),       // I2C1 SDA - pb9
    );

    // configure wm8731 over i2c
    loggit!("configure WM8731");
    let mut i2c = hal::i2c::I2cExt::i2c(
        unsafe { pac::Peripherals::steal().I2C1 },
        i2c1_pins,
        100_000.Hz(),
        ccdr.peripheral.I2C1,
        &ccdr.clocks,
    );
    let device_id = wm8731::DEVICE_ID_A;
    for (register, value) in wm8731::DEFAULT_CONFIG {
        //loggit!("configure wm8731 register: {:?}", register);

        let register = *register as u8;

        // 7 bit register address + 9 bits of data
        let frame = [
            ((register << 1) & 0xfe) | ((value >> 8) & 0x01) as u8,
            (value & 0xff) as u8,
        ];

        match i2c.write(device_id, &frame) {
            Ok(()) => (),
            Err(e) => {
                loggit!("error configuring wm8731, aborting: {:?}", e);
                panic!("error configuring wm8731, aborting.");
            }
        }
        cortex_m::asm::delay(50_000);
    }

    let sai2_pins = (
        pins.SEED_PIN_24.into_alternate(),       // SAI2 MCLK
        pins.SEED_PIN_28.into_alternate(),       // SAI2 SCK
        pins.SEED_PIN_27.into_alternate(),       // SAI2 FS
        pins.SEED_PIN_25.into_alternate(),       // SAI2 SD B
        pins.SEED_PIN_26.into_alternate(),       // SAI2 SD A
    );

    use hal::rcc::ResetEnable;
    //let sai2_prec = ccdr.peripheral.SAI2.enable().reset();
    let sai2_prec = ccdr.peripheral.SAI2;

    let audio_interface = audio::Interface::init(
        &ccdr.clocks,
        sai2_prec,
        sai2_pins,
        ccdr.peripheral.DMA1
    ).unwrap();

    // - audio callback -------------------------------------------------------

    // handle callback with function pointer
    #[cfg(not(feature = "alloc"))]
    let audio_interface = {
        fn callback(fs: f32, block: &mut audio::Block) {
            static mut OSC_1: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);
            static mut OSC_2: osc::Wavetable = osc::Wavetable::new(osc::Shape::Saw);
            unsafe { OSC_1.dx = (1. / fs) * 110.00 };
            unsafe { OSC_2.dx = (1. / fs) * 110.00 };
            for frame in block {
                *frame = (unsafe { OSC_1.step() }, unsafe { OSC_2.step() });
                //let (_left, right) = *frame;
                //*frame = (unsafe { OSC_1.step() }, right);
            }
        }

        audio_interface.spawn(callback)
    };

    // handle callback with closure (needs alloc)
    #[cfg(any(feature = "alloc"))]
    let audio_interface = {
        let mut osc_1: osc::Wavetable = osc::Wavetable::new(osc::Shape::Sin);
        let mut osc_2: osc::Wavetable = osc::Wavetable::new(osc::Shape::Saw);

        audio_interface.spawn(move |fs, block| {
            osc_1.dx = (1. / fs) * 110.00;
            osc_2.dx = (1. / fs) * 110.00;
            for frame in block {
                *frame = (osc_1.step(), osc_2.step());
            }
        })
    };

    let audio_interface = match audio_interface {
        Ok(audio_interface) => audio_interface,
        Err(e) => {
            loggit!("Failed to start audio interface: {:?}", e);
            loop {}
        }
    };

    cortex_m::interrupt::free(|cs| {
        AUDIO_INTERFACE.borrow(cs).replace(Some(audio_interface));
    });

    loggit!("finished, intering main loop");

    // - main loop ------------------------------------------------------------

    let one_second = ccdr.clocks.sys_ck().raw();

    loop {
        led_user.on();
        asm::delay(one_second);
        led_user.off();
        asm::delay(one_second);
    }
}

// - interrupts ---------------------------------------------------------------

/// interrupt handler for: dma1, stream0 - rx
#[interrupt]
fn DMA1_STR0() {
    unsafe {
        static mut COUNTER: usize = 0;
        if COUNTER % 1000_000 == 0 {
            loggit!("irq: dma1_str0 {}", COUNTER);
        }
        COUNTER += 1;
    }
    cortex_m::interrupt::free(|cs| {
        if let Some(audio_interface) = AUDIO_INTERFACE.borrow(cs).borrow_mut().as_mut() {
            match audio_interface.handle_interrupt_dma1_str0() {
                Ok(()) => (),
                Err(e) => {
                    loggit!("Failed to handle interrupt: {:?}", e);
                }
            };
        }
    });
}

/*
/// interrupt handler for: dma1, stream1 - tx
#[interrupt]
fn DMA1_STR1() {
    unsafe {
        static mut COUNTER: usize = 0;
        if COUNTER % 1000_000 == 0 {
            loggit!("irq: dma1_str1 {}", COUNTER);
        }
        COUNTER += 1;
    }
    cortex_m::interrupt::free(|cs| {
        if let Some(audio_interface) = AUDIO_INTERFACE.borrow(cs).borrow_mut().as_mut() {
            match audio_interface.handle_interrupt_dma1_str1() {
                Ok(()) => (),
                Err(e) => {
                    loggit!("Failed to handle interrupt: {:?}", e);
                }
            };
        }
    });
}
*/
