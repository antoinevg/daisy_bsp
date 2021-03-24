/*#![allow(dead_code)]
#![allow(non_snake_case)]
#![allow(unused_imports)]
#![allow(unused_unsafe)]
#![allow(unused_variables)]
*/

use stm32h7xx_hal as hal;
//use hal::prelude::*;

//use hal::pac;
//use hal::hal as embedded_hal;

use crate::audio;
use crate::clocks;
use crate::led;
use crate::pins::*;
use crate::usart;


// - global static state ------------------------------------------------------

// `no_mangle` is used here to prevent linking different minor
// versions of this crate as that would let you `take` the core
// peripherals more than once (one per minor version)
#[no_mangle]
static DAISY_BOARD: () = ();

/// Set to `true` when `take` was called to make `Board` a singleton.
static mut TAKEN: bool = false;




// - Board --------------------------------------------------------------------

pub struct Board;

impl Board {
    #[inline]
    pub fn take() -> Option<Self> {
        cortex_m::interrupt::free(|_| {
            if unsafe { TAKEN } {
                None
            } else {
                Some(unsafe { Board::steal() })
            }
        })
    }

    #[inline]
    pub unsafe fn steal() -> Self {
        Board
    }

    pub fn freeze_clocks(&self,
                         pwr: hal::pwr::Pwr,
                         rcc: hal::rcc::Rcc,
                         syscfg: &hal::device::SYSCFG) -> hal::rcc::Ccdr {
        clocks::configure(pwr, rcc, syscfg)
    }

    /// Takes the board's GPIO peripherals and split them into ZST's
    /// representing the individual GPIO pins used by the board.
    pub fn split_gpios(&self,
                       gpioa: hal::gpio::gpioa::Parts,
                       gpiob: hal::gpio::gpiob::Parts,
                       gpioc: hal::gpio::gpioc::Parts,
                       gpiod: hal::gpio::gpiod::Parts,
                       gpioe: hal::gpio::gpioe::Parts,
                       _gpiof: hal::gpio::gpiof::Parts,
                       gpiog: hal::gpio::gpiog::Parts,
                       _gpioh: hal::gpio::gpioh::Parts,
                       _gpioi: hal::gpio::gpioi::Parts,
                       _gpioj: hal::gpio::gpioj::Parts,
                       _gpiok: hal::gpio::gpiok::Parts) -> Pins {
        Pins {
            SEED_PIN_0: gpiob.pb12,
            SEED_PIN_1: gpioc.pc11,
            SEED_PIN_2: gpioc.pc10,
            SEED_PIN_3: gpioc.pc9,
            SEED_PIN_4: gpioc.pc8,
            SEED_PIN_5: gpiod.pd2,
            SEED_PIN_6: gpioc.pc12,
            SEED_PIN_7: gpiog.pg10,
            SEED_PIN_8: gpiog.pg11,
            SEED_PIN_9: gpiob.pb4,
            SEED_PIN_10: gpiob.pb5,
            SEED_PIN_11: gpiob.pb8,
            SEED_PIN_12: gpiob.pb9,
            SEED_PIN_13: gpiob.pb6,
            SEED_PIN_14: gpiob.pb7,
            SEED_PIN_15: gpioc.pc0,
            SEED_PIN_16: gpioa.pa3,
            SEED_PIN_17: gpiob.pb1,
            SEED_PIN_18: gpioa.pa7,
            SEED_PIN_19: gpioa.pa6,
            SEED_PIN_20: gpioc.pc1,
            SEED_PIN_21: gpioc.pc4,
            SEED_PIN_22: gpioa.pa5,
            SEED_PIN_23: gpioa.pa4,
            SEED_PIN_24: gpioa.pa1,
            SEED_PIN_25: gpioa.pa0,
            SEED_PIN_26: gpiod.pd11,
            SEED_PIN_27: gpiog.pg9,
            SEED_PIN_28: gpioa.pa2,
            SEED_PIN_29: gpiob.pb14,
            SEED_PIN_30: gpiob.pb15,
            LED_USER: gpioc.pc7,
            AK4556: AK4556Pins {
                PDN:    gpiob.pb11, // Codec Reset
                MCLK_A: gpioe.pe2,  // SAI1 MCLK_A
                SCK_A:  gpioe.pe5,  // SAI1 SCK_A
                FS_A:   gpioe.pe4,  // SAI1 FS_A
                SD_A:   gpioe.pe6,  // SAI1 SD_A
                SD_B:   gpioe.pe3,  // SAI1 SD_B
            },
            FMC: (),
            SDRAM: (),
            USB2: USB2Pins {
                DN: gpioa.pa11, // USB2 D-
                DP: gpioa.pa12, // USB2 D+
            }
        }
    }

    pub fn split_sai1(&self,
                      clocks: &hal::rcc::CoreClocks,
                      sai1_prec: hal::rcc::rec::Sai1,
                      #[allow(unused_variables)] dma1_prec: hal::rcc::rec::Dma1,
                      pins: AK4556Pins) -> audio::Interface {

        let pins = (pins.PDN.into_push_pull_output(),
                    pins.MCLK_A.into_alternate_af6(),
                    pins.SCK_A.into_alternate_af6(),
                    pins.FS_A.into_alternate_af6(),
                    pins.SD_A.into_alternate_af6(),
                    pins.SD_B.into_alternate_af6());


        let sai1_prec: hal::rcc::rec::Sai1 = sai1_prec.kernel_clk_mux(hal::rcc::rec::Sai1ClkSel::PLL3_P);

        #[cfg(not(feature = "audio_hal"))]
        let sai1_interface = audio::Interface::init(clocks,
                                                    sai1_prec,
                                                    pins).unwrap();
        #[cfg(feature = "audio_hal")]
        let sai1_interface = audio::Interface::init(clocks,
                                                    sai1_prec,
                                                    pins,
                                                    dma1_prec).unwrap();
        sai1_interface
    }

    pub fn split_led_user(&self, pin: LedUserPin) -> led::LedUser {
        led::LedUser::new(pin)
    }

    pub fn split_usart1(&self,
                        clocks: &hal::rcc::CoreClocks,
                        usart1_prec: hal::rcc::rec::Usart1,
                        pins: (SeedPin29, SeedPin30)) -> usart::Interface {
        let pins = (
            pins.0.into_alternate_af4(),  // USART1 TX - GPIO29 - Pin 36 <= GPIOB 14
            pins.1.into_alternate_af4(),  // USART1 RX - GPIO30 - Pin 37 => GPIOB 15
        );
        let usart1_interface = usart::Interface::init(clocks,
                                                      usart1_prec,
                                                      pins).unwrap();
        usart1_interface
    }
}


// - macros -------------------------------------------------------------------

#[macro_export]
macro_rules! board_freeze_clocks {
    ($board:expr, $dp:expr) => {
        {
            use daisy_bsp::hal::prelude::_stm32h7xx_hal_pwr_PwrExt;
            use daisy_bsp::hal::prelude::_stm32h7xx_hal_rcc_RccExt;
            $board.freeze_clocks($dp.PWR.constrain(),
                                 $dp.RCC.constrain(),
                                 &$dp.SYSCFG)
        }
    }
}


#[macro_export]
macro_rules! board_split_gpios {
    ($board:expr, $ccdr:expr, $dp:expr) => {
        {
            use daisy_bsp::hal::gpio::GpioExt;
            $board.split_gpios($dp.GPIOA.split($ccdr.peripheral.GPIOA),
                               $dp.GPIOB.split($ccdr.peripheral.GPIOB),
                               $dp.GPIOC.split($ccdr.peripheral.GPIOC),
                               $dp.GPIOD.split($ccdr.peripheral.GPIOD),
                               $dp.GPIOE.split($ccdr.peripheral.GPIOE),
                               $dp.GPIOF.split($ccdr.peripheral.GPIOF),
                               $dp.GPIOG.split($ccdr.peripheral.GPIOG),
                               $dp.GPIOH.split($ccdr.peripheral.GPIOH),
                               $dp.GPIOI.split($ccdr.peripheral.GPIOI),
                               $dp.GPIOJ.split($ccdr.peripheral.GPIOJ),
                               $dp.GPIOK.split($ccdr.peripheral.GPIOK))
        }
    }
}


#[macro_export]
macro_rules! board_split_audio {
    ($board:expr, $ccdr:expr, $pins:expr) => {
        {
            $board.split_sai1(&$ccdr.clocks,
                              $ccdr.peripheral.SAI1,
                              $ccdr.peripheral.DMA1,
                              $pins.AK4556)
        }
    }
}


#[macro_export]
macro_rules! board_split_midi {
    ($board:expr, $ccdr:expr, $pins:expr) => {
        {
            $board.split_usart1(&$ccdr.clocks,
                                $ccdr.peripheral.USART1,
                                ($pins.SEED_PIN_29, $pins.SEED_PIN_30))
        }
    }
}


#[macro_export]
macro_rules! board_split_leds {
    ($pins:expr) => {
        {
            daisy_bsp::led::Leds {
                USER: daisy_bsp::led::LedUser::new($pins.LED_USER)
            }
        }
    }
}
