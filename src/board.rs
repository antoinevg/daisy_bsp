use stm32h7xx_hal as hal;

use crate::clocks;
use crate::pins::*;

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

    pub fn freeze_clocks(
        &self,
        pwr: hal::pwr::Pwr,
        rcc: hal::rcc::Rcc,
        syscfg: &hal::device::SYSCFG,
    ) -> hal::rcc::Ccdr {
        clocks::configure(pwr, rcc, syscfg)
    }

    /// Takes the board's GPIO peripherals and split them into ZST's
    /// representing the individual GPIO pins used by the board.
    pub fn split_gpios(
        &self,
        gpioa: hal::gpio::gpioa::Parts,
        gpiob: hal::gpio::gpiob::Parts,
        gpioc: hal::gpio::gpioc::Parts,
        gpiod: hal::gpio::gpiod::Parts,
        gpioe: hal::gpio::gpioe::Parts,
        gpiof: hal::gpio::gpiof::Parts,
        gpiog: hal::gpio::gpiog::Parts,
    ) -> Pins {
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
                PDN: gpiob.pb11,   // Codec Reset
                MCLK_A: gpioe.pe2, // SAI1 MCLK_A
                SCK_A: gpioe.pe5,  // SAI1 SCK_A
                FS_A: gpioe.pe4,   // SAI1 FS_A
                SD_A: gpioe.pe6,   // SAI1 SD_A
                SD_B: gpioe.pe3,   // SAI1 SD_B
            },
            FMC: FMCPins {
                IO0: gpiof.pf8,
                IO1: gpiof.pf9,
                IO2: gpiof.pf7,
                IO3: gpiof.pf6,
                SCK: gpiof.pf10,
                CS: gpiog.pg6,
            },
            SDRAM: (),
            USB2: USB2Pins {
                DN: gpioa.pa11, // USB2 D-
                DP: gpioa.pa12, // USB2 D+
            },
        }
    }
}

// - macros -------------------------------------------------------------------

#[macro_export]
macro_rules! board_freeze_clocks {
    ($board:expr, $dp:expr) => {{
        use daisy_bsp::hal::prelude::_stm32h7xx_hal_pwr_PwrExt;
        use daisy_bsp::hal::prelude::_stm32h7xx_hal_rcc_RccExt;
        $board.freeze_clocks($dp.PWR.constrain(), $dp.RCC.constrain(), &$dp.SYSCFG)
    }};
}

#[macro_export]
macro_rules! board_split_gpios {
    ($board:expr, $ccdr:expr, $dp:expr) => {{
        use daisy_bsp::hal::gpio::GpioExt;
        $board.split_gpios(
            $dp.GPIOA.split($ccdr.peripheral.GPIOA),
            $dp.GPIOB.split($ccdr.peripheral.GPIOB),
            $dp.GPIOC.split($ccdr.peripheral.GPIOC),
            $dp.GPIOD.split($ccdr.peripheral.GPIOD),
            $dp.GPIOE.split($ccdr.peripheral.GPIOE),
            $dp.GPIOF.split($ccdr.peripheral.GPIOF),
            $dp.GPIOG.split($ccdr.peripheral.GPIOG),
        )
    }};
}

#[macro_export]
macro_rules! board_split_audio {
    ($ccdr:expr, $pins:expr) => {{
        let pins = (
            $pins.AK4556.PDN.into_push_pull_output(),
            $pins.AK4556.MCLK_A.into_alternate(),
            $pins.AK4556.SCK_A.into_alternate(),
            $pins.AK4556.FS_A.into_alternate(),
            $pins.AK4556.SD_A.into_alternate(),
            $pins.AK4556.SD_B.into_alternate(),
        );

        let sai1_prec = $ccdr
            .peripheral
            .SAI1
            .kernel_clk_mux(daisy_bsp::hal::rcc::rec::Sai1ClkSel::PLL3_P);

        daisy_bsp::audio::Interface::init(&$ccdr.clocks, sai1_prec, pins, $ccdr.peripheral.DMA1)
            .unwrap()
    }};
}

#[macro_export]
macro_rules! board_split_midi {
    ($ccdr:expr, $pins:expr) => {{
        let pins = (
            $pins.SEED_PIN_13.into_alternate(), // USART1 TX
            $pins.SEED_PIN_14.into_alternate(), // USART1 RX
        );

        daisy_bsp::midi::Interface::init(&$ccdr.clocks, $ccdr.peripheral.USART1, pins).unwrap()
    }};
}

#[macro_export]
macro_rules! board_split_leds {
    ($pins:expr) => {{
        daisy_bsp::led::Leds {
            USER: daisy_bsp::led::LedUser::new($pins.LED_USER),
        }
    }};
}
