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
                unsafe {
                    TAKEN = true;
                }
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
        #[allow(unused_variables)] // GPIO H is only utilized on Daisy 1.1
        gpioh: hal::gpio::gpioh::Parts,
        gpioi: hal::gpio::gpioi::Parts,
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
            SDRAM: SDRAMPins {
                A0: gpiof.pf0,
                A1: gpiof.pf1,
                A2: gpiof.pf2,
                A3: gpiof.pf3,
                A4: gpiof.pf4,
                A5: gpiof.pf5,
                A6: gpiof.pf12,
                A7: gpiof.pf13,
                A8: gpiof.pf14,
                A9: gpiof.pf15,
                A10: gpiog.pg0,
                A11: gpiog.pg1,
                A12: gpiog.pg2,
                BA0: gpiog.pg4,
                BA1: gpiog.pg5,
                D0: gpiod.pd14,
                D1: gpiod.pd15,
                D2: gpiod.pd0,
                D3: gpiod.pd1,
                D4: gpioe.pe7,
                D5: gpioe.pe8,
                D6: gpioe.pe9,
                D7: gpioe.pe10,
                D8: gpioe.pe11,
                D9: gpioe.pe12,
                D10: gpioe.pe13,
                D11: gpioe.pe14,
                D12: gpioe.pe15,
                D13: gpiod.pd8,
                D14: gpiod.pd9,
                D15: gpiod.pd10,
                D16: gpioh.ph8,
                D17: gpioh.ph9,
                D18: gpioh.ph10,
                D19: gpioh.ph11,
                D20: gpioh.ph12,
                D21: gpioh.ph13,
                D22: gpioh.ph14,
                D23: gpioh.ph15,
                D24: gpioi.pi0,
                D25: gpioi.pi1,
                D26: gpioi.pi2,
                D27: gpioi.pi3,
                D28: gpioi.pi6,
                D29: gpioi.pi7,
                D30: gpioi.pi9,
                D31: gpioi.pi10,
                NBL0: gpioe.pe0,
                NBL1: gpioe.pe1,
                NBL2: gpioi.pi4,
                NBL3: gpioi.pi5,
                SDCKE0: gpioh.ph2,
                SDCLK: gpiog.pg8,
                SDNCAS: gpiog.pg15,
                SDNE0: gpioh.ph3,
                SDRAS: gpiof.pf11,
                SDNWE: gpioh.ph5,
            },
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

/// Configure SDRAM memory and retrieve its handle.
#[macro_export]
macro_rules! board_split_sdram {
    ($cp:expr, $dp:expr, $ccdr:expr, $pins:expr) => {{
        use daisy_bsp::hal::delay::DelayFromCountDownTimer;
        use daisy_bsp::hal::prelude::*;
        let mut delay = DelayFromCountDownTimer::new($dp.TIM3.timer(
            100.Hz(),
            $ccdr.peripheral.TIM3,
            &$ccdr.clocks,
        ));
        let sdram = daisy_bsp::sdram::SDRAM::new(
            $pins.SDRAM,
            &$ccdr.clocks,
            $dp.FMC,
            $ccdr.peripheral.FMC,
            &mut $cp.MPU,
            &mut $cp.SCB,
            &mut delay,
        );
        delay.free().free();
        sdram
    }};
}
