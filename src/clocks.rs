use crate::hal;
use hal::pac;
use hal::prelude::*;
use hal::pwr;
use hal::rcc;
use hal::time::Hertz;
use hal::time::MegaHertz;

use crate::audio;

// - constants ----------------------------------------------------------------

// SAI clock uses pll3
const PLL3_P: Hertz = Hertz::Hz(audio::FS.raw() * 256);

// - types --------------------------------------------------------------------

pub trait SeedCrystal {
    const CRYSTAL_FREQ: MegaHertz = MegaHertz::MHz(16);

    fn use_seed_crystal(self) -> Self;
}

impl SeedCrystal for rcc::Rcc {
    fn use_seed_crystal(self) -> Self {
        self.use_hse(Self::CRYSTAL_FREQ.convert())
    }
}

// - configure ----------------------------------------------------------------

/// Configures the 16 MHz crystal, a 480 MHz system clock and PLL3 for
/// SAI audio
///
/// The Daisy Seed has a 16 MHz crystal wired to the MCU's high-speed
/// external oscillator pins. We enable that, and use it to drive the
/// full 480 MHz system clock.
///
/// Usage:
///
/// ```
/// let dp = pac::Peripherals::take().unwrap();
/// let ccdr = configure(dp.PWR.constrain(), dp.RCC.constrain(), &dp.SYSCFG);
/// let clocks = configure(rcc);
/// ```
pub fn configure(pwr: pwr::Pwr, rcc: rcc::Rcc, syscfg: &pac::SYSCFG) -> rcc::Ccdr {
    let pwrcfg = pwr.vos0(syscfg).freeze();

    #[cfg(not(feature = "log-itm"))]
    let ccdr = rcc
        .use_seed_crystal() // high speed external crystal @ 16 MHz
        .pll1_strategy(rcc::PllConfigStrategy::Iterative) // pll1 drives system clock
        .sys_ck(480.MHz()) // system clock @ 480 MHz
        .pll1_q_ck(48.MHz()) // spi display @ 48 MHz
        .pll3_p_ck(PLL3_P) // audio clock  @ 12.288 MHz
        .per_ck(4.MHz()) // peripheral clock @ 4 MHz
        .freeze(pwrcfg, syscfg);

    #[cfg(any(feature = "log-itm"))]
    let ccdr = rcc
        .use_seed_crystal() // high speed external crystal @ 16 MHz
        .pll1_strategy(rcc::PllConfigStrategy::Iterative) // pll1 drives system clock
        .sys_ck(480.MHz()) // system clock @ 480 MHz
        .pll1_q_ck(48.MHz()) // spi display @ 48 MHz
        .pll1_r_ck(480.MHz()) // for TRACECK
        .pll3_p_ck(PLL3_P) // audio clock  @ 12.288 MHz
        .per_ck(4.MHz()) // peripheral clock @ 4 MHz
        .freeze(pwrcfg, syscfg);

    // enable itm support
    #[cfg(any(feature = "log-itm"))]
    unsafe {
        let swo_frequency = 2_000_000;
        let mut cp = cortex_m::Peripherals::steal();
        let dp = pac::Peripherals::steal();
        crate::itm::enable_itm(
            &mut cp.DCB,
            &dp.DBGMCU,
            &mut cp.ITM,
            ccdr.clocks.c_ck().raw(),
            swo_frequency,
        );
    }

    ccdr
}
