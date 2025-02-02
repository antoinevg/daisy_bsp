//! Configuration of and access to the on-board SDRAM memory.
//!
//! From: https://github.com/zlosynth/daisy/blob/main/src/sdram.rs
//!
//! Based on the HAL example and libdaisy-rust.
//! * <https://github.com/stm32-rs/stm32h7xx-hal/blob/99b409d4c1a58795690719bee08e8e5cb8cd3449/examples/fmc.rs>
//! * <https://github.com/x37v/libdaisy-rust/blob/develop/examples/sdram.rs>

use stm32_fmc::devices::as4c16m32msa_6;

use crate::hal;
use crate::pins::SDRAMPins;
use hal::fmc::FmcExt;
use hal::gpio::Speed;
use hal::hal::blocking::delay::DelayUs;

const SIZE: usize = 64 * 1024 * 1024;

// Refer to ARMv7-M Architecture Reference Manual ARM DDI 0403
// Version E.b Section B3.5
const MEMFAULTENA: u32 = 1 << 16;

pub struct SDRAM {
    pub base_address: *mut u32,
}

impl SDRAM {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        pins: SDRAMPins,
        clocks: &hal::rcc::CoreClocks,
        dp_fmc: hal::device::FMC,
        ccdr_fmc: hal::rcc::rec::Fmc,
        cp_mpu: &mut hal::pac::MPU,
        cp_scb: &mut hal::pac::SCB,
        delay: &mut impl DelayUs<u8>,
    ) -> Self {
        disable_mpu(cp_mpu, cp_scb);
        let base_address = initialize_sdram(pins, clocks, dp_fmc, ccdr_fmc, delay);
        configure_mpu_for_sdram(cp_mpu, base_address);
        enable_mpu(cp_mpu, cp_scb);
        Self { base_address }
    }
}

impl SDRAM {
    pub fn size(&self) -> usize {
        SIZE
    }
}

fn initialize_sdram(
    pins: SDRAMPins,
    clocks: &hal::rcc::CoreClocks,
    dp_fmc: hal::device::FMC,
    ccdr_fmc: hal::rcc::rec::Fmc,
    delay: &mut impl DelayUs<u8>,
) -> *mut u32 {
    macro_rules! configure_pins {
        ($($pin:expr),*) => {
            (
                $(
                    $pin.into_push_pull_output()
                        .speed(Speed::VeryHigh)
                        .into_alternate::<12>()
                        .internal_pull_up(true)
                ),*
            )
        };
    }

    let sdram_pins = configure_pins! {
        pins.A0, pins.A1, pins.A2, pins.A3, pins.A4, pins.A5, pins.A6, pins.A7,
        pins.A8, pins.A9, pins.A10, pins.A11, pins.A12, pins.BA0, pins.BA1,
        pins.D0, pins.D1, pins.D2, pins.D3, pins.D4, pins.D5, pins.D6, pins.D7,
        pins.D8, pins.D9, pins.D10, pins.D11, pins.D12, pins.D13, pins.D14,
        pins.D15, pins.D16, pins.D17, pins.D18, pins.D19, pins.D20, pins.D21,
        pins.D22, pins.D23, pins.D24, pins.D25, pins.D26, pins.D27, pins.D28,
        pins.D29, pins.D30, pins.D31, pins.NBL0, pins.NBL1, pins.NBL2,
        pins.NBL3, pins.SDCKE0, pins.SDCLK, pins.SDNCAS, pins.SDNE0, pins.SDRAS,
        pins.SDNWE
    };

    dp_fmc
        .sdram(
            sdram_pins,
            as4c16m32msa_6::As4c16m32msa {},
            ccdr_fmc,
            clocks,
        )
        .init(delay)
}

/// Configure region 0.
///
/// Cacheable, outer and inner write-back, no write allocate. So reads are
/// cached, but writes always write all the way to SDRAM.
fn configure_mpu_for_sdram(cp_mpu: &mut hal::pac::MPU, base_address: *mut u32) {
    const REGION_NUMBER0: u32 = 0x00;
    const REGION_FULL_ACCESS: u32 = 0x03;
    const REGION_CACHEABLE: u32 = 0x01;
    const REGION_WRITE_BACK: u32 = 0x01;
    const REGION_ENABLE: u32 = 0x01;
    unsafe {
        cp_mpu.rnr.write(REGION_NUMBER0);
        cp_mpu.rbar.write((base_address as u32) & !0x1F);
        cp_mpu.rasr.write(
            (REGION_FULL_ACCESS << 24)
                | (REGION_CACHEABLE << 17)
                | (REGION_WRITE_BACK << 16)
                | (log2minus1(SIZE as u32) << 1)
                | REGION_ENABLE,
        );
    }
}

fn log2minus1(sz: u32) -> u32 {
    for i in 5..=31 {
        if sz == (1 << i) {
            return i - 1;
        }
    }
    panic!("Unable to calculate");
}

/// Enable MPU.
fn enable_mpu(cp_mpu: &mut hal::pac::MPU, cp_scb: &mut hal::pac::SCB) {
    const MPU_ENABLE: u32 = 0x01;
    const MPU_DEFAULT_MMAP_FOR_PRIVILEGED: u32 = 0x04;
    unsafe {
        cp_mpu
            .ctrl
            .modify(|r| r | MPU_DEFAULT_MMAP_FOR_PRIVILEGED | MPU_ENABLE);

        cp_scb.shcsr.modify(|r| r | MEMFAULTENA);

        // Ensure MPU settings take effect.
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }
}

/// Disable and reset MPU.
fn disable_mpu(cp_mpu: &mut hal::pac::MPU, cp_scb: &mut hal::pac::SCB) {
    unsafe {
        // Make sure outstanding transfers are done.
        cortex_m::asm::dmb();

        cp_scb.shcsr.modify(|r| r & !MEMFAULTENA);

        // Disable the MPU and clear the control register.
        cp_mpu.ctrl.write(0);
    }
}
