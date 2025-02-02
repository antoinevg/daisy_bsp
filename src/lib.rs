#![cfg_attr(feature = "alloc", feature(alloc_error_handler))]
#![deny(warnings)]
#![no_std]

//! Board support crate for Daisy hardware
//!
//! # Usage - see examples/

// - modules ------------------------------------------------------------------

#[cfg(any(feature = "alloc"))]
pub mod alloc;
pub mod audio;
pub mod board;
pub mod clocks;
pub mod flash;
#[cfg(any(feature = "log-itm"))]
pub mod itm;
pub mod led;
pub mod midi;
pub mod pins;
#[cfg(any(feature = "fmc"))]
pub mod sdram;

// - log macros ---------------------------------------------------------------

#[cfg(any(feature = "log-itm"))]
#[macro_export]
macro_rules! loggit {
    ($($arg:tt)*) => (
        let itm = unsafe { &mut *cortex_m::peripheral::ITM::PTR };
        cortex_m::iprintln!(&mut itm.stim[0], $($arg)*);
    )
}

#[cfg(not(feature = "log-itm"))]
#[macro_export]
macro_rules! loggit {
    ($($arg:tt)*) => (
        cortex_m_semihosting::hprintln!($($arg)*);
    )
}

// - exports ------------------------------------------------------------------

pub use hal::hal as embedded_hal;
pub use hal::pac;
pub use stm32h7xx_hal as hal;

pub use board::Board;
