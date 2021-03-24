#![feature(alloc_error_handler)]

//! Board support crate for Daisy hardware
//!
//! # Usage - see examples/
//! ```

#![deny(warnings)]
#![no_std]

pub use stm32h7xx_hal as hal;
pub use hal::hal as embedded_hal;
pub use hal::pac;


// - alloc --------------------------------------------------------------------

extern crate alloc;
use static_alloc::Bump;

#[global_allocator]
static A: Bump<[u8; 1 << 12]> = Bump::uninit();

#[alloc_error_handler]
fn on_oom(_layout: core::alloc::Layout) -> ! {
    cortex_m::asm::bkpt();
    loop {}
}


// - modules ------------------------------------------------------------------

pub mod board;
pub use board::Board;
pub mod clocks;
pub mod led;
pub mod pins;
pub mod midi;

// TODO proper logging with compile-time feature selection of: semihosting/itm/rtt
//pub mod itm;
//#[macro_use]
//pub mod itm_macros;

#[cfg(not(feature = "audio_hal"))]
pub mod audio;
#[cfg(feature = "audio_hal")]
pub mod audio_hal;
#[cfg(feature = "audio_hal")]
pub use crate::audio_hal as audio;
