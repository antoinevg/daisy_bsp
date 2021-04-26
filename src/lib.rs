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

pub mod audio;
pub mod board;
pub mod clocks;
#[cfg(any(feature = "log-itm"))]
pub mod itm;
pub mod led;
pub mod midi;
pub mod pins;

pub use board::Board;
