#[cfg(not(feature = "audio_hal"))]
pub use stm32h7xx_hal as hal;
#[cfg(feature = "audio_hal")]
pub use stm32h7xx_hal_dma as hal;

use hal::hal as embedded_hal;

use hal::gpio;
use embedded_hal::digital::v2::OutputPin;


// - traits -------------------------------------------------------------------

/// Generic LED
pub trait Led {
    /// Turns the LED off
    fn off(&mut self);

    /// Turns the LED on
    fn on(&mut self);
}


// - types --------------------------------------------------------------------

#[allow(non_snake_case)]
pub struct Leds {
    pub USER: LedUser,
}

pub struct LedUser(pub gpio::gpioc::PC7<gpio::Output<gpio::PushPull>>);

impl LedUser {
    pub fn new<Mode>(pin: gpio::gpioc::PC7<Mode>) -> Self {
        LedUser(pin.into_push_pull_output())
    }
}

impl Led for LedUser {
    fn on(&mut self) {
        self.0.set_high().unwrap();
    }

    fn off(&mut self) {
        self.0.set_low().unwrap();
    }
}
