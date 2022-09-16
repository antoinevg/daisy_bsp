use hal::gpio;
pub use stm32h7xx_hal as hal;

// - traits -------------------------------------------------------------------

/// Generic LED
pub trait Led {
    /// Turns the LED off
    fn off(&mut self);

    /// Turns the LED on
    fn on(&mut self);
}

// - types --------------------------------------------------------------------

pub struct UserLed(pub gpio::gpioc::PC7<gpio::Output<gpio::PushPull>>);

impl UserLed {
    pub fn new(pin: gpio::gpioc::PC7<hal::gpio::Analog>) -> Self {
        UserLed(pin.into_push_pull_output())
    }
}

impl Led for UserLed {
    fn on(&mut self) {
        self.0.set_high();
    }

    fn off(&mut self) {
        self.0.set_low()
    }
}

#[allow(non_snake_case)]
pub struct UserLeds {
    pub user: UserLed,
}
