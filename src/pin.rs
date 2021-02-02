#[cfg(not(feature = "audio_hal"))]
pub use stm32h7xx_hal as hal;
#[cfg(feature = "audio_hal")]
pub use stm32h7xx_hal_dma as hal;

use hal::gpio;


#[allow(non_snake_case)]
pub struct Pins {
    // https://github.com/electro-smith/DaisyWiki/wiki/2.-Daisy-Seed-Pinout
    pub SEED_00: gpio::gpiob::PB12<gpio::Analog>,  // PIN_01, USB OTG ID, I2C3 SCL
    pub SEED_01: gpio::gpioc::PC11<gpio::Analog>,  // PIN_02, SD Data3, USART3 Rx
    pub SEED_02: gpio::gpioc::PC10<gpio::Analog>,  // PIN_03, SD Data2, USART3 Tx
    pub SEED_03: gpio::gpioc::PC9 <gpio::Analog>,  // PIN_04, SD Data1, I2C3 SDA
    pub SEED_04: gpio::gpioc::PC8 <gpio::Analog>,  // PIN_05, SD Data0
    pub SEED_05: gpio::gpiod::PD2 <gpio::Analog>,  // PIN_06, SD CMD, UART5 Rx
    pub SEED_06: gpio::gpioc::PC12<gpio::Analog>,  // PIN_07, SD CLK, UART5 Tx
    pub SEED_07: gpio::gpiog::PG10<gpio::Analog>,  // PIN_08, SPI1 CS
    pub SEED_08: gpio::gpiog::PG11<gpio::Analog>,  // PIN_09, SPI1 SCK, SPDIFRX1
    pub SEED_09: gpio::gpiob::PB4 <gpio::Alternate<gpio::AF0>>,  // PIN_10, SPI1 MOSI
    pub SEED_10: gpio::gpiob::PB5 <gpio::Analog>,  // PIN_11, SPI1 MISO
    pub SEED_11: gpio::gpiob::PB8 <gpio::Analog>,  // PIN_12, I2C1 SCL, UART4 Rx
    pub SEED_12: gpio::gpiob::PB9 <gpio::Analog>,  // PIN_13, I2C1 SDA, UART4 Tx
    pub SEED_13: gpio::gpiob::PB6 <gpio::Analog>,  // PIN_14, USART1 Tx, I2C4 SCL
    pub SEED_14: gpio::gpiob::PB7 <gpio::Analog>,  // PIN_15, USART1 Rx, I2C4 SDA
    pub SEED_15: gpio::gpioc::PC0 <gpio::Analog>,  // PIN_22, ADC 0
    pub SEED_16: gpio::gpioa::PA3 <gpio::Analog>,  // PIN_23, ADC 1
    pub SEED_17: gpio::gpiob::PB1 <gpio::Analog>,  // PIN_24, ADC 2
    pub SEED_18: gpio::gpioa::PA7 <gpio::Analog>,  // PIN_25, ADC 3
    pub SEED_19: gpio::gpioa::PA6 <gpio::Analog>,  // PIN_26, ADC 4
    pub SEED_20: gpio::gpioc::PC1 <gpio::Analog>,  // PIN_27, ADC 5
    pub SEED_21: gpio::gpioc::PC4 <gpio::Analog>,  // PIN_28, ADC 6
    pub SEED_22: gpio::gpioa::PA5 <gpio::Analog>,  // PIN_29, DAC OUT 2, ADC 7
    pub SEED_23: gpio::gpioa::PA4 <gpio::Analog>,  // PIN_30, DAC OUT 1, ADC 8
    pub SEED_24: gpio::gpioa::PA1 <gpio::Analog>,  // PIN_31, SAI2 MCLK, ADC 9
    pub SEED_25: gpio::gpioa::PA0 <gpio::Analog>,  // PIN_32, SAI2 SD B, ADC 10
    pub SEED_26: gpio::gpiod::PD11<gpio::Analog>,  // PIN_33, SAI2 SD A
    pub SEED_27: gpio::gpiog::PG9 <gpio::Analog>,  // PIN_34, SAI2 SD FS
    pub SEED_28: gpio::gpioa::PA2 <gpio::Analog>,  // PIN_35, SAI2 SCK, ADC 11
    //pub SEED_29: gpio::gpiob::PB14<gpio::Analog>,  // PIN_36, USB D-, USART1 Tx
    //pub SEED_30: gpio::gpiob::PB15<gpio::Analog>,  // PIN_37, USB D+, USART1 Rx

    // board
    //pub AK4556_PDN:  gpio::gpiob::PB11<gpio::Output<gpio::PushPull>>,
    //pub SAI1_MCLK_A: gpio::gpioe::PE2<gpio::Alternate<hal::gpio::AF6>>,
    //pub SAI1_SCK_A:  gpio::gpioe::PE5<gpio::Alternate<hal::gpio::AF6>>,
    //pub SAI1_FS_A:   gpio::gpioe::PE4<gpio::Alternate<hal::gpio::AF6>>,
    //pub SAI1_SD_A:   gpio::gpioe::PE6<gpio::Alternate<hal::gpio::AF6>>,
    //pub SAI1_SD_B:   gpio::gpioe::PE3<gpio::Alternate<hal::gpio::AF6>>,
}
