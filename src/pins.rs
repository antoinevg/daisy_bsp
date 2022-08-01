use stm32h7xx_hal as hal;

// - types --------------------------------------------------------------------

pub type SeedPin0 = hal::gpio::gpiob::PB12<hal::gpio::Analog>; // PIN_01, USB OTG ID, I2C3 SCL
pub type SeedPin1 = hal::gpio::gpioc::PC11<hal::gpio::Analog>; // PIN_02, SD Data3, USART3 Rx
pub type SeedPin2 = hal::gpio::gpioc::PC10<hal::gpio::Analog>; // PIN_03, SD Data2, USART3 Tx
pub type SeedPin3 = hal::gpio::gpioc::PC9<hal::gpio::Analog>; // PIN_04, SD Data1, I2C3 SDA
pub type SeedPin4 = hal::gpio::gpioc::PC8<hal::gpio::Analog>; // PIN_05, SD Data0
pub type SeedPin5 = hal::gpio::gpiod::PD2<hal::gpio::Analog>; // PIN_06, SD CMD, UART5 Rx
pub type SeedPin6 = hal::gpio::gpioc::PC12<hal::gpio::Analog>; // PIN_07, SD CLK, UART5 Tx
pub type SeedPin7 = hal::gpio::gpiog::PG10<hal::gpio::Analog>; // PIN_08, SPI1 CS
pub type SeedPin8 = hal::gpio::gpiog::PG11<hal::gpio::Analog>; // PIN_09, SPI1 SCK, SPDIFRX1
pub type SeedPin9 = hal::gpio::gpiob::PB4<hal::gpio::Alternate<0>>; // PIN_10, SPI1 MOSI
pub type SeedPin10 = hal::gpio::gpiob::PB5<hal::gpio::Analog>; // PIN_11, SPI1 MISO
pub type SeedPin11 = hal::gpio::gpiob::PB8<hal::gpio::Analog>; // PIN_12, I2C1 SCL, UART4 Rx
pub type SeedPin12 = hal::gpio::gpiob::PB9<hal::gpio::Analog>; // PIN_13, I2C1 SDA, UART4 Tx
pub type SeedPin13 = hal::gpio::gpiob::PB6<hal::gpio::Analog>; // PIN_14, USART1 Tx, I2C4 SCL
pub type SeedPin14 = hal::gpio::gpiob::PB7<hal::gpio::Analog>; // PIN_15, USART1 Rx, I2C4 SDA
pub type SeedPin15 = hal::gpio::gpioc::PC0<hal::gpio::Analog>; // PIN_22, ADC 0
pub type SeedPin16 = hal::gpio::gpioa::PA3<hal::gpio::Analog>; // PIN_23, ADC 1
pub type SeedPin17 = hal::gpio::gpiob::PB1<hal::gpio::Analog>; // PIN_24, ADC 2
pub type SeedPin18 = hal::gpio::gpioa::PA7<hal::gpio::Analog>; // PIN_25, ADC 3
pub type SeedPin19 = hal::gpio::gpioa::PA6<hal::gpio::Analog>; // PIN_26, ADC 4
pub type SeedPin20 = hal::gpio::gpioc::PC1<hal::gpio::Analog>; // PIN_27, ADC 5
pub type SeedPin21 = hal::gpio::gpioc::PC4<hal::gpio::Analog>; // PIN_28, ADC 6
pub type SeedPin22 = hal::gpio::gpioa::PA5<hal::gpio::Analog>; // PIN_29, DAC OUT 2, ADC 7
pub type SeedPin23 = hal::gpio::gpioa::PA4<hal::gpio::Analog>; // PIN_30, DAC OUT 1, ADC 8
pub type SeedPin24 = hal::gpio::gpioa::PA1<hal::gpio::Analog>; // PIN_31, SAI2 MCLK, ADC 9
pub type SeedPin25 = hal::gpio::gpioa::PA0<hal::gpio::Analog>; // PIN_32, SAI2 SD B, ADC 10
pub type SeedPin26 = hal::gpio::gpiod::PD11<hal::gpio::Analog>; // PIN_33, SAI2 SD A
pub type SeedPin27 = hal::gpio::gpiog::PG9<hal::gpio::Analog>; // PIN_34, SAI2 SD FS
pub type SeedPin28 = hal::gpio::gpioa::PA2<hal::gpio::Analog>; // PIN_35, SAI2 SCK, ADC 11
pub type SeedPin29 = hal::gpio::gpiob::PB14<hal::gpio::Analog>; // PIN_36, USB1 D-, USART1 Tx
pub type SeedPin30 = hal::gpio::gpiob::PB15<hal::gpio::Analog>; // PIN_37, USB1 D+, USART1 Rx

pub type LedUserPin = hal::gpio::gpioc::PC7<hal::gpio::Analog>; // LED_USER

#[allow(non_snake_case)]
pub struct AK4556Pins {
    pub PDN: hal::gpio::gpiob::PB11<hal::gpio::Analog>, // Codec Reset
    pub MCLK_A: hal::gpio::gpioe::PE2<hal::gpio::Analog>, // SAI1 MCLK_A
    pub SCK_A: hal::gpio::gpioe::PE5<hal::gpio::Analog>, // SAI1 SCK_A
    pub FS_A: hal::gpio::gpioe::PE4<hal::gpio::Analog>, // SAI1 FS_A
    pub SD_A: hal::gpio::gpioe::PE6<hal::gpio::Analog>, // SAI1 SD_A
    pub SD_B: hal::gpio::gpioe::PE3<hal::gpio::Analog>, // SAI1 SD_B
}

#[allow(non_snake_case)]
pub struct USB2Pins {
    pub DN: hal::gpio::gpioa::PA11<hal::gpio::Analog>, // USB2 D-
    pub DP: hal::gpio::gpioa::PA12<hal::gpio::Analog>, // USB2 D+
}

#[allow(non_snake_case)]
pub struct FMCPins {
    // https://github.com/electro-smith/libDaisy/blob/3dda55e9ed55a2f8b6bc4fa6aa2c7ae134c317ab/src/per/qspi.c#L695
    pub IO0: hal::gpio::gpiof::PF8<hal::gpio::Analog>, // (SI)
    pub IO1: hal::gpio::gpiof::PF9<hal::gpio::Analog>, // (SO)
    pub IO2: hal::gpio::gpiof::PF7<hal::gpio::Analog>,
    pub IO3: hal::gpio::gpiof::PF6<hal::gpio::Analog>,
    pub SCK: hal::gpio::gpiof::PF10<hal::gpio::Analog>,
    pub CS: hal::gpio::gpiog::PG6<hal::gpio::Analog>,
}

// - Pins ---------------------------------------------------------------------

#[allow(non_snake_case)]
pub struct Pins {
    // https://github.com/electro-smith/DaisyWiki/wiki/2.-Daisy-Seed-Pinout
    pub SEED_PIN_0: SeedPin0,
    pub SEED_PIN_1: SeedPin1,
    pub SEED_PIN_2: SeedPin2,
    pub SEED_PIN_3: SeedPin3,
    pub SEED_PIN_4: SeedPin4,
    pub SEED_PIN_5: SeedPin5,
    pub SEED_PIN_6: SeedPin6,
    pub SEED_PIN_7: SeedPin7,
    pub SEED_PIN_8: SeedPin8,
    pub SEED_PIN_9: SeedPin9,
    pub SEED_PIN_10: SeedPin10,
    pub SEED_PIN_11: SeedPin11,
    pub SEED_PIN_12: SeedPin12,
    pub SEED_PIN_13: SeedPin13,
    pub SEED_PIN_14: SeedPin14,
    pub SEED_PIN_15: SeedPin15,
    pub SEED_PIN_16: SeedPin16,
    pub SEED_PIN_17: SeedPin17,
    pub SEED_PIN_18: SeedPin18,
    pub SEED_PIN_19: SeedPin19,
    pub SEED_PIN_20: SeedPin20,
    pub SEED_PIN_21: SeedPin21,
    pub SEED_PIN_22: SeedPin22,
    pub SEED_PIN_23: SeedPin23,
    pub SEED_PIN_24: SeedPin24,
    pub SEED_PIN_25: SeedPin25,
    pub SEED_PIN_26: SeedPin26,
    pub SEED_PIN_27: SeedPin27,
    pub SEED_PIN_28: SeedPin28,
    pub SEED_PIN_29: SeedPin29,
    pub SEED_PIN_30: SeedPin30,

    // board peripherals
    pub LED_USER: LedUserPin,
    pub AK4556: AK4556Pins,
    pub FMC: FMCPins,
    pub SDRAM: (), // TODO
    pub USB2: USB2Pins,
}
