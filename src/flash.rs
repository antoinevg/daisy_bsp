//! Basic driver for IS25LP064.

use crate::hal;
use crate::hal::gpio::Speed;
use crate::hal::prelude::*;
use crate::hal::xspi::{Config, Qspi, QspiMode, QspiWord};

use crate::pins::FMCPins;

// Commands from IS25LP064 datasheet
const WRITE_STATUS_REGISTRY_CMD: u8 = 0x01; // WRSR
const WRITE_CMD: u8 = 0x02; // PP
const READ_STATUS_REGISTRY_CMD: u8 = 0x05; // RDSR
const WRITE_ENABLE_CMD: u8 = 0x06; // WREN
const ENTER_QPI_MODE_CMD: u8 = 0x35; // QPIEN
const SET_READ_PARAMETERS_CMD: u8 = 0xC0; // SRP
const SECTOR_ERASE_CMD: u8 = 0xD7; // SER
const FAST_READ_QUAD_IO_CMD: u8 = 0xEB; // FRQIO

// Memory array specifications as defined in the datasheet
const SECTOR_SIZE: u32 = 4096;
const PAGE_SIZE: u32 = 256;
const MAX_ADDRESS: u32 = 0x7FFFFF;

/// Flash abstraction serves as a high level driver for the flash memory on the
/// board.
pub struct Flash {
    driver: Qspi<hal::pac::QUADSPI>,
}

impl Flash {
    /// Initialize the driver.
    ///
    /// # Example
    ///
    /// ```
    /// let board = daisy::Board::take().unwrap();
    /// let dp = daisy::pac::Peripherals::take().unwrap();
    ///
    /// let ccdr = board.freeze_clocks(dp.PWR.constrain(), dp.RCC.constrain(), &dp.SYSCFG);
    ///
    /// let pins = board.split_gpios(
    ///     dp.GPIOA.split(ccdr.peripheral.GPIOA),
    ///     dp.GPIOB.split(ccdr.peripheral.GPIOB),
    ///     dp.GPIOC.split(ccdr.peripheral.GPIOC),
    ///     dp.GPIOD.split(ccdr.peripheral.GPIOD),
    ///     dp.GPIOE.split(ccdr.peripheral.GPIOE),
    ///     dp.GPIOF.split(ccdr.peripheral.GPIOF),
    ///     dp.GPIOG.split(ccdr.peripheral.GPIOG),
    /// );
    ///
    /// let mut flash =
    ///     daisy::flash::Flash::new(&ccdr.clocks, dp.QUADSPI, ccdr.peripheral.QSPI, pins.FMC);
    /// ```
    ///
    /// See more under the `examples/` directory.
    pub fn new(
        clocks: &hal::rcc::CoreClocks,
        qspi_device: hal::device::QUADSPI,
        qspi_peripheral: hal::rcc::rec::Qspi,
        pins: FMCPins,
    ) -> Self {
        // Even though it is not directly used, CS pin must be acquired and configured
        let _cs = pins.CS.into_alternate_af10().set_speed(Speed::VeryHigh);

        let sck = pins.SCK.into_alternate_af9().set_speed(Speed::VeryHigh);
        let io0 = pins.IO0.into_alternate_af10().set_speed(Speed::VeryHigh);
        let io1 = pins.IO1.into_alternate_af10().set_speed(Speed::VeryHigh);
        let io2 = pins.IO2.into_alternate_af9().set_speed(Speed::VeryHigh);
        let io3 = pins.IO3.into_alternate_af9().set_speed(Speed::VeryHigh);

        let qspi = qspi_device.bank1(
            (sck, io0, io1, io2, io3),
            Config::new(133.mhz()).mode(QspiMode::OneBit),
            clocks,
            qspi_peripheral,
        );

        let mut flash = Self { driver: qspi };

        flash.reset_status_register();
        flash.reset_read_register();
        flash.enable_qpi_mode();

        flash
    }

    /// Blocking polling read.
    ///
    /// Will read as many consecutive bytes as needed to fill the buffer. When
    /// reaches end of the array, it would wrap around and continue reading from
    /// the beginning.
    ///
    /// # Panics
    ///
    /// Panics if the address is outside the range of the memory.
    pub fn read(&mut self, address: u32, buffer: &mut [u8]) {
        assert!(address <= MAX_ADDRESS);

        // Data must be queried by chunks of 32 (limitation of `read_extended`)
        for (i, chunk) in buffer.chunks_mut(32).enumerate() {
            self.driver
                .read_extended(
                    QspiWord::U8(FAST_READ_QUAD_IO_CMD),
                    QspiWord::U24(address + i as u32 * 32),
                    QspiWord::U8(0x00),
                    8,
                    chunk,
                )
                .unwrap();
        }
    }

    /// Blocking polling write.
    ///
    /// This method would write data of arbitrary length, starting on given
    /// address. In case end of the memory array is reached, it will wrap around
    /// and continue from the beginning.
    ///
    /// It is important to note that every memory sector that will be written to
    /// will get completelly erased, no matter where in it the address is.
    /// Sectors are aligned to 4 Kbytes.
    ///
    /// # Panics
    ///
    /// Panics if the address is outside the range of the memory.
    ///
    /// Panics if data is empty.
    pub fn write(&mut self, mut address: u32, data: &[u8]) {
        assert!(address <= MAX_ADDRESS);
        assert!(data.len() > 0);

        self.erase(address, data.len() as u32);

        let mut length = data.len() as u32;
        let mut start_cursor = 0;

        loop {
            // Calculate number of bytes between address and end of the page
            let page_remainder = PAGE_SIZE - (address & (PAGE_SIZE - 1));

            // Write data to the page in chunks of 32 (limitation of `write_extended`)
            let size = page_remainder.min(length) as usize;
            for (i, chunk) in data[start_cursor..start_cursor + size]
                .chunks(32)
                .enumerate()
            {
                self.enable_write();
                self.driver
                    .write_extended(
                        QspiWord::U8(WRITE_CMD),
                        QspiWord::U24(address + i as u32 * 32),
                        QspiWord::None,
                        chunk,
                    )
                    .unwrap();
                self.wait_for_write();
            }
            start_cursor += size;

            // Stop if this was the last needed page
            if length <= page_remainder {
                break;
            }
            length -= page_remainder;

            // Jump to the next page
            address += page_remainder;
            address %= MAX_ADDRESS;
        }
    }

    /// Erase `length` bytes at address `address` sector-by-sector and replace
    /// them with 0xFF.
    ///
    /// Note that even if the start or end of the block to be erased is in a
    /// middle of a sector, the whole sector will be removed.
    ///
    /// If the length overlaps the end of the memory array, the method will wrap
    /// to the beginning and continue erasing there.
    ///
    /// Learn more about the memory structure in the datasheet.
    ///
    /// # Panics
    ///
    /// Panics if the address is outside the range of the memory.
    ///
    /// Panics if length is zero.
    pub fn erase(&mut self, mut address: u32, mut length: u32) {
        assert!(address <= MAX_ADDRESS);
        assert!(length > 0);

        loop {
            // Erase the sector
            self.enable_write();
            self.driver
                .write_extended(
                    QspiWord::U8(SECTOR_ERASE_CMD),
                    QspiWord::U24(address),
                    QspiWord::None,
                    &[],
                )
                .unwrap();
            self.wait_for_write();

            // Calculate number of bytes between address and end of the sector
            let sector_remainder = SECTOR_SIZE - (address & (SECTOR_SIZE - 1));

            // Stop if this was the last affected sector
            if length <= sector_remainder {
                break;
            }
            length -= sector_remainder;

            // Jump to the next sector
            address += sector_remainder;
            address %= MAX_ADDRESS;
        }
    }

    /// Reset status registers into driver's defaults. This makes sure that the
    /// peripheral is configured as expected.
    fn reset_status_register(&mut self) {
        self.enable_write();

        self.driver
            .write_extended(
                QspiWord::U8(WRITE_STATUS_REGISTRY_CMD),
                QspiWord::U8(0b0000_0010),
                QspiWord::None,
                &[],
            )
            .unwrap();

        self.wait_for_write();
    }

    /// Reset read registers into driver's defaults. This makes sure that the
    /// peripheral is configured as expected.
    fn reset_read_register(&mut self) {
        self.enable_write();

        self.driver
            .write_extended(
                QspiWord::U8(SET_READ_PARAMETERS_CMD),
                QspiWord::U8(0b1111_1000),
                QspiWord::None,
                &[],
            )
            .unwrap();

        self.wait_for_write();
    }

    /// Enable quad mode to allow for faster reading.
    fn enable_qpi_mode(&mut self) {
        self.enable_write();

        self.driver
            .write_extended(
                QspiWord::U8(ENTER_QPI_MODE_CMD),
                QspiWord::None,
                QspiWord::None,
                &[],
            )
            .unwrap();

        self.driver.configure_mode(QspiMode::FourBit).unwrap();

        self.wait_for_write();
    }

    /// Allow write operation. This call is required before each and every
    /// command writting to the memory or registers as well as erasing memory.
    fn enable_write(&mut self) {
        self.driver
            .write_extended(
                QspiWord::U8(WRITE_ENABLE_CMD),
                QspiWord::None,
                QspiWord::None,
                &[],
            )
            .unwrap();
    }

    /// Wait for write operation to finish. This method is blocking and will
    /// poll WIP bit in status register. It can be used to monitor both write
    /// progress into memory as well as erasing and changes in registers.
    fn wait_for_write(&mut self) {
        loop {
            let mut status: [u8; 1] = [0xFF; 1];
            self.driver
                .read_extended(
                    QspiWord::U8(READ_STATUS_REGISTRY_CMD),
                    QspiWord::None,
                    QspiWord::None,
                    0,
                    &mut status,
                )
                .unwrap();

            if status[0] & 0x01 == 0 {
                break;
            }
        }
    }
}
