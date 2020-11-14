use stm32h7xx_hal as hal;
use hal::gpio;
use hal::time;
use hal::dma;
use hal::sai::{ self, SaiI2sExt, SaiChannel };

use hal::hal as embedded_hal;
use embedded_hal::digital::v2::OutputPin;

use hal::pac;
use pac::interrupt;

use core::convert::TryInto;
use alloc::prelude::v1::Box;


// = global constants =========================================================

pub const BLOCK_LENGTH: usize = 32;                             // 32 samples
pub const HALF_DMA_BUFFER_LENGTH: usize = BLOCK_LENGTH * 2;     //  2 channels
pub const DMA_BUFFER_LENGTH:usize = HALF_DMA_BUFFER_LENGTH * 2; //  2 half-blocks

pub const FS: time::Hertz = time::Hertz(48_000);


// = types ====================================================================

pub type Frame = (f32, f32);
pub type Block = [Frame; BLOCK_LENGTH];

type Sai1Pins = (
    gpio::gpiob::PB11<gpio::Output<gpio::PushPull>>,  // PDN
    gpio::gpioe::PE2<gpio::Alternate<gpio::AF6>>,     // MCLK_A
    gpio::gpioe::PE5<gpio::Alternate<gpio::AF6>>,     // SCK_A
    gpio::gpioe::PE4<gpio::Alternate<gpio::AF6>>,     // FS_A
    gpio::gpioe::PE6<gpio::Alternate<gpio::AF6>>,     // SD_A
    gpio::gpioe::PE3<gpio::Alternate<gpio::AF6>>,     // SD_B
);

type TransferDma1Str0 = dma::Transfer<dma::dma::Stream0<pac::DMA1>,
                                      pac::SAI1,
                                      dma::MemoryToPeripheral,
                                      &'static mut [u32; DMA_BUFFER_LENGTH]>;

type TransferDma1Str1 = dma::Transfer<dma::dma::Stream1<pac::DMA1>,
                                      pac::SAI1,
                                      dma::PeripheralToMemory,
                                      &'static mut [u32; DMA_BUFFER_LENGTH]>;

#[repr(C)]
pub struct OpaqueInterface { _private: [u8; 0] }


// = static data ==============================================================

#[link_section = ".sram1_bss"]
static mut TX_BUFFER: [u32; DMA_BUFFER_LENGTH] = [0; DMA_BUFFER_LENGTH];
#[link_section = ".sram1_bss"]
static mut RX_BUFFER: [u32; DMA_BUFFER_LENGTH] = [0; DMA_BUFFER_LENGTH];


// = audio::Interface =========================================================

type Error = u32;

pub struct Interface<'a> {
    pub fs: time::Hertz,
    closure: Option<Box<dyn FnMut(f32, &mut Block) + 'a>>,
    ak4556_reset: Option<gpio::gpiob::PB11<gpio::Output<gpio::PushPull>>>,
    hal_dma1_stream0: Option<TransferDma1Str0>,
    hal_dma1_stream1: Option<TransferDma1Str1>,
    hal_sai1: Option<hal::sai::Sai<pac::SAI1, hal::sai::I2S>>,
    _marker: core::marker::PhantomData<&'a *const ()>,
}


impl<'a> Interface<'a>
{
    pub fn new() -> Interface<'a> {
        Self {
            fs: FS,
            closure: None,
            ak4556_reset: None,
            hal_dma1_stream0: None,
            hal_dma1_stream1: None,
            hal_sai1: None,
            _marker: core::marker::PhantomData,
        }
    }

    pub fn init(
        clocks: &hal::rcc::CoreClocks,
        rec: hal::rcc::rec::Sai1, // reset and enable control
        pins: Sai1Pins,
        dma1_hal: hal::rcc::rec::Dma1
    ) -> Result<Interface<'a>, Error> {

        // - configure dma1 ---------------------------------------------------

        let dma1_streams = dma::dma::StreamsTuple::new(unsafe { pac::Peripherals::steal().DMA1 }, dma1_hal);

        // dma1 stream 0
        let tx_buffer: &'static mut [u32; DMA_BUFFER_LENGTH] = unsafe { &mut TX_BUFFER };
        let dma_config = dma::dma::DmaConfig::default()
            .priority(dma::config::Priority::High)
            .memory_increment(true)
            .peripheral_increment(false)
            .circular_buffer(true)
            .fifo_enable(false);
        let dma1_str0: dma::Transfer<_, _, dma::MemoryToPeripheral, _> = dma::Transfer::init(
            dma1_streams.0,
            unsafe { pac::Peripherals::steal().SAI1 },
            tx_buffer,
            None,
            dma_config,
        );

        // dma1 stream 1
        let rx_buffer: &'static mut [u32; DMA_BUFFER_LENGTH] = unsafe { &mut RX_BUFFER };
        let dma_config = dma_config.transfer_complete_interrupt(true)
                                   .half_transfer_interrupt(true);
        let dma1_str1: dma::Transfer<_, _, dma::PeripheralToMemory, _> = dma::Transfer::init(
            dma1_streams.1,
            unsafe { pac::Peripherals::steal().SAI1 },
            rx_buffer,
            None,
            dma_config,
        );

        // - configure sai1 ---------------------------------------------------

        let sai1_tx_config = sai::I2SChanConfig::new(sai::I2SDir::Tx)
            .set_frame_sync_active_high(true)
            .set_clock_strobe(sai::I2SClockStrobe::Falling);

        let sai1_rx_config = sai::I2SChanConfig::new(sai::I2SDir::Rx)
            .set_sync_type(sai::I2SSync::Internal)
            .set_frame_sync_active_high(true)
            .set_clock_strobe(sai::I2SClockStrobe::Rising);

        let sai1_pins = (
            pins.1,
            pins.2,
            pins.3,
            pins.4,
            Some(pins.5),
        );

        let sai1 = unsafe { pac::Peripherals::steal().SAI1 }.i2s_ch_a(
            sai1_pins,
            FS,
            sai::I2SDataSize::BITS_24,
            rec,
            clocks,
            sai1_tx_config,
            Some(sai1_rx_config),
        );

        Ok(Self {
            fs: FS,
            closure: None,
            ak4556_reset: Some(pins.0),
            hal_dma1_stream0: Some(dma1_str0),
            hal_dma1_stream1: Some(dma1_str1),
            hal_sai1: Some(sai1),
            _marker: core::marker::PhantomData,
        })
    }


    pub fn init_pac(
        &self,
        clocks: &hal::rcc::CoreClocks,
        rec: hal::rcc::rec::Sai1, // reset and enable control
        pins: Sai1Pins
    ) -> Result<(), Error> {

        // - Peripherals ------------------------------------------------------

        let rcc_pac = unsafe { pac::Peripherals::steal().RCC };
        let dma1_pac = unsafe { pac::Peripherals::steal().DMA1 };
        let dmamux1_pac = unsafe { pac::Peripherals::steal().DMAMUX1 };
        let sai1_pac = unsafe { pac::Peripherals::steal().SAI1 };

        // - DMA1 -------------------------------------------------------------

        // enable DMA1 peripheral clock
        rcc_pac.ahb1enr.modify(|_, w| w.dma1en().enabled());       // enable clock for dma peripheral
        unsafe { pac::NVIC::unmask(pac::Interrupt::DMA1_STR1); }   // unmask interrupt handler for dma 1, stream 1

        // disable DMA1 streams for configuration
        dma1_pac.st[0].cr.write(|w| w.en().disabled());
        while dma1_pac.st[0].cr.read().en().is_enabled() {}
        dma1_pac.st[1].cr.write(|w| w.en().disabled());
        while dma1_pac.st[1].cr.read().en().is_enabled() {}

        // clear DMA1 status registers
        unsafe { dma1_pac.lifcr.write(|w| w.bits(0)); }

        // configure DMA1 stream 0
        let ptr = unsafe { TX_BUFFER.as_ptr() as usize as u32 };
        let length = DMA_BUFFER_LENGTH as u16;
        let sai1_adr_addr = &sai1_pac.cha.dr as *const _ as u32;
        dma1_pac.st[0].par.write(|w| unsafe { w.pa().bits(sai1_adr_addr) }); // set peripheral address to SAI A
        dma1_pac.st[0].m0ar.write(|w| unsafe { w.m0a().bits(ptr) });         // transmit buffer address
        dma1_pac.st[0].ndtr.write(|w| w.ndt().bits(length));                 // transmit buffer length

        dma1_pac.st[0].cr.write(|w| {
            w.dir().memory_to_peripheral() // DMA_MEMORY_TO_PERIPH
             .pinc().fixed()               // DMA_PINC_DISABLE
             .minc().incremented()         // DMA_MINC_ENABLE
             .psize().bits32()             // DMA_PDATAALIGN_WORD
             .msize().bits32()             // DMA_MDATAALIGN_WORD
             .circ().enabled()             // DMA_CIRCULAR
             .pl().high()                  // DMA_PRIORITY_HIGH
        });

        // configure DMA1 stream 0 fifo
        dma1_pac.st[0].fcr.write(|w| {
            w.dmdis().enabled()            // DMA_FIFOMODE_DISABLE
        });

        // configure DMA1 stream 1
        let ptr = unsafe { RX_BUFFER.as_ptr() as usize as u32 };
        let length = DMA_BUFFER_LENGTH as u16;
        let sai1_bdr_addr = &sai1_pac.chb.dr as *const _ as u32;
        dma1_pac.st[1].par.write(|w| unsafe { w.pa().bits(sai1_bdr_addr) }); // set peripheral address to SAI B
        dma1_pac.st[1].m0ar.write(|w| unsafe { w.m0a().bits(ptr) });         // receive buffer address
        dma1_pac.st[1].ndtr.write(|w| w.ndt().bits(length));                 // receive buffer length

        dma1_pac.st[1].cr.write(|w| {
            w.dir().peripheral_to_memory() // DMA_PERIPH_TO_MEMORY
             .pinc().fixed()               // DMA_PINC_DISABLE
             .minc().incremented()         // DMA_MINC_ENABLE
             .psize().bits32()             // DMA_PDATAALIGN_WORD
             .msize().bits32()             // DMA_MDATAALIGN_WORD
             .circ().enabled()             // DMA_CIRCULAR
             .pl().high()                  // DMA_PRIORITY_HIGH
        });

        // configure DMA1 stream 1 fifo
        dma1_pac.st[1].fcr.write(|w| {
            w.dmdis().enabled()            // DMA_FIFOMODE_DISABLE
        });

        // clear DMA1 interrupt flags
        dma1_pac.lifcr.write(|w| w);

        // set DMAMUX1 stream 0 peripheral request to SAI1 block A
        dmamux1_pac.ccr[0].modify(|_, w| { w.dmareq_id().sai1a_dma() });

        // set DMAMUX1 stream 1 peripheral request to SAI1 block B
        dmamux1_pac.ccr[1].modify(|_, w| { w.dmareq_id().sai1b_dma() });

        // clear the DMAMUX1 synchro overrun flag
        dmamux1_pac.cfr.write(|w| {
            w.csof0().clear_bit()
        });

        // - SAI1 pac ---------------------------------------------------------

        let nbslot: u8 = 2;
        let frame_length: u8 = 32 * nbslot;                  // 24-bit audio ships in a 32-bit word
        let active_frame_length = frame_length / nbslot;

        rcc_pac.apb2enr.modify(|_, w| w.sai1en().enabled()); // enable clock for sai peripheral


        // - SAI A ------------------------------------------------------------

        // disable SAI1 block A for configuration
        sai1_pac.cha.cr1.modify(|_, w| w.saien().disabled());
        while sai1_pac.cha.cr1.read().saien().is_enabled() {}

        // disable SAI1 block A interrupts & clear flags
        sai1_pac.cha.im.write(|w| w);
        sai1_pac.cha.clrfr.write(|w| {
            w.clfsdet().clear()  // late frame synchronization detection
             .cafsdet().clear()  // anticipated frame synchronization detection
             .ccnrdy().clear()   // codec not ready
             .cwckcfg().clear()  // wrong clock configuration
             .cmutedet().clear() // mute detection
             .covrudr().clear()  // overrun / underrun
        });

        // flush SAI A fifo
        sai1_pac.cha.cr2.modify(|_, w| w.fflush().flush());

        // reset sai A
        sai1_pac.cha.cr1.write(|w| w);

        // calculate mck_div
        let nodiv = false;
        let master_oversampling = false;
        fn sai_a_ker_ck(rec: &hal::rcc::rec::Sai1, clocks: &hal::rcc::CoreClocks) -> Option<time::Hertz> {
            use hal::rcc::rec::Sai1ClkSel;  // aka pac::rcc::d2ccip1r::SAI1SEL_A;
            use hal::Variant::Val;
            match rec.get_kernel_clk_mux() {
                Val(Sai1ClkSel::PLL1_Q) => clocks.pll1_q_ck(),
                Val(Sai1ClkSel::PLL2_P) => clocks.pll2_p_ck(),
                Val(Sai1ClkSel::PLL3_P) => clocks.pll3_p_ck(),
                Val(Sai1ClkSel::I2S_CKIN) => unimplemented!(),
                Val(Sai1ClkSel::PER) => clocks.per_ck(),
                _ => unreachable!(),
            }
        }
        let sai_ck_a = sai_a_ker_ck(&rec, clocks)
            .expect("SAI kernel clock is not configured. Have you set it with: \
                     ccdr.peripheral.SAI1.kernel_clk_mux(hal::rcc::rec::Sai1ClkSel::PLL3_P) ?");
        let clock_ratio = if master_oversampling { 512 } else { 256 };
        let mck_div = if nodiv {
            (sai_ck_a.0 * 10) / (self.fs.0 * frame_length as u32)
        } else {
            (sai_ck_a.0 * 10) / (self.fs.0 * clock_ratio as u32)
        };
        let mck_div = mck_div / 10;
        let mck_div: u8 = mck_div.try_into()
            .expect("SAI kernel clock is out of range for required MCLK");

        // configure SAI A cr1
        sai1_pac.cha.cr1.write(|w| unsafe {
            w.mode().master_tx()                    // SAI_MODEMASTER_TX
             .syncen().asynchronous()               // SAI_ASYNCHRONOUS
             .outdriv().on_start()                  // SAI_OUTPUTDRIVE_DISABLE
             .nodiv().master_clock()                // SAI_MASTERDIVIDER_ENABLED
             .mono().stereo()                       // SAI_STEREOMODE
             .mckdiv().bits(mck_div)                // master clock divider

             // protocol
             .prtcfg().free()                       // SAI_FREE_PROTOCOL
             .lsbfirst().msb_first()                // SAI_FIRSTBIT_MSB
             .ckstr().falling_edge()                // SAI_CLOCKSTROBING_FALLINGEDGE
             .ds().bit24()                          // SAI_DATASIZE_24
        });

        // configure SAI A cr2
        sai1_pac.cha.cr2.write(|w| {
            w.fth().empty()                         // SAI_FIFOTHRESHOLD_EMPTY
             .comp().no_companding()                // SAI_NOCOMPANDING
             .tris().clear_bit()
        });

        // configure SAI A frame
        sai1_pac.cha.frcr.write(|w| unsafe {
            w.fsdef().set_bit()                     // SAI_FS_CHANNEL_IDENTIFICATION
             .fspol().rising_edge()                 // SAI_FS_ACTIVE_HIGH
             .fsoff().on_first()                    // SAI_FS_FIRSTBIT
             .frl().bits(frame_length - 1)          // FrameLength
             .fsall().bits(active_frame_length - 1) // ActiveFrameLength
        });

        // configure SAI A slot
        sai1_pac.cha.slotr.write(|w| unsafe {
            w.sloten().bits(0b0000_0011)            // set active slots
             .fboff().bits(0)                       // first_bit_offset
             .nbslot().bits(nbslot - 1)             // number_of_slots
             .slotsz().bit32()                      // SAI_SLOTSIZE_32B
        });

        // - SAI B ------------------------------------------------------------

        // disable sai block b for configuration
        sai1_pac.chb.cr1.modify(|_, w| w.saien().disabled());
        while sai1_pac.chb.cr1.read().saien().is_enabled() {}

        // flush SAI B fifo
        sai1_pac.chb.cr2.modify(|_, w| w.fflush().flush());

        // reset sai B
        sai1_pac.chb.cr1.write(|w| w);

        // configure SAI B cr1
        sai1_pac.chb.cr1.write(|w| unsafe {
            w.mode().slave_rx()                        // SAI_SLAVE_RX
             .syncen().internal()                      // SAI_SYNCHRONOUS
             .outdriv().on_start()                     // SAI_OUTPUTDRIVE_DISABLE
             .nodiv().master_clock()                   // SAI_MASTERDIVIDER_ENABLED
             .mono().stereo()                          // SAI_STEREOMODE
             .mckdiv().bits(0)                         // master clock divider

                                                       // protocol
             .prtcfg().free()                          // SAI_FREE_PROTOCOL
             .lsbfirst().msb_first()                   // SAI_FIRSTBIT_MSB
             .ckstr().rising_edge()                    // SAI_CLOCKSTROBINGRISINGEDGE
             .ds().bit24()                             // SAI_DATASIZE_24
        });

        // configure SAI B cr2
        sai1_pac.chb.cr2.write(|w| {
            w.fth().empty()                            // SAI_FIFOTHRESHOLD_EMPTY
             .comp().no_companding()                   // SAI_NOCOMPANDING
             .tris().clear_bit()
        });

        // configure SAI B frame
        sai1_pac.chb.frcr.write(|w| unsafe {
            w.fsdef().set_bit()                        // SAI_FS_CHANNEL_IDENTIFICATION
                .fspol().rising_edge()                 // SAI_FS_ACTIVE_HIGH
                .fsoff().on_first()                    // SAI_FS_FIRSTBIT
                .frl().bits(frame_length - 1)          // FrameLength
                .fsall().bits(active_frame_length - 1) // ActiveFrameLength
        });

        // configure SAI B slot
        sai1_pac.chb.slotr.write(|w| unsafe {
            w.sloten().bits(0b0000_0011)               // set active slots
             .fboff().bits(0)                          // first_bit_offset
             .nbslot().bits(nbslot - 1)                // number_of_slots
             .slotsz().bit32()                         // SAI_SLOTSIZE_32B
        });

        // disable SAI B interrupts & clear flags
        sai1_pac.chb.im.write(|w| w);
        sai1_pac.chb.clrfr.write(|w| {
            w.clfsdet().clear()                        // late frame synchronization detection
             .cafsdet().clear()                        // anticipated frame synchronization detection
             .ccnrdy().clear()                         // codec not ready
             .cwckcfg().clear()                        // wrong clock configuration
             .cmutedet().clear()                       // mute detection
             .covrudr().clear()                        // overrun / underrun
        });

        // - SAI SYNC -------------------------------------------------------------
                                                       // disable sync outputs
        sai1_pac.gcr.modify(|_, w| unsafe {
            w.syncout().bits(0)
             .syncin().bits(0)                         // SAI_SYNCEXT_DISABLE
        });

        // - AK4556 -----------------------------------------------------------

        let mut ak4556_reset = pins.0;
        ak4556_reset.set_low().unwrap();
        use cortex_m::asm;
        asm::delay(480_000);     // ~ 1ms (datasheet specifies minimum 150ns)
        ak4556_reset.set_high().unwrap();

        Ok(())
    }


    pub fn start<F: FnMut(f32, &mut Block) + 'a>(&mut self, closure: F) -> Result<&mut Self, Error> {
        self.closure = Some(Box::new(closure));

        // TODO implement drop for Interface so we can set INTERFACE_PTR to None
        let opaque_interface_ptr: *const OpaqueInterface = unsafe {
            core::mem::transmute::<*const Interface,
                                   *const OpaqueInterface>(self)
        };
        unsafe { INTERFACE_PTR = Some(opaque_interface_ptr); }

        //self.start_pac()?;
        self.start_hal()?;

        Ok(self) // TODO TypeState for a started interface
    }

    pub fn start_hal(&mut self) -> Result<(), Error> {

        // - AK4556 -----------------------------------------------------------

        let ak4556_reset = self.ak4556_reset.as_mut().unwrap();
        ak4556_reset.set_low().unwrap();
        use cortex_m::asm;
        asm::delay(480_000);     // ~ 1ms (datasheet specifies minimum 150ns)
        ak4556_reset.set_high().unwrap();


        // - start audio ------------------------------------------------------

        // unmask interrupt handler for dma 1, stream 1
        unsafe { pac::NVIC::unmask(pac::Interrupt::DMA1_STR1); }

        let dma1_str0 = self.hal_dma1_stream0.as_mut().unwrap();
        let dma1_str1 = self.hal_dma1_stream1.as_mut().unwrap();
        let sai1 = self.hal_sai1.as_mut().unwrap();

        dma1_str1.start(|_sai1_rb| {
            sai1.enable_dma(SaiChannel::ChannelB);
        });

        dma1_str0.start(|sai1_rb| {
            sai1.enable_dma(SaiChannel::ChannelA);

            // wait until sai1's fifo starts to receive data
            //hprintln!("sai fifo waiting to receive data").unwrap();
            while sai1_rb.cha.sr.read().flvl().is_empty() { }
            //hprintln!("sai fifo receiving data").unwrap();

            sai1.enable();
        });

        Ok(())
    }

    pub fn start_pac(&mut self) -> Result<(), Error> {
        let dma1_pac = unsafe { pac::Peripherals::steal().DMA1 };
        let sai1_pac = unsafe { pac::Peripherals::steal().SAI1 };

        // enable DMA1 interrupts
        dma1_pac.st[1].cr.modify(|_, w| {
            w.tcie().enabled()         // tx complete irq
             .htie().enabled()         // half tx complete irq
             .teie().enabled()         // tx error irq
             .dmeie().enabled()        // direct mode error
        });

        // enable DMA1 streams
        dma1_pac.st[0].cr.modify(|_, w| w.en().enabled());
        while dma1_pac.st[0].cr.read().en().is_disabled() {}
        dma1_pac.st[1].cr.modify(|_, w| w.en().enabled());
        while dma1_pac.st[1].cr.read().en().is_disabled() {}

        // enable SAI1 dma requests
        sai1_pac.cha.cr1.modify(|_, w| w.dmaen().enabled());
        while sai1_pac.cha.cr1.read().dmaen().is_disabled() {}
        sai1_pac.chb.cr1.modify(|_, w| w.dmaen().enabled());
        while sai1_pac.chb.cr1.read().dmaen().is_disabled() {}

        // wait till SAI1 fifo is not empty - TODO timeout
        //hprintln!("sai fifo waiting to receive data").unwrap();
        while sai1_pac.cha.sr.read().flvl().is_empty() { }
        //hprintln!("sai fifo receiving data").unwrap();

        // flush SAI1 blocks
        sai1_pac.cha.cr2.modify(|_, w| w.fflush().flush());
        sai1_pac.chb.cr2.modify(|_, w| w.fflush().flush());

        // enable SAI1 blocks
        sai1_pac.cha.cr1.modify(|_, w| w.saien().enabled());
        while sai1_pac.cha.cr1.read().saien().is_disabled() {}
        sai1_pac.chb.cr1.modify(|_, w| w.saien().enabled());
        while sai1_pac.chb.cr1.read().saien().is_disabled() {}

        Ok(())
    }
}


// = dma rx interrupt handler =================================================

static mut INTERFACE_PTR: Option<*const OpaqueInterface> = None;

#[inline(always)]
fn dma1_str1_hal() {
    if unsafe { INTERFACE_PTR.is_none() } {
        return;
    }

    // reconstitute mutable reference to Interface
    let interface_ptr: *const OpaqueInterface = unsafe { INTERFACE_PTR }.unwrap();
    let interface_ptr: *mut Interface = unsafe {
        core::mem::transmute::<*const OpaqueInterface,
                               *mut Interface>(interface_ptr)
    };
    let interface: &mut Interface = unsafe { &mut *interface_ptr };

    let transfer = interface.hal_dma1_stream1.as_mut().unwrap();

    let skip = if transfer.get_half_transfer_flag() {
        transfer.clear_half_transfer_interrupt();
        (0, HALF_DMA_BUFFER_LENGTH)

    } else if transfer.get_transfer_complete_flag() {
        transfer.clear_transfer_complete_interrupt();
        (HALF_DMA_BUFFER_LENGTH, 0)

    } else {
        // TODO handle error flags once HAL supports them
        return;
    };

    dma_common(skip);
}


#[inline(always)]
fn _dma1_str1_pac() {
    let dma1 = unsafe { pac::Peripherals::steal().DMA1 };

    let lisr = dma1.lisr.read();

    let skip = if lisr.htif1().is_half() {
        dma1.lifcr.write(|w| w.chtif1().clear());
        (0, HALF_DMA_BUFFER_LENGTH)

    } else if lisr.tcif1().is_complete() {
        dma1.lifcr.write(|w| w.ctcif1().clear());
        (HALF_DMA_BUFFER_LENGTH, 0)

    } else if lisr.teif1().is_error() {
        dma1.lifcr.write(|w| w.cteif1().clear());
        //hprintln!("dma1_stream1::irq error: teif1").unwrap();
        return;
    } else if lisr.dmeif1().is_error() {
        dma1.lifcr.write(|w| w.cdmeif1().clear());
        //hprintln!("dma1_stream1::irq error: cdmeif1").unwrap();
        return;
    } else if lisr.feif1().is_error() {
        dma1.lifcr.write(|w| w.cfeif1().clear());
        //hprintln!("dma1_stream1::irq error: feif1").unwrap();
        return;
    } else {
        //hprintln!("dma1_stream1::irq error: unknown - {}", lisr.bits()).unwrap();
        return;
    };

    dma_common(skip);
}


#[inline(always)]
fn dma_common(skip: (usize, usize)) {
    // convert audio data from u32 to f32
    use core::num::Wrapping;
    #[inline(always)]
    fn u32_to_f32(y: u32) -> f32 {
        let y = (Wrapping(y) + Wrapping(0x0080_0000)).0 & 0x00FF_FFFF; // convert to i32
        let y = (y as f32 / 8_388_608.) - 1.;  // (2^24) / 2
        y
    }

    // convert audio data from f32 to u24
    #[inline(always)]
    pub fn f32_to_u24(x: f32) -> u32 {
        //return (int16_t) __SSAT((int32_t) (x * 32767.f), 16);
        let x = x * 8_388_607.;
        let x = if x > 8_388_607. {
            8_388_607.
        } else if x < -8_388_608. {
            -8_388_608.
        } else {
            x
        };
        (x as i32) as u32
    }

    // callback buffer
    static mut BLOCK: Block = [(0., 0.); BLOCK_LENGTH];

    // convert & copy rx buffer to callback buffer
    let mut dma_index: usize = 0;
    let mut block_index: usize = 0;
    while dma_index < HALF_DMA_BUFFER_LENGTH {
        let rx0: usize = dma_index + skip.1;
        let rx1: usize = rx0 + 1;

        let rx_y0 = unsafe { RX_BUFFER[rx0] };
        let rx_y1 = unsafe { RX_BUFFER[rx1] };

        let y0 = u32_to_f32(rx_y0);
        let y1 = u32_to_f32(rx_y1);
        unsafe { BLOCK[block_index] = (y1, y0); }

        dma_index += 2;
        block_index += 1;
    }

    // invoke closure
    if let Some(interface_ptr) = unsafe { INTERFACE_PTR }  {
        let interface_ptr: *mut Interface = unsafe {
            core::mem::transmute::<*const OpaqueInterface,
                                   *mut Interface>(interface_ptr)
        };
        if let Some(closure) = unsafe { &mut (*interface_ptr).closure } {
            let fs = unsafe { (*interface_ptr).fs };
            closure(fs.0 as f32, unsafe { &mut BLOCK });
        }
    }

    // convert & copy callback buffer to tx buffer
    let mut dma_index: usize = 0;
    let mut block_index: usize = 0;
    while dma_index < HALF_DMA_BUFFER_LENGTH {
        let tx0: usize = dma_index + skip.0;
        let tx1: usize = tx0 + 1;

        let (y0, y1) = unsafe { BLOCK[block_index] };
        unsafe { TX_BUFFER[tx1] = f32_to_u24(y0) };
        unsafe { TX_BUFFER[tx0] = f32_to_u24(y1) };

        dma_index += 2;
        block_index += 1;
    }
}


#[interrupt]
fn DMA1_STR1() {
    //dma1_str1_pac();
    dma1_str1_hal();
}
