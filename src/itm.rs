/// Adapted from: https://gist.github.com/diondokter/5740aeb145e123c5b4dac7c7b32e36f6
///
/// Register addresses:
///
///   SWO    base: 0x5C00_3000
///   SWTF   base: 0x5C00_4000  (SWO funnnel)
///   DBGMCU base: 0x5C00_1000
///   ITM    base: 0xE000_0000  (Instrumentation Trace Macrocell)
///   DCB    base: 0xE000_EDF0  (Debug Control Block)

pub unsafe fn enable_itm(
    dcb: &mut cortex_m::peripheral::DCB,
    dbgmcu: &stm32h7xx_hal::stm32::DBGMCU,
    itm: &mut cortex_m::peripheral::ITM,
    c_ck: u32,
    swo_frequency: u32,
) {
    // DCB_DEMCR: Set TRCENA. Enables DWT and ITM units  (DEMCR = Debug Exception and Monitor Control)
    //*(0xE000_EDFC as *mut u32) |= 1 << 24;
    dcb.enable_trace();

    // DBGMCU_CR: Ensure debug blocks are clocked before interacting with them  (page 3350)
    // *(0x5C00_1004 as *mut u32)
    dbgmcu.cr.modify(|_, w| {
        w.d3dbgcken()
            .set_bit() // D3 domain debug clock enable
            .d1dbgcken()
            .set_bit() // D1 domain debug clock enable
            .traceclken()
            .set_bit() // Enable trace port clock, TRACECLK
            .dbgsleep_d1()
            .set_bit() // Automatic clock stop/power-down disabled
    });

    // SWO_LAR: Unlock SWO (LAR = Lock Access Register)
    *(0x5c00_3fb0 as *mut u32) = 0xC5AC_CE55;

    // SWTF_LAR: Unlock SWTF
    *(0x5c00_4fb0 as *mut u32) = 0xC5AC_CE55;

    // SWO_CODR Register: Set SWO speed
    *(0x5c00_3010 as *mut _) = (c_ck / swo_frequency) - 1;

    // SWO_SPPR Register:
    // 1 = Manchester, 2 = NRZ
    *(0x5c00_30f0 as *mut _) = 0x2;

    // SWTF_CTRL Trace Funnel: Enable for CM7
    *(0x5c00_4000 as *mut u32) |= (0 << 1)               | // ENS1 - Slave port S1 enabled (Cortex-M4)
        (1 << 0); // ENS0 - Slave port S0 enabled (Cortex-M7)

    // ITM_LAR: Unlock ITM
    // *(0xE000_0FB0 as *mut u32) = 0xC5AC_CE55;
    itm.lar.write(0xC5AC_CE55);

    // ITM_TER: Enable lower 8 stimulus ports (TER = Trace Enable Register)
    // *(0xE000_0E00 as *mut u32)
    itm.ter[0].write(1);

    // ITM_TCR:  Enable ITM (TCR = Trace Control Register)
    // *(0xE000_0E80 as *mut u32)
    itm.tcr.write(
        (0b00_0001 << 16) | // TraceBusID
        (1 << 3)          | // TXENA  - hardware event packet forwarding enable (enable SWO output ?)
        (1 << 0), // ITMENA - enable the ITM
    );
}
