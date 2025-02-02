/**
 * See: https://github.com/electro-smith/libDaisy/blob/master/core/STM32H750IB_flash.lds
 *      https://github.com/stm32-rs/stm32h7xx-hal/blob/master/memory.x
 *      https://github.com/mtthw-meyer/libdaisy-rust/blob/master/memory.x
 */

MEMORY
{
    FLASH     (RX)  : ORIGIN = 0x08000000, LENGTH = 128K
    DTCMRAM   (RWX) : ORIGIN = 0x20000000, LENGTH = 128K
    SRAM      (RWX) : ORIGIN = 0x24000000, LENGTH = 512K
    RAM_D2    (RWX) : ORIGIN = 0x30000000, LENGTH = 288K
    RAM_D3    (RWX) : ORIGIN = 0x38000000, LENGTH = 64K
    ITCMRAM   (RWX) : ORIGIN = 0x00000000, LENGTH = 64K
    SDRAM     (RWX) : ORIGIN = 0xc0000000, LENGTH = 64M
    QSPIFLASH (RX)  : ORIGIN = 0x90000000, LENGTH = 8M
}

/* stm32h7xx-hal uses a PROVIDE that expects RAM symbol to exist */
REGION_ALIAS(RAM, DTCMRAM);

SECTIONS
{
    .sram1_bss (NOLOAD) :
    {
        . = ALIGN(4);
        _ssram1_bss = .;

        PROVIDE(__sram1_bss_start__ = _sram1_bss);
        *(.sram1_bss)
        *(.sram1_bss*)
        . = ALIGN(4);
        _esram1_bss = .;

        PROVIDE(__sram1_bss_end__ = _esram1_bss);
    } > RAM_D2

    .sdram_bss (NOLOAD) :
    {
        . = ALIGN(4);
        _ssdram_bss = .;

        PROVIDE(__sdram_bss_start = _ssdram_bss);
        *(.sdram_bss)
        *(.sdram_bss*)
        . = ALIGN(4);
        _esdram_bss = .;

        PROVIDE(__sdram_bss_end = _esdram_bss);
    } > SDRAM

    .sram (NOLOAD) :
    {
        . = ALIGN(4);
        _ssram = .;

        PROVIDE(__sram_start__ = _sram);
        *(.sram)
        *(.sram*)
        . = ALIGN(4);
        _esram = .;

        PROVIDE(__sram_end__ = _esram);
    } > SRAM
}
