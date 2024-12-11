ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx) : ORIGIN =0x08000000, LENGTH =2048K
    SRAM(rwx) : ORIGIN =0x20000000, LENGTH =512K
}

SECTIONS
{
    .text : 
    {
        *(.isr_vector)
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
        KEEP(*(.init))
        KEEP(*(.fini))
        *(.eh_frame)
        *(.ARM.exidx)

        . = ALIGN(4);
        _etext = .;
    } > FLASH

    .data :
    {
        _sdata = .;
        *(.data)
        *(.data.*)
        KEEP(*(.init_array))
        KEEP(*(.fini_array))

        . = ALIGN(4);
        _edata = .;
    } > SRAM AT > FLASH

    .bss :
    {
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
         __bss_end__ = _ebss;
    } > SRAM
}