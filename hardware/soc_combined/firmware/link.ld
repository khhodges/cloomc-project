/*
 * hardware/soc_combined/firmware/link.ld
 *
 * Memory map from sapphire.v (address decoder):
 *   BRAM  0xF9000000  32 KB  r/w  (code + data, CPU resets to 0xF9000000)
 *   APB   0xF8000000  16 MB  r/w  (peripherals — UART at 0xF8010000)
 *
 * We split the single 32 KB BRAM into:
 *   ROM  0xF9000000  28 KB  .text + .rodata
 *   RAM  0xF9007000   4 KB  .data + .bss + stack
 *
 * The .data section is VMA 0xF9007000, LMA immediately after .rodata
 * (AT > ROM in the SECTIONS block), so crt0 copies it at boot.
 */

MEMORY
{
    ROM (rx)  : ORIGIN = 0xF9000000, LENGTH = 28K
    RAM (rwx) : ORIGIN = 0xF9007000, LENGTH = 4K
}

ENTRY(_start)

SECTIONS
{
    .text : {
        *(.text.startup)
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } > ROM

    .data : {
        _data_start = .;
        *(.data*)
        . = ALIGN(4);
        _data_end = .;
    } > RAM AT > ROM

    _data_load = LOADADDR(.data);

    .bss : {
        _bss_start = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end = .;
    } > RAM

    _stack_top = ORIGIN(RAM) + LENGTH(RAM);
}
