/*
 * Linker Script for ROM-to-RAM Initialization
 * ROM: [0x0000, 0x7FFF] - Code and initial data values
 * RAM: [0x8000, 0xFFFF] - Runtime data and stack
 */
OUTPUT_ARCH( "riscv" )
ENTRY( _start )

MEMORY
{
  ROM (rx)  : ORIGIN = 0x00000000, LENGTH = 32K
  RAM (rwx) : ORIGIN = 0x00008000, LENGTH = 32K
}

PROVIDE(_stack_top = ORIGIN(RAM) + LENGTH(RAM));

SECTIONS
{
  /* Code section in ROM */
  .text :
  {
    *(.text .text.*)
    _etext = .;  /* End of code - where data initialization values start */
  } > ROM

  /* Data section: VMA in RAM, LMA in ROM after code */
  .data : AT(_etext)
  {
    . = ALIGN(4);
    _sdata = .;           /* Start of data in RAM (VMA) */
    _ldata = LOADADDR(.data);  /* Location of init values in ROM (LMA) */
    *(.data .data.*)
    *(.sdata .sdata.*)
    . = ALIGN(4);
    _edata = .;           /* End of data in RAM */
  } > RAM

  /* BSS section in RAM (uninitialized, no ROM storage needed) */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss .bss.*)
    *(.sbss .sbss.*)
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  /* Symbols for initialization code */
  _data_size = _edata - _sdata;
}
