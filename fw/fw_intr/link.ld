OUTPUT_ARCH( "riscv" )
ENTRY(rvtest_entry_point)
SECTIONS
{
  . = 0x80000000;
  .boot_constants : { *(.boot_constants) }
  . = ALIGN(0x1000);
  .text.init : { *(.text.init) }
  . = ALIGN(0x1000);
  .tohost : { *(.tohost) }
  . = ALIGN(0x1000);
  .text : { *(.text) }
  . = ALIGN(0x1000);
  .data : { *(.data) }
  .data.string : { *(.data.string)}
  . = ALIGN(0x1000);
  __bss_start = .;
  .bss : { *(.bss) }
  __bss_end = .;
  _end = .;
}
