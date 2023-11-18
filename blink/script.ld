/* CH32V203C8T6 */

ENTRY(_start)

__stack_size = 2048;

PROVIDE(_stack_size = __stack_size);

MEMORY {
	FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 64K
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
}

SECTIONS {
  .init :	{
    _sinit = .;
    . = ALIGN(4);
    KEEP(*(SORT_NONE(.init)))
    . = ALIGN(4);
    _einit = .;
  } >FLASH AT>FLASH

  /* .vector : {
    *(.vector);
    . = ALIGN(64);
  } >FLASH AT>FLASH */

  .text :	{
    . = ALIGN(4);
    *(.text)
    *(.text.*)
    *(.rodata)
    *(.rodata*)
    *(.gnu.linkonce.t.*)
    . = ALIGN(4);
  } >FLASH AT>FLASH


}