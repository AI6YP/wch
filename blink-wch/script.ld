/* CH32V203C8T6 */

ENTRY( _start )

__stack_size = 1024;

PROVIDE( _stack_size = __stack_size );

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

  .vector : {
    *(.vector);
    . = ALIGN(64);
  } >FLASH AT>FLASH

  .text : {
    . = ALIGN(4);
    *(.text)
    *(.text.*)
    *(.rodata)
    *(.rodata*)
    *(.gnu.linkonce.t.*)
    . = ALIGN(4);
  } >FLASH AT>FLASH

  .dalign : {
    . = ALIGN(4);
    PROVIDE(_data_vma = .);
  } >RAM AT>FLASH

  .dlalign : {
    . = ALIGN(4);
    PROVIDE(_data_lma = .);
  } >FLASH AT>FLASH

  .data : {
    /* *(.gnu.linkonce.r.*) */
    *(.data .data.*)
    /* *(.gnu.linkonce.d.*) */
    . = ALIGN(8);
    /* PROVIDE( __global_pointer$ = . + 0x800 ); */
    *(.sdata .sdata.*)
    *(.sdata2.*)
    *(.gnu.linkonce.s.*)
    . = ALIGN(8);
    *(.srodata.cst16)
    *(.srodata.cst8)
    *(.srodata.cst4)
    *(.srodata.cst2)
    *(.srodata .srodata.*)
    . = ALIGN(4);
    PROVIDE( _edata = .);
  } >RAM AT>FLASH

  .bss : {
    . = ALIGN(4);
    PROVIDE( _sbss = .);
    *(.sbss*)
    *(.gnu.linkonce.sb.*)
    *(.bss*)
    *(.gnu.linkonce.b.*)
    *(COMMON*)
    . = ALIGN(4);
    PROVIDE( _ebss = .);
  } >RAM AT>FLASH

  PROVIDE( _end = _ebss);
  PROVIDE( end = . );

  /* .stack ORIGIN(RAM) + LENGTH(RAM) - __stack_size : {
    PROVIDE( _heap_end = . );
    . = ALIGN(4);
    PROVIDE(_susrstack = . );
    . = . + __stack_size;
    PROVIDE( _eusrstack = .);
  } >RAM */
    PROVIDE( _eusrstack = ORIGIN(RAM) + LENGTH(RAM) - __stack_size);

}
