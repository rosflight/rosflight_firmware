/*
******************************************************************************
**
**  File        : LinkerScript.ld
**
**  Author      : STM32CubeIDE
**
**  Abstract    : Linker script for STM32H7 series
**                2048Kbytes FLASH and 1056Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** Copyright (c) 2023 STMicroelectronics.
** All rights reserved.
**
** This software is licensed under terms that can be found in the LICENSE file
** in the root directory of this software component.
** If no LICENSE file comes with this software, it is provided AS-IS.
**
****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1);    /* end of RAM */
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x10000;      /* required amount of heap note _user_heap_stack is one location */
_Min_Stack_Size = 0x20000; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
  FLASH (rx)     : ORIGIN = 0x08000000, LENGTH = 1024K /* Program Flash */
  FLASH2 (rw)    : ORIGIN = 0x08100000, LENGTH = 1024K /* Parameter Flash */
  DTCMRAM (rw)   : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_D1 (xrw)   : ORIGIN = 0x24000000, LENGTH = 496K /* Domain 1 AXI SRAM */
  SDDMARAM (rw)  : ORIGIN = 0x2407C000, LENGTH =  16K /* Domain 1 AXI SRAM for SDMMC DMA RAM*/
  SRAM1 (xrw)    : ORIGIN = 0x30000000, LENGTH = 128K /* Domain 2 SRAM1 - Not used */
  SRAM2 (xrw)    : ORIGIN = 0x30020000, LENGTH = 128K /* Domain 2 SRAM2 - Not used */
  SRAM3 (rw)     : ORIGIN = 0x30040000, LENGTH =  32K /* Domain 2 SRAM3 - Use for "my_dma_buffers" */
  SRAM4 (rw)     : ORIGIN = 0x38000000, LENGTH =  64K /*  Domain 3 SRAM4 - Use for "my_bdma_buffers" */
  ITCMRAM (xrw)  : ORIGIN = 0x00000000, LENGTH =  64K /* Allocated for "my_buffers" */
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab (READONLY)  : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM (READONLY) : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array (READONLY)   :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH

  .init_array (READONLY) :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH

  .fini_array (READONLY):
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM_D1 AT> FLASH

  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM_D1

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM_D1

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  dma_ram :
  {
    dma_ramstart = .;        /* create a global symbol at data start */
    . = ALIGN(32);
     
  } >SRAM3 AT > FLASH /* Don't know why, but J-Link needs the AT>FLASH*/
  
  bdma_ram :
  {
     bdma_ramstart = .;        /* create a global symbol at data start */
    . = ALIGN(32);
     
  } >SRAM4  AT > FLASH /* Don't know why, but J-Link needs the AT>FLASH*/

  dtcm_ram :
  {
     dtcm_ramstart = .;        /* create a global symbol at data start */
    . = ALIGN(32);
  } >DTCMRAM  AT > FLASH  /* Don't know why, but J-Link needs the AT>FLASH*/
  
  sddma_ram :
  {
     sddma_ramstart = .;        /* create a global symbol at data start */
  
    . = ALIGN(32);
 
  } >SDDMARAM AT > FLASH /* Don't know why, but J-Link needs the AT>FLASH*/

  .ARM.attributes 0 : { *(.ARM.attributes) }
}


