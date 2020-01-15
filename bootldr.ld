/*
 * Rival - A RISC-V RV32I CPU
 *
 * bootldr.ld: Linker control file for the bootloader
 *
 * Copyright (c) 2020, Helmut Sipos <helmut.sipos@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

ENTRY(_start)

/* Specify the memory areas */
MEMORY
{
	/* CPU-internal RAM */
	iRam (rwx) : ORIGIN = 0x00000000, LENGTH = 2K

	/* External RAM */
	eRam (rwx) : ORIGIN = 0x00000800, LENGTH = 1M
}

SECTIONS
{
	/* Executable section. Vectors first then boot loader code */
	.text : {
                . = ALIGN(4);
                startup.o (vectors);
                startup.o (.text);
		bootldr.o (.text);
	} > iRam

	/* const  data section */
	.rodata : {
                startup.o (.rodata);
		bootldr.o (.rodata);
	} > iRam

	/* Initialized data section */
	.data : {
                startup.o (.data);
		bootldr.o (.data);
	} > iRam

	/* Uninitialized data section */
	.bss : {
                startup.o (.bss);
		bootldr.o (.bss);
	} > iRam
}

