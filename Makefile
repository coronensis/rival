#
# Rival - A RISC-V RV32I CPU
#
# Makefile: MAKE(1) control file with targets for building the different parts
#                   of the SoC demonstrating the Rival CPU
#
# Copyright (c) 2020, Helmut Sipos <helmut.sipos@gmail.com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# Compiler etc. from the cross tool chain
CROSS_GCC     = riscv32-unknown-elf-gcc
CROSS_AS      = riscv32-unknown-elf-as
CROSS_LD      = riscv32-unknown-elf-ld
CROSS_NM      = riscv32-unknown-elf-gcc-nm
CROSS_OBJDUMP = riscv32-unknown-elf-objdump
CROSS_OBJCOPY = riscv32-unknown-elf-objcopy
CROSS_SIZE    = riscv32-unknown-elf-size

# Common flags
COMMON_CFLAGS = -std=gnu89 \
		-Werror -Wall -Wextra -pedantic \
		-Wshadow -Wstrict-prototypes -Wunreachable-code -Wundef \
		-Wdeclaration-after-statement

# Flags for the cross compiler
CROSS_CFLAGS = $(COMMON_CFLAGS) \
		-ffreestanding -nostdlib \
		-march=rv32i -mabi=ilp32 \
		-Os

# Flags for the native compiler
CFLAGS = $(COMMON_CFLAGS) \
	-O3 -g3

all: clean mkramimg bootldr.bin corecos.bit

bootldr.bin: startup.S bootldr.c
	$(CROSS_GCC) $(CROSS_CFLAGS) -c startup.S
	$(CROSS_GCC) $(CROSS_CFLAGS) -c bootldr.c
	$(CROSS_GCC) $(CROSS_CFLAGS) -Wl,-Map,bootldr.map,-N -T bootldr.ld -o bootldr.elf startup.o bootldr.o
	$(CROSS_OBJDUMP) -h -S -C bootldr.elf > bootldr.lst
	$(CROSS_NM) -n bootldr.elf > bootldr.sym
	$(CROSS_OBJCOPY) -S -O binary bootldr.elf bootldr.bin
	$(CROSS_SIZE) bootldr.elf
	./mkramimg bootldr.bin cpu.vhd

mkramimg: mkramimg.c
	gcc $(CFLAGS) -o mkramimg mkramimg.c

ad: ad.c
	gcc $(CFLAGS) -o ad ad.c

# Create the FPGA configuration bit stream by synthesizing the hardware description
corecos.bit:
	./synthesize

# Configure the FPGA with the generated bit stream
install:
	xc3sprog -c jtaghs1 -v -p 0 corecos.bit

# Remove build artefacts
clean:
	rm -rf _ngo rival.gise corecos.bgn corecos.bit corecos_bitgen.xwbt \
	       	corecos.bld corecos.cmd_log corecos.drc corecos_guide.ncd \
		corecos.lso corecos_map.map corecos_map.mrp corecos_map.ncd \
		corecos_map.ngm corecos_map.xrpt corecos.ncd corecos.ngc \
		corecos.ngd corecos_ngdbuild.xrpt corecos.ngr corecos.pad \
		corecos_pad.csv corecos_pad.txt corecos.par corecos_par.xrpt \
		corecos.pcf corecos.prj corecos.ptwx corecos.stx \
		corecos_summary.xml corecos.syr corecos.twr corecos.twx \
	       	corecos.unroutes corecos_usage.xml corecos.ut corecos.xpi \
		corecos.xst corecos_xst.xrpt rival.xise usage_statistics_webtalk.html \
		corecos_vhdl.prj webtalk.log webtalk_pn.xml xlnx_auto_0_xdb \
		_xmsgs xst .Xil cpu_tb_beh.prj cpu_tb_isim_beh.exe \
		cpu_tb_isim_beh.wdb cpu_tb_stx_beh.prj fuse.log fuse.xmsgs \
		fuseRelaunch.cmd isim.cmd isim.log corecos_envsettings.html \
		xilinxsim.ini isim iseconfig corecos_summary.html \
		*.o *.elf *.bin *.lst *.sym *.map bootldr.txt mkramimg ad

