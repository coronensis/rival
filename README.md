# Rival - A RISC-V RV32I Softcore CPU
  
## Overview

A simple RISC-V RV32I implementation based on my previous journey
into the CPU implementation world: [Dwarf](https://github.com/coronensis/dwarf).
Focused on concepts and simplicity rater than on completeness or
(serious i.e. industrial) real-world useability.

Contrary to what the name suggests, it rivals nothing.
I just needed a name for the project that contained "Ri"(sc) and "V" :)
After considering "River" and "Raven" (both are already taken) I settled
with "Rival" which seem to be stil unoccupied in this domain (at least
according to Google).


## Motivation

+ Pastime
+ Get in touch with RISC-V
+ The special kind of entertainment that comes from developing
 and bringing up a system without proper debugging tools
+ Next step after [Dwarf](https://github.com/coronensis/dwarf)
+ "Material" needed for [Corecos](https://github.com/coronensis/corecos)

## General Characteristics

Aims at implementing the RISC-V RV32I profile according to the specification.
Not quite there yet, but the CPU already executes code compiled with  the 
RISC-V gcc toolchain. Next to extend the CPU environment to be able to
run the  [RISC-V Compliance test suite](https://github.com/riscv/riscv-compliance). I expect a good part of it to pass right from the start...

+ Implements the FENCE and ECALL instructions as a NOPs
+ Two-stage pipeline
+ **No** cache, branch prediction, out of order execution, speculative execution, MMU

Beware: Uses non-standard VHDL including Xilinx specific stuff. 
This should easily be replaceable if someone ever feels the urge to port this thing.

## Documentation

## Future Plans / TODOs

## Software / Hardware

Building a GCC based RISC-V RV32I bare-metal toolchain:

git clone --recursive https://github.com/riscv/riscv-gnu-toolchain
cd riscv-gnu-toolchain
./configure --prefix=/opt/riscv --with-arch=rv32i --with-abi=ilp32
make
(or sudo make, depending on your access rights to the target directory)

Don't forget to add /opt/riscv/bin to your PATH environment variable.

Developed under GNU/Linux - [Debian](https://www.debian.org/) distribution

+ Uses the [Xilinx ISE WebPACK Design Software v 14.7](https://www.xilinx.com/products/design-tools/ise-design-suite/ise-webpack.html) for synthesis
+ Uses the [xc3sprog suite of utilities](http://xc3sprog.sourceforge.net/)
+ Uses the [JTAG-HS1 programming cable](https://store.digilentinc.com/jtag-hs1-programming-cable-limited-time/) for downloading the FPGA configuration bitstream
+ Tested on the [Spartan®-3 Starter Board](https://store.digilentinc.com/spartan-3-board-retired/). This board is rather old and no longer available.

## References

## Homepage And Source Code Repository

https://github.com/coronensis/rival

## Contact
Helmut Sipos <helmut.sipos@gmail.com>
