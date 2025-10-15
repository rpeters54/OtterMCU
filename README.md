
# Otter MCU

Created by: Riley Peters

## TODO:

### Core Modifications:

 - [x]: Refactor otter_cu_fsm to merge the Fetch and Execute cycles into one
 - [x]: Generate a few simple end-to-end functional tests to ensure proper operation
 - [x]: Add support for other mandatory CSRs as described in the RISC-V spec
 - [x]: Add support for ECALL, EBREAK, WFI, FENCE, and CSRxx instructions needed to be RV32I_Zicsr compliant
 - [x]: Test new CSRs, ensure they react properly

### Adding Riscof RISC-V Arch Test Suite:

 - [x]: Create the DUT Python plugin, linker script, and test stub needed to run
 - [x]: Configure the sail_riscv architectural model to run with the proper standard
 - [x]: Pass all compliance tests

### Adding RISC-V Formal Support:

 - [x]: Add rvfi traces for standard signals and implemented CSRs
 - [x]: Install and verify RV-Formal works locally
 - [X]: Pass all formal checks


## RISCOF - KNOWN ISSUES:

- misaligned instructions caused by riscv32-unknown-elf-gcc, fixed by removing linker relaxation
- clone function grabs the commit of the latest release, not the latest commit. need to manually specify commit version with --get-version

## BUGS FOUND:

- Allowed illegal instructions
- jalr is supposed to clear its lsb
- multi-cycle stalls can not rely on imem value remaining constant

