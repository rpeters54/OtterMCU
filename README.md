
# Otter MCU

Created by Riley Peters

TODO:

- [X]: Refactor otter_cu_fsm to merge the Fetch and Execute cycles into one
- [ ]: Generate a few simple end-to-end functional tests to ensure proper operation
- [X]: Add support for other mandatory CSRs as described in the RISC-V spec
- [X]: Add support for ECALL, EBREAK, WFI, FENCE, and CSR* instructions needed to be RV32I compliant
- [ ]: Test new CSRs, ensure they react properly
- [ ]: Add final rvfi traces, and work until core is spec compliant


RISCOF - KNOWN ISSUES:

- misaligned instructions caused by riscv32-unknown-elf-gcc, fixed by removing linker relaxation
- clone function grabs the commit of the latest release, not the latest commit. need to manually specify commit version with --get-version

BUGS FOUND:

- Allowed illegal instructions
- jalr is supposed to clear its lsb

