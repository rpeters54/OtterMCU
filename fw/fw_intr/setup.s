.section .text
    .global interrupt_handler
    .global setup_interrupts
    .align 2

#-----------------------------------------------------------------------
# Interrupt Service Routine (ISR)
#-----------------------------------------------------------------------
interrupt_handler:
    # Save register context
    addi sp, sp, -4
    sw   t0, 0(sp)

    # Set the interrupt handled flag
    la   t0, interrupt_fired_flag
    li   t1, 1
    sw   t1, 0(t0)

    # Restore context
    lw   t0, 0(sp)
    addi sp, sp, 4
    mret

#-----------------------------------------------------------------------
# Setup Function (called from C)
#-----------------------------------------------------------------------
setup_interrupts:
    # Set the address of our ISR the mtvec
    la   t0, interrupt_handler
    csrw mtvec, t0

    # Enable Machine External Interrupts in mie
    li   t0, 0x800
    csrs mie, t0

    # Enable interrupts globally in mstatus
    li   t0, 0x8
    csrs mstatus, t0
    ret
