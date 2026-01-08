.section .text
    .global interrupt_handler
    .global setup_interrupts
    .align 2

interrupt_handler:
    # Save register context (need both t0 and t1)
    addi sp, sp, -8
    sw   t0, 0(sp)
    sw   t1, 4(sp)
    
    # Set the interrupt handled flag
    la   t0, interrupt_fired_flag
    li   t1, 1
    sw   t1, 0(t0)
    
    # Restore context
    lw   t1, 4(sp)
    lw   t0, 0(sp)
    addi sp, sp, 8
    mret

setup_interrupts:
    # Set the address of our ISR in mtvec
    la   t0, interrupt_handler
    csrw mtvec, t0
    # Enable relevant interrupt in mie
    csrw mie, x0
    csrs mie, a0
    # Enable interrupts globally in mstatus (MIE bit)
    li   t0, 0x8
    csrs mstatus, t0
    ret
