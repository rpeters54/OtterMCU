    .equ CSR_MSTATUS, 0x300
    .equ CSR_MIE,     0x304
    .equ CSR_MTVEC,   0x305
    .equ CSR_MEPC,    0x341
    .equ CSR_MCAUSE,  0x342

    .equ MSTATUS_MIE, 0x00000008

    # Enable MSIE (bit 3), MTIE (bit 7), MEIE (bit 11)
    .equ MIE_ISR_EN,  0x00000888

    .section .data
    .align 4

# Testbench-visible handshake words
old_mtvec:  .word 0
new_mtvec:  .word 0

old_mie:    .word 0
new_mie:    .word 0

old_mstatus: .word 0
new_mstatus: .word 0

# Testbench-visible handshake words
msi_req:    .word 0   # write 1 when we want SW interrupt asserted
msi_ack:    .word 0   # ISR writes 1 when MSIE interrupt handled

mtimer_req: .word 0   # write 1 when we want TIMER interrupt asserted
mtimer_ack: .word 0   # ISR writes 1 when MTIE interrupt handled

mext_req:   .word 0   # write 1 when we want EXT interrupt asserted
mext_ack:   .word 0   # ISR writes 1 when MEIE interrupt handled

done_flag:  .word 0   # set to 1 after all three pass

# Optional: per-interrupt hit counters for extra checking
cnt_msie:   .word 0
cnt_mtie:   .word 0
cnt_meie:   .word 0

    .section .text
    .globl main
main:
    # update mtvec to point to the isr
    la      t0, isr
    csrrw   t1, CSR_MTVEC, t0
    la      t2, old_mtvec
    sw      t1, 0(t2)
    la      t2, new_mtvec
    sw      t0, 0(t2)

    # enable all types of interrupts
    li      t0, MIE_ISR_EN
    csrrs   t1, CSR_MIE, t0
    la      t2, old_mie
    sw      t1, 0(t2)
    csrr    t3, CSR_MIE
    la      t2, new_mie
    sw      t3, 0(t2)

    # enable interrupts globally
    li      t0, MSTATUS_MIE
    csrrs   t1, CSR_MSTATUS, t0
    la      t2, old_mstatus
    sw      t1, 0(t2)
    csrr    t3, CSR_MSTATUS
    la      t2, new_mstatus
    sw      t3, 0(t2)

    # ---- Stage 1: request Software interrupt (MSIE) ----
    li      t4, 1
    la      t5, msi_req
    sw      t4, 0(t5)       # tell the testbench: assert MSIP now
wait_msie_ack:
    la      t6, msi_ack
    lw      t0, 0(t6)
    beqz    t0, wait_msie_ack
    sw      zero, 0(t5)     # clear request word (optional)

    # ---- Stage 2: request Timer interrupt (MTIE) ----
    li      t4, 1
    la      t5, mtimer_req
    sw      t4, 0(t5)       # tell the testbench: assert MTIP now
wait_mtie_ack:
    la      t6, mtimer_ack
    lw      t0, 0(t6)
    beqz    t0, wait_mtie_ack
    sw      zero, 0(t5)

    # ---- Stage 3: request External interrupt (MEIE) ----
    li      t4, 1
    la      t5, mext_req
    sw      t4, 0(t5)       # tell the testbench: assert MEIP now
wait_meie_ack:
    la      t6, mext_ack
    lw      t0, 0(t6)
    beqz    t0, wait_meie_ack
    sw      zero, 0(t5)

    # All three interrupts handled; signal done and park
    li      t4, 1
    la      t5, done_flag
    sw      t4, 0(t5)

halt_forever:
    wfi
    j       halt_forever

# -------------------- Interrupt Service Routine --------------------
# mtvec is programmed in DIRECT mode to this entry.
# Distinguish interrupt cause and ack the corresponding flag.
# mcause layout (M-mode): bit[31]=1 => interrupt, bits[3:0] cause:
#  3 = machine software, 7 = machine timer, 11 = machine external
isr:
    addi    sp, sp, -16
    sw      ra, 12(sp)
    sw      t0, 8(sp)
    sw      t1, 4(sp)
    sw      t2, 0(sp)

    csrr    t0, CSR_MCAUSE

    # ensure it really is an interrupt (bit 31 set)
    srai    t1, t0, 31
    beqz    t1, isr_done_restore    # not an interrupt? just return

    # extract cause code (clear bit 31)
    li      t1, 0x7FFFFFFF
    and     t0, t0, t1

    # cause == 3 (MSIE)
    li      t1, 3
    bne     t0, t1, check_mtie
    # ack MSIE
    la      t2, msi_ack
    li      t1, 1
    sw      t1, 0(t2)
    # bump counter
    la      t2, cnt_msie
    lw      t1, 0(t2)
    addi    t1, t1, 1
    sw      t1, 0(t2)
    j       isr_done_restore

check_mtie:
    li      t1, 7
    bne     t0, t1, check_meie
    # ack MTIE
    la      t2, mtimer_ack
    li      t1, 1
    sw      t1, 0(t2)
    # bump counter
    la      t2, cnt_mtie
    lw      t1, 0(t2)
    addi    t1, t1, 1
    sw      t1, 0(t2)
    j       isr_done_restore

check_meie:
    li      t1, 11
    bne     t0, t1, isr_done_restore
    # ack MEIE
    la      t2, mext_ack
    li      t1, 1
    sw      t1, 0(t2)
    # bump counter
    la      t2, cnt_meie
    lw      t1, 0(t2)
    addi    t1, t1, 1
    sw      t1, 0(t2)
    # fallthrough to epilogue

isr_done_restore:
    lw      ra, 12(sp)
    lw      t0, 8(sp)
    lw      t1, 4(sp)
    lw      t2, 0(sp)
    addi    sp, sp, 16
    mret

fail:
    j fail

