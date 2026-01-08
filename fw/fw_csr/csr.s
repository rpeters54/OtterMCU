    .option norvc
    .equ OUT_BASE,   0x00010000       # stream of observed values

    .section .text
    .globl csr_test
csr_test:
    la      t0, trap_handler
    csrw    mtvec, t0
    li      t1, (1 << 3)              # MSTATUS.MIE
    csrrc   x0, mstatus, t1

    li      s0, OUT_BASE

    # mscratch = 0
    csrrw   x0, mscratch, x0

    # CSRRW
    li      t1, 0xA5A5A5A5
    csrrw   t0, mscratch, t1
    sw      t0, 0(s0)                 # [0]
    csrr    t2, mscratch
    sw      t2, 0(s0)                 # [1]

    # CSRRS
    li      t4, 0x00000F0F
    csrrs   t0, mscratch, t4
    sw      t0, 0(s0)                 # [2]
    csrr    t2, mscratch
    sw      t2, 0(s0)                 # [3]

    # CSRRS rs1=0 (no write)
    csrrs   t0, mscratch, x0
    sw      t0, 0(s0)                 # [4]
    csrr    s1, mscratch              # was t7
    sw      s1, 0(s0)                 # [5]

    # CSRRC
    li      t4, 0x000000F0
    csrrc   t0, mscratch, t4
    sw      t0, 0(s0)                 # [6]
    csrr    t2, mscratch
    sw      t2, 0(s0)                 # [7]

    # CSRRC rs1=0 (no write)
    csrrc   t0, mscratch, x0
    sw      t0, 0(s0)                 # [8]
    csrr    s1, mscratch              # was t7
    sw      s1, 0(s0)                 # [9]

    # CSRRWI zimm=31
    csrrwi  t0, mscratch, 31
    sw      t0, 0(s0)                 # [10]
    csrr    t2, mscratch
    sw      t2, 0(s0)                 # [11]

    # CSRRSI (after clear)
    csrrw   x0, mscratch, x0
    csrrsi  t0, mscratch, 0x15
    sw      t0, 0(s0)                 # [12]
    csrr    t2, mscratch
    sw      t2, 0(s0)                 # [13]

    # CSRRSI zimm=0 (no write)
    csrrsi  t0, mscratch, 0
    sw      t0, 0(s0)                 # [14]
    csrr    s1, mscratch              # was t7
    sw      s1, 0(s0)                 # [15]

    # CSRRCI (start from 0xFF)
    li      t0, 0xFF
    csrrw   x0, mscratch, t0
    csrrci  t0, mscratch, 0x0F
    sw      t0, 0(s0)                 # [16]
    csrr    t2, mscratch
    sw      t2, 0(s0)                 # [17]

    # CSRRCI zimm=0 (no write)
    csrrci  t0, mscratch, 0
    sw      t0, 0(s0)                 # [18]
    csrr    s1, mscratch              # was t7
    sw      s1, 0(s0)                 # [19]

    # ebreak
    ebreak
    csrr    t0, mcause
    sw      t0, 0(s0)                 # [20]

    # ecall (M-mode)
    ecall
    csrr    t0, mcause
    sw      t0, 0(s0)                 # [21]

    # wfi outcome
    wfi
    csrr    t0, mcause
    sw      t0, 0(s0)                 # [22]

1:  j       1b

    .align 4
trap_handler:
    # skip the faulting instruction
    csrr    t0, mepc
    addi    t0, t0, 4
    csrw    mepc, t0
    mret
