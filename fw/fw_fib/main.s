
    .section .boot_constants
    .align 4
    .global tohost_addr
    .dword 0
    .global begin_sig_addr
    .dword 0
    .global end_sig_addr
    .dword 0


    .section .text.init
    .global rvtest_entry_point
rvtest_entry_point:
    # Initialize stack pointer (grows downward from high address)
    la sp, _end
    li t0, 0x100000         # 1MB stack space
    add sp, sp, t0
    
    # Clear BSS section (zero uninitialized variables)
    la t0, __bss_start
    la t1, __bss_end
    
clear_bss_loop:
    beq t0, t1, clear_bss_done
    sw zero, 0(t0)
    addi t0, t0, 4
    j clear_bss_loop
clear_bss_done:
    # Call main function
    call main
    
    # If main returns, loop forever
end_loop:
    j end_loop

