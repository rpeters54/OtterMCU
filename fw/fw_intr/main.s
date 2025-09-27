.global _start

_start:
    # Initialize stack pointer
    la sp, _stack_top
    
    # Copy initialized data from ROM to RAM
    la t0, _ldata        # Source: ROM location of init data
    la t1, _sdata        # Destination: Start of data in RAM
    la t2, _edata        # End of data in RAM
    
copy_data_loop:
    beq t1, t2, copy_data_done  # If we've reached the end, we're done
    lw t3, 0(t0)         # Load word from ROM
    sw t3, 0(t1)         # Store word to RAM
    addi t0, t0, 4       # Increment source pointer
    addi t1, t1, 4       # Increment destination pointer
    j copy_data_loop     # Continue loop

copy_data_done:
    # Clear BSS section (zero uninitialized variables)
    la t0, _sbss         # Start of BSS
    la t1, _ebss         # End of BSS
    
clear_bss_loop:
    beq t0, t1, clear_bss_done  # If we've reached the end, we're done
    sw zero, 0(t0)       # Store zero
    addi t0, t0, 4       # Increment pointer
    j clear_bss_loop     # Continue loop

clear_bss_done:
    # Call main function
    call main
    
    # If main returns, loop forever
end_loop:
    j end_loop
