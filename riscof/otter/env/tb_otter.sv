`timescale 1ns / 1ps
module tb_otter;

    // -- Parameters --

    parameter MEM_EXP       = 28;
    parameter MEM_SIZE      = 2**MEM_EXP;

    parameter RESET_VEC     = 32'h80001000;
    parameter TOHOST_PTR    = 32'h80000000;
    parameter SIG_START_PTR = 32'h80000004;
    parameter SIG_END_PTR   = 32'h80000008;

    reg clk = 0;
    reg rst = 1;

    always begin
        #5 clk = ~clk;
    end

    initial begin
        repeat (5) @(posedge clk);
        rst = 0;
    end

    initial begin
        repeat (100000) @(posedge clk);
        $error("Timeout");
        $finish(1);
    end

    // create word-addressible behavioral memory
    reg [31:0] prog_mem  [0:MEM_SIZE/4-1];

    reg [31:0] tohost_addr, sig_start_addr, sig_end_addr;

    string mem_file, sig_file;

    // memfile and sigfile plusargs
    initial begin
        if (!$value$plusargs("MEMFILE=%s", mem_file)) $fatal("Need +MEMFILE");
        if (!$value$plusargs("SIGFILE=%s", sig_file)) $fatal("Need +SIGFILE");

        $display("MEMFILE=%s",   mem_file);
        $display("SIGFILE=%s",   sig_file);
        $readmemh(mem_file, prog_mem);

        @(negedge rst);
        @(posedge clk);
        tohost_addr    = prog_mem[TOHOST_PTR[MEM_EXP-1:2]];
        sig_start_addr = prog_mem[SIG_START_PTR[MEM_EXP-1:2]];
        sig_end_addr   = prog_mem[SIG_END_PTR[MEM_EXP-1:2]];

        $display("INFO: tohost_addr    = 0x%08h", tohost_addr);
        $display("INFO: sig_start_addr = 0x%08h", sig_start_addr);
        $display("INFO: sig_end_addr   = 0x%08h", sig_end_addr);
    end


    `define RISCV_FORMAL
    `ifdef RISCV_FORMAL

    reg [63:0] cycle_count = 0;
    string log_line;
    
    // Track previous CSR values to detect changes
    reg [31:0] prev_mstatus = 0;
    reg [31:0] prev_mtvec = 0;
    reg [31:0] prev_mepc = 0;
    reg [31:0] prev_mcause = 0;
    reg [31:0] prev_mtval = 0;
    reg [31:0] prev_mip = 0;
    reg [31:0] prev_mie = 0;
    
    always @(posedge clk) begin
        if (!rst) begin
            cycle_count <= cycle_count + 1;

            // Log only on instruction retirement using the RVFI valid signal
            if (mcu.rvfi_valid) begin
                log_line = $sformatf("[%06d] PC: %08h | INST: %08h",
                    cycle_count, mcu.rvfi_pc_rdata, mcu.rvfi_insn);

                // Log register writes (if rd is not x0)
                if (mcu.rvfi_rd_addr != 0) begin
                    log_line = $sformatf("%s | RD[%02d] <= %08h",
                        log_line, mcu.rvfi_rd_addr, mcu.rvfi_rd_wdata);
                end

                // === SIGNATURE REGION TRACKING ===
                // Log data memory writes with special highlighting for signature region
                if (|mcu.rvfi_mem_wmask) begin
                    if (mcu.rvfi_mem_addr >= sig_start_addr && mcu.rvfi_mem_addr < sig_end_addr) begin
                        log_line = $sformatf("%s | *** SIG_WR[%08h] <= %08h (mask:%b) ***",
                            log_line, mcu.rvfi_mem_addr, mcu.rvfi_mem_wdata, mcu.rvfi_mem_wmask);
                        $display("===> SIGNATURE WRITE at offset 0x%08h: data=0x%08h", 
                            mcu.rvfi_mem_addr - sig_start_addr, mcu.rvfi_mem_wdata);
                    end else begin
                        log_line = $sformatf("%s | MEM_WR[%08h] <= %08h (mask:%b)",
                            log_line, mcu.rvfi_mem_addr, mcu.rvfi_mem_wdata, mcu.rvfi_mem_wmask);
                    end
                end

                // Log data memory reads
                if (|mcu.rvfi_mem_rmask) begin
                    log_line = $sformatf("%s | MEM_RD[%08h] => %08h (mask:%b)",
                        log_line, mcu.rvfi_mem_addr, mcu.rvfi_mem_rdata, mcu.rvfi_mem_rmask);
                end

                $display(log_line);
            end
            
            // === TRAP/INTERRUPT CSR TRACKING ===
            // Detect trap/interrupt by checking if we took an interrupt or trap
            if (mcu.rvfi_valid && (mcu.rvfi_trap || mcu.rvfi_intr)) begin
                $display("================================================================================");
                if (mcu.rvfi_intr) begin
                    $display(">>> INTERRUPT TAKEN at cycle %0d", cycle_count);
                end else begin
                    $display(">>> TRAP TAKEN at cycle %0d", cycle_count);
                end
                $display("    Faulting PC:  0x%08h", mcu.rvfi_pc_rdata);
                $display("    Next PC:      0x%08h", mcu.rvfi_pc_wdata);
                $display("--------------------------------------------------------------------------------");
            end
            
            // Check for CSR changes (accessing internal CSR module)
            // Note: Adjust path if your CSR module has a different instance name
            if (mcu.csr.mstatus !== prev_mstatus) begin
                $display("    CSR mstatus:  0x%08h -> 0x%08h (MIE=%b, MPIE=%b)", 
                    prev_mstatus, mcu.csr.mstatus, 
                    mcu.csr.mstatus[3], mcu.csr.mstatus[7]);
                prev_mstatus <= mcu.csr.mstatus;
            end
            
            if (mcu.csr.mtvec !== prev_mtvec) begin
                $display("    CSR mtvec:    0x%08h -> 0x%08h (BASE=0x%08h, MODE=%0d)", 
                    prev_mtvec, mcu.csr.mtvec,
                    {mcu.csr.mtvec[31:2], 2'b00}, mcu.csr.mtvec[1:0]);
                prev_mtvec <= mcu.csr.mtvec;
            end
            
            if (mcu.csr.mepc !== prev_mepc) begin
                $display("    CSR mepc:     0x%08h -> 0x%08h", 
                    prev_mepc, mcu.csr.mepc);
                prev_mepc <= mcu.csr.mepc;
            end
            
            if (mcu.csr.mcause !== prev_mcause) begin
                $display("    CSR mcause:   0x%08h -> 0x%08h (Interrupt=%b, Code=%0d)", 
                    prev_mcause, mcu.csr.mcause,
                    mcu.csr.mcause[31], mcu.csr.mcause[30:0]);
                prev_mcause <= mcu.csr.mcause;
            end
            
            if (mcu.csr.mtval !== prev_mtval) begin
                $display("    CSR mtval:    0x%08h -> 0x%08h", 
                    prev_mtval, mcu.csr.mtval);
                prev_mtval <= mcu.csr.mtval;
            end
            
            if (mcu.csr.mip !== prev_mip) begin
                $display("    CSR mip:      0x%08h -> 0x%08h (MEIP=%b, MTIP=%b, MSIP=%b)", 
                    prev_mip, mcu.csr.mip,
                    mcu.csr.mip[11], mcu.csr.mip[7], mcu.csr.mip[3]);
                prev_mip <= mcu.csr.mip;
            end
            
            if (mcu.csr.mie !== prev_mie) begin
                $display("    CSR mie:      0x%08h -> 0x%08h (MEIE=%b, MTIE=%b, MSIE=%b)", 
                    prev_mie, mcu.csr.mie,
                    mcu.csr.mie[11], mcu.csr.mie[7], mcu.csr.mie[3]);
                prev_mie <= mcu.csr.mie;
            end
            
            // Print separator after trap/interrupt and CSR updates
            if (mcu.rvfi_valid && (mcu.rvfi_trap || mcu.rvfi_intr)) begin
                $display("================================================================================");
            end
        end
    end
    `endif

    wire        dmem_r_en, dmem_w_en;
    wire [3:0]  dmem_w_strb;
    wire [31:0] imem_addr, dmem_addr, dmem_w_data;
    reg  [31:0] imem_r_data, dmem_r_data;

    // have MTIP pending to match sail behavior
    wire [31:0] intrpt = 32'h0000_0080;


    // NOTE: ignore missing connections for rvfi.
    // Added the define because the signals are useful for debugging

    /* verilator lint_off PINMISSING */
    otter_mcu #(
        .RESET_VEC(RESET_VEC)
    ) mcu (
        .clk(clk),
        .rst(rst),
        .intrpt(intrpt),

        // Instruction Memory Interface
        .imem_r_data(imem_r_data),
        .imem_addr(imem_addr),

        // Data Memory Interface
        .dmem_r_data(dmem_r_data),
        .dmem_r_en(dmem_r_en),
        .dmem_w_en(dmem_w_en),
        .dmem_w_strb(dmem_w_strb),
        .dmem_addr(dmem_addr),
        .dmem_w_data(dmem_w_data)
    );
    /* verilator lint_on PINMISSING */


    // -- Memory Interface Logic --

    integer sig_start_idx, sig_end_idx;
    always @(posedge clk) begin

        // Synchronous reads from prog_mem, 
        imem_r_data <= prog_mem[imem_addr[MEM_EXP-1:2]];

        // Synchronous reads from prog_mem or debug_mem
        if (dmem_r_en) begin
            dmem_r_data <= prog_mem[dmem_addr[MEM_EXP-1:2]];
        end

        // Synchronous writes to prog_mem or debug_mem
        if (dmem_w_en) begin
            if (dmem_w_strb[0]) prog_mem[dmem_addr[MEM_EXP-1:2]][ 7: 0] <= dmem_w_data[ 7: 0];
            if (dmem_w_strb[1]) prog_mem[dmem_addr[MEM_EXP-1:2]][15: 8] <= dmem_w_data[15: 8];
            if (dmem_w_strb[2]) prog_mem[dmem_addr[MEM_EXP-1:2]][23:16] <= dmem_w_data[23:16];
            if (dmem_w_strb[3]) prog_mem[dmem_addr[MEM_EXP-1:2]][31:24] <= dmem_w_data[31:24];

            // Check for test completion
            if (dmem_addr == tohost_addr && dmem_w_data != 0) begin
                $display("INFO: Test finished (tohost = 0x%08h)", dmem_w_data);
                $display("INFO: Writing signature to %s", sig_file);
                // Dump signature region from fixed location at end of address space

                sig_start_idx = (sig_start_addr >> 2) & ((MEM_SIZE/4) - 1);
                sig_end_idx   = ((sig_end_addr  >> 2) & ((MEM_SIZE/4) - 1)) - 1;

                if (sig_end_idx < sig_start_idx ||
                    sig_start_idx < 0 ||
                    sig_end_idx >= (MEM_SIZE/4)) begin
                  $fatal(1, "Bad signature range: start=%0d end=%0d (mem words=%0d)",
                         sig_start_idx, sig_end_idx, MEM_SIZE/4);
                end

                $display("INFO: Dumping signature words [%0d .. %0d]", sig_start_idx, sig_end_idx);

                $writememh(
                    sig_file,
                    prog_mem,
                    sig_start_idx,
                    sig_end_idx
                );
                $finish;
            end
        end
    end

endmodule
