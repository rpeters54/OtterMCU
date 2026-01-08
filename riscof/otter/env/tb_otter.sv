`timescale 1ns / 1ps
module tb_otter;

    // -- Parameters --
    parameter BRAM_BYTES    = 2**28;         // must cover your .hex
    parameter RESET_VEC     = 32'h8000_1000; // match firmware link addr
    parameter TOHOST_PTR    = 32'h8000_0000;
    parameter SIG_START_PTR = 32'h8000_0004;
    parameter SIG_END_PTR   = 32'h8000_0008;
    parameter I_DLY         = 2;             // IMEM fixed latency cycles
    parameter D_DLY         = 2;             // DMEM fixed latency cycles

    localparam BRAM_WIDTH   = $clog2(BRAM_BYTES);

    // ---------- Clock / reset ----------
    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;

    initial begin
        #100;
        rst = 0;
    end

    localparam WAIT_CYCLES  = 100000;
    initial begin
        repeat (WAIT_CYCLES) @(posedge clk);
        $error("Timeout");
        $finish(1);
    end

    // ---------- SoC IO bus wires ----------
    // TB drives these into the SoC (slave -> master)
    reg  [31:0] tb_iobus_data  = 32'h0;

    // SoC drives these out (master -> slave)
    wire        w_iobus_re;
    wire        w_iobus_we;
    wire [3:0]  w_iobus_sel;
    wire [31:0] w_iobus_addr;
    wire [31:0] w_iobus_wdata;

    // have MTIP pending to match sail behavior
    wire [31:0] intrpt = 32'h0000_0080;

    // ---------- DUT: SoC (includes MCU + BRAM memory) ----------
    otter_soc #(
        .BRAM_BYTES (BRAM_BYTES),
        .RESET_VEC  (RESET_VEC)
    ) dut (
        .i_clk          (clk),
        .i_rst          (rst),
        .i_intrpt       (intrpt),

        // External IO bus (everything outside BRAM range)
        .i_iobus_data   (tb_iobus_data),
        .o_iobus_re     (w_iobus_re),

        .o_iobus_we     (w_iobus_we),
        .o_iobus_sel    (w_iobus_sel),
        .o_iobus_addr   (w_iobus_addr),
        .o_iobus_data   (w_iobus_wdata)
    );

    // ---------- MMIO responder (single-cycle, TB-owned) ----------
    // - ACKs one cycle after seeing a request to IO_ADDR
    // - Returns zero on reads (writes carry the data from w_iobus_wdata)
    wire iobus_req     = w_iobus_re || w_iobus_we;
    wire iobus_is_mmio = iobus_req && !w_iobus_addr[31];

    reg mmio_req_q;

    always @(posedge clk) begin
        if (rst) begin
            mmio_req_q    <= 1'b0;
            tb_iobus_data <= 32'h0000_0000;
        end else begin
            mmio_req_q    <= iobus_is_mmio;
            tb_iobus_data <= 32'h0000_0000;
        end
    end

    reg [31:0] tohost_addr, sig_start_addr, sig_end_addr;
    string mem_file, sig_file;

    // memfile and sigfile plusargs
    initial begin
        if (!$value$plusargs("MEMFILE=%s", mem_file)) $fatal("Need +MEMFILE");
        if (!$value$plusargs("SIGFILE=%s", sig_file)) $fatal("Need +SIGFILE");

        $display("MEMFILE=%s",   mem_file);
        $display("SIGFILE=%s",   sig_file);
        $readmemh(mem_file, dut.u_otter_mem.w_mem);

        @(negedge rst);
        @(posedge clk);
        tohost_addr    = dut.u_otter_mem.w_mem[TOHOST_PTR[BRAM_WIDTH-1:2]];
        sig_start_addr = dut.u_otter_mem.w_mem[SIG_START_PTR[BRAM_WIDTH-1:2]];
        sig_end_addr   = dut.u_otter_mem.w_mem[SIG_END_PTR[BRAM_WIDTH-1:2]];

        $display("INFO: tohost_addr    = 0x%08h", tohost_addr);
        $display("INFO: sig_start_addr = 0x%08h", sig_start_addr);
        $display("INFO: sig_end_addr   = 0x%08h", sig_end_addr);
    end

    // -- Memory Interface Logic --

    integer sig_start_idx, sig_end_idx;
    always @(posedge clk) begin
        // Check for test completion
        if (dut.w_dmem_we && dut.w_dmem_addr == tohost_addr && dut.w_dmem_w_data != 0) begin
            $display("INFO: Test finished (tohost = 0x%08h)", dut.w_dmem_w_data);
            $display("INFO: Writing signature to %s", sig_file);

            // Dump signature region from fixed location at end of address space
            sig_start_idx = (sig_start_addr >> 2) & ((BRAM_BYTES/4) - 1);
            sig_end_idx   = ((sig_end_addr  >> 2) & ((BRAM_BYTES/4) - 1)) - 1;

            if (sig_end_idx < sig_start_idx ||
                sig_start_idx < 0 ||
                sig_end_idx >= (BRAM_BYTES/4)) begin
              $fatal(1, "Bad signature range: start=%0d end=%0d (mem words=%0d)",
                     sig_start_idx, sig_end_idx, BRAM_BYTES/4);
            end

            $display("INFO: Dumping signature words [%0d .. %0d]", sig_start_idx, sig_end_idx);

            $writememh(
                sig_file,
                dut.u_otter_mem.w_mem,
                sig_start_idx,
                sig_end_idx
            );
            $finish;
        end
    end

    initial begin
        // Name as needed
        $dumpfile("waveform.vcd");
        $dumpvars(0);
    end

endmodule
