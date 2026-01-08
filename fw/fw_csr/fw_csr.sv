`timescale 1ns / 1ps

module fw_csr;

    // ---------- Parameters ----------
    localparam ROM_FILE     = "fw_csr.hex";

    // MMIO finish address
    localparam IO_ADDR      = 32'h0001_0000;

    localparam WAIT_CYCLES  = 1000;

    // SoC memory/boot
    parameter BRAM_BYTES   = 2**16;         // must cover your .hex
    parameter RESET_VEC    = 32'h8000_1000; // match firmware link addr
    parameter I_DLY        = 2;             // IMEM fixed latency cycles
    parameter D_DLY        = 2;             // DMEM fixed latency cycles

    // ---------- Clock / reset ----------
    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;

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

    reg  [31:0] intrpt;

    // ---------- DUT: SoC (includes MCU + BRAM memory) ----------
    otter_soc #(
        .ROM_FILE   (ROM_FILE),
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
    wire iobus_is_mmio = iobus_req && (w_iobus_addr == IO_ADDR);

    reg mmio_req_q;
    always @(posedge clk) begin
        if (rst) begin
            mmio_req_q      <= 1'b0;
            tb_iobus_data   <= 32'h0000_0000;
        end else begin
            mmio_req_q    <= iobus_is_mmio;
            tb_iobus_data <= 32'h0000_0000;
        end
    end

    // --------------------------------------------------------------------
    // Progress tracking
    // --------------------------------------------------------------------
    reg [31:0] reported_mask;   // bits the FW has acknowledged (writes to FINISH_ADDR)
    reg [31:0] fired_mask;      // bits we have actually pulsed

    integer cycles;
    reg [31:0] ready_bits;
    reg [31:0] finish_bits;

    // Test Sequence and Verification
    initial begin
        int expected_results[$];
        int test_count = 0;

        $display("======================================================");
        $display("Starting CSR Testbench");
        $display("======================================================");

        // --- Populate Queue with Expected Results ---
        expected_results = {
            32'h00000000,
            32'hA5A5A5A5,
            32'hA5A5A5A5,
            32'hA5A5AFAF,
            32'hA5A5AFAF,
            32'hA5A5AFAF,
            32'hA5A5AFAF,
            32'hA5A5AF0F,
            32'hA5A5AF0F,
            32'hA5A5AF0F,
            32'hA5A5AF0F,
            32'h0000001F,
            32'h00000000,
            32'h00000015,
            32'h00000015,
            32'h00000015,
            32'h000000FF,
            32'h000000F0,
            32'h000000F0,
            32'h000000F0,
            32'h00000003,
            32'h0000000B,
            32'h0000000B
        };


        // 1. Reset the processor
        rst      = 1'b1;
        intrpt   = 32'b0;
        repeat (5) @(posedge clk);
        rst      = 1'b0;
        $display("Reset released at time %0t.", $time);
        forever @(posedge clk) begin
            if (!rst && iobus_is_mmio && w_iobus_we) begin
                int expected_val;
                test_count++;

                if (expected_results.size() == 0) begin
                    $error("FAIL: Received more results than expected! Test count: %0d", test_count);
                    $finish(1);
                end

                expected_val = expected_results.pop_front();

                if (w_iobus_wdata == expected_val) begin
                    $display("PASS [%0d]: Saw 0x%h (%0d), Expected 0x%h (%0d)", 
                            test_count, w_iobus_wdata, $signed(w_iobus_wdata), expected_val, $signed(expected_val));
                end else begin
                    $error("FAIL [%0d]: Saw 0x%h (%0d), Expected 0x%h (%0d)", 
                            test_count, w_iobus_wdata, $signed(w_iobus_wdata), expected_val, $signed(expected_val));
                    $finish(1);
                end

                // Check for test completion
                if (expected_results.size() == 0) begin
                    $display("\n======================================================");
                    $display("ALL CSR TESTS PASSED!");
                    $display("======================================================");
                    $finish;
                end
            end
        end
    end

    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, fw_csr);
    end

endmodule
