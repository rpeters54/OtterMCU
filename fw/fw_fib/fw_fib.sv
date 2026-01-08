`timescale 1ns / 1ps

module fw_fib;

    // ---------- Parameters ----------
    localparam ROM_FILE     = "fw_fib.hex";

    // MMIO finish address
    localparam IO_ADDR      = 32'h0001_0000;

    localparam WAIT_CYCLES  = 100000;

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
            mmio_req_q    <= 1'b0;
            tb_iobus_data <= 32'h0000_0000;
        end else begin
            mmio_req_q    <= iobus_is_mmio;
            tb_iobus_data <= 32'h0000_0000;
        end
    end


    // --------------------------------------------------------------------
    // Progress tracking
    // --------------------------------------------------------------------

    // Test Sequence and Verification
    int expected_val;
    int expected_results[$];

    initial begin
        int test_count = 0;

        $display("======================================================");
        $display("Starting Testbench for Fibonacci");
        $display("======================================================");

        // --- Populate Queue with Expected Results ---
        // These values must match the sequence of operations in your C code.
        expected_results = {
            0,
            1,
            1,
            2,
            3,
            5,
            8,
            13,
            21,
            34
        };

        // 1. Reset the processor
        rst      = 1'b1;
        intrpt   = 32'b0;
        repeat (5) @(posedge clk);
        rst      = 1'b0;
        $display("Reset released at time %0t.", $time);
        forever @(posedge clk) begin
            if (!rst && iobus_is_mmio && w_iobus_we) begin
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
                    $display("ALL FIBONACCI TESTS PASSED!");
                    $display("======================================================");
                    $finish;
                end
            end
        end
    end

    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, fw_fib);
    end

endmodule
