`timescale 1ns / 1ps

module fw_alu;

    // Parameters
    localparam CLK_PERIOD = 10;
    localparam ROM_FILE   = "fw_alu.hex";
    localparam IO_ADDR    = 32'hFFFF0000;
    localparam TIMEOUT_CYCLES = 2000;

    // Signals to connect to the DUT
    reg clk;
    reg rst;
    reg intrpt;
    reg [31:0] iobus_in;

    wire [31:0] iobus_out;
    wire [31:0] iobus_addr;
    wire        iobus_wr;

    // Instantiate the Device Under Test (DUT)
    otter_soc #(
        .ROM_FILE(ROM_FILE)
    ) dut (
        .clk(clk),
        .rst(rst),
        .intrpt(intrpt),
        .iobus_in(iobus_in),
        .iobus_out(iobus_out),
        .iobus_addr(iobus_addr),
        .iobus_wr(iobus_wr)
    );

    // Clock Generator
    initial begin
        clk = 0;
        forever #(CLK_PERIOD / 2) clk = ~clk;
    end

    // Test Sequence and Verification
    initial begin
        int expected_results[$];
        int test_count = 0;

        $display("======================================================");
        $display("Starting Testbench for ALU Verification");
        $display("======================================================");

        // --- Populate Queue with Expected Results ---
        // These values must match the sequence of operations in your C code.

        // R-Type Tests (a=20, b=-10, ua=20, ub=30)
        expected_results.push_back(10);           // ADD: 20 + (-10) = 10
        expected_results.push_back(30);           // SUB: 20 - (-10) = 30
        expected_results.push_back(83886080);        // SLL: 20 << (-10 & 0x1F) = 20 << 22 = 83886080
        expected_results.push_back(0);            // SLT: 20 < -10 (false) = 0
        expected_results.push_back(1);            // SLT: -10 < 20 (true) = 1
        expected_results.push_back(1);            // SLTU: 20 < 30 (true) = 1
        expected_results.push_back(0);            // SLTU: 30 < 20 (false) = 0
        expected_results.push_back(32'hFFFFFFE2); // XOR: 20 ^ -10 = 0x14 ^ 0xFFFFFFF6 = 0xFFFFFFE2
        expected_results.push_back(0);            // SRL: 20 >> (-10 & 0x1F) = 20 >> 22 = 0
        expected_results.push_back(0);            // SRA: 20 >> (-10 & 0x1F) = 20 >> 22 = 0
        expected_results.push_back(32'hFFFFFFFF); // SRA: -10 >> 20 = 0xFFFFFFF6 >> 20 = 0xFFFFFFFF = -1
        expected_results.push_back(32'hFFFFFFF6); // OR: 20 | -10 = 0x14 | 0xFFFFFFF6 = 0xFFFFFFF6
        expected_results.push_back(20);           // AND: 20 & -10 = 0x14 & 0xFFFFFFF6 = 0x14 = 20

        // I-Type Tests (a=50, neg_a=-50, ua=50)
        expected_results.push_back(65);           // ADDI: 50 + 15 = 65
        expected_results.push_back(35);           // ADDI: 50 + (-15) = 35
        expected_results.push_back(0);            // SLTI: 50 < 15 (false) = 0
        expected_results.push_back(1);            // SLTI: 50 < 100 (true) = 1
        expected_results.push_back(0);            // SLTIU: 50 < 15 (false) = 0
        expected_results.push_back(1);            // SLTIU: 50 < 100 (true) = 1
        expected_results.push_back(61);           // XORI: 50 ^ 15 = 0x32 ^ 0x0F = 0x3D = 61
        expected_results.push_back(63);           // ORI: 50 | 15 = 0x32 | 0x0F = 0x3F = 63
        expected_results.push_back(2);            // ANDI: 50 & 15 = 0x32 & 0x0F = 0x02 = 2
        expected_results.push_back(400);          // SLLI: 50 << 3 = 400
        expected_results.push_back(6);            // SRLI: 50 >> 3 = 6
        expected_results.push_back(6);            // SRAI: 50 >> 3 = 6
        expected_results.push_back(32'hFFFFFFF9); // SRAI: -50 >> 3 = 0xFFFFFFCE >> 3 = 0xFFFFFFF9 = -7

        // --- Start Simulation ---
        // 1. Reset the processor
        rst      = 1'b1;
        iobus_in = 32'b0;
        intrpt   = 1'b0;
        repeat (5) @(posedge clk);
        rst      = 1'b0;
        $display("Reset released at time %0t.", $time);

        // 2. Fork a timeout process
        fork
            begin
                #(TIMEOUT_CYCLES * CLK_PERIOD);
                $error("FAIL: Simulation timed out after %0d cycles.", TIMEOUT_CYCLES);
                $finish(1);
            end
        join_none

        // 3. Monitor the iobus for writes and verify results
        forever @(posedge clk) begin
            if (!rst && iobus_wr && iobus_addr == IO_ADDR) begin
                int expected_val;
                test_count++;

                if (expected_results.size() == 0) begin
                    $error("FAIL: Received more results than expected! Test count: %0d", test_count);
                    $finish(1);
                end

                expected_val = expected_results.pop_front();

                if (iobus_out == expected_val) begin
                    $display("PASS [%0d]: Saw 0x%h (%0d), Expected 0x%h (%0d)", 
                            test_count, iobus_out, $signed(iobus_out), expected_val, $signed(expected_val));
                end else begin
                    $error("FAIL [%0d]: Saw 0x%h (%0d), Expected 0x%h (%0d)", 
                            test_count, iobus_out, $signed(iobus_out), expected_val, $signed(expected_val));
                    $finish(1);
                end

                // Check for test completion
                if (expected_results.size() == 0) begin
                    $display("\n======================================================");
                    $display("ALL ALU TESTS PASSED!");
                    $display("======================================================");
                    $finish;
                end
            end
        end
    end

    // Optional: Dump waves for debugging
    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, fw_alu);
    end

endmodule
