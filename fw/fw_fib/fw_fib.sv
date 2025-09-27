`timescale 1ns / 1ps

module fw_fib;

    // Parameters
    localparam CLK_PERIOD = 10;
    localparam ROM_FILE   = "fw_fib.hex";
    localparam IO_ADDR    = 32'hFFFF0000;
    localparam TIMEOUT_CYCLES = 10000;

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
        $display("Starting Fibonacci Testbench");
        $display("======================================================");

        // --- Populate Queue with Expected Results ---
        expected_results.push_back(0);
        expected_results.push_back(1);
        expected_results.push_back(1);
        expected_results.push_back(2);
        expected_results.push_back(3);
        expected_results.push_back(5);
        expected_results.push_back(8);
        expected_results.push_back(13);
        expected_results.push_back(21);
        expected_results.push_back(34);

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
        $dumpvars(0, fw_fib);
    end

endmodule
