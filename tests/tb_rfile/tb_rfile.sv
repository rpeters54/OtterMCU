`timescale 1ns / 1ps

module tb_rfile;

    // Test bench signals
    reg clk;
    reg [4:0] r_addr1, r_addr2;
    reg w_en;
    reg [4:0] w_addr;
    reg [31:0] w_data;
    wire [31:0] r_rs1, r_rs2;

    // Instantiate the DUT (Device Under Test)
    otter_rfile dut (
        .i_clk(clk),
        .i_r_addr1(r_addr1),
        .i_r_addr2(r_addr2),
        .i_w_en(w_en),
        .i_w_addr(w_addr),
        .i_w_data(w_data),
        .o_r_rs1(r_rs1),
        .o_r_rs2(r_rs2)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns period (100MHz)
    end

    initial begin
        // Name as needed
        $dumpfile("tb_rfile.vcd");
        $dumpvars(0);
    end

    // Test variables
    integer i, j;
    reg [31:0] expected_data;
    integer error_count = 0;

    // Main test sequence
    initial begin
        $display("=== Register File Unit Tests ===");
        $display("Time: %0t", $time);

        // Initialize inputs
        drive_dut(0, 0, 0, 0, 0);

        // TEST 1: Initial state verification
        $display("\n--- TEST 1: Initial State Verification ---");
        test_initial_state();

        // TEST 2: Basic write and read operations
        $display("\n--- TEST 2: Basic Write/Read Operations ---");
        test_basic_write_read();

        // TEST 3: Register 0 write protection
        $display("\n--- TEST 3: Register 0 Write Protection ---");
        test_register_zero_protection();

        // TEST 4: Dual read functionality
        $display("\n--- TEST 4: Dual Read Functionality ---");
        test_dual_read();

        // TEST 5: Write enable control
        $display("\n--- TEST 5: Write Enable Control ---");
        test_write_enable_control();

        // TEST 6: Address boundary testing
        $display("\n--- TEST 6: Address Boundary Testing ---");
        test_address_boundaries();

        // TEST 7: Data pattern testing
        $display("\n--- TEST 7: Data Pattern Testing ---");
        test_data_patterns();

        // TEST 8: Simultaneous read/write operations
        $display("\n--- TEST 8: Simultaneous Read/Write Operations ---");
        test_simultaneous_read_write();

        // TEST 9: All registers comprehensive test
        $display("\n--- TEST 9: All Registers Comprehensive Test ---");
        test_all_registers();

        // Final results
        $display("\n=== TEST SUMMARY ===");
        if (error_count == 0) begin
            $display("ALL TESTS PASSED! ✓");
        end else begin
            $error("TESTS FAILED: %0d errors detected ✗", error_count);
        end
        $display("Simulation completed at time: %0t", $time);

        $finish(error_count > 0);
    end

    // Task: Test initial state
    task test_initial_state();
        begin
            $display("Testing initial register values...");
            //
            // Check all registers are initially zero
            for (i = 0; i < 32; i++) begin
                r_addr1 = i[4:0];
                if (r_rs1 !== 32'h0) begin
                    $error("ERROR: Register %0d not initialized to 0. Got: 0x%h", i, r_rs1);
                    error_count = error_count + 1;
                end
            end

            $display("Done");
        end
    endtask
    
    // Task: Test basic write and read
    task test_basic_write_read();
        begin
            $display("Testing basic write and read operations...");

            // write to x1
            drive_dut(0, 0, 1, 5'd1, 32'hDEADBEEF);
            // read from x1
            drive_dut(5'd1, 0, 0, 0, 0);

            if (r_rs1 !== 32'hDEADBEEF) begin
                $error("ERROR: Write/Read failed. Expected: 0xDEADBEEF, Got: 0x%h", r_rs1);
                error_count = error_count + 1;
            end

            $display("Done");
        end
    endtask

    // Task: Test register 0 protection
    task test_register_zero_protection();
        begin
            $display("Testing register 0 write protection...");

            // write to x0
            drive_dut(0, 0, 1, 5'd0, 32'hDEADBEEF);
            // read from x0
            drive_dut(5'd0, 0, 0, 0, 0);

            if (r_rs1 !== 32'h0) begin
                $error("ERROR: Register 0 not protected! Got: 0x%h", r_rs1);
                error_count = error_count + 1;
            end

            $display("Done");
        end
    endtask

    // Task: Test dual read functionality
    task test_dual_read();
        begin
            $display("Testing dual read functionality...");

            // write to x5
            drive_dut(0, 0, 1, 5'd5, 32'h12345678);
            // write to x10
            drive_dut(0, 0, 1, 5'd10, 32'h87654321);
            // read from both
            drive_dut(5'd5, 5'd10, 0, 0, 0);

            if (r_rs1 !== 32'h12345678 || r_rs2 !== 32'h87654321) begin
                $error("ERROR: Dual read failed. r_rs1: 0x%h, r_rs2: 0x%h", r_rs1, r_rs2);
                error_count = error_count + 1;
            end

            $display("Done");
        end
    endtask

    // Task: Test write enable control
    task test_write_enable_control();
        begin
            $display("Testing write enable control...");

            // write to x3
            drive_dut(0, 0, 1, 5'd3, 32'hABCDEF00);
            // write to x3 without enabling
            drive_dut(0, 0, 0, 5'd3, 32'h11111111);
            // read from x3
            drive_dut(5'd3, 0, 0, 0, 0);

            if (r_rs1 !== 32'hABCDEF00) begin
                $error("ERROR: Write enable not working. Expected: 0xABCDEF00, Got: 0x%h", r_rs1);
                error_count = error_count + 1;
            end

            $display("Done");
        end
    endtask

    // Task: Test address boundaries
    task test_address_boundaries();
        begin
            $display("Testing address boundary conditions...");

            drive_dut(0, 0, 1, 5'd31, 32'hFFFFFFFF);
            drive_dut(5'd31, 0, 0, 0, 0);

            if (r_rs1 !== 32'hFFFFFFFF) begin
                $error("ERROR: Register 31 access failed. Got: 0x%h", r_rs1);
                error_count = error_count + 1;
            end

             $display("Done");
        end
    endtask

    // Task: Test various data patterns
    task test_data_patterns();
        reg [31:0] test_patterns [0:7];
        begin
            $display("Testing various data patterns...");
            test_patterns[0] = 32'h00000000;
            test_patterns[1] = 32'hFFFFFFFF;
            test_patterns[2] = 32'hAAAAAAAA;
            test_patterns[3] = 32'h55555555;
            test_patterns[4] = 32'h12345678;
            test_patterns[5] = 32'h87654321;
            test_patterns[6] = 32'hF0F0F0F0;
            test_patterns[7] = 32'h0F0F0F0F;

            for (i = 0; i < 8; i = i + 1) begin
                drive_dut(0, 0, 1, i[4:0] + 5'd20, test_patterns[i]);
            end
            drive_dut(0, 0, 0, 0, 0);

            // Verify all patterns
            for (i = 0; i < 8; i = i + 1) begin
                drive_dut(i[4:0] + 5'd20, 0, 0, 0, 0);
                if (r_rs1 !== test_patterns[i]) begin
                    $error("ERROR: Pattern test failed for register %0d. Expected: 0x%h, Got: 0x%h", 
                            i+20, test_patterns[i], r_rs1);
                    error_count = error_count + 1;
                end
            end

            $display("Done");
        end
    endtask

    // Task: Test simultaneous read/write
    task test_simultaneous_read_write();
        begin
            $display("Testing simultaneous read/write operations...");

            // write to x15
            drive_dut(0, 0, 1, 5'd15, 32'hCAFEBABE);
            drive_dut(0, 0, 0, 0, 0);
            // write to x16 and read from x15
            drive_dut(5'd15, 0, 1, 5'd16, 32'hDEADC0DE);
            drive_dut(5'd15, 0, 0, 0, 0);

            // Check read operation worked
            if (r_rs1 !== 32'hCAFEBABE) begin
                $error("ERROR: Simultaneous read failed. Got: 0x%h", r_rs1);
                error_count = error_count + 1;
            end

            drive_dut(5'd16, 0, 0, 0, 0);

            if (r_rs1 !== 32'hDEADC0DE) begin
                $display("ERROR: Simultaneous write failed. Got: 0x%h", r_rs1);
                error_count = error_count + 1;
            end

            $display("Done");
        end
    endtask

    // Task: Comprehensive test of all registers
    task test_all_registers();
        begin
            $display("Testing all 32 registers comprehensively...");

            // Write unique values to all registers (except 0)
            for (i = 1; i < 32; i = i + 1) begin
                drive_dut(0, 0, 1, i[4:0], 32'h1000_0000 + i);
            end
            drive_dut(0, 0, 0, 0, 0);

            // Verify all registers have correct values
            for (i = 0; i < 32; i = i + 1) begin
                drive_dut(i[4:0], 0, 0, 0, 0);

                expected_data = (i == 0) ? 32'h0 : (32'h1000_0000 + i);
                if (r_rs1 !== expected_data) begin
                    $error("ERROR: Register %0d verification failed. Expected: 0x%h, Got: 0x%h", 
                            i, expected_data, r_rs1);
                    error_count = error_count + 1;
                end
            end

            $display("Done");
        end
    endtask

    task drive_dut (
        input [4:0]  i_r_addr1,
        input [4:0]  i_r_addr2,
        input        i_w_en,
        input [4:0]  i_w_addr,
        input [31:0] i_w_data
    );

        /* verilator lint_off INITIALDLY */
        r_addr1 <= i_r_addr1;
        r_addr2 <= i_r_addr2;
        w_en    <= i_w_en;
        w_addr  <= i_w_addr;
        w_data  <= i_w_data;
        /* verilator lint_on INITIALDLY */
        @(posedge clk);

    endtask

    // Monitor for debugging (optional)
    initial begin
        if ($test$plusargs("VERBOSE")) begin
            $monitor("Time: %0t | clk: %b | w_en: %b | w_addr: %0d | w_data: 0x%h | r_addr1: %0d | r_addr2: %0d | r_rs1: 0x%h | r_rs2: 0x%h",
                    $time, clk, w_en, w_addr, w_data, r_addr1, r_addr2, r_rs1, r_rs2);
        end
    end

endmodule
