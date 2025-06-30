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
    Reg_File dut (
        .clk(clk),
        .r_addr1(r_addr1),
        .r_addr2(r_addr2),
        .w_en(w_en),
        .w_addr(w_addr),
        .w_data(w_data),
        .r_rs1(r_rs1),
        .r_rs2(r_rs2)
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
        r_addr1 = 0;
        r_addr2 = 0;
        w_en = 0;
        w_addr = 0;
        w_data = 0;
        
        // Wait for initial block to complete
        #1;
        
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
            $display("TESTS FAILED: %0d errors detected ✗", error_count);
        end
        $display("Simulation completed at time: %0t", $time);
        
        #50;
        $finish;
    end
    
    // Task: Test initial state
    task test_initial_state();
        begin
            $display("Testing initial register values...");
            
            // Check all registers are initially zero
            for (i = 0; i < 32; i++) begin
                r_addr1 = i[4:0];
                #1; // Allow combinational logic to settle
                if (r_rs1 !== 32'h0) begin
                    $display("ERROR: Register %0d not initialized to 0. Got: 0x%h", i, r_rs1);
                    error_count++;
                end
            end
            
            if (error_count == 0)
                $display("✓ All registers properly initialized to 0");
        end
    endtask
    
    // Task: Test basic write and read
    task test_basic_write_read();
        begin
            $display("Testing basic write and read operations...");
            
            // Write to register 1
            @(negedge clk);
            w_addr = 5'd1;
            w_data = 32'hDEADBEEF;
            w_en = 1;
            @(negedge clk);
            w_en = 0;
            @(negedge clk);
            
            // Read from register 1
            r_addr1 = 5'd1;
            #1;
            if (r_rs1 !== 32'hDEADBEEF) begin
                $display("ERROR: Write/Read failed. Expected: 0xDEADBEEF, Got: 0x%h", r_rs1);
                error_count++;
            end else begin
                $display("✓ Basic write/read successful");
            end
        end
    endtask
    
    // Task: Test register 0 protection
    task test_register_zero_protection();
        begin
            $display("Testing register 0 write protection...");
            
            // Attempt to write to register 0
            @(negedge clk);
            w_addr = 5'd0;
            w_data = 32'hDEADBEEF;
            w_en = 1;
            @(negedge clk);
            w_en = 0;
            @(negedge clk);
            
            // Read register 0
            r_addr1 = 5'd0;
            #1;
            if (r_rs1 !== 32'h0) begin
                $display("ERROR: Register 0 not protected! Got: 0x%h", r_rs1);
                error_count++;
            end else begin
                $display("✓ Register 0 properly protected");
            end
        end
    endtask
    
    // Task: Test dual read functionality
    task test_dual_read();
        begin
            $display("Testing dual read functionality...");
            
            // Write different values to two registers
            @(negedge clk);
            w_en = 1;
            w_addr = 5'd5;
            w_data = 32'h12345678;
            @(negedge clk);
            
            w_addr = 5'd10;
            w_data = 32'h87654321;
            @(negedge clk);
            w_en = 0;
            @(negedge clk);
            
            // Read both simultaneously
            r_addr1 = 5'd5;
            r_addr2 = 5'd10;
            #1;
            
            if (r_rs1 !== 32'h12345678 || r_rs2 !== 32'h87654321) begin
                $display("ERROR: Dual read failed. r_rs1: 0x%h, r_rs2: 0x%h", r_rs1, r_rs2);
                error_count++;
            end else begin
                $display("✓ Dual read functionality working correctly");
            end
        end
    endtask
    
    // Task: Test write enable control
    task test_write_enable_control();
        begin
            $display("Testing write enable control...");
            
            // Write a value to register 3
            @(negedge clk);
            w_addr = 5'd3;
            w_data = 32'hABCDEF00;
            w_en = 1;
            @(negedge clk);
            w_en = 0;
            #1;
            
            // Attempt to write with w_en = 0
            w_data = 32'h11111111;
            w_en = 0;
            @(negedge clk);
            
            // Check that register wasn't changed
            r_addr1 = 5'd3;
            #1;
            if (r_rs1 !== 32'hABCDEF00) begin
                $display("ERROR: Write enable not working. Expected: 0xABCDEF00, Got: 0x%h", r_rs1);
                error_count++;
            end else begin
                $display("✓ Write enable control working correctly");
            end
        end
    endtask
    
    // Task: Test address boundaries
    task test_address_boundaries();
        begin
            $display("Testing address boundary conditions...");
            
            // Test register 31 (highest address)
            @(negedge clk);
            w_addr = 5'd31;
            w_data = 32'hFFFFFFFF;
            w_en = 1;
            @(negedge clk);
            w_en = 0;
            @(negedge clk);
            
            r_addr1 = 5'd31;
            #1;
            if (r_rs1 !== 32'hFFFFFFFF) begin
                $display("ERROR: Register 31 access failed. Got: 0x%h", r_rs1);
                error_count++;
            end else begin
                $display("✓ Address boundary testing passed");
            end
        end
    endtask
    
    // Task: Test various data patterns
    task test_data_patterns();
        begin
            logic [31:0] test_patterns [0:7];

            $display("Testing various data patterns...");
            test_patterns[0] = 32'h00000000;
            test_patterns[1] = 32'hFFFFFFFF;
            test_patterns[2] = 32'hAAAAAAAA;
            test_patterns[3] = 32'h55555555;
            test_patterns[4] = 32'h12345678;
            test_patterns[5] = 32'h87654321;
            test_patterns[6] = 32'hF0F0F0F0;
            test_patterns[7] = 32'h0F0F0F0F;
            
            @(negedge clk);
            w_en = 1;
            for (i = 0; i < 8; i++) begin
                w_addr = i[4:0] + 5'd20; // Use registers 20-27
                w_data = test_patterns[i];
                @(negedge clk);
            end
            w_en = 0;
            @(negedge clk);
            
            // Verify all patterns
            for (i = 0; i < 8; i++) begin
                r_addr1 = i[4:0] + 5'd20;
                #1;
                if (r_rs1 !== test_patterns[i]) begin
                    $display("ERROR: Pattern test failed for register %0d. Expected: 0x%h, Got: 0x%h", 
                            i+20, test_patterns[i], r_rs1);
                    error_count++;
                end
            end
            
            if (error_count == 0)
                $display("✓ Data pattern testing passed");
        end
    endtask
    
    // Task: Test simultaneous read/write
    task test_simultaneous_read_write();
        begin
            $display("Testing simultaneous read/write operations...");
            
            // Write to register 15
            @(negedge clk);
            w_addr = 5'd15;
            w_data = 32'hCAFEBABE;
            w_en = 1;
            @(negedge clk);
            w_en = 0;
            @(negedge clk);
            
            // Simultaneously read from register 15 while writing to register 16
            r_addr1 = 5'd15;
            w_addr = 5'd16;
            w_data = 32'hDEADC0DE;
            w_en = 1;
            @(negedge clk);
            w_en = 0;
            @(negedge clk);
            
            // Check both operations worked
            if (r_rs1 !== 32'hCAFEBABE) begin
                $display("ERROR: Simultaneous read failed. Got: 0x%h", r_rs1);
                error_count++;
            end
            
            r_addr1 = 5'd16;
            #1;
            if (r_rs1 !== 32'hDEADC0DE) begin
                $display("ERROR: Simultaneous write failed. Got: 0x%h", r_rs1);
                error_count++;
            end
            
            if (error_count == 0)
                $display("✓ Simultaneous read/write operations passed");
        end
    endtask
    
    // Task: Comprehensive test of all registers
    task test_all_registers();
        begin
            $display("Testing all 32 registers comprehensively...");
            
            @(negedge clk);
            w_en = 1;
            // Write unique values to all registers (except 0)
            for (i = 1; i < 32; i++) begin
                w_addr = i[4:0];
                w_data = 32'h1000_0000 + i; // Unique pattern for each register
                @(negedge clk);
            end
            w_en = 0;
            @(negedge clk);
            
            // Verify all registers have correct values
            for (i = 0; i < 32; i++) begin
                r_addr1 = i[4:0];
                #1;
                expected_data = (i == 0) ? 32'h0 : (32'h1000_0000 + i);
                if (r_rs1 !== expected_data) begin
                    $display("ERROR: Register %0d verification failed. Expected: 0x%h, Got: 0x%h", 
                            i, expected_data, r_rs1);
                    error_count++;
                end
            end
            
            if (error_count == 0)
                $display("✓ All registers comprehensive test passed");
        end
    endtask
    
    // Monitor for debugging (optional)
    initial begin
        if ($test$plusargs("VERBOSE")) begin
            $monitor("Time: %0t | clk: %b | w_en: %b | w_addr: %0d | w_data: 0x%h | r_addr1: %0d | r_addr2: %0d | r_rs1: 0x%h | r_rs2: 0x%h",
                    $time, clk, w_en, w_addr, w_data, r_addr1, r_addr2, r_rs1, r_rs2);
        end
    end

endmodule