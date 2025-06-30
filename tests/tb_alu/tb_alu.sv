
`timescale 1ns / 1ps
`include "../../rtl/Defines.svh"

module tb_alu;

    // Testbench signals
    reg  [31:0] src_a;
    reg  [31:0] src_b;
    reg  [3:0]  func;
    wire [31:0] result;
    
    // Expected result for checking
    reg  [31:0] expected;
    
    // Test counters
    integer test_count = 0;
    integer pass_count = 0;
    integer fail_count = 0;

    // Instantiate the ALU
    ALU dut (
        .src_a(src_a),
        .src_b(src_b),
        .func(func),
        .result(result)
    );

    initial begin
        // Name as needed
        $dumpfile("tb_alu.vcd");
        $dumpvars(0);
    end

    // Task to check results
    task check_result(
        input [31:0] exp,
        input string test_name
    );
        begin
            test_count = test_count + 1;
            if (result === exp) begin
                $display("PASS: %s | A=%h, B=%h, func=%b, result=%h", 
                         test_name, src_a, src_b, func, result);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL: %s | A=%h, B=%h, func=%b, result=%h, expected=%h", 
                         test_name, src_a, src_b, func, result, exp);
                fail_count = fail_count + 1;
            end
            #1; // Small delay between tests
        end
    endtask

    // Main test sequence
    initial begin
        $display("=== ALU Testbench Started ===");
        $display("Time: %0t", $time);
        
        // Initialize inputs
        src_a = 0;
        src_b = 0;
        func = 0;
        #10;

        // Test ADD function
        $display("\n--- Testing ADD Function ---");
        func = ALU_ADD;
        
        // Basic addition
        src_a = 32'h12345678; src_b = 32'h87654321; expected = 32'h99999999;
        #1; check_result(expected, "ADD basic");
        
        // Add with zero
        src_a = 32'h12345678; src_b = 32'h00000000; expected = 32'h12345678;
        #1; check_result(expected, "ADD with zero");
        
        // Add negative numbers
        src_a = 32'hFFFFFFFF; src_b = 32'h00000001; expected = 32'h00000000;
        #1; check_result(expected, "ADD overflow");
        
        // Add maximum values
        src_a = 32'h7FFFFFFF; src_b = 32'h00000001; expected = 32'h80000000;
        #1; check_result(expected, "ADD max positive");

        // Test SUB function
        $display("\n--- Testing SUB Function ---");
        func = ALU_SUB;
        
        // Basic subtraction
        src_a = 32'h87654321; src_b = 32'h12345678; expected = 32'h7530ECA9;
        #1; check_result(expected, "SUB basic");
        
        // Subtract zero
        src_a = 32'h12345678; src_b = 32'h00000000; expected = 32'h12345678;
        #1; check_result(expected, "SUB zero");
        
        // Subtract same number
        src_a = 32'h12345678; src_b = 32'h12345678; expected = 32'h00000000;
        #1; check_result(expected, "SUB same");
        
        // Subtract larger from smaller
        src_a = 32'h12345678; src_b = 32'h87654321; expected = 32'h8ACF1357;
        #1; check_result(expected, "SUB underflow");

        // Test OR function
        $display("\n--- Testing OR Function ---");
        func = ALU_OR;
        
        // Basic OR
        src_a = 32'h0F0F0F0F; src_b = 32'hF0F0F0F0; expected = 32'hFFFFFFFF;
        #1; check_result(expected, "OR basic");
        
        // OR with zero
        src_a = 32'h12345678; src_b = 32'h00000000; expected = 32'h12345678;
        #1; check_result(expected, "OR with zero");
        
        // OR with all ones
        src_a = 32'h12345678; src_b = 32'hFFFFFFFF; expected = 32'hFFFFFFFF;
        #1; check_result(expected, "OR with all ones");

        // Test AND function
        $display("\n--- Testing AND Function ---");
        func = ALU_AND;
        
        // Basic AND
        src_a = 32'h0F0F0F0F; src_b = 32'hF0F0F0F0; expected = 32'h00000000;
        #1; check_result(expected, "AND basic");
        
        // AND with zero
        src_a = 32'h12345678; src_b = 32'h00000000; expected = 32'h00000000;
        #1; check_result(expected, "AND with zero");
        
        // AND with all ones
        src_a = 32'h12345678; src_b = 32'hFFFFFFFF; expected = 32'h12345678;
        #1; check_result(expected, "AND with all ones");
        
        // AND same value
        src_a = 32'h12345678; src_b = 32'h12345678; expected = 32'h12345678;
        #1; check_result(expected, "AND same");

        // Test XOR function
        $display("\n--- Testing XOR Function ---");
        func = ALU_XOR;
        
        // Basic XOR
        src_a = 32'h0F0F0F0F; src_b = 32'hF0F0F0F0; expected = 32'hFFFFFFFF;
        #1; check_result(expected, "XOR basic");
        
        // XOR with zero
        src_a = 32'h12345678; src_b = 32'h00000000; expected = 32'h12345678;
        #1; check_result(expected, "XOR with zero");
        
        // XOR same value
        src_a = 32'h12345678; src_b = 32'h12345678; expected = 32'h00000000;
        #1; check_result(expected, "XOR same");

        // Test SRL function (Shift Right Logical)
        $display("\n--- Testing SRL Function ---");
        func = ALU_SRL;
        
        // Basic right shift
        src_a = 32'h80000000; src_b = 32'h00000001; expected = 32'h40000000;
        #1; check_result(expected, "SRL basic");
        
        // Shift by 0
        src_a = 32'h12345678; src_b = 32'h00000000; expected = 32'h12345678;
        #1; check_result(expected, "SRL by 0");
        
        // Shift by 31
        src_a = 32'h80000000; src_b = 32'h0000001F; expected = 32'h00000001;
        #1; check_result(expected, "SRL by 31");
        
        // Shift by more than 31 (should use only lower 5 bits)
        src_a = 32'h80000000; src_b = 32'h00000021; expected = 32'h40000000;
        #1; check_result(expected, "SRL by 33 (mod 32)");

        // Test SLL function (Shift Left Logical)
        $display("\n--- Testing SLL Function ---");
        func = ALU_SLL;
        
        // Basic left shift
        src_a = 32'h00000001; src_b = 32'h00000001; expected = 32'h00000002;
        #1; check_result(expected, "SLL basic");
        
        // Shift by 0
        src_a = 32'h12345678; src_b = 32'h00000000; expected = 32'h12345678;
        #1; check_result(expected, "SLL by 0");
        
        // Shift by 31
        src_a = 32'h00000001; src_b = 32'h0000001F; expected = 32'h80000000;
        #1; check_result(expected, "SLL by 31");
        
        // Shift by more than 31
        src_a = 32'h00000001; src_b = 32'h00000021; expected = 32'h00000002;
        #1; check_result(expected, "SLL by 33 (mod 32)");

        // Test SRA function (Shift Right Arithmetic)
        $display("\n--- Testing SRA Function ---");
        func = ALU_SRA;
        
        // Arithmetic right shift positive
        src_a = 32'h40000000; src_b = 32'h00000001; expected = 32'h20000000;
        #1; check_result(expected, "SRA positive");
        
        // Arithmetic right shift negative
        src_a = 32'h80000000; src_b = 32'h00000001; expected = 32'hC0000000;
        #1; check_result(expected, "SRA negative");
        
        // Shift by 0
        src_a = 32'h80000000; src_b = 32'h00000000; expected = 32'h80000000;
        #1; check_result(expected, "SRA by 0");
        
        // Shift negative by 31
        src_a = 32'h80000000; src_b = 32'h0000001F; expected = 32'hFFFFFFFF;
        #1; check_result(expected, "SRA negative by 31");

        // Test SLT function (Set Less Than)
        $display("\n--- Testing SLT Function ---");
        func = ALU_SLT;
        
        // Basic less than (positive)
        src_a = 32'h00000001; src_b = 32'h00000002; expected = 32'h00000001;
        #1; check_result(expected, "SLT basic true");
        
        // Basic not less than
        src_a = 32'h00000002; src_b = 32'h00000001; expected = 32'h00000000;
        #1; check_result(expected, "SLT basic false");
        
        // Equal values
        src_a = 32'h00000001; src_b = 32'h00000001; expected = 32'h00000000;
        #1; check_result(expected, "SLT equal");
        
        // Negative vs positive
        src_a = 32'hFFFFFFFF; src_b = 32'h00000001; expected = 32'h00000001;
        #1; check_result(expected, "SLT negative < positive");
        
        // Two negatives
        src_a = 32'hFFFFFFFE; src_b = 32'hFFFFFFFF; expected = 32'h00000001;
        #1; check_result(expected, "SLT two negatives");

        // Test SLTU function (Set Less Than Unsigned)
        $display("\n--- Testing SLTU Function ---");
        func = ALU_SLTU;
        
        // Basic unsigned less than
        src_a = 32'h00000001; src_b = 32'h00000002; expected = 32'h00000001;
        #1; check_result(expected, "SLTU basic true");
        
        // Basic unsigned not less than
        src_a = 32'h00000002; src_b = 32'h00000001; expected = 32'h00000000;
        #1; check_result(expected, "SLTU basic false");
        
        // Unsigned comparison (large numbers)
        src_a = 32'h7FFFFFFF; src_b = 32'h80000000; expected = 32'h00000001;
        #1; check_result(expected, "SLTU unsigned large");
        
        // Unsigned comparison (reverse)
        src_a = 32'h80000000; src_b = 32'h7FFFFFFF; expected = 32'h00000000;
        #1; check_result(expected, "SLTU unsigned large rev");

        // Test LUI function
        $display("\n--- Testing LUI Function ---");
        func = ALU_LUI;
        
        // LUI just copies src_a
        src_a = 32'h12345678; src_b = 32'h87654321; expected = 32'h12345678;
        #1; check_result(expected, "LUI copy src_a");
        
        src_a = 32'h00000000; src_b = 32'hFFFFFFFF; expected = 32'h00000000;
        #1; check_result(expected, "LUI zero");
        
        src_a = 32'hFFFFFFFF; src_b = 32'h00000000; expected = 32'hFFFFFFFF;
        #1; check_result(expected, "LUI all ones");

        // Test default case
        $display("\n--- Testing Default Case ---");
        func = 4'b1111; // Invalid function
        src_a = 32'h12345678; src_b = 32'h87654321; expected = 32'hDEADDEAD;
        #1; check_result(expected, "Default case");

        // Test summary
        $display("\n=== Test Summary ===");
        $display("Total tests: %0d", test_count);
        $display("Passed: %0d", pass_count);
        $display("Failed: %0d", fail_count);
        
        if (fail_count == 0) begin
            $display("ALL TESTS PASSED!");
        end else begin
            $display("SOME TESTS FAILED!");
        end
        
        $display("=== ALU Testbench Completed ===");
        $finish;
    end

endmodule