`timescale 1ns / 1ps
`include "otter_defines.vh"

module tb_alu;


    reg clk;
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end


    initial begin
        $dumpfile("tb_alu.vcd");
        $dumpvars(0);
    end

   // testbench signals
    reg  [31:0] src_a;
    reg  [31:0] src_b;
    reg  [3:0]  func;
    wire [31:0] result;

    // Test counters
    integer test_count = 0;
    integer pass_count = 0;
    integer fail_count = 0;

    // Instantiate the ALU
    otter_alu dut (
        .i_src_a(src_a),
        .i_src_b(src_b),
        .i_func(func),
        .o_result(result)
    );

    initial begin
        $display("=== ALU Testbench Started ===");
        $display("Time: %0t", $time);

        // --- Testing ADD Function ---
        $display("\n--- Testing ADD Function ---");
        
        drive_dut(ALU_FUNC_SEL_ADD, 32'h12345678, 32'h87654321);
        check_expect(32'h99999999, "ADD basic");
        
        drive_dut(ALU_FUNC_SEL_ADD, 32'h12345678, 32'h00000000);
        check_expect(32'h12345678, "ADD with zero");
        
        drive_dut(ALU_FUNC_SEL_ADD, 32'hFFFFFFFF, 32'h00000001);
        check_expect(32'h00000000, "ADD overflow");
        
        drive_dut(ALU_FUNC_SEL_ADD, 32'h7FFFFFFF, 32'h00000001);
        check_expect(32'h80000000, "ADD max positive");


        // --- Testing SUB Function ---
        $display("\n--- Testing SUB Function ---");
        
        drive_dut(ALU_FUNC_SEL_SUB, 32'h87654321, 32'h12345678);
        check_expect(32'h7530ECA9, "SUB basic");
        
        drive_dut(ALU_FUNC_SEL_SUB, 32'h12345678, 32'h00000000);
        check_expect(32'h12345678, "SUB zero");
        
        drive_dut(ALU_FUNC_SEL_SUB, 32'h12345678, 32'h12345678);
        check_expect(32'h00000000, "SUB same");
        
        drive_dut(ALU_FUNC_SEL_SUB, 32'h12345678, 32'h87654321);
        check_expect(32'h8ACF1357, "SUB underflow");


        // --- Testing OR Function ---
        $display("\n--- Testing OR Function ---");
        
        drive_dut(ALU_FUNC_SEL_OR, 32'h0F0F0F0F, 32'hF0F0F0F0);
        check_expect(32'hFFFFFFFF, "OR basic");
        
        drive_dut(ALU_FUNC_SEL_OR, 32'h12345678, 32'h00000000);
        check_expect(32'h12345678, "OR with zero");
        
        drive_dut(ALU_FUNC_SEL_OR, 32'h12345678, 32'hFFFFFFFF);
        check_expect(32'hFFFFFFFF, "OR with all ones");


        // --- Testing AND Function ---
        $display("\n--- Testing AND Function ---");
        
        drive_dut(ALU_FUNC_SEL_AND, 32'h0F0F0F0F, 32'hF0F0F0F0);
        check_expect(32'h00000000, "AND basic");
        
        drive_dut(ALU_FUNC_SEL_AND, 32'h12345678, 32'h00000000);
        check_expect(32'h00000000, "AND with zero");
        
        drive_dut(ALU_FUNC_SEL_AND, 32'h12345678, 32'hFFFFFFFF);
        check_expect(32'h12345678, "AND with all ones");
        
        drive_dut(ALU_FUNC_SEL_AND, 32'h12345678, 32'h12345678);
        check_expect(32'h12345678, "AND same");


        // --- Testing XOR Function ---
        $display("\n--- Testing XOR Function ---");
        
        drive_dut(ALU_FUNC_SEL_XOR, 32'h0F0F0F0F, 32'hF0F0F0F0);
        check_expect(32'hFFFFFFFF, "XOR basic");
        
        drive_dut(ALU_FUNC_SEL_XOR, 32'h12345678, 32'h00000000);
        check_expect(32'h12345678, "XOR with zero");
        
        drive_dut(ALU_FUNC_SEL_XOR, 32'h12345678, 32'h12345678);
        check_expect(32'h00000000, "XOR same");


        // --- Testing SRL Function (Shift Right Logical) ---
        $display("\n--- Testing SRL Function ---");
        
        drive_dut(ALU_FUNC_SEL_SRL, 32'h80000000, 32'h00000001);
        check_expect(32'h40000000, "SRL basic");
        
        drive_dut(ALU_FUNC_SEL_SRL, 32'h12345678, 32'h00000000);
        check_expect(32'h12345678, "SRL by 0");
        
        drive_dut(ALU_FUNC_SEL_SRL, 32'h80000000, 32'h0000001F);
        check_expect(32'h00000001, "SRL by 31");
        
        drive_dut(ALU_FUNC_SEL_SRL, 32'h80000000, 32'h00000021);
        check_expect(32'h40000000, "SRL by 33 (mod 32)");


        // --- Testing SLL Function (Shift Left Logical) ---
        $display("\n--- Testing SLL Function ---");
        
        drive_dut(ALU_FUNC_SEL_SLL, 32'h00000001, 32'h00000001);
        check_expect(32'h00000002, "SLL basic");
        
        drive_dut(ALU_FUNC_SEL_SLL, 32'h12345678, 32'h00000000);
        check_expect(32'h12345678, "SLL by 0");
        
        drive_dut(ALU_FUNC_SEL_SLL, 32'h00000001, 32'h0000001F);
        check_expect(32'h80000000, "SLL by 31");
        
        drive_dut(ALU_FUNC_SEL_SLL, 32'h00000001, 32'h00000021);
        check_expect(32'h00000002, "SLL by 33 (mod 32)");


        // --- Testing SRA Function (Shift Right Arithmetic) ---
        $display("\n--- Testing SRA Function ---");
        
        drive_dut(ALU_FUNC_SEL_SRA, 32'h40000000, 32'h00000001);
        check_expect(32'h20000000, "SRA positive");
        
        drive_dut(ALU_FUNC_SEL_SRA, 32'h80000000, 32'h00000001);
        check_expect(32'hC0000000, "SRA negative");
        
        drive_dut(ALU_FUNC_SEL_SRA, 32'h80000000, 32'h00000000);
        check_expect(32'h80000000, "SRA by 0");
        
        drive_dut(ALU_FUNC_SEL_SRA, 32'h80000000, 32'h0000001F);
        check_expect(32'hFFFFFFFF, "SRA negative by 31");


        // --- Testing SLT Function (Set Less Than) ---
        $display("\n--- Testing SLT Function ---");
        
        drive_dut(ALU_FUNC_SEL_SLT, 32'h00000001, 32'h00000002);
        check_expect(32'h00000001, "SLT basic true");
        
        drive_dut(ALU_FUNC_SEL_SLT, 32'h00000002, 32'h00000001);
        check_expect(32'h00000000, "SLT basic false");
        
        drive_dut(ALU_FUNC_SEL_SLT, 32'h00000001, 32'h00000001);
        check_expect(32'h00000000, "SLT equal");
        
        drive_dut(ALU_FUNC_SEL_SLT, 32'hFFFFFFFF, 32'h00000001);
        check_expect(32'h00000001, "SLT negative < positive");
        
        drive_dut(ALU_FUNC_SEL_SLT, 32'hFFFFFFFE, 32'hFFFFFFFF);
        check_expect(32'h00000001, "SLT two negatives");


        // --- Testing SLTU Function (Set Less Than Unsigned) ---
        $display("\n--- Testing SLTU Function ---");
        
        drive_dut(ALU_FUNC_SEL_SLTU, 32'h00000001, 32'h00000002);
        check_expect(32'h00000001, "SLTU basic true");
        
        drive_dut(ALU_FUNC_SEL_SLTU, 32'h00000002, 32'h00000001);
        check_expect(32'h00000000, "SLTU basic false");
        
        drive_dut(ALU_FUNC_SEL_SLTU, 32'h7FFFFFFF, 32'h80000000);
        check_expect(32'h00000001, "SLTU unsigned large");
        
        drive_dut(ALU_FUNC_SEL_SLTU, 32'h80000000, 32'h7FFFFFFF);
        check_expect(32'h00000000, "SLTU unsigned large rev");


        // --- Testing LUI Function ---
        $display("\n--- Testing LUI Function ---");
        
        drive_dut(ALU_FUNC_SEL_LUI, 32'h12345678, 32'h87654321);
        check_expect(32'h87654321, "LUI descending digits");
        
        drive_dut(ALU_FUNC_SEL_LUI, 32'h00000000, 32'hFFFFFFFF);
        check_expect(32'hFFFFFFFF, "LUI all ones");
        
        drive_dut(ALU_FUNC_SEL_LUI, 32'hFFFFFFFF, 32'h00000000);
        check_expect(32'h00000000, "LUI zero");

        // --- Test Summary ---
        $display("\n=== Test Summary ===");
        $display("Total tests: %0d", test_count);
        $display("Passed: %0d", pass_count);
        $display("Failed: %0d", fail_count);
        $display("=== ALU Testbench Completed ===");

        if (fail_count == 0) begin
            $display("ALL TESTS PASSED!");
            $finish;
        end else begin
            $error("SOME TESTS FAILED!");
        end
    end

    task drive_dut (
        input [3:0]  test_func,
        input [31:0] test_src_a,
        input [31:0] test_src_b
    );

        /* verilator lint_off INITIALDLY */
        func  <= test_func;
        src_a <= test_src_a;
        src_b <= test_src_b;
        /* verilator lint_on INITIALDLY */
        @(posedge clk);

    endtask

    // Task to check results
    task check_expect(
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
        end
    endtask

endmodule
