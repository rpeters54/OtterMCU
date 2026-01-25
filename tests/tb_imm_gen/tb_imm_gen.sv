`timescale 1ns / 1ps
`include "otter_defines.vh"

module tb_imm_gen;

    reg clk;
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Inputs
    reg  [31:0] instrn;
    reg  [2:0]  imm_sel;

    // Output
    wire [31:0] immed;

    // Instantiate the DUT
    otter_imm_gen dut (
        .i_instrn(instrn),
        .i_imm_sel(imm_sel),
        .o_immed(immed)
    );

    initial begin
        // Name as needed
        $dumpfile("tb_imm_gen.vcd");
        $dumpvars(0);
    end

    initial begin
        $display("--- Starting Immediate Generator Test ---");

        // U-Type Tests
        drive_dut(32'hABCDE537, IMM_GEN_SEL_UPPER);
        check_expect(32'hABCDE000, "LUI Upper Immediate");

        // I-Type Tests
        drive_dut(32'h00558513, IMM_GEN_SEL_I_TYPE);
        check_expect(32'h00000005, "ADDI Positive (5)");
        
        drive_dut(32'hFFB58513, IMM_GEN_SEL_I_TYPE);
        check_expect(32'hFFFFFFFB, "ADDI Negative (-5)");

        // S-Type Tests
        drive_dut(32'h00A5A423, IMM_GEN_SEL_S_TYPE);
        check_expect(32'h00000008, "SW Positive Offset (8)");
        
        drive_dut(32'hFEA5AC23, IMM_GEN_SEL_S_TYPE);
        check_expect(32'hFFFFFFF8, "SW Negative Offset (-8)");

        // B-Type (Branch) Tests
        drive_dut(32'h00208A63, IMM_GEN_SEL_BRANCH);
        check_expect(32'h00000014, "BEQ Positive Offset (+20)");
        
        drive_dut(32'hFEE08663, IMM_GEN_SEL_BRANCH);
        check_expect(32'hFFFFF7EC, "BEQ Negative Offset (-20)");

        // J-Type (Jump) Tests
        drive_dut(32'h020000EF, IMM_GEN_SEL_JUMP);
        check_expect(32'h00000020, "JAL Positive Offset (+32)");
        
        drive_dut(32'hFE0000EF, IMM_GEN_SEL_JUMP);
        check_expect(32'hFFF007E0, "JAL Negative Offset (-32)");

        $display("--- ALL TESTS PASSED ---");
        $finish(0);
    end

    task drive_dut (
        input [31:0] test_instrn,
        input [2:0]  test_imm_sel
    ); 

        /* verilator lint_off INITIALDLY */
        instrn  <= test_instrn;
        imm_sel <= test_imm_sel;
        /* verilator lint_on INITIALDLY */
        @(posedge clk);

    endtask

    task check_expect (
        input [31:0] exp_immed,
        input string test_name
    );
        reg failed;
        begin

            failed = 1'b0;

            if (immed !== exp_immed) begin
                $display("FAIL: %s", test_name);
                $display("        Expected: 0x%h (%d)", exp_immed, $signed(exp_immed));
                $display("        Got:      0x%h (%d)", immed, $signed(immed));
                failed = 1'b1;
            end

            if (failed) begin
                $display("---------------------------------");
                $display("FAILED at time %0t", $time);
                $display("INSTRUCTION: 0x%h", instrn);
                $display("IMM_SEL:     %b", imm_sel);
                $display("---------------------------------");
                $finish(1);
            end
        end
    endtask

endmodule
