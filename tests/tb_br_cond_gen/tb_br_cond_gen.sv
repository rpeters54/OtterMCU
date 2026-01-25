`timescale 1ns / 1ps
`include "otter_defines.vh"

module tb_br_cond_gen;

    // Clock generation
    reg clk;
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    initial begin
        // Name as needed
        $dumpfile("tb_br_cond_gen.vcd");
        $dumpvars(0);
    end

    reg [2:0]  funct3;
    reg [31:0] rfile_r_rs1; 
    reg [31:0] rfile_r_rs2;
    wire       br_taken;

    otter_br_cond_gen bcg (
        .i_funct3(funct3),
        .i_rfile_r_rs1(rfile_r_rs1),
        .i_rfile_r_rs2(rfile_r_rs2),
        .o_br_taken(br_taken)
    );

    initial begin
        $display("--- Starting Branch Condition Generator Test ---");

        // --- BEQ Tests ---
        drive_dut(FUNCT3_B_BEQ, 32'd5, 32'd5);
        check_expect(1'b1, "BEQ Equal (5==5)");

        drive_dut(FUNCT3_B_BEQ, 32'd5, 32'd6);
        check_expect(1'b0, "BEQ Not Equal (5!=6)");

        drive_dut(FUNCT3_B_BEQ, 32'd0, 32'd0);
        check_expect(1'b1, "BEQ Zero (0==0)");

        drive_dut(FUNCT3_B_BEQ, 32'hFFFFFFFF, 32'hFFFFFFFF);
        check_expect(1'b1, "BEQ Neg Equal (-1==-1)");

        drive_dut(FUNCT3_B_BEQ, 32'hFFFFFFFF, 32'd0);
        check_expect(1'b0, "BEQ Neg Not Equal (-1!=0)");

        // --- BNE Tests ---
        drive_dut(FUNCT3_B_BNE, 32'd5, 32'd5);
        check_expect(1'b0, "BNE Equal (5==5)");

        drive_dut(FUNCT3_B_BNE, 32'd5, 32'd6);
        check_expect(1'b1, "BNE Not Equal (5!=6)");

        drive_dut(FUNCT3_B_BNE, 32'd0, 32'd0);
        check_expect(1'b0, "BNE Zero (0==0)");

        drive_dut(FUNCT3_B_BNE, 32'hFFFFFFFF, 32'hFFFFFFFF);
        check_expect(1'b0, "BNE Neg Equal (-1==-1)");

        drive_dut(FUNCT3_B_BNE, 32'hFFFFFFFF, 32'd0);
        check_expect(1'b1, "BNE Neg Not Equal (-1!=0)");

        // --- BLT Tests (Signed) ---
        drive_dut(FUNCT3_B_BLT, 32'd5, 32'd6);
        check_expect(1'b1, "BLT Positive Less (5 < 6)");

        drive_dut(FUNCT3_B_BLT, 32'd5, 32'd5);
        check_expect(1'b0, "BLT Equal (5 < 5)");

        drive_dut(FUNCT3_B_BLT, 32'd6, 32'd5);
        check_expect(1'b0, "BLT Greater (6 < 5)");

        drive_dut(FUNCT3_B_BLT, -32'sd5, 32'sd5);
        check_expect(1'b1, "BLT Neg < Pos (-5 < 5)");

        drive_dut(FUNCT3_B_BLT, 32'sd5, -32'sd5);
        check_expect(1'b0, "BLT Pos < Neg (5 < -5)");

        drive_dut(FUNCT3_B_BLT, -32'sd5, -32'sd6);
        check_expect(1'b0, "BLT Neg Greater (-5 < -6)");

        drive_dut(FUNCT3_B_BLT, -32'sd6, -32'sd5);
        check_expect(1'b1, "BLT Neg Less (-6 < -5)");

        // --- BGE Tests (Signed) ---
        drive_dut(FUNCT3_B_BGE, 32'd5, 32'd6);
        check_expect(1'b0, "BGE Less (5 >= 6)");

        drive_dut(FUNCT3_B_BGE, 32'd5, 32'd5);
        check_expect(1'b1, "BGE Equal (5 >= 5)");

        drive_dut(FUNCT3_B_BGE, 32'd6, 32'd5);
        check_expect(1'b1, "BGE Greater (6 >= 5)");

        drive_dut(FUNCT3_B_BGE, -32'sd5, 32'sd5);
        check_expect(1'b0, "BGE Neg >= Pos (-5 >= 5)");

        drive_dut(FUNCT3_B_BGE, 32'sd5, -32'sd5);
        check_expect(1'b1, "BGE Pos >= Neg (5 >= -5)");

        drive_dut(FUNCT3_B_BGE, -32'sd5, -32'sd6);
        check_expect(1'b1, "BGE Neg Greater (-5 >= -6)");

        drive_dut(FUNCT3_B_BGE, -32'sd6, -32'sd5);
        check_expect(1'b0, "BGE Neg Less (-6 >= -5)");

        // --- BLTU Tests (Unsigned) ---
        drive_dut(FUNCT3_B_BLTU, 32'd5, 32'd6);
        check_expect(1'b1, "BLTU Basic Less (5 < 6)");

        drive_dut(FUNCT3_B_BLTU, 32'd5, 32'd5);
        check_expect(1'b0, "BLTU Equal (5 < 5)");

        drive_dut(FUNCT3_B_BLTU, 32'd6, 32'd5);
        check_expect(1'b0, "BLTU Greater (6 < 5)");

        drive_dut(FUNCT3_B_BLTU, 32'hFFFFFFFF, 32'd5);
        check_expect(1'b0, "BLTU Max Unsigned vs 5");

        drive_dut(FUNCT3_B_BLTU, 32'd5, 32'hFFFFFFFF);
        check_expect(1'b1, "BLTU 5 vs Max Unsigned");

        drive_dut(FUNCT3_B_BLTU, 32'h80000000, 32'd0);
        check_expect(1'b0, "BLTU MSB Set vs 0");

        drive_dut(FUNCT3_B_BLTU, 32'd0, 32'h80000000);
        check_expect(1'b1, "BLTU 0 vs MSB Set");

        // --- BGEU Tests (Unsigned) ---
        drive_dut(FUNCT3_B_BGEU, 32'd5, 32'd6);
        check_expect(1'b0, "BGEU Less (5 >= 6)");

        drive_dut(FUNCT3_B_BGEU, 32'd5, 32'd5);
        check_expect(1'b1, "BGEU Equal (5 >= 5)");

        drive_dut(FUNCT3_B_BGEU, 32'd6, 32'd5);
        check_expect(1'b1, "BGEU Greater (6 >= 5)");

        drive_dut(FUNCT3_B_BGEU, 32'hFFFFFFFF, 32'd5);
        check_expect(1'b1, "BGEU Max Unsigned vs 5");

        drive_dut(FUNCT3_B_BGEU, 32'd5, 32'hFFFFFFFF);
        check_expect(1'b0, "BGEU 5 vs Max Unsigned");

        drive_dut(FUNCT3_B_BGEU, 32'h80000000, 32'd0);
        check_expect(1'b1, "BGEU MSB Set vs 0");

        drive_dut(FUNCT3_B_BGEU, 32'd0, 32'h80000000);
        check_expect(1'b0, "BGEU 0 vs MSB Set");

        // --- Invalid Case ---
        drive_dut(3'b010, 32'd5, 32'd5);
        check_expect(1'b0, "Default/Invalid Funct3");

        $display("--- ALL TESTS PASSED ---");
        $finish();
    end

    task drive_dut (
        input [2:0]  test_funct3,
        input [31:0] test_rfile_r_rs1, 
        input [31:0] test_rfile_r_rs2
    );

        /* verilator lint_off INITIALDLY */
        funct3      <= test_funct3;
        rfile_r_rs1 <= test_rfile_r_rs1;
        rfile_r_rs2 <= test_rfile_r_rs2;
        /* verilator lint_on INITIALDLY */
        @(posedge clk);

    endtask

    task check_expect (
        input        exp_br_taken,
        input string test_name
    );
        begin
            if (br_taken !== exp_br_taken) begin
                $display("FAIL: %s", test_name);
                $display("        Expected: %b", exp_br_taken);
                $display("        Got:      %b", br_taken);
                $display("---------------------------------");
                $display("FAILED at time %0t", $time);
                $display("FUNCT3: 0x%h", funct3);
                $display("RS1:    0x%h", rfile_r_rs1);
                $display("RS2:    0x%h", rfile_r_rs2);
                $display("---------------------------------");
                $error;
            end else begin
                $display("PASS: %s", test_name);
            end
        end
    endtask

endmodule
