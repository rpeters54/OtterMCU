`timescale 1ns / 1ps

`include "otter_defines.vh"


module tb_sanity;

    reg w_clk = 0;
    reg w_rst = 1;

    always begin
        #5 w_clk = ~w_clk;
    end

    initial begin
        repeat (5) @(posedge w_clk);
        w_rst = 0;
        repeat (10) @(posedge w_clk);
        $finish();
    end


	wire [31:0] w_intrpt       = '0;

    wire [31:0] w_imem_r_data = 32'h82f7b013;
    wire [31:0] w_imem_addr;

    wire        w_dmem_re;
    wire        w_dmem_we;
    wire [3:0]  w_dmem_sel;
    wire [31:0] w_dmem_addr;
    wire [31:0] w_dmem_w_data;
    wire [31:0] w_dmem_r_data = 0;

    `define RISCV_FORMAL

    /* verilator lint_off PINMISSING */
    otter_mcu # (
        .RESET_VEC(32'h0)
    ) mcu (
        .i_clk          (w_clk),
        .i_rst          (w_rst),
        .i_intrpt       (w_intrpt),

        .i_imem_r_data  (w_imem_r_data),
        .o_imem_addr    (w_imem_addr),

        .i_dmem_r_data  (w_dmem_r_data),
        .o_dmem_re      (w_dmem_re),
        .o_dmem_we      (w_dmem_we),
        .o_dmem_sel     (w_dmem_sel),
        .o_dmem_addr    (w_dmem_addr),
        .o_dmem_w_data  (w_dmem_w_data)
    );
    /* verilator lint_on PINMISSING */

    initial begin
        // Name as needed
        $dumpfile("tb_sltiu.vcd");
        $dumpvars(0);
    end


endmodule
