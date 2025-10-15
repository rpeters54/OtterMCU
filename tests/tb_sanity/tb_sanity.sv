`timescale 1ns / 1ps

`include "otter_defines.vh"


module tb_sanity;

    reg clk = 0;
    reg rst = 1;

    always begin
        #5 clk = ~clk;
    end

    initial begin
        repeat (5) @(posedge clk);
        rst = 0;
        repeat (10) @(posedge clk);
        $finish();
    end


	wire [31:0] intrpt       = '0;
	wire [31:0] imem_r_data  = 32'h82f7b013;
	wire [31:0] dmem_r_data  = '0;

	wire [31:0] imem_addr;
	wire        dmem_r_en;
	wire        dmem_w_en;
	wire [ 3:0] dmem_w_strb;
	wire [31:0] dmem_addr;
	wire [31:0] dmem_w_data;

    `define RISCV_FORMAL

    /* verilator lint_off PINMISSING */
    otter_mcu # (
        .RESET_VEC(32'h0)
    ) mcu (
        .clk(clk),
        .rst(rst),
        .intrpt(intrpt),

        .imem_r_data(imem_r_data),
        .imem_addr(imem_addr),

        .dmem_r_data(dmem_r_data),
        .dmem_r_en(dmem_r_en),
        .dmem_w_en(dmem_w_en),
        .dmem_w_strb(dmem_w_strb),
        .dmem_addr(dmem_addr),
        .dmem_w_data(dmem_w_data)
    );
    /* verilator lint_on PINMISSING */

    initial begin
        // Name as needed
        $dumpfile("tb_sltiu.vcd");
        $dumpvars(0);
    end


endmodule
