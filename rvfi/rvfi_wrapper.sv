`timescale 1ns / 1ps

`include "otter_defines.vh"

module rvfi_wrapper (
	`RVFI_OUTPUTS
	input         clk,
	input         rst
);

	(* keep *) `rvformal_rand_reg [31:0] intrpt;
	(* keep *) `rvformal_rand_reg [31:0] imem_r_data;
	(* keep *) `rvformal_rand_reg [31:0] dmem_r_data;

	(* keep *) wire [31:0] imem_addr;
	(* keep *) wire        dmem_r_data;
	(* keep *) wire        dmem_w_data;
	(* keep *) wire [ 3:0] dmem_w_strb;
	(* keep *) wire [31:0] dmem_addr;
	(* keep *) wire [31:0] dmem_w_data;

    otter_mcu # (
        .RESET_VEC('0)
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

endmodule
