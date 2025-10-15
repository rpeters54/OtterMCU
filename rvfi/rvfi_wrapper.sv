`timescale 1ns / 1ps

`include "otter_defines.vh"

`define CSR_MACRO_OP(NAME) \
    output [31:0] rvfi_csr_``NAME``_rmask, \
    output [31:0] rvfi_csr_``NAME``_wmask, \
    output [31:0] rvfi_csr_``NAME``_rdata, \
    output [31:0] rvfi_csr_``NAME``_wdata,

module rvfi_wrapper (
	`RVFI_OUTPUTS
	input         clock,
	input         reset
);

`undef CSR_MACRO_OP

	(* keep *) `rvformal_rand_reg [31:0] intrpt;
	(* keep *) `rvformal_rand_reg [31:0] imem_r_data;
	(* keep *) `rvformal_rand_reg [31:0] dmem_r_data;

	(* keep *) wire [31:0] imem_addr;
	(* keep *) wire        dmem_r_en;
	(* keep *) wire        dmem_w_en;
	(* keep *) wire [ 3:0] dmem_w_strb;
	(* keep *) wire [31:0] dmem_addr;
	(* keep *) wire [31:0] dmem_w_data;


`define CSR_MACRO_OP(NAME) \
    .rvfi_csr_``NAME``_rmask(rvfi_csr_``NAME``_rmask), \
    .rvfi_csr_``NAME``_wmask(rvfi_csr_``NAME``_wmask), \
    .rvfi_csr_``NAME``_rdata(rvfi_csr_``NAME``_rdata), \
    .rvfi_csr_``NAME``_wdata(rvfi_csr_``NAME``_wdata),

    otter_mcu # (
        .RESET_VEC('0)
    ) mcu (
        .clk(clock),
        .rst(reset),
        .intrpt(intrpt),

        `RVFI_INTERCONNECTS

        .imem_r_data(imem_r_data),
        .imem_addr(imem_addr),

        .dmem_r_data(dmem_r_data),
        .dmem_r_en(dmem_r_en),
        .dmem_w_en(dmem_w_en),
        .dmem_w_strb(dmem_w_strb),
        .dmem_addr(dmem_addr),
        .dmem_w_data(dmem_w_data)
    );

`undef CSR_MACRO_OP

endmodule
