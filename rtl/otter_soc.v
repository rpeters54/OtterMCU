`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/04/2022 08:07:08 PM
// Design Name: 
// Module Name: OTTER_MCU
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


`define RISCV_FORMAL
`include "otter_defines.vh"

module otter_soc #(
    parameter ROM_FILE  = "",
    parameter MEM_SIZE  = 2**16,
    parameter RESET_VEC = 0
) (
    input             i_clk,
    input             i_rst, 
    input      [31:0] i_intrpt, 
    input      [31:0] i_iobus_in,

`ifdef RISCV_FORMAL

`define CSR_MACRO_OP(NAME) \
    output [31:0] rvfi_csr_``NAME``_rmask, \
    output [31:0] rvfi_csr_``NAME``_wmask, \
    output [31:0] rvfi_csr_``NAME``_rdata, \
    output [31:0] rvfi_csr_``NAME``_wdata,

    `RVFI_OUTPUTS

`undef CSR_MACRO_OP

`endif

    output reg [31:0] o_iobus_out, 
    output reg [31:0] o_iobus_addr,
    output reg        o_iobus_we
);

    //-------------------------------------------------------------------//
    // MCU: RV32I Hart
    //-------------------------------------------------------------------//

    wire [31:0] w_imem_r_data;
    wire [31:0] w_imem_addr;

    wire        w_dmem_re;
    wire        w_dmem_we;
    wire [3:0]  w_dmem_sel;
    wire [31:0] w_dmem_addr;
    wire [31:0] w_dmem_w_data;
    reg  [31:0] w_dmem_r_data;

    otter_mcu # (
        .RESET_VEC(RESET_VEC)
    ) mcu (
        .i_clk          (i_clk),
        .i_rst          (i_rst),
        .i_intrpt       (i_intrpt),

    `ifdef RISCV_FORMAL

    `define CSR_MACRO_OP(NAME) \
        .rvfi_csr_``NAME``_rmask(rvfi_csr_``NAME``_rmask), \
        .rvfi_csr_``NAME``_wmask(rvfi_csr_``NAME``_wmask), \
        .rvfi_csr_``NAME``_rdata(rvfi_csr_``NAME``_rdata), \
        .rvfi_csr_``NAME``_wdata(rvfi_csr_``NAME``_wdata),

        `RVFI_INTERCONNECTS

    `undef CSR_MACRO_OP

    `endif

        .i_imem_r_data  (w_imem_r_data),
        .o_imem_addr    (w_imem_addr),

        .i_dmem_r_data  (w_dmem_r_data),
        .o_dmem_re      (w_dmem_re),
        .o_dmem_we      (w_dmem_we),
        .o_dmem_sel     (w_dmem_sel),
        .o_dmem_addr    (w_dmem_addr),
        .o_dmem_w_data  (w_dmem_w_data)
    );

    //-------------------------------------------------------------------//
    // Memory: stores all relevant data and instructions for the program
    //-------------------------------------------------------------------//

    reg         w_dram_re;
    reg         w_dram_we;
    reg  [3:0]  w_dram_sel;
    reg  [31:0] w_dram_addr;
    reg  [31:0] w_dram_w_data;
    reg  [31:0] w_dram_r_data;

    otter_mem #(
        .ROM_FILE(ROM_FILE),
        .MEM_SIZE(MEM_SIZE)
    ) u_otter_mem (
        .i_clk          (i_clk),

        .i_imem_addr    (w_imem_addr),
        .o_imem_r_data  (w_imem_r_data),

        .i_dmem_re      (w_dram_re),
        .i_dmem_we      (w_dram_we),
        .i_dmem_sel     (w_dram_sel),
        .i_dmem_addr    (w_dram_addr),
        .i_dmem_w_data  (w_dram_w_data),
        .o_dmem_r_data  (w_dram_r_data)
    );

    // memory mapped io
    always @(*) begin
        if (w_dmem_addr[31]) begin
            w_dram_re     = w_dmem_re;
            w_dram_we     = w_dmem_we;
            w_dram_sel    = w_dmem_sel;
            w_dram_addr   = w_dmem_addr;
            w_dram_w_data = w_dmem_w_data;

            w_dmem_r_data = w_dram_r_data;

            o_iobus_we    = 0;
            o_iobus_addr  = 0;
            o_iobus_out   = 0;
        end else begin
            w_dram_re     = 0;
            w_dram_we     = 0;
            w_dram_sel    = 0;
            w_dram_addr   = 0;
            w_dram_w_data = 0;

            w_dmem_r_data = i_iobus_in;

            o_iobus_we         = w_dmem_we;
            o_iobus_addr       = w_dmem_addr;
            o_iobus_out[ 7: 0] = w_dmem_sel[0] ? w_dmem_w_data[ 7: 0] : 0;
            o_iobus_out[15: 8] = w_dmem_sel[1] ? w_dmem_w_data[15: 8] : 0;
            o_iobus_out[23:16] = w_dmem_sel[2] ? w_dmem_w_data[23:16] : 0;
            o_iobus_out[31:24] = w_dmem_sel[3] ? w_dmem_w_data[31:24] : 0;
        end
    end

endmodule
