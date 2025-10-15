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

`define CSR_MACRO_OP(NAME) \
    output [31:0] rvfi_csr_``NAME``_rmask, \
    output [31:0] rvfi_csr_``NAME``_wmask, \
    output [31:0] rvfi_csr_``NAME``_rdata, \
    output [31:0] rvfi_csr_``NAME``_wdata,

module otter_soc #(
    parameter ROM_FILE  = "",
    parameter MEM_SIZE  = 2**16,
    parameter RESET_VEC = 0
) (
    input             clk,
    input             rst, 
    input      [31:0] intrpt, 
    input      [31:0] iobus_in,

`ifdef RISCV_FORMAL
    `RVFI_OUTPUTS
`endif

    output reg [31:0] iobus_out, 
    output reg [31:0] iobus_addr,
    output reg        iobus_wr
);

`undef CSR_MACRO_OP

    //-------------------------------------------------------------------//
    // MCU: RV32I Hart
    //-------------------------------------------------------------------//

    wire        dmem_r_en, dmem_w_en;
    wire [3:0]  dmem_w_strb;
    wire [31:0] imem_addr, dmem_addr, dmem_w_data;
    reg  [31:0] imem_r_data, dmem_r_data;

    otter_mcu # (
        .RESET_VEC(RESET_VEC)
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

    //-------------------------------------------------------------------//
    // Memory: stores all relevant data and instructions for the program
    //-------------------------------------------------------------------//

    otter_mem #(
        .ROM_FILE(ROM_FILE),
        .MEM_SIZE(MEM_SIZE)
    ) mem (
        .clk(clk),

        .imem_addr(imem_addr),
        .imem_r_data(imem_r_data),

        .dmem_r_en(dmem_r_en),
        .dmem_w_en(dmem_w_en),
        .dmem_w_strb(dmem_w_strb),
        .dmem_addr(dmem_addr),
        .dmem_w_data(dmem_w_data),
        .iobus_in(iobus_in),
        .dmem_r_data(dmem_r_data)
    );


    // memory mapped io
    always @(*) begin
        iobus_out[ 7: 0] = dmem_w_strb[0] ? dmem_w_data[ 7: 0] : 0;
        iobus_out[15: 8] = dmem_w_strb[1] ? dmem_w_data[15: 8] : 0;
        iobus_out[23:16] = dmem_w_strb[2] ? dmem_w_data[23:16] : 0;
        iobus_out[31:24] = dmem_w_strb[3] ? dmem_w_data[31:24] : 0;

        iobus_addr = dmem_addr;
        iobus_wr   = dmem_addr < MEM_SIZE ? 0 : dmem_w_en;
    end

endmodule
