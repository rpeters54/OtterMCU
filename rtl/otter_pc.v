`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/10/2022 02:27:31 PM
// Design Name: 
// Module Name: PC
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

`include "otter_defines.vh"

module otter_pc #(
    parameter RESET_VEC = 32'h0
) (
    input             i_clk,
    input             i_stall,
    input      [1:0]  i_pc_src_sel,
    input      [31:0] i_br_tgt_addr,
    input      [31:0] i_epc_addr,

    output reg [31:0] o_pc_addr,
    output reg [31:0] o_pc_next_addr
);

    // addresses should always be word-aligned
    localparam PC_ADDR_MASK = 32'hFFFF_FFFC;

    always @(*) begin
        case(i_pc_src_sel)
            PC_SRC_SEL_ADDR_INC  : o_pc_next_addr = o_pc_addr + 4;
            PC_SRC_SEL_BR_JUMP   : o_pc_next_addr = i_br_tgt_addr;
            PC_SRC_SEL_EPC       : o_pc_next_addr = i_epc_addr;
            PC_SRC_SEL_RESET_VEC : o_pc_next_addr = RESET_VEC;
        endcase
    end

    // pc register
    always @(posedge i_clk) begin
        if (!i_stall) begin
            o_pc_addr <= o_pc_next_addr & PC_ADDR_MASK;
        end
    end

endmodule
