`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/25/2022 04:42:14 PM
// Design Name: 
// Module Name: IMMED_GEN
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

/*
Uses concatenation to generate immediates
{#{value}} means that value is duplicated # times in the output 
*/
module otter_imm_gen (
    input      [31:0] i_instrn,
    input      [2:0]  i_imm_sel,

    output reg [31:0] o_immed
);

    wire [31:0] w_upper_immed  = {i_instrn[31:12], 12'd0};
    wire [31:0] w_i_type_immed = {{21{i_instrn[31]}}, i_instrn[30:20]};
    wire [31:0] w_s_type_immed = {{21{i_instrn[31]}}, i_instrn[30:25], i_instrn[11:7]};
    wire [31:0] w_branch_immed = {{20{i_instrn[31]}}, i_instrn[7], i_instrn[30:25], i_instrn[11:8], 1'd0};
    wire [31:0] w_jump_immed   = {{12{i_instrn[31]}}, i_instrn[19:12], i_instrn[20], i_instrn[30:21], 1'd0};

    always @(*) begin
        case (i_imm_sel)
            IMM_GEN_SEL_UPPER  : o_immed = w_upper_immed;
            IMM_GEN_SEL_I_TYPE : o_immed = w_i_type_immed;
            IMM_GEN_SEL_S_TYPE : o_immed = w_s_type_immed;
            IMM_GEN_SEL_BRANCH : o_immed = w_branch_immed;
            IMM_GEN_SEL_JUMP   : o_immed = w_jump_immed;
            default            : o_immed = 0;
        endcase
    end

endmodule
