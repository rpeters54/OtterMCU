`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/21/2022 12:45:31 PM
// Design Name: 
// Module Name: ALU
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

module otter_alu (
    input      [31:0] i_src_a, 
    input      [31:0] i_src_b,
    input      [3:0]  i_func,
    output reg [31:0] o_result
);

    always @(*) begin
        case(i_func)
            ALU_FUNC_SEL_ADD  : o_result = i_src_a + i_src_b;                            // add
            ALU_FUNC_SEL_SUB  : o_result = i_src_a - i_src_b;                            // sub
            ALU_FUNC_SEL_OR   : o_result = i_src_a | i_src_b;                            // or
            ALU_FUNC_SEL_AND  : o_result = i_src_a & i_src_b;                            // and
            ALU_FUNC_SEL_XOR  : o_result = i_src_a ^ i_src_b;                            // xor
            ALU_FUNC_SEL_SRL  : o_result = i_src_a >> i_src_b[4:0];                      // srl
            ALU_FUNC_SEL_SLL  : o_result = i_src_a << i_src_b[4:0];                      // sll
            ALU_FUNC_SEL_SRA  : o_result = $signed(i_src_a) >>> i_src_b[4:0];            // sra
            ALU_FUNC_SEL_SLT  : o_result = {31'd0, $signed(i_src_a) < $signed(i_src_b)}; // slt
            ALU_FUNC_SEL_SLTU : o_result = {31'd0, i_src_a < i_src_b};                   // sltu
            ALU_FUNC_SEL_LUI  : o_result = i_src_b;                                      // lui_copy
            default           : o_result = 0;                                            // default
        endcase
    end

endmodule
