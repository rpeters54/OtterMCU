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
    input      [31:0] src_a, 
    input      [31:0] src_b,
    input      [3:0]  func,
    output reg [31:0] result
);

    always @(*) begin
        case(func)
            ALU_ADD  : result = src_a + src_b;                            // add
            ALU_SUB  : result = src_a - src_b;                            // sub
            ALU_OR   : result = src_a | src_b;                            // or
            ALU_AND  : result = src_a & src_b;                            // and
            ALU_XOR  : result = src_a ^ src_b;                            // xor
            ALU_SRL  : result = src_a >> src_b[4:0];                      // srl
            ALU_SLL  : result = src_a << src_b[4:0];                      // sll
            ALU_SRA  : result = $signed(src_a) >>> src_b[4:0];            // sra
            ALU_SLT  : result = {31'd0, $signed(src_a) < $signed(src_b)}; // slt
            ALU_SLTU : result = {31'd0, src_a < src_b};                   // sltu
            ALU_LUI  : result = src_a;                                    // lui_copy
            default  : result = 32'hDEADDEAD;                             // default
        endcase
    end

endmodule
