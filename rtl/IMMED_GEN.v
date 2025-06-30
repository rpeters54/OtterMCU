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

/*
Uses concatenation to generate immediates
{#{value}} means that value is duplicated # times in the output 
*/
module Immed_Gen (
    input      [31:7] instrn,
    output reg [31:0] upper_immed, 
    output reg [31:0] i_type_immed, 
    output reg [31:0] s_type_immed, 
    output reg [31:0] branch_immed, 
    output reg [31:0] jump_immed
);
    
    always @(*) begin               
        upper_immed  = {instrn[31:12], 12'd0};
        i_type_immed = {{21{instrn[31]}}, instrn[30:20]};
        s_type_immed = {{21{instrn[31]}}, instrn[30:25], instrn[11:7]};
        branch_immed = {{20{instrn[31]}}, instrn[7], instrn[30:25], instrn[11:8], 1'd0};
        jump_immed   = {{12{instrn[31]}}, instrn[19:12], instrn[20], instrn[30:21], 1'd0};
    end

endmodule
