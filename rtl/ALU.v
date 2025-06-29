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


module ALU (
    input      [31:0] src_a, 
    input      [31:0] src_b,
    input      [3:0]  func,
    output reg [31:0] result
);
    
    always @(*) begin
        case(func)
            4'b0000 : result = src_a + src_b;                            // add
            4'b1000 : result = src_a - src_b;                            // sub
            4'b0110 : result = src_a | src_b;                            // or
            4'b0111 : result = src_a & src_b;                            // and
            4'b0100 : result = src_a ^ src_b;                            // xor
            4'b0101 : result = src_a >> src_b[4:0];                      // srl
            4'b0001 : result = src_a << src_b[4:0];                      // sll
            4'b1101 : result = $signed(src_a) >>> src_b[4:0];            // sra
            4'b0010 : result = {31'd0, $signed(src_a) < $signed(src_b)}; // slt
            4'b0011 : result = {31'd0, src_a < src_b};                   // sltu
            4'b1001 : result = src_a;                                    // lui_copy
            default : result = 32'hDEADDEAD;                             // default
        endcase
    end

endmodule
