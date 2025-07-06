`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: California Polytechnic University
// Engineer: Riley Peters 
// 
// Create Date: 10/12/2021 11:20:01 AM
// Design Name: 
// Module Name: MUX2_1
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


module Mux2_1 #(
    parameter WIDTH = 4
)(
    input  [WIDTH-1:0] zero,
    input  [WIDTH-1:0] one,
    input              sel,
    output [WIDTH-1:0] mux_out
);
    
    assign mux_out = sel ? one : zero;

endmodule
