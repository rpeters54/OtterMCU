`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/02/2021 01:24:48 PM
// Design Name: 
// Module Name: Mux4_1
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


module Mux4_1 #(
    parameter WIDTH = 4
)(
    input      [WIDTH-1:0] zero,
    input      [WIDTH-1:0] one,
    input      [WIDTH-1:0] two,
    input      [WIDTH-1:0] three,
    input      [1:0]       sel,
    output reg [WIDTH-1:0] mux_out
);
    
    always @(*) begin
        case (sel)
            2'd0:    mux_out = zero;
            2'd1:    mux_out = one;
            2'd2:    mux_out = two;
            2'd3:    mux_out = three;
            default: mux_out = '0;
        endcase
    end
endmodule
