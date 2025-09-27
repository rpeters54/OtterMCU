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

module otter_pc (
    input             clk,
    input             rst,
    input             w_en,
    input      [31:0] next_addr,

    output reg [31:0] addr,
    output     [31:0] addr_inc
);

    assign addr_inc = addr + 'd4;

    always @(posedge clk) begin
        if (rst) begin
            addr <= 0;
        end else if (w_en) begin
            addr <= next_addr & 32'hFFFF_FFFC;
        end
    end

endmodule
