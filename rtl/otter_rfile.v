`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/15/2022 05:43:44 PM
// Design Name: 
// Module Name: Reg_File
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


module otter_rfile (
    input         i_clk,
    input  [4:0]  i_r_addr1, 
    input  [4:0]  i_r_addr2, 
    input         i_w_en,
    input  [4:0]  i_w_addr,
    input  [31:0] i_w_data,
    output [31:0] o_r_rs1, 
    output [31:0] o_r_rs2
);

    // 31, 32-bit registers, x0 is hardwired to zero
    reg  [31:0] w_rfile [1:31]; 

    // registers are intially zeroed for testing
    initial begin
        for (int i = 1; i < 32; i++) begin
            w_rfile[i] = '0;
        end
    end

    // dual read functionality
    // select output register value by address
    assign o_r_rs1 = i_r_addr1 == '0 ? '0 : w_rfile[i_r_addr1];
    assign o_r_rs2 = i_r_addr2 == '0 ? '0 : w_rfile[i_r_addr2]; 

    // single write functionality
    always @(posedge i_clk) begin
        // can not write to register 0
        if (i_w_en && i_w_addr != '0) begin
            w_rfile[i_w_addr] <= i_w_data;
        end
    end

endmodule
