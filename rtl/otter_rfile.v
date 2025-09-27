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
    input         clk,
    input  [4:0]  r_addr1, 
    input  [4:0]  r_addr2, 
    input         w_en,
    input  [4:0]  w_addr,
    input  [31:0] w_data,
    output [31:0] r_rs1, 
    output [31:0] r_rs2
);

    // 31, 32-bit registers, x0 is hardwired to zero
    reg  [31:0] rfile [1:31]; 

    // registers are intially zeroed for testing
    initial begin
        for (int i = 1; i < 32; i++) begin
            rfile[i] = '0;
        end
    end

    // dual read functionality
    // select output register value by address
    assign r_rs1 = r_addr1 == '0 ? '0 : rfile[r_addr1];
    assign r_rs2 = r_addr2 == '0 ? '0 : rfile[r_addr2]; 

    // single write functionality
    always @(posedge clk) begin
        // can not write to register 0
        if (w_en && w_addr != '0) begin
            rfile[w_addr] <= w_data;
        end
    end


endmodule
