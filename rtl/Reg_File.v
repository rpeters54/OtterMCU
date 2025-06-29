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


module Reg_File (
    input             clk,
    input      [4:0]  r_addr1, 
    input      [4:0]  r_addr2, 
    input             w_en,
    input      [4:0]  w_addr,
    input      [31:0] w_data,
    output reg [31:0] r_rs1, 
    output reg [31:0] r_rs2
);
    
    // 32, 32-bit registers
    reg [31:0] rfile [0:31]; 
    
    // registers are intially zeroed for testing
    // r0 is zero forever
    initial begin
        for (int i = 0; i < 32; i++) begin
            rfile[i] <= 32'd0;
        end
    end
    
    // dual read functionality
    always @(*) begin
	    // select output register value by address
        r_rs1 = rfile[r_addr1];
        r_rs2 = rfile[r_addr2]; 
    end
    
    // single write functionality
    always @(posedge clk) begin
	    // can not write to register 0
        if (w_en && w_addr != 0) begin
            rfile[w_addr] <= w_data;
        end
    end
    
endmodule
