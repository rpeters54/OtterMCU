`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/30/2022 01:04:21 AM
// Design Name: 
// Module Name: Branch_Addr_Gen
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


module otter_br_addr_gen (
    input  [31:0] rs1, 
    input  [31:0] i_type_immed, 
    input  [31:0] branch_immed, 
    input  [31:0] jump_immed, 
    input  [31:0] prog_count,
    output [31:0] jal_addr, 
    output [31:0] branch_addr, 
    output [31:0] jalr_addr
);
    
    assign jal_addr    = jump_immed   + prog_count;
    assign branch_addr = branch_immed + prog_count;
    assign jalr_addr   = i_type_immed + rs1;

endmodule
