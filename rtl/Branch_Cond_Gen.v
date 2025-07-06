`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/30/2022 01:13:08 AM
// Design Name: 
// Module Name: Branch_Cond_Gen
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


module Branch_Cond_Gen (
    input  [31:0] rs1, 
    input  [31:0] rs2,
    output        br_eq,
    output        br_lt, 
    output        br_ltu
);

    //checks is rs1 is equal to or less than rs2 (signed and unsigned) 
    assign br_eq  = rs1 == rs2;
    assign br_lt  = $signed(rs1) < $signed(rs2);
    assign br_ltu = rs1 < rs2;

endmodule
