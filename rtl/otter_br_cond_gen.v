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

`include "otter_defines.vh"

module otter_br_cond_gen (
    input      [2:0]  i_funct3,
    input      [31:0] i_rfile_r_rs1, 
    input      [31:0] i_rfile_r_rs2,
    output reg        o_br_taken
);

    wire br_eq  = i_rfile_r_rs1 == i_rfile_r_rs2;
    wire br_lt  = $signed(i_rfile_r_rs1) < $signed(i_rfile_r_rs2);
    wire br_ltu = i_rfile_r_rs1 < i_rfile_r_rs2;

    always @(*) begin
        case (i_funct3) 
            FUNCT3_B_BEQ  : o_br_taken =  br_eq;
            FUNCT3_B_BNE  : o_br_taken = !br_eq;
            FUNCT3_B_BLT  : o_br_taken =  br_lt;
            FUNCT3_B_BGE  : o_br_taken = !br_lt;
            FUNCT3_B_BLTU : o_br_taken =  br_ltu;
            FUNCT3_B_BGEU : o_br_taken = !br_ltu;
            default       : o_br_taken = 0;
        endcase
    end

endmodule
