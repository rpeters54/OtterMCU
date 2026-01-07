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

`include "otter_defines.vh"

module otter_addr_gen (
    input      [1:0]  i_addr_gen_sel,
    input      [31:0] i_rfile_r_rs1,
    input      [31:0] i_immed,
    input      [31:0] i_pc_addr,

    output     [31:0] o_jal_addr,
    output     [31:0] o_jalr_addr,
    output     [31:0] o_branch_addr,
    output reg [31:0] o_dest_addr
);

    // BUGFIX: as per spec: jalr always clears bit zero
    localparam JALR_MASK = 32'hFFFF_FFFE;

    assign o_jal_addr    =  i_immed + i_pc_addr;
    assign o_branch_addr =  i_immed + i_pc_addr;
    assign o_jalr_addr   = (i_immed + i_rfile_r_rs1) & JALR_MASK;

    always @(*) begin
        case(i_addr_gen_sel)
            ADDR_GEN_SEL_JAL    : o_dest_addr = o_jal_addr;
            ADDR_GEN_SEL_JALR   : o_dest_addr = o_jalr_addr;
            ADDR_GEN_SEL_BRANCH : o_dest_addr = o_branch_addr;
            default             : o_dest_addr = 0;
        endcase
    end

endmodule
