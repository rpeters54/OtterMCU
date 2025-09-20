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


module otter_pc (
    input              clk,
    input              rst,
    input              w_en,
    input       [2:0]  src_sel,
    input       [31:0] jalr, 
    input       [31:0] branch, 
    input       [31:0] jal, 
    input       [31:0] mtvec, 
    input       [31:0] mepc,

`ifdef RISCV_FORMAL
    output      [31:0] next_addr,
`endif

    output reg  [31:0] addr,
    output      [31:0] addr_inc
);

    localparam PC_MASK  = 32'hFFFF_FFFC;

    reg [31:0] next_addr;

    assign addr_inc = addr + 'd4;
    always @(posedge clk) begin
        if (rst) begin
            addr <= 0;
        end else if (w_en) begin
            addr <= next_addr & PC_MASK;
        end
    end

    always @(*) begin
        case(src_sel)
            3'd0    : next_addr = addr_inc;
            3'd1    : next_addr = jalr;
            3'd2    : next_addr = branch;
            3'd3    : next_addr = jal;
            3'd4    : next_addr = mtvec;
            3'd5    : next_addr = mepc;
            default : next_addr = 32'hDEADDEAD;
        endcase
    end

endmodule
