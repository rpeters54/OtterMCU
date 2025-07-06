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


module PC (
    input              clk,
    input              rst,
    input              w_en,
    input       [2:0]  src_sel,
    input       [31:0] jalr, 
    input       [31:0] branch, 
    input       [31:0] jal, 
    input       [31:0] mtvec, 
    input       [31:0] mepc,

    output             pc_misalign,
    output reg  [31:0] addr,
    output      [31:0] next_addr
);
    
    localparam PC_MASK  = 32'hFFFF_FFFC;
    localparam LSB_MASK = 32'h0000_0003;

    reg [31:0] data_in;
    
    assign next_addr   = addr + 'd4;
    assign pc_misalign = |(data_in & LSB_MASK);
    
    always @(posedge clk) begin
        if (rst) begin
            addr <= 0;
        end else if (w_en) begin
            addr <= data_in & PC_MASK;
        end
    end

    always @(*) begin
        case(src_sel)
            3'd0    : data_in = next_addr;
            3'd1    : data_in = jalr;
            3'd2    : data_in = branch;
            3'd3    : data_in = jal;
            3'd4    : data_in = mtvec;
            3'd5    : data_in = mepc;
            default : data_in = 32'hDEADDEAD;
        endcase
    end

endmodule
