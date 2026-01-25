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


    `ifdef FORMAL

        reg f_past_valid = 0;
        always @(posedge i_clk) f_past_valid <= 1;

        // read consistency proof
        always @(*) begin
            if (i_r_addr1 == 0) begin
                assert(o_r_rs1 == 0);
            end
            if (i_r_addr2 == 0) begin
                assert(o_r_rs2 == 0);
            end
        end

        // write consistency proof
        (* anyconst *) reg [4:0] f_addr;
        reg        f_shadow_valid;
        reg [31:0] f_shadow_data;

        initial f_shadow_valid = 0;
        always @(posedge i_clk) if (f_past_valid && f_addr != 0) begin
            if (i_w_addr == f_addr && i_w_en) begin
                f_shadow_valid <= 1;
                f_shadow_data  <= i_w_data;
            end
            if (i_r_addr1 == f_addr && f_shadow_valid) begin
                assert(o_r_rs1 == f_shadow_data);
            end
            if (i_r_addr2 == f_addr && f_shadow_valid) begin
                assert(o_r_rs2 == f_shadow_data);
            end
        end

    `endif

endmodule
