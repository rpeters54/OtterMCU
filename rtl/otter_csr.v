`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/25/2022 06:08:24 PM
// Design Name: 
// Module Name: CSR
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

module otter_csr (
    input             clk,
    input             rst, 
    input             intrpt_taken, 
    input             w_en,
    input      [11:0] addr,
    input      [31:0] prog_count, 
    input      [31:0] w_data,
    output reg        csr_mie,
    output reg [31:0] csr_mepc, 
    output reg [31:0] csr_mtvec, 
    output reg [31:0] r_data
);

    always @(posedge clk) begin
        //reset all registers to zero
        if (rst == '1) begin
            csr_mepc  <= '0; 
            csr_mtvec <= '0; 
            csr_mie   <= '0;
        //interrupt state
        end else if (intrpt_taken == '1) begin
            csr_mie  <= '0; 
            csr_mepc <= prog_count;
        //synchronous write (used by csrrw)
        end else if (w_en == '1) begin
            case (addr) 
                CSR_MIE_ADDR   : csr_mie   <= w_data[0];
                CSR_MTVEC_ADDR : csr_mtvec <= w_data;
                CSR_MEPC_ADDR  : csr_mepc  <= w_data;
                default        : begin end
            endcase
        end
    end

    //asynchronous read
    always @(*) begin
        //r_data value is based on the addr input
        case (addr)
            CSR_MIE_ADDR   : r_data = {31'd0, csr_mie};
            CSR_MTVEC_ADDR : r_data = csr_mtvec;
            CSR_MEPC_ADDR  : r_data = csr_mepc;
            default        : r_data = '0;
        endcase
    end

endmodule
