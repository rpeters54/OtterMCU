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


module CSR (
    input             clk,
    input             rst, 
    input             int_taken, 
    input             w_en,
    input      [11:0] addr,
    input      [31:0] prog_count, 
    input      [31:0] w_data,
    output reg        csr_mie,
    output reg [31:0] csr_mepc, 
    output reg [31:0] csr_mtvec, 
    output reg [31:0] r_data
);
    
    always @ (posedge clk) begin
	    //reset all registers to zero
        if (rst == 1'b1) begin
            csr_mepc  <= 0; 
            csr_mtvec <= 0; 
            csr_mie   <= 0;
	    //interrupt state
        end else if (int_taken == 1'b1) begin
            csr_mie  <= 0; 
            csr_mepc <= prog_count;
	    //synchronous write (used by csrrw)
        end else if (w_en == 1'b1) begin
            case (addr) 
                12'h304 : csr_mie   <= w_data[0];
                12'h305 : csr_mtvec <= w_data;
                12'h341 : csr_mepc  <= w_data;
            endcase
        end
    end
    
    //asynchronous read
    always @(*) begin
	    //r_data value is based on the addr input
        case (addr)
            12'h304 : r_data = {31'd0, csr_mie};
            12'h305 : r_data = csr_mtvec;
            12'h341 : r_data = csr_mepc;
	        default : r_data = '0;
        endcase
    end

endmodule
