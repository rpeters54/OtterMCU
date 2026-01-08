`timescale 1ns / 1ps
module otter_mem #(
    parameter ROM_FILE   = "",
    parameter BRAM_BYTES = 2**16
) (
    input              i_clk,

    input  wire [31:0] i_imem_addr,
    output reg  [31:0] o_imem_r_data,

    input  wire        i_dmem_re,
    input  wire        i_dmem_we,
    input  wire [3:0]  i_dmem_sel,
    input  wire [31:0] i_dmem_addr,
    input  wire [31:0] i_dmem_w_data,
    output reg  [31:0] o_dmem_r_data
);

    localparam MEM_EXP = $clog2(BRAM_BYTES);

    reg [31:0] w_mem [0:BRAM_BYTES/4-1];

    initial begin
        if (ROM_FILE != "") begin
            $readmemh(ROM_FILE, w_mem);
        end
    end

    always @(posedge i_clk) begin
        o_imem_r_data <= w_mem[i_imem_addr[MEM_EXP-1:2]];
        if (i_dmem_re) begin
            o_dmem_r_data <= w_mem[i_dmem_addr[MEM_EXP-1:2]];
        end
        if (i_dmem_we) begin
            if (i_dmem_sel[0]) w_mem[i_dmem_addr[MEM_EXP-1:2]][ 7: 0] <= i_dmem_w_data[ 7: 0];
            if (i_dmem_sel[1]) w_mem[i_dmem_addr[MEM_EXP-1:2]][15: 8] <= i_dmem_w_data[15: 8];
            if (i_dmem_sel[2]) w_mem[i_dmem_addr[MEM_EXP-1:2]][23:16] <= i_dmem_w_data[23:16];
            if (i_dmem_sel[3]) w_mem[i_dmem_addr[MEM_EXP-1:2]][31:24] <= i_dmem_w_data[31:24];
        end
    end


endmodule
