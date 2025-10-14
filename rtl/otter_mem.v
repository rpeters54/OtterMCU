`timescale 1ns / 1ps
module otter_mem #(
    parameter ROM_FILE = "",
    parameter MEM_SIZE = 2**16
) (
    input              clk,

    input  wire [31:0] imem_addr,
    output reg  [31:0] imem_r_data,

    input  wire        dmem_r_en,
    input  wire        dmem_w_en,
    input  wire [3:0]  dmem_w_strb,
    input  wire [31:0] dmem_addr,
    input  wire [31:0] dmem_w_data,
    input  wire [31:0] iobus_in,
    output reg  [31:0] dmem_r_data
);

    localparam MEM_EXP = $clog2(MEM_SIZE);

    reg [31:0] mem [0:MEM_SIZE/4-1];

    initial begin
        if (ROM_FILE != "") begin
            $readmemh(ROM_FILE, mem);
        end
    end

    always @(posedge clk) begin
        imem_r_data <= mem[imem_addr[MEM_EXP-1:2]];
        if (dmem_r_en) begin
            if (dmem_addr < MEM_SIZE) begin
                dmem_r_data <= mem[dmem_addr[MEM_EXP-1:2]];
            end else begin
                dmem_r_data <= iobus_in;
            end
        end
        if (dmem_w_en && dmem_addr < MEM_SIZE) begin
            if (dmem_w_strb[0]) mem[dmem_addr[MEM_EXP-1:2]][ 7: 0] <= dmem_w_data[ 7: 0];
            if (dmem_w_strb[1]) mem[dmem_addr[MEM_EXP-1:2]][15: 8] <= dmem_w_data[15: 8];
            if (dmem_w_strb[2]) mem[dmem_addr[MEM_EXP-1:2]][23:16] <= dmem_w_data[23:16];
            if (dmem_w_strb[3]) mem[dmem_addr[MEM_EXP-1:2]][31:24] <= dmem_w_data[31:24];
        end
    end


endmodule
