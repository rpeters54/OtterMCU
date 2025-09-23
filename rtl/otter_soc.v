`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/04/2022 08:07:08 PM
// Design Name: 
// Module Name: OTTER_MCU
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

module otter_soc #(
    parameter ROM_FILE = "default.mem"
) (
    input             clk,
    input             rst, 
    input             intrpt, 
    input      [31:0] iobus_in,

`ifdef RISCV_FORMAL
    `RVFI_OUTPUTS
`endif

    output reg [31:0] iobus_out, 
    output reg [31:0] iobus_addr,
    output reg        iobus_wr
);

    //-------------------------------------------------------------------//
    // MCU: RV32I Hart
    //-------------------------------------------------------------------//

    wire        imem_r_en, dmem_r_en, dmem_w_en;
    wire [3:0]  dmem_w_strb;
    wire [31:0] imem_addr, dmem_r_data, dmem_addr, dmem_w_data;
    reg  [31:0] imem_r_data;

    otter_mcu mcu (
        .clk(clk),
        .rst(rst),
        .intrpt(intrpt),

        .imem_r_data(imem_r_data),
        .imem_r_en(imem_r_en),
        .imem_addr(imem_addr),

        .dmem_r_data(dmem_r_data),
        .dmem_r_en(dmem_r_en),
        .dmem_w_en(dmem_w_en),
        .dmem_w_strb(dmem_w_strb),
        .dmem_addr(dmem_addr),
        .dmem_w_data(dmem_w_data)
    );

    //-------------------------------------------------------------------//
    // Memory: stores all relevant data and instructions for the program
    //-------------------------------------------------------------------//

    localparam ADDR_EXP   = 14;
    localparam ADDR_SPACE = 2**ADDR_EXP;

    reg [31:0] imem [0:ADDR_SPACE-1];
    reg [31:0] dmem [0:ADDR_SPACE-1];

    initial begin
        $readmemh(DEFAULT_ROM, imem);
    end

    // internal imem
    always @(posedge clock) begin
        if (imem_r_en) begin
            imem_r_data <= imem[imem_addr[ADDR_EXP+1:2]];
        end
    end

    // internal dmem
    always @(posedge clock) begin
        if (dmem_r_en) begin
            if (dmem_addr[ADDR_EXP+1:2] < ADDR_SPACE) begin
                dmem_r_data <= dmem[dmem_addr[15:2]];
            end else begin
                dmem_r_data <= iobus_in;
            end
        end
        if (dmem_w_en && dmem_addr[ADDR_EXP+1:2] < ADDR_SPACE) begin
            if (dmem_w_strb[0]) dmem[dmem_addr[ADDR_EXP+1:2]][ 7: 0] <= dmem_w_data[ 7: 0];
            if (dmem_w_strb[1]) dmem[dmem_addr[ADDR_EXP+1:2]][15: 8] <= dmem_w_data[15: 8];
            if (dmem_w_strb[2]) dmem[dmem_addr[ADDR_EXP+1:2]][23:16] <= dmem_w_data[23:16];
            if (dmem_w_strb[3]) dmem[dmem_addr[ADDR_EXP+1:2]][31:24] <= dmem_w_data[31:24];
        end
    end

    // memory mapped io
    always @(*) begin
        iobus_out[ 7: 0] = dmem_w_strb[0] ? dmem_w_data[ 7: 0] : 0;
        iobus_out[15: 8] = dmem_w_strb[1] ? dmem_w_data[15: 8] : 0;
        iobus_out[23:16] = dmem_w_strb[2] ? dmem_w_data[23:16] : 0;
        iobus_out[31:24] = dmem_w_strb[3] ? dmem_w_data[31:24] : 0;

        iobus_addr = dmem_addr;
        iobus_wr   = dmem_addr[ADDR_EXP+1:2] < ADDR_SPACE ? dmem_w_en : 0;
    end

endmodule
