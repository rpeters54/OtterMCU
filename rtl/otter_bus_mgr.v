
`timescale 1ns / 1ps
`include "otter_defines.vh"

module otter_bus_mgr (
    input      [2:0]  i_funct3,
    input      [14:0] i_op_sel,
    input      [31:0] i_alu_result,
    input      [31:0] i_rfile_r_rs2,
    input      [31:0] i_dmem_r_data,

    output reg [3:0]  o_dmem_sel,
    output reg [31:0] o_dmem_addr,
    output reg [31:0] o_dmem_w_data,
    output reg [31:0] o_dmem_r_data
);

    // Mask and align the memory so values are read properly
    always @(*) begin
        o_dmem_sel    = 0;
        o_dmem_addr   = 0;
        o_dmem_w_data = 0;
        o_dmem_r_data = 0;

        if (i_op_sel[DCDR_OP_LOAD_IDX] || i_op_sel[DCDR_OP_STORE_IDX]) begin
            // present dmem data to the output interface
            o_dmem_sel = 4'b0000;
            case (i_funct3[1:0])
                2'b00   : o_dmem_sel = 4'b0001;
                2'b01   : o_dmem_sel = 4'b0011;
                2'b10   : o_dmem_sel = 4'b1111;
                default : ;
            endcase
            o_dmem_sel <<= i_alu_result[1:0];
            // address should always present as 4-byte aligned
            o_dmem_addr   = {i_alu_result[31:2], 2'd0};
        end

        // align the data and strb so that values are stored properly
        if (i_op_sel[DCDR_OP_STORE_IDX]) begin
            o_dmem_w_data = i_rfile_r_rs2 << {i_alu_result[1:0], 3'd0};
        end

        // align and mask data read from the output
        if (i_op_sel[DCDR_OP_LOAD_IDX]) begin
            o_dmem_r_data = i_dmem_r_data >> {i_alu_result[1:0], 3'd0};
            case (i_funct3)
                FUNCT3_I_LB  : o_dmem_r_data = {{24{o_dmem_r_data[7]}}, o_dmem_r_data[7:0]};
                FUNCT3_I_LH  : o_dmem_r_data = {{16{o_dmem_r_data[15]}}, o_dmem_r_data[15:0]};
                FUNCT3_I_LW  : o_dmem_r_data = o_dmem_r_data;
                FUNCT3_I_LBU : o_dmem_r_data = {24'd0, o_dmem_r_data[7:0]};
                FUNCT3_I_LHU : o_dmem_r_data = {16'd0, o_dmem_r_data[15:0]};
                default      : ;
            endcase
        end
    end

endmodule
