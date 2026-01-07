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

`include "otter_defines.vh"


module otter_addr_check (
    input      [14:0] i_op_sel,
    input      [2:0]  i_funct3,
    input             i_ill_instrn,
    input             i_br_taken,

    input      [31:0] i_instrn,
    input      [31:0] i_pc_addr,
    input      [31:0] i_addr_gen_jal,
    input      [31:0] i_addr_gen_jalr,
    input      [31:0] i_addr_gen_branch,
    input      [31:0] i_alu_result,

    output reg [3:0]  o_excp_sel,
    output reg [31:0] o_trap_mtval
);

    // based on op, check for address misalignment exception
    always @(*) begin
        if (i_ill_instrn) begin
            o_excp_sel = MCAUSE_SEL_INVLD_INSTRN;
        end else begin 
            o_excp_sel = 0;
            case (1'b1)
                i_op_sel[DCDR_OP_JAL_IDX]  : if (i_addr_gen_jal[1:0] != 2'b00) begin
                    o_excp_sel = MCAUSE_SEL_INSTRN_ADDR_MISALIGN;
                end
                i_op_sel[DCDR_OP_JALR_IDX] : if (i_addr_gen_jalr[1:0] != 2'b00) begin
                    o_excp_sel = MCAUSE_SEL_INSTRN_ADDR_MISALIGN;
                end
                i_op_sel[DCDR_OP_BRANCH_IDX] : if (i_br_taken && i_addr_gen_branch[1:0] != 2'b00) begin
                    o_excp_sel = MCAUSE_SEL_INSTRN_ADDR_MISALIGN;
                end
                i_op_sel[DCDR_OP_LOAD_IDX]   : if ((i_funct3 == FUNCT3_I_LW && i_alu_result[1:0] != 2'b00) 
                    || ((i_funct3 == FUNCT3_I_LH || i_funct3 == FUNCT3_I_LHU) && i_alu_result[0] != 1'b0)) begin
                    o_excp_sel = MCAUSE_SEL_LOAD_ADDR_MISALIGN;
                end
                i_op_sel[DCDR_OP_STORE_IDX]  : if ((i_funct3 == FUNCT3_S_SW && i_alu_result[1:0] != 2'b00)
                                                     || (i_funct3 == FUNCT3_S_SH && i_alu_result[0] != 1'b0)) begin
                    o_excp_sel = MCAUSE_SEL_STORE_ADDR_MISALIGN;
                end
                default            : o_excp_sel = 0;
            endcase
        end
    end

    // select mtval next value based on exception type
    always @(*) begin
        o_trap_mtval = 0;
        if (i_ill_instrn) begin
            o_trap_mtval = i_instrn;
        end else if (i_op_sel[DCDR_OP_EBREAK_IDX]) begin
            o_trap_mtval = i_pc_addr;
        end else if (|o_excp_sel) begin
            case (1'b1)
                i_op_sel[DCDR_OP_JAL_IDX]    : o_trap_mtval = i_addr_gen_jal;
                i_op_sel[DCDR_OP_JALR_IDX]   : o_trap_mtval = i_addr_gen_jalr;
                i_op_sel[DCDR_OP_BRANCH_IDX] : o_trap_mtval = i_addr_gen_branch;
                i_op_sel[DCDR_OP_LOAD_IDX]   |
                i_op_sel[DCDR_OP_STORE_IDX]  : o_trap_mtval = i_alu_result;
                default                         : o_trap_mtval = 0;
            endcase
        end
    end



`ifdef FORMAL

    // --- Formal Assertions --- 

    // input must be one hot encoded
    always @(*) begin
        assume ($onehot(i_op_sel));
    end

    // illegal instrn overrides all other exceptions
    always @(*) if (i_ill_instrn) begin
        assert(o_excp_sel   == DCDRCP_SEL_ILLEGAL);
        assert(o_trap_mtval == i_instrn);
    end

    // cases for all possible exceptions and trap values:

    always @(*) if (!i_ill_instrn) begin
        if (i_op_sel[DCDR_OP_JAL_IDX]) begin
            if (|i_addr_gen_jal[1:0]) begin
                assert(o_excp_sel   == DCDRCP_SEL_JUMP);
                assert(o_trap_mtval == i_addr_gen_jal);
            end else begin
                assert(o_excp_sel   == 0);
                assert(o_trap_mtval == 0);
            end
        end
    end

    always @(*) if (!i_ill_instrn) begin
        if (i_op_sel[DCDR_OP_JALR_IDX]) begin
            if (|i_addr_gen_jalr[1:0]) begin
                assert(o_excp_sel   == DCDRCP_SEL_JUMP);
                assert(o_trap_mtval == i_addr_gen_jalr);
            end else begin
                assert(o_excp_sel   == 0);
                assert(o_trap_mtval == 0);
            end
        end
    end


    always @(*) if (!i_ill_instrn) begin
        if (i_op_sel[DCDR_OP_BRANCH_IDX]) begin
            if (i_br_taken && |i_addr_gen_branch[1:0]) begin
                assert(o_excp_sel   == DCDRCP_SEL_JUMP);
                assert(o_trap_mtval == i_addr_gen_branch);
            end else begin
                assert(o_excp_sel   == 0);
                assert(o_trap_mtval == 0);
            end
        end
    end

    always @(*) if (!i_ill_instrn) begin
        if (i_op_sel[DCDR_OP_LOAD_IDX]) begin
            if (
                    (i_funct3 == FUNCT3_I_LW && |i_alu_result[1:0])
                ||  ((i_funct3 == FUNCT3_I_LH || i_funct3 == FUNCT3_I_LHU) && i_alu_result[0])
            ) begin
                assert(o_excp_sel   == DCDRCP_SEL_LOAD);
                assert(o_trap_mtval == i_alu_result);
            end else begin
                assert(o_excp_sel   == 0);
                assert(o_trap_mtval == 0);
            end
        end
    end

    always @(*) if (!i_ill_instrn) begin
        if (i_op_sel[DCDR_OP_STORE_IDX]) begin
            if (
                    (i_funct3 == FUNCT3_S_SW && |i_alu_result[1:0])
                ||  (i_funct3 == FUNCT3_S_SH && i_alu_result[0])
            ) begin
                assert(o_excp_sel   == DCDRCP_SEL_STORE);
                assert(o_trap_mtval == i_alu_result);
            end else begin
                assert(o_excp_sel   == 0);
                assert(o_trap_mtval == 0);
            end
        end
    end

    always @(*) if (!i_ill_instrn) begin
        if (i_op_sel[DCDR_OP_EBREAK_IDX]) begin
            assert(o_excp_sel   == 0);
            assert(o_trap_mtval == i_pc_addr);
        end
    end

`endif

endmodule
