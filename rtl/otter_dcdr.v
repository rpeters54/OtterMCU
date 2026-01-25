`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/02/2022 03:54:39 PM
// Desmode 
// Module Name: CU_DCDR
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

module otter_dcdr (
    input      [31:0] i_instrn,

    // selectors
    output reg [3:0]  o_alu_func_sel,
    output reg        o_alu_src_a_sel,
    output reg        o_alu_src_b_sel,
    output reg [1:0]  o_rfile_w_sel,
    output reg [2:0]  o_imm_gen_sel,
    output reg [1:0]  o_addr_gen_sel,
    output reg [14:0] o_op_sel,
    output reg        o_ill_instrn,

    // enablers
    output reg        o_rfile_we,
    output reg        o_dmem_re,
    output reg        o_dmem_we,
    output reg        o_csr_we
);

    reg w_csr_write_attempt, w_csr_addr_illegal, w_csr_is_readonly;

    // decompose instruction into component parts
    wire [6:0] w_funct7    = `INSTRN_FUNCT7(i_instrn);
    wire [4:0] w_rs2_addr  = `INSTRN_RS2_ADDR(i_instrn);
    wire [4:0] w_rs1_addr  = `INSTRN_RS1_ADDR(i_instrn);
    wire [2:0] w_funct3    = `INSTRN_FUNCT3(i_instrn);
    wire [4:0] w_rd_addr   = `INSTRN_RD_ADDR(i_instrn);
    wire [6:0] w_opcode    = `INSTRN_OPCODE(i_instrn);

    // used for the fm field of the fence instruction
    wire [3:0]  w_fm_fence = `INSTRN_FM_FENCE(i_instrn);
    wire [11:0] w_csr_addr = `INSTRN_CSR_ADDR(i_instrn);

    always @(*) begin
        // selector defaults
        o_alu_func_sel  = 0; 
        o_alu_src_a_sel = 0; 
        o_alu_src_b_sel = 0;
        o_rfile_w_sel   = 0; 
        o_imm_gen_sel   = 0;
        o_addr_gen_sel  = 0;
        o_op_sel        = 0;
        o_ill_instrn    = 0;

        // enabler defaults
        o_rfile_we   = 0;
        o_dmem_re    = 0; 
        o_dmem_we    = 0; 
        o_csr_we     = 0; 


        case(w_opcode)
            OPCODE_OP_REG : begin    // R-Type opcode
                o_alu_src_a_sel = ALU_SRC_A_SEL_RS1;
                o_alu_src_b_sel = ALU_SRC_B_SEL_RS2;
                o_rfile_w_sel   = RFILE_W_SEL_ALU_RESULT;
                o_op_sel        = DCDR_OP_SEL_REG;

                o_rfile_we      = 1;
                case ({w_funct7, w_funct3})  // arithmetic/logical operations including two registers
                    {FUNCT7_I_R_0, FUNCT3_R_ADD}  : o_alu_func_sel = ALU_FUNC_SEL_ADD;
                    {FUNCT7_I_R_1, FUNCT3_R_SUB}  : o_alu_func_sel = ALU_FUNC_SEL_SUB;
                    {FUNCT7_I_R_0, FUNCT3_R_SLL}  : o_alu_func_sel = ALU_FUNC_SEL_SLL;
                    {FUNCT7_I_R_0, FUNCT3_R_SLT}  : o_alu_func_sel = ALU_FUNC_SEL_SLT;
                    {FUNCT7_I_R_0, FUNCT3_R_SLTU} : o_alu_func_sel = ALU_FUNC_SEL_SLTU;
                    {FUNCT7_I_R_0, FUNCT3_R_XOR}  : o_alu_func_sel = ALU_FUNC_SEL_XOR;
                    {FUNCT7_I_R_0, FUNCT3_R_SRL}  : o_alu_func_sel = ALU_FUNC_SEL_SRL;
                    {FUNCT7_I_R_1, FUNCT3_R_SRA}  : o_alu_func_sel = ALU_FUNC_SEL_SRA;
                    {FUNCT7_I_R_0, FUNCT3_R_OR}   : o_alu_func_sel = ALU_FUNC_SEL_OR;
                    {FUNCT7_I_R_0, FUNCT3_R_AND}  : o_alu_func_sel = ALU_FUNC_SEL_AND;
                    default                       : o_ill_instrn   = 1;
                endcase
            end
            OPCODE_OP_IMM : begin // I-Type opcode *no loading
                o_alu_src_a_sel = ALU_SRC_A_SEL_RS1; 
                o_alu_src_b_sel = ALU_SRC_B_SEL_IMM;
                o_rfile_w_sel   = RFILE_W_SEL_ALU_RESULT;
                o_imm_gen_sel   = IMM_GEN_SEL_I_TYPE;
                o_op_sel        = DCDR_OP_SEL_IMM;

                o_rfile_we      = 1;
                case (w_funct3)
                    FUNCT3_I_ADDI  : o_alu_func_sel = ALU_FUNC_SEL_ADD;
                    FUNCT3_I_SLTI  : o_alu_func_sel = ALU_FUNC_SEL_SLT;
                    FUNCT3_I_SLTIU : o_alu_func_sel = ALU_FUNC_SEL_SLTU;
                    FUNCT3_I_ORI   : o_alu_func_sel = ALU_FUNC_SEL_OR;
                    FUNCT3_I_XORI  : o_alu_func_sel = ALU_FUNC_SEL_XOR;
                    FUNCT3_I_ANDI  : o_alu_func_sel = ALU_FUNC_SEL_AND;
                    FUNCT3_I_SLI   : begin
                        if (w_funct7 == FUNCT7_I_R_0) begin
                            o_alu_func_sel = ALU_FUNC_SEL_SLL;
                        end else begin
                            o_ill_instrn = 1;
                        end
                    end
                    FUNCT3_I_SRI   : begin
                        if (w_funct7 == FUNCT7_I_R_0) begin
                            o_alu_func_sel = ALU_FUNC_SEL_SRL;
                        end else if (w_funct7 == FUNCT7_I_R_1) begin
                            o_alu_func_sel = ALU_FUNC_SEL_SRA;
                        end else begin
                            o_ill_instrn = 1;
                        end
                    end
                    default       : begin 
                        o_ill_instrn = 1;
                    end
                endcase
            end
            OPCODE_JALR : begin    // I-Type opcode *jalr
                case (w_funct3) 
                    FUNCT3_I_JALR : begin
                        o_rfile_w_sel  = RFILE_W_SEL_PC_ADDR_INC;
                        o_imm_gen_sel  = IMM_GEN_SEL_I_TYPE;
                        o_addr_gen_sel = ADDR_GEN_SEL_JALR;
                        o_op_sel       = DCDR_OP_SEL_JALR; 

                        o_rfile_we     = 1;
                    end 
                    default : begin
                        o_ill_instrn = 1;
                    end
                endcase
            end
            OPCODE_LOAD : begin      // I-Type opcode *load instructions
                case (w_funct3)
                    FUNCT3_I_LB, FUNCT3_I_LH, FUNCT3_I_LW,
                    FUNCT3_I_LBU, FUNCT3_I_LHU : begin
                        o_alu_src_a_sel = ALU_SRC_A_SEL_RS1;
                        o_alu_src_b_sel = ALU_SRC_B_SEL_IMM;
                        o_alu_func_sel  = ALU_FUNC_SEL_ADD;
                        o_imm_gen_sel   = IMM_GEN_SEL_I_TYPE;
                        o_rfile_w_sel   = RFILE_W_SEL_DMEM_R_DATA;
                        o_op_sel        = DCDR_OP_SEL_LOAD; 

                        //o_rfile_we      = 1;
                        o_dmem_re       = 1;
                    end
                    default : begin
                        o_ill_instrn = 1;
                    end
                endcase
            end
            OPCODE_STORE : begin     // S-Type opcode *store instructions
                case (w_funct3)
                    FUNCT3_S_SB, FUNCT3_S_SH, FUNCT3_S_SW : begin
                        o_alu_src_a_sel = ALU_SRC_A_SEL_RS1; 
                        o_alu_src_b_sel = ALU_SRC_B_SEL_IMM;
                        o_alu_func_sel  = ALU_FUNC_SEL_ADD;
                        o_imm_gen_sel   = IMM_GEN_SEL_S_TYPE;
                        o_op_sel        = DCDR_OP_SEL_STORE; 

                        o_dmem_we       = 1;
                    end
                    default : begin
                        o_ill_instrn = 1;
                    end
                endcase
            end
            OPCODE_BRANCH : begin  // B-Type opcode
                case (w_funct3) // All store instructions; writing from registers to memory
                    FUNCT3_B_BEQ, FUNCT3_B_BNE, FUNCT3_B_BLT,
                    FUNCT3_B_BGE, FUNCT3_B_BLTU, FUNCT3_B_BGEU : begin
                        o_imm_gen_sel  = IMM_GEN_SEL_BRANCH;
                        o_addr_gen_sel = ADDR_GEN_SEL_BRANCH;
                        o_op_sel       = DCDR_OP_SEL_BRANCH; 
                    end
                    default : begin
                        o_ill_instrn = 1;
                    end
                endcase
            end
            OPCODE_LUI : begin
                o_alu_src_b_sel = ALU_SRC_B_SEL_IMM;      // Extends a 20-bit immediate (extra 12-bits after)
                o_rfile_w_sel   = RFILE_W_SEL_ALU_RESULT;
                o_alu_func_sel  = ALU_FUNC_SEL_LUI;                // Value passes through ALU and is stored in a register
                o_imm_gen_sel   = IMM_GEN_SEL_UPPER;
                o_op_sel        = DCDR_OP_SEL_LUI; 

                o_rfile_we      = 1;
            end
            OPCODE_AUIPC : begin
                o_alu_src_a_sel = ALU_SRC_A_SEL_PC; // Adds a U-type immediate to the program count
                o_alu_src_b_sel = ALU_SRC_B_SEL_IMM;   // which is stored in a register
                o_rfile_w_sel   = RFILE_W_SEL_ALU_RESULT;
                o_alu_func_sel  = ALU_FUNC_SEL_ADD;
                o_imm_gen_sel   = IMM_GEN_SEL_UPPER;
                o_op_sel        = DCDR_OP_SEL_AUIPC; 

                o_rfile_we      = 1;
            end
            OPCODE_JAL : begin
                o_rfile_w_sel  = RFILE_W_SEL_PC_ADDR_INC; // stores current location + 4 in a register
                o_imm_gen_sel  = IMM_GEN_SEL_JUMP;
                o_addr_gen_sel = ADDR_GEN_SEL_JAL;
                o_op_sel       = DCDR_OP_SEL_JAL;

                o_rfile_we     = 1;
            end
            OPCODE_FENCE : begin
                casez ({w_fm_fence, w_funct3})
                    {4'bz000, FUNCT3_FENCE}   : o_op_sel     = DCDR_OP_SEL_FENCE;
                    default                   : o_ill_instrn = 1;
                endcase
            end
            OPCODE_SYS : begin
                case (w_funct3)
                    FUNCT3_SYS_CSRRW, FUNCT3_SYS_CSRRS, FUNCT3_SYS_CSRRC, 
                    FUNCT3_SYS_CSRRWI, FUNCT3_SYS_CSRRSI, FUNCT3_SYS_CSRRCI : begin
                        // CSRRS and CSRRC are read-only compatible if rs1 is x0
                        w_csr_write_attempt = (w_funct3 == FUNCT3_SYS_CSRRW)
                            || (w_funct3 == FUNCT3_SYS_CSRRWI) 
                            || (w_rs1_addr != 5'b0);

                        // illegal if csr dne or write to read-only
                        if (w_csr_addr_illegal || (w_csr_write_attempt && w_csr_is_readonly)) begin
                            o_ill_instrn = 1;
                        end else begin
                            o_rfile_w_sel = RFILE_W_SEL_CSR_R_DATA;
                            o_op_sel      = DCDR_OP_SEL_WRITE;

                            o_rfile_we    = 1;
                            o_csr_we      = w_csr_write_attempt;
                        end
                    end
                    FUNCT3_SYS_TRAPS : begin
                        // check for spec compliance (rd=0, rs1=0)
                        if (w_rd_addr != 0 || w_rs1_addr != 0) begin
                            o_ill_instrn = 1;
                        end else begin
                            case ({w_funct7, w_rs2_addr})
                                FUNCT7_RS2_SYS_ECALL  : o_op_sel     = DCDR_OP_SEL_ECALL;
                                FUNCT7_RS2_SYS_EBREAK : o_op_sel     = DCDR_OP_SEL_EBREAK;
                                FUNCT7_RS2_SYS_MRET   : o_op_sel     = DCDR_OP_SEL_MRET;
                                FUNCT7_RS2_SYS_WFI    : o_op_sel     = DCDR_OP_SEL_WFI;
                                default               : o_ill_instrn = 1;
                            endcase
                        end
                    end
                    default : begin 
                        o_ill_instrn = 1;
                    end
                endcase
            end
            default : begin
                o_ill_instrn = 1;
            end
        endcase

        // illegal instructions must not read/write external state
        if (o_ill_instrn) begin
            o_alu_func_sel  = 0;
            o_alu_src_a_sel = 0;
            o_alu_src_b_sel = 0;
            o_rfile_w_sel   = 0;
            o_imm_gen_sel   = 0;
            o_addr_gen_sel  = 0;
            o_op_sel        = 0;

            o_rfile_we      = 0;
            o_dmem_re       = 0;
            o_dmem_we       = 0;
            o_csr_we        = 0;
        end
    end


    // check if csr address refers to implemented register
    always @(*) begin
        w_csr_addr_illegal = 1;
        case (w_csr_addr)
            CSR_MSTATUS_ADDR,
            CSR_MISA_ADDR,
            CSR_MIE_ADDR,
            CSR_MTVEC_ADDR,
            CSR_MSTATUSH_ADDR,
            CSR_MSCRATCH_ADDR,
            CSR_MEPC_ADDR,
            CSR_MCAUSE_ADDR,
            CSR_MTVAL_ADDR,
            CSR_MIP_ADDR,
            CSR_MVENDORID_ADDR,
            CSR_MARCHID_ADDR,
            CSR_MIMPID_ADDR,
            CSR_MHARTID_ADDR,
            CSR_MCONFIGPTR_ADDR : begin
                w_csr_addr_illegal = 0;
            end
            default : ;
        endcase
    end

    // check if the address is for a read-only register
    always @(*) begin
        w_csr_is_readonly = 0;
        case (w_csr_addr)
            CSR_MVENDORID_ADDR,
            CSR_MARCHID_ADDR,
            CSR_MIMPID_ADDR,
            CSR_MHARTID_ADDR,
            CSR_MCONFIGPTR_ADDR : begin
                w_csr_is_readonly = 1;
            end
            default : ;
        endcase
    end


`ifdef FORMAL

    // CSR write attempt per spec (rs1!=x0 means write for RS/RC)
    wire csr_f3_rwgrp = (w_funct3==FUNCT3_SYS_CSRRW) | (w_funct3==FUNCT3_SYS_CSRRS) | (w_funct3==FUNCT3_SYS_CSRRC)
                      | (w_funct3==FUNCT3_SYS_CSRRWI)| (w_funct3==FUNCT3_SYS_CSRRSI)| (w_funct3==FUNCT3_SYS_CSRRCI);
    wire csr_write_attempt = (w_funct3==FUNCT3_SYS_CSRRW) | (w_funct3==FUNCT3_SYS_CSRRWI)
                           | ((w_funct3==FUNCT3_SYS_CSRRS)  & (w_rs1_addr!=5'b0))
                           | ((w_funct3==FUNCT3_SYS_CSRRC)  & (w_rs1_addr!=5'b0))
                           | ((w_funct3==FUNCT3_SYS_CSRRSI) & (w_rs1_addr!=5'b0)) // rs1 encodes uimm in *I forms; nonzero means write
                           | ((w_funct3==FUNCT3_SYS_CSRRCI) & (w_rs1_addr!=5'b0));

    // ============= Generic invariants =============

    // Illegal ⇒ all selectors/enablers are zeroed
    always @* if (o_ill_instrn) begin
        assert(o_alu_func_sel  == 4'd0);
        assert(o_alu_src_a_sel == 1'd0);
        assert(o_alu_src_b_sel == 1'd0);
        assert(o_rfile_w_sel   == 2'd0);
        assert(o_imm_gen_sel   == 3'd0);
        assert(o_addr_gen_sel  == 2'd0);
        assert(o_op_sel        == 15'd0);

        assert(!o_rfile_we);
        assert(!o_dmem_re);
        assert(!o_dmem_we);
        assert(!o_csr_we);
    end


    // ============= OP_REG (R-type) =============
    // Valid pairs produce ALU operation, writeback, rs1/rs2 sources.
    wire is_op_reg = (w_opcode == OPCODE_OP_REG);
    always @* if (is_op_reg) begin
        // sources & writeback for all non-illegal R-type
        if (!o_ill_instrn) begin
            assert(o_alu_src_a_sel == ALU_SRC_A_SEL_RS1);
            assert(o_alu_src_b_sel == ALU_SRC_B_SEL_RS2);
            assert(o_rfile_w_sel   == RFILE_W_SEL_ALU_RESULT);
            assert(o_rfile_we);
            assert(!o_dmem_re);
            assert(!o_dmem_we);
            assert(!o_csr_we);
            assert(o_op_sel        == DCDR_OP_SEL_REG);
        end

        // Table of legal {funct7,funct3} → func
        unique case ({w_funct7,w_funct3})
            {FUNCT7_I_R_0, FUNCT3_R_ADD} : assert(o_alu_func_sel == ALU_FUNC_SEL_ADD && !o_ill_instrn);
            {FUNCT7_I_R_1, FUNCT3_R_SUB} : assert(o_alu_func_sel == ALU_FUNC_SEL_SUB && !o_ill_instrn);
            {FUNCT7_I_R_0, FUNCT3_R_SLL} : assert(o_alu_func_sel == ALU_FUNC_SEL_SLL && !o_ill_instrn);
            {FUNCT7_I_R_0, FUNCT3_R_SLT} : assert(o_alu_func_sel == ALU_FUNC_SEL_SLT && !o_ill_instrn);
            {FUNCT7_I_R_0, FUNCT3_R_SLTU}: assert(o_alu_func_sel == ALU_FUNC_SEL_SLTU && !o_ill_instrn);
            {FUNCT7_I_R_0, FUNCT3_R_XOR} : assert(o_alu_func_sel == ALU_FUNC_SEL_XOR && !o_ill_instrn);
            {FUNCT7_I_R_0, FUNCT3_R_SRL} : assert(o_alu_func_sel == ALU_FUNC_SEL_SRL && !o_ill_instrn);
            {FUNCT7_I_R_1, FUNCT3_R_SRA} : assert(o_alu_func_sel == ALU_FUNC_SEL_SRA && !o_ill_instrn);
            {FUNCT7_I_R_0, FUNCT3_R_OR}  : assert(o_alu_func_sel == ALU_FUNC_SEL_OR  && !o_ill_instrn);
            {FUNCT7_I_R_0, FUNCT3_R_AND} : assert(o_alu_func_sel == ALU_FUNC_SEL_AND && !o_ill_instrn);
            default:                      assert(o_ill_instrn);
        endcase
    end

    // ============= OP_IMM (I-type ALU) =============
    wire is_op_imm = (w_opcode == OPCODE_OP_IMM);
    always @* if (is_op_imm) begin
        if (!o_ill_instrn) begin
            assert(o_alu_src_a_sel == ALU_SRC_A_SEL_RS1);
            assert(o_alu_src_b_sel == ALU_SRC_B_SEL_IMM);
            assert(o_rfile_w_sel   == RFILE_W_SEL_ALU_RESULT);
            assert(o_imm_gen_sel   == IMM_GEN_SEL_I_TYPE);
            assert(o_rfile_we);
            assert(!o_dmem_re);
            assert(!o_dmem_we);
            assert(!o_csr_we);
            assert(o_op_sel        == DCDR_OP_SEL_IMM);
        end

        unique case (w_funct3)
            FUNCT3_I_ADDI : assert(o_alu_func_sel == ALU_FUNC_SEL_ADD  && !o_ill_instrn);
            FUNCT3_I_SLTI : assert(o_alu_func_sel == ALU_FUNC_SEL_SLT  && !o_ill_instrn);
            FUNCT3_I_SLTIU: assert(o_alu_func_sel == ALU_FUNC_SEL_SLTU && !o_ill_instrn);
            FUNCT3_I_ORI  : assert(o_alu_func_sel == ALU_FUNC_SEL_OR   && !o_ill_instrn);
            FUNCT3_I_XORI : assert(o_alu_func_sel == ALU_FUNC_SEL_XOR  && !o_ill_instrn);
            FUNCT3_I_ANDI : assert(o_alu_func_sel == ALU_FUNC_SEL_AND  && !o_ill_instrn);
            FUNCT3_I_SLI  : begin
                if (w_funct7 == FUNCT7_I_R_0) begin
                    assert(o_alu_func_sel == ALU_FUNC_SEL_SLL && !o_ill_instrn);
                end else begin
                    assert(o_ill_instrn);
                end
            end
            FUNCT3_I_SRI  : begin
                if (w_funct7 == FUNCT7_I_R_0) begin
                    assert(o_alu_func_sel == ALU_FUNC_SEL_SRL && !o_ill_instrn);
                end else if (w_funct7 == FUNCT7_I_R_1) begin
                    assert(o_alu_func_sel == ALU_FUNC_SEL_SRA && !o_ill_instrn);
                end else begin
                    assert(o_ill_instrn);
                end
            end
            default: assert(o_ill_instrn);
        endcase
    end

    // ============= JALR =============
    always @* if (w_opcode == OPCODE_JALR) begin
        if (w_funct3 == FUNCT3_I_JALR) begin
            assert(!o_ill_instrn);
            assert(o_rfile_we);
            assert(o_rfile_w_sel  == RFILE_W_SEL_PC_ADDR_INC);
            assert(o_imm_gen_sel  == IMM_GEN_SEL_I_TYPE);
            assert(o_addr_gen_sel == ADDR_GEN_SEL_JALR);
            assert(o_op_sel       == DCDR_OP_SEL_JALR);
            assert(!o_dmem_re);
            assert(!o_dmem_we);
            assert(!o_csr_we);
        end else begin
            assert(o_ill_instrn);
        end
    end

    // ============= LOADS =============
    always @* if (w_opcode == OPCODE_LOAD) begin
        unique case (w_funct3)
            FUNCT3_I_LB, FUNCT3_I_LH, FUNCT3_I_LW, FUNCT3_I_LBU, FUNCT3_I_LHU: begin
                assert(!o_ill_instrn);
                assert(o_alu_src_a_sel == ALU_SRC_A_SEL_RS1);
                assert(o_alu_src_b_sel == ALU_SRC_B_SEL_IMM);
                assert(o_alu_func_sel  == ALU_FUNC_SEL_ADD);
                assert(o_imm_gen_sel   == IMM_GEN_SEL_I_TYPE);
                assert(o_rfile_w_sel   == RFILE_W_SEL_DMEM_R_DATA);
                assert(o_op_sel        == DCDR_OP_SEL_LOAD);
                assert(!o_rfile_we);
                assert(o_dmem_re);
                assert(!o_dmem_we);
                assert(!o_csr_we);
            end
            default: assert(o_ill_instrn);
        endcase
    end

    // ============= STORES =============
    always @* if (w_opcode == OPCODE_STORE) begin
        unique case (w_funct3)
            FUNCT3_S_SB, FUNCT3_S_SH, FUNCT3_S_SW: begin
                assert(!o_ill_instrn);
                assert(o_alu_src_a_sel == ALU_SRC_A_SEL_RS1);
                assert(o_alu_src_b_sel == ALU_SRC_B_SEL_IMM);
                assert(o_alu_func_sel  == ALU_FUNC_SEL_ADD);
                assert(o_imm_gen_sel   == IMM_GEN_SEL_S_TYPE);
                assert(o_op_sel        == DCDR_OP_SEL_STORE);
                assert(!o_rfile_we);
                assert(!o_dmem_re);
                assert(o_dmem_we);
                assert(!o_csr_we);
            end
            default: assert(o_ill_instrn);
        endcase
    end

    // ============= BRANCHES =============
    always @* if (w_opcode == OPCODE_BRANCH) begin
        unique case (w_funct3)
            FUNCT3_B_BEQ, FUNCT3_B_BNE, FUNCT3_B_BLT,
            FUNCT3_B_BGE, FUNCT3_B_BLTU, FUNCT3_B_BGEU: begin
                assert(!o_ill_instrn);
                assert(o_imm_gen_sel  == IMM_GEN_SEL_BRANCH);
                assert(o_addr_gen_sel == ADDR_GEN_SEL_BRANCH);
                assert(o_op_sel       == DCDR_OP_SEL_BRANCH);
                assert(!o_rfile_we);
                assert(!o_dmem_re);
                assert(!o_dmem_we);
                assert(!o_csr_we);
            end
            default: assert(o_ill_instrn);
        endcase
    end

    // ============= LUI =============
    always @* if (w_opcode == OPCODE_LUI) begin
        assert(!o_ill_instrn);
        assert(o_alu_src_b_sel == ALU_SRC_B_SEL_IMM);
        assert(o_rfile_w_sel   == RFILE_W_SEL_ALU_RESULT);
        assert(o_alu_func_sel  == ALU_FUNC_SEL_LUI);
        assert(o_imm_gen_sel   == IMM_GEN_SEL_UPPER);
        assert(o_op_sel        == DCDR_OP_SEL_LUI);
        assert(o_rfile_we);
        assert(!o_dmem_re);
        assert(!o_dmem_we);
        assert(!o_csr_we);
    end

    // ============= AUIPC =============
    always @* if (w_opcode == OPCODE_AUIPC) begin
        assert(!o_ill_instrn);
        assert(o_alu_src_a_sel == ALU_SRC_A_SEL_PC);
        assert(o_alu_src_b_sel == ALU_SRC_B_SEL_IMM);
        assert(o_rfile_w_sel   == RFILE_W_SEL_ALU_RESULT);
        assert(o_alu_func_sel  == ALU_FUNC_SEL_ADD);
        assert(o_imm_gen_sel   == IMM_GEN_SEL_UPPER);
        assert(o_op_sel        == DCDR_OP_SEL_AUIPC);
        assert(o_rfile_we);
        assert(!o_dmem_re);
        assert(!o_dmem_we);
        assert(!o_csr_we);
    end

    // ============= JAL =============
    always @* if (w_opcode == OPCODE_JAL) begin
        assert(!o_ill_instrn);
        assert(o_rfile_we);
        assert(o_rfile_w_sel  == RFILE_W_SEL_PC_ADDR_INC);
        assert(o_imm_gen_sel  == IMM_GEN_SEL_JUMP);
        assert(o_addr_gen_sel == ADDR_GEN_SEL_JAL);
        assert(o_op_sel       == DCDR_OP_SEL_JAL);
        assert(!o_dmem_re);
        assert(!o_dmem_we);
        assert(!o_csr_we);
    end

    // ============= FENCE =============
    always @* if (w_opcode == OPCODE_FENCE) begin
        // Accept FENCE when fm[2:0]==0 and FENCE.I always
        if (w_funct3 == FUNCT3_FENCE  && w_fm_fence[2:0]==3'b000) begin
            assert(!o_ill_instrn);
            assert(o_op_sel == DCDR_OP_SEL_FENCE);
            // No external state changes
            assert(!o_rfile_we);
            assert(!o_dmem_re);
            assert(!o_dmem_we);
            assert(!o_csr_we);
        end else begin
            assert(o_ill_instrn);
        end
    end

    // ============= SYSTEM (CSR + TRAPS) =============
    always @* if (w_opcode == OPCODE_SYS) begin
        if (w_funct3 == FUNCT3_SYS_TRAPS) begin
            // must have rd=0 and rs1=0 per spec
            if (w_rd_addr != 5'd0 || w_rs1_addr != 5'd0) begin
                assert(o_ill_instrn);
            end else begin
                unique case ({w_funct7, w_rs2_addr})
                    FUNCT7_RS2_SYS_ECALL  : assert(o_op_sel == DCDR_OP_SEL_ECALL  && !o_ill_instrn);
                    FUNCT7_RS2_SYS_EBREAK : assert(o_op_sel == DCDR_OP_SEL_EBREAK && !o_ill_instrn);
                    FUNCT7_RS2_SYS_MRET   : assert(o_op_sel == DCDR_OP_SEL_MRET   && !o_ill_instrn);
                    FUNCT7_RS2_SYS_WFI    : assert(o_op_sel == DCDR_OP_SEL_WFI    && !o_ill_instrn);
                    default               : assert(o_ill_instrn);
                endcase
                // No register/mem/csr writes for traps themselves
                if (!o_ill_instrn) begin
                    assert(!o_rfile_we);
                    assert(!o_dmem_re);
                    assert(!o_dmem_we);
                    assert(!o_csr_we);
                end
            end
        end
        else if (csr_f3_rwgrp) begin
            // CSR instruction
            if (w_csr_addr_illegal) begin
                assert(o_ill_instrn);
            end else if (csr_write_attempt && w_csr_is_readonly) begin
                assert(o_ill_instrn);
            end else begin
                assert(!o_ill_instrn);
                assert(o_rfile_w_sel == RFILE_W_SEL_CSR_R_DATA);
                assert(o_op_sel      == DCDR_OP_SEL_WRITE);
                assert(o_rfile_we);
                assert(o_csr_we == csr_write_attempt);
                assert(!o_dmem_re);
                assert(!o_dmem_we);
            end
        end
        else begin
            assert(o_ill_instrn);
        end
    end

    // ============= Default opcode =============
    always @* begin
        case (w_opcode)
            OPCODE_OP_REG, OPCODE_OP_IMM, OPCODE_JALR, OPCODE_LOAD,
            OPCODE_STORE, OPCODE_BRANCH, OPCODE_LUI, OPCODE_AUIPC,
            OPCODE_JAL, OPCODE_FENCE, OPCODE_SYS: ;
            default: assert(o_ill_instrn);
        endcase
    end

    // A few sanity covers to ensure we explore interesting spaces
    // (not required, but helpful for debugging / wave dumps)
    always @(*) begin
        cover (w_opcode == OPCODE_LOAD  && !o_ill_instrn);
        cover (w_opcode == OPCODE_STORE && !o_ill_instrn);
        cover (w_opcode == OPCODE_BRANCH&& !o_ill_instrn);
        cover (w_opcode == OPCODE_JALR  && !o_ill_instrn);
        cover (w_opcode == OPCODE_SYS   && csr_f3_rwgrp && !o_ill_instrn);
        cover (w_opcode == OPCODE_SYS   && w_funct3==FUNCT3_SYS_TRAPS && !o_ill_instrn);
    end

`endif

endmodule
