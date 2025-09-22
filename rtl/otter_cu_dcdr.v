`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/02/2022 03:54:39 PM
// Design Name: 
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

module otter_cu_dcdr (
    input      [31:0] instrn,
    input             intrpt_taken, 
    input             br_eq, 
    input             br_lt, 
    input             br_ltu,
    output reg [3:0]  alu_func,
    output reg        alu_src_sel_a,
    output reg [1:0]  alu_src_sel_b,
    output reg [2:0]  pc_src_sel,
    output reg [1:0]  rfile_w_sel
);

    reg branch_base;
    wire [6:0] opcode, prefix_i_r;
    wire [2:0] func;
    wire       instrn_bit_30;

    assign opcode        = `INSTRN_OPCODE(instrn);
    assign func          = `INSTRN_FUNC(instrn);
    assign instrn_bit_30 = instrn[30];
    assign prefix_i_r    = `INSTRN_I_R_PREFIX(instrn);

    always @(*) begin
        alu_func      = '0; 
        alu_src_sel_a = '0; 
        alu_src_sel_b = '0;
        pc_src_sel    = '0; 
        rfile_w_sel   = '0; 
        branch_base   = '0;

        if (intrpt_taken == '1) begin  // Interrupt Case
            pc_src_sel = DCDR_PC_SRC_SEL_MTVEC;
        end else begin
            case(opcode)
                OPCODE_OP_REG : begin    // R-Type opcode
                    case ({prefix_i_r, func})  // arithmetic/logical operations including two registers
                        PRE_FUNC_R_ADD, PRE_FUNC_R_SUB, PRE_FUNC_R_SLL, PRE_FUNC_R_SLT,
                        PRE_FUNC_R_SLTU, PRE_FUNC_R_XOR, PRE_FUNC_R_SRL, PRE_FUNC_R_SRA,
                        PRE_FUNC_R_OR, PRE_FUNC_R_AND : begin
                            alu_src_sel_a = DCDR_ALU_SRC_SEL_A_RS1;
                            alu_src_sel_b = DCDR_ALU_SRC_SEL_B_RS2;
                            rfile_w_sel   = DCDR_RFILE_W_SEL_ALU_RESULT;
                            pc_src_sel    = DCDR_PC_SRC_SEL_ADDR_INC;
                            alu_func      = {instrn_bit_30, func}; 
                        end
                        default begin end
                    endcase
                end
                OPCODE_OP_IMM : begin    // I-Type opcode *no loading
                    case (func) // arithmetic/logical operations including a register and immediate
                        FUNC_I_ADDI, FUNC_I_SLTI, FUNC_I_SLTIU, FUNC_I_ORI,
                        FUNC_I_XORI, FUNC_I_ANDI, FUNC_I_SLI : begin
                            alu_src_sel_a = DCDR_ALU_SRC_SEL_A_RS1; 
                            alu_src_sel_b = DCDR_ALU_SRC_SEL_B_I_TYPE_IMM;
                            rfile_w_sel   = DCDR_RFILE_W_SEL_ALU_RESULT;
                            pc_src_sel    = DCDR_PC_SRC_SEL_ADDR_INC;
                            alu_func      = {1'b0, func};
                        end
                        FUNC_I_SRI : begin
                            case (prefix_i_r)
                                PREFIX_I_R_0, PREFIX_I_R_1 : begin
                                    alu_src_sel_a = DCDR_ALU_SRC_SEL_A_RS1; 
                                    alu_src_sel_b = DCDR_ALU_SRC_SEL_B_I_TYPE_IMM;
                                    rfile_w_sel   = DCDR_RFILE_W_SEL_ALU_RESULT;
                                    pc_src_sel    = DCDR_PC_SRC_SEL_ADDR_INC;
                                    alu_func      = {instrn_bit_30, func};
                                end
                                default begin end
                            endcase
                        end
                        default begin end
                    endcase
                end
                OPCODE_JALR : begin    // I-Type opcode *jalr
                    case (func) // jumps and links to the value stored in rs1 added to an I-Type immediate
                        FUNC_I_JALR : begin
                            rfile_w_sel = DCDR_RFILE_W_SEL_PC_ADDR_INC;
                            pc_src_sel  = DCDR_PC_SRC_SEL_JALR;
                        end
                        default begin end
                    endcase
                end
                OPCODE_LOAD : begin      // I-Type opcode *load instructions
                    case (func) // All load instructions; writing from memory to registers
                        FUNC_I_LB, FUNC_I_LH, FUNC_I_LW,
                        FUNC_I_LBU, FUNC_I_LHU : begin
                            alu_src_sel_a = DCDR_ALU_SRC_SEL_A_RS1;
                            alu_src_sel_b = DCDR_ALU_SRC_SEL_B_I_TYPE_IMM;
                            rfile_w_sel   = DCDR_RFILE_W_SEL_MEM_DATA_OUT;
                            pc_src_sel    = DCDR_PC_SRC_SEL_ADDR_INC;
                            alu_func      = ALU_ADD;
                        end
                        default begin end
                    endcase
                end
                OPCODE_STORE : begin     // S-Type opcode *store instructions
                    case (func) // All store instructions; writing from registers to memory
                        FUNC_S_SB, FUNC_S_SH, FUNC_S_SW : begin
                            alu_src_sel_a = DCDR_ALU_SRC_SEL_A_RS1; 
                            alu_src_sel_b = DCDR_ALU_SRC_SEL_B_S_TYPE_IMM;
                            pc_src_sel    = DCDR_PC_SRC_SEL_ADDR_INC;
                            alu_func      = ALU_ADD;
                        end
                        default begin end
                    endcase
                end
                OPCODE_BRANCH : begin  // B-Type opcode
                    case(`FUNC_BRANCH_BASE(func)) // select the base comparison value
                        `FUNC_BRANCH_BASE(FUNC_B_BEQ)  : branch_base = br_eq;  
                        `FUNC_BRANCH_BASE(FUNC_B_BLT)  : branch_base = br_lt;
                        `FUNC_BRANCH_BASE(FUNC_B_BLTU) : branch_base = br_ltu;
                        default : begin end // invalid opcode, nop
                    endcase
                    case(`FUNC_BRANCH_BASE(func)) // check result based on selector
                        `FUNC_BRANCH_BASE(FUNC_B_BEQ), 
                        `FUNC_BRANCH_BASE(FUNC_B_BLT),
                        `FUNC_BRANCH_BASE(FUNC_B_BLTU) : begin
                            if (branch_base != `FUNC_BRANCH_SEL(func)) begin
                                pc_src_sel = DCDR_PC_SRC_SEL_BRANCH;
                            end else begin
                                pc_src_sel = DCDR_PC_SRC_SEL_ADDR_INC;
                            end
                        end
                        default : begin end // invalid opcode, nop
                    endcase
                end
                OPCODE_LUI : begin       // lui opcode
                    alu_src_sel_a = DCDR_ALU_SRC_SEL_A_UPPER_IMM; // Extends a 20-bit immediate (extra 12-bits after)
                    rfile_w_sel   = DCDR_RFILE_W_SEL_ALU_RESULT;
                    pc_src_sel    = DCDR_PC_SRC_SEL_ADDR_INC;
                    alu_func      = ALU_LUI;                     // Value passes through ALU and is stored in a register
                end
                OPCODE_AUIPC : begin    // auipc opcode
                   alu_src_sel_a = DCDR_ALU_SRC_SEL_A_UPPER_IMM; // Adds a U-type immediate to the program count
                   alu_src_sel_b = DCDR_ALU_SRC_SEL_B_PC_ADDR;   // which is stored in a register
                   rfile_w_sel   = DCDR_RFILE_W_SEL_ALU_RESULT;
                   pc_src_sel    = DCDR_PC_SRC_SEL_ADDR_INC;
                   alu_func      = ALU_ADD;
                end
                OPCODE_JAL : begin    // J-Type opcode *jal
                   rfile_w_sel = DCDR_RFILE_W_SEL_PC_ADDR_INC; // stores current location + 4 in a register
                   pc_src_sel  = DCDR_PC_SRC_SEL_JAL;          // Jumps to a new location (updates PC value)
                end
                OPCODE_SYS : begin         // SYS opcode
                    case (func)
                        FUNC_SYS_CSRRW : begin
                            rfile_w_sel = DCDR_RFILE_W_SEL_CSR_R_DATA;
                            pc_src_sel  = DCDR_PC_SRC_SEL_ADDR_INC;
                        end
                        FUNC_SYS_MRET : begin
                            if (instrn[31:7] == PREFIX_SYS_MRET) begin
                                pc_src_sel = DCDR_PC_SRC_SEL_MEPC;
                            end
                        end
                        default : begin end // invalid opcode, nop
                    endcase
                end
                default : begin end
            endcase
        end
    end

`ifdef FORMAL

    // Default values for outputs (when no specific instruction matches)
    // The decoder initializes all outputs to '0'.
    always @(*) begin
        if (!intrpt_taken &&
            !(opcode == OPCODE_OP_REG && (
                {prefix_i_r, func} == PRE_FUNC_R_ADD  ||
                {prefix_i_r, func} == PRE_FUNC_R_SUB  ||
                {prefix_i_r, func} == PRE_FUNC_R_SLL  ||
                {prefix_i_r, func} == PRE_FUNC_R_SLT  ||
                {prefix_i_r, func} == PRE_FUNC_R_SLTU ||
                {prefix_i_r, func} == PRE_FUNC_R_XOR  ||
                {prefix_i_r, func} == PRE_FUNC_R_SRL  ||
                {prefix_i_r, func} == PRE_FUNC_R_SRA  ||
                {prefix_i_r, func} == PRE_FUNC_R_OR   ||
                {prefix_i_r, func} == PRE_FUNC_R_AND
            )) &&
            !(opcode == OPCODE_OP_IMM && (
                func == FUNC_I_ADDI  ||
                func == FUNC_I_SLTI  ||
                func == FUNC_I_SLTIU ||
                func == FUNC_I_ORI   ||
                func == FUNC_I_XORI  ||
                func == FUNC_I_ANDI  ||
                func == FUNC_I_SLI
            )) &&
            !(opcode == OPCODE_OP_IMM && func == FUNC_I_SRI && (prefix_i_r == PREFIX_I_R_0 || prefix_i_r == PREFIX_I_R_1)) &&
            !(opcode == OPCODE_JALR && func == FUNC_I_JALR) &&
            !(opcode == OPCODE_LOAD && (
                func == FUNC_I_LB  ||
                func == FUNC_I_LH  ||
                func == FUNC_I_LW  ||
                func == FUNC_I_LBU ||
                func == FUNC_I_LHU
            )) &&
            !(opcode == OPCODE_STORE && (
                func == FUNC_S_SB ||
                func == FUNC_S_SH ||
                func == FUNC_S_SW
            )) &&
            !(opcode == OPCODE_BRANCH && (
                `FUNC_BRANCH_BASE(func) == `FUNC_BRANCH_BASE(FUNC_B_BEQ) ||
                `FUNC_BRANCH_BASE(func) == `FUNC_BRANCH_BASE(FUNC_B_BLT) ||
                `FUNC_BRANCH_BASE(func) == `FUNC_BRANCH_BASE(FUNC_B_BLTU)
            )) &&
            !(opcode == OPCODE_LUI) &&
            !(opcode == OPCODE_AUIPC) &&
            !(opcode == OPCODE_JAL) &&
            !(opcode == OPCODE_SYS && func == FUNC_SYS_CSRRW) &&
            !(opcode == OPCODE_SYS && func == FUNC_SYS_MRET && instrn[31:7] == PREFIX_SYS_MRET)
        ) begin
            assert(alu_func      == '0);
            assert(alu_src_sel_a == '0);
            assert(alu_src_sel_b == '0);
            assert(pc_src_sel    == '0);
            assert(rfile_w_sel   == '0);
        end
    end

    // Interrupt Case
    always @(*) begin
        if (intrpt_taken == 1'b1) begin
            assert(pc_src_sel    == DCDR_PC_SRC_SEL_MTVEC);
            assert(alu_func      == '0);
            assert(alu_src_sel_a == '0);
            assert(alu_src_sel_b == '0);
            assert(rfile_w_sel   == '0);
        end
    end

    reg r_type_valid;
    always @(*) begin
        if (intrpt_taken == 1'b0 && opcode == OPCODE_OP_REG) begin

            // Specific R-type func checks
            case ({prefix_i_r, func})
                PRE_FUNC_R_ADD  : begin assert(alu_func == ALU_ADD);  r_type_valid = 1; end 
                PRE_FUNC_R_SUB  : begin assert(alu_func == ALU_SUB);  r_type_valid = 1; end  
                PRE_FUNC_R_SLL  : begin assert(alu_func == ALU_SLL);  r_type_valid = 1; end
                PRE_FUNC_R_SRL  : begin assert(alu_func == ALU_SRL);  r_type_valid = 1; end
                PRE_FUNC_R_SRA  : begin assert(alu_func == ALU_SRA);  r_type_valid = 1; end
                PRE_FUNC_R_XOR  : begin assert(alu_func == ALU_XOR);  r_type_valid = 1; end
                PRE_FUNC_R_OR   : begin assert(alu_func == ALU_OR);   r_type_valid = 1; end
                PRE_FUNC_R_AND  : begin assert(alu_func == ALU_AND);  r_type_valid = 1; end
                PRE_FUNC_R_SLT  : begin assert(alu_func == ALU_SLT);  r_type_valid = 1; end
                PRE_FUNC_R_SLTU : begin assert(alu_func == ALU_SLTU); r_type_valid = 1; end
                default         : begin assert(alu_func == '0);       r_type_valid = 0; end
            endcase

            if (r_type_valid) begin
                // Common R-type outputs
                assert(alu_src_sel_a == DCDR_ALU_SRC_SEL_A_RS1);
                assert(alu_src_sel_b == DCDR_ALU_SRC_SEL_B_RS2);
                assert(rfile_w_sel   == DCDR_RFILE_W_SEL_ALU_RESULT);
                assert(pc_src_sel    == DCDR_PC_SRC_SEL_ADDR_INC);
            end
        end
    end

`endif

endmodule
