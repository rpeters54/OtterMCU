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

    assign illegal_instrn = |csr_trap_cause_sel;

    // Helper signals for formal properties
    wire f_preempted = rst || illegal_instrn || csr_intrpt_vld;
    reg  f_past_valid;
    reg  f_past_rst;
    reg [31:0] f_prev_instrn;

    wire branch_taken = (funct3 == FUNCT3_B_BEQ  &&  br_eq)  ||
        (funct3 == FUNCT3_B_BNE  && !br_eq)  ||
        (funct3 == FUNCT3_B_BLT  &&  br_lt)  ||
        (funct3 == FUNCT3_B_BGE  && !br_lt)  ||
        (funct3 == FUNCT3_B_BLTU &&  br_ltu) ||
        (funct3 == FUNCT3_B_BGEU && !br_ltu);


    initial begin
        f_past_valid = 0;
        f_past_rst = 0;
        f_prev_cycle_was_load = 0;
        f_prev_instrn = '0;
    end

    always @(posedge clk) begin
        f_past_valid <= 1;
        f_past_rst   <= rst;
        f_prev_instrn <= instrn;
    end

    // --- Assumptions ---
    always @(*) begin
        if (!f_past_valid) assume(rst);
    end

    // --- Assertions ---
    always @(*) begin
        // On reset, the FSM must be in the INIT state.
        if (f_past_rst) assert(present_state == ST_INIT);

        // regardless of other inputs, if reset these should be valid
        if (rst)
            assert(pc_src_sel    == PC_SRC_SEL_RESET_VEC &&
                   csr_op_sel    == CSR_OP_RESET &&
                   next_state    == ST_INIT &&
                   pc_w_en       == 1 &&
                   rfile_w_en    == 0 &&
                   dmem_w_en     == 0 &&
                   dmem_r_en     == 0 &&
                   csr_w_en      == 0 &&
                   stall         == 0);

        // interrupts take priority over synchronous traps
        if (present_state == ST_EXEC && !rst && csr_intrpt_vld && illegal_instrn)
            assert(csr_op_sel == CSR_OP_INTRPT);

        // When any trap or interrupt is taken, architectural state writes must be disabled.
        if (present_state == ST_EXEC && !rst && csr_intrpt_vld && illegal_instrn)
            assert(next_state    == ST_EXEC &&
                   pc_w_en       == 1 &&
                   rfile_w_en    == 0 &&
                   dmem_w_en     == 0 &&
                   dmem_r_en     == 0 &&
                   csr_w_en      == 0 &&
                   stall         == 0);

        // R-Type Control Signals
        if (present_state == ST_EXEC && opcode == OPCODE_OP_REG && !f_preempted)
            assert(alu_src_sel_a == ALU_SRC_SEL_A_RS1 &&
                   alu_src_sel_b == ALU_SRC_SEL_B_RS2 &&
                   rfile_w_sel   == RFILE_W_SEL_ALU_RESULT &&
                   alu_func      == {instrn[30], funct3} &&
                   pc_src_sel    == PC_SRC_SEL_ADDR_INC &&
                   pc_w_en       == 1 &&
                   rfile_w_en    == 1 &&
                   dmem_w_en     == 0 &&
                   dmem_r_en     == 0 &&
                   csr_w_en      == 0 &&
                   stall         == 0);

        // I-Type Control Signals
        if (present_state == ST_EXEC && opcode == OPCODE_OP_IMM && !f_preempted)
            assert(alu_src_sel_a == ALU_SRC_SEL_A_RS1 &&
                   alu_src_sel_b == ALU_SRC_SEL_B_I_TYPE_IMM &&
                   rfile_w_sel   == RFILE_W_SEL_ALU_RESULT &&
                   alu_func      == {(funct3 == FUNCT3_I_SRI) && instrn[30], funct3} &&
                   pc_src_sel    == PC_SRC_SEL_ADDR_INC &&
                   pc_w_en       == 1 &&
                   rfile_w_en    == 1 &&
                   dmem_w_en     == 0 &&
                   dmem_r_en     == 0 &&
                   csr_w_en      == 0 &&
                   stall         == 0);

        // JALR Control Signals
        if (present_state == ST_EXEC && opcode == OPCODE_JALR && !f_preempted)
            assert(rfile_w_sel   == RFILE_W_SEL_PC_ADDR_INC &&
                   pc_src_sel    == PC_SRC_SEL_JALR &&
                   pc_w_en       == 1 &&
                   rfile_w_en    == 1 &&
                   dmem_w_en     == 0 &&
                   dmem_r_en     == 0 &&
                   csr_w_en      == 0 &&
                   stall         == 0);

        // LOAD instruction (first cycle)
        if (present_state == ST_EXEC && opcode == OPCODE_LOAD && !f_preempted)
            assert(alu_src_sel_a == ALU_SRC_SEL_A_RS1 && 
                   alu_src_sel_b == ALU_SRC_SEL_B_I_TYPE_IMM &&
                   alu_func      == ALU_ADD &&
                   next_state    == ST_WR_BK &&
                   pc_w_en       == 0 &&
                   rfile_w_en    == 0 &&
                   dmem_w_en     == 0 &&
                   dmem_r_en     == 1 &&
                   csr_w_en      == 0 &&
                   stall         == 1);

        // LOAD instruction (write back)
        if (present_state == ST_WR_BK && !rst)
            assert(next_state    == ST_EXEC &&
                   pc_w_en       == 1 &&
                   rfile_w_en    == 1 &&
                   dmem_w_en     == 0 &&
                   dmem_r_en     == 0 &&
                   csr_w_en      == 0 &&
                   stall         == 0);

        // STORE instruction
        if (present_state == ST_EXEC && opcode == OPCODE_STORE && !f_preempted)
            assert(pc_w_en       == 1 &&
                   rfile_w_en    == 0 &&
                   dmem_w_en     == 1 &&
                   dmem_r_en     == 0 &&
                   csr_w_en      == 0 &&
                   stall         == 0);

        // Branch instruction
       if (present_state == ST_EXEC && opcode == OPCODE_BRANCH && !f_preempted) begin
            if (branch_taken) begin
                assert(addr_branch_alignment == '0 &&
                       pc_src_sel == PC_SRC_SEL_BRANCH);
            end else begin
                assert(pc_src_sel == PC_SRC_SEL_ADDR_INC);
            end
       end

        // Load Upper Immediate instruction
        if (present_state == ST_EXEC && opcode == OPCODE_LUI && !f_preempted)
            assert(alu_src_sel_a == ALU_SRC_SEL_A_UPPER_IMM &&
                   rfile_w_sel   == RFILE_W_SEL_ALU_RESULT &&
                   alu_func      == ALU_LUI &&
                   rfile_w_en    == 1);

        // Add Upper Immediate to PC instruction
        if (present_state == ST_EXEC && opcode == OPCODE_AUIPC && !f_preempted)
            assert(alu_src_sel_a == ALU_SRC_SEL_A_UPPER_IMM &&
                   alu_src_sel_b == ALU_SRC_SEL_B_PC_ADDR &&
                   rfile_w_sel   == RFILE_W_SEL_ALU_RESULT &&
                   alu_func      == ALU_ADD &&
                   rfile_w_en    == 1);

        // JAL instruction.
        if (present_state == ST_EXEC && opcode == OPCODE_JAL && !f_preempted)
            assert(pc_src_sel == PC_SRC_SEL_JAL && 
                   rfile_w_en == 1 && 
                   rfile_w_sel == RFILE_W_SEL_PC_ADDR_INC);

        //FIXME: fails to cover all legal Fences
        // Fence is valid
        if (present_state == ST_EXEC && (instrn == {PREFIX_FENCE, OPCODE_FENCE} || instrn == {PREFIX_FENCE_I, OPCODE_FENCE})) begin
            assert(!illegal_instrn);
        end

        // Property: For an ECALL instruction.
        if (present_state == ST_EXEC && instrn == 32'h00000073 && !f_preempted) // Full ECALL encoding
            assert(csr_op_sel == CSR_OP_ECALL && pc_src_sel == PC_SRC_SEL_MTVEC);

        //TODO: add remaining System Opcode checks
    end

    // --- Coverage Checks ---
    always @(posedge clk) begin
        if (!rst) begin
            cover(present_state == ST_EXEC && opcode == OPCODE_OP_REG && !f_preempted);
            cover(present_state == ST_EXEC && opcode == OPCODE_OP_IMM && !f_preempted);
            cover(present_state == ST_EXEC && opcode == OPCODE_JALR && !f_preempted);
            cover(present_state == ST_EXEC && opcode == OPCODE_LOAD && !f_preempted);
            cover(present_state == ST_EXEC && opcode == OPCODE_STORE && !f_preempted);
            cover(present_state == ST_EXEC && opcode == OPCODE_BRANCH && !f_preempted);
            cover(present_state == ST_EXEC && opcode == OPCODE_LUI && !f_preempted);
            cover(present_state == ST_EXEC && opcode == OPCODE_AUIPC && !f_preempted);
            cover(present_state == ST_EXEC && opcode == OPCODE_JAL && !f_preempted);
            cover(present_state == ST_EXEC && opcode == OPCODE_FENCE && !f_preempted);
            cover(present_state == ST_EXEC && opcode == OPCODE_SYSTEM && !f_preempted);
            cover(present_state == ST_WR_BK);
            cover(present_state == ST_EXEC && csr_intrpt_vld);
            cover(present_state == ST_EXEC && illegal_instrn && !csr_intrpt_vld);
        end
    end
`endif

endmodule
