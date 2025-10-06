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

module otter_cu_dcdr (
    input            clk,
    input            rst,
    input     [31:0] instrn,

    // Branch Signals
    input            br_eq, 
    input            br_lt, 
    input            br_ltu,

    // CSR Signals
    input            csr_intrpt_vld,
    input            csr_addr_vld,
    input            csr_read_only,

    // Alignment Signals
    input      [1:0] addr_store_alignment,
    input      [1:0] addr_load_alignment,
    input      [1:0] addr_jalr_alignment,
    input      [1:0] addr_branch_alignment,
    input      [1:0] addr_jal_alignment,

`ifdef RISCV_FORMAL
    output reg       intrpt_taken,
    output reg       trap_taken,
    output     [1:0] state,
`endif

    // selectors
    output reg [3:0] alu_func,
    output reg       alu_src_sel_a,
    output reg [1:0] alu_src_sel_b,
    output reg [1:0] rfile_w_sel,
    output reg [2:0] pc_src_sel,
    output reg [2:0] csr_op_sel,
    output reg [2:0] csr_trap_cause_sel,
    output reg [3:0] dmem_w_base_strb,

    // enablers
    output reg        pc_w_en, 
    output reg        rfile_w_en,
    output reg        dmem_w_en,
    output reg        dmem_r_en,
    output reg        csr_w_en,
    output reg        stall
);

    reg write_attempt;

    // decompose instruction into component parts
    wire [6:0] funct7   = `INSTRN_FUNCT7(instrn);
    wire [4:0] rs2_addr = `INSTRN_RS2_ADDR(instrn);
    wire [4:0] rs1_addr = `INSTRN_RS1_ADDR(instrn);
    wire [2:0] funct3   = `INSTRN_FUNCT3(instrn);
    wire [4:0] rd_addr  = `INSTRN_RD_ADDR(instrn);
    wire [6:0] opcode   = `INSTRN_OPCODE(instrn);

    // used for the fm field of the fence instruction
    wire [3:0] fm_fence = `INSTRN_FM_FENCE(instrn);

    // state variables and initial values
    reg [1:0] present_state, next_state;
    initial begin
        present_state = ST_INIT;
	    next_state    = ST_INIT;
    end

    // state update block
    always @(posedge clk) begin
        present_state <= next_state;
    end

`ifdef RISCV_FORMAL
    assign state = present_state;
`endif

    always @(*) begin

`ifdef RISCV_FORMAL
        intrpt_taken = '0;
        trap_taken   = '0;
`endif
        // selector defaults
        alu_func         = ALU_ADD; 
        alu_src_sel_a    = ALU_SRC_SEL_A_RS1; 
        alu_src_sel_b    = ALU_SRC_SEL_B_RS2;
        rfile_w_sel      = RFILE_W_SEL_PC_ADDR_INC; 
        pc_src_sel       = PC_SRC_SEL_ADDR_INC; 
        csr_trap_cause_sel   = TRAP_CAUSE_SEL_NO_TRAP; 
        csr_op_sel       = CSR_OP_WRITE;
        dmem_w_base_strb = '0;

        // enabler defaults
        pc_w_en      = '1; 
        rfile_w_en   = '0; 
        dmem_w_en    = '0; 
        dmem_r_en    = '0; 
        csr_w_en     = '0; 
        stall        = '0;

        // fsm decoder
        case (present_state)
            ST_INIT : begin
                pc_w_en    = '0;
                stall      = '1;
                next_state = ST_EXEC;
            end
            ST_EXEC :  begin
                next_state = ST_EXEC;

                case(opcode)
                    OPCODE_OP_REG : begin    // R-Type opcode
                        case ({funct7, funct3})  // arithmetic/logical operations including two registers
                            {FUNCT7_I_R_0, FUNCT3_R_ADD},
                            {FUNCT7_I_R_1, FUNCT3_R_SUB}, 
                            {FUNCT7_I_R_0, FUNCT3_R_SLL},
                            {FUNCT7_I_R_0, FUNCT3_R_SLT},
                            {FUNCT7_I_R_0, FUNCT3_R_SLTU},
                            {FUNCT7_I_R_0, FUNCT3_R_XOR},
                            {FUNCT7_I_R_0, FUNCT3_R_SRL},
                            {FUNCT7_I_R_1, FUNCT3_R_SRA},
                            {FUNCT7_I_R_0, FUNCT3_R_OR},
                            {FUNCT7_I_R_0, FUNCT3_R_AND} : begin
                                alu_src_sel_a = ALU_SRC_SEL_A_RS1;
                                alu_src_sel_b = ALU_SRC_SEL_B_RS2;
                                rfile_w_sel   = RFILE_W_SEL_ALU_RESULT;
                                alu_func      = {instrn[30], funct3}; 

                                rfile_w_en = '1;
                            end
                            default begin 
                                csr_trap_cause_sel = TRAP_CAUSE_SEL_INVLD_INSTRN;
                            end
                        endcase
                    end
                    OPCODE_OP_IMM : begin    // I-Type opcode *no loading
                        casez ({funct7, funct3}) // arithmetic/logical operations including a register and immediate
                            {FUNCT7_I_R_Z, FUNCT3_I_ADDI},
                            {FUNCT7_I_R_Z, FUNCT3_I_SLTI}, 
                            {FUNCT7_I_R_Z, FUNCT3_I_SLTIU}, 
                            {FUNCT7_I_R_Z, FUNCT3_I_ORI},
                            {FUNCT7_I_R_Z, FUNCT3_I_XORI}, 
                            {FUNCT7_I_R_Z, FUNCT3_I_ANDI}, 
                            {FUNCT7_I_R_0, FUNCT3_I_SLI}, 
                            {FUNCT7_I_R_0, FUNCT3_I_SRI},
                            {FUNCT7_I_R_1, FUNCT3_I_SRI} : begin
                                alu_src_sel_a = ALU_SRC_SEL_A_RS1; 
                                alu_src_sel_b = ALU_SRC_SEL_B_I_TYPE_IMM;
                                rfile_w_sel   = RFILE_W_SEL_ALU_RESULT;
                                alu_func      = {(funct3 == FUNCT3_I_SRI) && instrn[30], funct3};

                                rfile_w_en = '1;
                            end
                            default begin 
                                csr_trap_cause_sel = TRAP_CAUSE_SEL_INVLD_INSTRN;
                            end
                        endcase
                    end
                    OPCODE_JALR : begin    // I-Type opcode *jalr
                        if (funct3 == FUNCT3_I_JALR && addr_jalr_alignment == 2'b00) begin
                            rfile_w_sel = RFILE_W_SEL_PC_ADDR_INC;
                            pc_src_sel  = PC_SRC_SEL_JALR;

                            rfile_w_en = '1;
                        end else begin
                            csr_trap_cause_sel = TRAP_CAUSE_SEL_INSTRN_ADDR_MISALIGN;
                        end
                    end
                    OPCODE_LOAD : begin      // I-Type opcode *load instructions
                        alu_src_sel_a = ALU_SRC_SEL_A_RS1;
                        alu_src_sel_b = ALU_SRC_SEL_B_I_TYPE_IMM;
                        alu_func      = ALU_ADD;

                        casez ({funct3, addr_load_alignment})
                            {FUNCT3_I_LB,  2'bzz},
                            {FUNCT3_I_LH,  2'bz0}, 
                            {FUNCT3_I_LW,  2'b00},
                            {FUNCT3_I_LBU, 2'bzz}, 
                            {FUNCT3_I_LHU, 2'bz0} : begin
                                dmem_r_en  = '1;
                                pc_w_en    = '0;
                                // loads require a write back stall because of
                                // single-cycle memory read delay
                                stall      = '1; 
                                next_state = ST_WR_BK;
                            end
                            default begin 
                                // distinguish between faults caused by
                                // invalid instructions and addr misalign
                                case(funct3)
                                    FUNCT3_I_LB, FUNCT3_I_LH, FUNCT3_I_LW,
                                    FUNCT3_I_LBU, FUNCT3_I_LHU : begin
                                        csr_trap_cause_sel = TRAP_CAUSE_SEL_LOAD_ADDR_MISALIGN;
                                    end
                                    default : begin
                                        csr_trap_cause_sel = TRAP_CAUSE_SEL_INVLD_INSTRN;
                                    end
                                endcase
                            end
                        endcase
                    end
                    OPCODE_STORE : begin     // S-Type opcode *store instructions
                        alu_src_sel_a = ALU_SRC_SEL_A_RS1; 
                        alu_src_sel_b = ALU_SRC_SEL_B_S_TYPE_IMM;
                        alu_func      = ALU_ADD;

                        casez ({funct3, addr_store_alignment})
                            {FUNCT3_S_SB, 2'bzz},
                            {FUNCT3_S_SH, 2'bz0},
                            {FUNCT3_S_SW, 2'b00} : begin

                                case (funct3)
                                    FUNCT3_S_SB : dmem_w_base_strb = 4'b0001;
                                    FUNCT3_S_SH : dmem_w_base_strb = 4'b0011;
                                    FUNCT3_S_SW : dmem_w_base_strb = 4'b1111;
                                    default     : dmem_w_base_strb = 4'b0000;
                                endcase

                                dmem_w_en = '1;
                            end
                            default begin 
                                // distinguish between faults caused by
                                // invalid instructions and addr misalign
                                case(funct3)
                                    FUNCT3_S_SB, FUNCT3_S_SH, FUNCT3_S_SW : begin
                                        csr_trap_cause_sel = TRAP_CAUSE_SEL_STORE_ADDR_MISALIGN;
                                    end
                                    default : begin
                                        csr_trap_cause_sel = TRAP_CAUSE_SEL_INVLD_INSTRN;
                                    end
                                endcase
                            end
                        endcase
                    end
                    OPCODE_BRANCH : begin  // B-Type opcode
                        case (funct3) // All store instructions; writing from registers to memory
                            FUNCT3_B_BEQ, FUNCT3_B_BNE, FUNCT3_B_BLT,
                            FUNCT3_B_BGE, FUNCT3_B_BLTU, FUNCT3_B_BGEU : begin
                                if ( // branch taken
                                    (funct3 == FUNCT3_B_BEQ  &&  br_eq)  ||
                                    (funct3 == FUNCT3_B_BNE  && !br_eq)  ||
                                    (funct3 == FUNCT3_B_BLT  &&  br_lt)  ||
                                    (funct3 == FUNCT3_B_BGE  && !br_lt)  ||
                                    (funct3 == FUNCT3_B_BLTU &&  br_ltu) ||
                                    (funct3 == FUNCT3_B_BGEU && !br_ltu)
                                ) begin
                                    // only mark misaligned address if branch
                                    // taken (as per spec)
                                    if (addr_branch_alignment  == 2'b00) begin
                                        pc_src_sel = PC_SRC_SEL_BRANCH;
                                    end else begin
                                        csr_trap_cause_sel = TRAP_CAUSE_SEL_INSTRN_ADDR_MISALIGN;
                                    end
                                end
                            end
                            default begin
                                csr_trap_cause_sel = TRAP_CAUSE_SEL_INVLD_INSTRN;
                            end
                        endcase
                    end
                    OPCODE_LUI : begin
                        alu_src_sel_a = ALU_SRC_SEL_A_UPPER_IMM; // Extends a 20-bit immediate (extra 12-bits after)
                        rfile_w_sel   = RFILE_W_SEL_ALU_RESULT;
                        alu_func      = ALU_LUI;                     // Value passes through ALU and is stored in a register

                        rfile_w_en = '1;
                    end
                    OPCODE_AUIPC : begin
                        alu_src_sel_a = ALU_SRC_SEL_A_UPPER_IMM; // Adds a U-type immediate to the program count
                        alu_src_sel_b = ALU_SRC_SEL_B_PC_ADDR;   // which is stored in a register
                        rfile_w_sel   = RFILE_W_SEL_ALU_RESULT;
                        alu_func      = ALU_ADD;

                        rfile_w_en = '1;
                    end
                    OPCODE_JAL : begin
                        if (addr_jal_alignment == 2'b00) begin
                            rfile_w_sel = RFILE_W_SEL_PC_ADDR_INC; // stores current location + 4 in a register
                            pc_src_sel  = PC_SRC_SEL_JAL;          // Jumps to a new location (updates PC value)

                            rfile_w_en = '1;
                        end else begin
                            csr_trap_cause_sel = TRAP_CAUSE_SEL_INSTRN_ADDR_MISALIGN;
                        end
                    end
                    OPCODE_FENCE : begin
                        casez ({fm_fence, funct3})
                            {FM_FENCE, FUNCT3_FENCE},
                            {4'bzzzz, FUNCT3_FENCE_I} : begin
                                // nop
                            end
                            default : begin
                                csr_trap_cause_sel = TRAP_CAUSE_SEL_INVLD_INSTRN;
                            end
                        endcase
                    end
                    OPCODE_SYS : begin
                        case (funct3)
                            FUNCT3_SYS_CSRRW, FUNCT3_SYS_CSRRS, FUNCT3_SYS_CSRRC, 
                            FUNCT3_SYS_CSRRWI, FUNCT3_SYS_CSRRSI, FUNCT3_SYS_CSRRCI : begin

                                // Values shared between all CSR instructions
                                rfile_w_sel = RFILE_W_SEL_CSR_R_DATA;
                                rfile_w_en = '1;

                                // CSRRS and CSRRC are read-only compatible if rs1 is x0
                                write_attempt = (funct3 == FUNCT3_SYS_CSRRW)
                                    || (funct3 == FUNCT3_SYS_CSRRWI) 
                                    || (rs1_addr != 5'b0);

                                // illegal if csr dne or write to read-only
                                if (!csr_addr_vld || (write_attempt && csr_read_only)) begin
                                    csr_trap_cause_sel = TRAP_CAUSE_SEL_INVLD_INSTRN;
                                end else begin
                                    rfile_w_sel = RFILE_W_SEL_CSR_R_DATA;
                                    rfile_w_en  = 1'b1;
                                    csr_w_en    = write_attempt;
                                    csr_op_sel  = CSR_OP_WRITE;
                                end
                            end
                            FUNCT3_SYS_TRAPS : begin
                                case ({funct7, rs2_addr})
                                    FUNCT7_RS2_SYS_ECALL : begin
                                        pc_src_sel = PC_SRC_SEL_MTVEC;
                                        csr_op_sel = CSR_OP_ECALL;
                                    end
                                    FUNCT7_RS2_SYS_EBREAK : begin
                                        pc_src_sel = PC_SRC_SEL_MTVEC;
                                        csr_op_sel = CSR_OP_EBREAK;
                                    end
                                    FUNCT7_RS2_SYS_MRET : begin
                                        pc_src_sel = PC_SRC_SEL_MEPC;
                                        csr_op_sel = CSR_OP_MRET;
                                    end
                                    FUNCT7_RS2_SYS_WFI : begin
                                        // TODO: make this more than just
                                        // a nop
                                        csr_op_sel = CSR_OP_WFI;
                                    end
                                    default begin
                                        csr_trap_cause_sel = TRAP_CAUSE_SEL_INVLD_INSTRN;
                                    end
                                endcase
                            end
                            default : begin 
                                csr_trap_cause_sel = TRAP_CAUSE_SEL_INVLD_INSTRN;
                            end
                        endcase
                    end
                    default : begin 
                        csr_trap_cause_sel = TRAP_CAUSE_SEL_INVLD_INSTRN;
                    end
                endcase

                // interrupt case
                if (csr_intrpt_vld) begin
                    pc_src_sel = PC_SRC_SEL_MTVEC;
                    csr_op_sel = CSR_OP_INTRPT;
`ifdef RISCV_FORMAL
                    intrpt_taken   = '1;
`endif
                // trap case
                end else if (|csr_trap_cause_sel) begin
                    pc_src_sel = PC_SRC_SEL_MTVEC;
                    csr_op_sel = CSR_OP_TRAP;
`ifdef RISCV_FORMAL
                    trap_taken   = '1;
`endif
                end

                // Avoid writing back if interrupt/trap occurs
                if (csr_intrpt_vld || |csr_trap_cause_sel) begin
                    pc_w_en      = '1; 
                    rfile_w_en   = '0; 
                    dmem_w_en    = '0; 
                    dmem_r_en    = '0; 
                    csr_w_en     = '0; 
                    stall        = '0; 

                    next_state = ST_EXEC;
                end
            end
            ST_WR_BK : begin
                // memory reads require an extra clock cycle
                rfile_w_sel = RFILE_W_SEL_DMEM_R_DATA;
                rfile_w_en  = '1;
                next_state  = ST_EXEC;

            end
            default : begin 
                next_state = ST_INIT;
            end
        endcase


        // reset case (outside of fsm since it may occur any cycle)
        if (rst) begin
            pc_src_sel = PC_SRC_SEL_RESET_VEC;
            csr_op_sel = CSR_OP_RESET;

            pc_w_en      = '1;
            rfile_w_en   = '0; 
            dmem_w_en    = '0; 
            dmem_r_en    = '0; 
            csr_w_en     = '0; 
            stall        = '0; 

            next_state   = ST_INIT;
        end
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
                   pc_w_en       == '1 &&
                   rfile_w_en    == '0 &&
                   dmem_w_en     == '0 &&
                   dmem_r_en     == '0 &&
                   csr_w_en      == '0 &&
                   stall         == '0);

        // interrupts take priority over synchronous traps
        if (present_state == ST_EXEC && !rst && csr_intrpt_vld && illegal_instrn)
            assert(csr_op_sel == CSR_OP_INTRPT);

        // When any trap or interrupt is taken, architectural state writes must be disabled.
        if (present_state == ST_EXEC && !rst && csr_intrpt_vld && illegal_instrn)
            assert(next_state    == ST_EXEC &&
                   pc_w_en       == '1 &&
                   rfile_w_en    == '0 &&
                   dmem_w_en     == '0 &&
                   dmem_r_en     == '0 &&
                   csr_w_en      == '0 &&
                   stall         == '0);

        // R-Type Control Signals
        if (present_state == ST_EXEC && opcode == OPCODE_OP_REG && !f_preempted)
            assert(alu_src_sel_a == ALU_SRC_SEL_A_RS1 &&
                   alu_src_sel_b == ALU_SRC_SEL_B_RS2 &&
                   rfile_w_sel   == RFILE_W_SEL_ALU_RESULT &&
                   alu_func      == {instrn[30], funct3} &&
                   pc_src_sel    == PC_SRC_SEL_ADDR_INC &&
                   pc_w_en       == '1 &&
                   rfile_w_en    == '1 &&
                   dmem_w_en     == '0 &&
                   dmem_r_en     == '0 &&
                   csr_w_en      == '0 &&
                   stall         == '0);

        // I-Type Control Signals
        if (present_state == ST_EXEC && opcode == OPCODE_OP_IMM && !f_preempted)
            assert(alu_src_sel_a == ALU_SRC_SEL_A_RS1 &&
                   alu_src_sel_b == ALU_SRC_SEL_B_I_TYPE_IMM &&
                   rfile_w_sel   == RFILE_W_SEL_ALU_RESULT &&
                   alu_func      == {(funct3 == FUNCT3_I_SRI) && instrn[30], funct3} &&
                   pc_src_sel    == PC_SRC_SEL_ADDR_INC &&
                   pc_w_en       == '1 &&
                   rfile_w_en    == '1 &&
                   dmem_w_en     == '0 &&
                   dmem_r_en     == '0 &&
                   csr_w_en      == '0 &&
                   stall         == '0);

        // JALR Control Signals
        if (present_state == ST_EXEC && opcode == OPCODE_JALR && !f_preempted)
            assert(rfile_w_sel   == RFILE_W_SEL_PC_ADDR_INC &&
                   pc_src_sel    == PC_SRC_SEL_JALR &&
                   pc_w_en       == '1 &&
                   rfile_w_en    == '1 &&
                   dmem_w_en     == '0 &&
                   dmem_r_en     == '0 &&
                   csr_w_en      == '0 &&
                   stall         == '0);

        // LOAD instruction (first cycle)
        if (present_state == ST_EXEC && opcode == OPCODE_LOAD && !f_preempted)
            assert(alu_src_sel_a == ALU_SRC_SEL_A_RS1 && 
                   alu_src_sel_b == ALU_SRC_SEL_B_I_TYPE_IMM &&
                   alu_func      == ALU_ADD &&
                   next_state    == ST_WR_BK &&
                   pc_w_en       == '0 &&
                   rfile_w_en    == '0 &&
                   dmem_w_en     == '0 &&
                   dmem_r_en     == '1 &&
                   csr_w_en      == '0 &&
                   stall         == '1);

        // LOAD instruction (write back)
        if (present_state == ST_WR_BK && !rst)
            assert(next_state    == ST_EXEC &&
                   pc_w_en       == '1 &&
                   rfile_w_en    == '1 &&
                   dmem_w_en     == '0 &&
                   dmem_r_en     == '0 &&
                   csr_w_en      == '0 &&
                   stall         == '0);

        // STORE instruction
        if (present_state == ST_EXEC && opcode == OPCODE_STORE && !f_preempted)
            assert(pc_w_en       == '1 &&
                   rfile_w_en    == '0 &&
                   dmem_w_en     == '1 &&
                   dmem_r_en     == '0 &&
                   csr_w_en      == '0 &&
                   stall         == '0);

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
                   rfile_w_en    == '1);

        // Add Upper Immediate to PC instruction
        if (present_state == ST_EXEC && opcode == OPCODE_AUIPC && !f_preempted)
            assert(alu_src_sel_a == ALU_SRC_SEL_A_UPPER_IMM &&
                   alu_src_sel_b == ALU_SRC_SEL_B_PC_ADDR &&
                   rfile_w_sel   == RFILE_W_SEL_ALU_RESULT &&
                   alu_func      == ALU_ADD &&
                   rfile_w_en    == '1);

        // JAL instruction.
        if (present_state == ST_EXEC && opcode == OPCODE_JAL && !f_preempted)
            assert(pc_src_sel == PC_SRC_SEL_JAL && 
                   rfile_w_en == '1 && 
                   rfile_w_sel == RFILE_W_SEL_PC_ADDR_INC);

        // Fence is valid
        if (present_state == ST_EXEC && (instrn == {PREFIX_FENCE, OPCODE_FENCE} || instrn == {PREFIX_FENCE_I, OPCODE_FENCE})) begin
            assert(!illegal_instrn);
        end

        // Property: For an ECALL instruction.
        if (present_state == ST_EXEC && instrn == 32'h00000073 && !f_preempted) // Full ECALL encoding
            assert(csr_op_sel == CSR_OP_ECALL && pc_src_sel == PC_SRC_SEL_MTVEC);
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
