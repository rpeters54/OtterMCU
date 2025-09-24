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
    input             ext_intrpt,

    // Branch Signals
    input             br_eq, 
    input             br_lt, 
    input             br_ltu,

    // PC Signals
    input      [31:0] pc_addr,
    input      [31:0] pc_addr_inc,

    // Dmem Signal
    input      [31:0] dmem_r_data,

    // ALU Signals
    input      [31:0] alu_result,

    // RegFile Signals
    input      [31:0] rfile_r_rs1, 
    input      [31:0] rfile_r_rs2,

    // Immediate Signals
    input      [31:0] i_type_immed,
    input      [31:0] s_type_immed,
    input      [31:0] upper_immed,
    input      [31:0] z_immed,

    // Addr Gen Signals
    input      [31:0] jalr_addr,
    input      [31:0] branch_addr,
    input      [31:0] jal_addr,

    // CSR Signals
    input      [11:0] csr_addr,
    input             csr_valid,
    input             csr_read_only,
    input      [31:0] csr_r_data,
    input      [31:0] csr_mstatus_value,
    input      [31:0] csr_mie_value,
    input      [31:0] csr_mtvec_value,
    input      [31:0] csr_mepc_value,
    input      [31:0] csr_mcause_value,
    input      [31:0] csr_mip_value,

    // stateless outputs (dependent on decoded inst)
    output reg [3:0]  alu_func,
    output reg [31:0] alu_src_a,
    output reg [31:0] alu_src_b,
    output reg [31:0] rfile_w_data,
    output reg [31:0] pc_next_addr,
    output reg [3:0]  dmem_w_strb,

    // stateful outputs (dependent on fsm)
    output reg        pc_w_en, 
    output reg        rfile_w_en, 
    output reg        dmem_w_en, 
    output reg        dmem_r_en, 
    output reg        csr_w_en,

    // values passed back to csr needed to track interrupt changes
    output reg [31:0] csr_mstatus_next,
    output reg [31:0] csr_mie_next,
    output reg [31:0] csr_mtvec_next,
    output reg [31:0] csr_mepc_next,
    output reg [31:0] csr_mcause_next,
    output reg [31:0] csr_mip_next
);


    // Input value used for all CSR read-write instructions
    reg [31:0] csr_input_value;
    reg        intrpt_taken;

    // read and write address used for load and store validity checks
    wire [31:0] dmem_r_addr = rfile_r_rs1 + i_type_immed;
    wire [31:0] dmem_w_addr = rfile_r_rs1 + s_type_immed;

    // decompose instruction into component parts
    wire [6:0] funct7   = `INSTRN_FUNCT7(instrn);
    wire [4:0] rs2_addr = `INSTRN_RS2_ADDR(instrn);
    wire [4:0] rs1_addr = `INSTRN_RS1_ADDR(instrn);
    wire [2:0] funct3   = `INSTRN_FUNCT3(instrn);
    wire [4:0] rd_addr  = `INSTRN_RD_ADDR(instrn);
    wire [6:0] opcode   = `INSTRN_OPCODE(instrn);


    // state variables and initial values
    reg [2:0] present_state, next_state;
    initial begin
        present_state = ST_INIT;
	    next_state    = ST_INIT;
    end

    // state update block
    always @(posedge clk) begin
        if (rst == '1) begin
            present_state   <= ST_INIT;
        end else begin
            present_state   <= next_state;
        end
    end


    always @(*) begin

        // stateless defaults
        alu_func        = ALU_ADD; 
        alu_src_sel_a   = ALU_SRC_SEL_A_RS1; 
        alu_src_sel_b   = ALU_SRC_SEL_B_RS2;
        pc_src_sel      = PC_SRC_SEL_ADDR_INC; 
        rfile_w_sel     = RFILE_W_SEL_PC_ADDR_INC; 
        illegal_instrn  = '0;
        csr_input_value = '0;

        // stateful defaults
        pc_w_en      = '1; 
        rfile_w_en   = '0; 
        dmem_w_en    = '0; 
        dmem_r_en    = '0; 
        csr_w_en     = '0; 

        csr_mstatus_next = csr_mstatus_value;
        csr_mtvec_next   = csr_mtvec_value;
        csr_mepc_next    = csr_mepc_value;
        csr_mcause_next  = csr_mcause_value;

        // external interrupt check
        mip_next = {21'd0, ext_intrpt, 10'd0} & CSR_MIP_MASK;
        mie_next = {21'd0, ext_intrpt, 10'd0} & CSR_MIE_MASK; 
        intrpt_taken = |(mie_next & mip_next) & csr_mstatus_next[3];

        case (present_state)
            ST_INIT : begin
                pc_w_en    = '0; 
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
                                illegal_instrn = '1;
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
                                illegal_instrn = '1;
                            end
                        endcase
                    end
                    OPCODE_JALR : begin    // I-Type opcode *jalr
                        case (funct3) // jumps and links to the value stored in rs1 added to an I-Type immediate
                            FUNCT3_I_JALR : begin
                                rfile_w_sel = RFILE_W_SEL_PC_ADDR_INC;
                                pc_src_sel  = PC_SRC_SEL_JALR;

                                rfile_w_en = '1;
                            end
                            default begin 
                                illegal_instrn = '1;
                            end
                        endcase

                        // imem misalignment check
                        if (jalr_addr & 32'b11) begin
                            illegal_instrn = '1;
                        end
                    end
                    OPCODE_LOAD : begin      // I-Type opcode *load instructions
                        casez ({funct3, dmem_r_addr[1:0]}) // All load instructions; writing from memory to registers
                            {FUNCT3_I_LB,  2'bzz},
                            {FUNCT3_I_LH,  2'bz0}, 
                            {FUNCT3_I_LW,  2'b00},
                            {FUNCT3_I_LBU, 2'bzz}, 
                            {FUNCT3_I_LHU, 2'bz0} : begin
                                alu_src_sel_a = ALU_SRC_SEL_A_RS1;
                                alu_src_sel_b = ALU_SRC_SEL_B_I_TYPE_IMM;
                                rfile_w_sel   = RFILE_W_SEL_DMEM_R_DATA;
                                alu_func      = ALU_ADD;

                                dmem_r_en  = '1;
                                pc_w_en    = '0;
                                next_state = ST_WR_BK;
                            end
                            default begin 
                                illegal_instrn = '1;
                            end
                        endcase
                    end
                    OPCODE_STORE : begin     // S-Type opcode *store instructions
                        casez ({funct3, dmem_w_addr[1:0]}) // All store instructions; writing from registers to memory
                            {FUNCT3_S_SB, 2'bzz},
                            {FUNCT3_S_SH, 2'bz0},
                            {FUNCT3_S_SW, 2'b00} : begin
                                alu_src_sel_a = ALU_SRC_SEL_A_RS1; 
                                alu_src_sel_b = ALU_SRC_SEL_B_S_TYPE_IMM;
                                alu_func      = ALU_ADD;

                                dmem_w_strb = 4'b 1111;
                                case (insn_funct3)
                                    FUNCT3_S_SB: begin dmem_w_strb = 4'b 0001; end
                                    FUNCT3_S_SH: begin dmem_w_strb = 4'b 0011; end
                                    FUNCT3_S_SW: begin dmem_w_strb = 4'b 1111; end
                                endcase
                                dmem_w_strb = dmem_w_strb << dmem_w_addr[1:0];

                                dmem_w_en = '1;
                            end
                            default begin 
                                illegal_instrn = '1;
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
                                    pc_src_sel = PC_SRC_SEL_BRANCH;
                                end
                            end
                            default begin
                                illegal_instrn = '1;
                            end
                        endcase

                        // imem misalignment check
                        if (branch_addr & 32'b11) begin
                            illegal_instrn = '1;
                        end
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
                        rfile_w_sel = RFILE_W_SEL_PC_ADDR_INC; // stores current location + 4 in a register
                        pc_src_sel  = PC_SRC_SEL_JAL;          // Jumps to a new location (updates PC value)

                        rfile_w_en = '1;

                        // imem misalignment check
                        if (jal_addr & 32'b11) begin
                            illegal_instrn = '1;
                        end
                    end
                    OPCODE_FENCE : begin
                        casez ({funct7, rs2_addr, rs1_addr, funct3, rd_addr})
                            PREFIX_FENCE, PREFIX_FENCE_I : begin
                                // nop
                            end
                            default : begin
                                illegal_instrn = '1;
                            end
                        endcase
                    end
                    OPCODE_SYS : begin
                        case (funct3)
                            FUNCT3_SYS_CSRRW, FUNCT3_SYS_CSRRS, FUNCT3_SYS_CSRRC, 
                            FUNCT3_SYS_CSRRWI, FUNCT3_SYS_CSRRSI, FUNCT3_SYS_CSRRCI : begin

                                // Values shared between all CSR instructions
                                alu_src_sel_a = ALU_SRC_SEL_A_CSR_INPUT;
                                alu_src_sel_b = ALU_SRC_SEL_B_CSR_R_DATA;
                                rfile_w_sel = RFILE_W_SEL_CSR_R_DATA;
                                rfile_w_en = '1;

                                // Instruction specfic input and ALU op select
                                case (funct3)
                                    FUNCT3_SYS_CSRRW, FUNCT3_SYS_CSRRWI : begin
                                        alu_func = ALU_LUI;
                                    end
                                    FUNCT3_SYS_CSRRS, FUNCT3_SYS_CSRRSI : begin
                                        alu_func = ALU_OR;
                                    end
                                    FUNCT3_SYS_CSRRC, FUNCT3_SYS_CSRRCI : begin
                                        alu_func = ALU_AND;
                                    end
                                    default begin end
                                endcase

                                // Use RS1 or z_immed depending on the instrn
                                case (funct3)
                                    FUNCT3_SYS_CSRRW, FUNCT3_SYS_CSRRS  : begin
                                        csr_input_value = rfile_r_rs1;
                                    end
                                    FUNCT3_SYS_CSRRC : begin
                                        csr_input_value = ~rfile_r_rs1;
                                    end
                                    FUNCT3_SYS_CSRRWI, FUNCT3_SYS_CSRRSI : begin
                                        csr_input_value = z_immed;
                                    end
                                    FUNCT3_SYS_CSRRCI : begin
                                        csr_input_value = ~z_immed;
                                    end
                                    default begin end
                                endcase

                                // RS and RC are read-only if rd is x0
                                case (funct3)
                                    FUNCT3_SYS_CSRRW, FUNCT3_SYS_CSRRWI : begin
                                        csr_w_en = '1;
                                    end
                                    FUNCT3_SYS_CSRRS, FUNCT3_SYS_CSRRSI,
                                    FUNCT3_SYS_CSRRC, FUNCT3_SYS_CSRRCI : begin
                                        if (rd_addr == '0) begin
                                            csr_w_en = '0;
                                        end else begin
                                            csr_w_en = '1;
                                        end
                                    end
                                    default begin end
                                endcase

                                // illegal if csr dne or write to read-only
                                if (!csr_valid || csr_w_en && csr_read_only) begin
                                    illegal_instrn = '1;
                                end
                            end
                            FUNCT3_SYS_TRAPS : begin
                                case ({funct7, rs2_addr})
                                    FUNCT7_RS2_SYS_ECALL : begin
                                        pc_src_sel = PC_SRC_SEL_MTVEC_DIRECT;
                                        csr_mepc_next = pc_addr;
                                        csr_mcause_next = MCAUSE_ECALL_M_MODE;
                                        csr_mstatus_next[7] = csr_mstatus_value[3];
                                        csr_mstatus_next[3] = 0;
                                    end
                                    FUNCT7_RS2_SYS_EBREAK : begin
                                        pc_src_sel = PC_SRC_SEL_MTVEC_DIRECT;
                                        csr_mepc_next = pc_addr;
                                        csr_mcause_next = MCAUSE_BREAKPOINT;
                                        csr_mstatus_next[7] = csr_mstatus_value[3];
                                        csr_mstatus_next[3] = 0;
                                    end
                                    FUNCT7_RS2_SYS_MRET : begin
                                        pc_src_sel = PC_SRC_SEL_MEPC;
                                        csr_mcause_next = '0;
                                        csr_mstatus_next[3] = csr_mstatus_value[7];
                                    end
                                    FUNCT7_RS2_SYS_WFI : begin
                                        // nop
                                    end
                                    default begin
                                        illegal_instrn = '1;
                                    end
                                endcase
                            end
                            default : begin 
                                illegal_instrn = '1;
                            end
                        endcase
                    end
                    default : begin 
                        illegal_instrn = '1;
                    end
                endcase

                // interrupt case
                if (intrpt_taken) begin
                    if (csr_mtvec_value & 1) begin 
                        pc_src_sel = PC_SRC_SEL_MTVEC_VEC;
                    end else begin
                        pc_src_sel = PC_SRC_SEL_MTVEC_DIRECT;
                    end
                    csr_mepc_next = pc_addr;
                    csr_mcause_next = 1 << 31 | CSR_MEIE_BIT;
                    csr_mstatus_next[7] = 1;
                    csr_mstatus_next[3] = 0;
                // trap case
                end else if (illegal_instrn) begin
                    pc_src_sel = PC_SRC_SEL_MTVEC;
                    csr_mepc_next = pc_addr;
                    csr_mcause_next = MCAUSE_INVALID_INSTRUCTION;
                    csr_mstatus_next[7] = csr_mstatus_value[3];
                    csr_mstatus_next[3] = 0;
                end

                // Avoid writing back if interrupt/trap occurs
                if (intrpt_taken || illegal_instrn) begin
                    pc_w_en      = '1; 
                    rfile_w_en   = '0; 
                    dmem_w_en    = '0; 
                    dmem_r_en    = '0; 
                    csr_w_en     = '0; 

                    csr_mstatus_next = csr_mstatus_value;
                    csr_mie_next     = csr_mie_value;
                    csr_mtvec_next   = csr_mtvec_value;
                    csr_mepc_next    = csr_mepc_value;
                    csr_mcause_next  = csr_mcause_value;
                    csr_mip_next     = csr_mip_value;

                    next_state = ST_EXEC;
                end
            end
            ST_WR_BK : begin
                // memory reads require an extra clock cycle
                rfile_w_en = '1;
                next_state = ST_EXEC;
            end
            default : begin 
                next_state = ST_INIT;
            end
        endcase
    end


    // reg file write input MUX
    reg [1:0] rfile_w_sel;
    always @(*) begin
        case (rfile_w_sel) 
            RFILE_W_SEL_PC_ADDR_INC : rfile_w_data = pc_addr_inc;
            RFILE_W_SEL_CSR_R_DATA  : rfile_w_data = csr_r_data;
            RFILE_W_SEL_DMEM_R_DATA : rfile_w_data = dmem_r_data;
            RFILE_W_SEL_ALU_RESULT  : rfile_w_data = alu_result;
        endcase
    end

    // alu source MUXes
    reg [1:0] alu_src_sel_a;
    reg [2:0] alu_src_sel_b;
    always @(*) begin
        case (alu_src_sel_a)
            ALU_SRC_SEL_A_RS1       : alu_src_a = rfile_r_rs1;
            ALU_SRC_SEL_A_UPPER_IMM : alu_src_a = upper_immed;
            ALU_SRC_SEL_A_CSR_INPUT : alu_src_a = csr_input_value;
            default                 : alu_src_a = 32'hDEADDEAD;
        endcase
        case (alu_src_sel_b) 
            ALU_SRC_SEL_B_RS2         : alu_src_b = rfile_r_rs2;
            ALU_SRC_SEL_B_I_TYPE_IMM  : alu_src_b = i_type_immed;
            ALU_SRC_SEL_B_S_TYPE_IMM  : alu_src_b = s_type_immed;
            ALU_SRC_SEL_B_PC_ADDR     : alu_src_b = pc_addr;
            ALU_SRC_SEL_B_CSR_R_VALUE : alu_src_b = csr_r_value;
            default                   : alu_src_b = 32'hDEADDEAD;
        endcase
    end

    // bit-masked versions of the jump addresses for alignment safety
    wire [31:0] jalr_addr_masked    = jalr_addr    & 32'b11;
    wire [31:0] jal_addr_masked     = jal_addr     & 32'b11;
    wire [31:0] branch_addr_masked  = branch_addr  & 32'b11;

    // pc source MUX
    reg [2:0] pc_src_sel;
    always @(*) begin
        case(pc_src_sel)
            PC_SRC_SEL_ADDR_INC     : pc_next_addr = pc_addr_inc     & 32'b11;
            PC_SRC_SEL_JALR         : pc_next_addr = jalr_addr       & 32'b11;
            PC_SRC_SEL_BRANCH       : pc_next_addr = branch_addr     & 32'b11;
            PC_SRC_SEL_JAL          : pc_next_addr = jal_addr        & 32'b11;
            PC_SRC_SEL_MTVEC_DIRECT : pc_next_addr = csr_mtvec_value & 32'b11;
            PC_SRC_SEL_MEPC         : pc_next_addr = csr_mepc_value  & 32'b11;
            // Since the only implemented interrupt is external,
            // this is hardcoded to CSR_MEIE_BIT.
            // use interrupt bit location if more than one exists.
            PC_SRC_SEL_MTVEC_VEC    : pc_next_addr = csr_mtvec_value & 32'b11 + CSR_MEIE_BIT << 2;
            default                 : pc_next_addr = 32'hDEADDEAD;
        endcase
    end

`ifdef FORMAL

    // Default values for outputs (when no specific instruction matches)
    // The decoder initializes all outputs to '0'.
    always @(*) begin
        if (!trap_taken &&
            !(opcode == OPCODE_OP_REG && (
                {funct7, funct3} == FUNCT7_FUNCT3_R_ADD  ||
                {funct7, funct3} == FUNCT7_FUNCT3_R_SUB  ||
                {funct7, funct3} == FUNCT7_FUNCT3_R_SLL  ||
                {funct7, funct3} == FUNCT7_FUNCT3_R_SLT  ||
                {funct7, funct3} == FUNCT7_FUNCT3_R_SLTU ||
                {funct7, funct3} == FUNCT7_FUNCT3_R_XOR  ||
                {funct7, funct3} == FUNCT7_FUNCT3_R_SRL  ||
                {funct7, funct3} == FUNCT7_FUNCT3_R_SRA  ||
                {funct7, funct3} == FUNCT7_FUNCT3_R_OR   ||
                {funct7, funct3} == FUNCT7_FUNCT3_R_AND
            )) &&
            !(opcode == OPCODE_OP_IMM && (
                func == FUNCT3_I_ADDI  ||
                func == FUNCT3_I_SLTI  ||
                func == FUNCT3_I_SLTIU ||
                func == FUNCT3_I_ORI   ||
                func == FUNCT3_I_XORI  ||
                func == FUNCT3_I_ANDI  ||
                func == FUNCT3_I_SLI
            )) &&
            !(opcode == OPCODE_OP_IMM && func == FUNCT3_I_SRI && (funct7 == FUNCT7_I_R_0 || funct7 == FUNCT7_I_R_1)) &&
            !(opcode == OPCODE_JALR && func == FUNCT3_I_JALR) &&
            !(opcode == OPCODE_LOAD && (
                func == FUNCT3_I_LB  ||
                func == FUNCT3_I_LH  ||
                func == FUNCT3_I_LW  ||
                func == FUNCT3_I_LBU ||
                func == FUNCT3_I_LHU
            )) &&
            !(opcode == OPCODE_STORE && (
                func == FUNCT3_S_SB ||
                func == FUNCT3_S_SH ||
                func == FUNCT3_S_SW
            )) &&
            !(opcode == OPCODE_BRANCH && (
                `FUNCT3_BRANCH_BASE(func) == `FUNCT3_BRANCH_BASE(FUNCT3_B_BEQ) ||
                `FUNCT3_BRANCH_BASE(func) == `FUNCT3_BRANCH_BASE(FUNCT3_B_BLT) ||
                `FUNCT3_BRANCH_BASE(func) == `FUNCT3_BRANCH_BASE(FUNCT3_B_BLTU)
            )) &&
            !(opcode == OPCODE_LUI) &&
            !(opcode == OPCODE_AUIPC) &&
            !(opcode == OPCODE_JAL) &&
            !(opcode == OPCODE_SYS && func == FUNCT3_SYS_CSRRW) &&
            !(opcode == OPCODE_SYS && func == FUNCT3_SYS_MRET && instrn[31:7] == FUNCT7_SYS_MRET)
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
        if (trap_taken == 1'b1) begin
            assert(pc_src_sel    == PC_SRC_SEL_MTVEC);
            assert(alu_func      == '0);
            assert(alu_src_sel_a == '0);
            assert(alu_src_sel_b == '0);
            assert(rfile_w_sel   == '0);
        end
    end

    reg r_type_valid;
    always @(*) begin
        if (trap_taken == 1'b0 && opcode == OPCODE_OP_REG) begin

            // Specific R-type func checks
            case ({funct7, funct3})
                FUNCT7_FUNCT3_R_ADD  : begin assert(alu_func == ALU_ADD);  r_type_valid = 1; end 
                FUNCT7_FUNCT3_R_SUB  : begin assert(alu_func == ALU_SUB);  r_type_valid = 1; end  
                FUNCT7_FUNCT3_R_SLL  : begin assert(alu_func == ALU_SLL);  r_type_valid = 1; end
                FUNCT7_FUNCT3_R_SRL  : begin assert(alu_func == ALU_SRL);  r_type_valid = 1; end
                FUNCT7_FUNCT3_R_SRA  : begin assert(alu_func == ALU_SRA);  r_type_valid = 1; end
                FUNCT7_FUNCT3_R_XOR  : begin assert(alu_func == ALU_XOR);  r_type_valid = 1; end
                FUNCT7_FUNCT3_R_OR   : begin assert(alu_func == ALU_OR);   r_type_valid = 1; end
                FUNCT7_FUNCT3_R_AND  : begin assert(alu_func == ALU_AND);  r_type_valid = 1; end
                FUNCT7_FUNCT3_R_SLT  : begin assert(alu_func == ALU_SLT);  r_type_valid = 1; end
                FUNCT7_FUNCT3_R_SLTU : begin assert(alu_func == ALU_SLTU); r_type_valid = 1; end
                default         : begin assert(alu_func == '0);       r_type_valid = 0; end
            endcase

            if (r_type_valid) begin
                // Common R-type outputs
                assert(alu_src_sel_a == ALU_SRC_SEL_A_RS1);
                assert(alu_src_sel_b == ALU_SRC_SEL_B_RS2);
                assert(rfile_w_sel   == RFILE_W_SEL_ALU_RESULT);
                assert(pc_src_sel    == PC_SRC_SEL_ADDR_INC);
            end
        end
    end

`endif

endmodule
