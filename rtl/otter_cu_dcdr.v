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
    input             trap_taken, 

    input             br_eq, 
    input             br_lt, 
    input             br_ltu,

    input      [31:0] pc_addr_inc,
    input      [31:0] csr_r_data,
    input      [31:0] mem_data_out,
    input      [31:0] alu_result,

    input      [31:0] rfile_r_rs1, 
    input      [31:0] upper_immed,

    input      [31:0] rfile_r_rs2,
    input      [31:0] i_type_immed,
    input      [31:0] s_type_immed,
    input      [31:0] pc_addr,

    input      [31:0] jalr, 
    input      [31:0] branch, 
    input      [31:0] jal, 
    input      [31:0] mtvec, 
    input      [31:0] mepc,

    output reg [3:0]  alu_func,
    output reg [31:0] alu_src_a,
    output reg [31:0] alu_src_b,
    output reg [31:0] rfile_w_data,
    output reg [31:0] pc_next_addr,
    output reg [3:0]  dmem_w_strb,

    output reg        mret_instrn,
    output reg        ecall_instrn,
    output reg        ebreak_instrn,
    output reg        illegal_instrn
);


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

    always @(*) begin
        alu_func       = ALU_ADD; 
        alu_src_sel_a  = ALU_SRC_SEL_A_RS1; 
        alu_src_sel_b  = ALU_SRC_SEL_B_RS2;
        pc_src_sel     = PC_SRC_SEL_ADDR_INC; 
        rfile_w_sel    = RFILE_W_SEL_PC_ADDR_INC; 
        mret_instrn    = '0;
        ecall_instrn   = '0;
        ebreak_instrn  = '0;
        illegal_instrn = '0;

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
                    end
                    default begin 
                        illegal_instrn = '1;
                    end
                endcase
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
                        rfile_w_sel   = RFILE_W_SEL_MEM_DATA_OUT;
                        alu_func      = ALU_ADD;
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
            end
            OPCODE_LUI : begin
                alu_src_sel_a = ALU_SRC_SEL_A_UPPER_IMM; // Extends a 20-bit immediate (extra 12-bits after)
                rfile_w_sel   = RFILE_W_SEL_ALU_RESULT;
                alu_func      = ALU_LUI;                     // Value passes through ALU and is stored in a register
            end
            OPCODE_AUIPC : begin
               alu_src_sel_a = ALU_SRC_SEL_A_UPPER_IMM; // Adds a U-type immediate to the program count
               alu_src_sel_b = ALU_SRC_SEL_B_PC_ADDR;   // which is stored in a register
               rfile_w_sel   = RFILE_W_SEL_ALU_RESULT;
               alu_func      = ALU_ADD;
            end
            OPCODE_JAL : begin
               rfile_w_sel = RFILE_W_SEL_PC_ADDR_INC; // stores current location + 4 in a register
               pc_src_sel  = PC_SRC_SEL_JAL;          // Jumps to a new location (updates PC value)
            end
            OPCODE_FENCE : begin
                casez ({funct7, rs2_addr, rs1_addr, funct3, rd_addr})
                    PREFIX_FENCE : begin
                        // nop
                    end
                    PREFIX_FENCE_I : begin
                        // nop
                    end
                    default : begin
                        illegal_instrn = '1;
                    end
                endcase
            end
            OPCODE_SYS : begin
                case (funct3)
                    FUNCT3_SYS_CSRRW : begin
                        rfile_w_sel = RFILE_W_SEL_CSR_R_DATA;
                        pc_src_sel  = PC_SRC_SEL_ADDR_INC;
                    end
                    FUNCT3_SYS_TRAPS : begin
                        case ({funct7, rs2_addr})
                            FUNCT7_RS2_SYS_ECALL : begin
                                pc_src_sel = PC_SRC_SEL_MTVEC;
                                ecall_instrn = '1;
                            end
                            FUNCT7_RS2_SYS_EBREAK : begin
                                pc_src_sel = PC_SRC_SEL_MTVEC;
                                ebreak_instrn = '1;
                            end
                            FUNCT7_RS2_SYS_MRET : begin
                                pc_src_sel = PC_SRC_SEL_MEPC;
                                mret_instrn = '1;
                            end
                            FUNCT7_RS2_SYS_WFI : begin
                                // treated as nop
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

        if (trap_taken == '1) begin  // Interrupt Case
            pc_src_sel = PC_SRC_SEL_MTVEC;
        end
    end


    // reg file write input MUX
    reg [1:0] rfile_w_sel;
    always @(*) begin
        case (rfile_w_sel) 
            RFILE_W_SEL_PC_ADDR_INC  : rfile_w_data = pc_addr_inc;
            RFILE_W_SEL_CSR_R_DATA   : rfile_w_data = csr_r_data;
            RFILE_W_SEL_MEM_DATA_OUT : rfile_w_data = mem_data_out;
            RFILE_W_SEL_ALU_RESULT   : rfile_w_data = alu_result;
        endcase
    end

    // alu source MUXes
    reg       alu_src_sel_a;
    reg [1:0] alu_src_sel_b;
    always @(*) begin
        case (alu_src_sel_a)
            ALU_SRC_SEL_A_RS1       : alu_src_a = rfile_r_rs1;
            ALU_SRC_SEL_A_UPPER_IMM : alu_src_a = upper_immed;
        endcase
        case (alu_src_sel_b) 
            ALU_SRC_SEL_B_RS2        : alu_src_b = rfile_r_rs2;
            ALU_SRC_SEL_B_I_TYPE_IMM : alu_src_b = i_type_immed;
            ALU_SRC_SEL_B_S_TYPE_IMM : alu_src_b = s_type_immed;
            ALU_SRC_SEL_B_PC_ADDR    : alu_src_b = pc_addr;
        endcase
    end

    // pc source MUX
    reg [2:0] pc_src_sel;
    always @(*) begin
        case(pc_src_sel)
            PC_SRC_SEL_ADDR_INC : pc_next_addr = pc_addr_inc;
            PC_SRC_SEL_JALR     : pc_next_addr = jalr;
            PC_SRC_SEL_BRANCH   : pc_next_addr = branch;
            PC_SRC_SEL_JAL      : pc_next_addr = jal;
            PC_SRC_SEL_MTVEC    : pc_next_addr = mtvec;
            PC_SRC_SEL_MEPC     : pc_next_addr = mepc;
            default             : pc_next_addr = 32'hDEADDEAD;
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
