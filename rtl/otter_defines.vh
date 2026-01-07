`ifndef DEFINES
`define DEFINES

    //----------------//
    // INSTRN Defines
    //----------------//

    `define INSTRN_CSR_ADDR(instrn)   (instrn[31:20])
    `define INSTRN_CSR(instrn)        (instrn[31:20])
    `define INSTRN_MEM_SIZE(instrn)   (instrn[13:12])
    `define INSTRN_MEM_SIGN(instrn)   (instrn[14])
    `define INSTRN_FM_FENCE(instrn)   (instrn[31:28])

    `define INSTRN_FUNCT7(instrn)     (instrn[31:25])
    `define INSTRN_RS2_ADDR(instrn)   (instrn[24:20])
    `define INSTRN_RS1_ADDR(instrn)   (instrn[19:15])
    `define INSTRN_FUNCT3(instrn)     (instrn[14:12])
    `define INSTRN_RD_ADDR(instrn)    (instrn[11:7])
    `define INSTRN_OPCODE(instrn)     (instrn[6:0])

    // Opcodes
    localparam OPCODE_OP_REG = 7'b0110011;
    localparam OPCODE_OP_IMM = 7'b0010011;
    localparam OPCODE_JALR   = 7'b1100111;
    localparam OPCODE_LOAD   = 7'b0000011;
    localparam OPCODE_STORE  = 7'b0100011;
    localparam OPCODE_BRANCH = 7'b1100011;
    localparam OPCODE_LUI    = 7'b0110111;
    localparam OPCODE_AUIPC  = 7'b0010111;
    localparam OPCODE_JAL    = 7'b1101111;
    localparam OPCODE_FENCE  = 7'b0001111;
    localparam OPCODE_SYS    = 7'b1110011;

    // Opcode FUNCT3s
    localparam FUNCT3_I_JALR    = 3'b000;

    localparam FUNCT3_I_LB      = 3'b000;
    localparam FUNCT3_I_LH      = 3'b001;
    localparam FUNCT3_I_LW      = 3'b010;
    localparam FUNCT3_I_LBU     = 3'b100;
    localparam FUNCT3_I_LHU     = 3'b101;

    localparam FUNCT3_I_ADDI    = 3'b000;
    localparam FUNCT3_I_SLTI    = 3'b010;
    localparam FUNCT3_I_SLTIU   = 3'b011;
    localparam FUNCT3_I_ORI     = 3'b110;
    localparam FUNCT3_I_XORI    = 3'b100;
    localparam FUNCT3_I_ANDI    = 3'b111;
    localparam FUNCT3_I_SLI     = 3'b001;
    localparam FUNCT3_I_SRI     = 3'b101;

    localparam FUNCT3_B_BEQ     = 3'b000;
    localparam FUNCT3_B_BNE     = 3'b001;
    localparam FUNCT3_B_BLT     = 3'b100;
    localparam FUNCT3_B_BGE     = 3'b101;
    localparam FUNCT3_B_BLTU    = 3'b110;
    localparam FUNCT3_B_BGEU    = 3'b111;

    localparam FUNCT3_S_SB      = 3'b000;
    localparam FUNCT3_S_SH      = 3'b001;
    localparam FUNCT3_S_SW      = 3'b010;

    localparam FUNCT3_R_ADD     = 3'b000;
    localparam FUNCT3_R_SUB     = 3'b000;
    localparam FUNCT3_R_SLL     = 3'b001;
    localparam FUNCT3_R_SLT     = 3'b010;
    localparam FUNCT3_R_SLTU    = 3'b011;
    localparam FUNCT3_R_XOR     = 3'b100;
    localparam FUNCT3_R_SRL     = 3'b101;
    localparam FUNCT3_R_SRA     = 3'b101;
    localparam FUNCT3_R_OR      = 3'b110;
    localparam FUNCT3_R_AND     = 3'b111;

    localparam FUNCT3_FENCE     = 3'b000;
    localparam FUNCT3_FENCE_I   = 3'b001;

    localparam FUNCT3_SYS_CSRRW  = 3'b001;
    localparam FUNCT3_SYS_CSRRS  = 3'b010;
    localparam FUNCT3_SYS_CSRRC  = 3'b011;
    localparam FUNCT3_SYS_CSRRWI = 3'b101;
    localparam FUNCT3_SYS_CSRRSI = 3'b110;
    localparam FUNCT3_SYS_CSRRCI = 3'b111;
    localparam FUNCT3_SYS_TRAPS  = 3'b000;

    // Instruction FUNCT7s
    localparam FUNCT7_I_R_0 = 7'b0000000;
    localparam FUNCT7_I_R_1 = 7'b0100000;

    localparam FUNCT7_RS2_SYS_ECALL  = 12'h000;
    localparam FUNCT7_RS2_SYS_EBREAK = 12'h001;
    localparam FUNCT7_RS2_SYS_MRET   = 12'h302;
    localparam FUNCT7_RS2_SYS_WFI    = 12'h105;

    localparam DCDR_OP_SEL_REG     = 15'h0001;
    localparam DCDR_OP_SEL_IMM     = 15'h0002;
    localparam DCDR_OP_SEL_JALR    = 15'h0004;
    localparam DCDR_OP_SEL_LOAD    = 15'h0008;
    localparam DCDR_OP_SEL_STORE   = 15'h0010;
    localparam DCDR_OP_SEL_BRANCH  = 15'h0020;
    localparam DCDR_OP_SEL_LUI     = 15'h0040;
    localparam DCDR_OP_SEL_AUIPC   = 15'h0080;
    localparam DCDR_OP_SEL_JAL     = 15'h0100;
    localparam DCDR_OP_SEL_FENCE   = 15'h0200;
    localparam DCDR_OP_SEL_WRITE   = 15'h0400;
    localparam DCDR_OP_SEL_ECALL   = 15'h0800;
    localparam DCDR_OP_SEL_EBREAK  = 15'h1000;
    localparam DCDR_OP_SEL_MRET    = 15'h2000;
    localparam DCDR_OP_SEL_WFI     = 15'h4000;

    localparam DCDR_OP_REG_IDX     = 'd0;
    localparam DCDR_OP_IMM_IDX     = 'd1;
    localparam DCDR_OP_JALR_IDX    = 'd2;
    localparam DCDR_OP_LOAD_IDX    = 'd3;
    localparam DCDR_OP_STORE_IDX   = 'd4;
    localparam DCDR_OP_BRANCH_IDX  = 'd5;
    localparam DCDR_OP_LUI_IDX     = 'd6;
    localparam DCDR_OP_AUIPC_IDX   = 'd7;
    localparam DCDR_OP_JAL_IDX     = 'd8;
    localparam DCDR_OP_FENCE_IDX   = 'd9;
    localparam DCDR_OP_WRITE_IDX   = 'd10;
    localparam DCDR_OP_ECALL_IDX   = 'd11;
    localparam DCDR_OP_EBREAK_IDX  = 'd12;
    localparam DCDR_OP_MRET_IDX    = 'd13;
    localparam DCDR_OP_WFI_IDX     = 'd14;

    localparam DCDR_EXCP_SEL_ILLEGAL = 4'h1;
    localparam DCDR_EXCP_SEL_JUMP    = 4'h2;
    localparam DCDR_EXCP_SEL_STORE   = 4'h4;
    localparam DCDR_EXCP_SEL_LOAD    = 4'h8;

    localparam DCDR_EXCP_ILLEGAL_IDX = 'd0;
    localparam DCDR_EXCP_JUMP_IDX    = 'd1;
    localparam DCDR_EXCP_STORE_IDX   = 'd2;
    localparam DCDR_EXCP_LOAD_IDX    = 'd3;

    `define DCDR_OP_CSR_BITS(dcdr_op) (dcdr_op[14:10])

    localparam CSR_OP_SEL_WRITE   = 5'h01;
    localparam CSR_OP_SEL_ECALL   = 5'h02;
    localparam CSR_OP_SEL_EBREAK  = 5'h04;
    localparam CSR_OP_SEL_MRET    = 5'h08;
    localparam CSR_OP_SEL_WFI     = 5'h10;

    localparam CSR_OP_WRITE_IDX   = 'd0;
    localparam CSR_OP_SECALL_IDX  = 'd1;
    localparam CSR_OP_EBREAK_IDX  = 'd2;
    localparam CSR_OP_MRET_IDX    = 'd3;
    localparam CSR_OP_WFI_IDX     = 'd4;

    //--------------//
    // ALU Defines
    //--------------//

    localparam ALU_FUNC_SEL_ADD  = 4'd0;
    localparam ALU_FUNC_SEL_SUB  = 4'd1;
    localparam ALU_FUNC_SEL_OR   = 4'd2;
    localparam ALU_FUNC_SEL_AND  = 4'd3;
    localparam ALU_FUNC_SEL_XOR  = 4'd4;
    localparam ALU_FUNC_SEL_SRL  = 4'd5;
    localparam ALU_FUNC_SEL_SLL  = 4'd6;
    localparam ALU_FUNC_SEL_SRA  = 4'd7;
    localparam ALU_FUNC_SEL_SLT  = 4'd8;
    localparam ALU_FUNC_SEL_SLTU = 4'd9;
    localparam ALU_FUNC_SEL_LUI  = 4'd10;

    localparam ALU_SRC_A_SEL_RS1 = 1'd0;
    localparam ALU_SRC_A_SEL_PC  = 1'd1;

    localparam ALU_SRC_B_SEL_RS2 = 1'd0;
    localparam ALU_SRC_B_SEL_IMM = 1'd1;

    //--------------//
    // CSR Defines
    //--------------//

    // Read-Only Addresses
    localparam CSR_MVENDORID_ADDR  = 12'hF11;
    localparam CSR_MARCHID_ADDR    = 12'hF12;
    localparam CSR_MIMPID_ADDR     = 12'hF13;
    localparam CSR_MHARTID_ADDR    = 12'hF14;
    localparam CSR_MCONFIGPTR_ADDR = 12'hF15;

    // Trap Addresses
    localparam CSR_MSTATUS_ADDR  = 12'h300;
    localparam CSR_MISA_ADDR     = 12'h301;
    localparam CSR_MIE_ADDR      = 12'h304;
    localparam CSR_MTVEC_ADDR    = 12'h305;
    localparam CSR_MSTATUSH_ADDR = 12'h310;
    localparam CSR_MSCRATCH_ADDR = 12'h340;
    localparam CSR_MEPC_ADDR     = 12'h341;
    localparam CSR_MCAUSE_ADDR   = 12'h342;
    localparam CSR_MTVAL_ADDR    = 12'h343;
    localparam CSR_MIP_ADDR      = 12'h344;

	localparam CSR_MCYCLE_ADDR    = 12'hB00;
	localparam CSR_MINSTRET_ADDR  = 12'hB02;
	localparam CSR_MCYCLEH_ADDR   = 12'hB80;
	localparam CSR_MINSTRETH_ADDR = 12'hB82;

    // Selector for trap causes used for selecting MCAUSE
    localparam MCAUSE_SEL_INVLD_INSTRN         = 4'h1;
    localparam MCAUSE_SEL_INSTRN_ADDR_MISALIGN = 4'h2;
    localparam MCAUSE_SEL_STORE_ADDR_MISALIGN  = 4'h4;
    localparam MCAUSE_SEL_LOAD_ADDR_MISALIGN   = 4'h8;

    // MCAUSE Codes
    localparam MCAUSE_CODE_INSTRN_ADDR_MISALIGN = 32'h00000000;
    localparam MCAUSE_CODE_INVLD_INSTRN         = 32'h00000002;
    localparam MCAUSE_CODE_BREAKPOINT           = 32'h00000003;
    localparam MCAUSE_CODE_LOAD_ADDR_MISALIGN   = 32'h00000004;
    localparam MCAUSE_CODE_STORE_ADDR_MISALIGN  = 32'h00000006;
    localparam MCAUSE_CODE_ECALL_M_MODE         = 32'h0000000b;

    localparam MCAUSE_CODE_MACHINE_SOFTWARE_INTERRUPT = 32'h80000003;
    localparam MCAUSE_CODE_MACHINE_TIMER_INTERRUPT    = 32'h80000007;
    localparam MCAUSE_CODE_MACHINE_EXTERNAL_INTERRUPT = 32'h8000000b;

    // CSR Func Selectors
    localparam CSR_OP_NOP    = 3'd0;
    localparam CSR_OP_WRITE  = 3'd1;
    localparam CSR_OP_ECALL  = 3'd2;
    localparam CSR_OP_EBREAK = 3'd3;
    localparam CSR_OP_MRET   = 3'd4;
    localparam CSR_OP_WFI    = 3'd5;

    localparam CSR_FUNCT3_LOW_RW = 2'b01;
    localparam CSR_FUNCT3_LOW_RS = 2'b10;
    localparam CSR_FUNCT3_LOW_RC = 2'b11;
    localparam CSR_FUNCT3_HIGH_REG = 1'b0;
    localparam CSR_FUNCT3_HIGH_IMM = 1'b1;

    // Read-Only Values (MISA treated as read-only zero)
    localparam CSR_MVENDORID_VALUE  = 32'h0000_0000;
    localparam CSR_MARCHID_VALUE    = 32'h0000_0000;
    localparam CSR_MIMPID_VALUE     = 32'h0000_0000;
    localparam CSR_MHARTID_VALUE    = 32'h0000_0000;
    localparam CSR_MCONFIGPTR_VALUE = 32'h0000_0000;
    localparam CSR_MISA_VALUE       = 32'h4000_0100; // RV32I + Zicsr

    // All Register Masks
    localparam CSR_MSTATUS_MASK    = 32'h0000_0088; // bit 3 = MIE and bit 7 = MPIE are writeable, remaining bits are ignored
    localparam CSR_MSTATUSH_MASK   = 32'h0000_0000; // everything is left as default
    localparam CSR_MIP_MASK        = 32'hFFFF_0888; // bits 31-15 = Custom IRQs, bit 11 = External IRQ, bit 7 = Timer IRQ, bit 3 = Software IRQ
    localparam CSR_MIE_MASK        = 32'hFFFF_0888; // bits 31-15 = Custom IRQs, bit 11 = External IRQ, bit 7 = Timer IRQ, bit 3 = Software IRQ
    localparam CSR_MTVEC_MASK      = 32'hFFFF_FFFD; // bit 1 must always remain zero, MODE can only be 0 = Direct, 1 = Vec
    localparam CSR_MCAUSE_MASK     = 32'h0000_001F; // only lowest 5 bits of mcause are writeable
    localparam CSR_MEPC_MASK       = 32'hFFFF_FFFC; // 2 lsbs of mepc are not writeable
    localparam CSR_MTVAL_MASK      = 32'hFFFF_FFFF; // mtval is fully writeable by software
    localparam CSR_MSCRATCH_MASK   = 32'hFFFF_FFFF; // user-defined scratch space, the world is your oyster

    //--------------//
    // MEM Defines
    //--------------//

    localparam MEM_SIZE_BYTE   = 2'd0;
    localparam MEM_SIZE_H_WORD = 2'd1;
    localparam MEM_SIZE_WORD   = 2'd2;

    //--------------//
    // RFILE Defines
    //--------------//

    localparam RFILE_W_SEL_PC_ADDR_INC  = 2'd0;
    localparam RFILE_W_SEL_CSR_R_DATA   = 2'd1;
    localparam RFILE_W_SEL_DMEM_R_DATA  = 2'd2;
    localparam RFILE_W_SEL_ALU_RESULT   = 2'd3;

    //--------------//
    // PC Defines
    //--------------//

    localparam PC_SRC_SEL_ADDR_INC  = 2'd0;
    localparam PC_SRC_SEL_BR_JUMP   = 2'd1;
    localparam PC_SRC_SEL_EPC       = 2'd2;
    localparam PC_SRC_SEL_RESET_VEC = 2'd3;

    //-----------------//
    // IMM GEN Defines
    //-----------------//

    localparam IMM_GEN_SEL_UPPER  = 3'd0;
    localparam IMM_GEN_SEL_I_TYPE = 3'd1;
    localparam IMM_GEN_SEL_S_TYPE = 3'd2;
    localparam IMM_GEN_SEL_BRANCH = 3'd3;
    localparam IMM_GEN_SEL_JUMP   = 3'd4;

    //-----------------//
    // ADDR GEN Defines
    //-----------------//

    localparam ADDR_GEN_SEL_JAL    = 2'd0;
    localparam ADDR_GEN_SEL_BRANCH = 2'd1;
    localparam ADDR_GEN_SEL_JALR   = 2'd2;

    //--------------//
    // FSM Defines
    //--------------//

    localparam ST_INIT   = 2'd0;
    localparam ST_EXEC   = 2'd1;
    localparam ST_WR_BK  = 2'd2;

    //--------------//
    // RVFI Defines
    //--------------//

    `define RVFI_CSR_LIST \
        `CSR_MACRO_OP(mstatus)    \
        `CSR_MACRO_OP(misa)       \
        `CSR_MACRO_OP(mie)        \
        `CSR_MACRO_OP(mtvec)      \
        `CSR_MACRO_OP(mstatush)   \
        `CSR_MACRO_OP(mscratch)   \
        `CSR_MACRO_OP(mepc)       \
        `CSR_MACRO_OP(mcause)     \
        `CSR_MACRO_OP(mtval)      \
        `CSR_MACRO_OP(mip)        \
        `CSR_MACRO_OP(mvendorid)  \
        `CSR_MACRO_OP(marchid)    \
        `CSR_MACRO_OP(mimpid)     \
        `CSR_MACRO_OP(mhartid)    \
        `CSR_MACRO_OP(mconfigptr)

    `define RVFI_OUTPUTS                  \
        output reg        rvfi_valid,     \
        output reg [63:0] rvfi_order,     \
        output reg [31:0] rvfi_insn,      \
        output reg        rvfi_trap,      \
        output reg        rvfi_halt,      \
        output reg        rvfi_intr,      \
        output reg [ 1:0] rvfi_mode,      \
        output reg [ 1:0] rvfi_ixl,       \
        output reg [ 4:0] rvfi_rs1_addr,  \
        output reg [ 4:0] rvfi_rs2_addr,  \
        output reg [31:0] rvfi_rs1_rdata, \
        output reg [31:0] rvfi_rs2_rdata, \
        output reg [ 4:0] rvfi_rd_addr,   \
        output reg [31:0] rvfi_rd_wdata,  \
        output reg [31:0] rvfi_pc_rdata,  \
        output reg [31:0] rvfi_pc_wdata,  \
        output reg [31:0] rvfi_mem_addr,  \
        output reg [ 3:0] rvfi_mem_rmask, \
        output reg [ 3:0] rvfi_mem_wmask, \
        output reg [31:0] rvfi_mem_rdata, \
        output reg [31:0] rvfi_mem_wdata, \
        `RVFI_CSR_LIST

    `define RVFI_INTERCONNECTS                  \
        .rvfi_valid(rvfi_valid),     \
        .rvfi_order(rvfi_order),     \
        .rvfi_insn(rvfi_insn),      \
        .rvfi_trap(rvfi_trap),      \
        .rvfi_halt(rvfi_halt),      \
        .rvfi_intr(rvfi_intr),      \
        .rvfi_mode(rvfi_mode),      \
        .rvfi_ixl(rvfi_ixl),       \
        .rvfi_rs1_addr(rvfi_rs1_addr),  \
        .rvfi_rs2_addr(rvfi_rs2_addr),  \
        .rvfi_rs1_rdata(rvfi_rs1_rdata), \
        .rvfi_rs2_rdata(rvfi_rs2_rdata), \
        .rvfi_rd_addr(rvfi_rd_addr),   \
        .rvfi_rd_wdata(rvfi_rd_wdata),  \
        .rvfi_pc_rdata(rvfi_pc_rdata),  \
        .rvfi_pc_wdata(rvfi_pc_wdata),  \
        .rvfi_mem_addr(rvfi_mem_addr),  \
        .rvfi_mem_rmask(rvfi_mem_rmask), \
        .rvfi_mem_wmask(rvfi_mem_wmask), \
        .rvfi_mem_rdata(rvfi_mem_rdata), \
        .rvfi_mem_wdata(rvfi_mem_wdata), \
        `RVFI_CSR_LIST

`endif
