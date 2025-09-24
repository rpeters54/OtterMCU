`ifndef DEFINES
`define DEFINES

    //----------------//
    // INSTRN Defines
    //----------------//

    `define INSTRN_CSR(instrn)        (instrn[31:20])
    `define INSTRN_MEM_SIZE(instrn)   (instrn[13:12])
    `define INSTRN_MEM_SIGN(instrn)   (instrn[14])

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
    localparam OPCODE_FENCE  = 7'b0001111
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

    localparam FUNCT3_SYS_CSRRW  = 3'b001;
    localparam FUNCT3_SYS_CSRRS  = 3'b010;
    localparam FUNCT3_SYS_CSRRC  = 3'b011;
    localparam FUNCT3_SYS_CSRRWI = 3'b101;
    localparam FUNCT3_SYS_CSRRSI = 3'b110;
    localparam FUNCT3_SYS_CSRRCI = 3'b111;
    localparam FUNCT3_SYS_TRAPS  = 3'b000;

    // Instruction FUNCT7s
    localparam FUNCT7_I_R_Z = 7'bzzzzzzz;
    localparam FUNCT7_I_R_0 = 7'b0000000;
    localparam FUNCT7_I_R_1 = 7'b0100000;

    localparam FUNCT7_RS2_SYS_ECALL  = 12'h000;
    localparam FUNCT7_RS2_SYS_EBREAK = 12'h001;
    localparam FUNCT7_RS2_SYS_MRET   = 12'h302;
    localparam FUNCT7_RS2_SYS_WFI    = 12'h205;

    // Instruction Prefixes (everything except opcode)
    localparam PREFIX_FENCE   = 25'b0000_zzzz_zzzz_00000_001_00000
    localparam PREFIX_FENCE_I = 25'b0000_0000_0000_00000_001_00000

    //--------------//
    // ALU Defines
    //--------------//

    localparam ALU_ADD  = 4'b0000;
    localparam ALU_SUB  = 4'b1000;
    localparam ALU_OR   = 4'b0110;
    localparam ALU_AND  = 4'b0111;
    localparam ALU_XOR  = 4'b0100;
    localparam ALU_SRL  = 4'b0101;
    localparam ALU_SLL  = 4'b0001;
    localparam ALU_SRA  = 4'b1101;
    localparam ALU_SLT  = 4'b0010;
    localparam ALU_SLTU = 4'b0011;
    localparam ALU_LUI  = 4'b1001;

    localparam ALU_SRC_SEL_A_RS1       = 2'd0;
    localparam ALU_SRC_SEL_A_UPPER_IMM = 2'd1;
    localparam ALU_SRC_SEL_A_CSR_INPUT = 2'd2;

    localparam ALU_SRC_SEL_B_RS2         = 2'd0;
    localparam ALU_SRC_SEL_B_I_TYPE_IMM  = 2'd1;
    localparam ALU_SRC_SEL_B_S_TYPE_IMM  = 2'd2;
    localparam ALU_SRC_SEL_B_PC_ADDR     = 2'd3;
    localparam ALU_SRC_SEL_B_CSR_R_VALUE = 2'd4;

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


    // Read-Only Values (MISA treated as read-only zero)
    localparam CSR_MVENDORID_VALUE  = 32'h0000_0000;
    localparam CSR_MARCHID_VALUE    = 32'h0000_0000;
    localparam CSR_MIMPID_VALUE     = 32'h0000_0000;
    localparam CSR_MHARTID_VALUE    = 32'h0000_0000;
    localparam CSR_MCONFIGPTR_VALUE = 32'h0000_0000;


    // All Writeable Register Masks

    // bit 3 = MIE and bit 7 = MPIE are writeable, remaining bits are ignored
    localparam CSR_MSTATUS_MASK  = 32'h0000_0088;

    // everything is left as default
    localparam CSR_MSTATUSH_MASK = 32'h0000_0000;

    // XLEN = 32, no extensions
    localparam CSR_MISA_MASK     = 32'h0000_0000;

    // bit 11 = External IRQ, bit 7 = Timer IRQ, bit 3 = Software IRQ
    // bits 31-15 = Custom IRQs
    localparam CSR_MIE_MASK      = 32'hFFFF_0888;
    localparam CSR_MIP_MASK      = 32'hFFFF_0888;

    // bit 1 must always remain zero, MODE can only be 0 = Direct, 1 = Vec
    localparam CSR_MTVEC_MASK    = 32'hFFFF_FFFD;

    localparam CSR_MCAUSE_MASK   = 32'h0000_001F;
    localparam CSR_MEPC_MASK     = 32'hFFFF_FFFC;

    // behavior is user-defined, so ignoring for simplicity
    localparam CSR_MTVAL_MASK    = 32'h0000_0000;

    // user-defined scratch space, the world is your oyster
    localparam CSR_MSCRATCH_MASK = 32'hFFFF_FFFF;

    localparam CSR_MEIE_BIT = 32'h0000_0800;

    localparam MCAUSE_MACHINE_SOFTWARE_INTERRUPT = 32'h80000003;
    localparam MCAUSE_MACHINE_TIMER_INTERRUPT    = 32'h80000007;
    localparam MCAUSE_MACHINE_EXTERNAL_INTERRUPT = 32'h8000000b;

    localparam MCAUSE_INVALID_INSTRUCTION      = 32'h00000002;
    localparam MCAUSE_BREAKPOINT               = 32'h00000003;
    localparam MCAUSE_ECALL_M_MODE             = 32'h0000000b;

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

    `define PC_MEM_ADDR1(pc)    (pc[15:2])

    localparam PC_SRC_SEL_ADDR_INC     = 3'd0;
    localparam PC_SRC_SEL_JALR         = 3'd1;
    localparam PC_SRC_SEL_BRANCH       = 3'd2;
    localparam PC_SRC_SEL_JAL          = 3'd3;
    localparam PC_SRC_SEL_MTVEC_DIRECT = 3'd4;
    localparam PC_SRC_SEL_MEPC         = 3'd5;
    localparam PC_SRC_SEL_MEPC_VEC     = 3'd6;

    //--------------//
    // FSM Defines
    //--------------//

    localparam ST_INIT   = 3'd0;
    localparam ST_FETCH  = 3'd1;
    localparam ST_EXEC   = 3'd2;
    localparam ST_WR_BK  = 3'd3;
    localparam ST_INTRPT = 3'd4;

    //--------------//
    // RVFI Defines
    //--------------//

    `define RFVI_CSR_TRACE(NAME)                   \
        output reg [31:0] rvfi_csr_``NAME``_rmask, \
        output reg [31:0] rvfi_csr_``NAME``_wmask, \
        output reg [31:0] rvfi_csr_``NAME``_rdata, \
        output reg [31:0] rvfi_csr_``NAME``_wdata,

    `define RFVI_CSRS_TRACES  \
        `RFVI_CSR_TRACE(mie)  \
        `RFVI_CSR_TRACE(mepc) \
        `RFVI_CSR_TRACE(mtvec)

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
        `RFVI_CSRS_TRACES

`endif
