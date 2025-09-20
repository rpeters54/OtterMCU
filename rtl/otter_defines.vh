

`ifndef DEFINES
`define DEFINES


    `define INSTRN_OPCODE(instrn)     (instrn[6:0])
    `define INSTRN_I_R_PREFIX(instrn) (instrn[31:25])
    `define INSTRN_FUNC(instrn)       (instrn[14:12])
    `define INSTRN_CSR(instrn)        (instrn[31:20])
    `define INSTRN_RS1_ADDR(instrn)   (instrn[19:15])
    `define INSTRN_RS2_ADDR(instrn)   (instrn[24:20])
    `define INSTRN_RSD_ADDR(instrn)   (instrn[11:7])

    `define FUNC_BRANCH_BASE(func)    (func[2:1])
    `define FUNC_BRANCH_SEL(func)     (func[0])

    // Control Unit States
    localparam ST_INIT   = 3'd0;
    localparam ST_FETCH  = 3'd1;
    localparam ST_EXEC   = 3'd2;
    localparam ST_WR_BK  = 3'd3;
    localparam ST_INTRPT = 3'd4;

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
    localparam OPCODE_SYS    = 7'b1110011;

    // Opcode Funcs
    localparam FUNC_I_JALR    = 3'b000;

    localparam FUNC_I_LB      = 3'b000;
    localparam FUNC_I_LH      = 3'b001;
    localparam FUNC_I_LW      = 3'b010;
    localparam FUNC_I_LBU     = 3'b100;
    localparam FUNC_I_LHU     = 3'b101;

    localparam FUNC_I_ADDI    = 3'b000;
    localparam FUNC_I_SLTI    = 3'b010;
    localparam FUNC_I_SLTIU   = 3'b011;
    localparam FUNC_I_ORI     = 3'b110;
    localparam FUNC_I_XORI    = 3'b100;
    localparam FUNC_I_ANDI    = 3'b111;
    localparam FUNC_I_SLI     = 3'b001;
    localparam FUNC_I_SRI     = 3'b101;

    localparam FUNC_B_BEQ     = 3'b000;
    localparam FUNC_B_BNE     = 3'b001;
    localparam FUNC_B_BLT     = 3'b100;
    localparam FUNC_B_BGE     = 3'b101;
    localparam FUNC_B_BLTU    = 3'b110;
    localparam FUNC_B_BGEU    = 3'b111;

    localparam FUNC_S_SB      = 3'b000;
    localparam FUNC_S_SH      = 3'b001;
    localparam FUNC_S_SW      = 3'b010;

    localparam FUNC_R_ADD     = 3'b000;
    localparam FUNC_R_SUB     = 3'b000;
    localparam FUNC_R_SLL     = 3'b001;
    localparam FUNC_R_SLT     = 3'b010;
    localparam FUNC_R_SLTU    = 3'b011;
    localparam FUNC_R_XOR     = 3'b100;
    localparam FUNC_R_SRL     = 3'b101;
    localparam FUNC_R_SRA     = 3'b101;
    localparam FUNC_R_OR      = 3'b110;
    localparam FUNC_R_AND     = 3'b111;

    localparam FUNC_SYS_CSRRW = 3'b001;
    localparam FUNC_SYS_MRET  = 3'b000;

    // Instruction Prefixes
    localparam PREFIX_I_R_0 = 7'b0000000;
    localparam PREFIX_I_R_1 = 7'b0100000;

    localparam PREFIX_SYS_MRET    = 25'h061_0000;

    localparam PRE_FUNC_R_ADD     = { PREFIX_I_R_0, FUNC_R_ADD };
    localparam PRE_FUNC_R_SUB     = { PREFIX_I_R_1, FUNC_R_SUB };
    localparam PRE_FUNC_R_SLL     = { PREFIX_I_R_0, FUNC_R_SLL };
    localparam PRE_FUNC_R_SLT     = { PREFIX_I_R_0, FUNC_R_SLT };
    localparam PRE_FUNC_R_SLTU    = { PREFIX_I_R_0, FUNC_R_SLTU};
    localparam PRE_FUNC_R_XOR     = { PREFIX_I_R_0, FUNC_R_XOR };
    localparam PRE_FUNC_R_SRL     = { PREFIX_I_R_0, FUNC_R_SRL };
    localparam PRE_FUNC_R_SRA     = { PREFIX_I_R_1, FUNC_R_SRA };
    localparam PRE_FUNC_R_OR      = { PREFIX_I_R_0, FUNC_R_OR  };
    localparam PRE_FUNC_R_AND     = { PREFIX_I_R_0, FUNC_R_AND };

    

    // ALU Functions
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

    // CSR Register Addresses
    localparam CSR_MIE_ADDR   = 12'h304;
    localparam CSR_MTVEC_ADDR = 12'h305;
    localparam CSR_MEPC_ADDR  = 12'h341;

    // Values for the MEM_SIZE parameters
    localparam MEM_SIZE_BYTE   = 2'd0;
    localparam MEM_SIZE_H_WORD = 2'd1;
    localparam MEM_SIZE_WORD   = 2'd2;

    // DCDR Defines
    localparam DCDR_ALU_SRC_SEL_A_RS1       = 1'd0;
    localparam DCDR_ALU_SRC_SEL_A_UPPER_IMM = 1'd1;

    localparam DCDR_ALU_SRC_SEL_B_RS2        = 2'd0;
    localparam DCDR_ALU_SRC_SEL_B_I_TYPE_IMM = 2'd1;
    localparam DCDR_ALU_SRC_SEL_B_S_TYPE_IMM = 2'd2;
    localparam DCDR_ALU_SRC_SEL_B_PC_ADDR    = 2'd3;
    
    localparam DCDR_RFILE_W_SEL_PC_ADDR_INC  = 2'd0;
    localparam DCDR_RFILE_W_SEL_CSR_R_DATA   = 2'd1;
    localparam DCDR_RFILE_W_SEL_MEM_DATA_OUT = 2'd2;
    localparam DCDR_RFILE_W_SEL_ALU_RESULT   = 2'd3;

    localparam DCDR_PC_SRC_SEL_ADDR_INC = 3'd0;
    localparam DCDR_PC_SRC_SEL_JALR     = 3'd1;
    localparam DCDR_PC_SRC_SEL_BRANCH   = 3'd2;
    localparam DCDR_PC_SRC_SEL_JAL      = 3'd3;
    localparam DCDR_PC_SRC_SEL_MTVEC    = 3'd4;
    localparam DCDR_PC_SRC_SEL_MEPC     = 3'd5;



    //--------------//
    // RVFI Defines
    //--------------//

    `define RFVI_CSR_TRACE(NAME)                   \
        output reg [31:0] rvfi_csr_``NAME``_rmask, \
        output reg [31:0] rvfi_csr_``NAME``_wmask, \
        output reg [31:0] rvfi_csr_``NAME``_rdata, \
        output reg [31:0] rvfi_csr_``NAME``_wdata,

    `define RFVI_CSRS_TRACES /* Machine Trap CSRs */ \
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
