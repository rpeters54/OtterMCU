

`ifndef DEFINES
`define DEFINES

    // Control Unit States
    localparam ST_INIT   = 3'd0;
    localparam ST_FETCH  = 3'd1;
    localparam ST_EXEC   = 3'd2;
    localparam ST_WR_BK  = 3'd3;
    localparam ST_INTRPT = 3'd4;

    // Opcodes
    localparam OPCODE_R_TYPE         = 7'b0110011;
    localparam OPCODE_I_TYPE_NO_LOAD = 7'b0010011;
    localparam OPCODE_I_TYPE_JALR    = 7'b1100111;
    localparam OPCODE_I_TYPE_LOAD    = 7'b0000011;
    localparam OPCODE_S_TYPE         = 7'b0100011;
    localparam OPCODE_B_TYPE         = 7'b1100011;
    localparam OPCODE_LUI            = 7'b0110111;
    localparam OPCODE_AUIPC          = 7'b0010111;
    localparam OPCODE_J_TYPE_JAL     = 7'b1101111;
    localparam OPCODE_INTRPT         = 7'b1110011;

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

`endif
