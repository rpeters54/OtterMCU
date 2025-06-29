

`ifndef OPCODES
`define OPCODES

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

`endif
