
`include "otter_defines.vh"

module otter_trap_gen (
    input [31:0] instrn,
    input [31:0] jal_addr,
    input [31:0] jalr_addr,
    input [31:0] branch_addr,
    input        mem_misalign,
    output reg   trap
);
    localparam LSB_MASK = 32'h0000_0003;

    always @(*) begin
        case (`INSTRN_OPCODE(instrn))
            OPCODE_JALR   : trap = |(jalr_addr   & LSB_MASK); // check for addr misalignment
            OPCODE_JAL    : trap = |(jal_addr    & LSB_MASK);
            OPCODE_BRANCH : trap = |(branch_addr & LSB_MASK);
            default : trap = '1;
        endcase
    end

endmodule

