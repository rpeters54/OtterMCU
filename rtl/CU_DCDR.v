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

`include "Defines.svh"

module CU_DCDR (
    input      [6:0]   opcode,
    input      [14:12] func,
    input              instrn_bit_30, 
    input              intrpt_taken, 
    input              br_eq, 
    input              br_lt, 
    input              br_ltu,
    output reg [3:0]   alu_func,
    output reg         alu_src_sel_a,
    output reg [1:0]   alu_src_sel_b,
    output reg [2:0]   pc_src_sel,
    output reg [1:0]   rfile_w_sel
);

    reg comparison;
    
    always @(*) begin
        alu_func      = '0; 
        alu_src_sel_a = '0; 
        alu_src_sel_b = '0;
        pc_src_sel    = '0; 
        rfile_w_sel   = '0; 
        comparison    = '0;
	
        if (intrpt_taken == '1) begin  // Interrupt Case
            pc_src_sel = 'd4;
        end else begin
            case(opcode)
                OPCODE_OP_REG : begin    // R-Type opcode
                    alu_src_sel_a = 'd0; // arithmetic/logical operations including two registers
                    alu_src_sel_b = 'd0;
                    rfile_w_sel   = 'd3;
                    pc_src_sel    = 'd0;
                    alu_func = {instrn_bit_30, func}; // alu_func varies based on command
                end
                OPCODE_OP_IMM : begin    // I-Type opcode *no loading
                    alu_src_sel_a = 'd0; // arithmetic/logical operations including a register
                    alu_src_sel_b = 'd1; // and an immediate
                    rfile_w_sel   = 'd3;
                    pc_src_sel    = 'd0;
                    alu_func = {1'b0, func};  // alu_func varies based on command
		            if (func == 3'b101) begin // if-statement caused by immediate
                        alu_func = {instrn_bit_30, func};
	                end
                end
                OPCODE_JALR : begin    // I-Type opcode *jalr
                    pc_src_sel  = 'd1; // jumps and links to the value stored in rs1
                    rfile_w_sel = 'd0; // added to an I-Type immediate
                end
                OPCODE_LOAD : begin      // I-Type opcode *load instructions
                    alu_src_sel_a = 'd0; // All load instructions; writing from memory to registers
                    alu_src_sel_b = 'd1;
                    alu_func      = 'd0;
                    rfile_w_sel   = 'd2;
                    pc_src_sel    = 'd0;
                end
                OPCODE_STORE : begin     // S-Type opcode *store instructions
                    alu_src_sel_a = 'd0; // All store instructions; writing from registers to memory
                    alu_src_sel_b = 'd2;
                    alu_func      = 'd0;
                    pc_src_sel    = 'd0;
                end
                OPCODE_BRANCH : begin  // B-Type opcode
                    case(func[14:13])
                        2'd0    : comparison = br_eq;
                        2'd2    : comparison = br_lt;
                        2'd3    : comparison = br_ltu;
                        default : comparison = '0;
                    endcase
		            if (comparison != func[12]) begin // given instruction, output is based on condition
                        pc_src_sel = 'd2;
                    end
                end
                OPCODE_LUI : begin       // lui opcode
                    alu_src_sel_a = 'd1; // Extends a 20-bit immediate (extra 12-bits after)
                    alu_func      = 'd9; // Value passes through ALU and is stored in a register
                    rfile_w_sel   = 'd3;
                    pc_src_sel    = 'd0;
                end
                OPCODE_AUIPC : begin    // auipc opcode
                   alu_src_sel_a = 'd1; // Adds a U-type immediate to the program count
                   alu_src_sel_b = 'd3; // which is stored in a register
                   alu_func      = 'd0;
                   rfile_w_sel   = 'd3;
                   pc_src_sel    = 'd0;
                end
                OPCODE_JAL : begin    // J-Type opcode *jal
                   pc_src_sel  = 'd3; // Jumps to a new location (updates PC value)
                   rfile_w_sel = 'd0; // stores current location + 4 in a register
                end
                OPCODE_INTRPT : begin         // INTR opcode
                    if (func[12] == '1) begin // csrrw
                        rfile_w_sel = 'd1;           
                        pc_src_sel  = 'd0;
                    end else begin            // mret
                        pc_src_sel = 'd5;
                    end
                end
                default : begin end
            endcase
        end
    end
endmodule
