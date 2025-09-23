`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/04/2022 08:07:08 PM
// Design Name: 
// Module Name: OTTER_MCU
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

module otter_mcu (
    input         clk,
    input         rst, 
    input         intrpt, 

`ifdef RISCV_FORMAL
    `RVFI_OUTPUTS
`endif

    input  [31:0] imem_r_data,
    output        imem_r_en,
    output [31:0] imem_addr,

    input  [31:0] dmem_r_data,
    output        dmem_r_en,
    output        dmem_w_en,
    output [3:0]  dmem_w_strb,
    output [31:0] dmem_addr,
    output [31:0] dmem_w_data
);

    //---------------------------------------------------------//
    // Program Counter: keeps track of the current instruction
    //---------------------------------------------------------//

    wire [31:0] pc_next_addr, pc_addr, pc_addr_inc;
    wire pc_w_en;

    otter_pc pc (
        .clk(clk),
        .rst(rst),
        .w_en(pc_w_en),
        .next_addr(pc_next_addr),
        .addr(pc_addr),
        .addr_inc(pc_addr_inc)
    );

    //----------------------------------------------------------------------//
    // REG_FILE w/ Input MUX: contains all registers needed for the program
    //----------------------------------------------------------------------//

    reg [31:0] rfile_w_data;
    wire [31:0] rfile_r_rs1, rfile_r_rs2;
    wire rfile_w_en;

    wire [4:0] rfile_r_addr1 = `INSTRN_RS1_ADDR(imem_r_data);
    wire [4:0] rfile_r_addr2 = `INSTRN_RS2_ADDR(imem_r_data);
    wire [4:0] rfile_w_addr  = `INSTRN_RD_ADDR(imem_r_data);

    otter_rfile rf (
        .clk(clk),
        .r_addr1(rfile_r_addr1), 
        .r_addr2(rfile_r_addr2), 
        .w_en(rfile_w_en), 
        .w_addr(rfile_w_addr), 
        .w_data(rfile_w_data), 
        .r_rs1(rfile_r_rs1), 
        .r_rs2(rfile_r_rs2)
    );

    //----------------------------------------------------------------------------------------//
    // Immediate Generator: creates all types of immediates needed for different instructions
    //----------------------------------------------------------------------------------------//

    wire [31:0] upper_immed, i_type_immed, s_type_immed, 
                branch_immed, jump_immed;

    otter_imm_gen imd (
        .instrn(imem_r_data),
        .upper_immed(upper_immed), 
        .i_type_immed(i_type_immed), 
        .s_type_immed(s_type_immed), 
        .branch_immed(branch_immed), 
        .jump_immed(jump_immed)
    );

    //-----------------------------------------------------------------------//
    // ALU w/ Input MUXES: location of all logical and arithmetic operations
    //-----------------------------------------------------------------------//

    wire [31:0] alu_result;
    reg  [31:0] alu_src_a, alu_src_b;
    wire [3:0] alu_func;

    otter_alu alu (
        .src_a(alu_src_a), 
        .src_b(alu_src_b),
        .func(alu_func),
        .result(alu_result)
    );

    //----------------------------------------------------------------------------------------//
    // Branch Address Generator: generates addresses for use in branch and jump instrucitions
    //----------------------------------------------------------------------------------------//

    wire [31:0] addr_gen_jalr, addr_gen_branch, addr_gen_jal;

    otter_br_addr_gen bag (
        .rs1(rfile_r_rs1), 
        .i_type_immed(i_type_immed), 
        .branch_immed(branch_immed), 
        .jump_immed(jump_immed), 
        .prog_count(pc_addr),
        .jal_addr(addr_gen_jal), 
        .branch_addr(addr_gen_branch), 
        .jalr_addr(addr_gen_jalr)
    );

    //-----------------------------------------------------------------------------------------------//
    // Branch Condition Generator: verifies conditions of rs1 and rs2 for use in branch instructions
    //-----------------------------------------------------------------------------------------------//

    wire cond_gen_eq, cond_gen_lt, cond_gen_ltu;

    otter_br_cond_gen bcg (
        .rs1(rfile_r_rs1), 
        .rs2(rfile_r_rs2),
        .br_eq(cond_gen_eq),
        .br_lt(cond_gen_lt), 
        .br_ltu(cond_gen_ltu)
    );

    //------------------------------------------------------------------------------------------------------//
    // Control Unit Decoder: controls all mux selectors based on the instruction opcode and function number
    //------------------------------------------------------------------------------------------------------//

    wire mret_instrn, ecall_instrn, ebreak_instrn, illegal_instrn;

    otter_cu_dcdr dcdr (
        .instrn(imem_r_data), 

        .trap_taken(intrpt_taken), 

        .br_eq(cond_gen_eq), 
        .br_lt(cond_gen_lt), 
        .br_ltu(cond_gen_ltu),

        .pc_addr_inc(pc_addr_inc),
        .csr_r_data(csr_r_data),
        .mem_data_out(dmem_r_data),
        .alu_result(alu_result),

        .rfile_r_rs1(rfile_r_rs1),
        .upper_immed(upper_immed),

        .rfile_r_rs2(rfile_r_rs2),
        .i_type_immed(i_type_immed),
        .s_type_immed(s_type_immed),
        .pc_addr(pc_addr),

        .jalr(addr_gen_jalr),
        .branch(addr_gen_branch),
        .jal(addr_gen_jal),
        .mtvec(csr_mtvec),
        .mepc(csr_mepc),

        .alu_func(alu_func),
        .alu_src_a(alu_src_a),
        .alu_src_b(alu_src_b),
        .rfile_w_data(rfile_w_data),
        .pc_next_addr(pc_next_addr),
        .dmem_w_strb(dmem_w_strb),

        .mret_instrn(mret_instrn),
        .ecall_instrn(ecall_instrn),
        .ebreak_instrn(ebreak_instrn),
        .illegal_instrn(illegal_instrn)
    );

    //---------------------------------------------------------------------------------------//
    // Control Unit FSM: controls enable signals throughout the OTTER based on current state
    //---------------------------------------------------------------------------------------//

    wire csr_we, intrpt_taken, intrpt_vld;
    assign intrpt_vld = intrpt & csr_mie; 

    otter_cu_fsm fsm (
        .clk(clk),
        .rst(rst),
        .intrpt_vld(intrpt_vld),
        .instrn(imem_r_data),
        .pc_w_en(pc_w_en), 
        .rfile_w_en(rfile_w_en), 
        .dmem_w_en(dmem_w_en), 
        .imem_r_en(imem_r_en), 
        .dmem_r_en(dmem_r_en), 
        .csr_we(csr_we), 
        .intrpt_taken(intrpt_taken)
    );

    //----------------------------------------------------------------------//
    // Control State Registers: handles interrupt state data and triggering
    //----------------------------------------------------------------------//

    // CSR wires
    wire        csr_mie;
    wire [11:0] csr_addr;
    wire [31:0] csr_mtvec, csr_mepc, csr_w_data, csr_r_data;

    assign csr_w_data = rfile_r_rs1;
    assign csr_addr = `INSTRN_CSR(imem_r_data);
    otter_csr csr (
        .clk(clk),
        .rst(rst), 
        .intrpt_taken(intrpt_taken), 
        .w_en(csr_we),
        .addr(csr_addr),
        .prog_count(pc_addr), 
        .w_data(csr_w_data),
        .csr_mie(csr_mie),
        .csr_mepc(csr_mepc), 
        .csr_mtvec(csr_mtvec), 
        .r_data(csr_r_data)
    );

    //----------------------------------------------------------------------------//
    // RISC-V Formal Interface: Set of Bindings Used in Sim to Verify Correctness
    //----------------------------------------------------------------------------//

    `ifdef RISCV_FORMAL

        always @(posedge clk) begin
            // retire current instruction when moving to the next one
            rvfi_valid <= !rst && pc_w_en;
            // have a monotonically increasing counter that tracks the instruction order
            rvfi_order <= rst ? 0 : rvfi_order + rvfi_valid;
            // current instruction fetched from memory
            rvfi_insn <= mem_inst_out;
            // flag if next instruction is illegal
            rvfi_trap <= /* TBD */;
            rvfi_intr <= /* TBD */;

            // Fixed values
            rvfi_halt <= 0; // Never Halts
            rvfi_mode <= 3; // Machine Mode
            rvfi_ixl <= 1;  // Always 32-bit

            // rfile sources and corresponding data
            rvfi_rs1_addr  <= rfile_r_addr1;
            rvfi_rs2_addr  <= rfile_r_addr2;
            rvfi_rs1_rdata <= rfile_r_rs1;
            rvfi_rs2_rdata <= rfile_r_rs2;

            // rfile dest traces
            rvfi_rd_addr  <= rfile_w_en ? rfile_w_addr : '0;
            rvfi_rd_wdata <= rfile_w_en ? rfile_w_data : '0;

            // pc addresses
            rvfi_pc_rdata <= pc_addr;
            rvfi_pc_wdata <= pc_next_addr;

            // memory bindings
            `define RISCV_FORMAL_ALIGNED_MEM
            rvfi_mem_addr  <= mem_addr_2;
            case ({mem_rden2, mem_size})
                {1'b1, MEM_SIZE_BYTE}   : rvfi_mem_rmask <= 32'h0000_00FF;
                {1'b1, MEM_SIZE_H_WORD} : rvfi_mem_rmask <= 32'h0000_FFFF;
                {1'b1, MEM_SIZE_WORD}   : rvfi_mem_rmask <= 32'hFFFF_FFFF;
                default                 : rvfi_mem_rmask <= 32'h0000_0000;
            endcase
            rvfi_mem_rdata <= mem_rden2 ? mem_data_out : '0;
            case ({mem_we2, mem_size})
                {1'b1, MEM_SIZE_BYTE}   : rvfi_mem_wmask <= 32'h0000_00FF;
                {1'b1, MEM_SIZE_H_WORD} : rvfi_mem_wmask <= 32'h0000_FFFF;
                {1'b1, MEM_SIZE_WORD}   : rvfi_mem_wmask <= 32'hFFFF_FFFF;
                default                 : rvfi_mem_wmask <= 32'h0000_0000;
            endcase
            rvfi_mem_wdata <= mem_we2 ? mem_din_2 : '0;
        end

        always @(*) begin
            rvfi_csr_mie_rmask = '0;
            rvfi_csr_mie_wmask = '0;
            rvfi_csr_mie_rdata = '0;
            rvfi_csr_mie_wdata = '0;

            rvfi_csr_mtvec_rmask = '0;
            rvfi_csr_mtvec_wmask = '0;
            rvfi_csr_mtvec_rdata = '0;
            rvfi_csr_mtvec_wdata = '0;

            rvfi_csr_mepc_rmask = '0;
            rvfi_csr_mepc_wmask = '0;
            rvfi_csr_mepc_rdata = '0;
            rvfi_csr_mepc_wdata = '0;

            if (rvfi_valid && `INSTRN_OPCODE(rvfi_insn) == OPCODE_SYS && `INSTRN_FUNC(rvfi_insn) == FUNC_SYS_CSRRW) begin
                case (`INSTRN_CSR(rvfi_insn))
                    CSR_MIE_ADDR : begin
                        rvfi_csr_mie_rmask = 32'h0000_0001;
                        rvfi_csr_mie_wmask = 32'h0000_0001;
                        rvfi_csr_mie_rdata = csr_r_data;
                        rvfi_csr_mie_wdata = csr_w_data;
                    end
                    CSR_MTVEC_ADDR : begin
                        rvfi_csr_mtvec_rmask = 32'hFFFF_FFFF;
                        rvfi_csr_mtvec_wmask = 32'hFFFF_FFFF;
                        rvfi_csr_mtvec_rdata = csr_r_data;
                        rvfi_csr_mtvec_wdata = csr_w_data;
                    end
                    CSR_MEPC_ADDR : begin
                        rvfi_csr_mepc_rmask = 32'hFFFF_FFFF;
                        rvfi_csr_mepc_wmask = 32'hFFFF_FFFC;
                        rvfi_csr_mepc_rdata = csr_r_data;
                        rvfi_csr_mepc_wdata = csr_w_data;   
                    end
                endcase
            end
        end
    `endif
    

endmodule
