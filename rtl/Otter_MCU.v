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

module otter_mcu #(
    parameter ROM_FILE = "default.mem"
) (
    input         clk,
    input         rst, 
    input         intrpt, 
    input  [31:0] iobus_in,

`ifdef RISCV_FORMAL
    `RVFI_OUTPUTS
`endif

    output [31:0] iobus_out, 
    output [31:0] iobus_addr,
    output        iobus_wr
);

    assign iobus_addr = alu_result;
    assign iobus_out  = rfile_r_rs2;

    //---------------------------------------------------------//
    // Program Counter: keeps track of the current instruction
    //---------------------------------------------------------//

    wire [31:0] pc_next_addr, pc_addr, pc_addr_inc;

    otter_pc pc (
        .clk(clk),
        .rst(cu_rst),
        .w_en(pc_w_en),
        .src_sel(PC_src_sel),
        .jalr(addr_gen_jalr),
        .branch(addr_gen_branch),
        .jal(addr_gen_jal),
        .mtvec(csr_mtvec),
        .mepc(csr_mepc),
`ifdef RISCV_FORMAL
        .next_addr(pc_next_addr),
`endif
        .addr(pc_addr),
        .addr_inc(pc_addr_inc)
    );

    //-------------------------------------------------------------------//
    // Memory: stores all relevant data and instructions for the program
    //-------------------------------------------------------------------//

    // Memory wires
    wire [31:0] mem_addr_2, mem_din_2, mem_inst_out, mem_data_out;
    wire [13:0] mem_addr_1;
    wire [1:0]  mem_size;
    wire        mem_sign;

    assign mem_addr_1 = pc_addr[15:2];
    assign mem_addr_2 = alu_result;
    assign mem_din_2 = rfile_r_rs2;
    assign mem_size = mem_inst_out[13:12];
    assign mem_sign = mem_inst_out[14];

    otter_mem #(ROM_FILE) mem (
        .MEM_CLK(clk), 
        .MEM_RDEN1(mem_rden1), 
        .MEM_RDEN2(mem_rden2),
        .MEM_WE2(mem_we2), 
        .MEM_ADDR1(mem_addr_1), 
        .MEM_ADDR2(mem_addr_2),
        .MEM_DIN2(mem_din_2), 
        .MEM_SIZE(mem_size), 
        .MEM_SIGN(mem_sign), 
`ifdef RISCV_FORMAL
        .MISALIGN(mem_misalign),
`endif
        .IO_IN(iobus_in), 
        .IO_WR(iobus_wr),
        .MEM_DOUT1(mem_inst_out), 
        .MEM_DOUT2(mem_data_out)
    );

    //----------------------------------------------------------------------//
    // REG_FILE w/ Input MUX: contains all registers needed for the program
    //----------------------------------------------------------------------//

    // Reg_File wires
    reg [31:0] rfile_w_data;
    wire [31:0] rfile_r_rs1, rfile_r_rs2;
    wire [4:0]  rfile_r_addr1, rfile_r_addr2, rfile_w_addr;

    assign rfile_r_addr1 = `INSTRN_RS1_ADDR(mem_inst_out);
    assign rfile_r_addr2 = `INSTRN_RS2_ADDR(mem_inst_out);
    assign rfile_w_addr  = `INSTRN_RSD_ADDR(mem_inst_out);

    always @(*) begin
        case (rfile_w_sel) 
            2'd0 : rfile_w_data = pc_addr_inc;
            2'd1 : rfile_w_data = csr_r_data;
            2'd2 : rfile_w_data = mem_data_out;
            2'd3 : rfile_w_data = alu_result;
        endcase
    end

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

    // Immed_Gen wires
    wire [31:0] upper_immed, i_type_immed, s_type_immed, 
                branch_immed, jump_immed;

    otter_imm_gen imd (
        .instrn(mem_inst_out),
        .upper_immed(upper_immed), 
        .i_type_immed(i_type_immed), 
        .s_type_immed(s_type_immed), 
        .branch_immed(branch_immed), 
        .jump_immed(jump_immed)
    );

    //-----------------------------------------------------------------------//
    // ALU w/ Input MUXES: location of all logical and arithmetic operations
    //-----------------------------------------------------------------------//

    // ALU connections
    wire [31:0] alu_result;
    reg  [31:0] alu_src_a, alu_src_b;
 
    always @(*) begin
        case (alu_src_sel_a)
            1'd0 : alu_src_a = rfile_r_rs1;
            1'd1 : alu_src_a = upper_immed;
        endcase
        case (alu_src_sel_b) 
            2'd0 : alu_src_b = rfile_r_rs2;
            2'd1 : alu_src_b = i_type_immed;
            2'd2 : alu_src_b = s_type_immed;
            2'd3 : alu_src_b = pc_addr;
        endcase
    end
    otter_alu alu (
        .src_a(alu_src_a), 
        .src_b(alu_src_b),
        .func(alu_func),
        .result(alu_result)
    );

    //----------------------------------------------------------------------------------------//
    // Branch Address Generator: generates addresses for use in branch and jump instrucitions
    //----------------------------------------------------------------------------------------//

    // Branch_Addr_Gen wires
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

    // Branch_Cond_Gen wires
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

    // CU_DCDR wires
    wire alu_src_sel_a;
    wire [1:0] alu_src_sel_b, rfile_w_sel;
    wire [2:0] PC_src_sel;
    wire [3:0] alu_func;

    otter_cu_dcdr dcdr (
        .instrn(mem_inst_out), 
        .intrpt_taken(intrpt_taken), 
        .br_eq(cond_gen_eq), 
        .br_lt(cond_gen_lt), 
        .br_ltu(cond_gen_ltu),
        .alu_func(alu_func),
        .alu_src_sel_a(alu_src_sel_a),
        .alu_src_sel_b(alu_src_sel_b),
        .pc_src_sel(PC_src_sel),
        .rfile_w_sel(rfile_w_sel)
    );

    //---------------------------------------------------------------------------------------//
    // Control Unit FSM: controls enable signals throughout the OTTER based on current state
    //---------------------------------------------------------------------------------------//

    // CU_FSM wires
    wire pc_w_en, rfile_w_en, mem_we2, mem_rden1, mem_misalign,
         mem_rden2, cu_rst, csr_WE, intrpt_taken, intrpt_vld, invld_opcode;

    assign intrpt_vld = intrpt & csr_mie; 

    otter_cu_fsm fsm (
        .clk(clk),
        .rst(rst),
        .mem_misalign(mem_misalign),
        .intrpt_vld(intrpt_vld),
        .instrn(mem_inst_out),
`ifdef RISCV_FORMAL
        .invld_opcode(invld_opcode),
`endif
        .pc_w_en(pc_w_en), 
        .rfile_w_en(rfile_w_en), 
        .mem_we2(mem_we2), 
        .mem_rden1(mem_rden1), 
        .mem_rden2(mem_rden2), 
        .cu_rst(cu_rst), 
        .csr_we(csr_WE), 
        .intrpt_taken(intrpt_taken)
    );

    //----------------------------------------------------------------------//
    // Control State Registers: handles interrupt state data and triggering
    //----------------------------------------------------------------------//

    // CSR wires
    wire        csr_mie;
    wire [31:0] csr_mtvec, csr_mepc, csr_w_data, csr_r_data;

    assign csr_w_data = rfile_r_rs1;
    otter_csr csr (
        .clk(clk),
        .rst(cu_rst), 
        .intrpt_taken(intrpt_taken), 
        .w_en(csr_WE),
        .addr(mem_inst_out[31:20]),
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

            rvfi_trap <= mem_misalign || (pc_next_addr & '3 > 0) || invld_opcode;
            rvfi_intr <= /* TBD */;

            // Fixed values
            rvfi_halt <= 0;
            rvfi_mode <= 3;
            rvfi_ixl <= 1;

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
                        rvfi_csr_mepc_wmask = 32'hFFFF_FFFF;
                        rvfi_csr_mepc_rdata = csr_r_data;
                        rvfi_csr_mepc_wdata = csr_w_data;   
                    end
                endcase
            end
        end
    `endif
    

endmodule
