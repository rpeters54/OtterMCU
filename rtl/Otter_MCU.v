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


module Otter_MCU #(
    parameter ROM_FILE = "default.mem"
) (
    input         clk,
    input         rst, 
    input         intrpt, 
    input  [31:0] iobus_in,
    output [31:0] iobus_out, 
    output [31:0] iobus_addr,
    output        iobus_wr
);
    
    //------------------//
    // Wire Definitions
    //------------------//

    // PC wires
    wire [31:0] pc_addr, pc_next_addr;

    // Memory wires
    wire [31:0] mem_inst_out, mem_data_out;

    // Reg_File wires
    wire [31:0] rfile_w_data, rfile_r_rs1, rfile_r_rs2;

    // Immed_Gen wires
    wire [31:0] upper_immed, i_type_immed, s_type_immed, 
                branch_immed, jump_immed;

    // ALU wires
    wire [31:0] alu_src_a, alu_src_b, alu_result;

    // Branch_Addr_Gen wires
    wire [31:0] addr_gen_jalr, addr_gen_branch, addr_gen_jal;
    
    // Branch_Cond_Gen wires
    wire cond_gen_eq, cond_gen_lt, cond_gen_ltu;

    // CU_DCDR wires
    wire alu_src_sel_a;
    wire [1:0] alu_src_sel_b, rfile_w_sel;
    wire [2:0] PC_src_sel;
    wire [3:0] alu_func;
    
    // CU_FSM wires
    wire pc_w_en, rfile_w_en, mem_we2, mem_rden1, 
         mem_rden2, cu_rst, csr_WE, intrpt_taken, intrpt_vld;
        
    // CSR wires
    wire        csr_mie;
    wire [31:0] csr_mtvec, csr_mepc, csr_rd;
    
    //--------------------//
    // Output Assignments
    //--------------------//

    assign iobus_addr = alu_result;
    assign iobus_out  = rfile_r_rs2;
    
    //---------------------------------------------------------//
    // Program Counter: keeps track of the current instruction
    //---------------------------------------------------------//

    PC pc (
        .clk(clk),
        .rst(cu_rst),
        .w_en(pc_w_en),
        .src_sel(PC_src_sel),
        .jalr(addr_gen_jalr),
        .branch(addr_gen_branch),
        .jal(addr_gen_jal),
        .mtvec(csr_mtvec),
        .mepc(csr_mepc),
        .addr(pc_addr),
        .next_addr(pc_next_addr)
    );

    //-------------------------------------------------------------------//
    // Memory: stores all relevant data and instructions for the program
    //-------------------------------------------------------------------//

    Memory #(ROM_FILE) mem (
        .MEM_CLK(clk), 
        .MEM_RDEN1(mem_rden1), 
        .MEM_RDEN2(mem_rden2),
        .MEM_WE2(mem_we2), 
        .MEM_ADDR1(pc_addr[15:2]), 
        .MEM_ADDR2(alu_result),
        .MEM_DIN2(rfile_r_rs2), 
        .MEM_SIZE(mem_inst_out[13:12]), 
        .MEM_SIGN(mem_inst_out[14]), 
        .IO_IN(iobus_in), 
        .IO_WR(iobus_wr),
        .MEM_DOUT1(mem_inst_out), 
        .MEM_DOUT2(mem_data_out)
    );

    //----------------------------------------------------------------------//
    // REG_FILE w/ Input MUX: contains all registers needed for the program
    //----------------------------------------------------------------------//

    Mux4_1 #(32) reg_mux (
        .zero(pc_next_addr), 
        .one(csr_rd), 
        .two(mem_data_out), 
        .three(alu_result), 
        .sel(rfile_w_sel), 
        .mux_out(rfile_w_data)
    );
    Reg_File rf (
        .clk(clk),
        .r_addr1(mem_inst_out[19:15]), 
        .r_addr2(mem_inst_out[24:20]), 
        .w_en(rfile_w_en), 
        .w_addr(mem_inst_out[11:7]), 
        .w_data(rfile_w_data), 
        .r_rs1(rfile_r_rs1), 
        .r_rs2(rfile_r_rs2)
    );

    //----------------------------------------------------------------------------------------//
    // Immediate Generator: creates all types of immediates needed for different instructions
    //----------------------------------------------------------------------------------------//

    Immed_Gen imd (
        .instrn(mem_inst_out[31:7]),
        .upper_immed(upper_immed), 
        .i_type_immed(i_type_immed), 
        .s_type_immed(s_type_immed), 
        .branch_immed(branch_immed), 
        .jump_immed(jump_immed)
    );
    
    //-----------------------------------------------------------------------//
    // ALU w/ Input MUXES: location of all logical and arithmetic operations
    //-----------------------------------------------------------------------//

    Mux2_1 #(32) src_a_mux (
        .zero(rfile_r_rs1), 
        .one(upper_immed), 
        .sel(alu_src_sel_a), 
        .mux_out(alu_src_a)
    );
    Mux4_1 #(32) src_b_mux (
        .zero(rfile_r_rs2), 
        .one(i_type_immed), 
        .two(s_type_immed), 
        .three(pc_addr), 
        .sel(alu_src_sel_b), 
        .mux_out(alu_src_b)
    );
    ALU alu (
        .src_a(alu_src_a), 
        .src_b(alu_src_b),
        .func(alu_func),
        .result(alu_result)
    );

    //----------------------------------------------------------------------------------------//
    // Branch Address Generator: generates addresses for use in branch and jump instrucitions
    //----------------------------------------------------------------------------------------//

    Branch_Addr_Gen bag (
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

    Branch_Cond_Gen bcg (
        .rs1(rfile_r_rs1), 
        .rs2(rfile_r_rs2),
        .br_eq(cond_gen_eq),
        .br_lt(cond_gen_lt), 
        .br_ltu(cond_gen_ltu)
    );

    //------------------------------------------------------------------------------------------------------//
    // Control Unit Decoder: controls all mux selectors based on the instruction opcode and function number
    //------------------------------------------------------------------------------------------------------//

    CU_DCDR dcdr (
        .opcode(mem_inst_out[6:0]),
        .func(mem_inst_out[14:12]),
        .instrn_bit_30(mem_inst_out[30]), 
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

    assign intrpt_vld = intrpt & csr_mie; 
    CU_FSM fsm (
        .clk(clk),
        .rst(rst),
        .intrpt_vld(intrpt_vld),
        .opcode(mem_inst_out[6:0]),
        .func(mem_inst_out[14:12]),
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
    CSR csr (
        .clk(clk),
        .rst(cu_rst), 
        .intrpt_taken(intrpt_taken), 
        .w_en(csr_WE),
        .addr(mem_inst_out[31:20]),
        .prog_count(pc_addr), 
        .w_data(rfile_r_rs1),
        .csr_mie(csr_mie),
        .csr_mepc(csr_mepc), 
        .csr_mtvec(csr_mtvec), 
        .r_data(csr_rd)
    );    
    

endmodule
