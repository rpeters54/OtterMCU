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
    parameter RESET_VEC = 32'h0
) (
    input             i_clk,
    input             i_rst, 
    input      [31:0] i_intrpt, 

`ifdef RISCV_FORMAL

`define CSR_MACRO_OP(NAME) \
    output [31:0] rvfi_csr_``NAME``_rmask, \
    output [31:0] rvfi_csr_``NAME``_wmask, \
    output [31:0] rvfi_csr_``NAME``_rdata, \
    output [31:0] rvfi_csr_``NAME``_wdata,

    `RVFI_OUTPUTS
    `RVFI_BUS_OUTPUTS

`undef CSR_MACRO_OP

`endif

    input      [31:0] i_imem_r_data,
    output     [31:0] o_imem_addr,

    input      [31:0] i_dmem_r_data,
    output reg        o_dmem_re,
    output reg        o_dmem_we,
    output     [3:0]  o_dmem_sel,
    output     [31:0] o_dmem_addr,
    output     [31:0] o_dmem_w_data
);



    //---------------------------------------------------------//
    // Program Counter: keeps track of the current instruction
    //---------------------------------------------------------//


    reg         w_stall;
    reg  [1:0]  w_pc_src_sel;
    wire [31:0] w_br_tgt_addr;
    wire [31:0] w_epc_addr;
    wire [31:0] w_pc_addr;
    wire [31:0] w_pc_next_addr;

    otter_pc #(
        .RESET_VEC(RESET_VEC)
    ) u_otter_pc  (
        .i_clk          (i_clk),
        .i_stall        (w_stall),
        /////////////////////////////////////
        .i_pc_src_sel   (w_pc_src_sel),
        .i_br_tgt_addr  (w_br_tgt_addr),
        .i_epc_addr     (w_epc_addr),
        /////////////////////////////////////
        .o_pc_addr      (w_pc_addr),
        .o_pc_next_addr (w_pc_next_addr)
    );


    // Avoid advancing instruction when stalled
    reg        w_stall_q;
    reg [31:0] w_instrn_q;

    assign      o_imem_addr = w_stall   ? w_pc_addr  : w_pc_next_addr;
    wire [31:0] w_instrn    = w_stall_q ? w_instrn_q : i_imem_r_data;

    always @(posedge i_clk) begin
        // track the previous instruction and stall for writebacks
        w_instrn_q <= w_instrn;
        w_stall_q  <= w_stall;
    end


    //------------------------------------------------------------------------------------------------------//
    // Control Unit Decoder: controls all mux selectors based on the instruction opcode and function number
    //------------------------------------------------------------------------------------------------------//

    wire [3:0]  w_dcdr_alu_func_sel;
    wire        w_dcdr_alu_src_a_sel;
    wire        w_dcdr_alu_src_b_sel;
    wire [1:0]  w_dcdr_rfile_w_sel;
    wire [2:0]  w_dcdr_imm_gen_sel;
    wire [1:0]  w_dcdr_addr_gen_sel;
    wire [14:0] w_dcdr_op_sel;
    wire        w_dcdr_ill_instrn;

    wire        w_dcdr_rfile_we;
    wire        w_dcdr_dmem_re;
    wire        w_dcdr_dmem_we;
    wire        w_dcdr_csr_we;

    otter_dcdr u_otter_dcdr (
        .i_instrn        (w_instrn), 

        .o_alu_func_sel  (w_dcdr_alu_func_sel),
        .o_alu_src_a_sel (w_dcdr_alu_src_a_sel),
        .o_alu_src_b_sel (w_dcdr_alu_src_b_sel),
        .o_rfile_w_sel   (w_dcdr_rfile_w_sel),
        .o_imm_gen_sel   (w_dcdr_imm_gen_sel),
        .o_addr_gen_sel  (w_dcdr_addr_gen_sel),
        .o_op_sel        (w_dcdr_op_sel),
        .o_ill_instrn    (w_dcdr_ill_instrn),

        .o_rfile_we      (w_dcdr_rfile_we),
        .o_dmem_re       (w_dcdr_dmem_re),
        .o_dmem_we       (w_dcdr_dmem_we),
        .o_csr_we        (w_dcdr_csr_we)
    );

    //------------------------------------------------------------------------------------------------------//
    // Control Unit FSM: control the enable signals based on the current
    // processor state and potential exceptions
    //------------------------------------------------------------------------------------------------------//

    reg  [3:0]  w_alu_func_sel;
    reg         w_alu_src_a_sel;
    reg         w_alu_src_b_sel;
    reg  [1:0]  w_rfile_w_sel;
    reg  [2:0]  w_imm_gen_sel;
    reg  [1:0]  w_addr_gen_sel;
    reg  [14:0] w_op_sel;
    reg         w_ill_instrn;

    reg         w_rfile_we;
    reg         w_csr_we;

    reg         w_valid;
    reg         w_excp;
    reg         w_trap;
    wire        w_br_taken;

    reg  [1:0]  w_present_state;
    reg  [1:0]  w_next_state;

    always @(posedge i_clk) begin
        w_present_state <= w_next_state;
    end

    // zero out the decoder select signals when 
    // an instruction is not executing
    always @(*) begin
        w_alu_func_sel  = 0;
        w_alu_src_a_sel = 0;
        w_alu_src_b_sel = 0;
        w_rfile_w_sel   = 0;
        w_imm_gen_sel   = 0;
        w_addr_gen_sel  = 0;
        w_op_sel        = 0;
        w_ill_instrn    = 0;

        if (!i_rst && (w_present_state == ST_EXEC || w_present_state == ST_WR_BK)) begin
            w_alu_func_sel  = w_dcdr_alu_func_sel;
            w_alu_src_a_sel = w_dcdr_alu_src_a_sel;
            w_alu_src_b_sel = w_dcdr_alu_src_b_sel;
            w_rfile_w_sel   = w_dcdr_rfile_w_sel;
            w_imm_gen_sel   = w_dcdr_imm_gen_sel;
            w_addr_gen_sel  = w_dcdr_addr_gen_sel;
            w_op_sel        = w_dcdr_op_sel;
            w_ill_instrn    = w_dcdr_ill_instrn;
        end
    end


    always @(*) begin
        w_rfile_we      = 0; 
        o_dmem_we       = 0; 
        o_dmem_re       = 0; 
        w_csr_we        = 0; 

        w_pc_src_sel    = PC_SRC_SEL_RESET_VEC;
        w_valid         = 0;
        w_stall         = 0;
        w_next_state    = ST_INIT;

        if (!i_rst) begin
            case (w_present_state)
                ST_INIT : begin
                    w_pc_src_sel = PC_SRC_SEL_RESET_VEC;
                    w_valid      = 0;
                    w_stall      = 0;
                    w_next_state = ST_EXEC;
                end
                ST_EXEC : begin
                    w_rfile_we      = w_dcdr_rfile_we;
                    o_dmem_we       = w_dcdr_dmem_we;
                    o_dmem_re       = w_dcdr_dmem_re;
                    w_csr_we        = w_dcdr_csr_we;

                    if (w_excp) begin
                        // if an interrupt occurs, zero out enables
                        // and jump to epc. Instruction does not retire
                        w_pc_src_sel    = PC_SRC_SEL_EPC;
                        w_valid         = 0;
                        w_stall         = 0;
                        w_next_state    = ST_EXEC;

                        w_rfile_we      = 0;
                        o_dmem_we       = 0;
                        o_dmem_re       = 0;
                        w_csr_we        = 0;
                    end else begin
                        // set pc to br/jump if either takes place
                        if ((w_op_sel[DCDR_OP_BRANCH_IDX] && w_br_taken) || w_op_sel[DCDR_OP_JALR_IDX] || w_op_sel[DCDR_OP_JAL_IDX]) begin
                            w_pc_src_sel = PC_SRC_SEL_BR_JUMP;
                        end else begin
                            w_pc_src_sel = PC_SRC_SEL_ADDR_INC;
                        end

                        // move to writeback if load occurs
                        if (w_op_sel[DCDR_OP_LOAD_IDX]) begin
                            w_valid         = 0;
                            w_stall         = 1;
                            w_next_state    = ST_WR_BK;
                        end else begin
                            w_valid         = 1;
                            w_stall         = 0;
                            w_next_state    = ST_EXEC;
                        end

                        // if a synchronous trap (ecall/illegal instruction)
                        // occurs, zero out the enables and jump to epc
                        if (w_trap) begin
                            w_pc_src_sel    = PC_SRC_SEL_EPC;
                            w_valid         = 1;
                            w_stall         = 0;
                            w_next_state    = ST_EXEC;

                            w_rfile_we      = 0;
                            o_dmem_we       = 0;
                            o_dmem_re       = 0;
                            w_csr_we        = 0;
                        end
                    end
                end
                ST_WR_BK : begin
                    w_rfile_we    = 1;
                    o_dmem_re     = 0;

                    w_pc_src_sel  = PC_SRC_SEL_ADDR_INC;
                    w_valid       = 1;
                    w_stall       = 0;
                    w_next_state  = ST_EXEC;
                end
                default : ;
            endcase
        end
    end


    //----------------------------------------------------------------------//
    // REG_FILE w/ Input MUX: contains all registers needed for the program
    //----------------------------------------------------------------------//

    wire [4:0]  w_rfile_r_addr1 = `INSTRN_RS1_ADDR(w_instrn);
    wire [4:0]  w_rfile_r_addr2 = `INSTRN_RS2_ADDR(w_instrn);
    wire [4:0]  w_rfile_w_addr  = `INSTRN_RD_ADDR(w_instrn);
    wire [31:0] w_rfile_r_rs1;
    wire [31:0] w_rfile_r_rs2;
    reg  [31:0] w_rfile_w_data;

    wire [31:0] w_csr_r_data;
    wire [31:0] w_dmem_r_data;
    wire [31:0] w_alu_result;

    always @(*) begin
        case (w_rfile_w_sel)
            RFILE_W_SEL_PC_ADDR_INC : w_rfile_w_data = w_pc_addr + 4;
            RFILE_W_SEL_CSR_R_DATA  : w_rfile_w_data = w_csr_r_data;
            RFILE_W_SEL_DMEM_R_DATA : w_rfile_w_data = w_dmem_r_data;
            RFILE_W_SEL_ALU_RESULT  : w_rfile_w_data = w_alu_result;
        endcase
    end
    otter_rfile u_otter_rfile (
        .i_clk      (i_clk),
        .i_r_addr1  (w_rfile_r_addr1), 
        .i_r_addr2  (w_rfile_r_addr2), 
        .i_w_en     (w_rfile_we), 
        .i_w_addr   (w_rfile_w_addr), 
        .i_w_data   (w_rfile_w_data), 
        .o_r_rs1    (w_rfile_r_rs1), 
        .o_r_rs2    (w_rfile_r_rs2)
    );

    //----------------------------------------------------------------------------------------//
    // Immediate Generator: creates all types of immediates needed for different instructions
    //----------------------------------------------------------------------------------------//

    wire [31:0] w_immed;
    otter_imm_gen u_otter_imm_gen (
        .i_instrn   (w_instrn),
        .i_imm_sel  (w_imm_gen_sel),
        .o_immed    (w_immed)
    );

    //-----------------------------------------------------------------------//
    // ALU w/ Input MUXES: location of all logical and arithmetic operations
    //-----------------------------------------------------------------------//

    // Select what sources go into the ALU
    reg  [31:0] w_alu_src_a;
    reg  [31:0] w_alu_src_b;
    always @(*) begin
        case (w_alu_src_a_sel)
            ALU_SRC_A_SEL_RS1 : w_alu_src_a = w_rfile_r_rs1;
            ALU_SRC_A_SEL_PC  : w_alu_src_a = w_pc_addr;
        endcase
        case (w_alu_src_b_sel) 
            ALU_SRC_B_SEL_RS2 : w_alu_src_b = w_rfile_r_rs2;
            ALU_SRC_B_SEL_IMM : w_alu_src_b = w_immed;
        endcase
    end
    otter_alu u_otter_alu (
        .i_src_a    (w_alu_src_a), 
        .i_src_b    (w_alu_src_b),
        .i_func     (w_alu_func_sel),
        .o_result   (w_alu_result)
    );

    //----------------------------------------------------------------------------------------//
    // Branch Address Generator: generates addresses for use in branch and jump instrucitions
    //----------------------------------------------------------------------------------------//

    wire [31:0] w_addr_gen_jal;
    wire [31:0] w_addr_gen_branch;
    wire [31:0] w_addr_gen_jalr;

    otter_addr_gen u_otter_addr_gen (
        .i_addr_gen_sel (w_addr_gen_sel),
        .i_rfile_r_rs1  (w_rfile_r_rs1), 
        .i_immed        (w_immed), 
        .i_pc_addr      (w_pc_addr),

        .o_jal_addr     (w_addr_gen_jal),
        .o_jalr_addr    (w_addr_gen_jalr),
        .o_branch_addr  (w_addr_gen_branch),
        .o_dest_addr    (w_br_tgt_addr)
    );

    //-----------------------------------------------------------------------------------------------//
    // Branch Condition Generator: verifies conditions of rs1 and rs2 for use in branch instructions
    //-----------------------------------------------------------------------------------------------//

    wire [2:0] w_funct3 = `INSTRN_FUNCT3(w_instrn);

    otter_br_cond_gen u_otter_br_cond_gen (
        .i_funct3       (w_funct3),
        .i_rfile_r_rs1  (w_rfile_r_rs1), 
        .i_rfile_r_rs2  (w_rfile_r_rs2),
        .o_br_taken     (w_br_taken)
    );

    //-----------------------------------------------------------------------------------------------//
    // Addr Alignment Check: Check to verify that addresses are word aligned,
    // tracks exception if not
    //-----------------------------------------------------------------------------------------------//

    wire [3:0]  w_excp_sel;
    wire [31:0] w_trap_mtval;

    otter_addr_check u_otter_addr_check (
        .i_op_sel           (w_op_sel),
        .i_ill_instrn       (w_ill_instrn),
        .i_funct3           (w_funct3),
        .i_br_taken         (w_br_taken),

        .i_instrn           (w_instrn),
        .i_pc_addr          (w_pc_addr),
        .i_addr_gen_jal     (w_addr_gen_jal),
        .i_addr_gen_jalr    (w_addr_gen_jalr),
        .i_addr_gen_branch  (w_addr_gen_branch),
        .i_alu_result       (w_alu_result),

        .o_excp_sel         (w_excp_sel),
        .o_trap_mtval       (w_trap_mtval)
    );

    //----------------------------------------------------------------------//
    // Control State Registers: handles interrupt state data and triggering
    //----------------------------------------------------------------------//

    // CSR wires
    wire [4:0]  w_csr_op_sel = `DCDR_OP_CSR_BITS(w_op_sel);
    wire [11:0] w_csr_addr   = `INSTRN_CSR_ADDR(w_instrn);
    reg  [31:0] w_csr_w_data;

    // Choose data source for csrs based on instruction imm/reg bit
    wire [31:0] w_z_type_immed_wb = {27'd0, w_instrn[19:15]};
    always @(*) begin
        case (w_funct3[2])
            CSR_FUNCT3_HIGH_REG : w_csr_w_data = w_rfile_r_rs1;
            CSR_FUNCT3_HIGH_IMM : w_csr_w_data = w_z_type_immed_wb;
        endcase
    end

    wire w_mask_intrpt = w_present_state != ST_EXEC;

`ifdef RISCV_FORMAL

`define CSR_MACRO_OP(NAME) \
    wire [31:0] w_``NAME``; \
    wire [31:0] w_``NAME``_next;

    `RVFI_CSR_LIST

`undef CSR_MACRO_OP

`endif

    otter_csr u_otter_csr (
        .i_clk          (i_clk),
        .i_rst          (i_rst),
        .i_intrpt       (i_intrpt),
        .i_mask_intrpt  (w_mask_intrpt),
        .i_excp         (w_excp_sel),

        .i_csr_we       (w_csr_we),
        .i_csr_op_sel   (w_csr_op_sel),
        .i_csr_rx_sel   (w_funct3[1:0]),
        .i_csr_addr     (w_csr_addr),
        .i_csr_w_data   (w_csr_w_data),
        .o_csr_r_data   (w_csr_r_data),

        .i_trap_pc      (w_pc_addr), 
        .i_trap_mtval   (w_trap_mtval),

        // output traces of all csrs and their next
        // state if riscv-formal is active
    `ifdef RISCV_FORMAL

    `define CSR_MACRO_OP(NAME) \
        .o_``NAME``(w_``NAME``), \
        .o_``NAME``_next(w_``NAME``_next),

        `RVFI_CSR_LIST

    `undef CSR_MACRO_OP

    `endif

        .o_epc_addr     (w_epc_addr),
        .o_excp         (w_excp),
        .o_trap         (w_trap)
    );

    //----------------------------------------------------------------------//
    // Bus Manager: Manages Signals Presented to the Output Bus
    //----------------------------------------------------------------------//

    otter_bus_mgr u_otter_bus_mgr (
        .i_funct3       (w_funct3),
        .i_op_sel       (w_op_sel),
        .i_alu_result   (w_alu_result),
        .i_rfile_r_rs2  (w_rfile_r_rs2),
        .i_dmem_r_data  (i_dmem_r_data),

        .o_dmem_sel     (o_dmem_sel),
        .o_dmem_addr    (o_dmem_addr),
        .o_dmem_w_data  (o_dmem_w_data),
        .o_dmem_r_data  (w_dmem_r_data)
    );

    //----------------------------------------------------------------------------//
    // RISC-V Formal Interface: Set of Bindings Used in Sim to Verify Correctness
    //----------------------------------------------------------------------------//

`ifdef RISCV_FORMAL

    otter_rvfi u_otter_rvfi (
        .i_clk             (i_clk),
        .i_rst             (i_rst),
        .i_stall           (w_stall),
        .i_valid           (w_valid),
        .i_excp            (w_excp),
        .i_trap            (w_trap),

        .i_instrn          (w_instrn),
        .i_op_sel          (w_op_sel),

        .i_br_taken        (w_br_taken),
        .i_pc_addr         (w_pc_addr),
        .i_br_tgt_addr     (w_br_tgt_addr),
        .i_epc_addr        (w_epc_addr),

        .i_rfile_we        (w_rfile_we),
        .i_rfile_w_data    (w_rfile_w_data),
        .i_rfile_r_rs1     (w_rfile_r_rs1),
        .i_rfile_r_rs2     (w_rfile_r_rs2),

        .i_imem_addr       (o_imem_addr),
        .i_imem_r_data     (i_imem_r_data),

        .i_dmem_sel        (o_dmem_sel),
        .i_dmem_addr       (o_dmem_addr),
        .i_dmem_r_data     (i_dmem_r_data),
        .i_dmem_w_data     (o_dmem_w_data),

        //=============================//
        // CSR Signals
        //=============================//
        // output traces of all csrs and their next
        // state if riscv-formal is active
    `define CSR_MACRO_OP(NAME) \
        .i_``NAME``(w_``NAME``), \
        .i_``NAME``_next(w_``NAME``_next),

        `RVFI_CSR_LIST

    `undef CSR_MACRO_OP

    `define CSR_MACRO_OP(NAME) \
        .rvfi_csr_``NAME``_rmask(rvfi_csr_``NAME``_rmask), \
        .rvfi_csr_``NAME``_wmask(rvfi_csr_``NAME``_wmask), \
        .rvfi_csr_``NAME``_rdata(rvfi_csr_``NAME``_rdata), \
        .rvfi_csr_``NAME``_wdata(rvfi_csr_``NAME``_wdata),

        `RVFI_INTERCONNECTS
        `RVFI_BUS_INTERCONNECTS

    `undef CSR_MACRO_OP
        ._dummy(1'b0)
    );

`endif

endmodule
