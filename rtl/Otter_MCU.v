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

`define CSR_MACRO_OP(NAME) \
    output reg [31:0] rvfi_csr_``NAME``_rmask, \
    output reg [31:0] rvfi_csr_``NAME``_wmask, \
    output reg [31:0] rvfi_csr_``NAME``_rdata, \
    output reg [31:0] rvfi_csr_``NAME``_wdata,

module otter_mcu #(
    parameter RESET_VEC = 0
) (
    input             clk,
    input             rst, 
    input      [31:0] intrpt, 

`ifdef RISCV_FORMAL
    `RVFI_OUTPUTS
`endif

    input      [31:0] imem_r_data,
    output reg [31:0] imem_addr,

    input      [31:0] dmem_r_data,
    output            dmem_r_en,
    output            dmem_w_en,
    output     [3:0]  dmem_w_strb,
    output     [31:0] dmem_addr,
    output     [31:0] dmem_w_data
);

`undef CSR_MACRO_OP

    wire [31:0] alu_result;
    wire [31:0] rfile_r_rs1, rfile_r_rs2;

    assign dmem_addr   = alu_result;

    //------------------------------------------------------------------------------------------------------//
    // Control Unit Decoder: controls all mux selectors based on the instruction opcode and function number
    //------------------------------------------------------------------------------------------------------//

    wire cond_gen_eq;
    wire cond_gen_lt; 
    wire cond_gen_ltu;

    wire csr_intrpt_vld;
    wire csr_read_only;
    wire csr_addr_vld;

    wire [3:0] alu_func;
    wire      alu_src_sel_a;
    wire [1:0] alu_src_sel_b;
    wire [1:0] rfile_w_sel;
    wire [2:0] pc_src_sel;
    wire [2:0] csr_op_sel;
    wire [2:0] csr_trap_cause_sel;
    wire [3:0] dmem_w_base_strb;

    wire [31:0] i_type_immed, s_type_immed;
    wire [1:0] addr_load_alignment  = rfile_r_rs1[1:0] + i_type_immed[1:0];
    wire [1:0] addr_store_alignment = rfile_r_rs1[1:0] + s_type_immed[1:0];

    wire pc_w_en;
    wire rfile_w_en;
    wire csr_w_en;
    wire dcdr_stall;

    // Addr Gen values
    wire [31:0] addr_gen_jalr, addr_gen_branch, addr_gen_jal;

`ifdef RISCV_FORMAL
        wire       dcdr_intrpt_taken, dcdr_trap_taken;
        wire [1:0] dcdr_state;
`endif

    otter_cu_dcdr dcdr (
        .clk(clk),
        .rst(rst),
        .instrn(imem_r_data), 

        .br_eq(cond_gen_eq), 
        .br_lt(cond_gen_lt), 
        .br_ltu(cond_gen_ltu),

        .csr_intrpt_vld(csr_intrpt_vld),
        .csr_addr_vld(csr_addr_vld),
        .csr_read_only(csr_read_only),

        .addr_load_alignment(addr_load_alignment),
        .addr_store_alignment(addr_store_alignment),
        .addr_jalr_alignment(addr_gen_jalr[1:0]),
        .addr_branch_alignment(addr_gen_branch[1:0]),
        .addr_jal_alignment(addr_gen_jal[1:0]),

`ifdef RISCV_FORMAL
        .intrpt_taken(dcdr_intrpt_taken),
        .trap_taken(dcdr_trap_taken),
        .state(dcdr_state),
`endif
        .alu_func(alu_func),
        .alu_src_sel_a(alu_src_sel_a),
        .alu_src_sel_b(alu_src_sel_b),
        .rfile_w_sel(rfile_w_sel),
        .pc_src_sel(pc_src_sel),
        .csr_op_sel(csr_op_sel),
        .csr_trap_cause_sel(csr_trap_cause_sel),
        .dmem_w_base_strb(dmem_w_base_strb),

        .pc_w_en(pc_w_en),
        .rfile_w_en(rfile_w_en),
        .dmem_w_en(dmem_w_en),
        .dmem_r_en(dmem_r_en),
        .csr_w_en(csr_w_en),
        .stall(dcdr_stall)
    );

    //-----------------------------------------------------------------------------------//
    // Input Selector Muxes: Muxes managed by the decoder, select inputs to other blocks
    //-----------------------------------------------------------------------------------//

    wire [2:0]  funct3 = `INSTRN_FUNCT3(imem_r_data);
    wire [6:0]  opcode = `INSTRN_OPCODE(imem_r_data);

    // PC values
    wire [31:0] pc_addr, pc_addr_inc;


    // Immediate Gen values
    wire [31:0] upper_immed, branch_immed, jump_immed, z_immed;

    // CSR values
    wire [31:0] csr_r_data, csr_mtvec, csr_mepc;

    // selector outputs
    reg  [31:0] alu_src_a, alu_src_b, rfile_w_data, pc_next_addr, csr_w_data,
        dmem_r_masked_data, csr_mtval_trap_addr;

    // Avoid advancing instruction when stalled
    always @(*) begin
        case (dcdr_stall)
            '1 : imem_addr = pc_addr;
            '0 : imem_addr = pc_next_addr;
        endcase
    end
    // Mask and align the memory so values are read properly
    always @(*) begin
        dmem_r_masked_data = dmem_r_data >> (addr_load_alignment * 8);
        case (funct3)
            FUNCT3_I_LB  : dmem_r_masked_data = {{24{dmem_r_masked_data[7]}}, dmem_r_masked_data[7:0]};
            FUNCT3_I_LH  : dmem_r_masked_data = {{16{dmem_r_masked_data[15]}}, dmem_r_masked_data[15:0]};
            FUNCT3_I_LW  : dmem_r_masked_data = dmem_r_masked_data;
            FUNCT3_I_LBU : dmem_r_masked_data = {24'd0, dmem_r_masked_data[7:0]};
            FUNCT3_I_LHU : dmem_r_masked_data = {16'd0, dmem_r_masked_data[15:0]};
            default      : ;
        endcase
    end
    // align the data and strb so that values are stored properly
    assign dmem_w_data = rfile_r_rs2 << (addr_store_alignment * 8);
    assign dmem_w_strb = dmem_w_base_strb << addr_store_alignment;
    // Select what sources go into the ALU
    always @(*) begin
        case (alu_src_sel_a)
            ALU_SRC_SEL_A_RS1         : alu_src_a = rfile_r_rs1;
            ALU_SRC_SEL_A_UPPER_IMM   : alu_src_a = upper_immed;
        endcase
        case (alu_src_sel_b) 
            ALU_SRC_SEL_B_RS2         : alu_src_b = rfile_r_rs2;
            ALU_SRC_SEL_B_I_TYPE_IMM  : alu_src_b = i_type_immed;
            ALU_SRC_SEL_B_S_TYPE_IMM  : alu_src_b = s_type_immed;
            ALU_SRC_SEL_B_PC_ADDR     : alu_src_b = pc_addr;
        endcase
    end
    // Select what register is written to by the rfile
    always @(*) begin
        case (rfile_w_sel) 
            RFILE_W_SEL_PC_ADDR_INC : rfile_w_data = pc_addr_inc;
            RFILE_W_SEL_CSR_R_DATA  : rfile_w_data = csr_r_data;
            RFILE_W_SEL_DMEM_R_DATA : rfile_w_data = dmem_r_masked_data;
            RFILE_W_SEL_ALU_RESULT  : rfile_w_data = alu_result;
        endcase
    end
    // Select what the next address is sourced from
    always @(*) begin
        case(pc_src_sel)
            PC_SRC_SEL_ADDR_INC  : pc_next_addr = pc_addr_inc;
            PC_SRC_SEL_JALR      : pc_next_addr = addr_gen_jalr;
            PC_SRC_SEL_BRANCH    : pc_next_addr = addr_gen_branch;
            PC_SRC_SEL_JAL       : pc_next_addr = addr_gen_jal;
            PC_SRC_SEL_MTVEC     : pc_next_addr = csr_mtvec;
            PC_SRC_SEL_MEPC      : pc_next_addr = csr_mepc;
            PC_SRC_SEL_RESET_VEC : pc_next_addr = RESET_VEC;
            default              : pc_next_addr = 32'hDEADDEAD;
        endcase
    end
    // Choose data source for csrs based on instruction imm/reg bit
    always @(*) begin
        case (funct3[2])
            CSR_FUNCT3_HIGH_REG : csr_w_data = rfile_r_rs1;
            CSR_FUNCT3_HIGH_IMM : csr_w_data = z_immed;
        endcase
    end
    // On trap for attempted misaligned imem/dmem access, set mtval accordingly 
    always @(*) begin
        csr_mtval_trap_addr = '0;
        case (csr_op_sel)
            CSR_OP_EBREAK : csr_mtval_trap_addr = pc_addr;
            CSR_OP_TRAP : begin
                case (csr_trap_cause_sel)
                    TRAP_CAUSE_SEL_INSTRN_ADDR_MISALIGN : begin
                        case (opcode)
                            OPCODE_JAL    : csr_mtval_trap_addr = addr_gen_jal;
                            OPCODE_JALR   : csr_mtval_trap_addr = addr_gen_jalr;
                            OPCODE_BRANCH : csr_mtval_trap_addr = addr_gen_branch;
                            default       : ;
                        endcase
                    end
                    TRAP_CAUSE_SEL_LOAD_ADDR_MISALIGN  : csr_mtval_trap_addr = alu_result;
                    TRAP_CAUSE_SEL_STORE_ADDR_MISALIGN : csr_mtval_trap_addr = alu_result;
                    default                            : ;
                endcase
            end
            default : ;
        endcase
    end

    //---------------------------------------------------------//
    // Program Counter: keeps track of the current instruction
    //---------------------------------------------------------//

    otter_pc pc (
        .clk(clk),
        .w_en(pc_w_en),
        .next_addr(pc_next_addr),
        .addr(pc_addr),
        .addr_inc(pc_addr_inc)
    );

    //----------------------------------------------------------------------//
    // REG_FILE w/ Input MUX: contains all registers needed for the program
    //----------------------------------------------------------------------//

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

    otter_imm_gen imd (
        .instrn(imem_r_data),
        .upper_immed(upper_immed), 
        .i_type_immed(i_type_immed), 
        .s_type_immed(s_type_immed), 
        .branch_immed(branch_immed), 
        .jump_immed(jump_immed),
        .z_immed(z_immed)
    );

    //-----------------------------------------------------------------------//
    // ALU w/ Input MUXES: location of all logical and arithmetic operations
    //-----------------------------------------------------------------------//

    otter_alu alu (
        .src_a(alu_src_a), 
        .src_b(alu_src_b),
        .func(alu_func),
        .result(alu_result)
    );

    //----------------------------------------------------------------------------------------//
    // Branch Address Generator: generates addresses for use in branch and jump instrucitions
    //----------------------------------------------------------------------------------------//

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

    otter_br_cond_gen bcg (
        .rs1(rfile_r_rs1), 
        .rs2(rfile_r_rs2),
        .br_eq(cond_gen_eq),
        .br_lt(cond_gen_lt), 
        .br_ltu(cond_gen_ltu)
    );

    //----------------------------------------------------------------------//
    // Control State Registers: handles interrupt state data and triggering
    //----------------------------------------------------------------------//

    // CSR wires
    wire [11:0] csr_addr = `INSTRN_CSR(imem_r_data);
    wire [31:0] csr_mstatus, csr_misa, csr_mie, csr_mstatush,
                csr_mscratch, csr_mcause, csr_mtval, csr_mip;

    wire [31:0] csr_mvendorid, csr_marchid, csr_mimpid, csr_mhartid,
                csr_mconfigptr;

    otter_csr csr (
        .clk(clk),
        .intrpt(intrpt), 

        .op_sel(csr_op_sel),
        .trap_cause_sel(csr_trap_cause_sel),
        .funct3_low(funct3[1:0]),
        .w_en(csr_w_en),
        .addr(csr_addr),
        .pc_addr(pc_addr), 
        .w_data(csr_w_data),
        .mtval_trap_addr(csr_mtval_trap_addr),

        .intrpt_vld(csr_intrpt_vld),
        .read_only(csr_read_only),
        .addr_vld(csr_addr_vld),
        .r_data(csr_r_data),

        .mstatus(csr_mstatus),
        .misa(csr_misa),
        .mie(csr_mie),
        .mtvec(csr_mtvec),
        .mstatush(csr_mstatush),
        .mscratch(csr_mscratch),
        .mepc(csr_mepc),
        .mcause(csr_mcause),
        .mtval(csr_mtval),
        .mip(csr_mip),

        .mvendorid(csr_mvendorid),
        .marchid(csr_marchid),
        .mimpid(csr_mimpid),
        .mhartid(csr_mhartid),
        .mconfigptr(csr_mconfigptr)
    );

    //----------------------------------------------------------------------------//
    // RISC-V Formal Interface: Set of Bindings Used in Sim to Verify Correctness
    //----------------------------------------------------------------------------//

    `ifdef RISCV_FORMAL
	
        wire next_rvfi_valid = (dcdr_state == ST_EXEC && !dmem_r_en) || dcdr_state == ST_WR_BK;
        reg  next_rvfi_intr; 

        always @(posedge clk) begin

            rvfi_valid <= next_rvfi_valid;

            // set for both execute and writeback states
            if (dcdr_state == ST_EXEC || dcdr_state == ST_WR_BK) begin
                // rfile dest traces
                rvfi_rd_addr   <= rfile_w_en ? rfile_w_addr : '0;
                rvfi_rd_wdata  <= rfile_w_en ? rfile_w_data : '0;
                rvfi_mem_rdata <= dmem_r_data;
            end

            // set for solely the execute state
            if (dcdr_state == ST_EXEC) begin
                next_rvfi_intr <= dcdr_trap_taken || dcdr_intrpt_taken;
                // have a monotonically increasing counter that tracks the instruction order
                rvfi_order <= rvfi_order + '1;
                // current instruction fetched from memory
                rvfi_insn <= imem_r_data;
                // flag if next instruction is illegal
                rvfi_trap <= dcdr_trap_taken;
                // NOTE: rvfi_intr needs a single cycle delay befor triggering
                // This allows it to be asserted as the first instruction of
                // the interrupt is retired which is what the docs say to do.
                rvfi_intr <= next_rvfi_intr;

                // Fixed values
                rvfi_halt <= 0; // Never Halts
                rvfi_mode <= 3; // Machine Mode
                rvfi_ixl  <= 1; // Always 32-bit

                // rfile sources and corresponding data
                rvfi_rs1_addr  <= rfile_r_addr1;
                rvfi_rs2_addr  <= rfile_r_addr2;
                rvfi_rs1_rdata <= rfile_r_rs1;
                rvfi_rs2_rdata <= rfile_r_rs2;

                // pc addresses
                rvfi_pc_rdata <= pc_addr;
                rvfi_pc_wdata <= pc_next_addr;

                // add checks for misaligned memory access
                `define RISCV_FORMAL_ALIGNED_MEM
                if (dmem_r_en || dmem_w_en) begin
                    rvfi_mem_addr <= dmem_addr;
                    case ({dmem_r_en, funct3})
                        {1'b1, FUNCT3_I_LB}  : begin rvfi_mem_rmask <= 4'b 0001 << addr_load_alignment; end
                        {1'b1, FUNCT3_I_LH}  : begin rvfi_mem_rmask <= 4'b 0011 << addr_load_alignment; end
                        {1'b1, FUNCT3_I_LW}  : begin rvfi_mem_rmask <= 4'b 1111 << addr_load_alignment; end
                        {1'b1, FUNCT3_I_LBU} : begin rvfi_mem_rmask <= 4'b 0001 << addr_load_alignment; end
                        {1'b1, FUNCT3_I_LHU} : begin rvfi_mem_rmask <= 4'b 0011 << addr_load_alignment; end
                        default: rvfi_mem_rmask <= 0;
                    endcase
                    rvfi_mem_wmask <= dmem_w_strb;
                    rvfi_mem_wdata <= dmem_w_data;
                end else begin
                    rvfi_mem_addr  <= 0;
                    rvfi_mem_rmask <= 0;
                    rvfi_mem_wmask <= 0;
                    rvfi_mem_wdata <= 0;
                end
            end

            if (rst || dcdr_state == ST_INIT) begin
                next_rvfi_intr <= 0;
                rvfi_valid <= 0;
                rvfi_order <= 0;
                rvfi_trap <= 0;
            end
        end

        always @(posedge clk) begin
            if (next_rvfi_valid) begin
            `define CSR_MACRO_OP(NAME) \
                rvfi_csr_``NAME``_rmask <= 32'hffff_ffff; \
                rvfi_csr_``NAME``_wmask <= 32'hffff_ffff; \
                rvfi_csr_``NAME``_rdata <= csr_``NAME``; \
                rvfi_csr_``NAME``_wdata <= csr_``NAME``;

                `RVFI_CSR_LIST
            `undef CSR_MACRO_OP
            end
        end
    `endif

endmodule
