
`timescale 1ns / 1ps
`include "otter_defines.vh"

module otter_rvfi (
    input             i_clk,
    input             i_rst,
    input             i_valid,
    input             i_excp,
    input             i_trap,

    // instruction signals
    input      [31:0] i_instrn,
    input      [14:0] i_op_sel,

    // pc selection
    input             i_br_taken,
    input      [31:0] i_pc_addr,
    input      [31:0] i_br_tgt_addr,
    input      [31:0] i_epc_addr,

    // rfile data
    input             i_rfile_we,
    input      [31:0] i_rfile_w_data,
    input      [31:0] i_rfile_r_rs1,
    input      [31:0] i_rfile_r_rs2,

    // dmem interface
    input      [3:0]  i_dmem_sel,
    input      [31:0] i_dmem_addr,
    input      [31:0] i_dmem_r_data,
    input      [31:0] i_dmem_w_data,

`define CSR_MACRO_OP(NAME) \
    input [31:0] i_``NAME``, \
    input [31:0] i_``NAME``_next,

    `RVFI_CSR_LIST

`undef CSR_MACRO_OP

`define CSR_MACRO_OP(NAME) \
    output reg [31:0] rvfi_csr_``NAME``_rmask, \
    output reg [31:0] rvfi_csr_``NAME``_wmask, \
    output reg [31:0] rvfi_csr_``NAME``_rdata, \
    output reg [31:0] rvfi_csr_``NAME``_wdata,

    `RVFI_OUTPUTS

`undef CSR_MACRO_OP
    // dummy signal needed because of trailing comma
    input _dummy
);

    wire w_jmp_taken = (
        (i_op_sel[DCDR_OP_BRANCH_IDX] && i_br_taken) ||
         i_op_sel[DCDR_OP_JAL_IDX] ||
         i_op_sel[DCDR_OP_JALR_IDX]
    );

    // csr read and write checks to check for rs1 and rd usage
    reg        w_csr_use_rs1;
    wire [2:0] w_funct3 = `INSTRN_FUNCT3(i_instrn);
    always @(*) begin
        w_csr_use_rs1  = 0;
        if (i_op_sel[DCDR_OP_WRITE_IDX]) begin
            case (w_funct3)
                FUNCT3_SYS_CSRRW, FUNCT3_SYS_CSRRS, FUNCT3_SYS_CSRRC : begin
                    w_csr_use_rs1  = 1;
                end
                FUNCT3_SYS_CSRRWI, FUNCT3_SYS_CSRRSI, FUNCT3_SYS_CSRRCI : begin
                    w_csr_use_rs1  = 0;
                end
                default : ;
            endcase
        end
    end

    wire w_use_rs2     = i_op_sel[DCDR_OP_REG_IDX]
                      || i_op_sel[DCDR_OP_STORE_IDX]
                      || i_op_sel[DCDR_OP_BRANCH_IDX];
    wire w_use_rs1     = w_use_rs2
                      || i_op_sel[DCDR_OP_IMM_IDX]
                      || i_op_sel[DCDR_OP_LOAD_IDX]
                      || i_op_sel[DCDR_OP_JALR_IDX]
                      || w_csr_use_rs1;

    reg [31:0] w_pc_next;
    always @(*) begin
        if (i_trap) begin
            w_pc_next = i_epc_addr;
        end else if (w_jmp_taken) begin
            w_pc_next = i_br_tgt_addr;
        end else begin
            w_pc_next = i_pc_addr + 4;
        end
    end


    // used to drive the rvfi_intr signal
    // registers when an exception occurs
    reg w_next_vld_is_trap_handler;
    always @(posedge i_clk) begin
        if (i_rst) begin
            w_next_vld_is_trap_handler <= 0;
        end else if (i_excp || i_trap) begin
            w_next_vld_is_trap_handler <= 1;
        end else if (i_valid) begin
            w_next_vld_is_trap_handler <= 0;
        end
    end

    //==============================//
    // RVFI Base Interface Manager
    //==============================//

    wire [4:0] w_rfile_w_addr  = `INSTRN_RD_ADDR(i_instrn);
    wire [4:0] w_rfile_r_addr1 = `INSTRN_RS1_ADDR(i_instrn);
    wire [4:0] w_rfile_r_addr2 = `INSTRN_RS2_ADDR(i_instrn);

    always @(posedge i_clk) begin

        // a valid instruction is retiring if the instruction in wb
        // is not a bubble and does not process an external interrupt
        rvfi_valid <= i_valid;

        // Fixed values
        rvfi_halt <= 0; // Never Halts
        rvfi_mode <= 3; // Machine Mode
        rvfi_ixl  <= 1; // Always 32-bit

        if (i_rst) begin
            rvfi_valid     <= 0;
            rvfi_order     <= 0;
            rvfi_insn      <= 0;
            rvfi_pc_rdata  <= 0;
            rvfi_pc_wdata  <= 0;
            rvfi_rd_addr   <= 0;
            rvfi_rd_wdata  <= 0;
            rvfi_rs1_addr  <= 0;
            rvfi_rs1_rdata <= 0;
            rvfi_rs2_addr  <= 0;
            rvfi_rs2_rdata <= 0;
            rvfi_trap      <= 0;
            rvfi_intr      <= 0;
            rvfi_mem_addr  <= 0;
            rvfi_mem_rmask <= 0;
            rvfi_mem_rdata <= 0;
            rvfi_mem_wmask <= 0;
            rvfi_mem_wdata <= 0;
        end else if (i_valid) begin
            // have a monotonically increasing counter that tracks the instruction order
            rvfi_order <= rvfi_order + 1;

            // current instruction fetched from memory
            rvfi_insn <= i_instrn;

            // pc addresses
            rvfi_pc_rdata <= i_pc_addr;
            rvfi_pc_wdata <= w_pc_next;

            // rfile dest traces
            if (i_rfile_we) begin
                rvfi_rd_addr   <= w_rfile_w_addr;
                rvfi_rd_wdata  <= w_rfile_w_addr != 0 ? i_rfile_w_data : 0;
            end else begin
                rvfi_rd_addr   <= 0;
                rvfi_rd_wdata  <= 0;
            end

            // rfile sources and corresponding data
            if (w_use_rs1) begin
                rvfi_rs1_addr  <= w_rfile_r_addr1;
                rvfi_rs1_rdata <= i_rfile_r_rs1;
            end else begin
                rvfi_rs1_addr  <= 0;
                rvfi_rs1_rdata <= 0;
            end
            if (w_use_rs2) begin
                rvfi_rs2_addr  <= w_rfile_r_addr2;
                rvfi_rs2_rdata <= i_rfile_r_rs2;
            end else begin
                rvfi_rs2_addr  <= 0;
                rvfi_rs2_rdata <= 0;
            end

            // flag if next instruction is a synchronous exception
            rvfi_trap <= i_trap;
            rvfi_intr <= w_next_vld_is_trap_handler;

            // add checks for memory access
            // NOTE: added trap_taken check to avoid showing mem reads/writes
            // on trap
            if (i_op_sel[DCDR_OP_STORE_IDX] && !i_trap) begin
                rvfi_mem_addr  <= i_dmem_addr;
                rvfi_mem_rmask <= 0;
                rvfi_mem_rdata <= 0;
                rvfi_mem_wmask <= i_dmem_sel;
                rvfi_mem_wdata <= i_dmem_w_data;
            end else if (i_op_sel[DCDR_OP_LOAD_IDX] && !i_trap) begin
                rvfi_mem_addr  <= i_dmem_addr;
                rvfi_mem_rmask <= i_dmem_sel;
                rvfi_mem_rdata <= i_dmem_r_data;
                rvfi_mem_wmask <= 0;
                rvfi_mem_wdata <= 0;
            end else begin
                rvfi_mem_addr  <= 0;
                rvfi_mem_rmask <= 0;
                rvfi_mem_rdata <= 0;
                rvfi_mem_wmask <= 0;
                rvfi_mem_wdata <= 0;
            end

            // add csr interface
        `define CSR_MACRO_OP(NAME) \
            rvfi_csr_``NAME``_rmask <= 32'hffff_ffff; \
            rvfi_csr_``NAME``_wmask <= 32'hffff_ffff; \
            rvfi_csr_``NAME``_rdata <= i_``NAME``; \
            rvfi_csr_``NAME``_wdata <= i_``NAME``_next;

            `RVFI_CSR_LIST

        `undef CSR_MACRO_OP
        end

    end

endmodule
