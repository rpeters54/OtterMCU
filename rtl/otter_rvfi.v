
`timescale 1ns / 1ps
`include "otter_defines.vh"

module otter_rvfi (
    input             i_clk,
    input             i_rst,
    input             i_stall,
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

    // imem interface
    input      [31:0] i_imem_addr,
    input      [31:0] i_imem_r_data,

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
    `RVFI_BUS_OUTPUTS

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

    //==============================//
    // RVFI IMEM/DMEM Bus Interface
    //==============================//

    reg w_stall_q;
    always @(posedge i_clk) begin
        if (i_rst) begin
            w_stall_q  <= 0;
        end else begin
            w_stall_q  <= i_stall;
        end
    end

    // ibus read address queue
    reg [31:0] w_ibus_addr_q;
    always @(posedge i_clk) begin
        if (i_rst) begin
            w_ibus_addr_q <= 0;
        end else if (!i_stall) begin
            w_ibus_addr_q <= i_imem_addr;
        end
    end

    // dbus read address queue
    reg [31:0] w_dbus_addr_q;
    always @(posedge i_clk) begin
        if (i_rst) begin
            w_dbus_addr_q <= 0;
        end else if (i_stall && i_op_sel[DCDR_OP_LOAD_IDX]) begin
            w_dbus_addr_q <= i_dmem_addr;
        end
    end

    // dbus interface signal handlers
    reg                   w_dbus_valid;
    wire                  w_dbus_insn    = 0;
    wire                  w_dbus_data    = 1;
    wire                  w_dbus_fault   = 0;
    reg  [XLEN - 1:0]     w_dbus_addr;
    reg  [XLEN / 8 - 1:0] w_dbus_rmask;
    reg  [XLEN / 8 - 1:0] w_dbus_wmask;
    reg  [XLEN - 1:0]     w_dbus_rdata;
    reg  [XLEN - 1:0]     w_dbus_wdata;
    always @(posedge i_clk) begin
        if (i_rst || i_trap || i_excp) begin
            w_dbus_valid <= 0;
            w_dbus_addr  <= 0;
            w_dbus_rmask <= 0;
            w_dbus_wmask <= 0;
            w_dbus_rdata <= 0;
            w_dbus_wdata <= 0;

        end else begin
            w_dbus_valid <= 0;
            w_dbus_addr  <= 0;
            w_dbus_rmask <= 0;
            w_dbus_wmask <= 0;
            w_dbus_rdata <= 0;
            w_dbus_wdata <= 0;

            // on write accept, present write transaction to interface
            if (i_op_sel[DCDR_OP_STORE_IDX]) begin
                w_dbus_valid <= 1;
                w_dbus_addr  <= i_dmem_addr;
                w_dbus_wdata <= i_dmem_w_data;
                w_dbus_wmask <= i_dmem_sel;
            end

            // on read acknowledge present read transaction to interface
            if (w_stall_q && i_op_sel[DCDR_OP_LOAD_IDX]) begin
                w_dbus_valid <= 1;
                w_dbus_rdata <= i_dmem_r_data;
                w_dbus_addr  <= w_dbus_addr_q;
                w_dbus_rmask <= 4'b1111;
            end
        end
    end

    // ibus interface signal handlers
    reg                   w_ibus_valid;
    wire                  w_ibus_insn    = 1;
    wire                  w_ibus_data    = 0;
    wire                  w_ibus_fault   = 0;
    reg  [XLEN     - 1:0] w_ibus_addr;
    reg  [XLEN / 8 - 1:0] w_ibus_rmask;
    wire [XLEN / 8 - 1:0] w_ibus_wmask   = 0;
    reg  [XLEN     - 1:0] w_ibus_rdata;
    wire [XLEN     - 1:0] w_ibus_wdata   = 0;
    always @(posedge i_clk) begin
        if (i_rst) begin
            w_ibus_valid <= 0;
            w_ibus_addr  <= 0;
            w_ibus_rmask <= 0;
            w_ibus_rdata <= 0;
        end else begin
            w_ibus_valid <= 0;
            w_ibus_addr  <= 0;
            w_ibus_rmask <= 0;
            w_ibus_rdata <= 0;

            // on read acknowledge present read transaction to interface
            if (!w_stall_q) begin
                w_ibus_valid <= 1;
                w_ibus_rdata <= i_imem_r_data;
                w_ibus_addr  <= w_ibus_addr_q;
                w_ibus_rmask <= 4'b1111;
            end
        end
    end

    // rvfi signal concatenates the dbus and ibus ports
    assign rvfi_bus_valid = {w_dbus_valid, w_ibus_valid};
    assign rvfi_bus_insn  = {w_dbus_insn,  w_ibus_insn};
    assign rvfi_bus_data  = {w_dbus_data,  w_ibus_data};
    assign rvfi_bus_fault = {w_dbus_fault, w_ibus_fault};
    assign rvfi_bus_addr  = {w_dbus_addr,  w_ibus_addr};
    assign rvfi_bus_rmask = {w_dbus_rmask, w_ibus_rmask};
    assign rvfi_bus_wmask = {w_dbus_wmask, w_ibus_wmask};
    assign rvfi_bus_rdata = {w_dbus_rdata, w_ibus_rdata};
    assign rvfi_bus_wdata = {w_dbus_wdata, w_ibus_wdata};

endmodule
