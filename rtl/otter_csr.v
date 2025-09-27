`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/25/2022 06:08:24 PM
// Design Name: 
// Module Name: CSR
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
//////////////////////////////////////////////////////////////////////////////////

`include "otter_defines.vh"

module otter_csr (
    input             clk,
    input             rst, 
    input             ext_intrpt,

    input      [2:0]  op,
    input      [1:0]  funct3_low,
    input             w_en,
    input      [11:0] addr,
    input      [31:0] pc_addr,
    input      [31:0] w_data,

    output reg        intrpt_vld,
    output reg        read_only,
    output reg        addr_vld,
    output reg [31:0] r_data,
    output reg [31:0] mtvec,
    output reg [31:0] mepc
);

    // trap related registers, WARL (Accepts any write, only reads legal result)
    reg [31:0] mstatus, misa, mie, mstatush, mscratch, mcause, mtval, mip;

    // temporary registers
    reg [31:0] result;

    // read-only csrs, will flag write attempts as illegal
    wire [31:0] mvendorid  = CSR_MVENDORID_VALUE;
    wire [31:0] marchid    = CSR_MARCHID_VALUE;
    wire [31:0] mimpid     = CSR_MIMPID_VALUE;
    wire [31:0] mhartid    = CSR_MHARTID_VALUE;
    wire [31:0] mconfigptr = CSR_MCONFIGPTR_VALUE;

    always @(posedge clk) begin
        //reset all writable registers to zero
        if (rst) begin
            mstatus  <= '0; 
            misa     <= 32'h40000100;
            mie      <= '0;
            mtvec    <= '0; 
            mstatush <= '0; 
            mscratch <= '0; 
            mepc     <= '0; 
            mcause   <= '0;
            mtval    <= '0; 
            mip      <= '0;
        end else begin
            case (op)
                CSR_OP_WRITE : begin
                    // writeback csr instruction changes if enabled
                    if (w_en) begin
                        case (addr) 
                            CSR_MSTATUS_ADDR  : mstatus  <= result & CSR_MSTATUS_MASK; 
                            CSR_MISA_ADDR     : misa     <= misa;
                            CSR_MIE_ADDR      : mie      <= result & CSR_MIE_MASK; 
                            CSR_MTVEC_ADDR    : mtvec    <= result & CSR_MTVEC_MASK; 
                            CSR_MSTATUSH_ADDR : mstatush <= result & CSR_MSTATUSH_MASK; 
                            CSR_MSCRATCH_ADDR : mscratch <= result & CSR_MSCRATCH_MASK; 
                            CSR_MEPC_ADDR     : mepc     <= result & CSR_MEPC_MASK; 
                            CSR_MCAUSE_ADDR   : mcause   <= result & CSR_MCAUSE_MASK; 
                            CSR_MTVAL_ADDR    : mtval    <= result & CSR_MTVAL_MASK; 
                            CSR_MIP_ADDR      : mip      <= (result & CSR_MIP_MASK) | (mip & ~CSR_MIP_MASK);
                            default           : begin end
                        endcase
                    end
                end
                CSR_OP_ECALL : begin
                    mepc       <= pc_addr;
                    mcause     <= MCAUSE_ECALL_M_MODE;
                    mstatus[7] <= mstatus[3];
                    mstatus[3] <= '0;
                end
                CSR_OP_EBREAK : begin
                    mepc       <= pc_addr;
                    mcause     <= MCAUSE_BREAKPOINT;
                    mstatus[7] <= mstatus[3];
                    mstatus[3] <= '0;
                end
                CSR_OP_MRET : begin
                    mcause     <= '0;
                    mstatus[3] <= mstatus[7];
                    mstatus[7] <= '1;
                end
                CSR_OP_INTRPT : begin
                    mepc       <= pc_addr;
                    mcause     <= 1 << 31 | CSR_MEIE_BIT;
                    mstatus[7] <= mstatus[3];
                    mstatus[3] <= '0;
                end
                CSR_OP_TRAP : begin
                    mepc       <= pc_addr;
                    mcause     <= MCAUSE_BREAKPOINT;
                    mstatus[7] <= mstatus[3];
                    mstatus[3] <= '0;
                end
                CSR_OP_WFI : ; // nop
                default : ;
            endcase
            // mip should update external interrupt check
            // regardless of operation
            mip[11] <= ext_intrpt;
        end 
    end

    always @(*) begin
        intrpt_vld = 1'b0;
        read_only  = 1'b1;
        addr_vld   = 1'b0;
        r_data     = 32'h0;
        result     = 32'h0;

        // external interrupt check
        if (mstatus[3] && (|(mie & mip))) begin
             intrpt_vld = 1'b1;
        end

        // get value and useful information about CSR pointed to
        // helps with tracking illegal instructions
        case (addr)
            CSR_MVENDORID_ADDR : begin r_data = mvendorid;  addr_vld = 1'b1; end
            CSR_MARCHID_ADDR   : begin r_data = marchid;    addr_vld = 1'b1; end
            CSR_MIMPID_ADDR    : begin r_data = mimpid;     addr_vld = 1'b1; end
            CSR_MHARTID_ADDR   : begin r_data = mhartid;    addr_vld = 1'b1; end
            CSR_MCONFIGPTR_ADDR: begin r_data = mconfigptr; addr_vld = 1'b1; end

            CSR_MSTATUS_ADDR   : begin r_data = mstatus;    read_only = 1'b0; addr_vld = 1'b1; end
            CSR_MISA_ADDR      : begin r_data = misa;       read_only = 1'b0; addr_vld = 1'b1; end
            CSR_MIE_ADDR       : begin r_data = mie;        read_only = 1'b0; addr_vld = 1'b1; end
            CSR_MTVEC_ADDR     : begin r_data = mtvec;      read_only = 1'b0; addr_vld = 1'b1; end
            CSR_MSTATUSH_ADDR  : begin r_data = mstatush;   read_only = 1'b0; addr_vld = 1'b1; end
            CSR_MSCRATCH_ADDR  : begin r_data = mscratch;   read_only = 1'b0; addr_vld = 1'b1; end
            CSR_MEPC_ADDR      : begin r_data = mepc;       read_only = 1'b0; addr_vld = 1'b1; end
            CSR_MCAUSE_ADDR    : begin r_data = mcause;     read_only = 1'b0; addr_vld = 1'b1; end
            CSR_MTVAL_ADDR     : begin r_data = mtval;      read_only = 1'b0; addr_vld = 1'b1; end
            CSR_MIP_ADDR       : begin r_data = mip;        read_only = 1'b0; addr_vld = 1'b1; end
            default: begin
                r_data     = 32'h0;
                addr_vld   = 1'b0;
                read_only  = 1'b1;
            end
        endcase
        case (funct3_low) 
            CSR_FUNCT3_LOW_RW : result = w_data;
            CSR_FUNCT3_LOW_RS : result = r_data |  w_data;
            CSR_FUNCT3_LOW_RC : result = r_data & ~w_data;
            default           : result = 32'hDEAD_BEEF;
        endcase
    end

`ifdef FORMAL
    reg f_past_valid;
    reg f_past_rst;
    reg f_past_w_en;
    reg f_past_ext_intrpt;

    // Registers to hold previous cycle's inputs/state for assertions
    reg [2:0]  f_past_op;
    reg [31:0] f_past_mstatus;
    reg [31:0] f_past_misa;
    reg [31:0] f_past_pc_addr;
    reg [31:0] f_past_addr;

    initial begin
        f_past_valid = 0;
        f_past_rst = 0;
        f_past_w_en = 0;
        f_past_ext_intrpt = 0;
        f_past_op = 0;
        f_past_mstatus = 0;
        f_past_misa = 0;
        f_past_pc_addr = 0;
    end

    always @(posedge clk) begin
        f_past_valid   <= 1;
        f_past_rst     <= rst;
        f_past_w_en    <= w_en;
        f_past_ext_intrpt <= ext_intrpt;
        f_past_op      <= op;
        f_past_mstatus <= mstatus;
        f_past_misa    <= misa;
        f_past_pc_addr <= pc_addr;
        f_past_addr    <= addr;
    end

    // --- Assumptions ---
    always @(*) begin
        // Assume reset is asserted at the beginning
        if (!f_past_valid) assume(rst);
    end

    // --- Assertions ---
    always @(posedge clk) begin
        if (!rst && !f_past_rst && f_past_valid) begin
            // Property: MISA is read-only and must never change its value after reset.
            A_MISA_READ_ONLY: assert(misa == f_past_misa);

            // Property: On an ECALL, mepc, mcause, and mstatus must be updated correctly.
            if (f_past_op == CSR_OP_ECALL) begin
                A_ECALL_MEPC:   assert(mepc == f_past_pc_addr);
                A_ECALL_MCAUSE: assert(mcause == MCAUSE_ECALL_M_MODE);
                A_ECALL_MSTATUS:assert(mstatus[3] == 1'b0 && mstatus[7] == f_past_mstatus[3]);
            end

            // Property: On an MRET, the mstatus interrupt stack must be popped correctly.
            if (f_past_op == CSR_OP_MRET) begin
                A_MRET_MSTATUS: assert(mstatus[3] == f_past_mstatus[7] && mstatus[7] == 1'b1);
            end

            // Property: On an interrupt, mepc, mcause, and mstatus must be updated correctly.
            if (f_past_op == CSR_OP_INTRPT) begin
                A_INTRPT_MEPC:   assert(mepc == f_past_pc_addr);
                A_INTRPT_MCAUSE: assert(mcause == 1 << 31 | CSR_MEIE_BIT);
                A_INTRPT_MSTATUS:assert(mstatus[3] == 1'b0 && mstatus[7] == f_past_mstatus[3]);
            end

            // Property: A CSR write to MIP must not affect the MEIP bit (11).
            // It must always reflect the value of ext_intrpt from the previous cycle.
            if (f_past_op == CSR_OP_WRITE && f_past_w_en && f_past_addr == CSR_MIP_ADDR) begin
                A_MIP_MEIP_READ_ONLY: assert(mip[11] == f_past_ext_intrpt);
            end
        end
    end

    // Combinational Assertions
    always @(*) begin
        // Property: The intrpt_vld signal must correctly reflect the interrupt status.
        A_INTRPT_VLD_LOGIC: assert(intrpt_vld == (mstatus[3] && (|(mie & mip))));
    end

    // --- Coverage Checks ---
    always @(posedge clk) begin
        if (!rst) begin
            C_ECALL: cover(op == CSR_OP_ECALL);
            C_MRET:  cover(op == CSR_OP_MRET);
            C_IRQ:   cover(op == CSR_OP_INTRPT);
            C_MIP_WRITE: cover(op == CSR_OP_WRITE && w_en && addr == CSR_MIP_ADDR);
        end
    end
`endif
endmodule
