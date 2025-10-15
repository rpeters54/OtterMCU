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
`define CSR_MACRO_OP(NAME) \
    output reg [31:0] rvfi_csr_``NAME``_rmask, \
    output reg [31:0] rvfi_csr_``NAME``_wmask, \
    output reg [31:0] rvfi_csr_``NAME``_rdata, \
    output reg [31:0] rvfi_csr_``NAME``_wdata,

module otter_csr (
    input             clk,
    input      [31:0] intrpt,

    input      [2:0]  op_sel,
    input      [2:0]  trap_cause_sel,
    input      [1:0]  funct3_low,
    input             w_en,
    input      [11:0] addr,
    input      [31:0] pc_addr,
    input      [31:0] w_data,
    input      [31:0] mtval_trap_addr,

    output reg        intrpt_vld,
    output reg        illegal_write,
    output reg        addr_vld,
    output reg [31:0] r_data,

`ifdef RISCV_FORMAL
    input             next_rvfi_valid,
    `RVFI_CSR_LIST
`endif

    // values used by pc mux to jump to mtvec/mepc on trap/mret
    output reg [31:0] pc_mtvec_addr,
    output reg [31:0] pc_mepc_addr
);

`undef CSR_MACRO_OP
    // Read-Only Values (MISA treated as read-only zero)
    localparam CSR_MVENDORID_VALUE  = 32'h0000_0000;
    localparam CSR_MARCHID_VALUE    = 32'h0000_0000;
    localparam CSR_MIMPID_VALUE     = 32'h0000_0000;
    localparam CSR_MHARTID_VALUE    = 32'h0000_0000;
    localparam CSR_MCONFIGPTR_VALUE = 32'h0000_0000;
    localparam CSR_MISA_VALUE       = 32'h4000_0100; // RV32I + Zicsr

    // All Register Masks
    localparam CSR_MSTATUS_MASK    = 32'h0000_0088; // bit 3 = MIE and bit 7 = MPIE are writeable, remaining bits are ignored
    localparam CSR_MSTATUSH_MASK   = 32'h0000_0000; // everything is left as default
    localparam CSR_MIE_MASK        = 32'hFFFF_0888; // bits 31-15 = Custom IRQs, bit 11 = External IRQ, bit 7 = Timer IRQ, bit 3 = Software IRQ
    localparam CSR_MTVEC_MASK      = 32'hFFFF_FFFD; // bit 1 must always remain zero, MODE can only be 0 = Direct, 1 = Vec
    localparam CSR_MCAUSE_MASK     = 32'h0000_001F; // only lowest 5 bits of mcause are writeable
    localparam CSR_MEPC_MASK       = 32'hFFFF_FFFC; // 2 lsbs of mepc are not writeable
    localparam CSR_MTVAL_MASK      = 32'hFFFF_FFFF; // mtval is fully writeable by software
    localparam CSR_MSCRATCH_MASK   = 32'hFFFF_FFFF; // user-defined scratch space, the world is your oyster

    localparam MCAUSE_INTRPT_BIT = 1 << 31;

    // temporary registers
    reg [31:0] result;
    reg [4:0]  intrpt_pending;

    // read-only csrs, will flag write attempts as illegal
    wire [31:0] mvendorid  = CSR_MVENDORID_VALUE;
    wire [31:0] marchid    = CSR_MARCHID_VALUE;
    wire [31:0] mimpid     = CSR_MIMPID_VALUE;
    wire [31:0] mhartid    = CSR_MHARTID_VALUE;
    wire [31:0] mconfigptr = CSR_MCONFIGPTR_VALUE;

    localparam CSR_MIP_INTRPT_MASK      = 32'hFFFF_0888;
    // read-only, WARL (Ignores writes but does not trap)
    wire [31:0] misa = CSR_MISA_VALUE;
    wire [31:0] mip  = intrpt & CSR_MIP_INTRPT_MASK;

    // trap related registers, WARL (Accepts any write, only reads legal result)
    reg [31:0] mstatus, mie, mtvec, mstatush, mscratch,
               mepc, mcause, mtval;

    reg [31:0] mstatus_next, mie_next, mtvec_next, mstatush_next,
               mscratch_next, mepc_next, mcause_next, mtval_next;


    // latch writeable csr updates each cycle
    always @(posedge clk) begin
        mstatus  <= mstatus_next;
        mie      <= mie_next;
        mtvec    <= mtvec_next;
        mstatush <= mstatush_next;
        mscratch <= mscratch_next;
        mepc     <= mepc_next;
        mcause   <= mcause_next;
        mtval    <= mtval_next;
    end

    always @(*) begin

        mstatus_next  = mstatus;
        mie_next      = mie;
        mtvec_next    = mtvec;
        mstatush_next = mstatush;
        mscratch_next = mscratch;
        mepc_next     = mepc;
        mcause_next   = mcause;
        mtval_next    = mtval;

        pc_mepc_addr  = mepc;
        pc_mtvec_addr = mtvec & 32'hFFFF_FFFC;

        //reset all writable registers to zero
        case (op_sel)
            CSR_OP_RESET : begin
                mstatus_next  = 0;
                mie_next      = 0;
                mtvec_next    = 0;
                mstatush_next = 0;
                mscratch_next = 0;
                mepc_next     = 0;
                mcause_next   = 0;
                mtval_next    = 0;
            end
            CSR_OP_WRITE : begin
                // writeback csr instruction changes if enabled
                if (w_en) begin
                    case (addr) 
                        CSR_MSTATUS_ADDR  : mstatus_next  = (result & CSR_MSTATUS_MASK)  | (mstatus  & ~CSR_MSTATUS_MASK);
                        CSR_MIE_ADDR      : mie_next      = (result & CSR_MIE_MASK)      | (mie      & ~CSR_MIE_MASK);
                        CSR_MTVEC_ADDR    : mtvec_next    = (result & CSR_MTVEC_MASK)    | (mtvec    & ~CSR_MTVEC_MASK);
                        CSR_MSTATUSH_ADDR : mstatush_next = (result & CSR_MSTATUSH_MASK) | (mstatush & ~CSR_MSTATUSH_MASK);
                        CSR_MSCRATCH_ADDR : mscratch_next = (result & CSR_MSCRATCH_MASK) | (mscratch & ~CSR_MSCRATCH_MASK);
                        CSR_MEPC_ADDR     : mepc_next     = (result & CSR_MEPC_MASK)     | (mepc     & ~CSR_MEPC_MASK);
                        CSR_MCAUSE_ADDR   : mcause_next   = (result & CSR_MCAUSE_MASK)   | (mcause   & ~CSR_MCAUSE_MASK);
                        CSR_MTVAL_ADDR    : mtval_next    = (result & CSR_MTVAL_MASK)    | (mtval    & ~CSR_MTVAL_MASK);
                        default           : ;
                    endcase
                end
            end
            CSR_OP_ECALL : begin
                mepc_next       = pc_addr;
                mcause_next     = MCAUSE_CODE_ECALL_M_MODE;
                mstatus_next[7] = mstatus[3];
                mstatus_next[3] = 0;
            end
            CSR_OP_EBREAK : begin
                mepc_next       = pc_addr;
                mcause_next     = MCAUSE_CODE_BREAKPOINT;
                mstatus_next[7] = mstatus[3];
                mstatus_next[3] = 0;
                mtval_next      = mtval_trap_addr;
            end
            CSR_OP_MRET : begin
                mcause_next     = 0;
                mstatus_next[3] = mstatus[7];
                mstatus_next[7] = 1;
            end
            CSR_OP_INTRPT : begin
                mepc_next       = pc_addr;
                mcause_next     = MCAUSE_INTRPT_BIT | {27'd0, intrpt_pending};
                mstatus_next[7] = mstatus[3];
                mstatus_next[3] = 0;

                // if vectored interrupts are enabled, adjust mtvec jump location
                if (mtvec[1]) begin
                    pc_mtvec_addr = mtvec & 32'hFFFF_FFFC + { 25'd0, intrpt_pending, 2'd0 };
                end
            end
            CSR_OP_TRAP : begin
                mepc_next       = pc_addr;
                mstatus_next[7] = mstatus[3];
                mstatus_next[3] = 0;
                mtval_next      = mtval_trap_addr;
                case (trap_cause_sel)
                    TRAP_CAUSE_SEL_INSTRN_ADDR_MISALIGN : mcause_next = MCAUSE_CODE_INSTRN_ADDR_MISALIGN;
                    TRAP_CAUSE_SEL_INVLD_INSTRN         : mcause_next = MCAUSE_CODE_INVLD_INSTRN;
                    TRAP_CAUSE_SEL_LOAD_ADDR_MISALIGN   : mcause_next = MCAUSE_CODE_LOAD_ADDR_MISALIGN;
                    TRAP_CAUSE_SEL_STORE_ADDR_MISALIGN  : mcause_next = MCAUSE_CODE_STORE_ADDR_MISALIGN;
                    default                             : ;
                endcase
            end
            CSR_OP_WFI : ; // nop
            default : ;
        endcase
    end

    // external interrupt check
    // if interrupts are globally enabled (mstatus[3] == 1)
    // and mie and mip have the same interrupt bit set, take it
    always @(*) begin
        intrpt_vld     =  0;
        intrpt_pending = '0;

        if (mstatus[3] && (|(mie & mip))) begin
            // user defined interrupts
            if      (intrpt[31]) intrpt_pending = 5'd31;
            else if (intrpt[30]) intrpt_pending = 5'd30;
            else if (intrpt[29]) intrpt_pending = 5'd29;
            else if (intrpt[28]) intrpt_pending = 5'd28;
            else if (intrpt[27]) intrpt_pending = 5'd27;
            else if (intrpt[26]) intrpt_pending = 5'd26;
            else if (intrpt[25]) intrpt_pending = 5'd25;
            else if (intrpt[24]) intrpt_pending = 5'd24;
            else if (intrpt[23]) intrpt_pending = 5'd23;
            else if (intrpt[22]) intrpt_pending = 5'd22;
            else if (intrpt[21]) intrpt_pending = 5'd21;
            else if (intrpt[20]) intrpt_pending = 5'd20;
            else if (intrpt[19]) intrpt_pending = 5'd19;
            else if (intrpt[18]) intrpt_pending = 5'd18;
            else if (intrpt[17]) intrpt_pending = 5'd17;
            else if (intrpt[16]) intrpt_pending = 5'd16;
            // M-mode interrupts
            else if (intrpt[11]) intrpt_pending = 5'd11;
            else if (intrpt[7])  intrpt_pending = 5'd7;
            else if (intrpt[3])  intrpt_pending = 5'd3;
            else                 intrpt_pending = 5'd0;

            intrpt_vld = 1;
        end
    end


    // csr combinational read
    // notifies decoder about illegal reads (addr_vld == 0)
    always @(*) begin
        illegal_write  = 1;
        addr_vld       = 0;
        r_data         = 0;

        case (addr)
            CSR_MVENDORID_ADDR : begin r_data = mvendorid;  addr_vld = 1; end
            CSR_MARCHID_ADDR   : begin r_data = marchid;    addr_vld = 1; end
            CSR_MIMPID_ADDR    : begin r_data = mimpid;     addr_vld = 1; end
            CSR_MHARTID_ADDR   : begin r_data = mhartid;    addr_vld = 1; end
            CSR_MCONFIGPTR_ADDR: begin r_data = mconfigptr; addr_vld = 1; end

            CSR_MSTATUS_ADDR   : begin r_data = mstatus;    illegal_write = 0; addr_vld = 1; end
            CSR_MISA_ADDR      : begin r_data = misa;       illegal_write = 0; addr_vld = 1; end
            CSR_MIE_ADDR       : begin r_data = mie;        illegal_write = 0; addr_vld = 1; end
            CSR_MTVEC_ADDR     : begin r_data = mtvec;      illegal_write = 0; addr_vld = 1; end
            CSR_MSTATUSH_ADDR  : begin r_data = mstatush;   illegal_write = 0; addr_vld = 1; end
            CSR_MSCRATCH_ADDR  : begin r_data = mscratch;   illegal_write = 0; addr_vld = 1; end
            CSR_MEPC_ADDR      : begin r_data = mepc;       illegal_write = 0; addr_vld = 1; end
            CSR_MCAUSE_ADDR    : begin r_data = mcause;     illegal_write = 0; addr_vld = 1; end
            CSR_MTVAL_ADDR     : begin r_data = mtval;      illegal_write = 0; addr_vld = 1; end
            CSR_MIP_ADDR       : begin r_data = mip;        illegal_write = 0; addr_vld = 1; end
            default: begin
                r_data         = 0;
                addr_vld       = 0;
                illegal_write  = 1;
            end
        endcase
    end

    // CSRxx instruction ALU
    always @(*) begin
        case (funct3_low) 
            CSR_FUNCT3_LOW_RW : result = w_data;
            CSR_FUNCT3_LOW_RS : result = r_data |  w_data;
            CSR_FUNCT3_LOW_RC : result = r_data & ~w_data;
            default           : result = '0;
        endcase
    end


`ifdef RISCV_FORMAL

    // dummy next registers to conform to rvfi format,
    // not needed outside of formal
    wire [31:0] mvendorid_next  = mvendorid;
    wire [31:0] marchid_next    = marchid;
    wire [31:0] mimpid_next     = mimpid;
    wire [31:0] mhartid_next    = mhartid;
    wire [31:0] mconfigptr_next = mconfigptr;

    wire [31:0] misa_next       = misa;
    wire [31:0] mip_next        = mip;

    // present csr values for rvfi
    always @(posedge clk) begin
        if (next_rvfi_valid) begin
        `define CSR_MACRO_OP(NAME) \
            rvfi_csr_``NAME``_rmask <= 32'hffff_ffff; \
            rvfi_csr_``NAME``_wmask <= 32'hffff_ffff; \
            rvfi_csr_``NAME``_rdata <= ``NAME``; \
            rvfi_csr_``NAME``_wdata <= ``NAME``_next;

            `RVFI_CSR_LIST
        `undef CSR_MACRO_OP
            if (w_en) begin
                case (addr)
                    CSR_MVENDORID_ADDR : rvfi_csr_mvendorid_wdata  <= result;
                    CSR_MARCHID_ADDR   : rvfi_csr_marchid_wdata    <= result;
                    CSR_MIMPID_ADDR    : rvfi_csr_mimpid_wdata     <= result;
                    CSR_MHARTID_ADDR   : rvfi_csr_mhartid_wdata    <= result;
                    CSR_MCONFIGPTR_ADDR: rvfi_csr_mconfigptr_wdata <= result;

                    CSR_MSTATUS_ADDR   : rvfi_csr_mstatus_wdata    <= result;
                    CSR_MISA_ADDR      : rvfi_csr_misa_wdata       <= result;
                    CSR_MIE_ADDR       : rvfi_csr_mie_wdata        <= result;
                    CSR_MTVEC_ADDR     : rvfi_csr_mtvec_wdata      <= result;
                    CSR_MSTATUSH_ADDR  : rvfi_csr_mstatush_wdata   <= result;
                    CSR_MSCRATCH_ADDR  : rvfi_csr_mscratch_wdata   <= result;
                    CSR_MEPC_ADDR      : rvfi_csr_mepc_wdata       <= result;
                    CSR_MCAUSE_ADDR    : rvfi_csr_mcause_wdata     <= result;
                    CSR_MTVAL_ADDR     : rvfi_csr_mtval_wdata      <= result;
                    CSR_MIP_ADDR       : rvfi_csr_mip_wdata        <= result;
                    default            : ;
                endcase
            end
        end
    end

`endif

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
        end
    end
`endif
endmodule
