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
    input             i_clk,
    input             i_rst,
    input             i_mask_intrpt,
    input      [31:0] i_intrpt,
    input      [3:0]  i_excp,

    input             i_csr_we,
    input      [4:0]  i_csr_op_sel,
    input      [1:0]  i_csr_rx_sel,
    input      [11:0] i_csr_addr,
    input      [31:0] i_csr_w_data,

    output reg [31:0] o_csr_r_data,

    input      [31:0] i_trap_pc,
    input      [31:0] i_trap_mtval,

`ifdef RISCV_FORMAL

`define CSR_MACRO_OP(NAME) \
    output reg [31:0] o_``NAME``, \
    output reg [31:0] o_``NAME``_next,

    `RVFI_CSR_LIST

`undef CSR_MACRO_OP

`endif

    // values used by pc mux to jump to mtvec/mepc on trap/mret
    output reg [31:0] o_epc_addr,
    output reg        o_excp,
    output reg        o_trap
);



    localparam MCAUSE_INTRPT_BIT   = 1 << 31;
    localparam EPC_ADDR_MASK       = 32'hFFFF_FFFC;

    // temporary registers
    reg [31:0] w_result, w_mcause_excp_next;
    reg [4:0]  w_intrpt_pending;
    reg        w_intrpt_vld;

    // read-only csrs, will flag write attempts as illegal
    wire [31:0] w_mvendorid  = CSR_MVENDORID_VALUE;
    wire [31:0] w_marchid    = CSR_MARCHID_VALUE;
    wire [31:0] w_mimpid     = CSR_MIMPID_VALUE;
    wire [31:0] w_mhartid    = CSR_MHARTID_VALUE;
    wire [31:0] w_mconfigptr = CSR_MCONFIGPTR_VALUE;

    // read-only, WARL (Ignores writes but does not trap)
    wire [31:0] w_misa       = CSR_MISA_VALUE;

    // trap related registers, WARL (Accepts any write, only reads legal result)
    reg [31:0] w_mstatus, w_mie, w_mtvec, w_mstatush, w_mscratch,
               w_mepc, w_mcause, w_mtval, w_mip;

    reg [31:0] w_mstatus_next, w_mie_next, w_mtvec_next, w_mstatush_next,
               w_mscratch_next, w_mepc_next, w_mcause_next, w_mtval_next, w_mip_next;

    // latch writeable csr updates each cycle
    always @(posedge i_clk) begin
        if (i_rst) begin
            w_mstatus  <= 0;
            w_mie      <= 0;
            w_mtvec    <= 0;
            w_mstatush <= 0;
            w_mscratch <= 0;
            w_mepc     <= 0;
            w_mcause   <= 0;
            w_mtval    <= 0;
            w_mip      <= 0;
        end else begin
            w_mstatus  <= w_mstatus_next;
            w_mie      <= w_mie_next;
            w_mtvec    <= w_mtvec_next;
            w_mstatush <= w_mstatush_next;
            w_mscratch <= w_mscratch_next;
            w_mepc     <= w_mepc_next;
            w_mcause   <= w_mcause_next;
            w_mtval    <= w_mtval_next;
            w_mip      <= w_mip_next;
        end
    end

    // external interrupt check
    // if interrupts are globally enabled (mstatus[3] == 1)
    // and mie and mip have the same interrupt bit set, take it
    always @(*) begin
        w_intrpt_vld     = 0;
        w_intrpt_pending = 0;

        if (w_mstatus[3] && (|(w_mie & w_mip))) begin
            // user defined interrupts
            if      (i_intrpt[31]) w_intrpt_pending = 5'd31;
            else if (i_intrpt[30]) w_intrpt_pending = 5'd30;
            else if (i_intrpt[29]) w_intrpt_pending = 5'd29;
            else if (i_intrpt[28]) w_intrpt_pending = 5'd28;
            else if (i_intrpt[27]) w_intrpt_pending = 5'd27;
            else if (i_intrpt[26]) w_intrpt_pending = 5'd26;
            else if (i_intrpt[25]) w_intrpt_pending = 5'd25;
            else if (i_intrpt[24]) w_intrpt_pending = 5'd24;
            else if (i_intrpt[23]) w_intrpt_pending = 5'd23;
            else if (i_intrpt[22]) w_intrpt_pending = 5'd22;
            else if (i_intrpt[21]) w_intrpt_pending = 5'd21;
            else if (i_intrpt[20]) w_intrpt_pending = 5'd20;
            else if (i_intrpt[19]) w_intrpt_pending = 5'd19;
            else if (i_intrpt[18]) w_intrpt_pending = 5'd18;
            else if (i_intrpt[17]) w_intrpt_pending = 5'd17;
            else if (i_intrpt[16]) w_intrpt_pending = 5'd16;
            // M-mode interrupts
            else if (i_intrpt[11]) w_intrpt_pending = 5'd11;
            else if (i_intrpt[7])  w_intrpt_pending = 5'd7;
            else if (i_intrpt[3])  w_intrpt_pending = 5'd3;
            else                   w_intrpt_pending = 5'd0;

            w_intrpt_vld = 1;
        end
    end

    always @(*) begin
        w_mstatus_next  = w_mstatus;
        w_mie_next      = w_mie;
        w_mtvec_next    = w_mtvec;
        w_mstatush_next = w_mstatush;
        w_mscratch_next = w_mscratch;
        w_mepc_next     = w_mepc;
        w_mcause_next   = w_mcause;
        w_mtval_next    = w_mtval;
        w_mip_next      = w_mip | (i_intrpt & CSR_MIP_MASK);
        w_result        = 0;

        o_epc_addr      = (w_mtvec & EPC_ADDR_MASK);
        o_excp          = 0;
        o_trap          = 0;

        if (i_rst) begin
            // NOP
        end else if (w_intrpt_vld && !i_mask_intrpt) begin
            w_mip_next        = 0;
            w_mepc_next       = i_trap_pc;
            w_mcause_next     = MCAUSE_INTRPT_BIT | {27'd0, w_intrpt_pending};
            w_mstatus_next[7] = w_mstatus[3];
            w_mstatus_next[3] = 0;
            o_excp            = 1;
            // if vectored interrupts are enabled, update epc
            if (w_mtvec[1]) begin
                o_epc_addr = (w_mtvec & EPC_ADDR_MASK) + { 25'd0, w_intrpt_pending, 2'd0 };
            end
        end else if (|i_excp) begin
            w_mepc_next       = i_trap_pc;
            w_mcause_next     = w_mcause_excp_next;
            w_mtval_next      = i_trap_mtval;
            w_mstatus_next[7] = w_mstatus[3];
            w_mstatus_next[3] = 0;
            o_trap            = 1;
        end else begin
            case (i_csr_op_sel)
                CSR_OP_SEL_WRITE : begin
                    if (i_csr_we) begin
                        case (i_csr_rx_sel)
                            CSR_FUNCT3_LOW_RW : w_result = i_csr_w_data;
                            CSR_FUNCT3_LOW_RS : w_result = o_csr_r_data |  i_csr_w_data;
                            CSR_FUNCT3_LOW_RC : w_result = o_csr_r_data & ~i_csr_w_data;
                            default   : ;
                        endcase
                        case (i_csr_addr)
                            CSR_MSTATUS_ADDR  : w_mstatus_next  = (w_result & CSR_MSTATUS_MASK)  | (w_mstatus  & ~CSR_MSTATUS_MASK);
                            CSR_MIE_ADDR      : w_mie_next      = (w_result & CSR_MIE_MASK)      | (w_mie      & ~CSR_MIE_MASK);
                            CSR_MTVEC_ADDR    : w_mtvec_next    = (w_result & CSR_MTVEC_MASK)    | (w_mtvec    & ~CSR_MTVEC_MASK);
                            CSR_MSTATUSH_ADDR : w_mstatush_next = (w_result & CSR_MSTATUSH_MASK) | (w_mstatush & ~CSR_MSTATUSH_MASK);
                            CSR_MSCRATCH_ADDR : w_mscratch_next = (w_result & CSR_MSCRATCH_MASK) | (w_mscratch & ~CSR_MSCRATCH_MASK);
                            CSR_MEPC_ADDR     : w_mepc_next     = (w_result & CSR_MEPC_MASK)     | (w_mepc     & ~CSR_MEPC_MASK);
                            CSR_MCAUSE_ADDR   : w_mcause_next   = (w_result & CSR_MCAUSE_MASK)   | (w_mcause   & ~CSR_MCAUSE_MASK);
                            CSR_MTVAL_ADDR    : w_mtval_next    = (w_result & CSR_MTVAL_MASK)    | (w_mtval    & ~CSR_MTVAL_MASK);
                            CSR_MIP_ADDR      : w_mip_next      = (w_result & CSR_MIP_MASK)      | (w_mip      & ~CSR_MIP_MASK);
                            default           : ; // writes to read-only CSRs are ignored
                        endcase
                    end
                end
                CSR_OP_SEL_ECALL : begin
                    w_mepc_next       = i_trap_pc;
                    w_mcause_next     = MCAUSE_CODE_ECALL_M_MODE;
                    w_mstatus_next[7] = w_mstatus[3];
                    w_mstatus_next[3] = 0;
                    o_trap            = 1;
                end
                CSR_OP_SEL_EBREAK : begin
                    w_mepc_next       = i_trap_pc;
                    w_mcause_next     = MCAUSE_CODE_BREAKPOINT;
                    w_mstatus_next[7] = w_mstatus[3];
                    w_mstatus_next[3] = 0;
                    w_mtval_next      = i_trap_mtval;
                    o_trap            = 1;
                end
                CSR_OP_SEL_MRET : begin
                    w_mstatus_next[3] = w_mstatus[7];
                    w_mstatus_next[7] = 1;
                    o_epc_addr        = w_mepc;
                    o_trap            = 1;
                end
                CSR_OP_SEL_WFI : ;
                default: ;
            endcase
        end
    end

    always @(*) begin
        w_mcause_excp_next = 0;
        case (i_excp)
            MCAUSE_SEL_INVLD_INSTRN         : w_mcause_excp_next = MCAUSE_CODE_INVLD_INSTRN;
            MCAUSE_SEL_INSTRN_ADDR_MISALIGN : w_mcause_excp_next = MCAUSE_CODE_INSTRN_ADDR_MISALIGN;
            MCAUSE_SEL_STORE_ADDR_MISALIGN  : w_mcause_excp_next = MCAUSE_CODE_STORE_ADDR_MISALIGN;
            MCAUSE_SEL_LOAD_ADDR_MISALIGN   : w_mcause_excp_next = MCAUSE_CODE_LOAD_ADDR_MISALIGN;
            default                         : ;
        endcase
    end

    // csr combinational read
    always @(*) begin
        o_csr_r_data = 0;
        case (i_csr_addr)
            CSR_MVENDORID_ADDR : o_csr_r_data = w_mvendorid;
            CSR_MARCHID_ADDR   : o_csr_r_data = w_marchid;
            CSR_MIMPID_ADDR    : o_csr_r_data = w_mimpid;
            CSR_MHARTID_ADDR   : o_csr_r_data = w_mhartid;
            CSR_MCONFIGPTR_ADDR: o_csr_r_data = w_mconfigptr;

            CSR_MSTATUS_ADDR   : o_csr_r_data = w_mstatus;
            CSR_MISA_ADDR      : o_csr_r_data = w_misa;
            CSR_MIE_ADDR       : o_csr_r_data = w_mie;
            CSR_MTVEC_ADDR     : o_csr_r_data = w_mtvec;
            CSR_MSTATUSH_ADDR  : o_csr_r_data = w_mstatush;
            CSR_MSCRATCH_ADDR  : o_csr_r_data = w_mscratch;
            CSR_MEPC_ADDR      : o_csr_r_data = w_mepc;
            CSR_MCAUSE_ADDR    : o_csr_r_data = w_mcause;
            CSR_MTVAL_ADDR     : o_csr_r_data = w_mtval;
            CSR_MIP_ADDR       : o_csr_r_data = w_mip;
            default            : ;
        endcase
    end


`ifdef RISCV_FORMAL

    // dummy next registers to conform to rvfi format,
    // not needed outside of formal
    wire [31:0] w_mvendorid_next  = w_mvendorid;
    wire [31:0] w_marchid_next    = w_marchid;
    wire [31:0] w_mimpid_next     = w_mimpid;
    wire [31:0] w_mhartid_next    = w_mhartid;
    wire [31:0] w_mconfigptr_next = w_mconfigptr;

    wire [31:0] w_misa_next       = w_misa;

    always @(*) begin

// attach signal monitors for the csr values if riscv-formal is active
`define CSR_MACRO_OP(NAME) \
    o_``NAME``      = w_``NAME``; \
    o_``NAME``_next = w_``NAME``_next;

        `RVFI_CSR_LIST

`undef CSR_MACRO_OP

        // special case when writing to a csr
        if (!i_rst && !(w_intrpt_vld && !i_mask_intrpt) && !|i_excp && i_csr_op_sel == CSR_OP_SEL_WRITE) begin
            case (i_csr_addr)
                CSR_MVENDORID_ADDR : o_mvendorid_next  = w_result;
                CSR_MARCHID_ADDR   : o_marchid_next    = w_result;
                CSR_MIMPID_ADDR    : o_mimpid_next     = w_result;
                CSR_MHARTID_ADDR   : o_mhartid_next    = w_result;
                CSR_MCONFIGPTR_ADDR: o_mconfigptr_next = w_result;

                CSR_MSTATUS_ADDR   : o_mstatus_next    = w_result;
                CSR_MISA_ADDR      : o_misa_next       = w_result;
                CSR_MIE_ADDR       : o_mie_next        = w_result;
                CSR_MTVEC_ADDR     : o_mtvec_next      = w_result;
                CSR_MSTATUSH_ADDR  : o_mstatush_next   = w_result;
                CSR_MSCRATCH_ADDR  : o_mscratch_next   = w_result;
                CSR_MEPC_ADDR      : o_mepc_next       = w_result;
                CSR_MCAUSE_ADDR    : o_mcause_next     = w_result;
                CSR_MTVAL_ADDR     : o_mtval_next      = w_result;
                CSR_MIP_ADDR       : o_mip_next        = w_result;
                default            : ;
            endcase
        end
    end

`endif


endmodule
