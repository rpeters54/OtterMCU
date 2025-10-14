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

    // trap related registers, WARL (Accepts any write, only reads legal result)
    output reg [31:0] mstatus,
    output     [31:0] misa,
    output reg [31:0] mie,
    output reg [31:0] mtvec,
    output reg [31:0] mstatush,
    output reg [31:0] mscratch,
    output reg [31:0] mepc,
    output reg [31:0] mcause,
    output reg [31:0] mtval,
    output     [31:0] mip,

    // read-only csrs, will flag write attempts as illegal
    output     [31:0] mvendorid,
    output     [31:0] marchid,
    output     [31:0] mimpid,
    output     [31:0] mhartid,
    output     [31:0] mconfigptr
);
    // Read-Only Values (MISA treated as read-only zero)
    localparam CSR_MVENDORID_VALUE  = 32'h0000_0000;
    localparam CSR_MARCHID_VALUE    = 32'h0000_0000;
    localparam CSR_MIMPID_VALUE     = 32'h0000_0000;
    localparam CSR_MHARTID_VALUE    = 32'h0000_0000;
    localparam CSR_MCONFIGPTR_VALUE = 32'h0000_0000;
    localparam CSR_MISA_VALUE       = 32'h4000_0100; // RV32I + Zicsr

    // All Writeable Register Masks
    // bit 3 = MIE and bit 7 = MPIE are writeable, remaining bits are ignored
    localparam CSR_MSTATUS_MASK  = 32'h0000_0088;
    // everything is left as default
    localparam CSR_MSTATUSH_MASK = 32'h0000_0000;

    // bit 11 = External IRQ, bit 7 = Timer IRQ, bit 3 = Software IRQ
    // bits 31-15 = Custom IRQs
    localparam CSR_MIE_MASK      = 32'hFFFF_0888;
    localparam CSR_MIP_MASK      = 32'hFFFF_0888;
    // bit 1 must always remain zero, MODE can only be 0 = Direct, 1 = Vec
    // for ease of implementation, I only allow direct
    localparam CSR_MTVEC_MASK    = 32'hFFFF_FFFC;
    localparam CSR_MCAUSE_MASK   = 32'h0000_001F;
    localparam CSR_MEPC_MASK     = 32'hFFFF_FFFC;
    // mtval is fully writeable by software
    localparam CSR_MTVAL_MASK    = 32'hFFFF_FFFF;
    // user-defined scratch space, the world is your oyster
    localparam CSR_MSCRATCH_MASK = 32'hFFFF_FFFF;

    // temporary registers
    reg [31:0] result;
    reg [4:0]  intrpt_pending;

    // read-only, write-illegal csr values
    assign mvendorid  = CSR_MVENDORID_VALUE;
    assign marchid    = CSR_MARCHID_VALUE;
    assign mimpid     = CSR_MIMPID_VALUE;
    assign mhartid    = CSR_MHARTID_VALUE;
    assign mconfigptr = CSR_MCONFIGPTR_VALUE;

    // read-only, ignore-write csr values
    assign misa = CSR_MISA_VALUE;
    assign mip  = intrpt & CSR_MIP_MASK;

    always @(posedge clk) begin
        //reset all writable registers to zero
        case (op_sel)
            CSR_OP_RESET : begin
                mstatus  <= '0;
                mie      <= '0;
                mtvec    <= '0;
                mstatush <= '0;
                mscratch <= '0;
                mepc     <= '0;
                mcause   <= '0;
                mtval    <= '0;
            end
            CSR_OP_WRITE : begin
                // writeback csr instruction changes if enabled
                if (w_en) begin
                    case (addr) 
                        CSR_MSTATUS_ADDR  : mstatus  <= (result & CSR_MSTATUS_MASK)  | (mstatus  & ~CSR_MSTATUS_MASK);
                        CSR_MISA_ADDR     : /* MISA is read-only, writes are ignored */;
                        CSR_MIE_ADDR      : mie      <= (result & CSR_MIE_MASK)      | (mie      & ~CSR_MIE_MASK);
                        CSR_MTVEC_ADDR    : mtvec    <= (result & CSR_MTVEC_MASK)    | (mtvec    & ~CSR_MTVEC_MASK);
                        CSR_MSTATUSH_ADDR : mstatush <= (result & CSR_MSTATUSH_MASK) | (mstatush & ~CSR_MSTATUSH_MASK);
                        CSR_MSCRATCH_ADDR : mscratch <= (result & CSR_MSCRATCH_MASK) | (mscratch & ~CSR_MSCRATCH_MASK);
                        CSR_MEPC_ADDR     : mepc     <= (result & CSR_MEPC_MASK)     | (mepc     & ~CSR_MEPC_MASK);
                        CSR_MCAUSE_ADDR   : mcause   <= (result & CSR_MCAUSE_MASK)   | (mcause   & ~CSR_MCAUSE_MASK);
                        CSR_MTVAL_ADDR    : mtval    <= (result & CSR_MTVAL_MASK)    | (mtval    & ~CSR_MTVAL_MASK);
                        CSR_MIP_ADDR      : /* MIP is read-only, writes are ignored */;
                        default           : ;
                    endcase
                end
            end
            CSR_OP_ECALL : begin
                mepc       <= pc_addr;
                mcause     <= MCAUSE_CODE_ECALL_M_MODE;
                mstatus[7] <= mstatus[3];
                mstatus[3] <= '0;
            end
            CSR_OP_EBREAK : begin
                mepc       <= pc_addr;
                mcause     <= MCAUSE_CODE_BREAKPOINT;
                mstatus[7] <= mstatus[3];
                mstatus[3] <= '0;
                mtval      <= mtval_trap_addr;
            end
            CSR_OP_MRET : begin
                mcause     <= '0;
                mstatus[3] <= mstatus[7];
                mstatus[7] <= '1;
            end
            CSR_OP_INTRPT : begin
                mepc       <= pc_addr;
                mcause     <= format_intrpt_mcause(intrpt_pending);
                mstatus[7] <= mstatus[3];
                mstatus[3] <= '0;
            end
            CSR_OP_TRAP : begin
                mepc       <= pc_addr;
                mstatus[7] <= mstatus[3];
                mstatus[3] <= '0;
                mtval      <= mtval_trap_addr;
                case (trap_cause_sel)
                    TRAP_CAUSE_SEL_INSTRN_ADDR_MISALIGN : mcause <= MCAUSE_CODE_INSTRN_ADDR_MISALIGN;
                    TRAP_CAUSE_SEL_INVLD_INSTRN         : mcause <= MCAUSE_CODE_INVLD_INSTRN;
                    TRAP_CAUSE_SEL_LOAD_ADDR_MISALIGN   : mcause <= MCAUSE_CODE_LOAD_ADDR_MISALIGN;
                    TRAP_CAUSE_SEL_STORE_ADDR_MISALIGN  : mcause <= MCAUSE_CODE_STORE_ADDR_MISALIGN;
                    default                             : ;
                endcase
            end
            CSR_OP_WFI : ; // nop
            default : ;
        endcase
    end

    // external interrupt check
    // if interrupts are globally enabled (mstatus[3] == '1)
    // and mie and mip have the same interrupt bit set, take it
    always @(*) begin
        intrpt_vld     = '0;
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

            intrpt_vld = 1'b1;
        end
    end


    // csr combinational read
    // notifies decoder about illegal reads (addr_vld == 0)
    always @(*) begin
        illegal_write      = '1;
        addr_vld       = '0;
        r_data         = '0;

        case (addr)
            CSR_MVENDORID_ADDR : begin r_data = mvendorid;  addr_vld = '1; end
            CSR_MARCHID_ADDR   : begin r_data = marchid;    addr_vld = '1; end
            CSR_MIMPID_ADDR    : begin r_data = mimpid;     addr_vld = '1; end
            CSR_MHARTID_ADDR   : begin r_data = mhartid;    addr_vld = '1; end
            CSR_MCONFIGPTR_ADDR: begin r_data = mconfigptr; addr_vld = '1; end

            CSR_MSTATUS_ADDR   : begin r_data = mstatus;    illegal_write = '0; addr_vld = '1; end
            CSR_MISA_ADDR      : begin r_data = misa;       illegal_write = '0; addr_vld = '1; end
            CSR_MIE_ADDR       : begin r_data = mie;        illegal_write = '0; addr_vld = '1; end
            CSR_MTVEC_ADDR     : begin r_data = mtvec;      illegal_write = '0; addr_vld = '1; end
            CSR_MSTATUSH_ADDR  : begin r_data = mstatush;   illegal_write = '0; addr_vld = '1; end
            CSR_MSCRATCH_ADDR  : begin r_data = mscratch;   illegal_write = '0; addr_vld = '1; end
            CSR_MEPC_ADDR      : begin r_data = mepc;       illegal_write = '0; addr_vld = '1; end
            CSR_MCAUSE_ADDR    : begin r_data = mcause;     illegal_write = '0; addr_vld = '1; end
            CSR_MTVAL_ADDR     : begin r_data = mtval;      illegal_write = '0; addr_vld = '1; end
            CSR_MIP_ADDR       : begin r_data = mip;        illegal_write = '0; addr_vld = '1; end
            default: begin
                r_data     = '0;
                addr_vld   = '0;
                illegal_write  = '1;
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

    function static [31:0] format_intrpt_mcause (
        input [4:0] pending
    );

    localparam MCAUSE_INTRPT_BIT = 1 << 31;
    format_intrpt_mcause = MCAUSE_INTRPT_BIT | {27'd0, pending};

    endfunction

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
