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
// 
//////////////////////////////////////////////////////////////////////////////////

`include "otter_defines.vh"

module otter_csr (
    input             clk,
    input             rst, 

    // CSR instruction related I/O
    input             w_en,
    input      [11:0] addr,
    input      [31:0] w_data,

    output reg [31:1] read_only,
    output reg [31:1] valid,
    output reg [31:0] r_data,

    // CSRs updated during ISR call
    input      [31:0] next_mstatus,
    input      [31:0] next_mcause,
    input      [31:0] next_mie,
    input      [31:0] next_mepc,

    output reg [31:0] mstatus_value,
    output reg [31:0] mcause_value,
    output reg [31:0] mie_value,
    output reg [31:0] mepc_value
);

    // trap related registers, WARL (Accepts any write, only reads legal result)
    reg [31:0] mstatus, misa, mie, mtvec, mstatush, mscratch, mepc, mcause, mtval, mip;

    // unimplemented writeable register, allows dummy writes, always reads zero
    // wire [63:0] mcycle     = '0;
    // wire [63:0] minstret   = '0;

    // read-only registers, rejects write attempts
    wire [31:0] mvendorid  = CSR_MVENDORID_VALUE;
    wire [31:0] marchid    = CSR_MARCHID_VALUE;
    wire [31:0] mimpid     = CSR_MIMPID_VALUE;
    wire [31:0] mhartid    = CSR_MHARTID_VALUE;
    wire [31:0] mconfigptr = CSR_MCONFIGPTR_VALUE;

    always @(posedge clk) begin
        //reset all writable registers to zero
        if (rst) begin
            mstatus  <= '0; 
            misa     <= '0;
            mie      <= '0;
            mtvec    <= '0; 
            mstatush <= '0; 
            mscratch <= '0; 
            mepc     <= '0; 
            mcause   <= '0;
            mtval    <= '0; 
            mip      <= '0;
        end else begin
            // track potential interrupt changes
            mstatus <= next_mstatus & CSR_MSTATUS_MASK;
            mie     <= next_mie     & CSR_MIE_MASK;
            mcause  <= next_mcause  & CSR_MCAUSE_MASK;
            mepc    <= next_mepc    & CSR_MEPC_MASK;

            // writeback csr instruction changes if enabled
            if (w_en) begin
                case (addr) 
                    CSR_MSTATUS_ADDR  : mstatus  <= w_data & CSR_MSTATUS_MASK; 
                    CSR_MISA_ADDR     : misa     <= w_data & CSR_MISA_MASK; 
                    CSR_MIE_ADDR      : mie      <= w_data & CSR_MIE_MASK; 
                    CSR_MTVEC_ADDR    : mtvec    <= w_data & CSR_MTVEC_MASK; 
                    CSR_MSTATUSH_ADDR : mstatush <= w_data & CSR_MSTATUSH_MASK; 
                    CSR_MSCRATCH_ADDR : mscratch <= w_data & CSR_MSCRATCH_MASK; 
                    CSR_MEPC_ADDR     : mepc     <= w_data & CSR_MEPC_MASK; 
                    CSR_MCAUSE_ADDR   : mcause   <= w_data & CSR_MCAUSE_MASK; 
                    CSR_MTVAL_ADDR    : mtval    <= w_data & CSR_MTVAL_MASK; 
                    CSR_MIP_ADDR      : mip      <= w_data & CSR_MIP_MASK; 
                    default           : begin end
                endcase
            end
        end 

        // update csr outputs
        mstatus_value <= mstatus;
        mie_value     <= mie;
        mcause_value  <= mcause;
        mepc_value    <= mepc;
    end

    // get value and useful information about CSR pointed to
    // helps with tracking illegal instructions
    always @(*) begin
        valid = '1;
        case (addr)
            CSR_MVENDORID_ADDR  : begin read_only = '1; r_data = mvendorid;  end
            CSR_MARCHID_ADDR    : begin read_only = '1; r_data = marchid;    end
            CSR_MIMPID_ADDR     : begin read_only = '1; r_data = mimpid;     end
            CSR_MHARTID_ADDR    : begin read_only = '1; r_data = mhartid;    end
            CSR_MCONFIGPTR_ADDR : begin read_only = '1; r_data = mconfigptr; end

            CSR_MSTATUS_ADDR    : begin read_only = '0; r_data = mstatus;    end
            CSR_MISA_ADDR       : begin read_only = '0; r_data = misa;       end
            CSR_MIE_ADDR        : begin read_only = '0; r_data = mie;        end
            CSR_MTVEC_ADDR      : begin read_only = '0; r_data = mtvec;      end
            CSR_MSTATUSH_ADDR   : begin read_only = '0; r_data = mstatush;   end
            CSR_MSCRATCH_ADDR   : begin read_only = '0; r_data = mscratch;   end
            CSR_MEPC_ADDR       : begin read_only = '0; r_data = mepc;       end
            CSR_MCAUSE_ADDR     : begin read_only = '0; r_data = mcause;     end
            CSR_MTVAL_ADDR      : begin read_only = '0; r_data = mtval;      end
            CSR_MIP_ADDR        : begin read_only = '0; r_data = mip;        end

            default             : begin read_only = '1; valid = '0; r_data = 32'hDEAD_BEEF; end
        endcase
    end

endmodule
