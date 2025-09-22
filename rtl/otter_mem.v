`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer: J. Callenes, P. Hummel
//
// Create Date: 01/27/2019 08:37:11 AM
// Module Name: OTTER_mem
// Project Name: Memory for OTTER RV32I RISC-V
// Tool Versions: Xilinx Vivado 2019.2
// Description: 64k Memory, dual access read single access write. Designed to
//              purposely utilize BRAM which requires synchronous reads and write
//              ADDR1 used for Program Memory Instruction. Word addressable so it
//              must be adapted from byte addresses in connection from PC
//              ADDR2 used for data access, both internal and external memory
//              mapped IO. ADDR2 is byte addressable.
//              RDEN1 and EDEN2 are read enables for ADDR1 and ADDR2. These are
//              needed due to synchronous reading
//              size used to specify reads as byte (0), half (1), or word (2)
//              sign used to specify unsigned (1) vs signed (0) extension
//              io_in is data from external IO and synchronously buffered
//
// Memory OTTER_MEMORY (
//    .clk   (),
//    .rden1 (),
//    .rden2 (),
//    .we2   (),
//    .addr1 (),
//    .addr2 (),
//    .din2  (),
//    .size  (),
//    .sign  (),
//    .io_in     (),
//    .misalign  (),
//    .io_wr     (),
//    .dout1 (),
//    .dout2 ()  
// );
//
// Revision:
// Revision 0.01 - Original by J. Callenes
// Revision 1.02 - Rewrite to simplify logic by P. Hummel
// Revision 1.03 - changed signal names, added instantiation template
// Revision 1.04 - added defualt to write case statement
// Revision 1.05 - changed MEM_WD to din2, changed default to save nothing
// Revision 1.06 - removed typo in instantiation template
// Revision 1.07 - remove unused wordAddr1 signal
// Revision 1.08 - formatting changes / translation to pure verilog (Riley Peters)
// Revision 1.09 - added mem alignment check and formal verif support (Riley Peters)
//
//////////////////////////////////////////////////////////////////////////////////


module otter_mem #(
    parameter ROM_FILE = "../mem/memory.mem"
) (
    input             clk,
    input             rden1, // read enable Instruction
    input             rden2, // read enable data
    input             we2,   // write enable.
    input      [13:0] addr1, // Instruction Memory word Addr (Connect to PC[15:2])
    input      [31:0] addr2, // Data Memory Addr
    input      [31:0] din2,  // Data to save
    input      [1:0]  size,  // 0-Byte, 1-Half, 2-Word
    input             sign,  // 1-unsigned 0-signed
    input      [31:0] io_in,     // Data from IO

`ifdef RISCV_FORMAL
    output            misalign,  // asserted on misaligned read/write
`endif
    output reg        io_wr,     // IO 1-write 0-read
    output reg [31:0] dout1, // Instruction
    output reg [31:0] dout2  // Data
);

    localparam MEM_VIEW = 1'b0;
    localparam IO_VIEW  = 1'b1;

    localparam UNSIGNED = 1'b1;
    localparam SIGNED   = 1'b0;

    localparam SIZE_BYTE   = 2'd0;
    localparam SIZE_H_WORD = 2'd1;
    localparam SIZE_WORD   = 2'd2;

    //-------------------------//
    // Memory Block Definition
    //-------------------------//  

    localparam [31:0] ADDR_SPACE = 2**14;
    (* rom_style = "{distributed | block}" *)
    (* ram_decomp = "power" *) reg [31:0] memory [0:ADDR_SPACE-1];
    
    initial begin
        $readmemh(ROM_FILE, memory, 0, ADDR_SPACE);
    end

    //-----------------//
    // Local Variables
    //-----------------//  

    reg we_addr_vld;    // active when saving (WE) to valid memory address
    wire [13:0] word_addr_2;
    wire [1:0] byte_offset;
    assign word_addr_2 = addr2[15:2];
    assign byte_offset = addr2[1:0];     // byte offset of memory address

    //------------------------------//
    // Synchronous Read/Write Block
    //------------------------------//  

    always @(posedge clk) begin
        // save data (WD) to memory (ADDR2)
        if (we_addr_vld == 1) begin                  // if write enable && in valid address space
            memory[word_addr_2] <= format_write(size, byte_offset, memory[word_addr_2], din2);
        end

        // read all data synchronously required for BRAM
        // need EN for extra load cycle to not change instruction
        if (rden1) begin
            dout1 <= memory[addr1];
        end

        if (rden2) begin
            case (addr2 >= ADDR_SPACE) 
                MEM_VIEW : dout2 <= format_read(sign, size, byte_offset, memory[word_addr_2]);  // output sized and sign extended data
                IO_VIEW  : dout2 <= io_in;                                                      // IO read from buffer
            endcase
        end
    end

    //------------------//
    // Memory Mapped IO
    //------------------//

    always @(*) begin
        if(addr2 >= ADDR_SPACE) begin  // external address range
            io_wr       = we2;         // IO Write
            we_addr_vld = '0;          // address beyond memory range
        end else begin
            io_wr       = '0;          // not MMIO
            we_addr_vld = we2;         // address in valid memory range
        end
    end

    //---------------------------//
    // Data Formatting Functions
    //---------------------------//

    function [31:0] format_write(
        input [1:0]  size,
        input [1:0]  offset,
        input [31:0] starter,
        input [31:0] updater
    );

        reg [31:0] temp;
        temp = starter;
        case({size, offset})
            {SIZE_BYTE,   2'd0} : temp[7:0]   = updater[7:0]; 
            {SIZE_BYTE,   2'd1} : temp[15:8]  = updater[7:0];
            {SIZE_BYTE,   2'd2} : temp[23:16] = updater[7:0];
            {SIZE_BYTE,   2'd3} : temp[31:24] = updater[7:0];
            {SIZE_H_WORD, 2'd0} : temp[15:0]  = updater[15:0]; 
            {SIZE_H_WORD, 2'd1} : temp[23:8]  = updater[15:0];
            {SIZE_H_WORD, 2'd2} : temp[31:16] = updater[15:0];
            {SIZE_WORD,   2'd0} : temp        = updater;          
            default begin end
        endcase
        format_write = temp;

    endfunction

    function [31:0] format_read(
        input        sign,
        input [1:0]  size,
        input [1:0]  offset,
        input [31:0] starter
    );

        reg [31:0] temp;
        temp = 32'd0;
        case({sign, size, offset})
            {SIGNED,   SIZE_BYTE,   2'd0} : temp = {{24{starter[7]}},starter[7:0]};
            {SIGNED,   SIZE_BYTE,   2'd1} : temp = {{24{starter[15]}},starter[15:8]};
            {SIGNED,   SIZE_BYTE,   2'd2} : temp = {{24{starter[23]}},starter[23:16]};
            {SIGNED,   SIZE_BYTE,   2'd3} : temp = {{24{starter[31]}},starter[31:24]};
            {SIGNED,   SIZE_H_WORD, 2'd0} : temp = {{16{starter[15]}},starter[15:0]};
            {SIGNED,   SIZE_H_WORD, 2'd1} : temp = {{16{starter[23]}},starter[23:8]};
            {SIGNED,   SIZE_H_WORD, 2'd2} : temp = {{16{starter[31]}},starter[31:16]};
            {SIGNED,   SIZE_WORD,   2'd0} : temp = starter;

            {UNSIGNED, SIZE_BYTE,   2'd0} : temp = {24'd0, starter[7:0]};
            {UNSIGNED, SIZE_BYTE,   2'd1} : temp = {24'd0, starter[15:8]};
            {UNSIGNED, SIZE_BYTE,   2'd2} : temp = {24'd0, starter[23:16]};
            {UNSIGNED, SIZE_BYTE,   2'd3} : temp = {24'd0, starter[31:24]};
            {UNSIGNED, SIZE_H_WORD, 2'd0} : temp = {16'd0, starter[15:0]};
            {UNSIGNED, SIZE_H_WORD, 2'd1} : temp = {16'd0, starter[23:8]};
            {UNSIGNED, SIZE_H_WORD, 2'd2} : temp = {16'd0, starter[31:16]};
            default begin end
        endcase
        format_read = temp;

    endfunction

    //------------------------------//
    // Alignment Check (for RVFI)
    //------------------------------//

    // Check for R/W address alignment, used for control unit trap
    `define BYTE(mem_size)   (!mem_size[1] && !mem_size[0])
    `define H_WORD(mem_size) (!mem_size[1] &&  mem_size[0])
    `define WORD(mem_size)   ( mem_size[1] && !mem_size[0])

    assign misalign = !(`BYTE(size) || (`H_WORD(size) && byte_offset != 2'd3) || (`WORD(size) && byte_offset == '0));

    //-------------------------//
    // SBY Formal Verification
    //-------------------------//

`ifdef FORMAL

    // assume reset asserted at start and then stays low and clock starts low
    reg f_past_clk_existed;
    initial f_past_clk_existed = 1'b0;
    always @(posedge clk) f_past_clk_existed <= 1'b1;
    initial assume(clk == 0);

    always @(*) begin
        // verify MMIO logic correctness
        if (addr2 >= ADDR_SPACE) begin
            assert(io_wr == we2);
            assert(we_addr_vld == 1'b0);
        end else begin
            assert(io_wr == 1'b0);
            assert(we_addr_vld == we2);
        end

        // check for misalignment
        assert(misalign == !(
            (size == SIZE_BYTE) || 
            ((size == SIZE_H_WORD) && addr2[1:0] != 2'd3) || 
            ((size == SIZE_WORD) && addr2[1:0] == 2'd0))
        );

        // assume reads and writes are mutually exclusive to save time
        // true for all cases that our cu_fsm relies on
        assume(!((rden1 || rden2) && we2));
    end

    // registers to hold values from the previous cycle for assertions
    reg        f_past_mem_rden1, f_past_mem_rden2, f_past_we_addr_vld;
    reg [13:0] f_past_mem_addr1;
    reg [31:0] f_past_mem_addr2, f_past_mem_din2, f_past_io_in;
    reg [1:0]  f_past_mem_size;
    reg        f_past_mem_sign;

    always @(posedge clk) begin
        f_past_mem_rden1   <= rden1;
        f_past_mem_rden2   <= rden2;
        f_past_we_addr_vld <= we_addr_vld;
        f_past_mem_addr1   <= addr1;
        f_past_mem_addr2   <= addr2;
        f_past_mem_din2    <= din2;
        f_past_mem_size    <= size;
        f_past_mem_sign    <= sign;
        f_past_io_in       <= io_in;
    end

    always @(posedge clk) begin
        if (f_past_clk_existed) begin // Only assert after the first clock edge

            // if a read was enabled on port 1 last cycle, the output now must match the memory content from last cycle
            if (f_past_mem_rden1) begin
                assert(dout1 == memory[f_past_mem_addr1]);
            end

            // If a read was enabled on port 2 last cycle...
            if (f_past_mem_rden2) begin
                if (f_past_mem_addr2 >= ADDR_SPACE) begin
                    //...for an IO address, the output must match the io_in from last cycle.
                    assert(dout2 == f_past_io_in);
                end else begin
                    //...for a memory address, the output must match the correctly formatted data from memory last cycle.
                    assert(dout2 == format_read(f_past_mem_sign, f_past_mem_size, f_past_mem_addr2[1:0], memory[f_past_mem_addr2[15:2]]));
                end
            end

            // If a write was enabled last cycle...
            if (f_past_we_addr_vld) begin
                // ...the corresponding memory location must now contain the correctly formatted new data.
                assert(memory[f_past_mem_addr2[15:2]] == format_write(f_past_mem_size, f_past_mem_addr2[1:0], memory[f_past_mem_addr2[15:2]], f_past_mem_din2));
            end
        end
    end

    (* anyconst *) reg [13:0] f_any_addr;
    reg [31:0] f_past_mem_at_any_addr;

    always @(posedge clk) begin
        f_past_mem_at_any_addr <= memory[f_any_addr];
    end

    always @(posedge clk) begin
        if (f_past_clk_existed) begin
            // If a write occurred last cycle, but to a DIFFERENT address than our arbitrary address...
            if (f_past_we_addr_vld && (f_past_mem_addr2[15:2] != f_any_addr)) begin
                 // ...then the content at our arbitrary address must match its value from the previous cycle.
                assert(memory[f_any_addr] == f_past_mem_at_any_addr);
            end
        end
    end

`endif

endmodule


