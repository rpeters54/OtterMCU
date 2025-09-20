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
//              MEM_SIZE used to specify reads as byte (0), half (1), or word (2)
//              MEM_SIGN used to specify unsigned (1) vs signed (0) extension
//              IO_IN is data from external IO and synchronously buffered
//
// Memory OTTER_MEMORY (
//    .MEM_CLK   (),
//    .MEM_RDEN1 (),
//    .MEM_RDEN2 (),
//    .MEM_WE2   (),
//    .MEM_ADDR1 (),
//    .MEM_ADDR2 (),
//    .MEM_DIN2  (),
//    .MEM_SIZE  (),
//    .MEM_SIGN  (),
//    .IO_IN     (),
//    .MISALIGN  (),
//    .IO_WR     (),
//    .MEM_DOUT1 (),
//    .MEM_DOUT2 ()  
// );
//
// Revision:
// Revision 0.01 - Original by J. Callenes
// Revision 1.02 - Rewrite to simplify logic by P. Hummel
// Revision 1.03 - changed signal names, added instantiation template
// Revision 1.04 - added defualt to write case statement
// Revision 1.05 - changed MEM_WD to MEM_DIN2, changed default to save nothing
// Revision 1.06 - removed typo in instantiation template
// Revision 1.07 - remove unused wordAddr1 signal
// Revision 1.08 - formatting changes / translation to pure verilog (Riley Peters)
// Revision 1.09 - added mem alignment check and formal verif support (Riley Peters)
//
//////////////////////////////////////////////////////////////////////////////////


module otter_mem #(
    parameter ROM_FILE = "../mem/memory.mem"
) (
    input             MEM_CLK,
    input             MEM_RDEN1, // read enable Instruction
    input             MEM_RDEN2, // read enable data
    input             MEM_WE2,   // write enable.
    input      [13:0] MEM_ADDR1, // Instruction Memory word Addr (Connect to PC[15:2])
    input      [31:0] MEM_ADDR2, // Data Memory Addr
    input      [31:0] MEM_DIN2,  // Data to save
    input      [1:0]  MEM_SIZE,  // 0-Byte, 1-Half, 2-Word
    input             MEM_SIGN,  // 1-unsigned 0-signed
    input      [31:0] IO_IN,     // Data from IO

`ifdef RISCV_FORMAL
    output            MISALIGN,  // asserted on misaligned read/write
`endif
    output reg        IO_WR,     // IO 1-write 0-read
    output reg [31:0] MEM_DOUT1, // Instruction
    output reg [31:0] MEM_DOUT2  // Data
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
    assign word_addr_2 = MEM_ADDR2[15:2];
    assign byte_offset = MEM_ADDR2[1:0];     // byte offset of memory address

    //------------------------------//
    // Synchronous Read/Write Block
    //------------------------------//  

    always @(posedge MEM_CLK) begin
        // save data (WD) to memory (ADDR2)
        if (we_addr_vld == 1) begin                  // if write enable && in valid address space
            memory[word_addr_2] <= format_write(MEM_SIZE, byte_offset, memory[word_addr_2], MEM_DIN2);
        end

        // read all data synchronously required for BRAM
        // need EN for extra load cycle to not change instruction
        if (MEM_RDEN1) begin                      
            MEM_DOUT1 <= memory[MEM_ADDR1];
        end

        if (MEM_RDEN2) begin                      
            case (MEM_ADDR2 >= ADDR_SPACE) 
                MEM_VIEW : MEM_DOUT2 <= format_read(MEM_SIGN, MEM_SIZE, byte_offset, memory[word_addr_2]);  // output sized and sign extended data
                IO_VIEW  : MEM_DOUT2 <= IO_IN;                                                              // IO read from buffer
            endcase
        end
    end

    //------------------//
    // Memory Mapped IO
    //------------------//

    always @(*) begin
        if(MEM_ADDR2 >= ADDR_SPACE) begin  // external address range
            IO_WR       = MEM_WE2;         // IO Write
            we_addr_vld = '0;              // address beyond memory range
        end else begin
            IO_WR       = '0;              // not MMIO
            we_addr_vld = MEM_WE2;         // address in valid memory range
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
        input [31:0] starter,
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

    assign MISALIGN = !(`BYTE(MEM_SIZE) || (`H_WORD(MEM_SIZE) && byte_offset != 2'd3) || (`WORD(MEM_SIZE) && byte_offset == '0));

    //-------------------------//
    // SBY Formal Verification
    //-------------------------//

    `ifdef FORMAL
        (* anyconst *)	wire [13:0] w_addr, r_addr_1;
        (* anyconst *)	wire [31:0] r_addr_2;
                        reg [31:0]	w_data, r_data_1, r_data_2;
                        reg        r_valid_1, r_valid_2;

        // Track previous cycle values for read-after-write checks
        reg [31:0] prev_mem_addr2, prev_r_addr_2;
        reg [31:0] prev_mem_addr1, prev_r_addr_1;
        reg        prev_mem_rden1, prev_mem_rden2;

        always @(posedge MEM_CLK) begin
            prev_mem_addr2  <= MEM_ADDR2;
            prev_r_addr_2   <= r_addr_2;
            prev_mem_addr1  <= MEM_ADDR1;
            prev_r_addr_1   <= r_addr_1;
            prev_mem_rden1  <= MEM_RDEN1;
            prev_mem_rden2  <= MEM_RDEN2;
        end

        // Verify Writes
        initial	assume(w_data == memory[w_addr]);
        always @(posedge MEM_CLK) begin
            if (MEM_WE2 && (MEM_ADDR2 < ADDR_SPACE) && (w_addr == word_addr_2)) begin
                w_data <= format_write(MEM_SIZE, byte_offset, w_data, MEM_DIN2);
            end
        end
        always @(*) begin
            assert(memory[w_addr] == w_data);
        end

        // Verify Reads
        initial begin
            r_valid_1 = '0; r_valid_2 = '0;
            r_data_1  = '0; r_data_2  = '0;
        end
        always @(posedge MEM_CLK) begin
            if (MEM_RDEN1 && (r_addr_1 == MEM_ADDR1)) begin
                r_valid_1 <= '1;
                r_data_1  <= memory[r_addr_1];
            end
            if (MEM_RDEN2 && (r_addr_2 == MEM_ADDR2)) begin
                r_valid_2 <= '1;
                if (MEM_ADDR2 >= ADDR_SPACE) begin
                    r_data_2 <= IO_IN;
                end else begin
                    r_data_2 <= format_read(MEM_SIGN, MEM_SIZE, byte_offset, memory[r_addr_2[15:2]]);
                end
            end
        end
        always @(*) begin
            if (r_valid_1 && prev_mem_rden1 && (prev_r_addr_1 == prev_mem_addr1)) begin
                assert(MEM_DOUT1 == r_data_1);
            end
            if (r_valid_2 && prev_mem_rden2 && (prev_r_addr_2 == prev_mem_addr2)) begin
                assert(MEM_DOUT2 == r_data_2);
            end
        end


        // Check address constraints
        always @(*) begin
            assert(MEM_ADDR1 < ADDR_SPACE);
        end

        // Check MMIO behavior
        always @(*) begin
            if (MEM_ADDR2 >= ADDR_SPACE) begin
                assert(IO_WR == MEM_WE2);
                assert(we_addr_vld == 1'b0);
            end else begin
                assert(IO_WR == 1'b0);
                assert(we_addr_vld == MEM_WE2);
            end
        end

        // Check misalignment detection
        always @(*) begin
            case(MEM_SIZE)
                SIZE_BYTE   : assert(MISALIGN == 1'b0);                     // Bytes are always aligned
                SIZE_H_WORD : assert(MISALIGN == (MEM_ADDR2[1:0] == 2'd3)); // Half-word misaligned on byte 3
                SIZE_WORD   : assert(MISALIGN == (MEM_ADDR2[1:0] != 2'd0)); // Word misaligned if not on word boundary
                default     : assert(MISALIGN == 1'b1);                     // Invalid size
            endcase
        end

        // Cover Properties
        always @(posedge MEM_CLK) begin
            cover(MEM_WE2 && MEM_ADDR2 < ADDR_SPACE);   // Cover memory writes
            cover(MEM_RDEN2 && MEM_ADDR2 < ADDR_SPACE); // Cover memory reads
            cover(MEM_ADDR2 >= ADDR_SPACE);             // Cover MMIO access
            cover(MISALIGN);                            // Cover misaligned access
        end

    `endif

 endmodule
