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
//    .IO_WR     (),
//    .MEM_DOUT1 (),
//    .MEM_DOUT2 ()  );
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
//
//////////////////////////////////////////////////////////////////////////////////

                                                                                                                             
module Memory #(
    parameter ROM_FILE = "default.mem"
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
    //output ERR,                // only used for testing
    output reg        IO_WR,     // IO 1-write 0-read
    output reg [31:0] MEM_DOUT1, // Instruction
    output reg [31:0] MEM_DOUT2  // Data
);
    


    wire [13:0] word_addr_2;
    wire [1:0] byte_offset;

    reg [31:0] mem_read_word, io_buffer, mem_read_sized;
    reg we_addr_vld;      // active when saving (WE) to valid memory address
       
    (* rom_style = "{distributed | block}" *)
    (* ram_decomp = "power" *) reg [31:0] memory [0:16383];
    
    initial begin
        $readmemh(ROM_FILE, memory, 0, 16383);
    end
    
    assign word_addr_2 = MEM_ADDR2[15:2];
    assign byte_offset = MEM_ADDR2[1:0];     // byte offset of memory address
         
    // NOT USED IN OTTER
    //Check for misalligned or out of bounds memory accesses
    //assign ERR = ((MEM_ADDR1 >= 2**ACTUAL_WIDTH)|| (MEM_ADDR2 >= 2**ACTUAL_WIDTH)
    //                || MEM_ADDR1[1:0] != 2'b0 || MEM_ADDR2[1:0] !=2'b0)? 1 : 0;
            
    // buffer the IO input for reading
    always @(posedge MEM_CLK) begin
        if(MEM_RDEN2) begin
            io_buffer <= IO_IN;
        end
    end
    
    // BRAM requires all reads and writes to occur synchronously
    always @(posedge MEM_CLK) begin
        // save data (WD) to memory (ADDR2)
        if (we_addr_vld == 1) begin     // write enable and valid address space
            case({MEM_SIZE,byte_offset})
                4'b0000: memory[word_addr_2][7:0]   <= MEM_DIN2[7:0];     // sb at byte offsets
                4'b0001: memory[word_addr_2][15:8]  <= MEM_DIN2[7:0];
                4'b0010: memory[word_addr_2][23:16] <= MEM_DIN2[7:0];
                4'b0011: memory[word_addr_2][31:24] <= MEM_DIN2[7:0];
                4'b0100: memory[word_addr_2][15:0]  <= MEM_DIN2[15:0];    // sh at byte offsets
                4'b0101: memory[word_addr_2][23:8]  <= MEM_DIN2[15:0];
                4'b0110: memory[word_addr_2][31:16] <= MEM_DIN2[15:0];
                4'b1000: memory[word_addr_2]        <= MEM_DIN2;          // sw
                default : begin end                                     // unsupported size, nop
            endcase
        end

        // read all data synchronously required for BRAM
        if (MEM_RDEN1) begin                      // need EN for extra load cycle to not change instruction
            MEM_DOUT1 <= memory[MEM_ADDR1];
        end

        if (MEM_RDEN2) begin                      // Read word from memory
            mem_read_word <= memory[word_addr_2];
        end
    end
       
    // Change the data word into sized bytes and sign extend
    always @(*) begin
        case({MEM_SIGN,MEM_SIZE,byte_offset})
            5'b00011: mem_read_sized = {{24{mem_read_word[31]}},mem_read_word[31:24]};  // signed byte
            5'b00010: mem_read_sized = {{24{mem_read_word[23]}},mem_read_word[23:16]};
            5'b00001: mem_read_sized = {{24{mem_read_word[15]}},mem_read_word[15:8]};
            5'b00000: mem_read_sized = {{24{mem_read_word[7]}},mem_read_word[7:0]};
                                        
            5'b00110: mem_read_sized = {{16{mem_read_word[31]}},mem_read_word[31:16]};  // signed half
            5'b00101: mem_read_sized = {{16{mem_read_word[23]}},mem_read_word[23:8]};
            5'b00100: mem_read_sized = {{16{mem_read_word[15]}},mem_read_word[15:0]};
                
            5'b01000: mem_read_sized = mem_read_word;                   // word
                  
            5'b10011: mem_read_sized = {24'd0,mem_read_word[31:24]};    // unsigned byte
            5'b10010: mem_read_sized = {24'd0,mem_read_word[23:16]};
            5'b10001: mem_read_sized = {24'd0,mem_read_word[15:8]};
            5'b10000: mem_read_sized = {24'd0,mem_read_word[7:0]};
                  
            5'b10110: mem_read_sized = {16'd0,mem_read_word[31:16]};    // unsigned half
            5'b10101: mem_read_sized = {16'd0,mem_read_word[23:8]};
            5'b10100: mem_read_sized = {16'd0,mem_read_word[15:0]};
                
            default:  mem_read_sized = 32'b0;     // unsupported size, byte offset combination
        endcase
    end
 
    // Memory Mapped IO
    always @(*) begin
        if(MEM_ADDR2 >= 32'h00010000) begin  // external address range
            IO_WR = MEM_WE2;                 // IO Write
            MEM_DOUT2 = io_buffer;            // IO read from buffer
            we_addr_vld = 0;                 // address beyond memory range
        end else begin
            IO_WR = 0;                  // not MMIO
            MEM_DOUT2 = mem_read_sized;   // output sized and sign extended data
            we_addr_vld = MEM_WE2;      // address in valid memory range
        end
    end
        
 endmodule
