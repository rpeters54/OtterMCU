`timescale 1ns / 1ps

module tb_intr;

    parameter MEM_EXP       = 16;
    parameter MEM_SIZE      = 2**MEM_EXP;

    reg clk = 0;
    reg rst = 1;

    always begin
        #5 clk = ~clk;
    end

    initial begin
        repeat (100000) @(posedge clk);
        $error("Timeout");
        $finish(1);
    end

    // create word-addressible behavioral memory
    reg [31:0] prog_mem  [0:MEM_SIZE/4-1];

    wire        dmem_r_en, dmem_w_en;
    wire [3:0]  dmem_w_strb;
    wire [31:0] imem_addr, dmem_addr, dmem_w_data;
    reg  [31:0] imem_r_data, dmem_r_data;

    reg [31:0] intrpt;


    otter_mcu #(
        .RESET_VEC(32'h0000_0000)
    ) mcu (
        .clk(clk),
        .rst(rst),
        .intrpt(intrpt),

        // Instruction Memory Interface
        .imem_r_data(imem_r_data),
        .imem_addr(imem_addr),

        // Data Memory Interface
        .dmem_r_data(dmem_r_data),
        .dmem_r_en(dmem_r_en),
        .dmem_w_en(dmem_w_en),
        .dmem_w_strb(dmem_w_strb),
        .dmem_addr(dmem_addr),
        .dmem_w_data(dmem_w_data)
    );

    // -- Memory Interface Logic --

    always @(posedge clk) begin

        // Synchronous reads from prog_mem, 
        imem_r_data <= prog_mem[imem_addr[MEM_EXP-1:2]];

        // Synchronous reads from prog_mem or debug_mem
        if (dmem_r_en) begin
            dmem_r_data <= prog_mem[dmem_addr[MEM_EXP-1:2]];
        end

        // Synchronous writes to prog_mem or debug_mem
        if (dmem_w_en) begin
            if (dmem_w_strb[0]) prog_mem[dmem_addr[MEM_EXP-1:2]][ 7: 0] <= dmem_w_data[ 7: 0];
            if (dmem_w_strb[1]) prog_mem[dmem_addr[MEM_EXP-1:2]][15: 8] <= dmem_w_data[15: 8];
            if (dmem_w_strb[2]) prog_mem[dmem_addr[MEM_EXP-1:2]][23:16] <= dmem_w_data[23:16];
            if (dmem_w_strb[3]) prog_mem[dmem_addr[MEM_EXP-1:2]][31:24] <= dmem_w_data[31:24];
        end
    end


    initial begin
        // reset the core
        repeat (5) @(posedge clk);
        rst = 0;

        // 

    end

endmodule
