`timescale 1ns / 1ps

module fw_intr;

    // ---------- Parameters ----------
    localparam ROM_FILE     = "fw_intr.hex";

    localparam WAIT_CYCLES  = 5000;

    localparam [31:0]  FINISH_ADDR  = 32'h0001_0000;     // *MMIO_FINISH_ADDR
    localparam [31:0]  READY_ADDR   = 32'h0002_0000;     // *INTRPT_READY
    localparam [31:0]  MIE_MASK     = 32'hFFFF_0888;     // bits 16..31 + {11,7,3}

    // SoC memory/boot
    parameter BRAM_BYTES   = 2**16;         // must cover your .hex
    parameter RESET_VEC    = 32'h8000_1000; // match firmware link addr
    parameter I_DLY        = 2;             // IMEM fixed latency cycles
    parameter D_DLY        = 2;             // DMEM fixed latency cycles
    parameter PULSE_CYCLES = 4;

    // ---------- Clock / reset ----------
    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;

    initial begin
        repeat (WAIT_CYCLES) @(posedge clk);
        $error("Timeout");
        $finish(1);
    end

    // ---------- SoC IO bus wires ----------
    // TB drives these into the SoC (slave -> master)
    reg  [31:0] tb_iobus_data  = 32'h0;

    // SoC drives these out (master -> slave)
    wire        w_iobus_re;
    wire        w_iobus_we;
    wire [3:0]  w_iobus_sel;
    wire [31:0] w_iobus_addr;
    wire [31:0] w_iobus_wdata;

    reg  [31:0] intrpt;

    // ---------- DUT: SoC (includes MCU + BRAM memory) ----------
    otter_soc #(
        .ROM_FILE   (ROM_FILE),
        .BRAM_BYTES (BRAM_BYTES),
        .RESET_VEC  (RESET_VEC)
    ) dut (
        .i_clk          (clk),
        .i_rst          (rst),
        .i_intrpt       (intrpt),

        // External IO bus (everything outside BRAM range)
        .i_iobus_data   (tb_iobus_data),
        .o_iobus_re     (w_iobus_re),

        .o_iobus_we     (w_iobus_we),
        .o_iobus_sel    (w_iobus_sel),
        .o_iobus_addr   (w_iobus_addr),
        .o_iobus_data   (w_iobus_wdata)
    );

    // ---------- MMIO responder (single-cycle, TB-owned) ----------
    // - ACKs one cycle after seeing a request to IO_ADDR
    // - Returns zero on reads (writes carry the data from w_iobus_wdata)
    wire iobus_req     = w_iobus_re || w_iobus_we;
    wire iobus_is_mmio = iobus_req && (w_iobus_addr == FINISH_ADDR || w_iobus_addr == READY_ADDR);

    reg mmio_req_q;
    always @(posedge clk) begin
        if (rst) begin
            mmio_req_q      <= 1'b0;
            tb_iobus_data   <= 32'h0000_0000;
        end else begin
            mmio_req_q    <= iobus_is_mmio;
            tb_iobus_data <= 32'h0000_0000;
        end
    end


    task automatic pulse(input integer signal);
        integer k; begin
            $display("[%0t] PULSE intrpt = %d", $time, signal);
            intrpt = signal;
            for (k = 0; k < PULSE_CYCLES; ++k) @(posedge clk);
            intrpt = 0;
        end
    endtask

    // Blocking wait for a write to a given MMIO address; returns the data written
    task automatic wait_for_write(input [31:0] addr, output reg [31:0] data);
        while (!(w_iobus_we && w_iobus_addr == addr)) begin
            @(posedge clk); #1;
        end
        data = w_iobus_wdata;
        $display("[%0t] MMIO write @0x%08x = 0x%08x", $time, addr, data);
    endtask

    // --------------------------------------------------------------------
    // Progress tracking
    // --------------------------------------------------------------------
    reg [31:0] reported_mask;   // bits the FW has acknowledged (writes to FINISH_ADDR)
    reg [31:0] fired_mask;      // bits we have actually pulsed

    integer cycles;
    reg [31:0] ready_bits;
    reg [31:0] finish_bits;

    initial begin
        $display("Starting encoded-ready 32-bit interrupt testbench");
        rst           = 1'b1;
        intrpt        = 32'h0000_0000;
        reported_mask = 32'h0000_0000;
        fired_mask    = 32'h0000_0000;

        // Reset deassert
        repeat (5) @(posedge clk);
        rst = 1'b0;
        $display("[%0t] Reset released", $time);

        // Loop until FW has acknowledged all bits in MIE_MASK
        while ( (reported_mask & MIE_MASK) != MIE_MASK ) begin
            wait_for_write(READY_ADDR, ready_bits);

            ready_bits = ready_bits & MIE_MASK;
            if (ready_bits == 0) begin
                $display("[%0t] NOTE: Ready value outside MIE_MASK ignored.", $time);
            end else begin
                pulse(ready_bits);
                fired_mask = fired_mask | ready_bits;
            end


            wait_for_write(FINISH_ADDR, finish_bits);
            reported_mask = reported_mask | (finish_bits & MIE_MASK);

            $display("[%0t] Progress: reported=0x%08x / expected=0x%08x",
                     $time, reported_mask & MIE_MASK, MIE_MASK);
        end

        $display("PASS: Firmware acknowledged all interrupts in MIE_MASK.");
        $finish;
    end


    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, fw_intr);
    end

endmodule
