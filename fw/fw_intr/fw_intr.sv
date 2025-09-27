`timescale 1ns / 1ps

module fw_intr;

    // Parameters
    localparam CLK_PERIOD = 10; // 10 ns clock period -> 100 MHz
    localparam ROM_FILE   = "fw_intr.hex"; // The program to load

    // Signals to connect to the DUT
    reg clk;
    reg rst;
    reg intrpt;
    reg [31:0] iobus_in;

    wire [31:0] iobus_out;
    wire [31:0] iobus_addr;
    wire        iobus_wr;

    // Instantiate the Device Under Test (DUT)
    otter_soc #(
        .ROM_FILE(ROM_FILE)
    ) dut (
        .clk(clk),
        .rst(rst),
        .intrpt(intrpt),
        .iobus_in(iobus_in),
        .iobus_out(iobus_out),
        .iobus_addr(iobus_addr),
        .iobus_wr(iobus_wr)
    );

    // Clock Generator
    initial begin
        clk = 0;
        forever #(CLK_PERIOD / 2) clk = ~clk;
    end

    initial begin
        $display("Starting Interrupt Testbench");

        // 1. Reset the processor
        rst = 1;
        intrpt = 0; // Keep interrupt line low initially
        repeat (5) @(posedge clk);
        rst = 0;
        $display("Reset released at time %0t", $time);

        // 2. Let the processor run to set up interrupts.
        //    Give it 50 cycles to be safe.
        repeat (50) @(posedge clk);
        $display("Firing interrupt at time %0t", $time);

        // 3. Fire the external interrupt for one cycle. ⚡
        intrpt = 1;
        @(posedge clk);
        intrpt = 0;

        // 4. Wait and monitor for the success signal.
        //    We'll wait up to 100 cycles.
        repeat (100) @(posedge clk) begin
            // Check if the CPU is writing to our MMIO "finish" address.
            if (dut.iobus_wr && dut.iobus_addr == 32'h10000) begin
                // Check if the value being written is our success code (1).
                if (dut.iobus_out == 32'h1) begin
                    $display("PASS: Interrupt successfully handled and program finished.");
                    $finish;
                end else begin
                    $error("FAIL: Interrupt received, but signal had invalid value %d.", dut.iobus_out);
                    $finish(1);
                end
            end
        end

        // If we get here, the test timed out.
        $error("FAIL: Test timed out. Success signal not received.");
        $finish(1);
    end

    // Optional: Dump waves for debugging
    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, fw_intr);
    end

endmodule
