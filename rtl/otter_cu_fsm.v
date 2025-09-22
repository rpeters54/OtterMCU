`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/02/2022 03:18:16 PM
// Design Name: 
// Module Name: CU_FSM
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

module otter_cu_fsm (
    input             clk,
    input             rst,
    input             mem_misalign,
    input             intrpt_vld,
    input      [31:0] instrn,
`ifdef RISCV_FORMAL
    output reg        invld_opcode,
`endif
    output reg        pc_w_en, 
    output reg        rfile_w_en, 
    output reg        mem_we2, 
    output reg        mem_rden1, 
    output reg        mem_rden2, 
    output reg        cu_rst, 
    output reg        csr_we, 
    output reg        intrpt_taken
);

    wire [6:0] opcode;
    wire [2:0] func;
    assign opcode = `INSTRN_OPCODE(instrn);
    assign func   = `INSTRN_FUNC(instrn);

    reg invld_opcode, prev_intrpt_vld, intrpt_queued;

    // state variables and initial values
    reg [2:0] present_state, next_state;
    initial begin
        present_state = ST_INIT;
	    next_state    = ST_INIT;
    end

    // state update block
    always @(posedge clk) begin
        if (rst == '1) begin
            present_state   <= ST_INIT;
        end else begin
            present_state   <= next_state;
        end
    end

    // Make interrupt valid signal a one-shot pulse
    always @(posedge clk) begin
        if (rst == '1) begin 
            prev_intrpt_vld <= '0;
            intrpt_queued   <= '0;
        end else begin
            prev_intrpt_vld <= intrpt_vld;
            if (!prev_intrpt_vld && intrpt_vld) begin
                intrpt_queued <= '1;
            end else if (present_state == ST_INTRPT) begin
                intrpt_queued <= '0;
            end
        end
    end
    
    always @(*) begin
        invld_opcode = '0;
        pc_w_en      = '1; 
        rfile_w_en   = '0; 
        mem_we2      = '0; 
        mem_rden1    = '0; 
        mem_rden2    = '0; 
        cu_rst       = '0;
        csr_we       = '0; 
        intrpt_taken = '0;

        case (present_state)
            ST_INIT : begin
                cu_rst     = '1;
                pc_w_en    = '0; 
                next_state = ST_FETCH;
            end
            ST_FETCH : begin
                mem_rden1  = '1;
                pc_w_en    = '0;
                next_state = ST_EXEC;
            end
            ST_EXEC :  begin
                next_state = ST_FETCH;

                // instruction cases
                case(opcode) 
                    OPCODE_OP_REG : begin  //R-Type opcode
                        rfile_w_en = '1;
                    end  
                    OPCODE_OP_IMM : begin  //I-Type opcode *logical instructions
                        rfile_w_en = '1;
                    end
                    OPCODE_JALR : begin  //I-Type opcode *jalr
                        rfile_w_en = '1;
                    end
                    OPCODE_LOAD : begin  //I-Type opcode *load instructions
                        if (!mem_misalign) begin
                            mem_rden2  = '1;
                            pc_w_en    = '0; 
                            next_state = ST_WR_BK;
                        end
                    end
                    OPCODE_STORE : begin  //S-Type opcode *store instructions
                        if (!mem_misalign) begin
                            mem_we2 = '1;
                        end
                    end
                    OPCODE_BRANCH : begin  //B-Type opcode;
                    end
                    OPCODE_LUI : begin  //lui opcode
                        rfile_w_en = '1;
                    end
                    OPCODE_AUIPC : begin  //auipc opcode
                        rfile_w_en = '1;
                    end
                    OPCODE_JAL : begin  //J-Type opcode *jal
                        rfile_w_en = '1;
                    end
                    OPCODE_SYS : begin //intrpt opcode
                        //only true for csrrw
                        if (func[0] == '1) begin
                            csr_we     = '1;
                            rfile_w_en = '1;
                        end
                    end
                    default : begin // unimplemented/undefined opcode, generate trap
                        invld_opcode = '1;
                    end
                endcase 

                // jump to interrupt if queued,
                // defer until next cycle if writing back
                if (intrpt_queued == '1 && next_state != ST_WR_BK) begin
                    next_state = ST_INTRPT;
                end	
            end
            //special case for load instructions
            ST_WR_BK : begin
                //memory reads require an extra clock cycle
                rfile_w_en = '1;
                if (intrpt_queued == '1) begin
                    next_state = ST_INTRPT;
                end else begin
                    next_state = ST_FETCH;
                end
            end
            //interrupt routine
            ST_INTRPT : begin
                //output signal sent to CSR and DCDR
                intrpt_taken  = '1;
                next_state = ST_FETCH;
            end
            default : begin 
                next_state = ST_INIT;
            end
        endcase 
    end


`ifdef FORMAL

    // helper booleans
    wire f_is_load  = (opcode == OPCODE_LOAD && !mem_misalign);
    wire f_is_store = (opcode == OPCODE_STORE && !mem_misalign);
    wire f_is_csrrw = (opcode == OPCODE_SYS && func[0] == 1'b1);

    reg f_past_rst;
    initial f_past_rst = 1'b1;
    always @(posedge clk) f_past_rst <= rst;

    // assume reset set initially, but never asserted afterward
    initial assume(rst);
    always @(*) assume(!(!f_past_rst && rst));

    always @(*) begin
        case (present_state)
            ST_INIT:
                assert(next_state == ST_FETCH);
            ST_FETCH:
                assert(next_state == ST_EXEC);
            ST_EXEC:
                if (f_is_load)
                    assert(next_state == ST_WR_BK);
                else if (intrpt_queued)
                    assert(next_state == ST_INTRPT);
                else
                    assert(next_state == ST_FETCH);
            ST_WR_BK:
                if (intrpt_queued)
                    assert(next_state == ST_INTRPT);
                else
                    assert(next_state == ST_FETCH);
            ST_INTRPT:
                assert(next_state == ST_FETCH);
            default:
                // We should never be in a default state
                assert(
                    present_state == ST_INIT || 
                    present_state == ST_FETCH || 
                    present_state == ST_EXEC || 
                    present_state == ST_WR_BK || 
                    present_state == ST_INTRPT
                );
        endcase

        // Prove that each control signal is asserted ONLY in the correct state(s).
        assert(cu_rst       == (present_state == ST_INIT));
        assert(mem_rden1    == (present_state == ST_FETCH));
        assert(intrpt_taken == (present_state == ST_INTRPT));

        assert(pc_w_en      == !((present_state == ST_INIT || present_state == ST_FETCH) || (present_state == ST_EXEC && f_is_load)));
        assert(mem_rden2    == (present_state == ST_EXEC && f_is_load));
        assert(mem_we2      == (present_state == ST_EXEC && f_is_store));
        assert(csr_we       == (present_state == ST_EXEC && f_is_csrrw));
        assert(rfile_w_en   == (present_state == ST_WR_BK ||
                                (present_state == ST_EXEC &&
                                 (opcode == OPCODE_OP_REG  || opcode == OPCODE_OP_IMM ||
                                  opcode == OPCODE_JALR    || opcode == OPCODE_LUI    ||
                                  opcode == OPCODE_AUIPC   || opcode == OPCODE_JAL)
                                ) ||
                                (present_state == ST_EXEC && f_is_csrrw)
                               ));
    end

    reg f_past_rose_intrpt_vld;
    reg f_past_was_intrpt_state;
    reg f_past_intrpt_queued;
    reg f_past_intrpt_valid;
    reg f_past_prev_intrpt_valid;

    // On every clock edge, sample the current values into the 'past' registers.
    always @(posedge clk) begin
        f_past_rose_intrpt_vld <= (!prev_intrpt_vld && intrpt_vld);
        f_past_was_intrpt_state <= (present_state == ST_INTRPT);
        f_past_intrpt_queued <= intrpt_queued;
        f_past_intrpt_vld <= intrpt_vld;
        f_past_prev_intrpt_valid <= prev_intrpt_vld;
    end

    // On every clock edge, check assertions based on the values from the *previous* cycle.
    always @(posedge clk) begin
        if (!f_past_rst) begin
            // If intrpt_vld rose in the previous cycle, intrpt_queued must now be high.
            if (f_past_rose_intrpt_vld) begin
                assert(intrpt_queued == 1'b1);
            end

            // If the FSM was in the interrupt state and intrpt_vld was stable, intrpt_queued must now be low.
            if (f_past_was_intrpt_state && f_past_prev_intrpt_valid == f_past_intrpt_vld) begin
                assert(intrpt_queued == 1'b0);
            end

            // If neither the set nor clear condition occurred, intrpt_queued must remain stable.
            if (!f_past_rose_intrpt_vld && !f_past_was_intrpt_state) begin
                assert(intrpt_queued == f_past_intrpt_queued);
            end
        end
    end

    // Prove that conflicting signals are never asserted in the same cycle.
    always @(*) begin
        assert(!(mem_rden1 && mem_rden2));
        assert(!(mem_rden1 && mem_we2));
        assert(!(mem_rden2 && mem_we2));
        assert(!(cu_rst && rfile_w_en));
    end

`endif

endmodule

