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

`include "Defines.svh"

module CU_FSM (
    input             clk,
    input             rst,
    input             pc_misalign,
    input             mem_misalign,
    input             intrpt_vld,
    input      [31:0] instrn,
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
    assign opcode = instrn[6:0];
    assign func   = instrn[14:12];

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
                intrpt_queued <= 1;
            end else if (present_state == ST_INTRPT) begin
                intrpt_queued <= 0;
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
                if (intrpt_queued == '1 && (opcode != OPCODE_LOAD || mem_misalign)) begin
                    next_state = ST_INTRPT;
                end else begin 
                    next_state = ST_FETCH;
                end	
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
                    OPCODE_INTRPT : begin //intrpt opcode
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

        // Verify State Transitions
        always @(*) begin
            case (present_state)
                ST_INIT : begin
                    assert(next_state == ST_FETCH);
                end
                ST_FETCH : begin
                    assert(next_state == ST_EXEC);
                end
                ST_EXEC : begin
                    if (opcode == OPCODE_LOAD && !mem_misalign) begin
                        assert(next_state == ST_WR_BK);
                    end else begin
                        if (intrpt_queued) begin
                            assert(next_state == ST_INTRPT);
                        end else begin
                            assert(next_state == ST_FETCH);
                        end
                    end
                end
                ST_WR_BK : begin
                    if (intrpt_queued) begin
                        assert(next_state == ST_INTRPT);
                    end else begin
                        assert(next_state == ST_FETCH);
                    end
                end
                ST_INTRPT : begin
                    assert(next_state == ST_FETCH);
                end
                default : begin 
                    assert(false);
                end
            endcase

            // Check all variables are set properly
            assert(mem_rden1 ==(present_state == ST_FETCH));
            assert(pc_w_en == !(present_state == ST_INIT || present_state == ST_FETCH || (present_state == ST_EXEC && next_state == ST_WR_BK)));
            assert(mem_rden2 == (present_state == ST_EXEC && next_state == ST_WR_BK));
            assert(mem_we2 == (present_state == ST_EXEC && opcode == OPCODE_STORE && !mem_misalign));
            assert(cu_rst == (present_state == ST_INIT));
            assert(csr_we == (present_state == ST_EXEC && opcode == OPCODE_INTRPT && (func[0] == '1)));
            assert(intrpt_taken == (present_state == ST_INTRPT));
            assert(rfile_w_en == (present_state == ST_WR_BK || (present_state == ST_EXEC && !(
                opcode == OPCODE_LOAD
                || opcode == OPCODE_STORE
                || opcode == OPCODE_BRANCH
                || (opcode == OPCODE_INTRPT && (func[0] == '0))
                || invld_opcode
            ))));
        end

    `endif

endmodule
