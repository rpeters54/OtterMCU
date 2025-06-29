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

`include "OPCODES.vh"

module CU_FSM (
    input              clk,
    input              rst,
    input              intrpt_vld,
    input      [6:0]   opcode,
    input      [14:12] func,
    output reg         pc_w_en, 
    output reg         rfile_w_en, 
    output reg         mem_we2, 
    output reg         mem_rden1, 
    output reg         mem_rden2, 
    output reg         cu_rst, 
    output reg         csr_we, 
    output reg         int_taken
);

    // states
    localparam ST_INIT   = 3'd0;
    localparam ST_FETCH  = 3'd1;
    localparam ST_EXEC   = 3'd2;
    localparam ST_WR_BK  = 3'd3;
    localparam ST_INTRPT = 3'd4;
    
    // state variables and initial values
    reg [2:0] present_state, next_state;
    initial begin
        present_state <= ST_INIT;
	next_state    <= ST_INIT;
    end

    // state update block
    always @(posedge clk) begin
	if (rst == '1) begin 
            present_state <= ST_INIT;
	begin else end
            present_state <= next_state;
	end
    end
    
    always @(*) begin
        pc_w_en   = '0; 
        rfile_w_en  = '0; 
        mem_we2    = '0; 
        mem_rden1  = '0; 
        mem_rden2  = '0; 
        cu_rst     = '0;
        csr_we    = '0; 
        int_taken = '0;
        case (present_state)
            ST_INIT : begin
                cu_rst = '1;
                next_state = ST_FETCH;
            end
            ST_FETCH : begin
                mem_rden1 = '1;
                next_state = ST_EXEC;
            end
            ST_EXEC :  begin
                if (intrpt_vld == '1) begin
                    next_state = ST_INTRPT;
                end else begin 
                    next_state = ST_FETCH;
                end	
		        case(opcode) 
                    OPCODE_R_TYPE : begin  //R-Type opcode
                        rfile_w_en = '1;
                        pc_w_en  = '1;
                    end
                    OPCODE_I_TYPE_NO_LOAD : begin  //I-Type opcode *logical instructions
                        rfile_w_en = '1;
                        pc_w_en  = '1;
                    end
                    OPCODE_I_TYPE_JALR : begin  //I-Type opcode *jalr
                        rfile_w_en = '1;
                        pc_w_en  = '1;
                    end
                    OPCODE_I_TYPE_LOAD : begin  //I-Type opcode *load instructions
                        mem_rden2   = '1;
                        next_state = ST_WR_BK;
                    end
                    OPCODE_S_TYPE : begin  //S-Type opcode *store instructions
                        mem_we2  = '1;
                        pc_w_en = '1;
                    end
                    OPCODE_B_TYPE : begin  //B-Type opcode;
                        pc_w_en = '1;
                    end
                    OPCODE_LUI : begin  //lui opcode
                        rfile_w_en = '1;
                        pc_w_en  = '1;
                    end
                    OPCODE_AUIPC : begin  //auipc opcode
                        rfile_w_en = '1;
                        pc_w_en  = '1;
                    end
                    OPCODE_J_TYPE_JAL : begin  //J-Type opcode *jal
                        rfile_w_en = '1;
                        pc_w_en  = '1;
                    end
                    OPCODE_INTRPT : begin //intrpt opcode
		                //true for mret and csrrw
                        pc_w_en = '1;             
                        //only true for csrrw
                        if (func[12] == '1) begin
                            csr_we   = '1;
                            rfile_w_en = '1;
                        end
                    end
                endcase 
            end
	        //special case for load instructions
            ST_WR_BK : begin        
                //memory reads require an extra clock cycle
	            rfile_w_en = '1;
                pc_w_en  = '1;
	            if (intrpt_vld == '1) begin
                    next_state = ST_INTRPT;
                begin else end
                    next_state = ST_FETCH;
                end
            end
	        //interrupt routine
            ST_INTRPT : begin
		        //output signal sent to CSR and DCDR
                int_taken = '1;
                pc_w_en  = '1;
                next_state = ST_FETCH;
            end
	        default : begin 
	            next_state = ST_INIT;
            end
        endcase 
    end
endmodule
