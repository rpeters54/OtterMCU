`timescale 1ns / 1ps
`include "otter_defines.vh"

module tb_dcdr;


    reg clk;
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    reg  [31:0] instrn;
    wire [3:0]  alu_func_sel;
    wire        alu_src_a_sel;
    wire        alu_src_b_sel;
    wire [1:0]  rfile_w_sel;
    wire [2:0]  imm_gen_sel;
    wire [1:0]  addr_gen_sel;
    wire [14:0] op_sel;
    wire        ill_instrn;

    wire        rfile_w_en;
    wire        dmem_we;
    wire        dmem_re;
    wire        csr_w_en;

    otter_dcdr u_otter_dcdr (
        .i_instrn(instrn), 

        .o_alu_func_sel(alu_func_sel),
        .o_alu_src_a_sel(alu_src_a_sel),
        .o_alu_src_b_sel(alu_src_b_sel),
        .o_rfile_w_sel(rfile_w_sel),
        .o_imm_gen_sel(imm_gen_sel),
        .o_addr_gen_sel(addr_gen_sel),
        .o_op_sel(op_sel),
        .o_ill_instrn(ill_instrn),

        .o_rfile_we(rfile_w_en),
        .o_dmem_we(dmem_we),
        .o_dmem_re(dmem_re),
        .o_csr_we(csr_w_en)
    );

    initial begin
        // Name as needed
        $dumpfile("tb_dcdr.vcd");
        $dumpvars(0);
    end

    localparam [14:0] X_OP_SEL   = 0;
    localparam [3:0]  X_ALU_FN   = 0;
    localparam        X_ALU_A    = 0;
    localparam        X_ALU_B    = 0;
    localparam [1:0]  X_RF_WSEL  = 0;
    localparam [2:0]  X_IMM_SEL  = 0;
    localparam [1:0]  X_ADDR_SEL = 0;

    initial begin

        $display("--- Starting Decoder Test ---");

        // -------------------------------------------------------------
        // Test: R-Type Instructions
        // -------------------------------------------------------------
        $display("--- Starting R-Type Tests ---");
        
        drive_dut(32'h00628533);
        check_expect(DCDR_OP_SEL_REG, 0, 1, 0, 0, 0, ALU_FUNC_SEL_ADD,  ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_RS2, RFILE_W_SEL_ALU_RESULT, X_IMM_SEL, X_ADDR_SEL, "ADD R-Type");
        
        drive_dut(32'h40628533);
        check_expect(DCDR_OP_SEL_REG, 0, 1, 0, 0, 0, ALU_FUNC_SEL_SUB,  ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_RS2, RFILE_W_SEL_ALU_RESULT, X_IMM_SEL, X_ADDR_SEL, "SUB R-Type");
        
        drive_dut(32'h00629533);
        check_expect(DCDR_OP_SEL_REG, 0, 1, 0, 0, 0, ALU_FUNC_SEL_SLL,  ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_RS2, RFILE_W_SEL_ALU_RESULT, X_IMM_SEL, X_ADDR_SEL, "SLL R-Type");
        
        drive_dut(32'h0062A533);
        check_expect(DCDR_OP_SEL_REG, 0, 1, 0, 0, 0, ALU_FUNC_SEL_SLT,  ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_RS2, RFILE_W_SEL_ALU_RESULT, X_IMM_SEL, X_ADDR_SEL, "SLT R-Type");
        
        drive_dut(32'h0062B533);
        check_expect(DCDR_OP_SEL_REG, 0, 1, 0, 0, 0, ALU_FUNC_SEL_SLTU, ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_RS2, RFILE_W_SEL_ALU_RESULT, X_IMM_SEL, X_ADDR_SEL, "SLTU R-Type");
        
        drive_dut(32'h0062C533);
        check_expect(DCDR_OP_SEL_REG, 0, 1, 0, 0, 0, ALU_FUNC_SEL_XOR,  ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_RS2, RFILE_W_SEL_ALU_RESULT, X_IMM_SEL, X_ADDR_SEL, "XOR R-Type");
        
        drive_dut(32'h0062D533);
        check_expect(DCDR_OP_SEL_REG, 0, 1, 0, 0, 0, ALU_FUNC_SEL_SRL,  ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_RS2, RFILE_W_SEL_ALU_RESULT, X_IMM_SEL, X_ADDR_SEL, "SRL R-Type");
        
        drive_dut(32'h4062D533);
        check_expect(DCDR_OP_SEL_REG, 0, 1, 0, 0, 0, ALU_FUNC_SEL_SRA,  ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_RS2, RFILE_W_SEL_ALU_RESULT, X_IMM_SEL, X_ADDR_SEL, "SRA R-Type");
        
        drive_dut(32'h0062E533);
        check_expect(DCDR_OP_SEL_REG, 0, 1, 0, 0, 0, ALU_FUNC_SEL_OR,   ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_RS2, RFILE_W_SEL_ALU_RESULT, X_IMM_SEL, X_ADDR_SEL, "OR R-Type");
        
        drive_dut(32'h0062F533);
        check_expect(DCDR_OP_SEL_REG, 0, 1, 0, 0, 0, ALU_FUNC_SEL_AND,  ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_RS2, RFILE_W_SEL_ALU_RESULT, X_IMM_SEL, X_ADDR_SEL, "AND R-Type");

        // -------------------------------------------------------------
        // Test: I-Type Instructions
        // -------------------------------------------------------------
        $display("--- Starting I-Type Tests ---");
        
        drive_dut(32'h00528513);
        check_expect(DCDR_OP_SEL_IMM, 0, 1, 0, 0, 0, ALU_FUNC_SEL_ADD,  ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_IMM, RFILE_W_SEL_ALU_RESULT, IMM_GEN_SEL_I_TYPE, X_ADDR_SEL, "ADDI I-Type");
        
        drive_dut(32'h0052A513);
        check_expect(DCDR_OP_SEL_IMM, 0, 1, 0, 0, 0, ALU_FUNC_SEL_SLT,  ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_IMM, RFILE_W_SEL_ALU_RESULT, IMM_GEN_SEL_I_TYPE, X_ADDR_SEL, "SLTI I-Type");
        
        drive_dut(32'h0052B513);
        check_expect(DCDR_OP_SEL_IMM, 0, 1, 0, 0, 0, ALU_FUNC_SEL_SLTU, ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_IMM, RFILE_W_SEL_ALU_RESULT, IMM_GEN_SEL_I_TYPE, X_ADDR_SEL, "SLTIU I-Type");
        
        drive_dut(32'h0052C513);
        check_expect(DCDR_OP_SEL_IMM, 0, 1, 0, 0, 0, ALU_FUNC_SEL_XOR,  ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_IMM, RFILE_W_SEL_ALU_RESULT, IMM_GEN_SEL_I_TYPE, X_ADDR_SEL, "XORI I-Type");
        
        drive_dut(32'h0052E513);
        check_expect(DCDR_OP_SEL_IMM, 0, 1, 0, 0, 0, ALU_FUNC_SEL_OR,   ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_IMM, RFILE_W_SEL_ALU_RESULT, IMM_GEN_SEL_I_TYPE, X_ADDR_SEL, "ORI I-Type");
        
        drive_dut(32'h0052F513);
        check_expect(DCDR_OP_SEL_IMM, 0, 1, 0, 0, 0, ALU_FUNC_SEL_AND,  ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_IMM, RFILE_W_SEL_ALU_RESULT, IMM_GEN_SEL_I_TYPE, X_ADDR_SEL, "ANDI I-Type");
        
        drive_dut(32'h00529513);
        check_expect(DCDR_OP_SEL_IMM, 0, 1, 0, 0, 0, ALU_FUNC_SEL_SLL,  ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_IMM, RFILE_W_SEL_ALU_RESULT, IMM_GEN_SEL_I_TYPE, X_ADDR_SEL, "SLLI I-Type");
        
        drive_dut(32'h0052D513);
        check_expect(DCDR_OP_SEL_IMM, 0, 1, 0, 0, 0, ALU_FUNC_SEL_SRL,  ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_IMM, RFILE_W_SEL_ALU_RESULT, IMM_GEN_SEL_I_TYPE, X_ADDR_SEL, "SRLI I-Type");
        
        drive_dut(32'h4052D513);
        check_expect(DCDR_OP_SEL_IMM, 0, 1, 0, 0, 0, ALU_FUNC_SEL_SRA,  ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_IMM, RFILE_W_SEL_ALU_RESULT, IMM_GEN_SEL_I_TYPE, X_ADDR_SEL, "SRAI I-Type");

        // -------------------------------------------------------------
        // Test: Load Instructions (OPCODE_LOAD)
        // -------------------------------------------------------------
        $display("--- Starting Load Tests ---");
        
        drive_dut(32'h00428503);
        check_expect(DCDR_OP_SEL_LOAD, 0, 0, 1, 0, 0, ALU_FUNC_SEL_ADD, ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_IMM, RFILE_W_SEL_DMEM_R_DATA, IMM_GEN_SEL_I_TYPE, X_ADDR_SEL, "LB Load");
        
        drive_dut(32'h00429503);
        check_expect(DCDR_OP_SEL_LOAD, 0, 0, 1, 0, 0, ALU_FUNC_SEL_ADD, ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_IMM, RFILE_W_SEL_DMEM_R_DATA, IMM_GEN_SEL_I_TYPE, X_ADDR_SEL, "LH Load");
        
        drive_dut(32'h0042A503);
        check_expect(DCDR_OP_SEL_LOAD, 0, 0, 1, 0, 0, ALU_FUNC_SEL_ADD, ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_IMM, RFILE_W_SEL_DMEM_R_DATA, IMM_GEN_SEL_I_TYPE, X_ADDR_SEL, "LW Load");
        
        drive_dut(32'h0042C503);
        check_expect(DCDR_OP_SEL_LOAD, 0, 0, 1, 0, 0, ALU_FUNC_SEL_ADD, ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_IMM, RFILE_W_SEL_DMEM_R_DATA, IMM_GEN_SEL_I_TYPE, X_ADDR_SEL, "LBU Load");
        
        drive_dut(32'h0042D503);
        check_expect(DCDR_OP_SEL_LOAD, 0, 0, 1, 0, 0, ALU_FUNC_SEL_ADD, ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_IMM, RFILE_W_SEL_DMEM_R_DATA, IMM_GEN_SEL_I_TYPE, X_ADDR_SEL, "LHU Load");
          
        // -------------------------------------------------------------
        // Test: Store Instructions (OPCODE_STORE)
        // -------------------------------------------------------------
        $display("--- Starting Store Tests ---");
        
        drive_dut(32'h00A28223);
        check_expect(DCDR_OP_SEL_STORE, 0, 0, 0, 1, 0, ALU_FUNC_SEL_ADD, ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_IMM, X_RF_WSEL, IMM_GEN_SEL_S_TYPE, X_ADDR_SEL, "SB Store");
        
        drive_dut(32'h00A29223);
        check_expect(DCDR_OP_SEL_STORE, 0, 0, 0, 1, 0, ALU_FUNC_SEL_ADD, ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_IMM, X_RF_WSEL, IMM_GEN_SEL_S_TYPE, X_ADDR_SEL, "SH Store");
        
        drive_dut(32'h00A2A223);
        check_expect(DCDR_OP_SEL_STORE, 0, 0, 0, 1, 0, ALU_FUNC_SEL_ADD, ALU_SRC_A_SEL_RS1, ALU_SRC_B_SEL_IMM, X_RF_WSEL, IMM_GEN_SEL_S_TYPE, X_ADDR_SEL, "SW Store");

        // -------------------------------------------------------------
        // Test: Branch Instructions
        // -------------------------------------------------------------
        $display("--- Starting Branch Tests ---");
        
        drive_dut(32'h00A28263);
        check_expect(DCDR_OP_SEL_BRANCH, 0, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, IMM_GEN_SEL_BRANCH, ADDR_GEN_SEL_BRANCH, "BEQ Branch");
        
        drive_dut(32'h00A29263);
        check_expect(DCDR_OP_SEL_BRANCH, 0, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, IMM_GEN_SEL_BRANCH, ADDR_GEN_SEL_BRANCH, "BNE Branch");
        
        drive_dut(32'h00A2C263);
        check_expect(DCDR_OP_SEL_BRANCH, 0, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, IMM_GEN_SEL_BRANCH, ADDR_GEN_SEL_BRANCH, "BLT Branch");
        
        drive_dut(32'h00A2D263);
        check_expect(DCDR_OP_SEL_BRANCH, 0, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, IMM_GEN_SEL_BRANCH, ADDR_GEN_SEL_BRANCH, "BGE Branch");
        
        drive_dut(32'h00A2E263);
        check_expect(DCDR_OP_SEL_BRANCH, 0, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, IMM_GEN_SEL_BRANCH, ADDR_GEN_SEL_BRANCH, "BLTU Branch");
        
        drive_dut(32'h00A2F263);
        check_expect(DCDR_OP_SEL_BRANCH, 0, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, IMM_GEN_SEL_BRANCH, ADDR_GEN_SEL_BRANCH, "BGEU Branch");

        // -------------------------------------------------------------
        // Test: U-Type Instructions
        // -------------------------------------------------------------
        $display("--- Starting U-Type Tests ---");
        
        drive_dut(32'hABCDE537);
        check_expect(DCDR_OP_SEL_LUI, 0, 1, 0, 0, 0, ALU_FUNC_SEL_LUI, X_ALU_A, ALU_SRC_B_SEL_IMM, RFILE_W_SEL_ALU_RESULT, IMM_GEN_SEL_UPPER, X_ADDR_SEL, "LUI U-Type");
        
        drive_dut(32'hABCDE517);
        check_expect(DCDR_OP_SEL_AUIPC, 0, 1, 0, 0, 0, ALU_FUNC_SEL_ADD, ALU_SRC_A_SEL_PC, ALU_SRC_B_SEL_IMM, RFILE_W_SEL_ALU_RESULT, IMM_GEN_SEL_UPPER, X_ADDR_SEL, "AUIPC U-Type");
          
        // -------------------------------------------------------------
        // Test: J-Type Instructions
        // -------------------------------------------------------------
        $display("--- Starting J-Type Tests ---");
        
        drive_dut(32'h0000056F);
        check_expect(DCDR_OP_SEL_JAL, 0, 1, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, RFILE_W_SEL_PC_ADDR_INC, IMM_GEN_SEL_JUMP, ADDR_GEN_SEL_JAL, "JAL J-Type");
        
        drive_dut(32'h00428567);
        check_expect(DCDR_OP_SEL_JALR, 0, 1, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, RFILE_W_SEL_PC_ADDR_INC, IMM_GEN_SEL_I_TYPE, ADDR_GEN_SEL_JALR, "JALR J-Type");

        // -------------------------------------------------------------
        // Test: Fence Instructions
        // -------------------------------------------------------------
        $display("--- Starting Fence Tests ---");
        
        drive_dut(32'h0000000F);
        check_expect(DCDR_OP_SEL_FENCE, 0, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "FENCE");

        // -------------------------------------------------------------
        // Test: System Instructions (OPCODE_SYS)
        // -------------------------------------------------------------
        $display("--- Starting System (CSR/Trap) Tests ---");
        
        // CSR instructions
        drive_dut(32'h30029573);
        check_expect(DCDR_OP_SEL_WRITE, 0, 1, 0, 0, 1, X_ALU_FN, X_ALU_A, X_ALU_B, RFILE_W_SEL_CSR_R_DATA, X_IMM_SEL, X_ADDR_SEL, "CSRRW");
        
        drive_dut(32'h3002A573);
        check_expect(DCDR_OP_SEL_WRITE, 0, 1, 0, 0, 1, X_ALU_FN, X_ALU_A, X_ALU_B, RFILE_W_SEL_CSR_R_DATA, X_IMM_SEL, X_ADDR_SEL, "CSRRS");
        
        drive_dut(32'h3002B573);
        check_expect(DCDR_OP_SEL_WRITE, 0, 1, 0, 0, 1, X_ALU_FN, X_ALU_A, X_ALU_B, RFILE_W_SEL_CSR_R_DATA, X_IMM_SEL, X_ADDR_SEL, "CSRRC");
        
        drive_dut(32'h3002D573);
        check_expect(DCDR_OP_SEL_WRITE, 0, 1, 0, 0, 1, X_ALU_FN, X_ALU_A, X_ALU_B, RFILE_W_SEL_CSR_R_DATA, X_IMM_SEL, X_ADDR_SEL, "CSRRWI");
        
        drive_dut(32'h3002E573);
        check_expect(DCDR_OP_SEL_WRITE, 0, 1, 0, 0, 1, X_ALU_FN, X_ALU_A, X_ALU_B, RFILE_W_SEL_CSR_R_DATA, X_IMM_SEL, X_ADDR_SEL, "CSRRSI");
        
        drive_dut(32'h3002F573);
        check_expect(DCDR_OP_SEL_WRITE, 0, 1, 0, 0, 1, X_ALU_FN, X_ALU_A, X_ALU_B, RFILE_W_SEL_CSR_R_DATA, X_IMM_SEL, X_ADDR_SEL, "CSRRCI");
        
        // CSR Special Cases (Read Only)
        drive_dut(32'h30002573);
        check_expect(DCDR_OP_SEL_WRITE, 0, 1, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, RFILE_W_SEL_CSR_R_DATA, X_IMM_SEL, X_ADDR_SEL, "CSRRS (rs1=0) Read-Only");
        
        drive_dut(32'h30003573);
        check_expect(DCDR_OP_SEL_WRITE, 0, 1, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, RFILE_W_SEL_CSR_R_DATA, X_IMM_SEL, X_ADDR_SEL, "CSRRC (rs1=0) Read-Only");

        // Trap Instructions
        drive_dut(32'h00000073);
        check_expect(DCDR_OP_SEL_ECALL, 0, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "ECALL");
        
        drive_dut(32'h00100073);
        check_expect(DCDR_OP_SEL_EBREAK, 0, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "EBREAK");
        
        drive_dut(32'h30200073);
        check_expect(DCDR_OP_SEL_MRET, 0, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "MRET");
        
        drive_dut(32'h10500073);
        check_expect(DCDR_OP_SEL_WFI, 0, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "WFI");

        // -------------------------------------------------------------
        // Test: Directed ILLEGAL Instruction Tests
        // -------------------------------------------------------------
        $display("--- Starting Illegal Instruction Tests ---");
        
        // Unknown Opcode (all 1s)
        drive_dut(32'hFFFFFFFF);
        check_expect(X_OP_SEL, 1, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "Illegal: All 1s");
        
        // Unknown Opcode (custom-0 region)
        drive_dut(32'h0000000B);
        check_expect(X_OP_SEL, 1, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "Illegal: Custom-0 Opcode");

        // R-Type bad funct
        drive_dut(32'h40007033);
        check_expect(X_OP_SEL, 1, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "Illegal: R-Type bad funct");

        // I-Type bad funct
        drive_dut(32'h02029513);
        check_expect(X_OP_SEL, 1, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "Illegal: SLLI bad funct7");
        
        drive_dut(32'h0202D513);
        check_expect(X_OP_SEL, 1, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "Illegal: SRLI/SRAI bad funct7");

        // Bad funct3 cases
        drive_dut(32'h00029567);
        check_expect(X_OP_SEL, 1, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "Illegal: JALR bad funct3");
        
        drive_dut(32'h0002B503);
        check_expect(X_OP_SEL, 1, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "Illegal: LOAD bad funct3");
        
        drive_dut(32'h0002C223);
        check_expect(X_OP_SEL, 1, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "Illegal: STORE bad funct3");
        
        drive_dut(32'h00022263);
        check_expect(X_OP_SEL, 1, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "Illegal: BRANCH bad funct3");
        
        drive_dut(32'h0000200F);
        check_expect(X_OP_SEL, 1, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "Illegal: FENCE bad funct3");
        
        drive_dut(32'h00004073);
        check_expect(X_OP_SEL, 1, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "Illegal: SYSTEM bad funct3");
          
        // Malformed System
        drive_dut(32'h00000573);
        check_expect(X_OP_SEL, 1, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "Illegal: Malformed ECALL (rd!=0)");
        
        drive_dut(32'h00028073);
        check_expect(X_OP_SEL, 1, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "Illegal: Malformed ECALL (rs1!=0)");
        
        drive_dut(32'hFFFFF073);
        check_expect(X_OP_SEL, 1, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "Illegal: Unknown Trap");

        // CSR access errors
        drive_dut(32'hF1129573);
        check_expect(X_OP_SEL, 1, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "Illegal: Write to Read-Only CSR");
        
        drive_dut(32'hFFF29573);
        check_expect(X_OP_SEL, 1, 0, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, X_RF_WSEL, X_IMM_SEL, X_ADDR_SEL, "Illegal: Access Non-Existent CSR");
        
        // Legal read of read-only CSR
        drive_dut(32'hF1102573);
        check_expect(DCDR_OP_SEL_WRITE, 0, 1, 0, 0, 0, X_ALU_FN, X_ALU_A, X_ALU_B, RFILE_W_SEL_CSR_R_DATA, X_IMM_SEL, X_ADDR_SEL, "CSR Legal Read of Read-Only");

        // -------------------------------------------------------------
        // Test: Fuzz Testing for Safety
        // -------------------------------------------------------------
        $display("--- Starting Fuzz Test ---");
        repeat (10000) begin
            drive_dut($random);
            fuzz_test();
        end

        $display("--- ALL TESTS PASSED ---");
        $finish;
    end

    task drive_dut (
        input [31:0] instrn_test
    );
        /* verilator lint_off INITIALDLY */
        instrn <= instrn_test;
        /* verilator lint_on INITIALDLY */
        @(posedge clk);
    endtask

    task check_expect(
        input [14:0] exp_op_sel,
        input        exp_ill_instrn,
        input        exp_rfile_w_en,
        input        exp_dmem_re,
        input        exp_dmem_we,
        input        exp_csr_w_en,
        input [3:0]  exp_alu_func_sel,
        input        exp_alu_src_a_sel,
        input        exp_alu_src_b_sel,
        input [1:0]  exp_rfile_w_sel,
        input [2:0]  exp_imm_gen_sel,
        input [1:0]  exp_addr_gen_sel,
        input string test_name
    );
        reg failed;
        begin
            failed = 0;
            if (ill_instrn !== exp_ill_instrn) begin
                $display("ERROR: ill_instrn | Expected: %h, Got: %h", exp_ill_instrn, ill_instrn);
                failed = 1;
            end
            if (rfile_w_en !== exp_rfile_w_en) begin
                $display("ERROR: rfile_w_en | Expected: %b, Got: %b", exp_rfile_w_en, rfile_w_en);
                failed = 1;
            end
            if (dmem_re !== exp_dmem_re) begin
                $display("ERROR: dmem_re | Expected: %b, Got: %b", exp_dmem_re, dmem_re);
                failed = 1;
            end
            if (dmem_we !== exp_dmem_we) begin
                $display("ERROR: dmem_we | Expected: %b, Got: %b", exp_dmem_we, dmem_we);
                failed = 1;
            end
            if (csr_w_en !== exp_csr_w_en) begin
                $display("ERROR: csr_w_en | Expected: %b, Got: %b", exp_csr_w_en, csr_w_en);
                failed = 1;
            end

            if (op_sel !== exp_op_sel) begin
                $display("ERROR: op_sel | Expected: %h, Got: %h", exp_op_sel, op_sel);
                failed = 1;
            end
            if (alu_func_sel !== exp_alu_func_sel) begin
                $display("ERROR: alu_func_sel | Expected: %h, Got: %h", exp_alu_func_sel, alu_func_sel);
                failed = 1;
            end
            if (alu_src_a_sel !== exp_alu_src_a_sel) begin
                $display("ERROR: alu_src_a | Expected: %b, Got: %b", exp_alu_src_a_sel, alu_src_a_sel);
                failed = 1;
            end
            if (alu_src_b_sel !== exp_alu_src_b_sel) begin
                $display("ERROR: alu_src_b | Expected: %b, Got: %b", exp_alu_src_b_sel, alu_src_b_sel);
                failed = 1;
            end
            if (rfile_w_sel !== exp_rfile_w_sel) begin
                $display("ERROR: rfile_w_sel | Expected: %h, Got: %h", exp_rfile_w_sel, rfile_w_sel);
                failed = 1;
            end
            if (imm_gen_sel !== exp_imm_gen_sel) begin
                $display("ERROR: imm_gen_sel | Expected: %h, Got: %h", exp_imm_gen_sel, imm_gen_sel);
                failed = 1;
            end
            if (addr_gen_sel !== exp_addr_gen_sel) begin
                $display("ERROR: addr_gen_sel | Expected: %h, Got: %h", exp_addr_gen_sel, addr_gen_sel);
                failed = 1;
            end

            if (failed) begin
                $display("---------------------------------");
                $display("FAILED: %s", test_name);
                $display("Time: %0t", $time);
                $display("INSTRUCTION: 0x%h", instrn);
                $display("---------------------------------");
                $error;
            end
        end
    endtask

    task fuzz_test();
        if (ill_instrn) begin
            // If illegal, NO external writes or bus cycles should happen
            if (rfile_w_en || dmem_we || dmem_re || csr_w_en) begin
                $display("ERROR: Fuzz test failed!");
                $display("Instruction 0x%h decoded as ILLEGAL but enabled external read/write", instrn);
                $error;
            end
        end
    endtask

endmodule
