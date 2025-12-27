`timescale 1ns / 1ps


// ------------------------------
// Top
// ------------------------------
module RV32I_TOP (
    input  logic        clk,
    input  logic        reset
);
    // IF/Memory 인터커넥트
    logic [31:0] instr_code, instr_rAddr;
    logic [31:0] dAddr, dWdata, dRdata;
    logic [3:0]  wstrb;

    // Instruction ROM
    instr_mem U_Instr_Mem (
        .instr_rAddr (instr_rAddr),
        .instr_code  (instr_code)
    );

    // Core
    RV32I_Core U_RV32I_core (
        .clk         (clk),
        .reset       (reset),
        .instr_code  (instr_code),
        .dRdata      (dRdata),
        .instr_rAddr (instr_rAddr),
        .wstrb       (wstrb),
        .dAddr       (dAddr),
        .dWdata      (dWdata)
    );

    // Data RAM  (SW=01, SH=10, SB=11)
    data_ram U_DATA_RAM (
        .clk    (clk),
        .wstrb  (wstrb),
        .dAddr  (dAddr),
        .dWdata (dWdata),
        .dRdata (dRdata)
    );
endmodule


// ------------------------------
// Core
// ------------------------------
module RV32I_Core (
    input  logic        clk,
    input  logic        reset,

    // from instr_mem / data_ram
    input  logic [31:0] instr_code,
    input  logic [31:0] dRdata,

    // to instr_mem / data_ram
    output logic [31:0] instr_rAddr,
    output logic [3:0]  wstrb,
    output logic [31:0] dAddr,
    output logic [31:0] dWdata
);
    // control → datapath 신호
    logic [3:0] alu_controls;
    logic       reg_wr_en;
    logic       alu_soure_mux_sel;
    logic [1:0] RegWdataSel;      // 00:ALU 01:Load 10:LUI 11:AUIPC/JAL
    logic       is_load;          // OP_ILTYPE
    logic       is_store;         // OP_STYPE
    logic       branch;
    logic       jal; //jal, jalr
    logic       jalr; // jalr

    // Control Unit
    control_unit U_Control_Unit (
        .instr_code         (instr_code),
        .alu_controls       (alu_controls),
        .reg_wr_en          (reg_wr_en),
        .alu_soure_mux_sel  (alu_soure_mux_sel),
        .is_store           (is_store),
        .RegWdataSel        (RegWdataSel),
        .is_load            (is_load),
        .branch             (branch),
        .jal                (jal), //jal, jalr
        .jalr               (jalr) // jalr
    );

    // Datapath
    datapath U_Data_Path (
        .clk                (clk),
        .reset              (reset),
        .instr_code         (instr_code),
        .alu_controls       (alu_controls),
        .reg_wr_en          (reg_wr_en),
        .is_load            (is_load),
        .is_store           (is_store),
        .jal                (jal), //jal, jalr
        .jalr               (jalr), // jalr
        .alu_soure_mux_sel  (alu_soure_mux_sel),
        .RegWdataSel        (RegWdataSel),
        .dRdata             (dRdata),
        .branch             (branch),
        .wstrb              (wstrb),
        .instr_rAddr        (instr_rAddr),
        .dAddr              (dAddr),
        .dWdata             (dWdata)
    );
endmodule
