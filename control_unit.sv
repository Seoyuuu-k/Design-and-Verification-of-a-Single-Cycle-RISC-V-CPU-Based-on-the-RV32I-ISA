`timescale 1ns / 1ps

`define  ADD 4'b0000
`define  SUB 4'b1000
`define  SLL 4'b0001
`define  SRL 4'b0101
`define  SRA 4'b1101
`define  SLT 4'b0010
`define  SLTU 4'b0011
`define  XOR 4'b0100
`define  OR 4'b0110
`define  AND 4'b0111

`define BEQ   3'b000
`define BNE   3'b001
`define BLT   3'b100
`define BGE   3'b101
`define BLTU  3'b110
`define BGEU  3'b111


`define  OP_RTYPE 7'b0110011
`define  OP_STYPE 7'b0100011
`define  OP_ILTYPE 7'b0000011  
`define  OP_ITYPE 7'b0010011
`define  OP_BTYPE 7'b1100011

`define  OP_LUI   7'b0110111
`define  OP_AUIPC 7'b0010111
`define  OP_JAL   7'b1101111
`define  OP_JALR  7'b1100111



module control_unit (
    input  logic [31:0] instr_code,
    output logic [ 3:0] alu_controls,
    output logic        reg_wr_en,
    output logic        alu_soure_mux_sel,
    output logic        is_store, //d_wr_en, // to SLU->RAM
    output logic [1:0]  RegWdataSel, // to write mux
    output logic        is_load, // to SLU
    output logic        branch, //b_type
    output logic        jal, //jal, jalr
    output logic        jalr // jalr
);

    wire [6:0] funct7 = instr_code[31:25];
    wire [2:0] funct3 = instr_code[14:12];
    wire [6:0] opcode = instr_code[6:0];

    logic [7:0] controls;

    //d_wr_en : 2'b00: write X , 2'b01: sw , 2'b10:sh, 2'b11 : sb

    // RegWdataSel 
    // 2'b00 : alu_result , 2'b01:rdata(Itype) , 2'b10: LUI
    // 2'b11 : AUIPC, JAl, JALR


    assign {RegWdataSel,alu_soure_mux_sel,reg_wr_en,is_store,branch,jal,jalr} = controls ; // 2,1,1,1,1,1,1
    assign is_load  = (opcode == `OP_ILTYPE);

    always_comb begin
        case (opcode)
            `OP_RTYPE  : controls = 8'b00_0_1_0_0_0_0;//00_0_1_0_0_00
            `OP_STYPE  : controls = 8'b00_1_0_1_0_0_0;//00_1_0_1_0_00
            `OP_ILTYPE : controls = 8'b01_1_1_0_0_0_0;//01_1_1_0_0_00
            `OP_ITYPE  : controls = 8'b00_1_1_0_0_0_0;//00_1_1_0_0_00
            `OP_BTYPE  : controls = 8'b00_0_0_0_1_0_0;//00_0_0_0_1_00
            `OP_LUI    : controls = 8'b10_0_1_0_0_0_0;//10_X_1_0_0_00 // x는 0으로 채워줌
            `OP_AUIPC  : controls = 8'b11_0_1_0_0_0_0;//11_X_1_0_0_00 
            `OP_JAL    : controls = 8'b11_0_1_0_0_1_0;//11_1_1_0_0_10
            `OP_JALR   : controls = 8'b11_0_1_0_0_1_1;//11_1_1_0_0_11
            default: controls = 8'b0000_0000;
        endcase
    end


    always_comb begin
        case (opcode) 
            //{funct[5],funct3[2:0]} // 이것들만 봐도 판단 가능
            `OP_RTYPE : alu_controls = {funct7[5], funct3}; // R-tupe
            `OP_STYPE : alu_controls = `ADD;
            `OP_ILTYPE: alu_controls = `ADD; 
            `OP_ITYPE : begin
                if(funct3==3'b101)  alu_controls = {funct7[5], funct3};
                else                alu_controls = {1'b0, funct3};
            end
            `OP_BTYPE : alu_controls = {1'b0, funct3} ;
            default   : alu_controls = 4'bx; 
        endcase
    end




endmodule
