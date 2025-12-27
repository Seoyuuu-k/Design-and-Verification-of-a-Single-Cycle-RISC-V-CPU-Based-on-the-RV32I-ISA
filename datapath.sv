`timescale 1ns / 1ps

// `include "define.sv"

`define  ADD   4'b0000
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


module datapath (
    input  logic        clk,
    input  logic        reset,
    input  logic [31:0] instr_code,
    input  logic [ 3:0] alu_controls,
    input  logic        reg_wr_en,
    input  logic        is_load,
    input  logic        is_store,
    input  logic        alu_soure_mux_sel,
    input  logic [1:0]  RegWdataSel,
    input  logic [31:0] dRdata,//from Ram
    input  logic        branch, // b_type
    input  logic        jal, //jal, jalr
    input  logic        jalr, // jalr
    output logic [3:0]  wstrb,
    output logic [31:0] instr_rAddr,
    output logic [31:0] dAddr,
    output logic [31:0] dWdata
);

    logic [31:0] w_regfile_rd1, w_regfile_rd2, w_alu_result, w_regfile_rd2_ALU, w_pc_reg_d_mux2;
    logic [31:0] imm_txt, w_reg_Wdata , w_drdata_extened, w_pc_next, w_pc_Muxout, w_auipc_jal_extend,w_wdata_adder_mux;
    logic pc_MuxSel , btaken ;
    logic w_reg_wr_en, load_misaligned;   

    assign pc_MuxSel = jal || (branch & btaken); // taken:internal, branch:external
    assign w_reg_wr_en = reg_wr_en & (!load_misaligned);//load_misaligned(== misaligned & load) 

    register_file U_REG_FILE (
        .clk  (clk),
        .RA1  (instr_code[19:15]),  // read address 1
        .RA2  (instr_code[24:20]),  // read address 2
        .WA   (instr_code[11:7]),   // write address
        .we   (w_reg_wr_en),                // write enable
        .WData(w_reg_Wdata),         // write data
        .RD1  (w_regfile_rd1),        // read data 1
        .RD2  (w_regfile_rd2)         // read data 2
    );

    ALU U_ALU (
        .a(w_regfile_rd1),
        .b(w_regfile_rd2_ALU),
        .alu_controls(alu_controls),
        .alu_result(w_alu_result),
        .btaken(btaken)
    );

    extend U_EXTEND (
        .instr_code(instr_code),
        .imm_txt(imm_txt)
    );

    mux_2X1 U_MUX_ALU_SRC_MUX(
        .sel(alu_soure_mux_sel),
        .x0(w_regfile_rd2), // 0 : regfile R2
        .x1(imm_txt), // 1 : imm[31:0]
        .y(w_regfile_rd2_ALU) 
    );

//4X1 mux

    mux_4X1 U_MUX_RegWdataMux(
        .sel(RegWdataSel),
        .x0(w_alu_result), // 00 : alu
        .x1(w_drdata_extened), // 01 : rdata (ILtype)
        .x2(imm_txt), // 10 : LUI // from extend direct
        .x3(w_auipc_jal_extend), // 11 : AUIPC , JAL, JALR
        .y(w_reg_Wdata) 
    );

    adder U_AUIPC_JAL_ADDER(
        .a(w_wdata_adder_mux), // from mux
        .b(instr_rAddr), // pc
        .result(w_auipc_jal_extend)
    );

    mux_2X1 U_MUX_Wdata_Adder(
        .sel(jal),
        .x0(imm_txt), // 0 : extend
        .x1(32'd4), // 1 : 4
        .y(w_wdata_adder_mux) // to regWdata_mux_adder
    );

//------------------------------------
    load_Store_Unit U_LSU(
        .is_load(is_load),      // opcode == OP_ILTYPE
        .is_store(is_store),     // opcode == OP_STYPE
        .funct3(instr_code[14:12]),       // size/unsigned 정보
        // 이미 올라온 주소/쓰기 데이터
        .w_dAddr(w_alu_result),      // byte address
        .w_dWdata(w_regfile_rd2),     // store write data (word)

        // to Data RAM
        .dAddr(dAddr),        // byte address (그대로 패스)
        .dWdata(dWdata),       // store write data (SB/SH는 하위만 유효)
        .wstrb(wstrb),      // 00:none 01:SW 10:SH 11:SB  (네 RAM 약속)

        //from ram
        .dRdata(dRdata),       // word-aligned read data

        // to WriteBack
        .load_ext(w_drdata_extened),     // 확장된 로드 데이터 (WB로)
        .trap(),        // misaligned trap
        .load_misaligned(load_misaligned) // AND gate용
    );


    // ----pc----------------//

    program_counter U_PC (
        .clk  (clk),
        .reset(reset),
        .d    (w_pc_next),
        .q   (instr_rAddr)
    );
    mux_2X1 U_MUX_PCreg_RS1_PC(
        .sel(jalr),
        .x0(instr_rAddr), // 0 : pc
        .x1(w_regfile_rd1), // 1 : rs1
        .y(w_pc_reg_d_mux2) // to pc_adder
    );

    adder U_PC_ADDER(
        .a(w_pc_reg_d_mux2),
        .b(w_pc_Muxout),
        .result(w_pc_next)
    );

    mux_2X1 U_PC_MUX(
        .sel(pc_MuxSel), // from control unit
        .x0(32'd4), // 0 : 4
        .x1(imm_txt), // 1 : imm_ext
        .y(w_pc_Muxout)   // to pc_adder
    );

endmodule


module adder (
    input logic [31:0] a,
    input logic [31:0] b,
    output logic [31:0] result
);
    assign result = a+b;
    
endmodule




module program_counter (
    input  logic        clk,
    input  logic        reset,
    input  logic [31:0] d,
    output logic [31:0] q
);
   
    register U_PC_REG (
        .clk(clk),
        .reset(reset),
        .d(d),
        .q(q)
    );
endmodule


module register (
    input  logic        clk,
    input  logic        reset,
    input  logic [31:0] d,
    output logic [31:0] q
);

    always_ff @(posedge clk, posedge reset) begin
        if (reset) begin
            q <= 0;
        end else begin
            q <= d;
        end
    end

endmodule

module register_file (
    input  logic        clk,
    input  logic [ 4:0] RA1,    // read address 1
    input  logic [ 4:0] RA2,    // read address 2
    input  logic [ 4:0] WA,     // write address
    input  logic        we,     // write enable
    input  logic [31:0] WData,  // write data
    output logic [31:0] RD1,    // read data 1
    output logic [31:0] RD2     // read data 2
);

    logic [31:0] reg_file[0:31];  // 32bit 32개.

// //-----------------for_I_tpep-----------------------------//
//     initial begin
//         reg_file[0] = 32'd0;
//         reg_file[1] = 32'd1;
//         reg_file[2] = 32'd2;
//         reg_file[3] = 32'hffff_ffff;
//         reg_file[4] = 32'd4;
//         reg_file[5] = 32'd5;
//         reg_file[6] = 32'd6;
//         reg_file[7] = 32'd7;
//         reg_file[8] = 32'd8;
//         reg_file[9] = 32'd9;
//         reg_file[10] = 32'd10;
//         reg_file[11] = 32'd11;
//         reg_file[12] = 32'd12;
//         reg_file[13] = 32'd13;
//         reg_file[14] = 32'd5; // 101
//         reg_file[15] = 32'h8000_080C; //msb1
//     end

//----------------for_S_type + JAL-----------------------------------//
        initial begin
        reg_file[0] = 32'd0;
        // reg_file[1] = 32'h1111_1111;
        // reg_file[2] = 32'h2222_2222;
        // reg_file[3] = 32'h3333_3333;
        // reg_file[4] = 32'h4444_4444;
        // reg_file[5] = 32'd2;
    end

//----------------for_S_type + JAL-----------------------------------//
    //     initial begin
    //     reg_file[0] = 32'd0;
    //     reg_file[2] = 32'h2;
    // end


//---------------------for_R_type-------------------------//
    // initial begin
    // reg_file [0] = 32'd0;// X0은 0으로 고정
    // reg_file [1] = 32'd1;
    // reg_file [2] = 32'd2;
    // reg_file [3] = 32'd3;
    // reg_file [4] = 32'd4;
    // reg_file [5] = 32'd5;
    // reg_file [6] = 32'd6; 
    // reg_file [7] = 32'd7;
    // reg_file [8] = 32'h0011_0000; // msb ==0
    // reg_file [9] = 32'h8011_0000; // msb == 1
    // reg_file [10] = 32'h8000_0000; // msb == 1
    // reg_file [11] = 32'h0; 

    // end



//----------------------for_U_type---------------------------//

    // initial begin
    //     reg_file [0] = 32'd0;
    // end

//----------------------------for_B_type-------------------//


//-------------------------------------------------------------//

    always_ff @(posedge clk) begin
    if (we && (WA != 5'd0)) begin // X0는 0 fixed!!!
        reg_file[WA] <= WData;
    end
    end

    assign RD1 = (RA1 != 5'd0) ? reg_file[RA1] : 32'd0;
    assign RD2 = (RA2 != 5'd0) ? reg_file[RA2] : 32'd0;

endmodule

module ALU (
    input  logic [31:0] a,
    input  logic [31:0] b,
    input  logic [ 3:0] alu_controls,
    output logic [31:0] alu_result,
    output logic        btaken
);



    always_comb begin
        case (alu_controls)
            `ADD:     alu_result = a + b;
            `SUB:     alu_result = a - b;
            `AND:     alu_result = a & b;
            `OR:      alu_result = a | b;
            `SLT:     alu_result =  ($signed (a) < $signed (b) ) ? 32'h1 : 32'h0; // msb extention 
            `SLTU:    alu_result = (a < b) ? 32'h1 : 32'h0;  //zero extention 
            `SLL :    alu_result = a<<b[4:0]; //zero extention 
            `SRL :    alu_result = a>>b[4:0]; //zero extention 
            `SRA :    alu_result = $signed (a)>>>b[4:0]; //msb extention
            `XOR :    alu_result = a ^ b;
            default: alu_result = 32'bx;
        endcase
    end
    

    //branch

    always_comb begin 
        case (alu_controls[2:0])
            `BEQ : btaken = ($signed(a)==$signed(b))? 1:0; 
            `BNE : btaken = ($signed(a)!=$signed(b))? 1:0; 
            `BLT : btaken = ($signed(a)<$signed(b))? 1:0; 
            `BGE : btaken = ($signed(a)>=$signed(b))? 1:0; 
            `BLTU : btaken = ($unsigned(a)<$unsigned(b))? 1:0; 
            `BGEU : btaken = ($unsigned(a)>=$unsigned(b))? 1:0; 
            default: btaken = 1'b0; // base, +4
        endcase
        
    end


endmodule



module extend (
    input  logic [31:0] instr_code,
    output logic [31:0] imm_txt
);
    
    wire [6:0] opcode = instr_code[6:0];
    wire [2:0] funct3 = instr_code[14:12];



    always_comb begin 
        case (opcode)
           `OP_RTYPE  : imm_txt = 32'bx;
           `OP_STYPE  : imm_txt = {{20{instr_code[31]}},instr_code[31:25],instr_code[11:7]};
           `OP_ILTYPE : imm_txt = {{20{instr_code[31]}},instr_code[31:20]};
           `OP_ITYPE  : imm_txt =  {{20{instr_code[31]}},instr_code[31:20]};    
           `OP_BTYPE  : imm_txt = {{20{instr_code[31]}}, instr_code[7], instr_code[30:25],instr_code[11:8], 1'b0}; //signed_extend
           `OP_LUI    : imm_txt = {instr_code[31:12],12'b0};
           `OP_AUIPC  : imm_txt = {instr_code[31:12],12'b0};
           `OP_JAL    : imm_txt = {{11{instr_code[31]}}, instr_code[31], instr_code[19:12],
                        instr_code[20], instr_code[30:21], 1'b0};
           `OP_JALR   : imm_txt = {{20{instr_code[31]}}, instr_code[31:20]};
            default: imm_txt = 32'bx;
        endcase

        
    end
    
endmodule

module mux_2X1 (
    input  logic        sel,
    input  logic [31:0] x0, // 0 : regfile R2
    input  logic [31:0] x1, // 1 : imm[31:0]
    output logic [31:0] y 
);

    assign y= (sel)? x1 : x0;
    
endmodule


module mux_4X1 (
    input  logic [1:0]  sel,
    input  logic [31:0] x0, // 00 : alu
    input  logic [31:0] x1, // 01 : rdata (ILtype)
    input  logic [31:0] x2, // 10 : LUI
    input  logic [31:0] x3, // 11 : AUIPC , JAL, JALR
    output logic [31:0] y 
);

    always_comb begin 
        case (sel)
            2'b00 : y = x0;
            2'b01 : y = x1;
            2'b10 : y = x2;
            2'b11 : y = x3;
            default: y = x0;
        endcase
    end
    
endmodule



    module load_Store_Unit (
        input  logic        is_load,      // opcode == OP_ILTYPE
        input  logic        is_store,     // opcode == OP_STYPE
        input  logic [2:0]  funct3,       // size/unsigned 정보
        // 이미 올라온 주소/쓰기 데이터
        input  logic [31:0] w_dAddr,      // byte address
        input  logic [31:0] w_dWdata,     // store write data (word)

        // to Data RAM
        output logic [31:0] dAddr,        // byte address (그대로 패스)
        output logic [31:0] dWdata,       // store write data (SB/SH는 하위만 유효)
        output logic [3:0]  wstrb,      // 00:none 01:SW 10:SH 11:SB  

        //from ram
        input  logic [31:0] dRdata,       // word-aligned read data

        // to WriteBack
        output logic [31:0] load_ext,     // 확장된 로드 데이터 (WB로)
        output logic        trap,        // misaligned trap
        output logic        load_misaligned // AND gate용
    );

   // 주소/오프셋
    assign dAddr = w_dAddr; // 그대로 계산값 넘겨주기


    wire [1:0] off = w_dAddr[1:0]; // 바이트 계산

    // funct3 디코드 (RISC-V load/store)
    wire lb  = is_load  && (funct3==3'b000);
    wire lh  = is_load  && (funct3==3'b001);
    wire lw  = is_load  && (funct3==3'b010);
    wire lbu = is_load  && (funct3==3'b100);
    wire lhu = is_load  && (funct3==3'b101);

    wire sb  = is_store && (funct3==3'b000);
    wire sh  = is_store && (funct3==3'b001);
    wire sw  = is_store && (funct3==3'b010);

    // misaligned 
    wire mis_lh = lh  && (off[0]   != 1'b0);
    wire mis_lhu= lhu && (off[0]   != 1'b0);
    wire mis_lw = lw  && (off      != 2'b00);
    wire mis_sh = sh  && (off[0]   != 1'b0);
    wire mis_sw = sw  && (off      != 2'b00);

    wire misaligned = mis_lh | mis_lhu | mis_lw | mis_sh | mis_sw;

    // trap 출력
    assign trap = misaligned;
    assign load_misaligned = misaligned & is_load ;

    // --------------------------
    // STORE 
    // --------------------------
    assign dWdata = w_dWdata;  // input그대로 넘겨줌
    
    always_comb begin 
        wstrb = 4'b0000; // 기본값

        if (!misaligned && is_store) begin
            unique case (funct3)
                3'b000: wstrb = 4'b0001 << off;           // SB
                3'b001: wstrb = off[1] ? 4'b1100 : 4'b0011; // SH (00=low, 10=high)
                3'b010: wstrb = 4'b1111;                  // SW
            endcase 
        end
    end
    

    // --------------------------
    // LOAD 
    // --------------------------
    logic [7:0]  sel_byte;
    logic [15:0] sel_half;

    always_comb begin
         sel_byte = 8'h00;  sel_half = 16'h0000; 
        unique case (off)
            2'd0: begin sel_byte = dRdata[7:0];   sel_half = dRdata[15:0];  end // aligned half
            2'd1: begin sel_byte = dRdata[15:8];  sel_half = 16'hxxxx;      end // misaligned half
            2'd2: begin sel_byte = dRdata[23:16]; sel_half = dRdata[31:16]; end // aligned half
            2'd3: begin sel_byte = dRdata[31:24]; sel_half = 16'hxxxx;      end // misaligned half
        endcase
    end

    always_comb begin
        load_ext = 32'h0000_0000;

        if (misaligned) begin
            load_ext = 32'hxxxx_xxxx; // load_misaligned ==1 이라서 load x
        end else if (is_load) begin // mis_aligned아닌!!
            unique case (funct3)
                3'b000: load_ext = {{24{sel_byte[7]}}, sel_byte};  // LB
                3'b001: load_ext = {{16{sel_half[15]}}, sel_half}; // LH
                3'b010: load_ext = dRdata;                         // LW
                3'b100: load_ext = {24'b0, sel_byte};              // LBU
                3'b101: load_ext = {16'b0, sel_half};              // LHU
                default: load_ext = dRdata;
            endcase
        end
    end
endmodule

