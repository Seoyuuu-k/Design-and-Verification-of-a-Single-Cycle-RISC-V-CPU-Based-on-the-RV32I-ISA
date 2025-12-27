`timescale 1ns/1ps


//fpga는 ip써서!!
// 여긴 다 설계 x 


module data_ram ( // word-aligned RAM (32b word array)
    input  logic        clk,
    input  logic [3:0]  wstrb,   
    input  logic [31:0] dAddr,     // byte address
    input  logic [31:0] dWdata,
    output logic [31:0] dRdata
);

  localparam DEPTH = 128;
  logic [31:0] data_mem [0:DEPTH-1]; // 100개 word
  // ram size : 처리할 수 있는 data 몇개까지?
  // 크게 잡는게 넉넉!
  // 너무 크면 시뮬할때 너무 오래걸림

  // 워드 인덱스
 wire [$clog2(DEPTH)-1:0] word_addr = dAddr[31:2]; // 7비트 → 128워드까지 커버

//--------------S_type볼때는 X-----------------------------//
//  초기값 (예: 0x87654321, 0x87654322, ...)
  // initial begin
  //   for (int i = 0; i < 10; i++) begin
  //     data_mem[i] = 32'h8765_4321 + i;
  //   end
  // end
  // initial begin
  //   data_mem[4]=32'h8765_4321;
  // end

  always_ff @(posedge clk) begin
    if (wstrb[0]) data_mem[word_addr][ 7:0 ]  <= dWdata[ 7:0 ];
    if (wstrb[1]) data_mem[word_addr][15:8 ]  <= dWdata[15:8 ];
    if (wstrb[2]) data_mem[word_addr][23:16]  <= dWdata[23:16];
    if (wstrb[3]) data_mem[word_addr][31:24]  <= dWdata[31:24];
  end

  // READ: 워드 단위로 바로 반환
  assign dRdata = data_mem[word_addr];


endmodule

