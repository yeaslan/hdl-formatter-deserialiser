module async_fifo #(
  parameter int WIDTH      = 32,
  parameter int DEPTH      = 8,
  parameter int ADDR_WIDTH = $clog2(DEPTH)
) (
  input  logic                 wr_clk,
  input  logic                 wr_rst_n,
  input  logic                 wr_en,
  input  logic [WIDTH-1:0]     wr_data,
  output logic                 wr_full,

  input  logic                 rd_clk,
  input  logic                 rd_rst_n,
  input  logic                 rd_en,
  output logic [WIDTH-1:0]     rd_data,
  output logic                 rd_empty
);

  timeunit 1ns;
  timeprecision 1ps;

  localparam int PTR_WIDTH = ADDR_WIDTH + 1;

  initial begin
    if ((1 << ADDR_WIDTH) != DEPTH) begin
      $error("async_fifo DEPTH (%0d) must be a power of two.", DEPTH);
    end
  end

  logic [WIDTH-1:0] mem [0:DEPTH-1];

  logic [PTR_WIDTH-1:0] wr_ptr_bin, wr_ptr_bin_next;
  logic [PTR_WIDTH-1:0] rd_ptr_bin, rd_ptr_bin_next;
  logic [PTR_WIDTH-1:0] wr_ptr_gray, wr_ptr_gray_next;
  logic [PTR_WIDTH-1:0] rd_ptr_gray, rd_ptr_gray_next;

  logic [PTR_WIDTH-1:0] rd_ptr_gray_sync1, rd_ptr_gray_sync2;
  logic [PTR_WIDTH-1:0] wr_ptr_gray_sync1, wr_ptr_gray_sync2;

  function automatic logic [PTR_WIDTH-1:0] bin2gray(input logic [PTR_WIDTH-1:0] bin);
    return (bin >> 1) ^ bin;
  endfunction

  function automatic logic [PTR_WIDTH-1:0] gray2bin(input logic [PTR_WIDTH-1:0] gray);
    logic [PTR_WIDTH-1:0] bin;
    for (int i = PTR_WIDTH-1; i >= 0; i--) begin
      if (i == PTR_WIDTH-1) begin
        bin[i] = gray[i];
      end else begin
        bin[i] = gray[i] ^ bin[i+1];
      end
    end
    return bin;
  endfunction

  // Write pointer logic
  always_ff @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
      wr_ptr_bin  <= '0;
      wr_ptr_gray <= '0;
    end else begin
      wr_ptr_bin  <= wr_ptr_bin_next;
      wr_ptr_gray <= wr_ptr_gray_next;
    end
  end

  assign wr_ptr_bin_next  = (!wr_full && wr_en) ? wr_ptr_bin + 1'b1 : wr_ptr_bin;
  assign wr_ptr_gray_next = bin2gray(wr_ptr_bin_next);

  // Memory write
  always_ff @(posedge wr_clk) begin
    if (!wr_full && wr_en) begin
      mem[wr_ptr_bin[ADDR_WIDTH-1:0]] <= wr_data;
    end
  end

  // Synchronize read pointer into write domain
  always_ff @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
      rd_ptr_gray_sync1 <= '0;
      rd_ptr_gray_sync2 <= '0;
    end else begin
      rd_ptr_gray_sync1 <= rd_ptr_gray;
      rd_ptr_gray_sync2 <= rd_ptr_gray_sync1;
    end
  end

  // Full flag generation
  always_comb begin
    logic [PTR_WIDTH-1:0] rd_ptr_bin_sync;
    rd_ptr_bin_sync = gray2bin(rd_ptr_gray_sync2);
    wr_full = (wr_ptr_bin_next[PTR_WIDTH-1:PTR_WIDTH-2] == ~rd_ptr_bin_sync[PTR_WIDTH-1:PTR_WIDTH-2]) &&
              (wr_ptr_bin_next[PTR_WIDTH-3:0] == rd_ptr_bin_sync[PTR_WIDTH-3:0]);
  end

  // Read pointer logic
  always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
      rd_ptr_bin  <= '0;
      rd_ptr_gray <= '0;
    end else begin
      rd_ptr_bin  <= rd_ptr_bin_next;
      rd_ptr_gray <= rd_ptr_gray_next;
    end
  end

  assign rd_ptr_bin_next  = (!rd_empty && rd_en) ? rd_ptr_bin + 1'b1 : rd_ptr_bin;
  assign rd_ptr_gray_next = bin2gray(rd_ptr_bin_next);

  // Synchronize write pointer into read domain
  always_ff @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
      wr_ptr_gray_sync1 <= '0;
      wr_ptr_gray_sync2 <= '0;
    end else begin
      wr_ptr_gray_sync1 <= wr_ptr_gray;
      wr_ptr_gray_sync2 <= wr_ptr_gray_sync1;
    end
  end

  // Empty flag generation
  always_comb begin
    logic [PTR_WIDTH-1:0] wr_ptr_bin_sync;
    wr_ptr_bin_sync = gray2bin(wr_ptr_gray_sync2);
    rd_empty = (rd_ptr_bin == wr_ptr_bin_sync);
  end

  // Memory read (combinational output)
  assign rd_data = mem[rd_ptr_bin[ADDR_WIDTH-1:0]];

endmodule : async_fifo
