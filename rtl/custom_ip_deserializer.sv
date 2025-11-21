module custom_ip_deserializer #(
  parameter int FRAME_GROUPS = 324
) (
  input  logic       clk_75mhz,
  input  logic       rst_n,
  input  logic       enable,
  input  logic [3:0] data_in,

  output logic       clk_20mhz,
  output logic [7:0] c_even_data,
  output logic [7:0] c_odd_data,
  output logic       pair_valid,
  output logic       frame_start,
  output logic       frame_end
);

  timeunit 1ns;
  timeprecision 1ps;

  localparam int GROUP_IDX_WIDTH = $clog2(FRAME_GROUPS);
  localparam logic [31:0] PHASE_INC = 32'd1145324612; // ~20 MHz from 75 MHz input

  logic [31:0] phase_accum_q, phase_accum_d;
  logic clk_20mhz_d;

  always_comb begin
    phase_accum_d = phase_accum_q + PHASE_INC;
    clk_20mhz_d   = phase_accum_d[31];
  end

  always_ff @(posedge clk_75mhz or negedge rst_n) begin
    if (!rst_n) begin
      phase_accum_q <= 32'd0;
      clk_20mhz     <= 1'b0;
    end else begin
      phase_accum_q <= phase_accum_d;
      clk_20mhz     <= clk_20mhz_d;
    end
  end

  logic enable_q;
  always_ff @(posedge clk_75mhz or negedge rst_n) begin
    if (!rst_n) begin
      enable_q <= 1'b0;
    end else begin
      enable_q <= enable;
    end
  end

  assign frame_start = enable && !enable_q;
  assign frame_end   = !enable && enable_q;

  logic [1:0] bit_phase_q, bit_phase_d;
  always_comb begin
    bit_phase_d = bit_phase_q;
    if (!enable) begin
      bit_phase_d = 2'd0;
    end else begin
      if (bit_phase_q == 2'd3) begin
        bit_phase_d = 2'd0;
      end else begin
        bit_phase_d = bit_phase_q + 2'd1;
      end
    end
  end

  logic [GROUP_IDX_WIDTH-1:0] group_idx_q, group_idx_d;
  logic frame_active_q, frame_active_d;
  logic pair_valid_d;
  logic [7:0] c_even_d, c_odd_d;
  logic [3:0] odd_low_q, odd_low_d;
  logic [3:0] odd_high_q, odd_high_d;
  logic [3:0] even_low_q, even_low_d;
  logic [3:0] even_high_q, even_high_d;

  always_comb begin
    frame_active_d = frame_active_q;
    if (frame_start) begin
      frame_active_d = 1'b1;
    end else if (frame_end) begin
      frame_active_d = 1'b0;
    end
  end

  always_comb begin
    group_idx_d = group_idx_q;
    if (frame_start) begin
      group_idx_d = '0;
    end else if (frame_active_q && pair_valid_d) begin
      group_idx_d = group_idx_q + 1;
    end
  end

  always_comb begin
    odd_low_d  = odd_low_q;
    odd_high_d = odd_high_q;
    even_low_d = even_low_q;
    even_high_d= even_high_q;
    pair_valid_d = 1'b0;
    c_even_d   = c_even_data;
    c_odd_d    = c_odd_data;

    if (!enable) begin
      odd_low_d   = '0;
      odd_high_d  = '0;
      even_low_d  = '0;
      even_high_d = '0;
    end else begin
      case (bit_phase_q)
        2'd0: begin
          odd_low_d[3]   = data_in[0];
          odd_high_d[3]  = data_in[1];
          even_low_d[3]  = data_in[2];
          even_high_d[3] = data_in[3];
        end
        2'd1: begin
          odd_low_d[2]   = data_in[0];
          odd_high_d[2]  = data_in[1];
          even_low_d[2]  = data_in[2];
          even_high_d[2] = data_in[3];
        end
        2'd2: begin
          odd_low_d[1]   = data_in[0];
          odd_high_d[1]  = data_in[1];
          even_low_d[1]  = data_in[2];
          even_high_d[1] = data_in[3];
        end
        default: begin
          odd_low_d[0]   = data_in[0];
          odd_high_d[0]  = data_in[1];
          even_low_d[0]  = data_in[2];
          even_high_d[0] = data_in[3];
        end
      endcase

      if (bit_phase_q == 2'd3) begin
        pair_valid_d = 1'b1;
        c_even_d     = {even_high_d, even_low_d};
        c_odd_d      = {odd_high_d, odd_low_d};
      end
    end
  end

  always_ff @(posedge clk_75mhz or negedge rst_n) begin
    if (!rst_n) begin
      bit_phase_q    <= 2'd0;
      group_idx_q    <= '0;
      frame_active_q <= 1'b0;
      odd_low_q      <= '0;
      odd_high_q     <= '0;
      even_low_q     <= '0;
      even_high_q    <= '0;
      pair_valid     <= 1'b0;
      c_even_data    <= 8'h00;
      c_odd_data     <= 8'h00;
    end else begin
      bit_phase_q    <= bit_phase_d;
      group_idx_q    <= group_idx_d;
      frame_active_q <= frame_active_d;
      odd_low_q      <= odd_low_d;
      odd_high_q     <= odd_high_d;
      even_low_q     <= even_low_d;
      even_high_q    <= even_high_d;
      pair_valid     <= pair_valid_d;
      if (pair_valid_d) begin
        c_even_data <= c_even_d;
        c_odd_data  <= c_odd_d;
      end
    end
  end

endmodule : custom_ip_deserializer
