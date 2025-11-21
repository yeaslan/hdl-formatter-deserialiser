module custom_ip_formatter #(
  parameter int FRAME_GROUPS     = 324,
  parameter int GUARD_CYCLES     = 1,
  parameter int MIN_GAP_CYCLES   = 8,
  parameter int CMD_FIFO_DEPTH   = 8,
  parameter int DATA_FIFO_DEPTH  = 512
) (
  input  logic         clk_ctrl,
  input  logic         clk_fast,
  input  logic         rst_n,

  // Command interface (AXI-Lite simplified handshake)
  input  logic         cmd_valid,
  output logic         cmd_ready,
  input  logic [31:0]  cmd_data,

  // Data input interface (AXI-Stream-like)
  input  logic         data_valid,
  output logic         data_ready,
  input  logic [15:0]  data_payload,

  // Serialized output interface
  output logic         clk_75mhz,
  output logic         enable,
  output logic [3:0]   data_out,
  output logic         frame_start,
  output logic         frame_end,
  output logic         underflow_flag
);

  timeunit 1ns;
  timeprecision 1ps;

  import custom_ip_pkg::*;

  localparam int CMD_ADDR_WIDTH  = $clog2(CMD_FIFO_DEPTH);
  localparam int DATA_ADDR_WIDTH = $clog2(DATA_FIFO_DEPTH);

  // Forward fast clock to downstream IP
  assign clk_75mhz = clk_fast;

  // --------------------------------------------------------------------------
  // Command FIFO (crosses from control clock domain to fast serialization domain)
  // --------------------------------------------------------------------------
  logic cmd_fifo_full;
  logic cmd_fifo_empty;
  logic cmd_fifo_rd_en;
  logic [31:0] cmd_fifo_rd_data;

  async_fifo #(
    .WIDTH      (32),
    .DEPTH      (CMD_FIFO_DEPTH),
    .ADDR_WIDTH (CMD_ADDR_WIDTH)
  ) u_cmd_fifo (
    .wr_clk   (clk_ctrl),
    .wr_rst_n (rst_n),
    .wr_en    (cmd_valid && cmd_ready),
    .wr_data  (cmd_data),
    .wr_full  (cmd_fifo_full),

    .rd_clk   (clk_fast),
    .rd_rst_n (rst_n),
    .rd_en    (cmd_fifo_rd_en),
    .rd_data  (cmd_fifo_rd_data),
    .rd_empty (cmd_fifo_empty)
  );

  assign cmd_ready = !cmd_fifo_full;

  // --------------------------------------------------------------------------
  // Data FIFO (crosses from control clock domain to fast serialization domain)
  // --------------------------------------------------------------------------
  logic data_fifo_full;
  logic data_fifo_empty;
  logic data_fifo_rd_en;
  logic [15:0] data_fifo_rd_data;

  async_fifo #(
    .WIDTH      (16),
    .DEPTH      (DATA_FIFO_DEPTH),
    .ADDR_WIDTH (DATA_ADDR_WIDTH)
  ) u_data_fifo (
    .wr_clk   (clk_ctrl),
    .wr_rst_n (rst_n),
    .wr_en    (data_valid && data_ready),
    .wr_data  (data_payload),
    .wr_full  (data_fifo_full),

    .rd_clk   (clk_fast),
    .rd_rst_n (rst_n),
    .rd_en    (data_fifo_rd_en),
    .rd_data  (data_fifo_rd_data),
    .rd_empty (data_fifo_empty)
  );

  assign data_ready = !data_fifo_full;

  // --------------------------------------------------------------------------
  // Fast clock domain control logic
  // --------------------------------------------------------------------------
  typedef enum logic [2:0] {
    ST_IDLE,
    ST_GUARD_LEAD,
    ST_STREAM,
    ST_GUARD_TRAIL,
    ST_GAP
  } state_e;

  state_e state_q, state_d;

  logic [7:0]  current_even_byte, current_odd_byte;
  logic [7:0]  next_even_byte, next_odd_byte;
  logic [1:0]  bit_phase_q;
  logic [1:0]  bit_phase_next;
  localparam int GROUP_IDX_WIDTH = $clog2(FRAME_GROUPS);
  localparam int GAP_CNT_WIDTH   = (MIN_GAP_CYCLES > 0) ? $clog2(MIN_GAP_CYCLES + 1) : 1;
  localparam int GUARD_CNT_WIDTH = (GUARD_CYCLES > 0) ? $clog2(GUARD_CYCLES + 1) : 1;
  logic [GROUP_IDX_WIDTH-1:0] group_idx_q;
  logic [GROUP_IDX_WIDTH-1:0] group_idx_next;
  logic [GAP_CNT_WIDTH-1:0] gap_cnt_q;
  logic [GAP_CNT_WIDTH-1:0] gap_cnt_next;
  logic [GUARD_CNT_WIDTH-1:0] guard_cnt_q;
  logic [GUARD_CNT_WIDTH-1:0] guard_cnt_next;
  logic stream_done;
  logic load_group;
  logic load_first_group;
  logic load_next_group;
  logic need_fifo_word;
  command_modes_t current_modes;
  command_modes_t next_modes;

  // Default assignments
  assign enable = (state_q == ST_STREAM);

  // State transition logic
  always_comb begin
    state_d = state_q;
    case (state_q)
      ST_IDLE: begin
        if (!cmd_fifo_empty) begin
          if (GUARD_CYCLES != 0)
            state_d = ST_GUARD_LEAD;
          else
            state_d = ST_STREAM;
        end
      end
      ST_GUARD_LEAD: begin
        if (guard_cnt_q == GUARD_CYCLES-1) begin
          state_d = ST_STREAM;
        end
      end
      ST_STREAM: begin
        if (stream_done) begin
          if (GUARD_CYCLES != 0)
            state_d = ST_GUARD_TRAIL;
          else
            state_d = ST_GAP;
        end
      end
      ST_GUARD_TRAIL: begin
        if (guard_cnt_q == GUARD_CYCLES-1) begin
          state_d = ST_GAP;
        end
      end
      ST_GAP: begin
        if (gap_cnt_q == MIN_GAP_CYCLES-1) begin
          if (!cmd_fifo_empty)
            state_d = (GUARD_CYCLES != 0) ? ST_GUARD_LEAD : ST_STREAM;
          else
            state_d = ST_IDLE;
        end
      end
      default: state_d = ST_IDLE;
    endcase
  end

    // Determine when to fetch command
    assign cmd_fifo_rd_en = (state_q == ST_IDLE && state_d != ST_IDLE) ||
                            (state_q == ST_GAP && gap_cnt_q == MIN_GAP_CYCLES-1 &&
                             state_d != ST_GAP);

    // Load modes for the upcoming frame
    always_comb begin
      next_modes = current_modes;
      if (cmd_fifo_rd_en && !cmd_fifo_empty) begin
        next_modes = decode_command(cmd_fifo_rd_data);
      end
    end

  logic streaming_now;
  logic streaming_next;
  assign streaming_now  = (state_q == ST_STREAM);
  assign streaming_next = (state_d == ST_STREAM);

  // Bit phase tracking
  always_comb begin
    bit_phase_next = bit_phase_q;
    if (!streaming_now && !streaming_next) begin
      bit_phase_next = 2'd0;
    end else if (!streaming_now && streaming_next) begin
      bit_phase_next = 2'd0;
    end else begin
      if (bit_phase_q == 2'd3)
        bit_phase_next = 2'd0;
      else
        bit_phase_next = bit_phase_q + 2'd1;
    end
  end

  // Group index tracking
  always_comb begin
    group_idx_next = group_idx_q;
    if (state_q != ST_STREAM) begin
      group_idx_next = '0;
    end else if (bit_phase_q == 2'd3 && group_idx_q != FRAME_GROUPS-1) begin
      group_idx_next = group_idx_q + 1;
    end
  end

  assign stream_done = (state_q == ST_STREAM) &&
                       (bit_phase_q == 2'd3) &&
                       (group_idx_q == FRAME_GROUPS-1);

  // Guard counter
  always_comb begin
    guard_cnt_next = guard_cnt_q;
    if (state_q != state_d) begin
      guard_cnt_next = '0;
    end else if ((state_q == ST_GUARD_LEAD) || (state_q == ST_GUARD_TRAIL)) begin
      guard_cnt_next = guard_cnt_q + 1;
    end else begin
      guard_cnt_next = '0;
    end
  end

  // Gap counter
  always_comb begin
    gap_cnt_next = gap_cnt_q;
    if (state_q != ST_GAP) begin
      gap_cnt_next = '0;
    end else begin
      if (gap_cnt_q != MIN_GAP_CYCLES-1) begin
        gap_cnt_next = gap_cnt_q + 1;
      end
    end
  end

  // Determine when to load new group data
  assign load_first_group = (state_q != ST_STREAM) && (state_d == ST_STREAM);
  assign load_next_group  = (state_q == ST_STREAM) && (bit_phase_q == 2'd3) &&
                            (group_idx_q != FRAME_GROUPS-1);
  assign load_group       = load_first_group || load_next_group;

  command_modes_t modes_for_load;
  always_comb begin
    modes_for_load = current_modes;
    if (load_first_group) begin
      modes_for_load = next_modes;
    end
  end

  assign need_fifo_word = load_group &&
                          ((modes_for_load.even_mode == CHANNEL_ON) ||
                           (modes_for_load.odd_mode  == CHANNEL_ON));

  // Underflow tracking
  always_ff @(posedge clk_fast or negedge rst_n) begin
    if (!rst_n) begin
      underflow_flag <= 1'b0;
    end else if (load_group && need_fifo_word && data_fifo_empty) begin
      underflow_flag <= 1'b1;
    end
  end

  // Read enable generation for data FIFO
  always_ff @(posedge clk_fast or negedge rst_n) begin
    if (!rst_n) begin
      data_fifo_rd_en <= 1'b0;
    end else begin
      data_fifo_rd_en <= load_group && need_fifo_word && !data_fifo_empty;
    end
  end

  // Byte selection logic
    always_comb begin
      next_even_byte = current_even_byte;
      next_odd_byte  = current_odd_byte;
      if (load_group) begin
        logic [7:0] even_candidate;
        logic [7:0] odd_candidate;
        logic [15:0] fifo_word_local;
        logic [7:0] fifo_even_byte;
        logic [7:0] fifo_odd_byte;

        fifo_word_local = data_fifo_rd_data;
        fifo_even_byte  = fifo_word_local[7:0];
        fifo_odd_byte   = fifo_word_local[15:8];

        even_candidate = select_channel_byte(modes_for_load.even_mode, fifo_even_byte);
        odd_candidate  = select_channel_byte(modes_for_load.odd_mode,  fifo_odd_byte);

        if (need_fifo_word && data_fifo_empty) begin
          even_candidate = ZERO_VALUE;
          odd_candidate  = ZERO_VALUE;
        end

        next_even_byte = even_candidate;
        next_odd_byte  = odd_candidate;
      end
    end

  // Sequential register updates
  always_ff @(posedge clk_fast or negedge rst_n) begin
    if (!rst_n) begin
      state_q           <= ST_IDLE;
      bit_phase_q       <= 2'd0;
      group_idx_q       <= '0;
      guard_cnt_q       <= '0;
      gap_cnt_q         <= '0;
      current_even_byte <= 8'h00;
      current_odd_byte  <= 8'h00;
      current_modes     <= '{even_mode: CHANNEL_OFF, odd_mode: CHANNEL_OFF};
    end else begin
      state_q           <= state_d;
      bit_phase_q       <= bit_phase_next;
      group_idx_q       <= group_idx_next;
      guard_cnt_q       <= guard_cnt_next;
      gap_cnt_q         <= gap_cnt_next;
      current_even_byte <= next_even_byte;
      current_odd_byte  <= next_odd_byte;
      current_modes     <= next_modes;
    end
  end

  logic [7:0] even_byte_ser;
  logic [7:0] odd_byte_ser;
  logic [1:0] bit_phase_ser;

  always_comb begin
    even_byte_ser = current_even_byte;
    odd_byte_ser  = current_odd_byte;
    bit_phase_ser = bit_phase_q;
  end

  // Output data serialization
  always_ff @(posedge clk_fast or negedge rst_n) begin
    if (!rst_n) begin
      data_out <= 4'd0;
    end else if (streaming_now) begin
      data_out[0] <= odd_byte_ser[3 - bit_phase_ser];
      data_out[1] <= odd_byte_ser[7 - bit_phase_ser];
      data_out[2] <= even_byte_ser[3 - bit_phase_ser];
      data_out[3] <= even_byte_ser[7 - bit_phase_ser];
    end else begin
      data_out <= 4'd0;
    end
  end

  // Frame boundary pulses
  logic frame_start_pulse;

  generate
    if (GUARD_CYCLES == 0) begin : g_frame_start_direct
      always_comb begin
        frame_start_pulse = (state_q != ST_STREAM) && (state_d == ST_STREAM);
      end
    end else begin : g_frame_start_guard
      always_comb begin
        frame_start_pulse = (state_q == ST_IDLE) && (state_d == ST_GUARD_LEAD);
      end
    end
  endgenerate

  always_ff @(posedge clk_fast or negedge rst_n) begin
    if (!rst_n) begin
      frame_start <= 1'b0;
      frame_end   <= 1'b0;
    end else begin
      frame_start <= frame_start_pulse;
      frame_end   <= stream_done && (state_d != ST_STREAM);
    end
  end

endmodule : custom_ip_formatter
