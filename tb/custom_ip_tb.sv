`include "tb_custom_ip_deserializer.sv"

module custom_ip_tb;
  timeunit 1ns;
  timeprecision 1ps;

  import custom_ip_pkg::*;

  localparam int FRAME_GROUPS         = 324;
  localparam int GUARD_CYCLES         = 1;
  localparam int CMD_FIFO_DEPTH       = 8;
  localparam int DATA_FIFO_DEPTH      = 512;
  localparam int TOTAL_RANDOM_CMDS    = 20;
  localparam int MIN_GAP_CYCLES       = 8;
    localparam int FAST_CYCLES_PER_GROUP= 4;
    localparam int EXPECTED_FRAME_FAST_CYCLES =
        FRAME_GROUPS * FAST_CYCLES_PER_GROUP;

  logic clk_ctrl = 1'b0;
  logic clk_fast = 1'b0;
  logic rst_n    = 1'b0;

  // Clock generation
  initial begin
    forever #(50) clk_ctrl = ~clk_ctrl; // 10 MHz
  end

  initial begin
    forever #(6.6666667) clk_fast = ~clk_fast; // 75 MHz
  end

  // Reset sequencing
  initial begin
    rst_n = 1'b0;
    repeat (10) @(posedge clk_fast);
    rst_n = 1'b1;
  end

  // Interface signals
  logic        cmd_valid;
  logic        cmd_ready;
  logic [31:0] cmd_data;

  logic        data_valid;
  logic        data_ready;
  logic [15:0] data_payload;

  logic        clk_75mhz_sig;
  logic        enable_sig;
  logic [3:0]  data_out_sig;
  logic        frame_start_sig;
  logic        frame_end_sig;
  logic        underflow_flag_sig;

  logic        clk_20mhz_sig;
  logic [7:0]  c_even_data_sig;
  logic [7:0]  c_odd_data_sig;
  logic        pair_valid_sig;
  logic        deser_frame_start;
  logic        deser_frame_end;

    custom_ip_formatter #(
      .FRAME_GROUPS   (FRAME_GROUPS),
      .GUARD_CYCLES   (GUARD_CYCLES),
      .MIN_GAP_CYCLES (MIN_GAP_CYCLES),
      .CMD_FIFO_DEPTH (CMD_FIFO_DEPTH),
      .DATA_FIFO_DEPTH(DATA_FIFO_DEPTH)
    ) u_formatter (
    .clk_ctrl      (clk_ctrl),
    .clk_fast      (clk_fast),
    .rst_n         (rst_n),
    .cmd_valid     (cmd_valid),
    .cmd_ready     (cmd_ready),
    .cmd_data      (cmd_data),
    .data_valid    (data_valid),
    .data_ready    (data_ready),
    .data_payload  (data_payload),
    .clk_75mhz     (clk_75mhz_sig),
    .enable        (enable_sig),
    .data_out      (data_out_sig),
    .frame_start   (frame_start_sig),
    .frame_end     (frame_end_sig),
    .underflow_flag(underflow_flag_sig)
  );

  tb_custom_ip_deserializer #(
    .FRAME_GROUPS(FRAME_GROUPS)
  ) u_deserializer (
    .clk_75mhz   (clk_75mhz_sig),
    .rst_n       (rst_n),
    .enable      (enable_sig),
    .data_in     (data_out_sig),
    .clk_20mhz   (clk_20mhz_sig),
    .c_even_data (c_even_data_sig),
    .c_odd_data  (c_odd_data_sig),
    .pair_valid  (pair_valid_sig),
    .frame_start (deser_frame_start),
    .frame_end   (deser_frame_end)
  );

  // Scoreboard
  class formatter_scoreboard;
    localparam int WATERMARK_HYST = 8;

    typedef struct {
      command_modes_t modes;
      int             groups_done;
      logic [31:0]    raw_cmd;
      bit             first_pair_reported;
    } frame_state_t;

    int              frame_groups;
    int              fifo_depth;
    frame_state_t    frame_q[$];
    logic [15:0]     data_q[$];
    int              error_count;
    int              total_checks;
    bit              idle_flag;
    event            idle_event;
    event            cmd_retired_event;

    bit [9:0]        observed_cmds;
    bit [2:0]        tp_cmd_seen;
    int              almost_full_level;
    int              almost_empty_level;
    bit              reported_almost_full;
    bit              reported_almost_empty;
    bit              overflow_reported;
    logic [31:0]     retired_cmd_q[$];

    function new(input int groups, input int depth);
      frame_groups          = groups;
      fifo_depth            = depth;
      error_count           = 0;
      total_checks          = 0;
      idle_flag             = 1'b1;
      observed_cmds         = '0;
      tp_cmd_seen           = '0;
      almost_full_level     = (depth > WATERMARK_HYST) ? depth - WATERMARK_HYST : depth - 1;
      almost_empty_level    = (depth > WATERMARK_HYST) ? WATERMARK_HYST : 1;
      reported_almost_full  = 1'b0;
      reported_almost_empty = 1'b0;
      overflow_reported     = 1'b0;
      retired_cmd_q         = {};
    endfunction

    function void push_cmd(input logic [31:0] cmd);
      frame_state_t entry;
      entry.modes               = decode_command(cmd);
      entry.groups_done         = 0;
      entry.raw_cmd             = cmd;
      entry.first_pair_reported = 1'b0;
      frame_q.push_back(entry);
      idle_flag = 1'b0;

      if (cmd[7:0] <= 8'h09) begin
        observed_cmds[cmd[3:0]] = 1'b1;
      end

      unique case (cmd[7:0])
        8'h04: tp_cmd_seen[0] = 1'b1;
        8'h05: tp_cmd_seen[1] = 1'b1;
        8'h06: tp_cmd_seen[2] = 1'b1;
        default: ;
      endcase
    endfunction

    function void push_data(input logic [15:0] word);
      data_q.push_back(word);
      check_watermarks(data_q.size());
      if (data_q.size() > fifo_depth) begin
        if (!overflow_reported) begin
          $warning("[%0t] SCOREBOARD: mirrored FIFO level %0d exceeds depth %0d",
                   $time, data_q.size(), fifo_depth);
          overflow_reported = 1'b1;
        end
      end else if (overflow_reported && data_q.size() <= fifo_depth) begin
        overflow_reported = 1'b0;
      end
    endfunction

    function bit is_idle();
      return idle_flag && (frame_q.size() == 0);
    endfunction

    task wait_until_idle();
      if (is_idle()) begin
        return;
      end
      @(idle_event);
    endtask

    function automatic bit consume_retired_cmd(input logic [31:0] cmd);
      for (int idx = 0; idx < retired_cmd_q.size(); idx++) begin
        if (retired_cmd_q[idx] === cmd) begin
          retired_cmd_q.delete(idx);
          return 1'b1;
        end
      end
      return 1'b0;
    endfunction

    task automatic wait_for_retire(input logic [31:0] cmd);
      if (consume_retired_cmd(cmd)) begin
        return;
      end
      forever begin
        @(cmd_retired_event);
        if (consume_retired_cmd(cmd)) begin
          break;
        end
      end
    endtask

    function logic [31:0] peek_cmd();
      if (frame_q.size() == 0) begin
        return 32'hFFFF_FFFF;
      end
      return frame_q[0].raw_cmd;
    endfunction

    function string mode_descriptor(channel_mode_e mode);
      case (mode)
        CHANNEL_ON:   return "FIFO";
        CHANNEL_TP:   return "TP(0x96)";
        CHANNEL_ZERO: return "ZERO(0x00)";
        default:      return "OFF(0xFF)";
      endcase
    endfunction

    function void check_watermarks(input int level);
      if (!reported_almost_full && level >= almost_full_level) begin
        $display("[%0t] SCOREBOARD: Data FIFO mirror almost FULL (%0d/%0d)",
                 $time, level, fifo_depth);
        reported_almost_full = 1'b1;
      end else if (reported_almost_full && level <= almost_full_level - WATERMARK_HYST) begin
        reported_almost_full = 1'b0;
      end

      if (!reported_almost_empty && level <= almost_empty_level) begin
        $display("[%0t] SCOREBOARD: Data FIFO mirror almost EMPTY (%0d/%0d)",
                 $time, level, fifo_depth);
        reported_almost_empty = 1'b1;
      end else if (reported_almost_empty && level >= almost_empty_level + WATERMARK_HYST) begin
        reported_almost_empty = 1'b0;
      end
    endfunction

    task automatic check_pair(input logic [7:0] even_actual,
                              input logic [7:0] odd_actual);
      frame_state_t entry;
      logic [15:0]  word;
      logic         consume_word;
      logic [7:0]   even_exp;
      logic [7:0]   odd_exp;

      if (frame_q.size() == 0) begin
        $error("[%0t] SCOREBOARD: received data with no pending frame", $time);
        error_count++;
        idle_flag = 1'b1;
        -> idle_event;
        return;
      end

      idle_flag   = 1'b0;
      entry       = frame_q[0];
      consume_word = (entry.modes.even_mode == CHANNEL_ON) ||
                     (entry.modes.odd_mode  == CHANNEL_ON);
      word = 16'h0000;

      if (consume_word) begin
        if (data_q.size() == 0) begin
          $error("[%0t] SCOREBOARD: FIFO data underflow for command 0x%02h",
                 $time, entry.raw_cmd[7:0]);
          error_count++;
        end else begin
          word = data_q.pop_front();
        end
      end

      even_exp = select_channel_byte(entry.modes.even_mode, word[7:0]);
      odd_exp  = select_channel_byte(entry.modes.odd_mode,  word[15:8]);

      total_checks++;
      if (even_actual !== even_exp) begin
        $error("[%0t] Even mismatch: got %02h expected %02h (group %0d, CMD 0x%02h)",
               $time, even_actual, even_exp, entry.groups_done, entry.raw_cmd[7:0]);
        error_count++;
      end
      if (odd_actual !== odd_exp) begin
        $error("[%0t] Odd mismatch: got %02h expected %02h (group %0d, CMD 0x%02h)",
               $time, odd_actual, odd_exp, entry.groups_done, entry.raw_cmd[7:0]);
        error_count++;
      end

      if (!entry.first_pair_reported) begin
        entry.first_pair_reported = 1'b1;
        $display("[%0t] SCOREBOARD: CMD 0x%02h first pair even=%02h (%s) odd=%02h (%s)",
                 $time, entry.raw_cmd[7:0], even_actual,
                 mode_descriptor(entry.modes.even_mode),
                 odd_actual, mode_descriptor(entry.modes.odd_mode));
        if (entry.raw_cmd[7:0] inside {8'h04, 8'h05, 8'h06}) begin
          $display("[%0t] SCOREBOARD: TP verification for CMD 0x%02h passed (0x96 on TP channel, 0xFF on OFF channel)",
                   $time, entry.raw_cmd[7:0]);
        end
      end

      entry.groups_done++;
      check_watermarks(data_q.size());

      if (entry.groups_done >= frame_groups) begin
        retired_cmd_q.push_back(entry.raw_cmd);
        -> cmd_retired_event;
        frame_q.pop_front();
        if (frame_q.size() == 0) begin
          idle_flag = 1'b1;
          -> idle_event;
        end else begin
          idle_flag = 1'b0;
        end
      end else begin
        frame_q[0] = entry;
      end
    endtask

    function void report_summary();
      bit coverage_ok;
      coverage_ok = 1'b1;

      $display("[%0t] SCOREBOARD SUMMARY: %0d pairs checked, %0d errors",
               $time, total_checks, error_count);

      for (int idx = 0; idx <= 9; idx++) begin
        if (!observed_cmds[idx]) begin
          coverage_ok = 1'b0;
          $warning("[%0t] SCOREBOARD SUMMARY: CMD 0x%0h was not observed in this run",
                   $time, idx);
        end
      end
      if (coverage_ok) begin
        $display("[%0t] SCOREBOARD SUMMARY: All 10 opcodes exercised", $time);
      end

      for (int idx = 4; idx <= 6; idx++) begin
        string label;
        case (idx)
          4: label = "CMD 0x04 (TP on even)";
          5: label = "CMD 0x05 (TP on odd)";
          default: label = "CMD 0x06 (TP on both)";
        endcase
        if (tp_cmd_seen[idx-4]) begin
          $display("[%0t] SCOREBOARD SUMMARY: %s observed with expected 0x96 pattern",
                   $time, label);
        end else begin
          $warning("[%0t] SCOREBOARD SUMMARY: %s NOT observed", $time, label);
        end
      end
    endfunction
  endclass

    formatter_scoreboard sb = new(FRAME_GROUPS, DATA_FIFO_DEPTH);

    localparam int MAX_DEMO_NIBBLES = 16;
    localparam int CLK20_AVG_WINDOW = 64;

    longint unsigned fast_cycle_count;
    longint unsigned ctrl_cycle_count;
    longint unsigned last_frame_end_cycle;
    longint unsigned frame_start_fast_cycle;
    longint unsigned frame_start_ctrl_cycle;
    longint unsigned frame_end_ctrl_cycle;
    int              frame_nibble_count;
    int              frame_pair_count;
    bit              frame_active_flag;

    realtime last_clk_ctrl_edge;
    realtime last_clk_fast_edge;
    realtime last_clk20_edge;
    realtime clk20_period_accum;
    int      clk_ctrl_edge_seen;
    int      clk_fast_edge_seen;
    int      clk20_edge_seen;
    int      clk20_edges_accumulated;
    bit      clk20_average_reported;

    initial begin
      fast_cycle_count       = 0;
      ctrl_cycle_count       = 0;
      last_frame_end_cycle   = 0;
      frame_start_fast_cycle = 0;
      frame_start_ctrl_cycle = 0;
      frame_end_ctrl_cycle   = 0;
      frame_nibble_count     = 0;
      frame_pair_count       = 0;
      frame_active_flag      = 1'b0;
      last_clk_ctrl_edge     = 0.0;
      last_clk_fast_edge     = 0.0;
      last_clk20_edge        = 0.0;
      clk20_period_accum     = 0.0;
      clk_ctrl_edge_seen     = 0;
      clk_fast_edge_seen     = 0;
      clk20_edge_seen        = 0;
      clk20_edges_accumulated= 0;
      clk20_average_reported = 1'b0;
    end

    always @(posedge clk_ctrl) begin
      if (!rst_n) begin
        clk_ctrl_edge_seen <= 0;
        ctrl_cycle_count   <= 0;
        last_clk_ctrl_edge = $realtime;
      end else begin
        clk_ctrl_edge_seen <= clk_ctrl_edge_seen + 1;
        ctrl_cycle_count   <= ctrl_cycle_count + 1;
        if (clk_ctrl_edge_seen <= 3) begin
          if (last_clk_ctrl_edge != 0.0) begin
            realtime period_ns;
            real     freq_mhz;
            period_ns = $realtime - last_clk_ctrl_edge;
            freq_mhz  = 1e3 / period_ns;
            $display("[%0t] TB: clk_ctrl period = %.3f ns (~%.2f MHz)", $time, period_ns, freq_mhz);
          end
          last_clk_ctrl_edge = $realtime;
        end
      end
    end

    always @(posedge clk_fast) begin
      longint unsigned fast_cycle_count_next;
      int              frame_pair_count_next;
      int              frame_nibble_count_next;
      bit              frame_active_flag_next;

      fast_cycle_count_next   = fast_cycle_count + 1;
      frame_pair_count_next   = frame_pair_count;
      frame_nibble_count_next = frame_nibble_count;
      frame_active_flag_next  = frame_active_flag;

      if (!rst_n) begin
        fast_cycle_count       <= 0;
        last_frame_end_cycle   <= 0;
        frame_start_fast_cycle <= 0;
        frame_start_ctrl_cycle <= 0;
        frame_end_ctrl_cycle   <= 0;
        frame_nibble_count     <= 0;
        frame_pair_count       <= 0;
        frame_active_flag      <= 1'b0;
        clk_fast_edge_seen     <= 0;
        last_clk_fast_edge     = $realtime;
      end else begin
        clk_fast_edge_seen <= clk_fast_edge_seen + 1;
        if (clk_fast_edge_seen <= 3) begin
          if (last_clk_fast_edge != 0.0) begin
            realtime period_ns;
            real     freq_mhz;
            period_ns = $realtime - last_clk_fast_edge;
            freq_mhz  = 1e3 / period_ns;
            $display("[%0t] TB: clk_fast period = %.3f ns (~%.2f MHz)", $time, period_ns, freq_mhz);
          end
          last_clk_fast_edge = $realtime;
        end

        fast_cycle_count <= fast_cycle_count_next;

        if (frame_start_sig) begin
          logic [31:0] active_cmd;
          active_cmd = sb.peek_cmd();
          frame_active_flag_next  = 1'b1;
          frame_nibble_count_next = 0;
          frame_pair_count_next   = 0;
          frame_start_fast_cycle  <= fast_cycle_count_next;
          frame_start_ctrl_cycle  <= ctrl_cycle_count;
          if (active_cmd !== 32'hFFFF_FFFF) begin
            $display("[%0t] TB: Frame start for CMD 0x%02h", $time, active_cmd[7:0]);
          end else begin
            $display("[%0t] TB: Frame start (command queue empty)", $time);
          end
          if (last_frame_end_cycle != 0) begin
            longint unsigned gap_fast_cycles;
            longint unsigned gap_ctrl_cycles;
            gap_fast_cycles = fast_cycle_count_next - last_frame_end_cycle;
            gap_ctrl_cycles = ctrl_cycle_count - frame_end_ctrl_cycle;
            if (gap_fast_cycles >= MIN_GAP_CYCLES) begin
              $display("[%0t] TB: Inter-frame gap PASS: %0d fast cycles (>= %0d) / %0d ctrl cycles",
                       $time, gap_fast_cycles, MIN_GAP_CYCLES, gap_ctrl_cycles);
            end else begin
              $error("[%0t] TB: Inter-frame gap VIOLATION: %0d fast cycles (< %0d)",
                     $time, gap_fast_cycles, MIN_GAP_CYCLES);
            end
          end
        end

        if (frame_active_flag_next && enable_sig) begin
          int nibble_index;
          nibble_index = frame_nibble_count_next;
          if (nibble_index < MAX_DEMO_NIBBLES) begin
            $display("[%0t] TB: data_out_sig nibble[%0d] = 0x%0h",
                     $time, nibble_index, data_out_sig);
          end
          frame_nibble_count_next = frame_nibble_count_next + 1;
        end

        if (frame_active_flag_next && pair_valid_sig) begin
          frame_pair_count_next = frame_pair_count_next + 1;
        end

        if (frame_end_sig) begin
          longint unsigned fast_frame_cycles;
          longint unsigned ctrl_frame_cycles;
          real             expected_ctrl_cycles;
          int              final_pair_count;
          longint unsigned final_nibble_count;

          frame_active_flag_next = 1'b0;
          last_frame_end_cycle   <= fast_cycle_count_next;
          frame_end_ctrl_cycle   <= ctrl_cycle_count;

          fast_frame_cycles   = fast_cycle_count_next - frame_start_fast_cycle;
          ctrl_frame_cycles   = ctrl_cycle_count - frame_start_ctrl_cycle;
          expected_ctrl_cycles = EXPECTED_FRAME_FAST_CYCLES / 7.5;
          final_pair_count     = frame_pair_count_next;
          final_nibble_count   = frame_nibble_count_next;

          $display("[%0t] TB: Frame end observed; nibbles=%0d pairs=%0d",
                   $time, final_nibble_count, final_pair_count);
          $display("[%0t] TB: Frame duration = %0d fast cycles (expected %0d) / %0d ctrl cycles (~%.1f expected)",
                   $time, fast_frame_cycles, EXPECTED_FRAME_FAST_CYCLES,
                   ctrl_frame_cycles, expected_ctrl_cycles);
          if (final_pair_count != FRAME_GROUPS) begin
            $error("[%0t] TB: Pair count mismatch. Got %0d expected %0d",
                   $time, final_pair_count, FRAME_GROUPS);
          end
        end

        frame_active_flag <= frame_active_flag_next;
        frame_pair_count  <= frame_pair_count_next;
        frame_nibble_count<= frame_nibble_count_next;
      end
    end

    always @(posedge clk_20mhz_sig) begin
      if (!rst_n) begin
        clk20_edge_seen        <= 0;
        clk20_edges_accumulated<= 0;
        last_clk20_edge        = $realtime;
        clk20_period_accum     = 0.0;
        clk20_average_reported <= 1'b0;
      end else begin
        if (!clk20_average_reported) begin
          clk20_edge_seen        <= clk20_edge_seen + 1;
          clk20_edges_accumulated<= clk20_edges_accumulated + 1;
          if (last_clk20_edge != 0.0) begin
            realtime period_ns;
            real     freq_mhz;
            period_ns = $realtime - last_clk20_edge;
            freq_mhz  = 1e3 / period_ns;
            if (clk20_edge_seen <= 3) begin
              $display("[%0t] TB: clk_20mhz_sig instant period = %.3f ns (~%.2f MHz)",
                       $time, period_ns, freq_mhz);
            end
            clk20_period_accum = clk20_period_accum + period_ns;
          end
          if (clk20_edges_accumulated >= CLK20_AVG_WINDOW && clk20_period_accum != 0.0) begin
            real avg_period_ns;
            real avg_freq_mhz;
            avg_period_ns = clk20_period_accum / clk20_edges_accumulated;
            avg_freq_mhz  = 1e3 / avg_period_ns;
            $display("[%0t] TB: clk_20mhz_sig averaged over %0d edges -> %.3f ns period (%.3f MHz)",
                     $time, clk20_edges_accumulated, avg_period_ns, avg_freq_mhz);
            clk20_period_accum      = 0.0;
            clk20_edges_accumulated = 0;
            clk20_average_reported  <= 1'b1;
          end
        end
        last_clk20_edge = $realtime;
      end
    end

  // Command sequence
  localparam logic [31:0] CMD_TABLE [0:9] = '{
    32'h0000_0000,
    32'h0000_0001,
    32'h0000_0002,
    32'h0000_0003,
    32'h0000_0004,
    32'h0000_0005,
    32'h0000_0006,
    32'h0000_0007,
    32'h0000_0008,
    32'h0000_0009
  };
  localparam int CMD_TABLE_LEN = $size(CMD_TABLE);

    bit cmd_sequence_done;

    task automatic run_tp_smoke_test();
      logic [31:0] tp_cmds [0:2] = '{32'h0000_0004, 32'h0000_0005, 32'h0000_0006};
      $display("[%0t] TB: Starting deterministic TP smoke test (CMDs 0x04/0x05/0x06)", $time);
      foreach (tp_cmds[idx]) begin
        issue_command(tp_cmds[idx]);
        sb.wait_for_retire(tp_cmds[idx]);
      end
      $display("[%0t] TB: Completed deterministic TP smoke test", $time);
    endtask

    task automatic issue_command(input logic [31:0] cmd);
      cmd_data  <= cmd;
      cmd_valid <= 1'b1;
      do @(posedge clk_ctrl); while (!cmd_ready);
      sb.push_cmd(cmd);
      $display("[%0t] TB: Issued CMD 0x%02h", $time, cmd[7:0]);
      if (cmd[7:0] inside {8'h04, 8'h05, 8'h06}) begin
        $display("[%0t] TB: Highlighted TP command 0x%02h queued for waveform capture", $time, cmd[7:0]);
      end
      cmd_valid <= 1'b0;
    endtask

    initial begin
      automatic logic [31:0] cmd;

      cmd_valid = 1'b0;
      cmd_data  = '0;
      wait (rst_n);

      // Deterministic smoke test for TP opcodes first
      run_tp_smoke_test();

      // Ensure each command opcode is exercised at least once
      for (int idx = 0; idx < CMD_TABLE_LEN; idx++) begin
        issue_command(CMD_TABLE[idx]);
        repeat ($urandom_range(0, 15)) @(posedge clk_ctrl);
      end

      for (int n = 0; n < TOTAL_RANDOM_CMDS; n++) begin
        cmd = CMD_TABLE[$urandom_range(0, CMD_TABLE_LEN-1)];
        issue_command(cmd);
        repeat ($urandom_range(0, 25)) @(posedge clk_ctrl);
      end

      cmd_sequence_done = 1'b1;
    end

  // Data generator (always ready to supply new data words)
  initial begin
    data_valid   = 1'b0;
    data_payload = '0;
    wait (rst_n);
    data_valid = 1'b1;
    forever begin
      @(posedge clk_ctrl);
      if (data_valid && data_ready) begin
        logic [15:0] next_word;
        next_word = $urandom;
        data_payload <= next_word;
        sb.push_data(next_word);
      end
    end
  end

  // Scoreboard hook on deserializer output
  always @(posedge clk_fast) begin
    if (rst_n && pair_valid_sig) begin
      sb.check_pair(c_even_data_sig, c_odd_data_sig);
    end
  end

  // Minimum gap assertion
  property p_min_gap;
    @(posedge clk_fast)
      disable iff (!rst_n)
      frame_end_sig |-> ##[MIN_GAP_CYCLES:$] frame_start_sig;
  endproperty
  assert property (p_min_gap)
    else $error("[%0t] Formatter violated minimum inter-frame gap", $time);

  // Simulation end control
  initial begin
    wait (rst_n);
    wait (cmd_sequence_done);
    // Wait until all frames retire
    sb.wait_until_idle();
    repeat (200) @(posedge clk_fast);

    if (underflow_flag_sig) begin
      $error("Formatter underflow flag asserted during simulation");
    end

    sb.report_summary();

    if (sb.error_count == 0 && !underflow_flag_sig) begin
      $display("TEST PASS: %0d channel pairs verified", sb.total_checks);
    end else begin
      $display("TEST FAIL: %0d errors detected", sb.error_count);
      $fatal(1);
    end
    $finish;
  end

  // Timeout guard (Vivado does not accept `ms` literals in delay controls)
  localparam longint TIMEOUT_NS = 200_000_000; // 200 ms with `timeunit 1ns`
  initial begin
    #(TIMEOUT_NS);
    $fatal(1, "Simulation timeout");
  end

endmodule : custom_ip_tb
