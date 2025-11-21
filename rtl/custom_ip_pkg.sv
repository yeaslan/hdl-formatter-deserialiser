package custom_ip_pkg;

  timeunit 1ns;
  timeprecision 1ps;

  typedef enum logic [1:0] {
    CHANNEL_OFF  = 2'b00,
    CHANNEL_ON   = 2'b01,
    CHANNEL_TP   = 2'b10,
    CHANNEL_ZERO = 2'b11
  } channel_mode_e;

    typedef struct packed {
      channel_mode_e even_mode;
      channel_mode_e odd_mode;
    } command_modes_t;

    // IDLE (CHANNEL_OFF) outputs 0xFF on both channels per integration spec.
    localparam logic [7:0] TP_VALUE   = 8'h96;
    localparam logic [7:0] OFF_VALUE  = 8'hFF;
    localparam logic [7:0] ZERO_VALUE = 8'h00;

  function automatic command_modes_t decode_command(input logic [31:0] cmd);
    command_modes_t modes;
    modes.even_mode = CHANNEL_OFF;
    modes.odd_mode  = CHANNEL_OFF;
    unique case (cmd)
      32'h0000_0000: begin
        modes.even_mode = CHANNEL_OFF;
        modes.odd_mode  = CHANNEL_OFF;
      end
      32'h0000_0001: begin
        modes.even_mode = CHANNEL_ON;
        modes.odd_mode  = CHANNEL_OFF;
      end
      32'h0000_0002: begin
        modes.even_mode = CHANNEL_OFF;
        modes.odd_mode  = CHANNEL_ON;
      end
      32'h0000_0003: begin
        modes.even_mode = CHANNEL_ON;
        modes.odd_mode  = CHANNEL_ON;
      end
      32'h0000_0004: begin
        modes.even_mode = CHANNEL_TP;
        modes.odd_mode  = CHANNEL_OFF;
      end
      32'h0000_0005: begin
        modes.even_mode = CHANNEL_OFF;
        modes.odd_mode  = CHANNEL_TP;
      end
      32'h0000_0006: begin
        modes.even_mode = CHANNEL_TP;
        modes.odd_mode  = CHANNEL_TP;
      end
      32'h0000_0007: begin
        modes.even_mode = CHANNEL_ON;
        modes.odd_mode  = CHANNEL_TP;
      end
      32'h0000_0008: begin
        modes.even_mode = CHANNEL_TP;
        modes.odd_mode  = CHANNEL_ON;
      end
      32'h0000_0009: begin
        modes.even_mode = CHANNEL_ZERO;
        modes.odd_mode  = CHANNEL_ZERO;
      end
      default: begin
        modes.even_mode = CHANNEL_OFF;
        modes.odd_mode  = CHANNEL_OFF;
      end
    endcase
    return modes;
  endfunction

    function automatic logic [7:0] select_channel_byte(
      input channel_mode_e mode,
      input logic [7:0]    fifo_byte
    );
      unique case (mode)
        CHANNEL_ON:   return fifo_byte;
        CHANNEL_TP:   return TP_VALUE;
        CHANNEL_ZERO: return ZERO_VALUE;
        default:      return OFF_VALUE;
      endcase
    endfunction

    function automatic logic [7:0] mode_constant(input channel_mode_e mode);
      return select_channel_byte(mode, 8'h00);
    endfunction

endpackage : custom_ip_pkg
