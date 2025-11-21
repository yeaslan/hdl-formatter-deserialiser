module tb_custom_ip_deserializer #(
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

  custom_ip_deserializer #(
    .FRAME_GROUPS(FRAME_GROUPS)
  ) u_custom_ip_deserializer (
    .clk_75mhz  (clk_75mhz),
    .rst_n      (rst_n),
    .enable     (enable),
    .data_in    (data_in),
    .clk_20mhz  (clk_20mhz),
    .c_even_data(c_even_data),
    .c_odd_data (c_odd_data),
    .pair_valid (pair_valid),
    .frame_start(frame_start),
    .frame_end  (frame_end)
  );

endmodule : tb_custom_ip_deserializer
