/*
Copyright (c) 2024 Antmicro <www.antmicro.com>
SPDX-License-Identifier: Apache-2.0
*/

`timescale 1ns / 1ps

/*
 * I3C test
 */
module i3c_harness (
    input wire sda_ctrl_i,
    input wire scl_ctrl_i,
    input wire [4:0] debug_state_controller_i,

    input wire sda_tgt_i,
    input wire scl_tgt_i,
    input wire [4:0] debug_state_target_i,
    input wire [3:0] debug_detected_header_i,

    output wire sda_o,
    output wire scl_o
);

  assign sda_o = sda_tgt_i & sda_ctrl_i;
  assign scl_o = scl_tgt_i & scl_ctrl_i;

endmodule
