/*
Copyright (c) 2024 Antmicro <www.antmicro.com>
SPDX-License-Identifier: Apache-2.0
*/

`timescale 1ns / 1ps

/*
 * I3C test
 */
module i3c_harness (
    input wire sda_i,
    input wire scl_i,

    output wire sda_o,
    output wire scl_o,

    input wire pull_sda_low_i,

    input wire [4:0] debug_state_i
);

  assign sda_o = !pull_sda_low_i;
  assign scl_o = '1;

endmodule
