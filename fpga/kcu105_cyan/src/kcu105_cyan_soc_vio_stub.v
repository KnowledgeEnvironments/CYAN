// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// KCU105 Cyan SoC debug VIO synthesis boundary.
//
// Vivado 2020.1 can leave this generated stub stale when the VIO probe shape
// changes. Keep the top-level synthesis contract local and let the VIO OOC run
// provide the implementation checkpoint.

(* X_CORE_INFO = "vio,Vivado 2020.1", black_box, syn_black_box *)
module kcu105_cyan_soc_vio (
    input  wire         clk,
    input  wire [ 31:0] probe_in0,
    input  wire [ 31:0] probe_in1,
    input  wire [ 31:0] probe_in2,
    input  wire [ 31:0] probe_in3,
    input  wire [127:0] probe_in4,
    input  wire [255:0] probe_in5,
    input  wire [255:0] probe_in6,
    input  wire [127:0] probe_in7,
    input  wire [255:0] probe_in8,
    input  wire [255:0] probe_in9,
    input  wire [255:0] probe_in10,
    input  wire [255:0] probe_in11,
    output wire [ 31:0] probe_out0
);
endmodule
