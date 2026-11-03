// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - LG1 clean VC1/XMAP/DAC scanout

`default_nettype none

module lg1_vc1_xmap_dac_scanout #(
    parameter ENABLE_DIAGNOSTIC_RESOLVER = 1
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        cfg_valid,
    input  wire        cfg_write,
    input  wire        cfg_go,
    input  wire [10:0] cfg_offset,
    input  wire [ 2:0] cfg_sel,
    input  wire [ 7:0] cfg_wbyte,
    output reg  [ 7:0] cfg_rbyte,

    input  wire        sample_valid,
    // Asserted only for the real-time framebuffer stream.  The physical path
    // consumes the sequentially built DID line cache; arbitrary-coordinate
    // replay samples retain the combinational reference resolver.
    input  wire        sample_physical,
    input  wire        physical_enable,
    input  wire [ 9:0] sample_x,
    input  wire [ 9:0] sample_y,
    input  wire [ 7:0] sample_raw_index,
    // Retained for capture/interface provenance. LG1 has no XMAP mode which
    // selects this byte as an independently addressable CI8 framebuffer.
    input  wire [ 7:0] sample_raw_back_index,
    input  wire [ 1:0] sample_overlay,
    output reg         sample_de,
    output reg  [ 4:0] sample_did,
    output reg  [15:0] sample_mode,
    output reg  [ 1:0] sample_palette_bank,
    output reg  [ 7:0] sample_display_index,
    output reg  [ 7:0] sample_r,
    output reg  [ 7:0] sample_g,
    output reg  [ 7:0] sample_b,

    output reg         video_de,
    output reg         video_hsync,
    output reg         video_vsync,
    output reg  [ 7:0] video_r,
    output reg  [ 7:0] video_g,
    output reg  [ 7:0] video_b,
    output reg         video_valid,
    output reg         video_frame_start,
    output reg  [ 4:0] video_did,
    output reg  [ 7:0] video_raw_index,
    output reg  [ 7:0] video_index,
    output reg  [15:0] video_xmap_mode,
    output reg  [ 1:0] video_palette_bank,
    output reg  [ 2:0] video_resolver_class,

    // Coordinates registered with video_de/RGB.  They are retained for
    // full-frame physical-pipeline verification and synthesize away when the
    // board integration does not observe them.
    output reg  [ 9:0] video_x,
    output reg  [ 9:0] video_y,

    output wire [ 9:0] timing_x,
    output wire [ 9:0] timing_y,
    output wire        timing_visible,

    // Once-per-complete-physical-frame resolver classification.  This is
    // observation-only: no telemetry state feeds the pixel resolver.
    // resolver_telemetry = {configuration-stable, display-enable,
    // window-mode, window-seen, read-mask, overlay pixels, flood pixels,
    // cursor pixels, FRONT pixels resolving to non-black RGB}.
    // resolver_detail = {saturating FRONT nonzero-index count, FRONT DID-0
    // pixels, FRONT XMAP-bank-0 pixels, bank-1 pixels, bank-2 pixels}.
    output reg  [91:0] resolver_telemetry,
    output reg  [91:0] resolver_detail,
    output reg         resolver_frame_done,

    // One-clock raw VC1 vertical-retrace event. INT2 latches this event
    // until software pulses PORT_CONFIG.PCON_CLEARVRI low then high.
    output reg         retrace_irq
);
    localparam [10:0] CFG_DAC = 11'h7e8;
    localparam [10:0] CFG_VC1 = 11'h7f0;

    localparam [10:0] H_VISIBLE = 11'd1024;
    localparam [10:0] H_SYNC_START = 11'd1048;
    localparam [10:0] H_SYNC_END = 11'd1184;
    // The compatible timing table contains 264 duration units per
    // scanline.  VC1 advances each duration unit at one fifth of the 64 MHz
    // pixel clock, giving the physical 1,320-pixel line period.  H_TOTAL is
    // only the pre-table/fail-closed fallback; accepted tables replace it
    // with their decoded common line duration below.
    localparam [10:0] H_TOTAL = 11'd1320;

    localparam [ 9:0] V_VISIBLE = 10'd768;
    localparam [ 9:0] V_SYNC_START = 10'd771;
    localparam [ 9:0] V_SYNC_END = 10'd777;
    localparam [ 9:0] V_TOTAL = 10'd806;

    localparam [7:0] VC1_REVISION_ID = 8'h02;

    reg [10:0] h_ctr;
    reg [ 9:0] v_ctr;

    assign timing_x = h_ctr[9:0];
    assign timing_y = v_ctr;
    assign timing_visible = (h_ctr < H_VISIBLE) && (v_ctr < V_VISIBLE);

    reg [ 7:0] dac_rdmask;
    reg [ 7:0] dac_cmd0;
    reg [ 7:0] dac_cmd1;
    reg [ 7:0] dac_flood_lo;
    reg [ 7:0] dac_flood_hi;
    reg [ 7:0] dac_latch;
    reg [ 7:0] dac_addr;
    reg [ 1:0] dac_rgb_phase;
    reg [ 7:0] dac_rgb_stage_r;
    reg [ 7:0] dac_rgb_stage_g;
    reg [ 7:0] dac_rgb_read_r;
    reg [ 7:0] dac_rgb_read_g;
    reg [ 7:0] dac_rgb_read_b;

`ifndef SYNTHESIS
    // Arbitrary-coordinate diagnostic samples retain a combinational mirror.
    // The hardware pixel path below uses palette_mem's synchronous read port.
    reg [ 7:0] pal_r [0:255];
    reg [ 7:0] pal_g [0:255];
    reg [ 7:0] pal_b [0:255];
    reg [ 7:0] pal1_r [0:255];
    reg [ 7:0] pal1_g [0:255];
    reg [ 7:0] pal1_b [0:255];
    reg [ 7:0] pal2_r [0:255];
    reg [ 7:0] pal2_g [0:255];
    reg [ 7:0] pal2_b [0:255];
    reg [ 7:0] cur_pal_r [0:255];
    reg [ 7:0] cur_pal_g [0:255];
    reg [ 7:0] cur_pal_b [0:255];
`endif
    (* ram_style = "block" *) reg [23:0] palette_mem [0:1023];
    reg [23:0] palette_pixel_q;
    reg [23:0] palette_cfg_q;
    reg        palette_load_pending;
    reg [ 7:0] bt_overlay_r [0:15];
    reg [ 7:0] bt_overlay_g [0:15];
    reg [ 7:0] bt_overlay_b [0:15];
    reg [ 7:0] dac_window_regs [0:129];

    reg [ 7:0] vc1_regs [0:127];
`ifndef SYNTHESIS
    reg [ 7:0] vc1_xmap [0:127];
`endif
    (* ram_style = "distributed" *) reg [15:0] xmap_mode_mem [0:63];
    reg [15:0] xmap_pixel_q;
    reg [15:0] xmap_cfg_q;
    reg [ 7:0] vc1_sys_ctrl;
    reg [15:0] vc1_addr;
    reg [ 7:0] vc1_mpu_msb;

    wire [ 7:0] vc1_cfg_ram_rdata;
    wire [31:0] vc1_scan_ram_rdata;
    reg  [15:0] vc1_cfg_ram_addr;
    reg         vc1_scan_ram_en;
    reg  [15:0] vc1_scan_ram_addr;
    reg  [ 3:0] vc1_scan_ram_tag;
    reg         vc1_scan_rsp_valid;
    reg  [ 3:0] vc1_scan_rsp_tag;
    reg  [ 1:0] vc1_scan_rsp_lane;

`ifndef SYNTHESIS
    // Arbitrary-coordinate replay is intentionally a simulation-only profile.
    // Hardware uses the synchronous dual-port SRAM and sequential line caches.
    reg [7:0] vc1_ram_diag [0:65535];
`endif

    // Physical scanout DID cache.  VC1 display-list RAM is decoded once per
    // line, sequentially, instead of placing a 128-entry search in the pixel
    // critical path. Frame-table provenance is retained between adjacent
    // lines; after a mutation the bounded prefix walk may span multiple lines.
    reg [4:0] did_even [0:1023];
    reg [4:0] did_odd  [0:1023];
    reg       did_even_valid;
    reg       did_odd_valid;
    reg [9:0] did_even_line;
    reg [9:0] did_odd_line;

    localparam [1:0] DID_IDLE  = 2'd0;
    localparam [1:0] DID_FETCH = 2'd1;
    localparam [1:0] DID_LOAD  = 2'd2;
    localparam [1:0] DID_FILL  = 2'd3;

    localparam [3:0] SCAN_PTR_HI   = 4'd1;
    localparam [3:0] SCAN_PTR_LO   = 4'd2;
    localparam [3:0] SCAN_COUNT_HI = 4'd3;
    localparam [3:0] SCAN_COUNT_LO = 4'd4;
    localparam [3:0] SCAN_SEG_HI   = 4'd5;
    localparam [3:0] SCAN_SEG_LO   = 4'd6;
    localparam [3:0] SCAN_CURSOR_P0 = 4'd7;
    localparam [3:0] SCAN_CURSOR_P1 = 4'd8;
    localparam [3:0] SCAN_TIMING = 4'd9;

    // VC1 timing is byte-packed in the same SRAM as the DID and cursor data.
    // A bounded bootstrap walk derives only the vertical contract needed by
    // the board integration: total lines, the first visible line, and the
    // unique falling edge of C0 (VERT_INT_REX_N).  The walk borrows the
    // existing scan port before releasing it to the per-line DID builder;
    // there is deliberately no third RAM port or timing-table replica.
    localparam [3:0] TIMING_IDLE       = 4'd0;
    localparam [3:0] TIMING_HEADER_HI  = 4'd1;
    localparam [3:0] TIMING_HEADER_LO  = 4'd2;
    localparam [3:0] TIMING_RUN_PTR_HI = 4'd3;
    localparam [3:0] TIMING_RUN_PTR_LO = 4'd4;
    localparam [3:0] TIMING_RUN_COUNT  = 4'd5;
    localparam [3:0] TIMING_LINE_A     = 4'd6;
    localparam [3:0] TIMING_LINE_B     = 4'd7;
    localparam [3:0] TIMING_LINE_C     = 4'd8;
    localparam [3:0] TIMING_DURATION   = 4'd9;
    localparam [3:0] TIMING_NEXT_ZERO  = 4'd10;
    localparam [3:0] TIMING_NEXT_HI    = 4'd11;
    localparam [3:0] TIMING_NEXT_LO    = 4'd12;

    reg [3:0] timing_scan_state;
    reg       timing_scan_active;
    reg       timing_rebuild_pending;
    reg       timing_start_armed;
    reg       timing_apply_pending;
    reg       timing_active;
    reg [9:0] timing_frame_lines;
    reg [9:0] timing_candidate_frame_lines;
    reg [9:0] timing_raw_line;
    reg [9:0] timing_active_start_raw;
    reg [9:0] timing_irq_raw;
    reg [9:0] timing_irq_line;
    reg [10:0] timing_line_clocks;
    reg [11:0] timing_line_duration;
    reg [11:0] timing_reference_duration;
    reg        timing_reference_duration_valid;
    reg        timing_duration_mismatch;
    reg [15:0] timing_frame_base;
    reg [15:0] timing_frame_entry_addr;
    reg [15:0] timing_frame_end;
    reg [15:0] timing_line_ptr;
    reg [15:0] timing_line_addr;
    reg [15:0] timing_line_min;
    reg [15:0] timing_line_max;
    reg [7:0] timing_scan_hi;
    reg [7:0] timing_run_remaining;
    reg [6:0] timing_state_a;
    reg [6:0] timing_state_b;
    reg [6:0] timing_state_c;
    reg [5:0] timing_line_runs;
    reg       timing_line_active;
    reg       timing_active_start_seen;
    reg       timing_prev_c0_valid;
    reg       timing_prev_c0;
    reg       timing_first_c0_valid;
    reg       timing_first_c0;
    reg       timing_irq_seen;
    reg       timing_irq_multiple;

    wire [10:0] timing_horizontal_total = timing_active ?
        timing_line_clocks : H_TOTAL;

    reg [1:0] did_build_state;
    reg       did_build_buffer;
    reg [9:0] did_build_line;
    reg [15:0] did_build_ptr;
    reg [8:0] did_build_count;
    reg [8:0] did_build_entry;
    reg [9:0] did_build_x;
    reg [4:0] did_build_value;
    reg [7:0] did_scan_hi;
    // VC1 frame entries are row-indexed: 8000|N repeats the preceding line
    // pointer for N rows, then resumes at the row after that span. Covered
    // slots are not commands. Keep one proven span, never a guessed previous
    // pointer from whichever target happened to be requested after reset.
    reg       did_span_valid;
    reg [9:0] did_span_first;
    reg [9:0] did_span_end;
    reg [15:0] did_span_ptr;
    reg [9:0] did_walk_row;
    (* ram_style = "distributed" *) reg [15:0] did_segment_mem [0:127];
    reg [10:0] did_active_segment_x;
    reg [ 4:0] did_active_segment_value;
    reg [15:0] cursor_build_row_addr;
    reg [31:0] cursor_even_plane0;
    reg [31:0] cursor_even_plane1;
    reg [31:0] cursor_odd_plane0;
    reg [31:0] cursor_odd_plane1;
    reg        cursor_even_valid;
    reg        cursor_odd_valid;
    reg [ 9:0] cursor_even_line;
    reg [ 9:0] cursor_odd_line;
    // Xsgi can update CUR_XL/CUR_YL faster than the physical raster.  Hold a
    // single position for one complete source frame, then adopt the latest
    // position at the next (0,0) boundary.
    reg [15:0] cursor_frame_x;
    reg [15:0] cursor_frame_y;

    wire [9:0] did_target_line = (v_ctr < V_VISIBLE) ?
        ((v_ctr == (V_VISIBLE - 10'd1)) ? 10'd0 : v_ctr + 10'd1) :
        10'd0;
    wire did_target_buffer = did_target_line[0];
    wire did_target_valid = did_target_buffer ?
        (did_odd_valid && (did_odd_line == did_target_line)) :
        (did_even_valid && (did_even_line == did_target_line));
    wire did_sample_valid = sample_y[0] ?
        (did_odd_valid && (did_odd_line == sample_y)) :
        (did_even_valid && (did_even_line == sample_y));
    wire [4:0] did_sample_value = sample_y[0] ? did_odd[sample_x] :
                                                       did_even[sample_x];
    wire did_span_forward = did_span_valid && (did_target_line != 10'd0) &&
        (did_target_line >= did_build_line);
    wire did_target_span_hit = did_span_forward &&
        (did_target_line >= did_span_first) &&
        (did_target_line < did_span_end);
    wire [9:0] did_walk_start =
        (did_span_forward && (did_target_line >= did_span_end)) ?
        did_span_end : 10'd0;
    wire did_build_span_hit = did_span_valid &&
        (did_build_line >= did_span_first) &&
        (did_build_line < did_span_end);
    wire [15:0] did_frame_entry =
        {vc1_regs[7'h40], vc1_regs[7'h41]} +
        {5'h0, did_walk_row, 1'b0};
    wire did_segment_pending = did_build_entry < did_build_count;
    wire did_segment_reached = did_segment_pending &&
        (did_active_segment_x <= {1'b0, did_build_x});
    wire signed [16:0] cursor_target_origin_y =
        $signed({1'b0, cursor_frame_y}) - 17'sd39;
    wire signed [16:0] cursor_target_dy =
        $signed({7'h0, did_target_line}) - cursor_target_origin_y;
    wire cursor_target_normal = (vc1_sys_ctrl[5:4] == 2'b11) &&
        (vc1_regs[7'h26] != 8'h01) && (vc1_regs[7'h26] != 8'h03) &&
        ({vc1_regs[7'h20], vc1_regs[7'h21]} <= 16'hff00) &&
        (cursor_target_dy >= 17'sd0) && (cursor_target_dy < 17'sd32);
    wire [15:0] cursor_target_row_addr =
        {vc1_regs[7'h20], vc1_regs[7'h21]} +
        {9'h0, cursor_target_dy[4:0], 2'b00};

    wire [1:0] dac_palette_bank = dac_cmd0[5:4];
    // The LG1 Bt479 package is strapped for 8-bit operation, so command0
    // D0-D3 always mask OL0-OL3. Command1 D1 controls only whether MPU color
    // transfers carry six or eight data bits; it is not the external 8/6*
    // package pin and must not disable the overlay-input mask.
    wire [1:0] dac_overlay_mask = dac_cmd0[1:0];
    wire cfg_dac_access = cfg_valid && (cfg_offset == CFG_DAC);
    wire cfg_vc1_access = cfg_valid && (cfg_offset == CFG_VC1);
    // LIGHT's cursor interrupt saves register 0080 when an SRAM/XMAP
    // transfer is interrupted at an odd host address, then restores that
    // staged MSB at address-1. An even access must not publish a half word:
    // CONFIGSEL is shared with the DAC, so that restore can also occur with
    // a stale VC1 address and no subsequent odd data access at all.
    wire vc1_word_write = cfg_vc1_access && cfg_write && cfg_go &&
        ((cfg_sel == 3'd1) || (cfg_sel == 3'd2));
    wire vc1_word_commit = vc1_word_write && vc1_addr[0];
    wire [15:0] vc1_word_addr = {vc1_addr[15:1], 1'b0};
    wire [15:0] vc1_word_data = {vc1_mpu_msb, cfg_wbyte};
    wire vc1_cfg_ram_write = vc1_word_commit && (cfg_sel == 3'd2);
    wire vc1_did_cache_mutation = cfg_vc1_access && cfg_write && cfg_go &&
        ((vc1_cfg_ram_write &&
          (vc1_addr >= {vc1_regs[7'h40], vc1_regs[7'h41]})) ||
         ((cfg_sel == 3'd0) &&
          (vc1_addr[15:7] == 9'h0) &&
          (vc1_addr[6:0] >= 7'h40) &&
          (vc1_addr[6:0] <= 7'h45)));
    wire vc1_cursor_position_mutation = cfg_vc1_access && cfg_write &&
        cfg_go && (cfg_sel == 3'd0) && (vc1_addr[15:7] == 9'h0) &&
        (vc1_addr[6:0] >= 7'h22) && (vc1_addr[6:0] <= 7'h25);
    wire vc1_cursor_cache_mutation = cfg_vc1_access && cfg_write && cfg_go &&
        (vc1_cfg_ram_write || (cfg_sel == 3'd6) ||
         ((cfg_sel == 3'd0) && (vc1_addr[15:7] == 9'h0) &&
          (vc1_addr[6:0] >= 7'h20) && (vc1_addr[6:0] <= 7'h29) &&
          !vc1_cursor_position_mutation));

    // Port A is continuously prefetched. Address-register writes point the
    // synchronous read port at the newly programmed address on the same edge;
    // a successful SRAM read points it at the auto-incremented next byte.
    always @(*) begin
        vc1_cfg_ram_addr = vc1_addr;
        if (cfg_vc1_access && cfg_go) begin
            if (cfg_write && (cfg_sel == 3'd4))
                vc1_cfg_ram_addr = {vc1_addr[15:8], cfg_wbyte};
            else if (cfg_write && (cfg_sel == 3'd5))
                vc1_cfg_ram_addr = {cfg_wbyte, vc1_addr[7:0]};
            else if (!cfg_write && (cfg_sel == 3'd2))
                vc1_cfg_ram_addr = vc1_addr + 16'd1;
        end
    end

    lg1_vc1_ram_64k u_vc1_ram (
        .clk(clk),
        .cfg_en(1'b1),
        .cfg_write(vc1_cfg_ram_write),
        .cfg_addr(vc1_cfg_ram_addr),
        .cfg_wdata(vc1_word_data),
        .cfg_rdata(vc1_cfg_ram_rdata),
        .scan_en(vc1_scan_ram_en),
        .scan_addr(vc1_scan_ram_addr),
        .scan_rdata(vc1_scan_ram_rdata)
    );

    integer init_i;
    initial begin
        for (init_i = 0; init_i < 256; init_i = init_i + 1) begin
`ifndef SYNTHESIS
            pal_r[init_i] = 8'h0;
            pal_g[init_i] = 8'h0;
            pal_b[init_i] = 8'h0;
            pal1_r[init_i] = 8'h0;
            pal1_g[init_i] = 8'h0;
            pal1_b[init_i] = 8'h0;
            pal2_r[init_i] = 8'h0;
            pal2_g[init_i] = 8'h0;
            pal2_b[init_i] = 8'h0;
            cur_pal_r[init_i] = 8'h0;
            cur_pal_g[init_i] = 8'h0;
            cur_pal_b[init_i] = 8'h0;
`endif
            palette_mem[{2'd0, init_i[7:0]}] = 24'h0;
            palette_mem[{2'd1, init_i[7:0]}] = 24'h0;
            palette_mem[{2'd2, init_i[7:0]}] = 24'h0;
            palette_mem[{2'd3, init_i[7:0]}] = 24'h0;
        end
        for (init_i = 0; init_i < 16; init_i = init_i + 1) begin
            bt_overlay_r[init_i] = 8'h0;
            bt_overlay_g[init_i] = 8'h0;
            bt_overlay_b[init_i] = 8'h0;
        end
        for (init_i = 0; init_i < 130; init_i = init_i + 1)
            dac_window_regs[init_i] = 8'h0;
        for (init_i = 0; init_i < 128; init_i = init_i + 1) begin
            vc1_regs[init_i] = 8'h0;
`ifndef SYNTHESIS
            vc1_xmap[init_i] = 8'h0;
`endif
        end
        for (init_i = 0; init_i < 64; init_i = init_i + 1)
            xmap_mode_mem[init_i] = 16'h0;
`ifndef SYNTHESIS
        for (init_i = 0; init_i < 65536; init_i = init_i + 1)
            vc1_ram_diag[init_i] = 8'h0;
`endif
    end

    function [15:0] vc1_ram_word;
        input [15:0] a;
        begin
`ifndef SYNTHESIS
            vc1_ram_word = {vc1_ram_diag[a],
                            vc1_ram_diag[a + 16'd1]};
`else
            vc1_ram_word = 16'h0;
`endif
        end
    endfunction

    function [31:0] vc1_ram_dword;
        input [15:0] a;
        begin
`ifndef SYNTHESIS
            vc1_ram_dword = {
                vc1_ram_diag[a],
                vc1_ram_diag[a + 16'd1],
                vc1_ram_diag[a + 16'd2],
                vc1_ram_diag[a + 16'd3]
            };
`else
            vc1_ram_dword = 32'h0;
`endif
        end
    endfunction

    function [7:0] vc1_scan_word_byte;
        input [31:0] word;
        input [ 1:0] lane;
        begin
            case (lane)
                2'd0: vc1_scan_word_byte = word[31:24];
                2'd1: vc1_scan_word_byte = word[23:16];
                2'd2: vc1_scan_word_byte = word[15:8];
                default: vc1_scan_word_byte = word[7:0];
            endcase
        end
    endfunction

    wire [7:0] vc1_scan_byte =
        vc1_scan_word_byte(vc1_scan_ram_rdata, vc1_scan_rsp_lane);

    wire [15:0] did_frame_word = {did_scan_hi, vc1_scan_byte};
    // A widened sum preserves the bound before narrowing a legal row index.
    wire [15:0] did_decoded_end = {6'd0, did_walk_row} +
        (did_frame_word[15] ? {1'b0, did_frame_word[14:0]} : 16'd1);
    wire did_frame_word_valid = (did_decoded_end <= {6'd0, V_VISIBLE}) &&
        (!did_frame_word[15] ||
         (did_span_valid && (did_span_end == did_walk_row) &&
          (did_frame_word[14:0] != 15'd0)));
    wire [15:0] did_decoded_ptr = did_frame_word[15] ?
        did_span_ptr : did_frame_word;

    function did_line_count_valid;
        input [15:0] line_ptr;
        input [15:0] count;
        begin
            did_line_count_valid =
                (count != 16'h0) &&
                (count <= 16'd128) &&
                (line_ptr <= (16'hffff - (16'd2 + {count[14:0], 1'b0})));
        end
    endfunction

    function [15:0] xmap_word;
        input [4:0] did;
        input       odd_field;
        reg [6:0] off;
        begin
`ifndef SYNTHESIS
            off = {1'b0, did, 1'b0};
            if (odd_field)
                off = off + 7'h40;
            xmap_word = {vc1_xmap[off], vc1_xmap[off + 7'd1]};
`else
            off = 7'h0;
            xmap_word = 16'h0;
`endif
        end
    endfunction

    function [7:0] vc1_mask;
        input [6:0] a;
        begin
            case (a)
                // VC1 VID_ENAB is a six-bit register in the high byte at
                // 0x14.  LIGHT initializes it with 0x1d00, and the SGI VC1
                // diagnostic masks register tests to those six bits.
                7'h14: vc1_mask = 8'h3f;
                7'h22: vc1_mask = 8'h07;
                7'h24: vc1_mask = 8'h0f;
                7'h28: vc1_mask = 8'h07;
                7'h44: vc1_mask = 8'hff;
                7'h45: vc1_mask = 8'h07;
                7'h62: vc1_mask = 8'hff;
                default: vc1_mask = 8'hff;
            endcase
        end
    endfunction

    function [7:0] palette_component;
        input [1:0] bank;
        input [7:0] index;
        input [1:0] component;
        begin
`ifndef SYNTHESIS
            case (bank)
                2'd1: begin
                    case (component)
                        2'd0: palette_component = pal1_r[index];
                        2'd1: palette_component = pal1_g[index];
                        default: palette_component = pal1_b[index];
                    endcase
                end
                2'd2: begin
                    case (component)
                        2'd0: palette_component = pal2_r[index];
                        2'd1: palette_component = pal2_g[index];
                        default: palette_component = pal2_b[index];
                    endcase
                end
                2'd3: begin
                    case (component)
                        2'd0: palette_component = cur_pal_r[index];
                        2'd1: palette_component = cur_pal_g[index];
                        default: palette_component = cur_pal_b[index];
                    endcase
                end
                default: begin
                    case (component)
                        2'd0: palette_component = pal_r[index];
                        2'd1: palette_component = pal_g[index];
                        default: palette_component = pal_b[index];
                    endcase
                end
            endcase
`else
            palette_component = 8'h0;
`endif
        end
    endfunction

    function [7:0] xmap_ci_index;
        input [15:0] mode;
        input [7:0] raw_index;
        begin
            // mode[0] selects packed CI4; palette bank is [3:2], page [7:4].
            // Display readback consumes an unchanged raw RWAUX byte:
            // mode[1] clear selects low, set high.
            // Keep this absolute polarity paired with REX DBLDST/DBLSRC.
            // With CI4 disabled, software stores a logical map's colors at
            // pixel + (cmap & 8'hf0).  The overlapping mode encoding leaves
            // that CI8 page in mode[7:6], so apply the same offset modulo 256.
            if (mode[0])
                xmap_ci_index = {mode[7:4],
                                 mode[1] ? raw_index[7:4]
                                         : raw_index[3:0]};
            else
                xmap_ci_index = raw_index + {mode[7:6], 6'b0};
        end
    endfunction

    function [7:0] dac_mpu_color_write;
        input [7:0] value;
        begin
            // Bt479 command1 D1 controls MPU color-data width only.  In
            // six-bit mode D7:D6 are ignored and the six payload bits feed
            // the DAC MSBs; the package's external 8/6* pin stays in the
            // board's fixed eight-bit Bt479 configuration.
            dac_mpu_color_write = dac_cmd1[1] ? value
                                              : {value[5:0], 2'b00};
        end
    endfunction

    function [7:0] dac_mpu_color_read;
        input [7:0] value;
        begin
            // Six-bit MPU reads return the stored DAC MSBs in D5:D0 and
            // force D7:D6 low.
            dac_mpu_color_read = dac_cmd1[1] ? value
                                             : {2'b00, value[7:2]};
        end
    endfunction

    function [7:0] dac_control_value;
        input [7:0] address;
        input [7:0] value;
        begin
            // X2/Y1/Y2 and vertical-count high bytes expose twelve-bit
            // coordinate/count state. X1 high additionally carries the
            // window palette selector in D7:D4 and is intentionally full.
            if (((address <= 8'h7f) &&
                 ((address[2:0] == 3'd3) ||
                  (address[2:0] == 3'd5) ||
                  (address[2:0] == 3'd7))) ||
                (address == 8'h81))
                dac_control_value = value & 8'h0f;
            else if (address == 8'h83)
                // Command1 D7:D5 are hardwired low.
                dac_control_value = value & 8'h1f;
            else
                dac_control_value = value;
        end
    endfunction

    function [7:0] bt479_window_select;
        input [9:0] visible_x;
        input [9:0] visible_y;
        reg [11:0] bt_x;
        reg [11:0] bt_y;
        reg [11:0] x1;
        reg [11:0] x2;
        reg [11:0] y1;
        reg [11:0] y2;
        reg        inside_x;
        reg        inside_y;
        reg        flood;
        integer    window_i;
        begin
            // The Bt479 resets its position counters at the falling edge of
            // composite sync. Visible pixel (0,0) follows the programmed LG1
            // horizontal and vertical sync starts by 296 clocks and 35 lines.
            bt_x = {2'b00, visible_x} + 12'd296;
            bt_y = {2'b00, visible_y} + 12'd35;
            // {valid, flood, window[3:0], palette[1:0]}. Window 0 has the
            // highest priority, so a descending scan intentionally lets the
            // lower-numbered matching window replace an earlier match.
            bt479_window_select = 8'h00;
            for (window_i = 15; window_i >= 0;
                 window_i = window_i - 1) begin
                x1 = {dac_window_regs[window_i * 8 + 1][3:0],
                      dac_window_regs[window_i * 8 + 0]};
                x2 = {dac_window_regs[window_i * 8 + 3][3:0],
                      dac_window_regs[window_i * 8 + 2]};
                y1 = {dac_window_regs[window_i * 8 + 5][3:0],
                      dac_window_regs[window_i * 8 + 4]};
                y2 = {dac_window_regs[window_i * 8 + 7][3:0],
                      dac_window_regs[window_i * 8 + 6]};
                inside_x = (x1 != x2) &&
                    (((x1 < x2) && (bt_x >= x1) && (bt_x < x2)) ||
                     ((x2 < x1) && (bt_x >= x2) && (bt_x < x1)));
                inside_y = (y1 != y2) &&
                    (((y1 < y2) && (bt_y >= y1) && (bt_y < y2)) ||
                     ((y2 < y1) && (bt_y >= y2) && (bt_y < y1)));
                flood = (window_i < 8) ? dac_flood_lo[window_i] :
                                         dac_flood_hi[window_i - 8];
                // Palette values 4..15 are reserved by the Bt479. Keep the
                // deterministic framebuffer fallback for those encodings.
                if (inside_x && inside_y &&
                    (dac_window_regs[window_i * 8 + 1][7:6] == 2'b00))
                    bt479_window_select = {
                        1'b1, flood, window_i[3:0],
                        dac_window_regs[window_i * 8 + 1][5:4]
                    };
            end
        end
    endfunction

    function [7:0] dac_read_port_value;
        input [2:0] sel;
        begin
            case (sel)
                3'd0, 3'd3, 3'd4, 3'd7: dac_read_port_value = dac_addr;
                3'd1, 3'd5: begin
                    case (dac_rgb_phase)
                        2'd0: dac_read_port_value =
                            dac_mpu_color_read(dac_rgb_read_r);
                        2'd1: dac_read_port_value =
                            dac_mpu_color_read(dac_rgb_read_g);
                        default: dac_read_port_value =
                            dac_mpu_color_read(dac_rgb_read_b);
                    endcase
                end
                3'd2: dac_read_port_value = dac_rdmask;
                3'd6: begin
                    if (dac_addr <= 8'h81)
                        dac_read_port_value = dac_control_value(
                            dac_addr, dac_window_regs[dac_addr]);
                    else begin
                        case (dac_addr)
                            8'h82: dac_read_port_value = dac_cmd0;
                            8'h83: dac_read_port_value = dac_control_value(
                                dac_addr, dac_cmd1);
                            8'h84: dac_read_port_value = dac_flood_lo;
                            8'h85: dac_read_port_value = dac_flood_hi;
                            default: dac_read_port_value = 8'h00;
                        endcase
                    end
                end
                default: dac_read_port_value = dac_latch;
            endcase
        end
    endfunction

    function [7:0] vc1_register_read;
        input [15:0] a;
        begin
            if (a == 16'h0080)
                vc1_register_read = vc1_mpu_msb;
            else
                vc1_register_read = vc1_regs[a[6:0]];
        end
    endfunction

    function [7:0] vc1_read_port_value;
        input [2:0] sel;
        begin
            case (sel)
                3'd0: vc1_read_port_value = vc1_register_read(vc1_addr);
                3'd1: begin
`ifndef SYNTHESIS
                    vc1_read_port_value = vc1_xmap[vc1_addr[6:0]];
`else
                    vc1_read_port_value = vc1_addr[0] ? xmap_cfg_q[7:0] :
                                                       xmap_cfg_q[15:8];
`endif
                end
                3'd2: vc1_read_port_value = vc1_cfg_ram_rdata;
                3'd3: vc1_read_port_value = VC1_REVISION_ID;
                3'd4: vc1_read_port_value = vc1_addr[7:0];
                3'd5: vc1_read_port_value = vc1_addr[15:8];
                3'd6: vc1_read_port_value = vc1_sys_ctrl;
                default: vc1_read_port_value = dac_latch;
            endcase
        end
    endfunction

    task palette_write;
        input [1:0] bank;
        input [7:0] index;
        input [1:0] component;
        input [7:0] value;
        begin
`ifndef SYNTHESIS
            case (bank)
                2'd1: begin
                    case (component)
                        2'd0: pal1_r[index] <= value;
                        2'd1: pal1_g[index] <= value;
                        default: pal1_b[index] <= value;
                    endcase
                end
                2'd2: begin
                    case (component)
                        2'd0: pal2_r[index] <= value;
                        2'd1: pal2_g[index] <= value;
                        default: pal2_b[index] <= value;
                    endcase
                end
                2'd3: begin
                    case (component)
                        2'd0: cur_pal_r[index] <= value;
                        2'd1: cur_pal_g[index] <= value;
                        default: cur_pal_b[index] <= value;
                    endcase
                end
                default: begin
                    case (component)
                        2'd0: pal_r[index] <= value;
                        2'd1: pal_g[index] <= value;
                        default: pal_b[index] <= value;
                    endcase
                end
            endcase
`endif
        end
    endtask

    reg        dec_de;
    reg [ 4:0] dec_did;
    reg [15:0] dec_mode;
    reg [ 1:0] dec_bank;
    reg [ 7:0] dec_raw_index;
    reg [ 7:0] dec_index;
    reg [ 7:0] dec_r;
    reg [ 7:0] dec_g;
    reg [ 7:0] dec_b;

    reg [15:0] dec_did_ep;
    reg [15:0] dec_line_ptr;
    reg [15:0] dec_line_count;
    reg [15:0] dec_entry_addr;
    reg [10:0] dec_seg_x;
    reg [ 7:0] dec_x_hi;
    reg [ 7:0] dec_did_lo;
    reg [ 7:0] dec_overlay_index;
    reg [ 1:0] dec_masked_overlay;
    reg [ 7:0] dec_window_select;
    reg [15:0] dec_cursor_entry;
    reg [15:0] dec_cursor_x;
    reg [15:0] dec_cursor_y;
    reg signed [16:0] dec_cursor_origin_x;
    reg signed [16:0] dec_cursor_origin_y;
    reg signed [16:0] dec_cursor_dx;
    reg signed [16:0] dec_cursor_dy;
    reg [ 4:0] dec_cursor_cx;
    reg [ 4:0] dec_cursor_cy;
    reg [ 4:0] dec_cursor_bit;
    reg [31:0] dec_cursor_plane0;
    reg [31:0] dec_cursor_plane1;
    reg [ 7:0] dec_cursor_mode;
    reg [ 7:0] dec_cursor_bx;
    reg [10:0] dec_cursor_ly;
    reg [18:0] dec_cursor_special_offset;
    reg [19:0] dec_cursor_special_addr;
    reg [ 7:0] dec_cursor_special_byte;
    reg [ 2:0] dec_cursor_special_bit;
    integer dec_i;

    // Two registered stages for the real-time stream. Stage zero captures the
    // cached DID and raw planes; stage one registers XMAP/index selection; the
    // video outputs register palette/overlay/cursor conversion. Diagnostic
    // samples continue to use dec_* below so replay can address pixels in any
    // order without carrying pipeline state between rows.
    reg        pipe0_de;
    reg        pipe0_did_ready;
    reg        pipe0_hsync;
    reg        pipe0_vsync;
    reg [4:0]  pipe0_did;
    reg [9:0]  pipe0_x;
    reg [9:0]  pipe0_y;
    reg [7:0]  pipe0_front;
    reg [1:0]  pipe0_overlay;

    reg        xmap0_de;
    reg        xmap0_did_ready;
    reg        xmap0_hsync;
    reg        xmap0_vsync;
    reg [4:0]  xmap0_did;
    reg [9:0]  xmap0_x;
    reg [9:0]  xmap0_y;
    reg [7:0]  xmap0_front;
    reg [1:0]  xmap0_overlay;

    reg        pipe1_de;
    reg        pipe1_did_ready;
    reg        pipe1_hsync;
    reg        pipe1_vsync;
    reg [9:0]  pipe1_x;
    reg [9:0]  pipe1_y;
    reg [1:0]  pipe1_overlay;
    reg [4:0]  pipe1_did;
    reg [7:0]  pipe1_raw_index;
    reg [15:0] pipe1_xmap_mode;
    reg [1:0]  pipe1_bank;
    reg [7:0]  pipe1_index;

    reg [15:0] pipe_xmap_mode;
    reg [1:0]  pipe_xmap_bank;
    reg [7:0]  pipe_xmap_raw;
    reg [7:0]  pipe_xmap_index;
    reg [7:0]  pipe_overlay_index;
    reg [1:0]  pipe_masked_overlay;
    reg [7:0]  pipe_window_select;
    reg [15:0] pipe_cursor_entry;
    reg [15:0] pipe_cursor_x;
    reg [15:0] pipe_cursor_y;
    reg signed [16:0] pipe_cursor_origin_x;
    reg signed [16:0] pipe_cursor_origin_y;
    reg signed [16:0] pipe_cursor_dx;
    reg signed [16:0] pipe_cursor_dy;
    reg [4:0] pipe_cursor_cx;
    reg [4:0] pipe_cursor_cy;
    reg [4:0] pipe_cursor_bit;
    reg [31:0] pipe_cursor_plane0;
    reg [31:0] pipe_cursor_plane1;
    reg [7:0] pipe_cursor_mode;
    reg [7:0] pipe_cursor_index;
    reg [7:0] pipe_cursor_bx;
    reg [10:0] pipe_cursor_ly;
    reg [18:0] pipe_cursor_special_offset;
    reg [19:0] pipe_cursor_special_addr;
    reg [7:0] pipe_cursor_special_byte;
    reg [2:0] pipe_cursor_special_bit;
    reg [7:0] pipe_display_index;
    reg [1:0] pipe_palette_bank;
    reg [7:0] pipe_palette_index;
    reg pipe_flood;
    reg [23:0] pipe_flood_rgb;

    reg pipe2_de;
    reg pipe2_did_ready;
    reg pipe2_hsync;
    reg pipe2_vsync;
    reg [9:0] pipe2_x;
    reg [9:0] pipe2_y;
    reg [4:0] pipe2_did;
    reg [7:0] pipe2_raw_index;
    reg [15:0] pipe2_xmap_mode;
    reg [1:0] pipe2_palette_bank;
    reg [7:0] pipe2_index;
    reg pipe2_flood;
    reg [23:0] pipe2_flood_rgb;
    reg [2:0] pipe2_resolver_class;
    reg [1:0] pipe2_front_bank;

    localparam [2:0]
        RESOLVER_FRONT          = 3'd0,
        RESOLVER_WINDOW_PALETTE = 3'd1,
        RESOLVER_WINDOW_FLOOD   = 3'd2,
        RESOLVER_OVERLAY        = 3'd3,
        RESOLVER_CURSOR         = 3'd4;
    reg [2:0] pipe_resolver_class;

    reg        resolver_frame_active;
    reg [31:0] resolver_cfg_generation;
    reg [31:0] resolver_frame_cfg_generation;
    reg        resolver_start_display_enable;
    reg        resolver_start_window_mode;
    reg [7:0]  resolver_start_rdmask;
    reg        resolver_window_seen_counting;
    reg [19:0] resolver_overlay_count;
    reg [19:0] resolver_flood_count;
    reg [19:0] resolver_cursor_count;
    reg [19:0] resolver_front_nonzero_count;
    reg [11:0] resolver_front_index_nonzero_count;
    reg [19:0] resolver_front_did0_count;
    reg [19:0] resolver_front_bank0_count;
    reg [19:0] resolver_front_bank1_count;
    reg [19:0] resolver_front_bank2_count;
    reg        resolver_final_cfg_stable;
    reg        resolver_final_display_enable;
    reg        resolver_final_window_mode;
    reg        resolver_final_window_seen;
    reg [7:0]  resolver_final_rdmask;
    reg [19:0] resolver_final_overlay_count;
    reg [19:0] resolver_final_flood_count;
    reg [19:0] resolver_final_cursor_count;
    reg [19:0] resolver_final_front_nonzero_count;
    reg [11:0] resolver_final_front_index_nonzero_count;
    reg [19:0] resolver_final_front_did0_count;
    reg [19:0] resolver_final_front_bank0_count;
    reg [19:0] resolver_final_front_bank1_count;
    reg [19:0] resolver_final_front_bank2_count;
    reg        resolver_finalize_pending;

    wire resolver_cfg_mutation =
        (cfg_dac_access && cfg_write && cfg_go &&
         (((cfg_sel == 3'd1) && (dac_rgb_phase == 2'd2)) ||
          (cfg_sel == 3'd2) ||
          ((cfg_sel == 3'd5) && (dac_rgb_phase == 2'd2)) ||
          (cfg_sel == 3'd6))) ||
        (cfg_vc1_access && cfg_write && cfg_go &&
         (((cfg_sel == 3'd0) && !vc1_cursor_position_mutation) ||
          vc1_word_commit || (cfg_sel == 3'd6)));

    function [19:0] resolver_sat_inc;
        input [19:0] value;
        input        enable;
        begin
            resolver_sat_inc = (enable && (value != 20'hfffff)) ?
                value + 20'd1 : value;
        end
    endfunction

    function [11:0] resolver_sat_inc12;
        input [11:0] value;
        input        enable;
        begin
            resolver_sat_inc12 = (enable && (value != 12'hfff)) ?
                value + 12'd1 : value;
        end
    endfunction

    wire resolver_physical_pixel = physical_enable && pipe2_de;
    wire resolver_frame_first = resolver_physical_pixel &&
        (pipe2_x == 10'd0) && (pipe2_y == 10'd0);
    wire resolver_frame_last = resolver_physical_pixel &&
        (pipe2_x == 10'd1023) && (pipe2_y == 10'd767);
    wire [19:0] resolver_overlay_base = resolver_frame_first ?
        20'd0 : resolver_overlay_count;
    wire [19:0] resolver_flood_base = resolver_frame_first ?
        20'd0 : resolver_flood_count;
    wire [19:0] resolver_cursor_base = resolver_frame_first ?
        20'd0 : resolver_cursor_count;
    wire [19:0] resolver_front_nonzero_base = resolver_frame_first ?
        20'd0 : resolver_front_nonzero_count;
    wire [11:0] resolver_front_index_nonzero_base = resolver_frame_first ?
        12'd0 : resolver_front_index_nonzero_count;
    wire [19:0] resolver_front_did0_base = resolver_frame_first ?
        20'd0 : resolver_front_did0_count;
    wire [19:0] resolver_front_bank0_base = resolver_frame_first ?
        20'd0 : resolver_front_bank0_count;
    wire [19:0] resolver_front_bank1_base = resolver_frame_first ?
        20'd0 : resolver_front_bank1_count;
    wire [19:0] resolver_front_bank2_base = resolver_frame_first ?
        20'd0 : resolver_front_bank2_count;
    wire [19:0] resolver_overlay_after_pixel = resolver_sat_inc(
        resolver_overlay_base,
        pipe2_resolver_class == RESOLVER_OVERLAY);
    wire [19:0] resolver_flood_after_pixel = resolver_sat_inc(
        resolver_flood_base,
        pipe2_resolver_class == RESOLVER_WINDOW_FLOOD);
    wire [19:0] resolver_cursor_after_pixel = resolver_sat_inc(
        resolver_cursor_base,
        pipe2_resolver_class == RESOLVER_CURSOR);
    wire [19:0] resolver_front_nonzero_after_pixel = resolver_sat_inc(
        resolver_front_nonzero_base,
        (pipe2_resolver_class == RESOLVER_FRONT) &&
        (palette_pixel_q != 24'd0));
    wire [11:0] resolver_front_index_nonzero_after_pixel =
        resolver_sat_inc12(resolver_front_index_nonzero_base,
            (pipe2_resolver_class == RESOLVER_FRONT) &&
            (pipe2_index != 8'd0));
    wire [19:0] resolver_front_did0_after_pixel = resolver_sat_inc(
        resolver_front_did0_base,
        (pipe2_resolver_class == RESOLVER_FRONT) &&
        (pipe2_did == 5'd0));
    wire [19:0] resolver_front_bank0_after_pixel = resolver_sat_inc(
        resolver_front_bank0_base,
        (pipe2_resolver_class == RESOLVER_FRONT) &&
        (pipe2_front_bank == 2'd0));
    wire [19:0] resolver_front_bank1_after_pixel = resolver_sat_inc(
        resolver_front_bank1_base,
        (pipe2_resolver_class == RESOLVER_FRONT) &&
        (pipe2_front_bank == 2'd1));
    wire [19:0] resolver_front_bank2_after_pixel = resolver_sat_inc(
        resolver_front_bank2_base,
        (pipe2_resolver_class == RESOLVER_FRONT) &&
        (pipe2_front_bank == 2'd2));
    wire resolver_window_seen_after_pixel =
        (resolver_frame_first ? 1'b0 : resolver_window_seen_counting) ||
        (pipe2_resolver_class == RESOLVER_WINDOW_PALETTE) ||
        (pipe2_resolver_class == RESOLVER_WINDOW_FLOOD);
    wire [31:0] resolver_cfg_generation_next =
        resolver_cfg_generation + (resolver_cfg_mutation ? 32'd1 : 32'd0);
    wire resolver_cfg_stable_after_pixel =
        resolver_frame_cfg_generation == resolver_cfg_generation_next;
    wire resolver_input_frame_start = sample_physical && sample_valid &&
        (sample_x == 10'd0) && (sample_y == 10'd0);

    wire palette_commit = cfg_valid && cfg_write && cfg_dac_access &&
        cfg_go && (cfg_sel == 3'd1) && (dac_rgb_phase == 2'd2);
    wire [9:0] palette_write_addr = {dac_palette_bank, dac_addr};
    wire [23:0] palette_write_data = {
        dac_rgb_stage_r, dac_rgb_stage_g, dac_mpu_color_write(cfg_wbyte)};
    wire [9:0] palette_cfg_addr =
        (cfg_valid && cfg_write && cfg_dac_access && cfg_go &&
         (cfg_sel == 3'd3)) ? {dac_palette_bank, cfg_wbyte} :
                              {dac_palette_bank, dac_addr};
    wire [9:0] palette_pixel_addr =
        {pipe_palette_bank, pipe_palette_index};

    always @(posedge clk) begin
        if (palette_commit)
            palette_mem[palette_write_addr] <= palette_write_data;
        else
            palette_cfg_q <= palette_mem[palette_cfg_addr];
        palette_pixel_q <= palette_mem[palette_pixel_addr];
    end

    wire pipe_cursor_cache_valid = pipe1_y[0] ?
        (cursor_odd_valid && (cursor_odd_line == pipe1_y)) :
        (cursor_even_valid && (cursor_even_line == pipe1_y));
    wire [31:0] pipe_cursor_cache_plane0 = pipe1_y[0] ?
        cursor_odd_plane0 : cursor_even_plane0;
    wire [31:0] pipe_cursor_cache_plane1 = pipe1_y[0] ?
        cursor_odd_plane1 : cursor_even_plane1;

    wire xmap_write = vc1_word_commit && (cfg_sel == 3'd1);

    // DID parity memories have a single registered read port. Coordinates and
    // raw planes are captured on the same edge as the selected DID value.
    always @(posedge clk) begin
        pipe0_de <= rst_n && sample_physical && sample_valid;
        pipe0_did_ready <= rst_n && sample_physical && did_sample_valid;
        pipe0_hsync <= !((h_ctr >= H_SYNC_START) &&
                          (h_ctr < H_SYNC_END));
        pipe0_vsync <= !((v_ctr >= V_SYNC_START) &&
                          (v_ctr < V_SYNC_END));
        if (rst_n && sample_physical && did_sample_valid)
            pipe0_did <= sample_y[0] ? did_odd[sample_x] :
                                       did_even[sample_x];
        else
            pipe0_did <= 5'h0;
        pipe0_x <= sample_x;
        pipe0_y <= sample_y;
        pipe0_front <= sample_physical ? sample_raw_index : 8'h0;
        pipe0_overlay <= sample_physical ? sample_overlay : 2'h0;
    end

    // One synchronous XMAP lookup per physical pixel. Port A shares MPU
    // completed host words/readback; port B carries the even-field mode.
    always @(posedge clk) begin
        if (xmap_write) begin
            xmap_mode_mem[vc1_addr[6:1]] <= vc1_word_data;
        end else begin
            xmap_cfg_q <= xmap_mode_mem[vc1_addr[6:1]];
        end
        xmap_pixel_q <= xmap_mode_mem[{1'b0, pipe0_did}];

        xmap0_de <= rst_n && pipe0_de;
        xmap0_did_ready <= rst_n && pipe0_did_ready;
        xmap0_hsync <= pipe0_hsync;
        xmap0_vsync <= pipe0_vsync;
        xmap0_did <= pipe0_did;
        xmap0_x <= pipe0_x;
        xmap0_y <= pipe0_y;
        xmap0_front <= pipe0_front;
        xmap0_overlay <= pipe0_overlay;
    end

    // Array payload writes live in a clock-only process. The asynchronous
    // reset below invalidates tags and control state, never memory contents.
    // This distinction is required for Vivado RAM inference.
    always @(posedge clk) begin
        if ((did_build_state == DID_FILL) && !did_segment_reached) begin
            if (did_build_buffer)
                did_odd[did_build_x] <= did_build_value;
            else
                did_even[did_build_x] <= did_build_value;
        end

        if (vc1_scan_rsp_valid && (did_build_state == DID_FETCH) &&
            (vc1_scan_rsp_tag == SCAN_SEG_LO)) begin
            did_segment_mem[did_build_entry[6:0]] <=
                {did_scan_hi, vc1_scan_byte[7:5], vc1_scan_byte[4:0]};
        end

        if (did_build_state == DID_LOAD) begin
            {did_active_segment_x, did_active_segment_value} <=
                did_segment_mem[did_build_entry[6:0]];
        end else if ((did_build_state == DID_FILL) && did_segment_reached &&
                     ((did_build_entry + 9'd1) < did_build_count)) begin
            {did_active_segment_x, did_active_segment_value} <=
                did_segment_mem[did_build_entry[6:0] + 7'd1];
        end

        if (cfg_valid && cfg_write && cfg_dac_access && cfg_go) begin
            if ((cfg_sel == 3'd1) && (dac_rgb_phase == 2'd2)) begin
                palette_write(dac_palette_bank, dac_addr, 2'd0,
                              dac_rgb_stage_r);
                palette_write(dac_palette_bank, dac_addr, 2'd1,
                              dac_rgb_stage_g);
                palette_write(dac_palette_bank, dac_addr, 2'd2,
                              dac_mpu_color_write(cfg_wbyte));
            end
            if ((cfg_sel == 3'd5) && (dac_rgb_phase == 2'd2)) begin
                bt_overlay_r[dac_addr[3:0]] <= dac_rgb_stage_r;
                bt_overlay_g[dac_addr[3:0]] <= dac_rgb_stage_g;
                bt_overlay_b[dac_addr[3:0]] <=
                    dac_mpu_color_write(cfg_wbyte);
            end
            if ((cfg_sel == 3'd6) && (dac_addr <= 8'h81))
                dac_window_regs[dac_addr] <=
                    dac_control_value(dac_addr, cfg_wbyte);
        end

`ifndef SYNTHESIS
        if (xmap_write) begin
            vc1_xmap[{vc1_addr[6:1], 1'b0}] <= vc1_mpu_msb;
            vc1_xmap[{vc1_addr[6:1], 1'b1}] <= cfg_wbyte;
        end
`endif
    end

    always @(*) begin
        pipe_xmap_mode = xmap_pixel_q;
        pipe_xmap_bank = pipe_xmap_mode[3:2];
        // LG1 exposes one CPU-addressable CI8 pixel plane.  Packed CI4 modes
        // select one of that byte's two nibbles, not a second full-width REX
        // framebuffer.  The independently captured back input is retained at
        // this boundary for packet provenance, but it is not display authority.
        pipe_xmap_raw = xmap0_front;
        pipe_xmap_index = xmap_ci_index(pipe_xmap_mode, pipe_xmap_raw);
        // Bt479 pixel inputs P[7:0] are bitwise ANDed with the pixel read
        // mask before addressing color-palette RAM.  Overlay and cursor
        // lookups use their independent inputs and override this result in
        // the following stage.
        pipe_xmap_index = pipe_xmap_index & dac_rdmask;
    end

    always @(*) begin
        pipe_display_index = pipe1_index;
        pipe_palette_bank = pipe1_bank;
        pipe_palette_index = pipe1_index;
        pipe_flood = 1'b0;
        pipe_flood_rgb = 24'h0;
        pipe_resolver_class = RESOLVER_FRONT;
        pipe_overlay_index = 8'h0;
        pipe_masked_overlay = pipe1_overlay & dac_overlay_mask;
        pipe_window_select = 8'h0;
        pipe_cursor_entry = 16'h0;
        pipe_cursor_x = 16'h0;
        pipe_cursor_y = 16'h0;
        pipe_cursor_origin_x = 17'sh0;
        pipe_cursor_origin_y = 17'sh0;
        pipe_cursor_dx = 17'sh0;
        pipe_cursor_dy = 17'sh0;
        pipe_cursor_cx = 5'h0;
        pipe_cursor_cy = 5'h0;
        pipe_cursor_bit = 5'h0;
        pipe_cursor_plane0 = 32'h0;
        pipe_cursor_plane1 = 32'h0;
        pipe_cursor_mode = vc1_regs[7'h26];
        pipe_cursor_index = 8'h0;
        pipe_cursor_bx = vc1_regs[7'h27];
        pipe_cursor_ly = {vc1_regs[7'h28][2:0], vc1_regs[7'h29]};
        pipe_cursor_special_offset = 19'h0;
        pipe_cursor_special_addr = 20'h0;
        pipe_cursor_special_byte = 8'h0;
        pipe_cursor_special_bit = 3'h0;

        if (dac_cmd1[0]) begin
            pipe_window_select = bt479_window_select(pipe1_x, pipe1_y);
            if (pipe_window_select[7]) begin
                pipe_palette_bank = pipe_window_select[1:0];
                pipe_resolver_class = RESOLVER_WINDOW_PALETTE;
                if (pipe_window_select[6]) begin
                    pipe_flood = 1'b1;
                    pipe_resolver_class = RESOLVER_WINDOW_FLOOD;
                    pipe_flood_rgb = {
                        bt_overlay_r[0], bt_overlay_g[0], bt_overlay_b[0]};
                end
            end
        end

        if (pipe_masked_overlay != 2'h0) begin
            // VC1's framebuffer overlay mux has a fixed four-entry OVRLAY
            // map (Indigo Family TR, Figure 25): palette bank 3, submap 1.
            // AUXVIDEO_MAP at 0x62 controls the optional live-video input;
            // it does not redirect the framebuffer overlay plane.
            pipe_overlay_index = {4'h1, 2'b00, pipe_masked_overlay};
            pipe_display_index = pipe_overlay_index;
            pipe_palette_bank = 2'd3;
            pipe_palette_index = pipe_overlay_index;
            pipe_flood = 1'b0;
            pipe_resolver_class = RESOLVER_OVERLAY;
        end

        if (vc1_sys_ctrl[5:4] == 2'b11) begin
            pipe_cursor_entry = {vc1_regs[7'h20], vc1_regs[7'h21]};
            pipe_cursor_x = cursor_frame_x;
            pipe_cursor_y = cursor_frame_y;
            pipe_cursor_origin_x = $signed({1'b0, pipe_cursor_x}) - 17'sd140;
            pipe_cursor_origin_y = $signed({1'b0, pipe_cursor_y}) - 17'sd39;
            pipe_cursor_dx = $signed({7'h0, pipe1_x}) - pipe_cursor_origin_x;
            pipe_cursor_dy = $signed({7'h0, pipe1_y}) - pipe_cursor_origin_y;
            // EXPRESS programs exact mode 3 for the VC1 crosshair. IRIS GL
            // defines one-pixel horizontal and vertical lines spanning the
            // screen, with cursor origin (15,15) at their intersection.
            if ((pipe_cursor_mode == 8'h03) &&
                (($signed({7'h0, pipe1_x}) ==
                  (pipe_cursor_origin_x + 17'sd15)) ||
                 ($signed({7'h0, pipe1_y}) ==
                  (pipe_cursor_origin_y + 17'sd15)))) begin
                pipe_cursor_index = {pipe_cursor_mode[5:0], 2'b01};
                pipe_display_index = pipe_cursor_index;
                pipe_palette_bank = pipe_cursor_mode[7:6];
                pipe_palette_index = pipe_cursor_index;
                pipe_flood = 1'b0;
                pipe_resolver_class = RESOLVER_CURSOR;
            end else if ((pipe_cursor_mode == 8'h01) &&
                // LIGHT programs exact mode 1 as a special one-bit cursor.
            // CUR_BX is bytes per row and CUR_LY is the row count.  Its XBM
            // data is uploaded without bit reversal, so bit zero is the
            // leftmost pixel in each byte.  Use a widened address sum so a
            // malformed footprint cannot wrap into low VC1 SRAM.
                (pipe_cursor_dx >= 17'sd0) &&
                (pipe_cursor_dy >= 17'sd0) &&
                (pipe_cursor_dy < $signed({6'h0, pipe_cursor_ly})) &&
                (pipe_cursor_dx[16:3] < {6'h0, pipe_cursor_bx})) begin
                pipe_cursor_special_offset =
                    (pipe_cursor_dy[10:0] * pipe_cursor_bx) +
                    {11'h0, pipe_cursor_dx[10:3]};
                pipe_cursor_special_addr =
                    {4'h0, pipe_cursor_entry} +
                    {1'b0, pipe_cursor_special_offset};
                pipe_cursor_special_bit = pipe_cursor_dx[2:0];
                if (pipe_cursor_special_addr <= 20'h0ffff) begin
`ifndef SYNTHESIS
                    pipe_cursor_special_byte =
                        vc1_ram_diag[pipe_cursor_special_addr[15:0]];
`endif
                    pipe_cursor_index = {
                        pipe_cursor_mode[5:0],
                        1'b0,
                        pipe_cursor_special_byte[pipe_cursor_special_bit]
                    };
                    pipe_display_index = pipe_cursor_index;
                    if (pipe_cursor_index[1:0] != 2'b00) begin
                        pipe_palette_bank = pipe_cursor_mode[7:6];
                        pipe_palette_index = pipe_cursor_index;
                        pipe_flood = 1'b0;
                        pipe_resolver_class = RESOLVER_CURSOR;
                    end
                end
            end else if ((pipe_cursor_mode != 8'h01) &&
                (pipe_cursor_mode != 8'h03) &&
                // LIGHT's normal cursor store is 32x32x2: each plane is
                // 32 rows of four big-endian bytes, and plane 1 begins 128
                // bytes after plane 0. Guard the complete 256-byte footprint
                // against wrapping the 64K VC1 SRAM address space.
                (pipe_cursor_entry <= 16'hff00) &&
                pipe_cursor_cache_valid &&
                (pipe_cursor_dx >= 17'sd0) &&
                (pipe_cursor_dx < 17'sd32) &&
                (pipe_cursor_dy >= 17'sd0) &&
                (pipe_cursor_dy < 17'sd32)) begin
                pipe_cursor_cx = pipe_cursor_dx[4:0];
                pipe_cursor_cy = pipe_cursor_dy[4:0];
                pipe_cursor_bit = 5'd31 - pipe_cursor_cx;
                pipe_cursor_plane0 = pipe_cursor_cache_plane0;
                pipe_cursor_plane1 = pipe_cursor_cache_plane1;
                pipe_cursor_index = {
                    pipe_cursor_mode[5:0],
                    pipe_cursor_plane1[pipe_cursor_bit],
                    pipe_cursor_plane0[pipe_cursor_bit]
                };
                pipe_display_index = pipe_cursor_index;
                if (pipe_cursor_index[1:0] != 2'b00) begin
                    // LIGHT documents CUR_MODE 0xc0 as map 3, submap 0.
                    // The two SRAM planes jointly supply the low palette code.
                    pipe_palette_bank = pipe_cursor_mode[7:6];
                    pipe_palette_index = pipe_cursor_index;
                    pipe_flood = 1'b0;
                    pipe_resolver_class = RESOLVER_CURSOR;
                end
            end
        end
    end

    always @(*) begin
        // LIGHT's Lg1TpBlank() controls the VC1 function bit in SYS_CTL:
        // clearing bit 7 turns the screen off and setting it restores video.
        // Keep timing and state decode alive while blanked, but suppress every
        // visible output just as the board's VC1 output gate does.
        dec_de = sample_valid && vc1_sys_ctrl[7];
        dec_did = 5'h0;
        dec_mode = 16'h0;
        dec_bank = 2'h0;
        dec_raw_index = sample_raw_index;
        dec_index = sample_raw_index;
        dec_r = 8'h0;
        dec_g = 8'h0;
        dec_b = 8'h0;
        dec_did_ep = {vc1_regs[7'h40], vc1_regs[7'h41]};
        dec_line_ptr = 16'h0;
        dec_line_count = 16'h0;
        dec_entry_addr = 16'h0;
        dec_seg_x = 11'h0;
        dec_x_hi = 8'h0;
        dec_did_lo = 8'h0;
        dec_overlay_index = 8'h0;
        dec_masked_overlay = sample_overlay & dac_overlay_mask;
        dec_window_select = 8'h0;
        dec_cursor_entry = 16'h0;
        dec_cursor_x = 16'h0;
        dec_cursor_y = 16'h0;
        dec_cursor_origin_x = 17'sh0;
        dec_cursor_origin_y = 17'sh0;
        dec_cursor_dx = 17'sh0;
        dec_cursor_dy = 17'sh0;
        dec_cursor_cx = 5'h0;
        dec_cursor_cy = 5'h0;
        dec_cursor_bit = 5'h0;
        dec_cursor_plane0 = 32'h0;
        dec_cursor_plane1 = 32'h0;
        dec_cursor_mode = vc1_regs[7'h26];
        dec_cursor_bx = vc1_regs[7'h27];
        dec_cursor_ly = {vc1_regs[7'h28][2:0], vc1_regs[7'h29]};
        dec_cursor_special_offset = 19'h0;
        dec_cursor_special_addr = 20'h0;
        dec_cursor_special_byte = 8'h0;
        dec_cursor_special_bit = 3'h0;

        if (sample_valid && ENABLE_DIAGNOSTIC_RESOLVER) begin
            if (sample_physical && did_sample_valid)
                dec_did = did_sample_value;
            else if (sample_physical)
                dec_did = 5'h0;

            if (!sample_physical) begin
            dec_line_ptr =
                vc1_ram_word(dec_did_ep + {5'h0, sample_y, 1'b0}) &
                16'h7fff;
            dec_line_count = vc1_ram_word(dec_line_ptr);

            if (did_line_count_valid(dec_line_ptr, dec_line_count)) begin
                for (dec_i = 0; dec_i < 128; dec_i = dec_i + 1) begin
                    if (dec_i < dec_line_count) begin
                        dec_entry_addr =
                            dec_line_ptr + 16'd2 + (dec_i[15:0] << 1);
`ifndef SYNTHESIS
                        dec_x_hi = vc1_ram_diag[dec_entry_addr];
                        dec_did_lo = vc1_ram_diag[dec_entry_addr + 16'd1];
`endif
                        dec_seg_x = {dec_x_hi, dec_did_lo[7:5]};
                        if (dec_seg_x <= {1'b0, sample_x})
                            dec_did = dec_did_lo[4:0];
                    end
                end
            end
            end

            // MODE_REG_*_E and MODE_REG_*_O are even/odd interlace-field
            // tables, not even/odd raster-line tables.  LG1's 1024x768
            // timing is non-interlaced, so every visible line uses the even
            // field table.  Indexing this with Y[0] blanked alternate lines
            // for visuals whose unused odd-field entry is zero (notably DID1).
            dec_mode = xmap_word(dec_did, 1'b0);
            dec_bank = dec_mode[3:2];
            // LG1 has one CPU-addressable CI8 pixel plane.  Packed CI4 modes
            // choose a nibble within it; no mode selects sample_raw_back.
            dec_raw_index = sample_raw_index;
            dec_index = xmap_ci_index(dec_mode, dec_raw_index);
            // Match the Bt479 fast-port address path: the pixel mask applies
            // to palette indices, not to the independent overlay inputs.
            dec_index = dec_index & dac_rdmask;
            dec_r = palette_component(dec_bank, dec_index, 2'd0);
            dec_g = palette_component(dec_bank, dec_index, 2'd1);
            dec_b = palette_component(dec_bank, dec_index, 2'd2);

            if (dac_cmd1[0]) begin
                dec_window_select = bt479_window_select(sample_x, sample_y);
                if (dec_window_select[7]) begin
                    dec_bank = dec_window_select[1:0];
                    dec_r = palette_component(dec_bank, dec_index, 2'd0);
                    dec_g = palette_component(dec_bank, dec_index, 2'd1);
                    dec_b = palette_component(dec_bank, dec_index, 2'd2);
                    if (dec_window_select[6]) begin
                        dec_r = bt_overlay_r[0];
                        dec_g = bt_overlay_g[0];
                        dec_b = bt_overlay_b[0];
                    end
                end
            end

            if (dec_masked_overlay != 2'h0) begin
                // Match the registered physical path: framebuffer overlay
                // pixels use fixed palette bank 3, OVRLAY submap 1.
                dec_bank = 2'd3;
                dec_overlay_index = {4'h1, 2'b00, dec_masked_overlay};
                dec_index = dec_overlay_index;
                dec_r = palette_component(dec_bank, dec_overlay_index, 2'd0);
                dec_g = palette_component(dec_bank, dec_overlay_index, 2'd1);
                dec_b = palette_component(dec_bank, dec_overlay_index, 2'd2);
            end

            if (vc1_sys_ctrl[5:4] == 2'b11) begin
                dec_cursor_entry = {vc1_regs[7'h20], vc1_regs[7'h21]};
                dec_cursor_x = {vc1_regs[7'h22], vc1_regs[7'h23]};
                dec_cursor_y = {vc1_regs[7'h24], vc1_regs[7'h25]};
                dec_cursor_origin_x = $signed({1'b0, dec_cursor_x}) -
                                      17'sd140;
                dec_cursor_origin_y = $signed({1'b0, dec_cursor_y}) -
                                      17'sd39;
                dec_cursor_dx = $signed({7'h0, sample_x}) -
                                dec_cursor_origin_x;
                dec_cursor_dy = $signed({7'h0, sample_y}) -
                                dec_cursor_origin_y;
                if ((dec_cursor_mode == 8'h03) &&
                    (($signed({7'h0, sample_x}) ==
                      (dec_cursor_origin_x + 17'sd15)) ||
                     ($signed({7'h0, sample_y}) ==
                      (dec_cursor_origin_y + 17'sd15)))) begin
                    dec_bank = dec_cursor_mode[7:6];
                    dec_index = {dec_cursor_mode[5:0], 2'b01};
                    dec_r = palette_component(dec_bank, dec_index, 2'd0);
                    dec_g = palette_component(dec_bank, dec_index, 2'd1);
                    dec_b = palette_component(dec_bank, dec_index, 2'd2);
                end else if ((dec_cursor_mode == 8'h01) &&
                    (dec_cursor_dx >= 17'sd0) &&
                    (dec_cursor_dy >= 17'sd0) &&
                    (dec_cursor_dy < $signed({6'h0, dec_cursor_ly})) &&
                    (dec_cursor_dx[16:3] < {6'h0, dec_cursor_bx})) begin
                    dec_cursor_special_offset =
                        (dec_cursor_dy[10:0] * dec_cursor_bx) +
                        {11'h0, dec_cursor_dx[10:3]};
                    dec_cursor_special_addr =
                        {4'h0, dec_cursor_entry} +
                        {1'b0, dec_cursor_special_offset};
                    dec_cursor_special_bit = dec_cursor_dx[2:0];
                    if (dec_cursor_special_addr <= 20'h0ffff) begin
`ifndef SYNTHESIS
                        dec_cursor_special_byte =
                            vc1_ram_diag[dec_cursor_special_addr[15:0]];
`endif
                        dec_bank = dec_cursor_mode[7:6];
                        dec_index = {
                            dec_cursor_mode[5:0],
                            1'b0,
                            dec_cursor_special_byte[dec_cursor_special_bit]
                        };
                        if (dec_index[1:0] != 2'b00) begin
                            dec_r = palette_component(dec_bank, dec_index, 2'd0);
                            dec_g = palette_component(dec_bank, dec_index, 2'd1);
                            dec_b = palette_component(dec_bank, dec_index, 2'd2);
                        end
                    end
                end else if ((dec_cursor_mode != 8'h01) &&
                    (dec_cursor_mode != 8'h03) &&
                    (dec_cursor_entry <= 16'hff00) &&
                    (dec_cursor_dx >= 17'sd0) &&
                    (dec_cursor_dx < 17'sd32) &&
                    (dec_cursor_dy >= 17'sd0) &&
                    (dec_cursor_dy < 17'sd32)) begin
                    dec_cursor_cx = dec_cursor_dx[4:0];
                    dec_cursor_cy = dec_cursor_dy[4:0];
                    dec_cursor_bit = 5'd31 - dec_cursor_cx;
                    dec_cursor_plane0 = vc1_ram_dword(
                        dec_cursor_entry + {9'h0, dec_cursor_cy, 2'b00});
                    dec_cursor_plane1 = vc1_ram_dword(
                        dec_cursor_entry + 16'd128 +
                        {9'h0, dec_cursor_cy, 2'b00});
                    dec_bank = dec_cursor_mode[7:6];
                    dec_index = {
                        dec_cursor_mode[5:0],
                        dec_cursor_plane1[dec_cursor_bit],
                        dec_cursor_plane0[dec_cursor_bit]
                    };
                    if (dec_index[1:0] != 2'b00) begin
                        dec_r = palette_component(dec_bank, dec_index, 2'd0);
                        dec_g = palette_component(dec_bank, dec_index, 2'd1);
                        dec_b = palette_component(dec_bank, dec_index, 2'd2);
                    end
                end
            end

            if (!vc1_sys_ctrl[7]) begin
                dec_r = 8'h0;
                dec_g = 8'h0;
                dec_b = 8'h0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_rbyte <= 8'h0;
            h_ctr <= 11'h0;
            // Start in vertical blank so the physical memory client can prime
            // line zero before video_de is first asserted.
            v_ctr <= V_VISIBLE;
            dac_rdmask <= 8'hff;
            dac_cmd0 <= 8'h0;
            dac_cmd1 <= 8'h0;
            dac_flood_lo <= 8'h0;
            dac_flood_hi <= 8'h0;
            dac_latch <= 8'h0;
            dac_addr <= 8'h0;
            dac_rgb_phase <= 2'h0;
            dac_rgb_stage_r <= 8'h0;
            dac_rgb_stage_g <= 8'h0;
            dac_rgb_read_r <= 8'h0;
            dac_rgb_read_g <= 8'h0;
            dac_rgb_read_b <= 8'h0;
            vc1_sys_ctrl <= 8'h0;
            vc1_addr <= 16'h0;
            vc1_mpu_msb <= 8'h0;
            did_even_valid <= 1'b0;
            did_odd_valid <= 1'b0;
            did_even_line <= 10'h0;
            did_odd_line <= 10'h0;
            did_build_state <= DID_IDLE;
            did_build_buffer <= 1'b0;
            did_build_line <= 10'h0;
            did_build_ptr <= 16'h0;
            did_build_count <= 9'h0;
            did_build_entry <= 9'h0;
            did_build_x <= 10'h0;
            did_build_value <= 5'h0;
            did_scan_hi <= 8'h0;
            did_span_valid <= 1'b0;
            did_span_first <= 10'd0;
            did_span_end <= 10'd0;
            did_span_ptr <= 16'd0;
            did_walk_row <= 10'd0;
            vc1_scan_ram_en <= 1'b0;
            vc1_scan_ram_addr <= 16'h0;
            vc1_scan_ram_tag <= 4'h0;
            vc1_scan_rsp_valid <= 1'b0;
            vc1_scan_rsp_tag <= 4'h0;
            vc1_scan_rsp_lane <= 2'h0;
            timing_scan_state <= TIMING_IDLE;
            timing_scan_active <= 1'b0;
            timing_rebuild_pending <= 1'b0;
            timing_start_armed <= 1'b0;
            timing_apply_pending <= 1'b0;
            timing_active <= 1'b0;
            timing_frame_lines <= V_TOTAL;
            timing_candidate_frame_lines <= 10'h0;
            timing_raw_line <= 10'h0;
            timing_active_start_raw <= 10'h0;
            timing_irq_raw <= 10'h0;
            timing_irq_line <= 10'h0;
            timing_line_clocks <= H_TOTAL;
            timing_line_duration <= 12'h0;
            timing_reference_duration <= 12'h0;
            timing_reference_duration_valid <= 1'b0;
            timing_duration_mismatch <= 1'b0;
            timing_frame_base <= 16'h0;
            timing_frame_entry_addr <= 16'h0;
            timing_frame_end <= 16'h0;
            timing_line_ptr <= 16'h0;
            timing_line_addr <= 16'h0;
            timing_line_min <= 16'hffff;
            timing_line_max <= 16'h0;
            timing_scan_hi <= 8'h0;
            timing_run_remaining <= 8'h0;
            timing_state_a <= 7'h0;
            timing_state_b <= 7'h0;
            timing_state_c <= 7'h0;
            timing_line_runs <= 6'h0;
            timing_line_active <= 1'b0;
            timing_active_start_seen <= 1'b0;
            timing_prev_c0_valid <= 1'b0;
            timing_prev_c0 <= 1'b1;
            timing_first_c0_valid <= 1'b0;
            timing_first_c0 <= 1'b1;
            timing_irq_seen <= 1'b0;
            timing_irq_multiple <= 1'b0;
            cursor_build_row_addr <= 16'h0;
            cursor_even_plane0 <= 32'h0;
            cursor_even_plane1 <= 32'h0;
            cursor_odd_plane0 <= 32'h0;
            cursor_odd_plane1 <= 32'h0;
            cursor_even_valid <= 1'b0;
            cursor_odd_valid <= 1'b0;
            cursor_even_line <= 10'h0;
            cursor_odd_line <= 10'h0;
            cursor_frame_x <= 16'h0;
            cursor_frame_y <= 16'h0;
            sample_de <= 1'b0;
            sample_did <= 5'h0;
            sample_mode <= 16'h0;
            sample_palette_bank <= 2'h0;
            sample_display_index <= 8'h0;
            sample_r <= 8'h0;
            sample_g <= 8'h0;
            sample_b <= 8'h0;
            video_de <= 1'b0;
            video_hsync <= 1'b1;
            video_vsync <= 1'b1;
            video_r <= 8'h0;
            video_g <= 8'h0;
            video_b <= 8'h0;
            video_valid <= 1'b0;
            video_frame_start <= 1'b0;
            video_did <= 5'h0;
            video_raw_index <= 8'h0;
            video_index <= 8'h0;
            video_xmap_mode <= 16'h0;
            video_palette_bank <= 2'h0;
            video_resolver_class <= RESOLVER_FRONT;
            video_x <= 10'h0;
            video_y <= 10'h0;
            pipe1_de <= 1'b0;
            pipe1_did_ready <= 1'b0;
            pipe1_hsync <= 1'b1;
            pipe1_vsync <= 1'b1;
            pipe1_x <= 10'h0;
            pipe1_y <= 10'h0;
            pipe1_overlay <= 2'h0;
            pipe1_did <= 5'h0;
            pipe1_raw_index <= 8'h0;
            pipe1_xmap_mode <= 16'h0;
            pipe1_bank <= 2'h0;
            pipe1_index <= 8'h0;
            pipe2_de <= 1'b0;
            pipe2_did_ready <= 1'b0;
            pipe2_hsync <= 1'b1;
            pipe2_vsync <= 1'b1;
            pipe2_x <= 10'h0;
            pipe2_y <= 10'h0;
            pipe2_did <= 5'h0;
            pipe2_raw_index <= 8'h0;
            pipe2_xmap_mode <= 16'h0;
            pipe2_palette_bank <= 2'h0;
            pipe2_index <= 8'h0;
            pipe2_flood <= 1'b0;
            pipe2_flood_rgb <= 24'h0;
            pipe2_resolver_class <= RESOLVER_FRONT;
            pipe2_front_bank <= 2'd0;
            resolver_frame_active <= 1'b0;
            resolver_cfg_generation <= 32'd0;
            resolver_frame_cfg_generation <= 32'd0;
            resolver_start_display_enable <= 1'b0;
            resolver_start_window_mode <= 1'b0;
            resolver_start_rdmask <= 8'd0;
            resolver_window_seen_counting <= 1'b0;
            resolver_overlay_count <= 20'd0;
            resolver_flood_count <= 20'd0;
            resolver_cursor_count <= 20'd0;
            resolver_front_nonzero_count <= 20'd0;
            resolver_front_index_nonzero_count <= 12'd0;
            resolver_front_did0_count <= 20'd0;
            resolver_front_bank0_count <= 20'd0;
            resolver_front_bank1_count <= 20'd0;
            resolver_front_bank2_count <= 20'd0;
            resolver_final_cfg_stable <= 1'b0;
            resolver_final_display_enable <= 1'b0;
            resolver_final_window_mode <= 1'b0;
            resolver_final_window_seen <= 1'b0;
            resolver_final_rdmask <= 8'd0;
            resolver_final_overlay_count <= 20'd0;
            resolver_final_flood_count <= 20'd0;
            resolver_final_cursor_count <= 20'd0;
            resolver_final_front_nonzero_count <= 20'd0;
            resolver_final_front_index_nonzero_count <= 12'd0;
            resolver_final_front_did0_count <= 20'd0;
            resolver_final_front_bank0_count <= 20'd0;
            resolver_final_front_bank1_count <= 20'd0;
            resolver_final_front_bank2_count <= 20'd0;
            resolver_finalize_pending <= 1'b0;
            resolver_telemetry <= 92'd0;
            resolver_detail <= 92'd0;
            resolver_frame_done <= 1'b0;
            palette_load_pending <= 1'b0;
            retrace_irq <= 1'b0;
        end else begin
            retrace_irq <= 1'b0;
            resolver_frame_done <= 1'b0;
            if (vc1_word_write && !vc1_addr[0])
                vc1_mpu_msb <= cfg_wbyte;
            if (resolver_cfg_mutation)
                resolver_cfg_generation <= resolver_cfg_generation + 32'd1;
            if (resolver_input_frame_start) begin
                resolver_frame_cfg_generation <=
                    resolver_cfg_generation;
                cursor_frame_x <= {vc1_regs[7'h22], vc1_regs[7'h23]};
                cursor_frame_y <= {vc1_regs[7'h24], vc1_regs[7'h25]};
            end
            if (resolver_finalize_pending) begin
                resolver_telemetry <= {
                    resolver_final_cfg_stable,
                    resolver_final_display_enable,
                    resolver_final_window_mode,
                    resolver_final_window_seen,
                    resolver_final_rdmask,
                    resolver_final_overlay_count,
                    resolver_final_flood_count,
                    resolver_final_cursor_count,
                    resolver_final_front_nonzero_count
                };
                resolver_detail <= {
                    resolver_final_front_index_nonzero_count,
                    resolver_final_front_did0_count,
                    resolver_final_front_bank0_count,
                    resolver_final_front_bank1_count,
                    resolver_final_front_bank2_count
                };
                resolver_frame_done <= 1'b1;
                resolver_finalize_pending <= 1'b0;
            end
            palette_load_pending <= cfg_valid && cfg_write &&
                cfg_dac_access && cfg_go && (cfg_sel == 3'd3);
`ifdef SYNTHESIS
            if (palette_load_pending) begin
                dac_rgb_read_r <= palette_cfg_q[23:16];
                dac_rgb_read_g <= palette_cfg_q[15:8];
                dac_rgb_read_b <= palette_cfg_q[7:0];
            end
`endif
            vc1_scan_rsp_valid <= vc1_scan_ram_en;
            vc1_scan_rsp_tag <= vc1_scan_ram_tag;
            vc1_scan_rsp_lane <= vc1_scan_ram_addr[1:0];
            vc1_scan_ram_en <= 1'b0;
            if (h_ctr == (timing_horizontal_total - 11'd1)) begin
                h_ctr <= 11'h0;
                if (timing_apply_pending && !vc1_sys_ctrl[1]) begin
                    // Apply a complete decode only on a raster-line boundary.
                    // Public coordinates stay visible-first, while execution
                    // begins at the programmed frame table's raw line zero.
                    v_ctr <= (timing_active_start_raw == 10'h0) ? 10'h0 :
                        timing_frame_lines - timing_active_start_raw;
                    timing_apply_pending <= 1'b0;
                    timing_active <= 1'b1;
                    // Timing channels are inactive-high while VTG is stopped.
                    // A low first C0 state therefore asserts VERT_INT_REX_N as
                    // soon as raw line zero is applied.
                    if (!vc1_sys_ctrl[0] && !timing_first_c0)
                        retrace_irq <= 1'b1;
                end else if (timing_active && !vc1_sys_ctrl[1]) begin
                    if (v_ctr == (timing_frame_lines - 10'd1))
                        v_ctr <= 10'h0;
                    else
                        v_ctr <= v_ctr + 10'd1;

                    // VC1_INTERRUPT and VC1_VTG are active-low system-control
                    // bits 0 and 1. C0's falling
                    // edge is normalized into the visible-first Y domain by
                    // the table bootstrap walk below.
                    if (!vc1_sys_ctrl[0] &&
                        (((v_ctr == (timing_frame_lines - 10'd1)) &&
                          (timing_irq_line == 10'h0)) ||
                         ((v_ctr != (timing_frame_lines - 10'd1)) &&
                          ((v_ctr + 10'd1) == timing_irq_line))))
                        retrace_irq <= 1'b1;
                end else if (v_ctr == (V_TOTAL - 10'd1)) begin
                    // Before a valid table is decoded, preserve the existing
                    // presentation raster but never fabricate a VC1 event.
                    v_ctr <= 10'h0;
                end else begin
                    v_ctr <= v_ctr + 10'd1;
                end
            end else begin
                h_ctr <= h_ctr + 11'd1;
            end

            // Start/restart only after the DID scan client has released the
            // single read port. Software loads the timing tables with VTG off and
            // explicitly arms this walk when it writes SYS_CTL with VTG low.
            if (timing_start_armed && timing_rebuild_pending &&
                !timing_scan_active && (did_build_state == DID_IDLE) &&
                !vc1_scan_ram_en && !vc1_scan_rsp_valid) begin
                timing_scan_active <= 1'b1;
                timing_scan_state <= TIMING_HEADER_HI;
                timing_active <= 1'b0;
                timing_apply_pending <= 1'b0;
                timing_frame_base <=
                    {1'b0, vc1_regs[7'h00][6:0], vc1_regs[7'h01]};
                timing_frame_entry_addr <=
                    {1'b0, vc1_regs[7'h00][6:0], vc1_regs[7'h01]} +
                    16'd2;
                timing_frame_end <=
                    {1'b0, vc1_regs[7'h00][6:0], vc1_regs[7'h01]} +
                    16'd1;
                timing_candidate_frame_lines <= 10'h0;
                timing_raw_line <= 10'h0;
                timing_active_start_raw <= 10'h0;
                timing_irq_raw <= 10'h0;
                timing_line_duration <= 12'h0;
                timing_reference_duration <= 12'h0;
                timing_reference_duration_valid <= 1'b0;
                timing_duration_mismatch <= 1'b0;
                timing_line_min <= 16'hffff;
                timing_line_max <= 16'h0;
                timing_state_a <= 7'h0;
                timing_state_b <= 7'h0;
                timing_state_c <= 7'h0;
                timing_active_start_seen <= 1'b0;
                timing_prev_c0_valid <= 1'b0;
                timing_prev_c0 <= 1'b1;
                timing_first_c0_valid <= 1'b0;
                timing_first_c0 <= 1'b1;
                timing_irq_seen <= 1'b0;
                timing_irq_multiple <= 1'b0;
                vc1_scan_ram_en <= 1'b1;
                vc1_scan_ram_addr <=
                    {1'b0, vc1_regs[7'h00][6:0], vc1_regs[7'h01]};
                vc1_scan_ram_tag <= SCAN_TIMING;
            end

            if (timing_scan_active && vc1_scan_rsp_valid &&
                (vc1_scan_rsp_tag == SCAN_TIMING)) begin
                case (timing_scan_state)
                    TIMING_HEADER_HI: begin
                        timing_scan_hi <= vc1_scan_byte;
                        timing_scan_state <= TIMING_HEADER_LO;
                        vc1_scan_ram_en <= 1'b1;
                        vc1_scan_ram_addr <= timing_frame_base + 16'd1;
                        vc1_scan_ram_tag <= SCAN_TIMING;
                    end
                    TIMING_HEADER_LO: begin
                        if (({timing_scan_hi, vc1_scan_byte} == 16'h0) ||
                            ({timing_scan_hi, vc1_scan_byte} > 16'd1023)) begin
                            timing_scan_active <= 1'b0;
                            timing_scan_state <= TIMING_IDLE;
                            timing_rebuild_pending <= 1'b0;
                            timing_start_armed <= 1'b0;
                            timing_active <= 1'b0;
                            timing_apply_pending <= 1'b0;
                        end else begin
                            timing_candidate_frame_lines <=
                                {timing_scan_hi[1:0], vc1_scan_byte};
                            timing_scan_state <= TIMING_RUN_PTR_HI;
                            vc1_scan_ram_en <= 1'b1;
                            vc1_scan_ram_addr <= timing_frame_base + 16'd2;
                            vc1_scan_ram_tag <= SCAN_TIMING;
                        end
                    end
                    TIMING_RUN_PTR_HI: begin
                        timing_scan_hi <= vc1_scan_byte;
                        timing_scan_state <= TIMING_RUN_PTR_LO;
                        vc1_scan_ram_en <= 1'b1;
                        vc1_scan_ram_addr <= timing_frame_entry_addr + 16'd1;
                        vc1_scan_ram_tag <= SCAN_TIMING;
                    end
                    TIMING_RUN_PTR_LO: begin
                        timing_line_ptr <=
                            {timing_scan_hi, vc1_scan_byte} & 16'h7fff;
                        timing_scan_state <= TIMING_RUN_COUNT;
                        vc1_scan_ram_en <= 1'b1;
                        vc1_scan_ram_addr <= timing_frame_entry_addr + 16'd2;
                        vc1_scan_ram_tag <= SCAN_TIMING;
                    end
                    TIMING_RUN_COUNT: begin
                        timing_frame_end <= timing_frame_entry_addr + 16'd2;
                        if ((vc1_scan_byte == 8'h0) ||
                            ({1'b0, timing_raw_line} +
                             {3'h0, vc1_scan_byte} >
                             {1'b0, timing_candidate_frame_lines})) begin
                            timing_scan_active <= 1'b0;
                            timing_scan_state <= TIMING_IDLE;
                            timing_rebuild_pending <= 1'b0;
                            timing_start_armed <= 1'b0;
                            timing_active <= 1'b0;
                            timing_apply_pending <= 1'b0;
                        end else begin
                            timing_run_remaining <= vc1_scan_byte;
                            timing_line_addr <= timing_line_ptr;
                            timing_line_min <=
                                (timing_line_ptr < timing_line_min) ?
                                timing_line_ptr : timing_line_min;
                            timing_line_max <=
                                (timing_line_ptr > timing_line_max) ?
                                timing_line_ptr : timing_line_max;
                            timing_line_runs <= 6'h0;
                            timing_line_active <= 1'b0;
                            timing_line_duration <= 12'h0;
                            timing_scan_state <= TIMING_LINE_A;
                            vc1_scan_ram_en <= 1'b1;
                            vc1_scan_ram_addr <= timing_line_ptr;
                            vc1_scan_ram_tag <= SCAN_TIMING;
                        end
                    end
                    TIMING_LINE_A: begin
                        if (timing_line_runs >= 6'd32) begin
                            timing_scan_active <= 1'b0;
                            timing_scan_state <= TIMING_IDLE;
                            timing_rebuild_pending <= 1'b0;
                            timing_start_armed <= 1'b0;
                            timing_active <= 1'b0;
                            timing_apply_pending <= 1'b0;
                        end else begin
                            timing_state_a <= vc1_scan_byte[6:0];
                            timing_line_runs <= timing_line_runs + 6'd1;
                            timing_line_max <=
                                (timing_line_addr > timing_line_max) ?
                                timing_line_addr : timing_line_max;
                            timing_line_addr <= timing_line_addr + 16'd1;
                            timing_scan_state <= vc1_scan_byte[7] ?
                                TIMING_DURATION : TIMING_LINE_B;
                            vc1_scan_ram_en <= 1'b1;
                            vc1_scan_ram_addr <= timing_line_addr + 16'd1;
                            vc1_scan_ram_tag <= SCAN_TIMING;
                        end
                    end
                    TIMING_LINE_B: begin
                        timing_state_b <= vc1_scan_byte[6:0];
                        timing_line_max <=
                            (timing_line_addr > timing_line_max) ?
                            timing_line_addr : timing_line_max;
                        timing_line_addr <= timing_line_addr + 16'd1;
                        timing_scan_state <= vc1_scan_byte[7] ?
                            TIMING_DURATION : TIMING_LINE_C;
                        vc1_scan_ram_en <= 1'b1;
                        vc1_scan_ram_addr <= timing_line_addr + 16'd1;
                        vc1_scan_ram_tag <= SCAN_TIMING;
                    end
                    TIMING_LINE_C: begin
                        timing_state_c <= vc1_scan_byte[6:0];
                        timing_line_max <=
                            (timing_line_addr > timing_line_max) ?
                            timing_line_addr : timing_line_max;
                        timing_line_addr <= timing_line_addr + 16'd1;
                        timing_scan_state <= TIMING_DURATION;
                        vc1_scan_ram_en <= 1'b1;
                        vc1_scan_ram_addr <= timing_line_addr + 16'd1;
                        vc1_scan_ram_tag <= SCAN_TIMING;
                    end
                    TIMING_DURATION: begin
                        timing_line_max <=
                            (timing_line_addr > timing_line_max) ?
                            timing_line_addr : timing_line_max;
                        if (vc1_scan_byte[6:0] == 7'h0) begin
                            timing_scan_active <= 1'b0;
                            timing_scan_state <= TIMING_IDLE;
                            timing_rebuild_pending <= 1'b0;
                            timing_start_armed <= 1'b0;
                            timing_active <= 1'b0;
                            timing_apply_pending <= 1'b0;
                        end else begin
                            timing_line_duration <= timing_line_duration +
                                {5'h0, vc1_scan_byte[6:0]};
                            if (timing_state_a[0])
                                timing_line_active <= 1'b1;
                            if (!timing_prev_c0_valid) begin
                                timing_prev_c0_valid <= 1'b1;
                                timing_prev_c0 <= timing_state_c[0];
                                timing_first_c0_valid <= 1'b1;
                                timing_first_c0 <= timing_state_c[0];
                            end else begin
                                if (timing_prev_c0 && !timing_state_c[0]) begin
                                    if (timing_irq_seen)
                                        timing_irq_multiple <= 1'b1;
                                    else begin
                                        timing_irq_seen <= 1'b1;
                                        timing_irq_raw <= timing_raw_line;
                                    end
                                end
                                timing_prev_c0 <= timing_state_c[0];
                            end
                            timing_line_addr <= timing_line_addr + 16'd1;
                            timing_scan_state <= vc1_scan_byte[7] ?
                                TIMING_NEXT_ZERO : TIMING_LINE_A;
                            vc1_scan_ram_en <= 1'b1;
                            vc1_scan_ram_addr <= timing_line_addr + 16'd1;
                            vc1_scan_ram_tag <= SCAN_TIMING;
                        end
                    end
                    TIMING_NEXT_ZERO: begin
                        timing_line_max <=
                            (timing_line_addr > timing_line_max) ?
                            timing_line_addr : timing_line_max;
                        if (vc1_scan_byte != 8'h0) begin
                            timing_scan_active <= 1'b0;
                            timing_scan_state <= TIMING_IDLE;
                            timing_rebuild_pending <= 1'b0;
                            timing_start_armed <= 1'b0;
                            timing_active <= 1'b0;
                            timing_apply_pending <= 1'b0;
                        end else begin
                            timing_line_addr <= timing_line_addr + 16'd1;
                            timing_scan_state <= TIMING_NEXT_HI;
                            vc1_scan_ram_en <= 1'b1;
                            vc1_scan_ram_addr <= timing_line_addr + 16'd1;
                            vc1_scan_ram_tag <= SCAN_TIMING;
                        end
                    end
                    TIMING_NEXT_HI: begin
                        timing_scan_hi <= vc1_scan_byte;
                        timing_line_max <=
                            (timing_line_addr > timing_line_max) ?
                            timing_line_addr : timing_line_max;
                        timing_line_addr <= timing_line_addr + 16'd1;
                        timing_scan_state <= TIMING_NEXT_LO;
                        vc1_scan_ram_en <= 1'b1;
                        vc1_scan_ram_addr <= timing_line_addr + 16'd1;
                        vc1_scan_ram_tag <= SCAN_TIMING;
                    end
                    TIMING_NEXT_LO: begin
                        timing_line_max <=
                            (timing_line_addr > timing_line_max) ?
                            timing_line_addr : timing_line_max;
                        if (!timing_active_start_seen && timing_line_active) begin
                            timing_active_start_seen <= 1'b1;
                            timing_active_start_raw <= timing_raw_line;
                        end

                        // Every line program in one accepted frame must have
                        // the same nonzero duration.  The line table controls
                        // cadence; a malformed or mixed-duration table must
                        // fail closed rather than partly changing the raster.
                        if (!timing_reference_duration_valid) begin
                            timing_reference_duration <= timing_line_duration;
                            timing_reference_duration_valid <= 1'b1;
                        end else if (timing_line_duration !=
                                     timing_reference_duration) begin
                            timing_duration_mismatch <= 1'b1;
                        end

                        if ((timing_raw_line + 10'd1) ==
                            timing_candidate_frame_lines) begin
                            // Close the circular frame before accepting it: C0
                            // may fall only on the last-line -> first-line edge.
                            if (timing_irq_multiple ||
                                !timing_first_c0_valid ||
                                !timing_reference_duration_valid ||
                                (timing_reference_duration == 12'h0) ||
                                (timing_reference_duration > 12'd409) ||
                                timing_duration_mismatch ||
                                (timing_line_duration !=
                                 timing_reference_duration) ||
                                !(timing_active_start_seen ||
                                  timing_line_active) ||
                                ((timing_irq_seen) && timing_prev_c0 &&
                                 !timing_first_c0) ||
                                (!(timing_irq_seen) &&
                                 !(timing_prev_c0 && !timing_first_c0))) begin
                                timing_active <= 1'b0;
                                timing_apply_pending <= 1'b0;
                            end else begin
                                timing_frame_lines <=
                                    timing_candidate_frame_lines;
                                timing_line_clocks <=
                                    {timing_reference_duration[8:0], 2'b00} +
                                    {2'b00, timing_reference_duration[8:0]};
                                if (timing_irq_seen) begin
                                    if (timing_active_start_seen) begin
                                        if (timing_irq_raw >=
                                            timing_active_start_raw)
                                            timing_irq_line <=
                                                timing_irq_raw -
                                                timing_active_start_raw;
                                        else
                                            timing_irq_line <=
                                                timing_irq_raw +
                                                timing_candidate_frame_lines -
                                                timing_active_start_raw;
                                    end else begin
                                        if (timing_irq_raw >= timing_raw_line)
                                            timing_irq_line <=
                                                timing_irq_raw - timing_raw_line;
                                        else
                                            timing_irq_line <=
                                                timing_irq_raw +
                                                timing_candidate_frame_lines -
                                                timing_raw_line;
                                    end
                                end else begin
                                    // The sole falling edge is the circular
                                    // wrap into raw line zero.
                                    if (timing_active_start_seen)
                                        timing_irq_line <=
                                            (timing_active_start_raw == 10'h0) ?
                                            10'h0 :
                                            timing_candidate_frame_lines -
                                            timing_active_start_raw;
                                    else
                                        timing_irq_line <=
                                            (timing_raw_line == 10'h0) ?
                                            10'h0 :
                                            timing_candidate_frame_lines -
                                            timing_raw_line;
                                end
                                timing_apply_pending <= 1'b1;
                            end
                            timing_scan_active <= 1'b0;
                            timing_scan_state <= TIMING_IDLE;
                            timing_rebuild_pending <= 1'b0;
                            timing_start_armed <= 1'b0;
                        end else begin
                            timing_raw_line <= timing_raw_line + 10'd1;
                            timing_line_runs <= 6'h0;
                            timing_line_active <= 1'b0;
                            timing_line_duration <= 12'h0;
                            if (timing_run_remaining > 8'd1) begin
                                timing_run_remaining <=
                                    timing_run_remaining - 8'd1;
                                timing_line_ptr <=
                                    {timing_scan_hi, vc1_scan_byte} & 16'h7fff;
                                timing_line_addr <=
                                    {timing_scan_hi, vc1_scan_byte} & 16'h7fff;
                                timing_line_min <=
                                    (({timing_scan_hi, vc1_scan_byte} &
                                      16'h7fff) < timing_line_min) ?
                                    ({timing_scan_hi, vc1_scan_byte} &
                                     16'h7fff) : timing_line_min;
                                timing_scan_state <= TIMING_LINE_A;
                                vc1_scan_ram_en <= 1'b1;
                                vc1_scan_ram_addr <=
                                    {timing_scan_hi, vc1_scan_byte} & 16'h7fff;
                                vc1_scan_ram_tag <= SCAN_TIMING;
                            end else begin
                                timing_frame_entry_addr <=
                                    timing_frame_entry_addr + 16'd3;
                                timing_scan_state <= TIMING_RUN_PTR_HI;
                                vc1_scan_ram_en <= 1'b1;
                                vc1_scan_ram_addr <=
                                    timing_frame_entry_addr + 16'd3;
                                vc1_scan_ram_tag <= SCAN_TIMING;
                            end
                        end
                    end
                    default: begin
                        timing_scan_active <= 1'b0;
                        timing_scan_state <= TIMING_IDLE;
                        timing_rebuild_pending <= 1'b0;
                        timing_start_armed <= 1'b0;
                        timing_active <= 1'b0;
                        timing_apply_pending <= 1'b0;
                    end
                endcase
            end

            pipe1_de <= xmap0_de;
            pipe1_did_ready <= xmap0_did_ready;
            pipe1_hsync <= xmap0_hsync;
            pipe1_vsync <= xmap0_vsync;
            pipe1_x <= xmap0_x;
            pipe1_y <= xmap0_y;
            pipe1_overlay <= xmap0_overlay;
            pipe1_did <= xmap0_did;
            pipe1_raw_index <= xmap0_front;
            pipe1_xmap_mode <= pipe_xmap_mode;
            pipe1_bank <= pipe_xmap_bank;
            pipe1_index <= pipe_xmap_index;

            pipe2_de <= pipe1_de;
            pipe2_did_ready <= pipe1_did_ready;
            pipe2_hsync <= pipe1_hsync;
            pipe2_vsync <= pipe1_vsync;
            pipe2_x <= pipe1_x;
            pipe2_y <= pipe1_y;
            pipe2_did <= pipe1_did;
            pipe2_raw_index <= pipe1_raw_index;
            pipe2_xmap_mode <= pipe1_xmap_mode;
            pipe2_palette_bank <= pipe_palette_bank;
            pipe2_index <= pipe_display_index;
            pipe2_flood <= pipe_flood;
            pipe2_flood_rgb <= pipe_flood_rgb;
            pipe2_resolver_class <= pipe_resolver_class;
            pipe2_front_bank <= pipe1_bank;

            if (resolver_physical_pixel &&
                (resolver_frame_active || resolver_frame_first)) begin
                resolver_frame_active <= !resolver_frame_last;
                resolver_window_seen_counting <=
                    resolver_window_seen_after_pixel;
                resolver_overlay_count <= resolver_overlay_after_pixel;
                resolver_flood_count <= resolver_flood_after_pixel;
                resolver_cursor_count <= resolver_cursor_after_pixel;
                resolver_front_nonzero_count <=
                    resolver_front_nonzero_after_pixel;
                resolver_front_index_nonzero_count <=
                    resolver_front_index_nonzero_after_pixel;
                resolver_front_did0_count <=
                    resolver_front_did0_after_pixel;
                resolver_front_bank0_count <=
                    resolver_front_bank0_after_pixel;
                resolver_front_bank1_count <=
                    resolver_front_bank1_after_pixel;
                resolver_front_bank2_count <=
                    resolver_front_bank2_after_pixel;
                if (resolver_frame_first) begin
                    resolver_start_display_enable <= vc1_sys_ctrl[7];
                    resolver_start_window_mode <= dac_cmd1[0];
                    resolver_start_rdmask <= dac_rdmask;
                end
                if (resolver_frame_last) begin
                    resolver_final_cfg_stable <=
                        resolver_cfg_stable_after_pixel;
                    resolver_final_display_enable <=
                        resolver_frame_first ? vc1_sys_ctrl[7] :
                                               resolver_start_display_enable;
                    resolver_final_window_mode <=
                        resolver_frame_first ? dac_cmd1[0] :
                                               resolver_start_window_mode;
                    resolver_final_window_seen <=
                        resolver_window_seen_after_pixel;
                    resolver_final_rdmask <=
                        resolver_frame_first ? dac_rdmask :
                                               resolver_start_rdmask;
                    resolver_final_overlay_count <=
                        resolver_overlay_after_pixel;
                    resolver_final_flood_count <=
                        resolver_flood_after_pixel;
                    resolver_final_cursor_count <=
                        resolver_cursor_after_pixel;
                    resolver_final_front_nonzero_count <=
                        resolver_front_nonzero_after_pixel;
                    resolver_final_front_index_nonzero_count <=
                        resolver_front_index_nonzero_after_pixel;
                    resolver_final_front_did0_count <=
                        resolver_front_did0_after_pixel;
                    resolver_final_front_bank0_count <=
                        resolver_front_bank0_after_pixel;
                    resolver_final_front_bank1_count <=
                        resolver_front_bank1_after_pixel;
                    resolver_final_front_bank2_count <=
                        resolver_front_bank2_after_pixel;
                    resolver_finalize_pending <= 1'b1;
                end
            end

            if (physical_enable) begin
                // A live VC1/XMAP/Bt479 update can change the resolver result
                // part-way through the active raster.  Keep computing the
                // physical pixel stream for diagnostics, but make the dirty
                // remainder invalid so the HDMI frame store cannot admit a
                // coordinate-complete mixed-generation frame.  The next
                // frame snapshots the new generation. Its DID must also have
                // been ready for this exact source pixel: a late-blank update
                // can leave row zero unbuilt despite a stable generation.
                // Carry that readiness with the pixel, not the current cache
                // tags. A missing line leaves this frame incomplete so the
                // display retains the preceding complete frame. DE and the
                // diagnostic pixel/coordinate stream remain continuous.
                video_valid <= pipe2_de && pipe2_did_ready &&
                               resolver_cfg_stable_after_pixel;
                video_frame_start <= pipe2_de && (pipe2_x == 10'd0) &&
                                     (pipe2_y == 10'd0);
                video_de <= pipe2_de && vc1_sys_ctrl[7];
                video_hsync <= pipe2_hsync;
                video_vsync <= pipe2_vsync;
                video_r <= (pipe2_de && vc1_sys_ctrl[7]) ?
                    (pipe2_flood ? pipe2_flood_rgb[23:16] :
                                   palette_pixel_q[23:16]) : 8'h0;
                video_g <= (pipe2_de && vc1_sys_ctrl[7]) ?
                    (pipe2_flood ? pipe2_flood_rgb[15:8] :
                                   palette_pixel_q[15:8]) : 8'h0;
                video_b <= (pipe2_de && vc1_sys_ctrl[7]) ?
                    (pipe2_flood ? pipe2_flood_rgb[7:0] :
                                   palette_pixel_q[7:0]) : 8'h0;
                video_x <= pipe2_x;
                video_y <= pipe2_y;
                video_did <= pipe2_did;
                video_raw_index <= pipe2_raw_index;
                video_index <= pipe2_index;
                video_xmap_mode <= pipe2_xmap_mode;
                video_palette_bank <= pipe2_palette_bank;
                video_resolver_class <= pipe2_resolver_class;
            end else begin
                video_valid <= sample_valid;
                video_frame_start <= sample_valid && (sample_x == 10'd0) &&
                                     (sample_y == 10'd0);
                video_de <= (h_ctr < H_VISIBLE) && (v_ctr < V_VISIBLE) &&
                            vc1_sys_ctrl[7];
                video_hsync <= !((h_ctr >= H_SYNC_START) &&
                                  (h_ctr < H_SYNC_END));
                video_vsync <= !((v_ctr >= V_SYNC_START) &&
                                  (v_ctr < V_SYNC_END));
                video_r <= (sample_valid && vc1_sys_ctrl[7]) ?
                           dec_r : 8'h0;
                video_g <= (sample_valid && vc1_sys_ctrl[7]) ?
                           dec_g : 8'h0;
                video_b <= (sample_valid && vc1_sys_ctrl[7]) ?
                           dec_b : 8'h0;
                video_x <= h_ctr[9:0];
                video_y <= v_ctr;
                video_did <= dec_did;
                video_raw_index <= sample_raw_index;
                video_index <= dec_index;
                video_xmap_mode <= dec_mode;
                video_palette_bank <= dec_bank;
                video_resolver_class <= RESOLVER_FRONT;
            end

            sample_de <= dec_de;
            sample_did <= dec_did;
            sample_mode <= dec_mode;
            sample_palette_bank <= dec_bank;
            sample_display_index <= dec_index;
            sample_r <= dec_r;
            sample_g <= dec_g;
            sample_b <= dec_b;

            case (did_build_state)
                DID_IDLE: begin
                    if ((h_ctr == 11'd0) && !did_target_valid &&
                        !vc1_scan_ram_en && !vc1_scan_rsp_valid &&
                        !timing_scan_active &&
                        !(timing_start_armed && timing_rebuild_pending) &&
                        !vc1_did_cache_mutation) begin
                        did_build_buffer <= did_target_buffer;
                        did_build_line <= did_target_line;
                        did_build_state <= DID_FETCH;
                        did_walk_row <= did_walk_start;
                        if (!did_span_forward)
                            did_span_valid <= 1'b0;
                        vc1_scan_ram_en <= 1'b1;
                        cursor_build_row_addr <= cursor_target_row_addr;
                        if (cursor_target_normal) begin
                            vc1_scan_ram_addr <= cursor_target_row_addr;
                            vc1_scan_ram_tag <= SCAN_CURSOR_P0;
                        end else if (did_target_span_hit) begin
                            did_build_ptr <= did_span_ptr;
                            vc1_scan_ram_addr <= did_span_ptr;
                            vc1_scan_ram_tag <= SCAN_COUNT_HI;
                        end else begin
                            vc1_scan_ram_addr <=
                                {vc1_regs[7'h40], vc1_regs[7'h41]} +
                                {5'h0, did_walk_start, 1'b0};
                            vc1_scan_ram_tag <= SCAN_PTR_HI;
                        end
                        if (did_target_buffer) begin
                            did_odd_valid <= 1'b0;
                            cursor_odd_valid <= 1'b0;
                            cursor_odd_plane0 <= 32'h0;
                            cursor_odd_plane1 <= 32'h0;
                        end else begin
                            did_even_valid <= 1'b0;
                            cursor_even_valid <= 1'b0;
                            cursor_even_plane0 <= 32'h0;
                            cursor_even_plane1 <= 32'h0;
                        end
                    end
                end
                DID_FETCH: begin end
                DID_LOAD: begin
                    // did_active_segment_* captures the synchronous memory
                    // output on this edge and is consumed starting next cycle.
                    did_build_state <= DID_FILL;
                end
                DID_FILL: begin
                    if (did_segment_reached) begin
                        did_build_value <= did_active_segment_value;
                        did_build_entry <= did_build_entry + 9'd1;
                    end else begin
                        if (did_build_x == 10'd1023) begin
                            did_build_state <= DID_IDLE;
                            if (did_build_buffer) begin
                                did_odd_valid <= 1'b1;
                                did_odd_line <= did_build_line;
                            end else begin
                                did_even_valid <= 1'b1;
                                did_even_line <= did_build_line;
                            end
                        end else begin
                            did_build_x <= did_build_x + 10'd1;
                        end
                    end
                end
                default: did_build_state <= DID_IDLE;
            endcase

            if (vc1_scan_rsp_valid && (did_build_state == DID_FETCH)) begin
                case (vc1_scan_rsp_tag)
                    SCAN_CURSOR_P0: begin
                        if (did_build_buffer)
                            cursor_odd_plane0 <= vc1_scan_ram_rdata;
                        else
                            cursor_even_plane0 <= vc1_scan_ram_rdata;
                        vc1_scan_ram_en <= 1'b1;
                        vc1_scan_ram_addr <= cursor_build_row_addr + 16'd128;
                        vc1_scan_ram_tag <= SCAN_CURSOR_P1;
                    end
                    SCAN_CURSOR_P1: begin
                        if (did_build_buffer) begin
                            cursor_odd_plane1 <= vc1_scan_ram_rdata;
                            cursor_odd_valid <= 1'b1;
                            cursor_odd_line <= did_build_line;
                        end else begin
                            cursor_even_plane1 <= vc1_scan_ram_rdata;
                            cursor_even_valid <= 1'b1;
                            cursor_even_line <= did_build_line;
                        end
                        vc1_scan_ram_en <= 1'b1;
                        if (did_build_span_hit) begin
                            did_build_ptr <= did_span_ptr;
                            vc1_scan_ram_addr <= did_span_ptr;
                            vc1_scan_ram_tag <= SCAN_COUNT_HI;
                        end else begin
                            vc1_scan_ram_addr <= did_frame_entry;
                            vc1_scan_ram_tag <= SCAN_PTR_HI;
                        end
                    end
                    SCAN_PTR_HI: begin
                        did_scan_hi <= vc1_scan_byte;
                        vc1_scan_ram_en <= 1'b1;
                        vc1_scan_ram_addr <= did_frame_entry + 16'd1;
                        vc1_scan_ram_tag <= SCAN_PTR_LO;
                    end
                    SCAN_PTR_LO: begin
                        if (did_frame_word_valid) begin
                            did_span_valid <= 1'b1;
                            if (!did_frame_word[15])
                                did_span_first <= did_walk_row;
                            did_span_end <= did_decoded_end[9:0];
                            did_span_ptr <= did_decoded_ptr;
                            vc1_scan_ram_en <= 1'b1;
                            if ({6'd0, did_build_line} < did_decoded_end) begin
                                did_build_ptr <= did_decoded_ptr;
                                vc1_scan_ram_addr <= did_decoded_ptr;
                                vc1_scan_ram_tag <= SCAN_COUNT_HI;
                            end else begin
                                // Resolve an arbitrary restart target by
                                // walking from a proven prefix. Every legal
                                // command advances at least one row, so this
                                // uses at most V_VISIBLE frame-word reads.
                                did_walk_row <= did_decoded_end[9:0];
                                vc1_scan_ram_addr <=
                                    {vc1_regs[7'h40], vc1_regs[7'h41]} +
                                    {5'd0, did_decoded_end[9:0], 1'b0};
                                vc1_scan_ram_tag <= SCAN_PTR_HI;
                            end
                        end else begin
                            // No predecessor or non-progressing/out-of-frame
                            // repeat: do not reinterpret its count as a RAM
                            // pointer. Retain the invalid-list DID0 fallback.
                            did_span_valid <= 1'b0;
                            did_build_count <= 9'd0;
                            did_build_entry <= 9'd0;
                            did_build_x <= 10'd0;
                            did_build_value <= 5'd0;
                            did_build_state <= DID_FILL;
                        end
                    end
                    SCAN_COUNT_HI: begin
                        did_scan_hi <= vc1_scan_byte;
                        vc1_scan_ram_en <= 1'b1;
                        vc1_scan_ram_addr <= did_build_ptr + 16'd1;
                        vc1_scan_ram_tag <= SCAN_COUNT_LO;
                    end
                    SCAN_COUNT_LO: begin
                        if (did_line_count_valid(
                                did_build_ptr,
                                {did_scan_hi, vc1_scan_byte})) begin
                            did_build_count <=
                                {1'b0, vc1_scan_byte};
                            did_build_entry <= 9'h0;
                            if (vc1_scan_byte != 8'h0) begin
                                vc1_scan_ram_en <= 1'b1;
                                vc1_scan_ram_addr <= did_build_ptr + 16'd2;
                                vc1_scan_ram_tag <= SCAN_SEG_HI;
                            end else begin
                                did_build_x <= 10'h0;
                                did_build_value <= 5'h0;
                                did_build_state <= DID_FILL;
                            end
                        end else begin
                            did_build_count <= 9'h0;
                            did_build_entry <= 9'h0;
                            did_build_x <= 10'h0;
                            did_build_value <= 5'h0;
                            did_build_state <= DID_FILL;
                        end
                    end
                    SCAN_SEG_HI: begin
                        did_scan_hi <= vc1_scan_byte;
                        vc1_scan_ram_en <= 1'b1;
                        vc1_scan_ram_addr <= did_build_ptr + 16'd3 +
                            {7'h0, did_build_entry[7:0], 1'b0};
                        vc1_scan_ram_tag <= SCAN_SEG_LO;
                    end
                    SCAN_SEG_LO: begin
                        if ((did_build_entry + 9'd1) < did_build_count) begin
                            did_build_entry <= did_build_entry + 9'd1;
                            vc1_scan_ram_en <= 1'b1;
                            vc1_scan_ram_addr <= did_build_ptr + 16'd4 +
                                {7'h0, did_build_entry[7:0], 1'b0};
                            vc1_scan_ram_tag <= SCAN_SEG_HI;
                        end else begin
                            did_build_entry <= 9'h0;
                            did_build_x <= 10'h0;
                            did_build_value <= 5'h0;
                            did_build_state <= DID_LOAD;
                        end
                    end
                    default: ;
                endcase
            end

            if (cfg_valid && cfg_write) begin
                // SET-page accesses exercise REX's readable staging latch;
                // they do not drive the selected DAC/VC1 device.  Keep the
                // returned byte coherent with the accepted write so a later
                // SET read does not expose the prior transaction's byte.
                if (!cfg_go)
                    cfg_rbyte <= cfg_wbyte;
                if (cfg_dac_access) begin
                    dac_latch <= cfg_wbyte;
                    if (cfg_go) begin
                        case (cfg_sel)
                            3'd0, 3'd4: begin
                                dac_addr <= cfg_wbyte;
                                dac_rgb_phase <= 2'd0;
                            end
                            3'd3: begin
                                // Loading a read address prefetches that RGB
                                // word and advances ADDR to the next location.
`ifndef SYNTHESIS
                                dac_rgb_read_r <= palette_component(
                                    dac_palette_bank, cfg_wbyte, 2'd0);
                                dac_rgb_read_g <= palette_component(
                                    dac_palette_bank, cfg_wbyte, 2'd1);
                                dac_rgb_read_b <= palette_component(
                                    dac_palette_bank, cfg_wbyte, 2'd2);
`endif
                                dac_addr <= cfg_wbyte + 8'd1;
                                dac_rgb_phase <= 2'd0;
                            end
                            3'd7: begin
                                dac_rgb_read_r <=
                                    bt_overlay_r[cfg_wbyte[3:0]];
                                dac_rgb_read_g <=
                                    bt_overlay_g[cfg_wbyte[3:0]];
                                dac_rgb_read_b <=
                                    bt_overlay_b[cfg_wbyte[3:0]];
                                dac_addr <= cfg_wbyte + 8'd1;
                                dac_rgb_phase <= 2'd0;
                            end
                            3'd1: begin
                                case (dac_rgb_phase)
                                    2'd0: begin
                                        dac_rgb_stage_r <=
                                            dac_mpu_color_write(cfg_wbyte);
                                        dac_rgb_phase <= 2'd1;
                                    end
                                    2'd1: begin
                                        dac_rgb_stage_g <=
                                            dac_mpu_color_write(cfg_wbyte);
                                        dac_rgb_phase <= 2'd2;
                                    end
                                    default: begin
                                        // The Bt479 concatenates R/G/B and
                                        // commits the complete 24-bit word on
                                        // the blue cycle.
                                        dac_rgb_phase <= 2'd0;
                                        dac_addr <= dac_addr + 8'd1;
                                    end
                                endcase
                            end
                            3'd2: dac_rdmask <= cfg_wbyte;
                            3'd5: begin
                                case (dac_rgb_phase)
                                    2'd0: begin
                                        dac_rgb_stage_r <=
                                            dac_mpu_color_write(cfg_wbyte);
                                        dac_rgb_phase <= 2'd1;
                                    end
                                    2'd1: begin
                                        dac_rgb_stage_g <=
                                            dac_mpu_color_write(cfg_wbyte);
                                        dac_rgb_phase <= 2'd2;
                                    end
                                    default: begin
                                        dac_rgb_phase <= 2'd0;
                                        dac_addr <= dac_addr + 8'd1;
                                    end
                                endcase
                            end
                            3'd6: begin
                                // Bt479 CONTROL uses the internal address
                                // register. LIGHT writes 0x82 once, followed
                                // by command0, command1, flood-low, and
                                // flood-high as four consecutive accesses.
                                if (dac_addr > 8'h81) begin
                                    case (dac_addr)
                                        8'h82: dac_cmd0 <= cfg_wbyte;
                                        8'h83: dac_cmd1 <= dac_control_value(
                                            dac_addr, cfg_wbyte);
                                        8'h84: dac_flood_lo <= cfg_wbyte;
                                        8'h85: dac_flood_hi <= cfg_wbyte;
                                        default: ;
                                    endcase
                                end
                                dac_addr <= dac_addr + 8'd1;
                            end
                            default: ;
                        endcase
                    end
                end else if (cfg_vc1_access) begin
                    // The MPU address ports are written before every VC1
                    // register/SRAM read and do not alter the display list.
                    // Likewise XMAP and SYS_CTL writes affect the downstream
                    // pixel decode, not the sequential DID line already being
                    // assembled.  Invalidate only state which can change the
                    // DID frame/line walk. LIGHT allocates video-timing and
                    // cursor SRAM below DID_EP; writes below that programmed
                    // base cannot alter the DID frame/list walk. SRAM writes
                    // at or above DID_EP remain conservatively invalidating.
                    if (vc1_did_cache_mutation) begin
                        did_even_valid <= 1'b0;
                        did_odd_valid <= 1'b0;
                        did_build_state <= DID_IDLE;
                        did_span_valid <= 1'b0;
                        did_walk_row <= 10'd0;
                    end
                    if (vc1_cursor_cache_mutation) begin
                        cursor_even_valid <= 1'b0;
                        cursor_odd_valid <= 1'b0;
                    end
                    // Timing SRAM is normally written only while VTG is off.
                    // Once a table has been accepted, invalidate only writes
                    // which overlap the decoded line-table span or the frame
                    // table itself; cursor and DID traffic must not restart
                    // the timing walk on every update.
                    if (vc1_cfg_ram_write &&
                        ((!timing_active && !timing_apply_pending) ||
                         ((vc1_addr >= timing_line_min) &&
                          (vc1_word_addr <= timing_line_max)) ||
                        ((vc1_addr >= timing_frame_base) &&
                          (vc1_word_addr <= timing_frame_end)))) begin
                        timing_rebuild_pending <= 1'b1;
                        if (timing_scan_active || timing_active ||
                            timing_apply_pending || timing_start_armed) begin
                            timing_scan_active <= 1'b0;
                            timing_scan_state <= TIMING_IDLE;
                            timing_active <= 1'b0;
                            timing_apply_pending <= 1'b0;
                            if (!vc1_sys_ctrl[1])
                                timing_start_armed <= 1'b1;
                        end
                    end
                    if (cfg_go && (cfg_sel == 3'd0) &&
                        ((vc1_addr == 16'h0000) ||
                         (vc1_addr == 16'h0001) ||
                         (vc1_addr == 16'h0014))) begin
                        timing_rebuild_pending <= 1'b1;
                        if (timing_scan_active || timing_active ||
                            timing_apply_pending || timing_start_armed) begin
                            timing_scan_active <= 1'b0;
                            timing_scan_state <= TIMING_IDLE;
                            timing_active <= 1'b0;
                            timing_apply_pending <= 1'b0;
                            if (!vc1_sys_ctrl[1])
                                timing_start_armed <= 1'b1;
                        end
                    end
                    dac_latch <= cfg_wbyte;
                    if (cfg_go) begin
                        case (cfg_sel)
                            3'd0: begin
                                if (vc1_addr != 16'h0080)
                                    vc1_regs[vc1_addr[6:0]] <=
                                        cfg_wbyte & vc1_mask(vc1_addr[6:0]);
                                vc1_addr <= vc1_addr + 16'd1;
                            end
                            3'd1: begin
                                vc1_addr <= vc1_addr + 16'd1;
                            end
                            3'd2: begin
`ifndef SYNTHESIS
                                if (vc1_word_commit) begin
                                    vc1_ram_diag[vc1_word_addr] <= vc1_mpu_msb;
                                    vc1_ram_diag[vc1_addr] <= cfg_wbyte;
                                end
`endif
                                vc1_addr <= vc1_addr + 16'd1;
                            end
                            3'd4: vc1_addr[7:0] <= cfg_wbyte;
                            3'd5: vc1_addr[15:8] <= cfg_wbyte;
                            3'd6: begin
                                vc1_sys_ctrl <= cfg_wbyte;
                                if (cfg_wbyte[1]) begin
                                    timing_scan_active <= 1'b0;
                                    timing_scan_state <= TIMING_IDLE;
                                    timing_start_armed <= 1'b0;
                                    timing_apply_pending <= 1'b0;
                                    timing_active <= 1'b0;
                                end else begin
                                    if (vc1_sys_ctrl[1] ||
                                        timing_rebuild_pending ||
                                        (!timing_active &&
                                         !timing_apply_pending)) begin
                                        timing_rebuild_pending <= 1'b1;
                                        timing_start_armed <= 1'b1;
                                        timing_scan_active <= 1'b0;
                                        timing_scan_state <= TIMING_IDLE;
                                        timing_apply_pending <= 1'b0;
                                        timing_active <= 1'b0;
                                    end
                                end
                            end
                            default: ;
                        endcase
                    end
                end
            end else if (cfg_valid && !cfg_write) begin
                if (cfg_dac_access) begin
                    if (cfg_go) begin
                        // REX GO returns the previous MPU latch while this
                        // access fetches the next device byte. LIGHT primes
                        // once with a dummy GO, then streams GO results;
                        // SET consumes the fetched byte without advancing.
                        cfg_rbyte <= dac_latch;
                        dac_latch <= dac_read_port_value(cfg_sel);
                        if ((cfg_sel == 3'd1) || (cfg_sel == 3'd5)) begin
                            if (dac_rgb_phase == 2'd2) begin
                                if (cfg_sel == 3'd1) begin
`ifndef SYNTHESIS
                                    dac_rgb_read_r <= palette_component(
                                        dac_palette_bank, dac_addr, 2'd0);
                                    dac_rgb_read_g <= palette_component(
                                        dac_palette_bank, dac_addr, 2'd1);
                                    dac_rgb_read_b <= palette_component(
                                        dac_palette_bank, dac_addr, 2'd2);
`else
                                    dac_rgb_read_r <= palette_cfg_q[23:16];
                                    dac_rgb_read_g <= palette_cfg_q[15:8];
                                    dac_rgb_read_b <= palette_cfg_q[7:0];
`endif
                                end else begin
                                    dac_rgb_read_r <=
                                        bt_overlay_r[dac_addr[3:0]];
                                    dac_rgb_read_g <=
                                        bt_overlay_g[dac_addr[3:0]];
                                    dac_rgb_read_b <=
                                        bt_overlay_b[dac_addr[3:0]];
                                end
                                dac_rgb_phase <= 2'd0;
                                dac_addr <= dac_addr + 8'd1;
                            end else begin
                                dac_rgb_phase <= dac_rgb_phase + 2'd1;
                            end
                        end else if (cfg_sel == 3'd6) begin
                            dac_addr <= dac_addr + 8'd1;
                        end
                    end else
                        cfg_rbyte <= dac_latch;
                end else if (cfg_vc1_access) begin
                    if (cfg_go) begin
                        cfg_rbyte <= dac_latch;
                        dac_latch <= vc1_read_port_value(cfg_sel);
                        if ((cfg_sel == 3'd0) || (cfg_sel == 3'd1) ||
                            (cfg_sel == 3'd2))
                            vc1_addr <= vc1_addr + 16'd1;
                    end else
                        cfg_rbyte <= dac_latch;
                end
            end

            // Position writes during a frame keep using the old cursor line
            // caches.  When the latest position is adopted at the next source
            // origin, discard those tags so subsequent rows are rebuilt for
            // the new frame-coherent Y coordinate.
            if (resolver_input_frame_start &&
                ((cursor_frame_x !=
                  {vc1_regs[7'h22], vc1_regs[7'h23]}) ||
                 (cursor_frame_y !=
                  {vc1_regs[7'h24], vc1_regs[7'h25]}))) begin
                cursor_even_valid <= 1'b0;
                cursor_odd_valid <= 1'b0;
            end
        end
    end
endmodule

`default_nettype wire
