// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - LG1 clean modular core skeleton
//
// This top does not include or depend on lg1.v. It is a compileable landing
// zone for the clean-sheet pipeline split: REX frontend, context queue,
// iterator, RB backend, SSRAM arbiter, and VC1/XMAP/DAC scanout.

`default_nettype none

module lg1_clean_core #(
    parameter ENABLE_SCANOUT = 1,
    parameter ENABLE_PHYSICAL_FETCH = 1,
    parameter ENABLE_DIAGNOSTIC_SAMPLES = 0
) (
    input  wire        clk,
    input  wire        rst_n,
    // Dedicated LG1 serial-domain boundary.  This interface-only tranche
    // keeps all behavior on clk; the following semantic tranche moves the
    // VC1/XMAP/DAC raster behind these pins.
    input  wire        serial_clk,
    input  wire        serial_rst_n,

    input  wire [14:0] addr,
    input  wire [63:0] wdata,
    output wire [63:0] rdata,
    input  wire [ 7:0] wmask,
    input  wire        valid,
    input  wire        write,
    output wire        ready,

    output wire        mem_req_valid,
    input  wire        mem_req_ready,
    output wire        mem_req_write,
    output wire [ 1:0] mem_req_plane,
    output wire [17:0] mem_req_addr,
    output wire [31:0] mem_req_wdata,
    output wire [ 3:0] mem_req_wstrb,
    input  wire        mem_rsp_valid,
    input  wire [31:0] mem_rsp_rdata,
    input  wire        mem_busy,

    output wire        scan_req_valid,
    input  wire        scan_req_ready,
    output wire [15:0] scan_req_addr,
    input  wire        scan_rsp_valid,
    input  wire [511:0] scan_rsp_rdata,
    input  wire        scanout_pause,

    output wire        video_de,
    output wire        video_hsync,
    output wire        video_vsync,
    output wire [ 7:0] video_r,
    output wire [ 7:0] video_g,
    output wire [ 7:0] video_b,
    output wire        video_valid,
    output wire        video_frame_start,
    output wire [ 9:0] video_x,
    output wire [ 9:0] video_y,
    output wire [ 4:0] video_did,
    output wire [ 7:0] video_raw_index,
    output wire [ 7:0] video_index,
    output wire [15:0] video_xmap_mode,
    output wire [ 1:0] video_palette_bank,
    output wire [ 2:0] video_resolver_class,
    output wire [91:0] resolver_telemetry,
    output wire [91:0] resolver_detail,
    output wire        resolver_frame_done,
    output wire [15:0] front_response_low_signature,
    output wire [15:0] front_response_high_signature,
    output wire [ 7:0] front_response_signature_generation,
    output wire        front_response_signature_valid,
    output wire        scanout_underrun,
    output wire [31:0] scanout_underrun_count,
    output wire [31:0] scanout_protocol_error_count,
    output wire        irq,

    input  wire        scan_sample_valid,
    input  wire [ 9:0] scan_sample_x,
    input  wire [ 9:0] scan_sample_y,
    input  wire [ 7:0] scan_sample_raw_index,
    input  wire [ 7:0] scan_sample_raw_back_index,
    input  wire [ 1:0] scan_sample_overlay,
    output wire        scan_sample_de,
    output wire [ 4:0] scan_sample_did,
    output wire [15:0] scan_sample_mode,
    output wire [ 1:0] scan_sample_palette_bank,
    output wire [ 7:0] scan_sample_display_index,
    output wire [ 7:0] scan_sample_r,
    output wire [ 7:0] scan_sample_g,
    output wire [ 7:0] scan_sample_b,

    output wire        busy
);
    // REVREG: P1 board, monitor code 0, and no video option.  Video presence
    // is active-low; returning 0x01 makes software initialize the absent SV1
    // option and its separate hardware-colormap residency path.
    localparam [7:0] LG1_BOARD_INFO = 8'h41;

    wire        fe_ctx_valid;
    wire        fe_ctx_ready;
    wire        fe_ctx_rwaux_write;
    wire        fe_ctx_solid_fill;
    wire        fe_ctx_pattern_fill;
    wire        fe_ctx_pattern_tail;
    wire        fe_ctx_copy_fill;
    wire        fe_ctx_line_draw;
    wire        fe_ctx_shade;
    wire [31:0] fe_ctx_command;
    wire [31:0] fe_ctx_data;
    wire [19:0] fe_ctx_lsmode;
    wire        fe_ctx_ls_reload;
    wire [ 3:0] fe_ctx_lane_mask;
    wire [11:0] fe_ctx_x;
    wire [11:0] fe_ctx_y;
    wire [11:0] fe_ctx_xstart;
    wire [11:0] fe_ctx_xend;
    wire [11:0] fe_ctx_yend;
    wire        fe_ctx_x_down;
    wire        fe_ctx_y_down;
    wire        fe_ctx_block;
    wire        fe_ctx_clip_xend;
    wire        fe_ctx_quadmode;
    wire [ 1:0] fe_ctx_plane;
    wire [ 7:0] fe_ctx_color;
    wire [19:0] fe_ctx_color_fixed;
    wire [19:0] fe_ctx_color1_fixed;
    wire [19:0] fe_ctx_color2_fixed;
    wire [20:0] fe_ctx_slope;
    wire [20:0] fe_ctx_slope1;
    wire [20:0] fe_ctx_slope2;
    wire [16:0] fe_ctx_minor_slope;
    wire        fe_ctx_minor_negative;
    wire [14:0] fe_ctx_line_frac;
    wire        fe_ctx_pattern_opaque;
    wire [ 7:0] fe_ctx_bg_color;
    wire [11:0] fe_ctx_copy_dx;
    wire [11:0] fe_ctx_copy_dy;
    wire [31:0] fe_ctx_aux2;
    wire [255:0] fe_ctx_smask;
    wire [ 3:0] fe_ctx_logic_op;
    wire [ 7:0] fe_ctx_pixel_writemask;
    wire        fe_ctx_colorcomp;
    wire [ 9:0] fe_ctx_aux1;
    wire [11:0] fe_ctx_retire_x;
    wire [11:0] fe_ctx_retire_y;
    wire        fe_ctx_retire_done;

    wire        q_ctx_valid;
    wire        q_ctx_ready;
    wire        q_ctx_rwaux_write;
    wire        q_ctx_solid_fill;
    wire        q_ctx_pattern_fill;
    wire        q_ctx_pattern_tail;
    wire        q_ctx_copy_fill;
    wire        q_ctx_line_draw;
    wire        q_ctx_shade;
    wire [31:0] q_ctx_command;
    wire [31:0] q_ctx_data;
    wire [19:0] q_ctx_lsmode;
    wire        q_ctx_ls_reload;
    wire [ 3:0] q_ctx_lane_mask;
    wire [11:0] q_ctx_x;
    wire [11:0] q_ctx_y;
    wire [11:0] q_ctx_xstart;
    wire [11:0] q_ctx_xend;
    wire [11:0] q_ctx_yend;
    wire        q_ctx_x_down;
    wire        q_ctx_y_down;
    wire        q_ctx_block;
    wire        q_ctx_clip_xend;
    wire        q_ctx_quadmode;
    wire [ 1:0] q_ctx_plane;
    wire [ 7:0] q_ctx_color;
    wire [19:0] q_ctx_color_fixed;
    wire [19:0] q_ctx_color1_fixed;
    wire [19:0] q_ctx_color2_fixed;
    wire [20:0] q_ctx_slope;
    wire [20:0] q_ctx_slope1;
    wire [20:0] q_ctx_slope2;
    wire [16:0] q_ctx_minor_slope;
    wire        q_ctx_minor_negative;
    wire [14:0] q_ctx_line_frac;
    wire        q_ctx_pattern_opaque;
    wire [ 7:0] q_ctx_bg_color;
    wire [11:0] q_ctx_copy_dx;
    wire [11:0] q_ctx_copy_dy;
    wire [31:0] q_ctx_aux2;
    wire [255:0] q_ctx_smask;
    wire [ 3:0] q_ctx_logic_op;
    wire [ 7:0] q_ctx_pixel_writemask;
    wire        q_ctx_colorcomp;
    wire [ 9:0] q_ctx_aux1;
    wire [11:0] q_ctx_retire_x;
    wire [11:0] q_ctx_retire_y;
    wire        q_ctx_retire_done;

    wire        pix_valid;
    wire        pix_ready;
    wire [11:0] pix_x;
    wire [11:0] pix_y;
    wire [31:0] pix_data;
    wire [ 1:0] pix_plane;
    wire [ 3:0] pix_mask;
    wire        pix_copy;
    wire [11:0] pix_src_x;
    wire [11:0] pix_src_y;
    wire [31:0] pix_aux2;
    wire [255:0] pix_smask;
    wire [ 3:0] pix_logic_op;
    wire [ 7:0] pix_pixel_writemask;
    wire        pix_colorcomp;
    wire [ 9:0] pix_aux1;
    wire        pix_rgbmode;
    wire        pix_rwaux;
    wire        pix_last;
    wire [11:0] pix_retire_x;
    wire [11:0] pix_retire_y;
    wire        pix_retire_done;
    wire        retire_valid;
    wire [11:0] retire_x;
    wire [11:0] retire_y;
    wire        retire_done;
    wire        ls_work_valid;
    wire [31:0] ls_work_pattern;
    wire [19:0] ls_work_mode;
    // Working stipple feedback belongs to the selected REX context. Once
    // TOGGLECTXT selects the other bank, the restored LSPATTERN/LSMODE pair
    // is authoritative until that context retires a stippled primitive.
    wire        ls_work_invalidate = valid && write && fe_ready &&
                                      config_region &&
                                      (cfg_offset == 11'h7f4);
    wire        fe_load_valid;
    wire        fe_load_ready;
    wire [11:0] fe_load_x;
    wire [11:0] fe_load_y;
    wire [11:0] fe_load_xend;
    wire        fe_load_x_down;
    wire        fe_load_block;
    wire        fe_load_quadmode;
    wire        fe_load_exhausted;
    wire [ 1:0] fe_load_plane;
    wire [ 9:0] fe_load_aux1;
    wire        fe_load_is_pixel;
    wire        rb_load_valid;
    wire        rb_load_ready;
    wire        rb_load_rsp_valid;
    wire [31:0] rb_load_rsp_data;
    wire        fe_load_rsp_ready;

    wire        rb_req_valid;
    wire        rb_req_ready;
    wire        rb_arb_req_ready;
    wire        rb_req_write;
    wire [ 1:0] rb_req_plane;
    wire [17:0] rb_req_addr;
    wire [31:0] rb_req_wdata;
    wire [ 3:0] rb_req_wstrb;
    wire        rb_rsp_valid;
    wire [31:0] rb_rsp_rdata;
    wire        scan_mem_req_valid;
    wire [15:0] scan_mem_req_addr;
    wire        scan_mem_rsp_valid = scan_rsp_valid;
    wire        scan_fetch_snoop_block;
    wire        rb_snoop_block = scan_fetch_snoop_block;
    wire        rb_arb_req_valid = rb_req_valid && !rb_snoop_block;
    assign rb_arb_req_ready = mem_req_ready;
    wire        rb_snoop_valid = rb_req_valid && rb_req_ready &&
                                  rb_req_write;
    assign rb_req_ready = mem_req_ready && !rb_snoop_block;
    assign rb_rsp_valid = mem_rsp_valid;
    assign rb_rsp_rdata = mem_rsp_rdata;
    assign mem_req_valid = rb_arb_req_valid;
    assign mem_req_write = rb_req_write;
    assign mem_req_plane = rb_req_plane;
    assign mem_req_addr = rb_req_addr;
    assign mem_req_wdata = rb_req_wdata;
    assign mem_req_wstrb = rb_req_wstrb;
    assign scan_req_valid = scan_mem_req_valid;
    assign scan_req_addr = scan_mem_req_addr;
    wire [ 9:0] scan_timing_x;
    wire [ 9:0] scan_timing_y;
    wire        scan_timing_visible;
    wire        fetch_sample_valid;
    wire [ 9:0] fetch_sample_x;
    wire [ 9:0] fetch_sample_y;
    wire [ 7:0] fetch_sample_front;
    wire [ 7:0] fetch_sample_back;
    wire [ 1:0] fetch_sample_overlay;
    wire        fetch_underrun;
    wire        diagnostic_sample_valid = ENABLE_DIAGNOSTIC_SAMPLES &&
                                           scan_sample_valid;
    wire        decoder_sample_valid = diagnostic_sample_valid ||
                                       ((ENABLE_PHYSICAL_FETCH != 0) &&
                                        fetch_sample_valid);
    wire [ 9:0] decoder_sample_x = diagnostic_sample_valid ?
        scan_sample_x : fetch_sample_x;
    wire [ 9:0] decoder_sample_y = diagnostic_sample_valid ?
        scan_sample_y : fetch_sample_y;
    wire [ 7:0] decoder_sample_front = diagnostic_sample_valid ?
        scan_sample_raw_index : fetch_sample_front;
    wire [ 7:0] decoder_sample_back = diagnostic_sample_valid ?
        scan_sample_raw_back_index : fetch_sample_back;
    wire [ 1:0] decoder_sample_overlay = diagnostic_sample_valid ?
        scan_sample_overlay : fetch_sample_overlay;
    wire        draw_busy;
    wire        rb_busy;
    wire [63:0] fe_rdata;
    wire        fe_ready;
    reg  [ 2:0] scan_cfgsel;
    reg  [ 7:0] wclock;
    wire [ 7:0] scan_cfg_src_rbyte;
    wire [ 7:0] scan_cfg_dst_rbyte;
    wire        scan_cfg_ready;
    wire        serial_cfg_valid;
    wire        serial_cfg_write;
    wire        serial_cfg_go;
    wire [10:0] serial_cfg_offset;
    wire [ 2:0] serial_cfg_sel;
    wire [ 7:0] serial_cfg_wbyte;
    wire        serial_retrace_irq;
    wire        machine_retrace_irq;
    wire        config_region = (addr[14:12] == 3'b100);
    wire [10:0] cfg_offset = {addr[10:2], 2'b00};
    wire        cfg_go = addr[11];
    wire        scan_cfg_access =
        config_region &&
        ((cfg_offset == 11'h7e8) || (cfg_offset == 11'h7f0));
    wire        core_cfg_access =
        config_region &&
        ((cfg_offset == 11'h7e4) || (cfg_offset == 11'h7ec));
    wire [ 7:0] cfg_wbyte = bus_wd_byte(wdata, wmask);
    wire [ 7:0] core_cfg_rbyte =
        (cfg_offset == 11'h7e4) ?
            ((scan_cfgsel == 3'd4) ? LG1_BOARD_INFO : wclock) :
            {5'h0, scan_cfgsel};

    // CONFIGMODE.CHIPBUSY is architectural REX activity only.  The shared
    // memory channel also carries autonomous scanout reads; including channel
    // or cache activity here can make a permanently active display look like
    // a permanently busy drawing engine to software.
    assign busy = fe_ctx_valid || q_ctx_valid || draw_busy || rb_busy ||
                  rb_req_valid;
    assign scanout_underrun = fetch_underrun;
    assign rdata = scan_cfg_access ?
                   {24'h0, scan_cfg_src_rbyte,
                    24'h0, scan_cfg_src_rbyte} :
                   core_cfg_access ?
                   {24'h0, core_cfg_rbyte, 24'h0, core_cfg_rbyte} :
                   fe_rdata;
    assign ready = scan_cfg_access ? scan_cfg_ready :
                   core_cfg_access ? valid : fe_ready;
    assign rb_load_valid = fe_load_valid && !fe_ctx_valid && !q_ctx_valid &&
                           !draw_busy && !pix_valid;
    assign fe_load_ready = rb_load_ready && !fe_ctx_valid && !q_ctx_valid &&
                           !draw_busy && !pix_valid;

    function [7:0] bus_wd_byte;
        input [63:0] data;
        input [ 7:0] mask;
        reg [31:0] word;
        begin
            word = (mask[7] || mask[6] || mask[5] || mask[4]) ?
                   data[63:32] : data[31:0];
            bus_wd_byte =
                (mask == 8'h80) ? data[63:56] :
                (mask == 8'h40) ? data[55:48] :
                (mask == 8'h20) ? data[47:40] :
                (mask == 8'h10) ? data[39:32] :
                (mask == 8'h08) ? data[31:24] :
                (mask == 8'h04) ? data[23:16] :
                (mask == 8'h02) ? data[15:8]  :
                (mask == 8'h01) ? data[7:0]   :
                                  word[7:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scan_cfgsel <= 3'h0;
            wclock <= 8'h0;
        end else if (valid && write && config_region &&
                     (cfg_offset == 11'h7ec)) begin
            scan_cfgsel <= cfg_wbyte[2:0];
        end else if (valid && write && config_region &&
                     (cfg_offset == 11'h7e4)) begin
            wclock <= cfg_wbyte;
        end
    end

    // VC1, XMAP and DAC are clocked by the original LG1 serial clock.  CPU
    // accesses remain ordered and one-outstanding across this boundary; a
    // read does not retire until the serial-side latch value has returned.
    lg1_config_cdc_bridge u_config_cdc (
        .src_clk(clk),
        .src_rst_n(rst_n),
        .src_valid(valid && scan_cfg_access),
        .src_write(write),
        .src_go(cfg_go),
        .src_offset(cfg_offset),
        .src_sel(scan_cfgsel),
        .src_wbyte(cfg_wbyte),
        .src_rbyte(scan_cfg_src_rbyte),
        .src_ready(scan_cfg_ready),
        .dst_clk(serial_clk),
        .dst_rst_n(serial_rst_n),
        .dst_valid(serial_cfg_valid),
        .dst_write(serial_cfg_write),
        .dst_go(serial_cfg_go),
        .dst_offset(serial_cfg_offset),
        .dst_sel(serial_cfg_sel),
        .dst_wbyte(serial_cfg_wbyte),
        .dst_rbyte(scan_cfg_dst_rbyte)
    );

    lg1_event_cdc_bridge u_retrace_cdc (
        .src_clk(serial_clk),
        .src_rst_n(serial_rst_n),
        .src_pulse(serial_retrace_irq),
        .dst_clk(clk),
        .dst_rst_n(rst_n),
        .dst_pulse(machine_retrace_irq)
    );

    lg1_rex_frontend u_frontend (
        .clk(clk),
        .rst_n(rst_n),
        .addr(addr),
        .wdata(wdata),
        .rdata(fe_rdata),
        .wmask(wmask),
        .valid(valid),
        .write(write),
        .ready(fe_ready),
        .chip_busy(busy),
        .load_valid(fe_load_valid),
        .load_ready(fe_load_ready),
        .load_x(fe_load_x),
        .load_y(fe_load_y),
        .load_xend(fe_load_xend),
        .load_x_down(fe_load_x_down),
        .load_block(fe_load_block),
        .load_quadmode(fe_load_quadmode),
        .load_exhausted(fe_load_exhausted),
        .load_plane(fe_load_plane),
        .load_aux1(fe_load_aux1),
        .load_is_pixel(fe_load_is_pixel),
        .load_rsp_valid(rb_load_rsp_valid),
        .load_rsp_data(rb_load_rsp_data),
        .load_rsp_ready(fe_load_rsp_ready),
        .ctx_valid(fe_ctx_valid),
        .ctx_rwaux_write(fe_ctx_rwaux_write),
        .ctx_solid_fill(fe_ctx_solid_fill),
        .ctx_pattern_fill(fe_ctx_pattern_fill),
        .ctx_pattern_tail(fe_ctx_pattern_tail),
        .ctx_copy_fill(fe_ctx_copy_fill),
        .ctx_line_draw(fe_ctx_line_draw),
        .ctx_shade(fe_ctx_shade),
        .ctx_command(fe_ctx_command),
        .ctx_data(fe_ctx_data),
        .ctx_lsmode(fe_ctx_lsmode),
        .ctx_ls_reload(fe_ctx_ls_reload),
        .ctx_lane_mask(fe_ctx_lane_mask),
        .ctx_x(fe_ctx_x),
        .ctx_y(fe_ctx_y),
        .ctx_xstart(fe_ctx_xstart),
        .ctx_xend(fe_ctx_xend),
        .ctx_yend(fe_ctx_yend),
        .ctx_x_down(fe_ctx_x_down),
        .ctx_y_down(fe_ctx_y_down),
        .ctx_block(fe_ctx_block),
        .ctx_clip_xend(fe_ctx_clip_xend),
        .ctx_quadmode(fe_ctx_quadmode),
        .ctx_plane(fe_ctx_plane),
        .ctx_color(fe_ctx_color),
        .ctx_color_fixed(fe_ctx_color_fixed),
        .ctx_color1_fixed(fe_ctx_color1_fixed),
        .ctx_color2_fixed(fe_ctx_color2_fixed),
        .ctx_slope(fe_ctx_slope),
        .ctx_slope1(fe_ctx_slope1),
        .ctx_slope2(fe_ctx_slope2),
        .ctx_minor_slope(fe_ctx_minor_slope),
        .ctx_minor_negative(fe_ctx_minor_negative),
        .ctx_line_frac(fe_ctx_line_frac),
        .ctx_pattern_opaque(fe_ctx_pattern_opaque),
        .ctx_bg_color(fe_ctx_bg_color),
        .ctx_copy_dx(fe_ctx_copy_dx),
        .ctx_copy_dy(fe_ctx_copy_dy),
        .ctx_aux2(fe_ctx_aux2),
        .ctx_smask(fe_ctx_smask),
        .ctx_logic_op(fe_ctx_logic_op),
        .ctx_pixel_writemask(fe_ctx_pixel_writemask),
        .ctx_colorcomp(fe_ctx_colorcomp),
        .ctx_aux1(fe_ctx_aux1),
        .ctx_retire_x(fe_ctx_retire_x),
        .ctx_retire_y(fe_ctx_retire_y),
        .ctx_retire_done(fe_ctx_retire_done),
        .ctx_ready(fe_ctx_ready),
        .retire_valid(retire_valid),
        .retire_x(retire_x),
        .retire_y(retire_y),
        .retire_done(retire_done),
        .ls_work_valid(ls_work_valid),
        .ls_work_pattern(ls_work_pattern),
        .ls_work_mode(ls_work_mode)
    );

    lg1_rex_context_queue u_context_queue (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(fe_ctx_valid),
        .in_ready(fe_ctx_ready),
        .in_rwaux_write(fe_ctx_rwaux_write),
        .in_solid_fill(fe_ctx_solid_fill),
        .in_pattern_fill(fe_ctx_pattern_fill),
        .in_pattern_tail(fe_ctx_pattern_tail),
        .in_copy_fill(fe_ctx_copy_fill),
        .in_line_draw(fe_ctx_line_draw),
        .in_shade(fe_ctx_shade),
        .in_command(fe_ctx_command),
        .in_data(fe_ctx_data),
        .in_lsmode(fe_ctx_lsmode),
        .in_ls_reload(fe_ctx_ls_reload),
        .in_lane_mask(fe_ctx_lane_mask),
        .in_x(fe_ctx_x),
        .in_y(fe_ctx_y),
        .in_xstart(fe_ctx_xstart),
        .in_xend(fe_ctx_xend),
        .in_yend(fe_ctx_yend),
        .in_x_down(fe_ctx_x_down),
        .in_y_down(fe_ctx_y_down),
        .in_block(fe_ctx_block),
        .in_clip_xend(fe_ctx_clip_xend),
        .in_quadmode(fe_ctx_quadmode),
        .in_plane(fe_ctx_plane),
        .in_color(fe_ctx_color),
        .in_color_fixed(fe_ctx_color_fixed),
        .in_color1_fixed(fe_ctx_color1_fixed),
        .in_color2_fixed(fe_ctx_color2_fixed),
        .in_slope(fe_ctx_slope),
        .in_slope1(fe_ctx_slope1),
        .in_slope2(fe_ctx_slope2),
        .in_minor_slope(fe_ctx_minor_slope),
        .in_minor_negative(fe_ctx_minor_negative),
        .in_line_frac(fe_ctx_line_frac),
        .in_pattern_opaque(fe_ctx_pattern_opaque),
        .in_bg_color(fe_ctx_bg_color),
        .in_copy_dx(fe_ctx_copy_dx),
        .in_copy_dy(fe_ctx_copy_dy),
        .in_aux2(fe_ctx_aux2),
        .in_smask(fe_ctx_smask),
        .in_logic_op(fe_ctx_logic_op),
        .in_pixel_writemask(fe_ctx_pixel_writemask),
        .in_colorcomp(fe_ctx_colorcomp),
        .in_aux1(fe_ctx_aux1),
        .in_retire_x(fe_ctx_retire_x),
        .in_retire_y(fe_ctx_retire_y),
        .in_retire_done(fe_ctx_retire_done),
        .out_valid(q_ctx_valid),
        .out_ready(q_ctx_ready),
        .out_rwaux_write(q_ctx_rwaux_write),
        .out_solid_fill(q_ctx_solid_fill),
        .out_pattern_fill(q_ctx_pattern_fill),
        .out_pattern_tail(q_ctx_pattern_tail),
        .out_copy_fill(q_ctx_copy_fill),
        .out_line_draw(q_ctx_line_draw),
        .out_shade(q_ctx_shade),
        .out_command(q_ctx_command),
        .out_data(q_ctx_data),
        .out_lsmode(q_ctx_lsmode),
        .out_ls_reload(q_ctx_ls_reload),
        .out_lane_mask(q_ctx_lane_mask),
        .out_x(q_ctx_x),
        .out_y(q_ctx_y),
        .out_xstart(q_ctx_xstart),
        .out_xend(q_ctx_xend),
        .out_yend(q_ctx_yend),
        .out_x_down(q_ctx_x_down),
        .out_y_down(q_ctx_y_down),
        .out_block(q_ctx_block),
        .out_clip_xend(q_ctx_clip_xend),
        .out_quadmode(q_ctx_quadmode),
        .out_plane(q_ctx_plane),
        .out_color(q_ctx_color),
        .out_color_fixed(q_ctx_color_fixed),
        .out_color1_fixed(q_ctx_color1_fixed),
        .out_color2_fixed(q_ctx_color2_fixed),
        .out_slope(q_ctx_slope),
        .out_slope1(q_ctx_slope1),
        .out_slope2(q_ctx_slope2),
        .out_minor_slope(q_ctx_minor_slope),
        .out_minor_negative(q_ctx_minor_negative),
        .out_line_frac(q_ctx_line_frac),
        .out_pattern_opaque(q_ctx_pattern_opaque),
        .out_bg_color(q_ctx_bg_color),
        .out_copy_dx(q_ctx_copy_dx),
        .out_copy_dy(q_ctx_copy_dy),
        .out_aux2(q_ctx_aux2),
        .out_smask(q_ctx_smask),
        .out_logic_op(q_ctx_logic_op),
        .out_pixel_writemask(q_ctx_pixel_writemask),
        .out_colorcomp(q_ctx_colorcomp),
        .out_aux1(q_ctx_aux1),
        .out_retire_x(q_ctx_retire_x),
        .out_retire_y(q_ctx_retire_y),
        .out_retire_done(q_ctx_retire_done)
    );

    lg1_draw_iter u_draw_iter (
        .clk(clk),
        .rst_n(rst_n),
        .ctx_valid(q_ctx_valid),
        .ctx_ready(q_ctx_ready),
        .ctx_rwaux_write(q_ctx_rwaux_write),
        .ctx_solid_fill(q_ctx_solid_fill),
        .ctx_pattern_fill(q_ctx_pattern_fill),
        .ctx_pattern_tail(q_ctx_pattern_tail),
        .ctx_copy_fill(q_ctx_copy_fill),
        .ctx_line_draw(q_ctx_line_draw),
        .ctx_shade(q_ctx_shade),
        .ctx_command(q_ctx_command),
        .ctx_data(q_ctx_data),
        .ctx_lsmode(q_ctx_lsmode),
        .ctx_ls_reload(q_ctx_ls_reload),
        .ls_work_invalidate(ls_work_invalidate),
        .ctx_lane_mask(q_ctx_lane_mask),
        .ctx_x(q_ctx_x),
        .ctx_y(q_ctx_y),
        .ctx_xstart(q_ctx_xstart),
        .ctx_xend(q_ctx_xend),
        .ctx_yend(q_ctx_yend),
        .ctx_x_down(q_ctx_x_down),
        .ctx_y_down(q_ctx_y_down),
        .ctx_block(q_ctx_block),
        .ctx_clip_xend(q_ctx_clip_xend),
        .ctx_quadmode(q_ctx_quadmode),
        .ctx_plane(q_ctx_plane),
        .ctx_color(q_ctx_color),
        .ctx_color_fixed(q_ctx_color_fixed),
        .ctx_color1_fixed(q_ctx_color1_fixed),
        .ctx_color2_fixed(q_ctx_color2_fixed),
        .ctx_slope(q_ctx_slope),
        .ctx_slope1(q_ctx_slope1),
        .ctx_slope2(q_ctx_slope2),
        .ctx_minor_slope(q_ctx_minor_slope),
        .ctx_minor_negative(q_ctx_minor_negative),
        .ctx_line_frac(q_ctx_line_frac),
        .ctx_pattern_opaque(q_ctx_pattern_opaque),
        .ctx_bg_color(q_ctx_bg_color),
        .ctx_copy_dx(q_ctx_copy_dx),
        .ctx_copy_dy(q_ctx_copy_dy),
        .ctx_aux2(q_ctx_aux2),
        .ctx_smask(q_ctx_smask),
        .ctx_logic_op(q_ctx_logic_op),
        .ctx_pixel_writemask(q_ctx_pixel_writemask),
        .ctx_colorcomp(q_ctx_colorcomp),
        .ctx_aux1(q_ctx_aux1),
        .ctx_retire_x(q_ctx_retire_x),
        .ctx_retire_y(q_ctx_retire_y),
        .ctx_retire_done(q_ctx_retire_done),
        .pix_valid(pix_valid),
        .pix_ready(pix_ready),
        .pix_x(pix_x),
        .pix_y(pix_y),
        .pix_data(pix_data),
        .pix_plane(pix_plane),
        .pix_mask(pix_mask),
        .pix_copy(pix_copy),
        .pix_src_x(pix_src_x),
        .pix_src_y(pix_src_y),
        .pix_aux2(pix_aux2),
        .pix_smask(pix_smask),
        .pix_logic_op(pix_logic_op),
        .pix_pixel_writemask(pix_pixel_writemask),
        .pix_colorcomp(pix_colorcomp),
        .pix_aux1(pix_aux1),
        .pix_rgbmode(pix_rgbmode),
        .pix_rwaux(pix_rwaux),
        .pix_last(pix_last),
        .pix_retire_x(pix_retire_x),
        .pix_retire_y(pix_retire_y),
        .pix_retire_done(pix_retire_done),
        .ls_work_valid(ls_work_valid),
        .ls_work_pattern(ls_work_pattern),
        .ls_work_mode(ls_work_mode),
        .busy(draw_busy)
    );

    lg1_rb_backend u_rb_backend (
        .clk(clk),
        .rst_n(rst_n),
        .pix_valid(pix_valid),
        .pix_ready(pix_ready),
        .pix_x(pix_x),
        .pix_y(pix_y),
        .pix_data(pix_data),
        .pix_plane(pix_plane),
        .pix_mask(pix_mask),
        .pix_copy(pix_copy),
        .pix_src_x(pix_src_x),
        .pix_src_y(pix_src_y),
        .pix_aux2(pix_aux2),
        .pix_smask(pix_smask),
        .pix_logic_op(pix_logic_op),
        .pix_pixel_writemask(pix_pixel_writemask),
        .pix_colorcomp(pix_colorcomp),
        .pix_aux1(pix_aux1),
        .pix_rgbmode(pix_rgbmode),
        .pix_rwaux(pix_rwaux),
        .pix_last(pix_last),
        .pix_retire_x(pix_retire_x),
        .pix_retire_y(pix_retire_y),
        .pix_retire_done(pix_retire_done),
        .load_valid(rb_load_valid),
        .load_ready(rb_load_ready),
        .load_x(fe_load_x),
        .load_y(fe_load_y),
        .load_xend(fe_load_xend),
        .load_x_down(fe_load_x_down),
        .load_block(fe_load_block),
        .load_quadmode(fe_load_quadmode),
        .load_exhausted(fe_load_exhausted),
        .load_plane(fe_load_plane),
        .load_aux1(fe_load_aux1),
        .load_is_pixel(fe_load_is_pixel),
        .load_rsp_valid(rb_load_rsp_valid),
        .load_rsp_data(rb_load_rsp_data),
        .load_rsp_ready(fe_load_rsp_ready),
        .mem_req_valid(rb_req_valid),
        .mem_req_ready(rb_req_ready),
        .mem_req_write(rb_req_write),
        .mem_req_plane(rb_req_plane),
        .mem_req_addr(rb_req_addr),
        .mem_req_wdata(rb_req_wdata),
        .mem_req_wstrb(rb_req_wstrb),
        .mem_rsp_valid(rb_rsp_valid),
        .mem_rsp_rdata(rb_rsp_rdata),
        .retire_valid(retire_valid),
        .retire_x(retire_x),
        .retire_y(retire_y),
        .retire_done(retire_done),
        .busy(rb_busy)
    );

    generate
        if (ENABLE_SCANOUT && (ENABLE_PHYSICAL_FETCH != 0)) begin : gen_line_fetch
            lg1_scanout_line_fetch u_line_fetch (
                .clk(clk),
                .rst_n(rst_n),
                .serial_clk(serial_clk),
                .serial_rst_n(serial_rst_n),
                .timing_x(scan_timing_x),
                .timing_y(scan_timing_y),
                .timing_visible(scan_timing_visible),
                .pause(scanout_pause),
                .sample_valid(fetch_sample_valid),
                .sample_x(fetch_sample_x),
                .sample_y(fetch_sample_y),
                .sample_front(fetch_sample_front),
                .sample_back(fetch_sample_back),
                .sample_overlay(fetch_sample_overlay),
                .front_response_low_signature(
                    front_response_low_signature),
                .front_response_high_signature(
                    front_response_high_signature),
                .front_response_signature_generation(
                    front_response_signature_generation),
                .front_response_signature_valid(
                    front_response_signature_valid),
                .underrun(fetch_underrun),
                .underrun_count(scanout_underrun_count),
                .protocol_error_count(scanout_protocol_error_count),
                .rb_snoop_valid(rb_snoop_valid),
                .rb_snoop_plane(rb_req_plane),
                .rb_snoop_addr(rb_req_addr),
                .rb_snoop_wdata(rb_req_wdata),
                .rb_snoop_wstrb(rb_req_wstrb),
                .rb_snoop_block(scan_fetch_snoop_block),
                .mem_req_valid(scan_mem_req_valid),
                .mem_req_ready(scan_req_ready),
                .mem_req_addr(scan_mem_req_addr),
                .mem_rsp_valid(scan_rsp_valid),
                .mem_rsp_rdata(scan_rsp_rdata)
            );
        end else begin : gen_no_line_fetch
            assign scan_mem_req_valid = 1'b0;
            assign scan_mem_req_addr = 16'h0;
            assign scan_fetch_snoop_block = 1'b0;
            assign fetch_sample_valid = 1'b0;
            assign fetch_sample_x = 10'h0;
            assign fetch_sample_y = 10'h0;
            assign fetch_sample_front = 8'h0;
            assign fetch_sample_back = 8'h0;
            assign fetch_sample_overlay = 2'h0;
            assign front_response_low_signature = 16'd0;
            assign front_response_high_signature = 16'd0;
            assign front_response_signature_generation = 8'd0;
            assign front_response_signature_valid = 1'b0;
            assign fetch_underrun = 1'b0;
            assign scanout_underrun_count = 32'h0;
            assign scanout_protocol_error_count = 32'h0;
        end
    endgenerate

    generate
        if (ENABLE_SCANOUT) begin : gen_scanout
            lg1_vc1_xmap_dac_scanout #(
                .ENABLE_DIAGNOSTIC_RESOLVER(ENABLE_DIAGNOSTIC_SAMPLES)
            ) u_scanout (
                .clk(serial_clk),
                .rst_n(serial_rst_n),
                .cfg_valid(serial_cfg_valid),
                .cfg_write(serial_cfg_write),
                .cfg_go(serial_cfg_go),
                .cfg_offset(serial_cfg_offset),
                .cfg_sel(serial_cfg_sel),
                .cfg_wbyte(serial_cfg_wbyte),
                .cfg_rbyte(scan_cfg_dst_rbyte),
                .sample_valid(decoder_sample_valid),
                .sample_physical(!diagnostic_sample_valid &&
                                 (ENABLE_PHYSICAL_FETCH != 0) &&
                                 fetch_sample_valid),
                .physical_enable(ENABLE_PHYSICAL_FETCH != 0),
                .sample_x(decoder_sample_x),
                .sample_y(decoder_sample_y),
                .sample_raw_index(decoder_sample_front),
                .sample_raw_back_index(decoder_sample_back),
                .sample_overlay(decoder_sample_overlay),
                .sample_de(scan_sample_de),
                .sample_did(scan_sample_did),
                .sample_mode(scan_sample_mode),
                .sample_palette_bank(scan_sample_palette_bank),
                .sample_display_index(scan_sample_display_index),
                .sample_r(scan_sample_r),
                .sample_g(scan_sample_g),
                .sample_b(scan_sample_b),
                .video_de(video_de),
                .video_hsync(video_hsync),
                .video_vsync(video_vsync),
                .video_r(video_r),
                .video_g(video_g),
                .video_b(video_b),
                .video_valid(video_valid),
                .video_frame_start(video_frame_start),
                .video_did(video_did),
                .video_raw_index(video_raw_index),
                .video_index(video_index),
                .video_xmap_mode(video_xmap_mode),
                .video_palette_bank(video_palette_bank),
                .video_resolver_class(video_resolver_class),
                .video_x(video_x),
                .video_y(video_y),
                .timing_x(scan_timing_x),
                .timing_y(scan_timing_y),
                .timing_visible(scan_timing_visible),
                .resolver_telemetry(resolver_telemetry),
                .resolver_detail(resolver_detail),
                .resolver_frame_done(resolver_frame_done),
                .retrace_irq(serial_retrace_irq)
            );
        end else begin : gen_no_scanout
            assign scan_cfg_dst_rbyte = 8'h0;
            assign video_de = 1'b0;
            assign video_hsync = 1'b1;
            assign video_vsync = 1'b1;
            assign video_r = 8'h0;
            assign video_g = 8'h0;
            assign video_b = 8'h0;
            assign video_valid = 1'b0;
            assign video_frame_start = 1'b0;
            assign video_x = 10'h0;
            assign video_y = 10'h0;
            assign video_did = 5'h0;
            assign video_raw_index = 8'h0;
            assign video_index = 8'h0;
            assign video_xmap_mode = 16'h0;
            assign video_palette_bank = 2'h0;
            assign video_resolver_class = 3'h0;
            assign resolver_telemetry = 92'd0;
            assign resolver_detail = 92'd0;
            assign resolver_frame_done = 1'b0;
            assign scan_timing_x = 10'h0;
            assign scan_timing_y = 10'h0;
            assign scan_timing_visible = 1'b0;
            assign serial_retrace_irq = 1'b0;
            assign scan_sample_de = 1'b0;
            assign scan_sample_did = 5'h0;
            assign scan_sample_mode = 16'h0;
            assign scan_sample_palette_bank = 2'h0;
            assign scan_sample_display_index = 8'h0;
            assign scan_sample_r = 8'h0;
            assign scan_sample_g = 8'h0;
            assign scan_sample_b = 8'h0;
        end
    endgenerate
    assign irq = machine_retrace_irq;
endmodule

`default_nettype wire
