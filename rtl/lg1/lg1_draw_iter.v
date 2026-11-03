// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - LG1 clean draw iterator

`default_nettype none

module lg1_draw_iter (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        ctx_valid,
    output wire        ctx_ready,
    input  wire        ctx_rwaux_write,
    input  wire        ctx_solid_fill,
    input  wire        ctx_pattern_fill,
    input  wire        ctx_pattern_tail,
    input  wire        ctx_copy_fill,
    input  wire        ctx_line_draw,
    input  wire        ctx_shade,
    input  wire [31:0] ctx_command,
    input  wire [31:0] ctx_data,
    input  wire [19:0] ctx_lsmode,
    input  wire        ctx_ls_reload,
    input  wire        ls_work_invalidate,
    input  wire [ 3:0] ctx_lane_mask,
    input  wire [11:0] ctx_x,
    input  wire [11:0] ctx_y,
    input  wire [11:0] ctx_xstart,
    input  wire [11:0] ctx_xend,
    input  wire [11:0] ctx_yend,
    input  wire        ctx_x_down,
    input  wire        ctx_y_down,
    input  wire        ctx_block,
    input  wire        ctx_clip_xend,
    input  wire        ctx_quadmode,
    input  wire [ 1:0] ctx_plane,
    input  wire [ 7:0] ctx_color,
    input  wire [19:0] ctx_color_fixed,
    input  wire [19:0] ctx_color1_fixed,
    input  wire [19:0] ctx_color2_fixed,
    input  wire [20:0] ctx_slope,
    input  wire [20:0] ctx_slope1,
    input  wire [20:0] ctx_slope2,
    input  wire [16:0] ctx_minor_slope,
    input  wire        ctx_minor_negative,
    input  wire [14:0] ctx_line_frac,
    input  wire        ctx_pattern_opaque,
    input  wire [ 7:0] ctx_bg_color,
    input  wire [11:0] ctx_copy_dx,
    input  wire [11:0] ctx_copy_dy,
    input  wire [31:0] ctx_aux2,
    input  wire [255:0] ctx_smask,
    input  wire [ 3:0] ctx_logic_op,
    input  wire [ 7:0] ctx_pixel_writemask,
    input  wire        ctx_colorcomp,
    input  wire [ 9:0] ctx_aux1,
    input  wire [11:0] ctx_retire_x,
    input  wire [11:0] ctx_retire_y,
    input  wire        ctx_retire_done,

    output wire        pix_valid,
    input  wire        pix_ready,
    output wire [11:0] pix_x,
    output wire [11:0] pix_y,
    // One ready/valid beat covers the four CI8 byte lanes in the aligned
    // destination word containing pix_x.  Lane 0 is pix_data[31:24].
    output wire [31:0] pix_data,
    output wire [ 1:0] pix_plane,
    output wire [ 3:0] pix_mask,
    output wire        pix_copy,
    output wire [11:0] pix_src_x,
    output wire [11:0] pix_src_y,
    output wire [31:0] pix_aux2,
    output wire [255:0] pix_smask,
    output wire [ 3:0] pix_logic_op,
    output wire [ 7:0] pix_pixel_writemask,
    output wire        pix_colorcomp,
    output wire [ 9:0] pix_aux1,
    output wire        pix_rgbmode,
    output wire        pix_rwaux,
    output wire        pix_last,
    output wire [11:0] pix_retire_x,
    output wire [11:0] pix_retire_y,
    output wire        pix_retire_done,

    output wire        ls_work_valid,
    output wire [31:0] ls_work_pattern,
    output wire [19:0] ls_work_mode,

    output wire        busy
);
    localparam [31:0] CMD_LRQPOLY = 32'h02000000;
    localparam [31:0] CMD_ENLSPAT = 32'h00000800;
    localparam [31:0] CMD_LSADVLAST = 32'h00001000;
    localparam [31:0] CMD_LSCONTINUE = 32'h00002000;

    reg        active_rwaux;
    reg        active_fill;
    reg        active_pattern;
    reg        active_copy;
    reg        active_line;
    reg [31:0] command_r;
    reg [31:0] data_r;
    reg [ 3:0] rwaux_source_mask_r;
    reg [ 3:0] rwaux_pending_mask_r;
    reg [11:0] base_x_r;
    reg [11:0] base_y_r;
    reg [11:0] xstart_r;
    reg [11:0] xend_r;
    reg        x_down_r;
    reg        block_r;
    reg        quadmode_r;
    reg [ 1:0] plane_r;
    reg [11:0] fill_x_r;
    reg [11:0] fill_y_r;
    reg [11:0] fill_xstart_r;
    reg [11:0] fill_xend_r;
    reg [11:0] fill_yend_r;
    reg        fill_x_down_r;
    reg        fill_y_down_r;
    reg [ 1:0] fill_plane_r;
    reg [ 7:0] fill_color_r;
    reg        shade_r;
    reg signed [20:0] fill_shade_r;
    reg signed [20:0] fill_shade1_r;
    reg signed [20:0] fill_shade2_r;
    reg signed [20:0] fill_slope_r;
    reg signed [20:0] fill_slope1_r;
    reg signed [20:0] fill_slope2_r;
    reg [ 5:0] pattern_bit_r;
    reg [ 5:0] pattern_step_r;
    reg [11:0] pattern_x_r;
    reg [ 7:0] lsmode_count_r;
    reg [ 7:0] lsmode_initial_count_r;
    reg [ 5:0] lsmode_initial_bit_r;
    reg [ 7:0] lsmode_repeat_r;
    reg [ 5:0] lsmode_last_bit_r;
    reg        ls_work_valid_r;
    reg [31:0] ls_work_pattern_r;
    reg [ 7:0] ls_work_count_r;
    reg [ 7:0] ls_work_repeat_r;
    reg [ 5:0] ls_work_last_bit_r;
    reg        pattern_tail_r;
    reg        pattern_y_down_r;
    reg        pattern_opaque_r;
    reg [ 7:0] pattern_bg_color_r;
    reg [11:0] copy_dx_r;
    reg [11:0] copy_dy_r;
    reg [31:0] aux2_r;
    reg [255:0] smask_r;
    reg [ 3:0] logic_op_r;
    reg [ 7:0] pixel_writemask_r;
    reg        colorcomp_r;
    reg [ 9:0] aux1_r;
    reg [11:0] retire_x_r;
    reg [11:0] retire_y_r;
    reg        retire_done_r;
    reg [11:0] line_x_r;
    reg [11:0] line_y_r;
    reg [11:0] line_xend_r;
    reg [11:0] line_yend_r;
    reg        line_xmajor_r;
    reg        line_minor_negative_r;
    reg [15:0] line_frac_step_r;
    reg [15:0] line_frac_accum_r;

    assign ctx_ready =
        !active_rwaux && !active_fill && !active_pattern && !active_copy &&
        !active_line;

    function [7:0] lane_data;
        input [31:0] data;
        input [ 1:0] lane;
        begin
            case (lane)
                2'd0: lane_data = data[31:24];
                2'd1: lane_data = data[23:16];
                2'd2: lane_data = data[15:8];
                default: lane_data = data[7:0];
            endcase
        end
    endfunction

    // The hardware save operation exposes the working stipple with its next
    // bit in position zero.  Keep only the programmed 17--32-bit ring; bits
    // outside that ring are architecturally unused.
    function [31:0] rotate_lspattern;
        input [31:0] pattern;
        input [ 5:0] first_bit;
        input [ 5:0] last_bit;
        integer dst_bit;
        integer src_bit;
        begin
            rotate_lspattern = 32'h0;
            for (dst_bit = 0; dst_bit < 32; dst_bit = dst_bit + 1) begin
                if (dst_bit <= last_bit) begin
                    src_bit = dst_bit + {26'h0, first_bit};
                    if (src_bit > {26'h0, last_bit})
                        src_bit = src_bit - ({26'h0, last_bit} + 32'd1);
                    rotate_lspattern[31 - dst_bit] = pattern[31 - src_bit];
                end
            end
        end
    endfunction

    function block_x_allowed;
        input [11:0] px;
        input [11:0] x0;
        input [11:0] x1;
        input        x_down;
        input        block_mode;
        begin
            if (!block_mode)
                block_x_allowed = 1'b1;
            else if (x_down)
                block_x_allowed = (x0 - px) <= (x0 - x1);
            else
                block_x_allowed = (px - x0) <= (x1 - x0);
        end
    endfunction

    function [31:0] pixel_word;
        input [1:0] lane;
        input [7:0] pixel;
        begin
            case (lane)
                2'd0: pixel_word = {pixel, 24'h0};
                2'd1: pixel_word = {8'h0, pixel, 16'h0};
                2'd2: pixel_word = {16'h0, pixel, 8'h0};
                default: pixel_word = {24'h0, pixel};
            endcase
        end
    endfunction

    function [7:0] shade_ci8_pixel;
        input signed [20:0] shade;
        begin
            if (shade < 21'sd0)
                shade_ci8_pixel = 8'h00;
            else if (shade > 21'sd522240)
                shade_ci8_pixel = 8'hff;
            else
                shade_ci8_pixel = shade[18:11];
        end
    endfunction

    function [3:0] bayer4;
        input [11:0] px;
        input [11:0] py;
        begin
            case ({py[1:0], px[1:0]})
                4'h0: bayer4 = 4'd0;
                4'h1: bayer4 = 4'd8;
                4'h2: bayer4 = 4'd2;
                4'h3: bayer4 = 4'd10;
                4'h4: bayer4 = 4'd12;
                4'h5: bayer4 = 4'd4;
                4'h6: bayer4 = 4'd14;
                4'h7: bayer4 = 4'd6;
                4'h8: bayer4 = 4'd3;
                4'h9: bayer4 = 4'd11;
                4'ha: bayer4 = 4'd1;
                4'hb: bayer4 = 4'd9;
                4'hc: bayer4 = 4'd15;
                4'hd: bayer4 = 4'd7;
                4'he: bayer4 = 4'd13;
                default: bayer4 = 4'd5;
            endcase
        end
    endfunction

    function [2:0] dither3;
        input [7:0] c;
        input [3:0] threshold;
        reg [7:0] scaled;
        reg [2:0] base;
        begin
            scaled = (c >> 1) - (c >> 4);
            base = scaled[6:4];
            dither3 = ((scaled[3:0] > threshold) && (base != 3'd7)) ?
                      (base + 3'd1) : base;
        end
    endfunction

    function [1:0] dither2;
        input [7:0] c;
        input [3:0] threshold;
        reg [7:0] scaled;
        reg [1:0] base;
        begin
            scaled = (c >> 2) - (c >> 4);
            base = scaled[5:4];
            dither2 = ((scaled[3:0] > threshold) && (base != 2'd3)) ?
                      (base + 2'd1) : base;
        end
    endfunction

    // Host RGB streams present one 0xAABBGGRR word per pixel through
    // non-QUAD RGB RWAUX.  The high byte is padding/alpha, not a CI8 pixel.
    // RGBMODE and DITHER select this stream, so use the
    // same coordinate-phased RGB332 conversion as generated RGB instead of
    // discarding the command's dither contract.
    function [7:0] rgb_rwaux_pixel;
        input [31:0] data;
        input        dither;
        input [11:0] px;
        input [11:0] py;
        reg [7:0] red;
        reg [7:0] green;
        reg [7:0] blue;
        reg [3:0] threshold;
        begin
            red = data[7:0];
            green = data[15:8];
            blue = data[23:16];
            if (dither) begin
                threshold = bayer4(px, py);
                rgb_rwaux_pixel = {dither3(red, threshold),
                                    dither2(blue, threshold),
                                    dither3(green, threshold)};
            end else begin
                rgb_rwaux_pixel = {red[7:5], blue[7:6], green[7:5]};
            end
        end
    endfunction

    function [7:0] source_pixel;
        input        rgbmode;
        input        dither;
        input        shade;
        input [11:0] px;
        input [11:0] py;
        input signed [20:0] shade0;
        input signed [20:0] shade1;
        input signed [20:0] shade2;
        input [7:0] solid_color;
        reg [7:0] red;
        reg [7:0] green;
        reg [7:0] blue;
        reg [3:0] threshold;
        begin
            if (rgbmode) begin
                red = shade_ci8_pixel(shade0);
                green = shade_ci8_pixel(shade1);
                blue = shade_ci8_pixel(shade2);
                if (dither) begin
                    threshold = bayer4(px, py);
                    source_pixel = {dither3(red, threshold),
                                    dither2(blue, threshold),
                                    dither3(green, threshold)};
                end else begin
                    source_pixel = {red[7:5], blue[7:6], green[7:5]};
                end
            end else if (shade) begin
                source_pixel = shade_ci8_pixel(shade0);
            end else begin
                source_pixel = solid_color;
            end
        end
    endfunction

    function [31:0] source_word;
        input        rgbmode;
        input        dither;
        input        shade;
        input signed [20:0] shade0_start;
        input signed [20:0] shade1_start;
        input signed [20:0] shade2_start;
        input signed [20:0] slope0;
        input signed [20:0] slope1;
        input signed [20:0] slope2;
        input [11:0] dst_x;
        input [11:0] dst_y;
        input [2:0] count;
        input x_down;
        input [7:0] solid_color;
        integer i;
        reg [11:0] px;
        reg signed [20:0] shade0;
        reg signed [20:0] shade1;
        reg signed [20:0] shade2;
        reg [7:0] pixel;
        begin
            source_word = (!rgbmode && !shade) ?
                          {solid_color, solid_color,
                           solid_color, solid_color} : 32'h0;
            shade0 = shade0_start;
            shade1 = shade1_start;
            shade2 = shade2_start;
            for (i = 0; i < 4; i = i + 1) begin
                if (i < count) begin
                    px = x_down ? (dst_x - i[11:0]) :
                                  (dst_x + i[11:0]);
                    pixel = source_pixel(rgbmode, dither, shade, px, dst_y,
                                         shade0, shade1, shade2, solid_color);
                    case (px[1:0])
                        2'd0: source_word[31:24] = pixel;
                        2'd1: source_word[23:16] = pixel;
                        2'd2: source_word[15:8] = pixel;
                        default: source_word[7:0] = pixel;
                    endcase
                    if (shade) begin
                        shade0 = shade0 + slope0;
                        shade1 = shade1 + slope1;
                        shade2 = shade2 + slope2;
                    end
                end
            end
        end
    endfunction

    function [31:0] shade_word;
        input signed [20:0] shade_start;
        input signed [20:0] slope;
        input [11:0] dst_x;
        input [2:0] count;
        input x_down;
        integer i;
        reg [11:0] px;
        reg signed [20:0] shade;
        reg [7:0] pixel;
        begin
            shade_word = 32'h0;
            shade = shade_start;
            for (i = 0; i < 4; i = i + 1) begin
                if (i < count) begin
                    px = x_down ? (dst_x - i[11:0]) :
                                  (dst_x + i[11:0]);
                    pixel = shade_ci8_pixel(shade);
                    case (px[1:0])
                        2'd0: shade_word[31:24] = pixel;
                        2'd1: shade_word[23:16] = pixel;
                        2'd2: shade_word[15:8] = pixel;
                        default: shade_word[7:0] = pixel;
                    endcase
                    shade = shade + slope;
                end
            end
        end
    endfunction

    function signed [20:0] shade_advance;
        input signed [20:0] shade_start;
        input signed [20:0] slope;
        input [2:0] count;
        reg signed [20:0] value;
        integer i;
        begin
            value = shade_start;
            for (i = 0; i < 4; i = i + 1)
                if (i < count)
                    value = value + slope;
            shade_advance = value;
        end
    endfunction

    function [3:0] rwaux_allowed_source_mask;
        input [3:0] source_mask;
        input [11:0] start_px;
        input [11:0] row_y;
        input [11:0] x0;
        input [11:0] x1;
        input        x_down;
        input        block_mode;
        integer i;
        reg [11:0] dst_x;
        begin
            rwaux_allowed_source_mask = 4'h0;
            for (i = 0; i < 4; i = i + 1) begin
                dst_x = x_down ? (start_px - i[11:0]) :
                                 (start_px + i[11:0]);
                if (source_mask[i] && (row_y < 12'd1024) &&
                    (dst_x < 12'd1024) &&
                    block_x_allowed(dst_x, x0, x1, x_down, block_mode))
                    rwaux_allowed_source_mask[i] = 1'b1;
            end
        end
    endfunction

    function [11:0] rwaux_anchor_x;
        input [3:0] source_mask;
        input [11:0] start_px;
        input        x_down;
        integer i;
        reg found;
        begin
            rwaux_anchor_x = start_px;
            found = 1'b0;
            for (i = 0; i < 4; i = i + 1) begin
                if (source_mask[i] && !found) begin
                    rwaux_anchor_x = x_down ? (start_px - i[11:0]) :
                                             (start_px + i[11:0]);
                    found = 1'b1;
                end
            end
        end
    endfunction

    function [3:0] rwaux_word_source_mask;
        input [3:0] source_mask;
        input [11:0] start_px;
        input [11:0] anchor_px;
        input        x_down;
        integer i;
        reg [11:0] dst_x;
        begin
            rwaux_word_source_mask = 4'h0;
            for (i = 0; i < 4; i = i + 1) begin
                dst_x = x_down ? (start_px - i[11:0]) :
                                 (start_px + i[11:0]);
                if (source_mask[i] &&
                    (dst_x[11:2] == anchor_px[11:2]))
                    rwaux_word_source_mask[i] = 1'b1;
            end
        end
    endfunction

    function [31:0] rwaux_word_data;
        input [31:0] data;
        input [3:0] source_mask;
        input [11:0] start_px;
        input        x_down;
        integer i;
        reg [11:0] dst_x;
        begin
            rwaux_word_data = 32'h0;
            for (i = 0; i < 4; i = i + 1) begin
                dst_x = x_down ? (start_px - i[11:0]) :
                                 (start_px + i[11:0]);
                if (source_mask[i]) begin
                    case (dst_x[1:0])
                        2'd0: rwaux_word_data[31:24] = lane_data(data, i[1:0]);
                        2'd1: rwaux_word_data[23:16] = lane_data(data, i[1:0]);
                        2'd2: rwaux_word_data[15:8] = lane_data(data, i[1:0]);
                        default: rwaux_word_data[7:0] = lane_data(data, i[1:0]);
                    endcase
                end
            end
        end
    endfunction

    function [3:0] rwaux_word_lane_mask;
        input [3:0] source_mask;
        input [11:0] start_px;
        input        x_down;
        integer i;
        reg [11:0] dst_x;
        begin
            rwaux_word_lane_mask = 4'h0;
            for (i = 0; i < 4; i = i + 1) begin
                dst_x = x_down ? (start_px - i[11:0]) :
                                 (start_px + i[11:0]);
                if (source_mask[i])
                    rwaux_word_lane_mask[dst_x[1:0]] = 1'b1;
            end
        end
    endfunction

    function [2:0] copy_group_count;
        input [11:0] dst_x;
        input [11:0] end_x;
        input        x_down;
        reg [11:0] distance;
        reg [ 2:0] word_room;
        reg [ 2:0] rect_room;
        begin
            if (x_down) begin
                distance = dst_x - end_x;
                word_room = {1'b0, dst_x[1:0]} + 3'd1;
            end else begin
                distance = end_x - dst_x;
                word_room = 3'd4 - {1'b0, dst_x[1:0]};
            end

            rect_room = (distance >= 12'd3) ? 3'd4 :
                        ({1'b0, distance[1:0]} + 3'd1);

            if (word_room < rect_room)
                copy_group_count = word_room;
            else
                copy_group_count = rect_room;
        end
    endfunction

    function [3:0] copy_lane_mask;
        input [11:0] dst_x;
        input [ 2:0] count;
        input        x_down;
        integer i;
        reg [1:0] lane;
        begin
            copy_lane_mask = 4'h0;
            for (i = 0; i < 4; i = i + 1) begin
                if (i < count) begin
                    if (x_down)
                        lane = dst_x[1:0] - i[1:0];
                    else
                        lane = dst_x[1:0] + i[1:0];
                    copy_lane_mask[lane] = 1'b1;
                end
            end
        end
    endfunction

    function [3:0] copy_visible_lane_mask;
        input [11:0] dst_x;
        input [11:0] src_x;
        input [11:0] dst_y;
        input [11:0] src_y;
        input [ 2:0] count;
        input        x_down;
        integer i;
        reg [11:0] lane_dst_x;
        reg [11:0] lane_src_x;
        begin
            copy_visible_lane_mask = 4'h0;
            if ((dst_y < 12'd1024) && (src_y < 12'd1024)) begin
                for (i = 0; i < 4; i = i + 1) begin
                    if (i < count) begin
                        if (x_down) begin
                            lane_dst_x = dst_x - i[11:0];
                            lane_src_x = src_x - i[11:0];
                        end else begin
                            lane_dst_x = dst_x + i[11:0];
                            lane_src_x = src_x + i[11:0];
                        end
                        if ((lane_dst_x < 12'd1024) &&
                            (lane_src_x < 12'd1024))
                            copy_visible_lane_mask[lane_dst_x[1:0]] = 1'b1;
                    end
                end
            end
        end
    endfunction

    wire [11:0] rwaux_active_x =
        rwaux_anchor_x(rwaux_pending_mask_r, base_x_r, x_down_r);
    wire [3:0] rwaux_active_source_mask =
        rwaux_word_source_mask(rwaux_pending_mask_r, base_x_r,
                               rwaux_active_x, x_down_r);
    wire [3:0] rwaux_active_lane_mask =
        rwaux_word_lane_mask(rwaux_active_source_mask, base_x_r, x_down_r);
    wire [31:0] rwaux_source_data =
        (command_r[14] && !quadmode_r) ?
        {rgb_rwaux_pixel(data_r, command_r[15], base_x_r, base_y_r), 24'h0} :
        data_r;
    wire [31:0] rwaux_active_data =
        rwaux_word_data(rwaux_source_data, rwaux_active_source_mask,
                        base_x_r, x_down_r);
    // With STOPONX and no XYCONTINUE, REX repeats one host quad across the
    // programmed span.  XYCONTINUE instead advances once per host transfer in
    // the frontend, so it must never enter this local repeat loop.
    wire rwaux_repeat_span = command_r[8] && !command_r[7];
    wire [11:0] rwaux_span_step = quadmode_r ? 12'd4 : 12'd1;
    wire [11:0] rwaux_next_span_x = x_down_r ?
        (base_x_r - rwaux_span_step) : (base_x_r + rwaux_span_step);
    wire [3:0] rwaux_next_span_mask = rwaux_allowed_source_mask(
        rwaux_source_mask_r, rwaux_next_span_x, base_y_r,
        xstart_r, xend_r, x_down_r, 1'b1);
    wire rwaux_word_done =
        (rwaux_pending_mask_r == 4'h0) ||
        ((rwaux_pending_mask_r & ~rwaux_active_source_mask) == 4'h0);
    wire fill_visible =
        (fill_x_r < 12'd1024) &&
        (fill_y_r < 12'd1024) &&
        block_x_allowed(fill_x_r, fill_xstart_r, fill_xend_r,
                        fill_x_down_r, 1'b1);
    wire [11:0] pattern_x_full = pattern_x_r;
    wire pattern_on = data_r[31 - pattern_bit_r[4:0]];
    wire pattern_write = pattern_on || pattern_opaque_r;
    // LRQPOLY is a one-coverage-sample stream, not a packed 32-bit bitmap.
    // The frontend supplies xstart=xend for each transparent sample so it
    // still emits the ordered zero-mask retirement marker at this boundary.
    wire pattern_limit_x =
        block_r || command_r[8] || ((command_r & CMD_LRQPOLY) != 32'h0);
    wire pattern_at_x_end =
        pattern_limit_x && (pattern_x_full == xend_r);
    wire pattern_visible =
        pattern_write &&
        (base_y_r < 12'd1024) &&
        (pattern_x_full < 12'd1024) &&
        block_x_allowed(pattern_x_full, xstart_r, xend_r,
                        x_down_r, pattern_limit_x);
    // The two shipped LG1 producers use different per-axis XYMOVE conventions
    // after LIGHT converts software Y into inverted REX coordinates.  LIGHT
    // Lg1TpBmove writes destination-source in X, so recover source X by
    // subtraction, but its software Y delta becomes source REX Y by addition.
    // The host copy path writes source-destination on both axes and is selected
    // by its zero COMMAND ROP. Keep the X decision with the queued
    // command snapshot; both producers recover source Y by addition.
    wire copy_xstate_rop = command_r[31:28] == 4'h0;
    wire [11:0] copy_src_x_now = copy_xstate_rop ?
        (fill_x_r + copy_dx_r) : (fill_x_r - copy_dx_r);
    wire [11:0] copy_src_y_now = fill_y_r + copy_dy_r;
    wire [2:0] rect_emit_count =
        active_copy ? copy_group_count(fill_x_r, fill_xend_r,
                                       fill_x_down_r) :
        (active_fill ? copy_group_count(fill_x_r, fill_xend_r,
                                        fill_x_down_r) :
                       3'd1);
    wire [3:0] active_rect_lane_mask =
        active_copy ? copy_visible_lane_mask(fill_x_r, copy_src_x_now,
                                             fill_y_r, copy_src_y_now,
                                             rect_emit_count, fill_x_down_r) :
                      copy_lane_mask(fill_x_r, rect_emit_count,
                                     fill_x_down_r);
    wire active_rect = active_fill || active_copy;
    wire rect_group_at_x_end =
        fill_x_down_r ?
            ((fill_x_r - {9'h0, rect_emit_count - 3'd1}) == fill_xend_r) :
            ((fill_x_r + {9'h0, rect_emit_count - 3'd1}) == fill_xend_r);
    wire fill_at_y_end = fill_y_r == fill_yend_r;
    wire [11:0] rect_x_step = {9'h0, rect_emit_count};

    wire rwaux_last = active_rwaux && rwaux_word_done &&
        (!rwaux_repeat_span || (rwaux_next_span_mask == 4'h0));
    // ENZPAT consumes at most one 32-bit host bitmap word.  ENLSPAT is a
    // programmable line stipple and repeats to STOPONX even when software
    // leaves XYCONTINUE set; one GO.LSPATTERN may cover a long title span.
    wire pattern_word_limited = !command_r[11];
    wire pattern_word_last = pattern_word_limited &&
                             (pattern_step_r == 6'd31);
    wire pattern_row_last = pattern_word_last || pattern_at_x_end;
    // A non-XYCONTINUE block pattern is one primitive spanning all rows.
    // Host-streamed XYCONTINUE words advance in the frontend instead, so they
    // must retain their one-segment-per-context behavior.
    wire pattern_has_next_block_row =
        block_r && !command_r[7] && (base_y_r != fill_yend_r);
    wire pattern_last = active_pattern && !pattern_tail_r &&
                        pattern_row_last && !pattern_has_next_block_row;
    wire rect_last = active_rect && rect_group_at_x_end && fill_at_y_end;
    wire line_last = active_line &&
        (line_xmajor_r ? (line_x_r == line_xend_r) :
                         (line_y_r == line_yend_r));
    wire line_pattern_on = data_r[31 - pattern_bit_r[4:0]];
    wire line_pattern_write = !command_r[11] || line_pattern_on ||
                              pattern_opaque_r;
    wire line_visible = line_pattern_write &&
                        (line_x_r < 12'd1024) &&
                        (line_y_r < 12'd1024);
    wire [7:0] line_fg_color = source_pixel(
        command_r[14], command_r[15], shade_r, line_x_r, line_y_r,
        fill_shade_r, fill_shade1_r, fill_shade2_r, fill_color_r);
    wire [7:0] line_color =
        (command_r[11] && !line_pattern_on) ? pattern_bg_color_r :
                                             line_fg_color;
    wire [7:0] pattern_fg_color = source_pixel(
        command_r[14], command_r[15], shade_r, pattern_x_full, base_y_r,
        fill_shade_r, fill_shade1_r, fill_shade2_r, fill_color_r);
    wire [16:0] line_frac_sum =
        {1'b0, line_frac_accum_r} + {1'b0, line_frac_step_r};
    wire line_minor_advance = line_frac_sum > 17'h08000;
    wire [16:0] line_frac_adjusted = line_minor_advance ?
        (line_frac_sum - 17'h08000) : line_frac_sum;
    wire [15:0] line_frac_next = line_frac_adjusted[15:0];
    wire [5:0] lspat_next_bit =
        (lsmode_count_r == 8'h0) ?
            ((pattern_bit_r == lsmode_last_bit_r) ?
             6'h0 : (pattern_bit_r + 6'd1)) :
            pattern_bit_r;
    wire [7:0] lspat_next_count =
        (lsmode_count_r == 8'h0) ?
            (lsmode_repeat_r - 8'd1) :
            (lsmode_count_r - 8'd1);
    wire [5:0] lspat_terminal_bit =
        ((command_r & CMD_LSADVLAST) != 32'h0) ?
            lspat_next_bit : pattern_bit_r;
    wire [7:0] lspat_terminal_count =
        ((command_r & CMD_LSADVLAST) != 32'h0) ?
            lspat_next_count : lsmode_count_r;
    wire [5:0] ls_work_length_code = ls_work_last_bit_r - 6'd16;

    assign ls_work_valid = ls_work_valid_r;
    assign ls_work_pattern = ls_work_pattern_r;
    assign ls_work_mode = ls_work_valid_r ?
        {ls_work_length_code[3:0],
         ls_work_repeat_r, ls_work_count_r} : 20'h0;

    // A terminal but fully clipped context still emits one zero-mask marker.
    // That marker cannot write memory, but it preserves ordered retirement
    // behind every earlier RB transaction.
    assign pix_valid =
        (active_rwaux && ((rwaux_active_lane_mask != 4'h0) || rwaux_last)) ||
        ((active_fill || active_copy) && (fill_visible || rect_last)) ||
        (active_pattern && (pattern_visible || pattern_last)) ||
        (active_line && (line_visible || line_last));
    assign pix_x = active_line ? line_x_r :
                   ((active_fill || active_copy) ? fill_x_r :
                   (active_pattern ? pattern_x_full : rwaux_active_x));
    assign pix_y = active_line ? line_y_r :
                   ((active_fill || active_copy) ? fill_y_r : base_y_r);
    assign pix_data = active_line ?
                      {line_color, line_color, line_color, line_color} :
                      (active_pattern ?
                      pixel_word(pattern_x_full[1:0],
                                 pattern_on ? pattern_fg_color :
                                              pattern_bg_color_r) :
                      (active_fill ?
                       source_word(command_r[14], command_r[15], shade_r,
                                   fill_shade_r, fill_shade1_r, fill_shade2_r,
                                   fill_slope_r, fill_slope1_r, fill_slope2_r,
                                   fill_x_r, fill_y_r, rect_emit_count,
                                   fill_x_down_r, fill_color_r) :
                       (active_copy ?
                       {fill_color_r, fill_color_r,
                        fill_color_r, fill_color_r} : rwaux_active_data)));
    assign pix_plane = (active_fill || active_copy || active_line) ?
                       fill_plane_r : plane_r;
    assign pix_mask = active_line ?
                      (line_visible ? (4'h1 << line_x_r[1:0]) : 4'h0) :
                      ((active_fill || active_copy) ?
                      (fill_visible ? active_rect_lane_mask : 4'h0) :
                      (active_pattern ?
                       (pattern_visible ? (4'h1 << pattern_x_full[1:0]) : 4'h0) :
                       rwaux_active_lane_mask));
    assign pix_copy = active_copy;
    assign pix_src_x = copy_src_x_now;
    assign pix_src_y = copy_src_y_now;
    assign pix_aux2 = aux2_r;
    assign pix_smask = smask_r;
    assign pix_logic_op = logic_op_r;
    assign pix_pixel_writemask = pixel_writemask_r;
    assign pix_colorcomp = colorcomp_r;
    assign pix_aux1 = aux1_r;
    assign pix_rgbmode = command_r[14];
    assign pix_rwaux = active_rwaux;
    assign pix_last = rwaux_last || pattern_last || rect_last || line_last;
    assign pix_retire_x = active_line ? line_x_r : retire_x_r;
    assign pix_retire_y = active_line ? line_y_r : retire_y_r;
    assign pix_retire_done = active_line ? line_last : retire_done_r;
    assign busy = active_rwaux || active_fill || active_pattern ||
                  active_copy || active_line || pix_valid;

    wire advance_rwaux = active_rwaux && pix_valid && pix_ready;
    wire advance_rect = active_rect &&
                        (fill_visible ? pix_ready : (!rect_last || pix_ready));
    wire advance_pattern = active_pattern &&
                           (pattern_visible ? pix_ready :
                            (!pattern_last || pix_ready));
    wire advance_line = active_line &&
                        (line_visible ? pix_ready :
                         (!line_last || pix_ready));
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_rwaux <= 1'b0;
            active_fill <= 1'b0;
            active_pattern <= 1'b0;
            active_copy <= 1'b0;
            active_line <= 1'b0;
            command_r <= 32'h0;
            data_r <= 32'h0;
            rwaux_source_mask_r <= 4'h0;
            rwaux_pending_mask_r <= 4'h0;
            base_x_r <= 12'h0;
            base_y_r <= 12'h0;
            xstart_r <= 12'h0;
            xend_r <= 12'h0;
            x_down_r <= 1'b0;
            block_r <= 1'b0;
            quadmode_r <= 1'b0;
            plane_r <= 2'h0;
            fill_x_r <= 12'h0;
            fill_y_r <= 12'h0;
            fill_xstart_r <= 12'h0;
            fill_xend_r <= 12'h0;
            fill_yend_r <= 12'h0;
            fill_x_down_r <= 1'b0;
            fill_y_down_r <= 1'b0;
            fill_plane_r <= 2'h0;
            fill_color_r <= 8'h0;
            shade_r <= 1'b0;
            fill_shade_r <= 21'sh0;
            fill_shade1_r <= 21'sh0;
            fill_shade2_r <= 21'sh0;
            fill_slope_r <= 21'sh0;
            fill_slope1_r <= 21'sh0;
            fill_slope2_r <= 21'sh0;
            pattern_bit_r <= 6'h0;
            pattern_step_r <= 6'h0;
            pattern_x_r <= 12'h0;
            lsmode_count_r <= 8'h0;
            lsmode_initial_count_r <= 8'h0;
            lsmode_initial_bit_r <= 6'h0;
            lsmode_repeat_r <= 8'h1;
            lsmode_last_bit_r <= 6'd16;
            ls_work_valid_r <= 1'b0;
            ls_work_pattern_r <= 32'h0;
            ls_work_count_r <= 8'h0;
            ls_work_repeat_r <= 8'h1;
            ls_work_last_bit_r <= 6'd16;
            pattern_tail_r <= 1'b0;
            pattern_y_down_r <= 1'b0;
            pattern_opaque_r <= 1'b0;
            pattern_bg_color_r <= 8'h0;
            copy_dx_r <= 12'h0;
            copy_dy_r <= 12'h0;
            aux2_r <= 32'h0;
            smask_r <= 256'h0;
            logic_op_r <= 4'h3;
            pixel_writemask_r <= 8'hff;
            colorcomp_r <= 1'b0;
            aux1_r <= 10'h0;
            retire_x_r <= 12'h0;
            retire_y_r <= 12'h0;
            retire_done_r <= 1'b0;
            line_x_r <= 12'h0;
            line_y_r <= 12'h0;
            line_xend_r <= 12'h0;
            line_yend_r <= 12'h0;
            line_xmajor_r <= 1'b0;
            line_minor_negative_r <= 1'b0;
            line_frac_step_r <= 16'h0;
            line_frac_accum_r <= 16'h0;
        end else begin
            if (advance_rwaux) begin
                if (rwaux_word_done) begin
                    if (rwaux_repeat_span &&
                        (rwaux_next_span_mask != 4'h0)) begin
                        base_x_r <= rwaux_next_span_x;
                        rwaux_pending_mask_r <= rwaux_next_span_mask;
                    end else begin
                        active_rwaux <= 1'b0;
                        rwaux_pending_mask_r <= 4'h0;
                    end
                end else begin
                    rwaux_pending_mask_r <=
                        rwaux_pending_mask_r & ~rwaux_active_source_mask;
                end
            end else if (advance_pattern) begin
                if (shade_r) begin
                    fill_shade_r <= fill_shade_r + fill_slope_r;
                    fill_shade1_r <= fill_shade1_r + fill_slope1_r;
                    fill_shade2_r <= fill_shade2_r + fill_slope2_r;
                end
                if (pattern_row_last) begin
                    if (pattern_tail_r) begin
                        pattern_tail_r <= 1'b0;
                        pattern_bit_r <= lsmode_initial_bit_r;
                        pattern_step_r <= 6'h0;
                        pattern_x_r <= base_x_r;
                        lsmode_count_r <= lsmode_initial_count_r;
                        if (pattern_y_down_r)
                            base_y_r <= base_y_r - 12'd1;
                        else
                            base_y_r <= base_y_r + 12'd1;
                    end else if (pattern_has_next_block_row) begin
                        pattern_bit_r <= lsmode_initial_bit_r;
                        pattern_step_r <= 6'h0;
                        pattern_x_r <= xstart_r;
                        lsmode_count_r <= lsmode_initial_count_r;
                        if (pattern_y_down_r)
                            base_y_r <= base_y_r - 12'd1;
                        else
                            base_y_r <= base_y_r + 12'd1;
                    end else begin
                        active_pattern <= 1'b0;
                        if ((command_r & CMD_ENLSPAT) != 32'h0) begin
                            ls_work_valid_r <= 1'b1;
                            ls_work_pattern_r <= rotate_lspattern(
                                data_r, lspat_terminal_bit,
                                lsmode_last_bit_r);
                            ls_work_count_r <= lspat_terminal_count;
                            ls_work_repeat_r <= lsmode_repeat_r;
                            ls_work_last_bit_r <= lsmode_last_bit_r;
                        end
                    end
                end else begin
                    pattern_step_r <= pattern_step_r + 6'd1;
                    if (x_down_r)
                        pattern_x_r <= pattern_x_r - 12'd1;
                    else
                        pattern_x_r <= pattern_x_r + 12'd1;
                    if (command_r[11]) begin
                        pattern_bit_r <= lspat_next_bit;
                        lsmode_count_r <= lspat_next_count;
                    end else begin
                        pattern_bit_r <= pattern_bit_r + 6'd1;
                    end
                end
            end else if (advance_line) begin
                if (shade_r) begin
                    fill_shade_r <= fill_shade_r + fill_slope_r;
                    fill_shade1_r <= fill_shade1_r + fill_slope1_r;
                    fill_shade2_r <= fill_shade2_r + fill_slope2_r;
                end
                if (line_last) begin
                    active_line <= 1'b0;
                    if ((command_r & CMD_ENLSPAT) != 32'h0) begin
                        ls_work_valid_r <= 1'b1;
                        ls_work_pattern_r <= rotate_lspattern(
                            data_r, lspat_terminal_bit,
                            lsmode_last_bit_r);
                        ls_work_count_r <= lspat_terminal_count;
                        ls_work_repeat_r <= lsmode_repeat_r;
                        ls_work_last_bit_r <= lsmode_last_bit_r;
                    end
                end else begin
                    line_frac_accum_r <= line_frac_next;
                    if (line_xmajor_r) begin
                        if (fill_x_down_r)
                            line_x_r <= line_x_r - 12'd1;
                        else
                            line_x_r <= line_x_r + 12'd1;
                        if (line_minor_advance) begin
                            if (line_minor_negative_r)
                                line_y_r <= line_y_r - 12'd1;
                            else
                                line_y_r <= line_y_r + 12'd1;
                        end
                    end else begin
                        if (fill_y_down_r)
                            line_y_r <= line_y_r - 12'd1;
                        else
                            line_y_r <= line_y_r + 12'd1;
                        if (line_minor_advance) begin
                            if (line_minor_negative_r)
                                line_x_r <= line_x_r - 12'd1;
                            else
                                line_x_r <= line_x_r + 12'd1;
                        end
                    end
                    if (command_r[11]) begin
                        pattern_bit_r <= lspat_next_bit;
                        lsmode_count_r <= lspat_next_count;
                    end
                end
            end else if (advance_rect) begin
                if (active_fill && shade_r) begin
                    fill_shade_r <= shade_advance(fill_shade_r,
                                                  fill_slope_r,
                                                  rect_emit_count);
                    fill_shade1_r <= shade_advance(fill_shade1_r,
                                                   fill_slope1_r,
                                                   rect_emit_count);
                    fill_shade2_r <= shade_advance(fill_shade2_r,
                                                   fill_slope2_r,
                                                   rect_emit_count);
                end
                if (rect_group_at_x_end && fill_at_y_end) begin
                    active_fill <= 1'b0;
                    active_copy <= 1'b0;
                end else if (rect_group_at_x_end) begin
                    fill_x_r <= fill_xstart_r;
                    if (fill_y_down_r)
                        fill_y_r <= fill_y_r - 12'd1;
                    else
                        fill_y_r <= fill_y_r + 12'd1;
                end else if (fill_x_down_r) begin
                    fill_x_r <= fill_x_r - rect_x_step;
                end else begin
                    fill_x_r <= fill_x_r + rect_x_step;
                end
            end else if (!active_rwaux && !active_fill && !active_pattern &&
                         !active_copy && !active_line && ctx_valid) begin
                active_rwaux <= ctx_rwaux_write;
                active_fill <= ctx_solid_fill;
                active_pattern <= ctx_pattern_fill;
                active_copy <= ctx_copy_fill;
                active_line <= ctx_line_draw;
                command_r <= ctx_command;
                data_r <= ctx_data;
                rwaux_source_mask_r <=
                    ctx_quadmode ? ctx_lane_mask : (ctx_lane_mask & 4'h1);
                rwaux_pending_mask_r <= rwaux_allowed_source_mask(
                    ctx_quadmode ? ctx_lane_mask : (ctx_lane_mask & 4'h1),
                    ctx_x, ctx_y, ctx_xstart, ctx_xend,
                    ctx_x_down,
                    // BLOCK/STOPONX commands already carry endpoint
                    // semantics. The frontend also marks the draw-image
                    // scanline form, whose 0x002000a1
                    // command is non-BLOCK/non-STOPONX but still clips the
                    // padded final host word to its freshly loaded XEND.
                    ctx_block ||
                    (ctx_command[8] && !ctx_command[7]) ||
                    ctx_clip_xend);
                base_x_r <= ctx_x;
                base_y_r <= ctx_y;
                xstart_r <= ctx_xstart;
                xend_r <= ctx_xend;
                x_down_r <= ctx_x_down;
                block_r <= ctx_block;
                quadmode_r <= ctx_quadmode;
                plane_r <= ctx_plane;
                fill_x_r <= ctx_x;
                fill_y_r <= ctx_y;
                fill_xstart_r <= ctx_xstart;
                fill_xend_r <= ctx_xend;
                fill_yend_r <= ctx_yend;
                fill_x_down_r <= ctx_x_down;
                fill_y_down_r <= ctx_y_down;
                fill_plane_r <= ctx_plane;
                fill_color_r <= ctx_color;
                shade_r <= ctx_shade;
                fill_shade_r <= $signed({1'b0, ctx_color_fixed});
                fill_shade1_r <= $signed({1'b0, ctx_color1_fixed});
                fill_shade2_r <= $signed({1'b0, ctx_color2_fixed});
                fill_slope_r <= $signed(ctx_slope);
                fill_slope1_r <= $signed(ctx_slope1);
                fill_slope2_r <= $signed(ctx_slope2);
                pattern_bit_r <= 6'h0;
                pattern_step_r <= 6'h0;
                pattern_x_r <= ctx_x;
                lsmode_count_r <= ctx_lsmode[7:0];
                lsmode_initial_count_r <= ctx_lsmode[7:0];
                lsmode_initial_bit_r <= 6'h0;
                lsmode_repeat_r <= (ctx_lsmode[15:8] == 8'h0) ?
                                   8'h1 : ctx_lsmode[15:8];
                lsmode_last_bit_r <= 6'd16 +
                                     {2'b00, ctx_lsmode[19:16]};
                if ((ctx_line_draw || ctx_pattern_fill) &&
                    ((ctx_command & CMD_ENLSPAT) != 32'h0) &&
                    ((ctx_command & CMD_LSCONTINUE) != 32'h0) &&
                    !ctx_ls_reload && ls_work_valid_r) begin
                    data_r <= ls_work_pattern_r;
                    pattern_bit_r <= 6'h0;
                    lsmode_count_r <= ls_work_count_r;
                    lsmode_initial_count_r <= ls_work_count_r;
                    lsmode_initial_bit_r <= 6'h0;
                    lsmode_repeat_r <= ls_work_repeat_r;
                    lsmode_last_bit_r <= ls_work_last_bit_r;
                end
                pattern_tail_r <= ctx_pattern_tail;
                pattern_y_down_r <= ctx_y_down;
                pattern_opaque_r <= ctx_pattern_opaque;
                pattern_bg_color_r <= ctx_bg_color;
                copy_dx_r <= ctx_copy_dx;
                copy_dy_r <= ctx_copy_dy;
                aux2_r <= ctx_aux2;
                smask_r <= ctx_smask;
                logic_op_r <= ctx_logic_op;
                pixel_writemask_r <= ctx_pixel_writemask;
                colorcomp_r <= ctx_colorcomp;
                aux1_r <= ctx_aux1;
                retire_x_r <= ctx_retire_x;
                retire_y_r <= ctx_retire_y;
                retire_done_r <= ctx_retire_done;
                line_x_r <= ctx_x;
                line_y_r <= ctx_y;
                line_xend_r <= ctx_xend;
                line_yend_r <= ctx_yend;
                line_xmajor_r <= ctx_command[6];
                line_minor_negative_r <= ctx_minor_negative;
                line_frac_step_r <= ctx_minor_slope[16] ?
                    (~ctx_minor_slope[15:0] + 16'd1) :
                    ctx_minor_slope[15:0];
                // Fixed-coordinate userland lines launch with INITFRAC clear;
                // their minor-axis start fraction is part of XSTARTF/YSTARTF
                // and must survive the context queue. Integer INITFRAC lines
                // retain LIGHT's half-pixel initialization.
                line_frac_accum_r <= {1'b0, ctx_line_frac};
            end

            // Context selection supersedes any same-cycle retirement from
            // the context being switched out.
            if (ls_work_invalidate)
                ls_work_valid_r <= 1'b0;
        end
    end

    wire unused_inputs = &{1'b0, ctx_command, ctx_block, ctx_clip_xend};
    wire unused_reduce = unused_inputs;
endmodule

`default_nettype wire
