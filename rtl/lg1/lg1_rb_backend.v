// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - LG1 clean RB-like memory backend

`default_nettype none

module lg1_rb_backend (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        pix_valid,
    output wire        pix_ready,
    input  wire [11:0] pix_x,
    input  wire [11:0] pix_y,
    // Four adjacent CI8 lanes per ready/valid beat.  The word containing
    // pix_x is addressed and pix_mask selects its destination byte lanes.
    input  wire [31:0] pix_data,
    input  wire [ 1:0] pix_plane,
    input  wire [ 3:0] pix_mask,
    input  wire        pix_copy,
    input  wire [11:0] pix_src_x,
    input  wire [11:0] pix_src_y,
    input  wire [31:0] pix_aux2,
    input  wire [255:0] pix_smask,
    input  wire [ 3:0] pix_logic_op,
    input  wire [ 7:0] pix_pixel_writemask,
    input  wire        pix_colorcomp,
    input  wire [ 9:0] pix_aux1,
    input  wire        pix_rgbmode,
    input  wire        pix_rwaux,
    input  wire        pix_last,
    input  wire [11:0] pix_retire_x,
    input  wire [11:0] pix_retire_y,
    input  wire        pix_retire_done,

    input  wire        load_valid,
    output wire        load_ready,
    input  wire [11:0] load_x,
    input  wire [11:0] load_y,
    input  wire [11:0] load_xend,
    input  wire        load_x_down,
    input  wire        load_block,
    input  wire        load_quadmode,
    input  wire        load_exhausted,
    input  wire [ 1:0] load_plane,
    input  wire [ 9:0] load_aux1,
    input  wire        load_is_pixel,
    output wire        load_rsp_valid,
    output reg  [31:0] load_rsp_data,
    input  wire        load_rsp_ready,

    output reg         mem_req_valid,
    input  wire        mem_req_ready,
    output reg         mem_req_write,
    output reg  [ 1:0] mem_req_plane,
    output reg  [17:0] mem_req_addr,
    output reg  [31:0] mem_req_wdata,
    output reg  [ 3:0] mem_req_wstrb,
    input  wire        mem_rsp_valid,
    input  wire [31:0] mem_rsp_rdata,

    output reg         retire_valid,
    output reg  [11:0] retire_x,
    output reg  [11:0] retire_y,
    output reg         retire_done,

    output wire        busy
);
    localparam [3:0] S_IDLE = 4'd0;
    localparam [3:0] S_COPY_WAIT = 4'd1;
    localparam [3:0] S_CID_WAIT = 4'd2;
    localparam [3:0] S_RMW_WAIT = 4'd3;
    localparam [3:0] S_WRITE = 4'd4;
    localparam [3:0] S_LOAD_REQ = 4'd5;
    localparam [3:0] S_LOAD_WAIT = 4'd6;
    localparam [3:0] S_LOAD_RSP = 4'd7;
    localparam [3:0] S_COPY_SECOND_WAIT = 4'd8;

    reg [3:0] state;
    reg [17:0] copy_dst_addr;
    reg [17:0] copy_src_addr0;
    reg [17:0] copy_src_addr1;
    reg [31:0] copy_src_word0;
    reg        copy_second_needed;
    reg [ 1:0] copy_plane;
    reg [ 3:0] copy_lane_mask;
    reg [11:0] copy_dst_x;
    reg [11:0] copy_dst_y;
    reg [11:0] copy_src_x;
    reg [11:0] copy_src_y;
    reg [31:0] copy_aux2;
    reg [255:0] copy_smask;
    reg [ 3:0] copy_logic_op;
    reg [ 7:0] copy_pixel_writemask;
    reg        copy_colorcomp;
    reg [ 9:0] copy_aux1;
    reg        copy_last;
    reg [11:0] copy_retire_x;
    reg [11:0] copy_retire_y;
    reg        copy_retire_done;
    reg [11:0] pending_x;
    reg [11:0] pending_y;
    reg [ 1:0] pending_plane;
    reg [31:0] pending_wdata;
    reg [ 3:0] pending_wstrb;
    reg [31:0] pending_aux2;
    reg [255:0] pending_smask;
    reg [ 3:0] pending_logic_op;
    reg [ 7:0] pending_pixel_writemask;
    reg        pending_colorcomp;
    reg [ 9:0] pending_aux1;
    reg        pending_last;
    reg [11:0] pending_retire_x;
    reg [11:0] pending_retire_y;
    reg        pending_retire_done;
    reg [11:0] load_cur_x;
    reg [11:0] load_cur_y;
    reg [11:0] load_end_x;
    reg        load_dir_down;
    reg        load_is_block;
    reg        load_is_quad;
    reg        load_is_exhausted;
    reg [ 1:0] load_selected_plane;
    reg [ 9:0] load_selected_aux1;
    reg        load_selected_is_pixel;
    reg [ 1:0] load_index;
    reg [31:0] load_assembled;

    assign pix_ready = (state == S_IDLE) && !mem_req_valid;
    assign load_ready = (state == S_IDLE) && !mem_req_valid && !pix_valid;
    assign load_rsp_valid = (state == S_LOAD_RSP);
    assign busy = (state != S_IDLE) || mem_req_valid || pix_valid ||
                  load_valid || retire_valid;

    function [17:0] pixel_mem_addr;
        input [11:0] px;
        input [11:0] py;
        begin
            pixel_mem_addr = {py[9:0], px[9:2]};
        end
    endfunction

    function [3:0] pixel_lane_strobe;
        input [1:0] lane;
        begin
            pixel_lane_strobe = 4'b0001 << lane;
        end
    endfunction

    function [31:0] pixel_lane_wdata;
        input [1:0] lane;
        input [7:0] pixel;
        begin
            case (lane)
                2'd0: pixel_lane_wdata = {pixel, 24'h0};
                2'd1: pixel_lane_wdata = {8'h0, pixel, 16'h0};
                2'd2: pixel_lane_wdata = {16'h0, pixel, 8'h0};
                default: pixel_lane_wdata = {24'h0, pixel};
            endcase
        end
    endfunction

    function [7:0] word_lane_data;
        input [31:0] data;
        input [ 1:0] lane;
        begin
            case (lane)
                2'd0: word_lane_data = data[31:24];
                2'd1: word_lane_data = data[23:16];
                2'd2: word_lane_data = data[15:8];
                default: word_lane_data = data[7:0];
            endcase
        end
    endfunction

    function [31:0] load_byte_insert;
        input [31:0] data;
        input [ 1:0] index;
        input [ 7:0] pixel;
        begin
            load_byte_insert = data;
            case (index)
                2'd0: load_byte_insert[31:24] = pixel;
                2'd1: load_byte_insert[23:16] = pixel;
                2'd2: load_byte_insert[15:8] = pixel;
                default: load_byte_insert[7:0] = pixel;
            endcase
        end
    endfunction

    function [7:0] logic_op_pixel;
        input [7:0] old_pixel;
        input [7:0] src_pixel;
        input [3:0] logic_op;
        begin
            case (logic_op)
                4'h0: logic_op_pixel = 8'h00;
                4'h1: logic_op_pixel = src_pixel & old_pixel;
                4'h2: logic_op_pixel = src_pixel & ~old_pixel;
                4'h3: logic_op_pixel = src_pixel;
                4'h4: logic_op_pixel = ~src_pixel & old_pixel;
                4'h5: logic_op_pixel = old_pixel;
                4'h6: logic_op_pixel = src_pixel ^ old_pixel;
                4'h7: logic_op_pixel = src_pixel | old_pixel;
                4'h8: logic_op_pixel = ~(src_pixel | old_pixel);
                4'h9: logic_op_pixel = ~(src_pixel ^ old_pixel);
                4'ha: logic_op_pixel = ~old_pixel;
                4'hb: logic_op_pixel = src_pixel | ~old_pixel;
                4'hc: logic_op_pixel = ~src_pixel;
                4'hd: logic_op_pixel = ~src_pixel | old_pixel;
                4'he: logic_op_pixel = ~(src_pixel & old_pixel);
                default: logic_op_pixel = 8'hff;
            endcase
        end
    endfunction

    // LG1 8/24 double buffering packs buffer 0 in the high nibble and buffer 1
    // in the low nibble. Convert generated CI4 or RGB121 into that logical
    // nibble, then replicate it before the destination mask selects buffers.
    function [7:0] amplify_doublebuffer_source_pixel;
        input [7:0] pixel;
        input [9:0] aux1_value;
        input       is_pixel;
        input       rgbmode;
        input       is_rwaux;
        reg [3:0] logical_pixel;
        begin
            // REX3 Table 22: 8/24-bit RGB double buffering stores two
            // RGB121 pixels as {G6,B7,R7,G7}; CI stores two CI4 values.
            logical_pixel = rgbmode ?
                {pixel[1], pixel[4], pixel[7], pixel[2]} : pixel[3:0];
            // LG1 host RGB RWAUX is a distinct 0xAABBGGRR,
            // one-word-per-pixel protocol.  lg1_draw_iter converts the
            // complete word to RGB332 before this packed-buffer stage; do
            // not amplify that converted byte as generated RGB121.
            if (is_pixel && aux1_value[5] && !(rgbmode && is_rwaux))
                amplify_doublebuffer_source_pixel =
                    {logical_pixel, logical_pixel};
            else
                amplify_doublebuffer_source_pixel = pixel;
        end
    endfunction

    function [31:0] amplify_doublebuffer_source_word;
        input [31:0] data;
        input [ 9:0] aux1_value;
        input        is_pixel;
        input        rgbmode;
        input        is_rwaux;
        begin
            amplify_doublebuffer_source_word = {
                amplify_doublebuffer_source_pixel(
                    data[31:24], aux1_value, is_pixel, rgbmode, is_rwaux),
                amplify_doublebuffer_source_pixel(
                    data[23:16], aux1_value, is_pixel, rgbmode, is_rwaux),
                amplify_doublebuffer_source_pixel(
                    data[15:8], aux1_value, is_pixel, rgbmode, is_rwaux),
                amplify_doublebuffer_source_pixel(
                    data[7:0], aux1_value, is_pixel, rgbmode, is_rwaux)
            };
        end
    endfunction

    // DBLSRC chooses which packed buffer supplies framebuffer reads and the
    // destination operand of a read/modify/write operation: clear selects
    // the low nibble, set selects high, matching native raw-byte readback.
    function [7:0] selected_doublebuffer_read_pixel;
        input [7:0] pixel;
        input [9:0] aux1_value;
        input       is_pixel;
        reg [3:0] selected;
        begin
            selected = aux1_value[4] ? pixel[7:4] : pixel[3:0];
            selected_doublebuffer_read_pixel =
                (is_pixel && aux1_value[5]) ?
                {selected, selected} : pixel;
        end
    endfunction

    function [31:0] selected_doublebuffer_read_word;
        input [31:0] data;
        input [ 9:0] aux1_value;
        input        is_pixel;
        begin
            selected_doublebuffer_read_word = {
                selected_doublebuffer_read_pixel(
                    data[31:24], aux1_value, is_pixel),
                selected_doublebuffer_read_pixel(
                    data[23:16], aux1_value, is_pixel),
                selected_doublebuffer_read_pixel(
                    data[15:8], aux1_value, is_pixel),
                selected_doublebuffer_read_pixel(
                    data[7:0], aux1_value, is_pixel)
            };
        end
    endfunction

    function [11:0] copy_source_x_for_lane;
        input [11:0] dst_anchor_x;
        input [11:0] src_anchor_x;
        input [ 1:0] dst_lane;
        reg [11:0] dst_lane_x;
        begin
            dst_lane_x = {dst_anchor_x[11:2], dst_lane};
            copy_source_x_for_lane =
                src_anchor_x + (dst_lane_x - dst_anchor_x);
        end
    endfunction

    function copy_needs_second_source;
        input [ 3:0] lane_mask;
        input [11:0] dst_anchor_x;
        input [11:0] src_anchor_x;
        input [11:0] src_y;
        integer i;
        reg [17:0] anchor_addr;
        reg [11:0] lane_src_x;
        begin
            anchor_addr = pixel_mem_addr(src_anchor_x, src_y);
            copy_needs_second_source = 1'b0;
            for (i = 0; i < 4; i = i + 1) begin
                lane_src_x = copy_source_x_for_lane(
                    dst_anchor_x, src_anchor_x, i[1:0]);
                if (lane_mask[i] &&
                    (pixel_mem_addr(lane_src_x, src_y) != anchor_addr))
                    copy_needs_second_source = 1'b1;
            end
        end
    endfunction

    function [17:0] copy_second_source_addr;
        input [ 3:0] lane_mask;
        input [11:0] dst_anchor_x;
        input [11:0] src_anchor_x;
        input [11:0] src_y;
        integer i;
        reg [17:0] anchor_addr;
        reg [11:0] lane_src_x;
        begin
            anchor_addr = pixel_mem_addr(src_anchor_x, src_y);
            copy_second_source_addr = anchor_addr;
            for (i = 0; i < 4; i = i + 1) begin
                lane_src_x = copy_source_x_for_lane(
                    dst_anchor_x, src_anchor_x, i[1:0]);
                if (lane_mask[i] &&
                    (pixel_mem_addr(lane_src_x, src_y) != anchor_addr))
                    copy_second_source_addr =
                        pixel_mem_addr(lane_src_x, src_y);
            end
        end
    endfunction

    function [31:0] assemble_copy_source_word;
        input [31:0] source_word0;
        input [31:0] source_word1;
        input [17:0] source_addr0;
        input [17:0] source_addr1;
        input [ 3:0] lane_mask;
        input [11:0] dst_anchor_x;
        input [11:0] src_anchor_x;
        input [11:0] src_y;
        input [ 9:0] aux1_value;
        input        is_pixel;
        integer i;
        reg [11:0] lane_src_x;
        reg [17:0] lane_src_addr;
        reg [31:0] selected_word;
        reg [ 7:0] selected_pixel;
        begin
            assemble_copy_source_word = 32'h0;
            for (i = 0; i < 4; i = i + 1) begin
                if (lane_mask[i]) begin
                    lane_src_x = copy_source_x_for_lane(
                        dst_anchor_x, src_anchor_x, i[1:0]);
                    lane_src_addr = pixel_mem_addr(lane_src_x, src_y);
                    selected_word = (lane_src_addr == source_addr0) ?
                                    source_word0 : source_word1;
                    selected_pixel = selected_doublebuffer_read_pixel(
                        word_lane_data(selected_word, lane_src_x[1:0]),
                        aux1_value, is_pixel);
                    case (i[1:0])
                        2'd0: assemble_copy_source_word[31:24] =
                                  selected_pixel;
                        2'd1: assemble_copy_source_word[23:16] =
                                  selected_pixel;
                        2'd2: assemble_copy_source_word[15:8] =
                                  selected_pixel;
                        default: assemble_copy_source_word[7:0] =
                                     selected_pixel;
                    endcase
                end
            end
        end
    endfunction

    // Host/DMA pixel reads expose the selected logical CI4 value, not the
    // replicated internal ROP operand used above.
    function [7:0] selected_doublebuffer_load_pixel;
        input [7:0] pixel;
        input [9:0] aux1_value;
        input       is_pixel;
        begin
            if (is_pixel && aux1_value[5])
                selected_doublebuffer_load_pixel =
                    {4'h0, aux1_value[4] ? pixel[7:4] : pixel[3:0]};
            else
                selected_doublebuffer_load_pixel = pixel;
        end
    endfunction

    function color_compare_pass;
        input       compare_enable;
        input [9:0] compare_aux1;
        input [7:0] old_pixel;
        input [7:0] src_pixel;
        begin
            if (!compare_enable) begin
                color_compare_pass = 1'b1;
            end else begin
                color_compare_pass = 1'b0;
                if (compare_aux1[6] && (src_pixel < old_pixel))
                    color_compare_pass = 1'b1;
                if (compare_aux1[7] && (src_pixel == old_pixel))
                    color_compare_pass = 1'b1;
                if (compare_aux1[8] && (src_pixel > old_pixel))
                    color_compare_pass = 1'b1;
            end
        end
    endfunction

    function [31:0] write_word_lane_data;
        input [31:0] data;
        input [ 1:0] lane;
        input [ 7:0] pixel;
        begin
            write_word_lane_data = data;
            case (lane)
                2'd0: write_word_lane_data[31:24] = pixel;
                2'd1: write_word_lane_data[23:16] = pixel;
                2'd2: write_word_lane_data[15:8] = pixel;
                default: write_word_lane_data[7:0] = pixel;
            endcase
        end
    endfunction

    function [31:0] apply_logic_word;
        input [31:0] old_word;
        input [31:0] src_word;
        input [ 3:0] lane_strobe;
        input [ 3:0] logic_op;
        input [ 7:0] pixel_writemask;
        input        colorcomp;
        input [ 9:0] aux1_value;
        input        is_pixel;
        integer i;
        reg [7:0] old_pixel;
        reg [7:0] logic_old_pixel;
        reg [7:0] src_pixel;
        reg [7:0] logic_pixel;
        reg [7:0] merged_pixel;
        begin
            apply_logic_word = old_word;
            for (i = 0; i < 4; i = i + 1) begin
                if (lane_strobe[i]) begin
                    old_pixel = word_lane_data(old_word, i[1:0]);
                    logic_old_pixel = selected_doublebuffer_read_pixel(
                        old_pixel, aux1_value, is_pixel);
                    src_pixel = word_lane_data(src_word, i[1:0]);
                    logic_pixel = logic_op_pixel(logic_old_pixel, src_pixel,
                                                 logic_op);
                    merged_pixel = (old_pixel & ~pixel_writemask) |
                                   (logic_pixel & pixel_writemask);
                    if (!color_compare_pass(colorcomp, aux1_value,
                                            logic_old_pixel, src_pixel))
                        merged_pixel = old_pixel;
                    apply_logic_word =
                        write_word_lane_data(apply_logic_word, i[1:0],
                                             merged_pixel);
                end
            end
        end
    endfunction

    function [31:0] smask_word;
        input [255:0] value;
        input [2:0] index;
        begin
            case (index)
                3'd0: smask_word = value[31:0];
                3'd1: smask_word = value[63:32];
                3'd2: smask_word = value[95:64];
                3'd3: smask_word = value[127:96];
                3'd4: smask_word = value[159:128];
                3'd5: smask_word = value[191:160];
                3'd6: smask_word = value[223:192];
                default: smask_word = value[255:224];
            endcase
        end
    endfunction

    function screen_mask_hit;
        input [11:0] px;
        input [11:0] py;
        input [31:0] sx;
        input [31:0] sy;
        begin
            screen_mask_hit =
                (px < 12'd1024) && (py < 12'd1024) &&
                (px[9:0] >= sx[9:0]) && (px[9:0] <= sx[25:16]) &&
                (py[9:0] >= sy[9:0]) && (py[9:0] <= sy[25:16]);
        end
    endfunction

    function cid_match_pass;
        input [3:0] cid;
        input [3:0] match_bits;
        begin
            if (match_bits == 4'hf)
                cid_match_pass = 1'b1;
            else if (cid < 4'd4)
                cid_match_pass = match_bits[cid[1:0]];
            else
                cid_match_pass = 1'b0;
        end
    endfunction

    function write_control_needs_cid;
        input [31:0] aux2;
        begin
            write_control_needs_cid =
                (aux2[30:29] != 2'b00) &&
                ((aux2[0] && (aux2[11:8] != 4'hf)) ||
                 (aux2[1] && (aux2[15:12] != 4'hf)) ||
                 (aux2[2] && (aux2[19:16] != 4'hf)) ||
                 (aux2[3] && (aux2[23:20] != 4'hf)));
        end
    endfunction

    function write_control_lane_allowed;
        input [31:0] aux2;
        input [255:0] smask_value;
        input [11:0] px;
        input [11:0] py;
        input [3:0] cid;
        reg secondary_mask_enabled;
        reg [2:0] secondary_in_enabled;
        reg [2:0] secondary_out_enabled;
        reg [2:0] secondary_mask_passes;
        reg primary_mask_pass;
        reg secondary_mask_pass;
        reg secondary_in_pass;
        reg secondary_out_pass;
        reg mask_inside;
        begin
            // SMASK0 is the window/GL bound. Positive secondary rectangles
            // form an inclusion union. Negative secondary rectangles cover
            // foreground windows, so drawing must remain outside EVERY one.
            // LIGHT figure 23 explicitly composes these groups: inside the
            // target inclusion AND outside each foreground exclusion. Keep
            // each mask's independent CID predicate in the same group.
            primary_mask_pass = !aux2[0];
            secondary_mask_enabled = |aux2[3:1];
            secondary_in_enabled = aux2[3:1] & aux2[7:5];
            secondary_out_enabled = aux2[3:1] & ~aux2[7:5];
            secondary_mask_passes = 3'b000;

            if (aux2[0]) begin
                mask_inside = screen_mask_hit(px, py,
                                              smask_word(smask_value, 3'd0),
                                              smask_word(smask_value, 3'd1));
                primary_mask_pass = (mask_inside == aux2[4]) &&
                                    cid_match_pass(cid, aux2[11:8]);
            end
            if (aux2[1]) begin
                mask_inside = screen_mask_hit(px, py,
                                              smask_word(smask_value, 3'd2),
                                              smask_word(smask_value, 3'd3));
                if ((mask_inside == aux2[5]) &&
                    cid_match_pass(cid, aux2[15:12]))
                    secondary_mask_passes[0] = 1'b1;
            end
            if (aux2[2]) begin
                mask_inside = screen_mask_hit(px, py,
                                              smask_word(smask_value, 3'd4),
                                              smask_word(smask_value, 3'd5));
                if ((mask_inside == aux2[6]) &&
                    cid_match_pass(cid, aux2[19:16]))
                    secondary_mask_passes[1] = 1'b1;
            end
            if (aux2[3]) begin
                mask_inside = screen_mask_hit(px, py,
                                              smask_word(smask_value, 3'd6),
                                              smask_word(smask_value, 3'd7));
                if ((mask_inside == aux2[7]) &&
                    cid_match_pass(cid, aux2[23:20]))
                    secondary_mask_passes[2] = 1'b1;
            end

            secondary_in_pass = !(|secondary_in_enabled) ||
                (|(secondary_mask_passes & secondary_in_enabled));
            secondary_out_pass = !(|secondary_out_enabled) ||
                ((secondary_mask_passes & secondary_out_enabled) ==
                 secondary_out_enabled);
            secondary_mask_pass = !secondary_mask_enabled ||
                (secondary_in_pass && secondary_out_pass);

            write_control_lane_allowed =
                (aux2[30:29] != 2'b00) && primary_mask_pass &&
                secondary_mask_pass;
        end
    endfunction

    function [3:0] cid_word_lane;
        input [31:0] data;
        input [1:0] lane;
        begin
            case (lane)
                2'd0: cid_word_lane = data[27:24];
                2'd1: cid_word_lane = data[19:16];
                2'd2: cid_word_lane = data[11:8];
                default: cid_word_lane = data[3:0];
            endcase
        end
    endfunction

    function [3:0] write_control_gated_wstrb;
        input [3:0] raw_wstrb;
        input [11:0] dst_x;
        input [11:0] dst_y;
        input [31:0] aux2;
        input [255:0] smask_value;
        input [31:0] cid_word;
        integer i;
        reg [11:0] lane_x_value;
        begin
            write_control_gated_wstrb = 4'h0;
            for (i = 0; i < 4; i = i + 1) begin
                lane_x_value = {dst_x[11:2], 2'b00} + i[11:0];
                if (raw_wstrb[i] &&
                    write_control_lane_allowed(
                        aux2, smask_value, lane_x_value, dst_y,
                        cid_word_lane(cid_word, i[1:0])))
                    write_control_gated_wstrb[i] = 1'b1;
            end
        end
    endfunction

    wire [3:0] pix_lane_wstrb = pix_mask;
    wire [31:0] pix_lane_wdata =
        amplify_doublebuffer_source_word(
            pix_data, pix_aux1, pix_aux2[30:29] == 2'b01, pix_rgbmode,
            pix_rwaux);
    wire [3:0] pix_wstrb_without_cid =
        write_control_gated_wstrb(pix_lane_wstrb, pix_x, pix_y, pix_aux2,
                                  pix_smask, 32'h0);
    wire pix_pixel_write_disabled =
        (pix_aux2[30:29] == 2'b01) && (pix_pixel_writemask == 8'h00);
    wire pix_write_needs_rmw =
        pix_colorcomp || (pix_logic_op != 4'h3) ||
        (pix_pixel_writemask != 8'hff);
    wire pending_write_needs_rmw =
        pending_colorcomp || (pending_logic_op != 4'h3) ||
        (pending_pixel_writemask != 8'hff);
    wire [3:0] pending_cid_wstrb =
        write_control_gated_wstrb(pending_wstrb, pending_x, pending_y,
                                  pending_aux2, pending_smask,
                                  mem_rsp_rdata);
    wire [31:0] copy_first_response_wdata = assemble_copy_source_word(
        mem_rsp_rdata, 32'h0, copy_src_addr0, copy_src_addr1,
        copy_lane_mask, copy_dst_x, copy_src_x, copy_src_y, copy_aux1,
        copy_aux2[30:29] == 2'b01);
    wire [31:0] copy_second_response_wdata = assemble_copy_source_word(
        copy_src_word0, mem_rsp_rdata, copy_src_addr0, copy_src_addr1,
        copy_lane_mask, copy_dst_x, copy_src_x, copy_src_y, copy_aux1,
        copy_aux2[30:29] == 2'b01);
    wire load_lane_in_span = !load_is_exhausted &&
        (!load_is_block ||
         (load_dir_down ? (load_cur_x >= load_end_x) :
                          (load_cur_x <= load_end_x)));
    wire load_lane_visible = load_lane_in_span &&
                             (load_cur_x < 12'd1024) &&
                             (load_cur_y < 12'd1024);

    // A grouped copy resolves one destination word only after every source
    // word has been sampled.  Keeping the destination launch in this task
    // gives aligned copies their original response-to-request latency while
    // the cross-word path pays only for its necessary second source read.
    task start_copy_destination;
        input [31:0] assembled_wdata;
        reg [3:0] gated_wstrb;
        reg       write_needs_rmw;
        begin
            gated_wstrb = write_control_gated_wstrb(
                copy_lane_mask, copy_dst_x, copy_dst_y, copy_aux2,
                copy_smask, 32'h0);
            write_needs_rmw = copy_colorcomp ||
                              (copy_logic_op != 4'h3) ||
                              (copy_pixel_writemask != 8'hff);

            pending_x <= copy_dst_x;
            pending_y <= copy_dst_y;
            pending_plane <= copy_plane;
            pending_wdata <= assembled_wdata;
            pending_wstrb <= copy_lane_mask;
            pending_aux2 <= copy_aux2;
            pending_smask <= copy_smask;
            pending_logic_op <= copy_logic_op;
            pending_pixel_writemask <= copy_pixel_writemask;
            pending_colorcomp <= copy_colorcomp;
            pending_aux1 <= copy_aux1;
            pending_last <= copy_last;
            pending_retire_x <= copy_retire_x;
            pending_retire_y <= copy_retire_y;
            pending_retire_done <= copy_retire_done;

            if (write_control_needs_cid(copy_aux2)) begin
                mem_req_valid <= 1'b1;
                mem_req_write <= 1'b0;
                mem_req_plane <= 2'h3;
                mem_req_addr <= copy_dst_addr;
                mem_req_wdata <= 32'h0;
                mem_req_wstrb <= 4'h0;
                state <= S_CID_WAIT;
            end else if (gated_wstrb != 4'h0) begin
                pending_wstrb <= gated_wstrb;
                mem_req_valid <= 1'b1;
                mem_req_write <= !write_needs_rmw;
                mem_req_plane <= copy_plane;
                mem_req_addr <= copy_dst_addr;
                mem_req_wdata <= assembled_wdata;
                mem_req_wstrb <= write_needs_rmw ? 4'h0 : gated_wstrb;
                state <= write_needs_rmw ? S_RMW_WAIT : S_WRITE;
            end else begin
                state <= S_IDLE;
                if (copy_last) begin
                    retire_valid <= 1'b1;
                    retire_x <= copy_retire_x;
                    retire_y <= copy_retire_y;
                    retire_done <= copy_retire_done;
                end
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            mem_req_valid <= 1'b0;
            mem_req_write <= 1'b0;
            mem_req_plane <= 2'h0;
            mem_req_addr <= 18'h0;
            mem_req_wdata <= 32'h0;
            mem_req_wstrb <= 4'h0;
            copy_dst_addr <= 18'h0;
            copy_src_addr0 <= 18'h0;
            copy_src_addr1 <= 18'h0;
            copy_src_word0 <= 32'h0;
            copy_second_needed <= 1'b0;
            copy_plane <= 2'h0;
            copy_lane_mask <= 4'h0;
            copy_dst_x <= 12'h0;
            copy_dst_y <= 12'h0;
            copy_src_x <= 12'h0;
            copy_src_y <= 12'h0;
            copy_aux2 <= 32'h0;
            copy_smask <= 256'h0;
            copy_logic_op <= 4'h3;
            copy_pixel_writemask <= 8'hff;
            copy_colorcomp <= 1'b0;
            copy_aux1 <= 10'h0;
            copy_last <= 1'b0;
            copy_retire_x <= 12'h0;
            copy_retire_y <= 12'h0;
            copy_retire_done <= 1'b0;
            pending_x <= 12'h0;
            pending_y <= 12'h0;
            pending_plane <= 2'h0;
            pending_wdata <= 32'h0;
            pending_wstrb <= 4'h0;
            pending_aux2 <= 32'h0;
            pending_smask <= 256'h0;
            pending_logic_op <= 4'h3;
            pending_pixel_writemask <= 8'hff;
            pending_colorcomp <= 1'b0;
            pending_aux1 <= 10'h0;
            pending_last <= 1'b0;
            pending_retire_x <= 12'h0;
            pending_retire_y <= 12'h0;
            pending_retire_done <= 1'b0;
            load_cur_x <= 12'h0;
            load_cur_y <= 12'h0;
            load_end_x <= 12'h0;
            load_dir_down <= 1'b0;
            load_is_block <= 1'b0;
            load_is_quad <= 1'b0;
            load_is_exhausted <= 1'b0;
            load_selected_plane <= 2'h0;
            load_selected_aux1 <= 10'h0;
            load_selected_is_pixel <= 1'b0;
            load_index <= 2'h0;
            load_assembled <= 32'h0;
            load_rsp_data <= 32'h0;
            retire_valid <= 1'b0;
            retire_x <= 12'h0;
            retire_y <= 12'h0;
            retire_done <= 1'b0;
        end else begin
            retire_valid <= 1'b0;
            if (mem_req_valid && mem_req_ready)
                mem_req_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (pix_valid && pix_ready) begin
                        if (pix_mask == 4'h0) begin
                            if (pix_last) begin
                                retire_valid <= 1'b1;
                                retire_x <= pix_retire_x;
                                retire_y <= pix_retire_y;
                                retire_done <= pix_retire_done;
                            end
                        end else if (pix_pixel_write_disabled) begin
                            // DOUBLEBUF with neither DBLDST selected maps the
                            // logical planemask to zero. Retire without a
                            // useless read/write of an unchanged pixel.
                            if (pix_last) begin
                                retire_valid <= 1'b1;
                                retire_x <= pix_retire_x;
                                retire_y <= pix_retire_y;
                                retire_done <= pix_retire_done;
                            end
                        end else if (pix_copy) begin
                            mem_req_valid <= 1'b1;
                            mem_req_write <= 1'b0;
                            mem_req_plane <= pix_plane;
                            mem_req_addr <= pixel_mem_addr(pix_src_x,
                                                           pix_src_y);
                            mem_req_wdata <= 32'h0;
                            mem_req_wstrb <= 4'h0;
                            copy_dst_addr <= pixel_mem_addr(pix_x, pix_y);
                            copy_src_addr0 <= pixel_mem_addr(pix_src_x,
                                                            pix_src_y);
                            copy_src_addr1 <= copy_second_source_addr(
                                pix_mask, pix_x, pix_src_x, pix_src_y);
                            copy_second_needed <= copy_needs_second_source(
                                pix_mask, pix_x, pix_src_x, pix_src_y);
                            copy_plane <= pix_plane;
                            copy_lane_mask <= pix_mask;
                            copy_dst_x <= pix_x;
                            copy_dst_y <= pix_y;
                            copy_src_x <= pix_src_x;
                            copy_src_y <= pix_src_y;
                            copy_aux2 <= pix_aux2;
                            copy_smask <= pix_smask;
                            copy_logic_op <= pix_logic_op;
                            copy_pixel_writemask <= pix_pixel_writemask;
                            copy_colorcomp <= pix_colorcomp;
                            copy_aux1 <= pix_aux1;
                            copy_last <= pix_last;
                            copy_retire_x <= pix_retire_x;
                            copy_retire_y <= pix_retire_y;
                            copy_retire_done <= pix_retire_done;
                            state <= S_COPY_WAIT;
                        end else begin
                            pending_x <= pix_x;
                            pending_y <= pix_y;
                            pending_plane <= pix_plane;
                            pending_wdata <= pix_lane_wdata;
                            pending_aux2 <= pix_aux2;
                            pending_smask <= pix_smask;
                            pending_logic_op <= pix_logic_op;
                            pending_pixel_writemask <= pix_pixel_writemask;
                            pending_colorcomp <= pix_colorcomp;
                            pending_aux1 <= pix_aux1;
                            pending_last <= pix_last;
                            pending_retire_x <= pix_retire_x;
                            pending_retire_y <= pix_retire_y;
                            pending_retire_done <= pix_retire_done;

                            if (write_control_needs_cid(pix_aux2)) begin
                                pending_wstrb <= pix_lane_wstrb;
                                mem_req_valid <= 1'b1;
                                mem_req_write <= 1'b0;
                                mem_req_plane <= 2'h3;
                                mem_req_addr <= pixel_mem_addr(pix_x, pix_y);
                                mem_req_wdata <= 32'h0;
                                mem_req_wstrb <= 4'h0;
                                state <= S_CID_WAIT;
                            end else if (pix_wstrb_without_cid != 4'h0) begin
                                pending_wstrb <= pix_wstrb_without_cid;
                                mem_req_valid <= 1'b1;
                                mem_req_write <= !pix_write_needs_rmw;
                                mem_req_plane <= pix_plane;
                                mem_req_addr <= pixel_mem_addr(pix_x, pix_y);
                                mem_req_wdata <= pix_write_needs_rmw ?
                                                 32'h0 : pix_lane_wdata;
                                mem_req_wstrb <= pix_write_needs_rmw ?
                                                 4'h0 : pix_wstrb_without_cid;
                                state <= pix_write_needs_rmw ?
                                         S_RMW_WAIT : S_WRITE;
                            end else if (pix_last) begin
                                retire_valid <= 1'b1;
                                retire_x <= pix_retire_x;
                                retire_y <= pix_retire_y;
                                retire_done <= pix_retire_done;
                            end
                        end
                    end else if (load_valid && load_ready) begin
                        load_cur_x <= load_x;
                        load_cur_y <= load_y;
                        load_end_x <= load_xend;
                        load_dir_down <= load_x_down;
                        load_is_block <= load_block;
                        load_is_quad <= load_quadmode;
                        load_is_exhausted <= load_exhausted;
                        load_selected_plane <= load_plane;
                        load_selected_aux1 <= load_aux1;
                        load_selected_is_pixel <= load_is_pixel;
                        load_index <= 2'h0;
                        load_assembled <= 32'h0;
                        state <= S_LOAD_REQ;
                    end
                end

                S_LOAD_REQ: begin
                    if (!load_lane_visible) begin
                        if (!load_is_quad || (load_index == 2'd3)) begin
                            load_rsp_data <=
                                load_byte_insert(load_assembled, load_index,
                                                 8'h0);
                            state <= S_LOAD_RSP;
                        end else begin
                            load_assembled <=
                                load_byte_insert(load_assembled, load_index,
                                                 8'h0);
                            load_index <= load_index + 2'd1;
                            load_cur_x <= load_dir_down ?
                                          load_cur_x - 12'd1 :
                                          load_cur_x + 12'd1;
                        end
                    end else if (!mem_req_valid) begin
                        mem_req_valid <= 1'b1;
                        mem_req_write <= 1'b0;
                        mem_req_plane <= load_selected_plane;
                        mem_req_addr <= pixel_mem_addr(load_cur_x, load_cur_y);
                        mem_req_wdata <= 32'h0;
                        mem_req_wstrb <= 4'h0;
                        state <= S_LOAD_WAIT;
                    end
                end

                S_LOAD_WAIT: begin
                    if (mem_rsp_valid) begin
                        if (!load_is_quad || (load_index == 2'd3)) begin
                            load_rsp_data <= load_byte_insert(
                                load_assembled, load_index,
                                selected_doublebuffer_load_pixel(
                                    word_lane_data(mem_rsp_rdata,
                                                   load_cur_x[1:0]),
                                    load_selected_aux1,
                                    load_selected_is_pixel));
                            state <= S_LOAD_RSP;
                        end else begin
                            load_assembled <= load_byte_insert(
                                load_assembled, load_index,
                                selected_doublebuffer_load_pixel(
                                    word_lane_data(mem_rsp_rdata,
                                                   load_cur_x[1:0]),
                                    load_selected_aux1,
                                    load_selected_is_pixel));
                            load_index <= load_index + 2'd1;
                            load_cur_x <= load_dir_down ?
                                          load_cur_x - 12'd1 :
                                          load_cur_x + 12'd1;
                            state <= S_LOAD_REQ;
                        end
                    end
                end

                S_LOAD_RSP: begin
                    if (load_rsp_ready)
                        state <= S_IDLE;
                end

                S_COPY_WAIT: begin
                    if (mem_rsp_valid) begin
                        copy_src_word0 <= mem_rsp_rdata;
                        if (copy_second_needed) begin
                            mem_req_valid <= 1'b1;
                            mem_req_write <= 1'b0;
                            mem_req_plane <= copy_plane;
                            mem_req_addr <= copy_src_addr1;
                            mem_req_wdata <= 32'h0;
                            mem_req_wstrb <= 4'h0;
                            state <= S_COPY_SECOND_WAIT;
                        end else begin
                            start_copy_destination(
                                copy_first_response_wdata);
                        end
                    end
                end

                S_COPY_SECOND_WAIT: begin
                    if (mem_rsp_valid) begin
                        start_copy_destination(
                            copy_second_response_wdata);
                    end
                end

                S_CID_WAIT: begin
                    if (mem_rsp_valid) begin
                        pending_wstrb <= pending_cid_wstrb;
                        if (pending_cid_wstrb == 4'h0) begin
                            state <= S_IDLE;
                            if (pending_last) begin
                                retire_valid <= 1'b1;
                                retire_x <= pending_retire_x;
                                retire_y <= pending_retire_y;
                                retire_done <= pending_retire_done;
                            end
                        end else begin
                            mem_req_valid <= 1'b1;
                            mem_req_write <= !pending_write_needs_rmw;
                            mem_req_plane <= pending_plane;
                            mem_req_addr <= pixel_mem_addr(pending_x,
                                                           pending_y);
                            mem_req_wdata <= pending_write_needs_rmw ?
                                             32'h0 : pending_wdata;
                            mem_req_wstrb <= pending_write_needs_rmw ?
                                4'h0 : pending_cid_wstrb;
                            state <= pending_write_needs_rmw ?
                                     S_RMW_WAIT : S_WRITE;
                        end
                    end
                end

                S_RMW_WAIT: begin
                    if (mem_rsp_valid) begin
                        mem_req_valid <= 1'b1;
                        mem_req_write <= 1'b1;
                        mem_req_plane <= pending_plane;
                        mem_req_addr <= pixel_mem_addr(pending_x, pending_y);
                        mem_req_wdata <= apply_logic_word(
                            mem_rsp_rdata, pending_wdata, pending_wstrb,
                            pending_logic_op, pending_pixel_writemask,
                            pending_colorcomp, pending_aux1,
                            pending_aux2[30:29] == 2'b01);
                        mem_req_wstrb <= pending_wstrb;
                        state <= S_WRITE;
                    end
                end

                S_WRITE: begin
                    if (mem_req_valid && mem_req_ready) begin
                        state <= S_IDLE;
                        if (pending_last) begin
                            retire_valid <= 1'b1;
                            retire_x <= pending_retire_x;
                            retire_y <= pending_retire_y;
                            retire_done <= pending_retire_done;
                        end
                    end else if (!mem_req_valid) begin
                        state <= S_IDLE;
                        if (pending_last) begin
                            retire_valid <= 1'b1;
                            retire_x <= pending_retire_x;
                            retire_y <= pending_retire_y;
                            retire_done <= pending_retire_done;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
