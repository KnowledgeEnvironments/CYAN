// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - LG1 clean command context queue skeleton

`default_nettype none

module lg1_rex_context_queue (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        in_valid,
    output wire        in_ready,
    input  wire        in_rwaux_write,
    input  wire        in_solid_fill,
    input  wire        in_pattern_fill,
    input  wire        in_pattern_tail,
    input  wire        in_copy_fill,
    input  wire        in_line_draw,
    input  wire        in_shade,
    input  wire [31:0] in_command,
    input  wire [31:0] in_data,
    input  wire [19:0] in_lsmode,
    input  wire        in_ls_reload,
    input  wire [ 3:0] in_lane_mask,
    input  wire [11:0] in_x,
    input  wire [11:0] in_y,
    input  wire [11:0] in_xstart,
    input  wire [11:0] in_xend,
    input  wire [11:0] in_yend,
    input  wire        in_x_down,
    input  wire        in_y_down,
    input  wire        in_block,
    input  wire        in_clip_xend,
    input  wire        in_quadmode,
    input  wire [ 1:0] in_plane,
    input  wire [ 7:0] in_color,
    input  wire [19:0] in_color_fixed,
    input  wire [19:0] in_color1_fixed,
    input  wire [19:0] in_color2_fixed,
    input  wire [20:0] in_slope,
    input  wire [20:0] in_slope1,
    input  wire [20:0] in_slope2,
    input  wire [16:0] in_minor_slope,
    input  wire        in_minor_negative,
    input  wire [14:0] in_line_frac,
    input  wire        in_pattern_opaque,
    input  wire [ 7:0] in_bg_color,
    input  wire [11:0] in_copy_dx,
    input  wire [11:0] in_copy_dy,
    input  wire [31:0] in_aux2,
    input  wire [255:0] in_smask,
    input  wire [ 3:0] in_logic_op,
    input  wire [ 7:0] in_pixel_writemask,
    input  wire        in_colorcomp,
    input  wire [ 9:0] in_aux1,
    input  wire [11:0] in_retire_x,
    input  wire [11:0] in_retire_y,
    input  wire        in_retire_done,

    output wire        out_valid,
    input  wire        out_ready,
    output wire        out_rwaux_write,
    output wire        out_solid_fill,
    output wire        out_pattern_fill,
    output wire        out_pattern_tail,
    output wire        out_copy_fill,
    output wire        out_line_draw,
    output wire        out_shade,
    output wire [31:0] out_command,
    output wire [31:0] out_data,
    output wire [19:0] out_lsmode,
    output wire        out_ls_reload,
    output wire [ 3:0] out_lane_mask,
    output wire [11:0] out_x,
    output wire [11:0] out_y,
    output wire [11:0] out_xstart,
    output wire [11:0] out_xend,
    output wire [11:0] out_yend,
    output wire        out_x_down,
    output wire        out_y_down,
    output wire        out_block,
    output wire        out_clip_xend,
    output wire        out_quadmode,
    output wire [ 1:0] out_plane,
    output wire [ 7:0] out_color,
    output wire [19:0] out_color_fixed,
    output wire [19:0] out_color1_fixed,
    output wire [19:0] out_color2_fixed,
    output wire [20:0] out_slope,
    output wire [20:0] out_slope1,
    output wire [20:0] out_slope2,
    output wire [16:0] out_minor_slope,
    output wire        out_minor_negative,
    output wire [14:0] out_line_frac,
    output wire        out_pattern_opaque,
    output wire [ 7:0] out_bg_color,
    output wire [11:0] out_copy_dx,
    output wire [11:0] out_copy_dy,
    output wire [31:0] out_aux2,
    output wire [255:0] out_smask,
    output wire [ 3:0] out_logic_op,
    output wire [ 7:0] out_pixel_writemask,
    output wire        out_colorcomp,
    output wire [ 9:0] out_aux1,
    output wire [11:0] out_retire_x,
    output wire [11:0] out_retire_y,
    output wire        out_retire_done
);
    localparam FIFO_BITS = 6;
    localparam FIFO_DEPTH = 1 << FIFO_BITS;
    localparam CONTEXT_WIDTH = 696;

    reg [FIFO_BITS-1:0] rd_ptr;
    reg [FIFO_BITS-1:0] wr_ptr;
    reg [FIFO_BITS:0]   count;

    // One packed record gives synthesis one 64x695 memory instead of 45
    // independently decoded arrays. Payload bits are never reset; count and
    // head_valid make stale storage architecturally invisible.
    // At only 64 words, the packed record is intentionally distributed RAM:
    // Watson V235 showed that this costs about 800 LUTs and avoids wasting a
    // bank of very wide, shallow block RAMs. The architectural requirement is
    // one packed memory rather than 45 independently decoded register arrays.
    (* ram_style = "distributed" *)
    reg [CONTEXT_WIDTH-1:0] context_mem [0:FIFO_DEPTH-1];
    reg [CONTEXT_WIDTH-1:0] head_context;
    reg                     head_valid;

    wire [CONTEXT_WIDTH-1:0] in_context = {
        in_rwaux_write, in_solid_fill, in_pattern_fill, in_pattern_tail,
        in_copy_fill, in_line_draw, in_shade, in_command, in_data,
        in_lsmode, in_ls_reload, in_lane_mask, in_x, in_y, in_xstart, in_xend, in_yend,
        in_x_down, in_y_down, in_block, in_clip_xend, in_quadmode, in_plane,
        in_color, in_color_fixed, in_color1_fixed, in_color2_fixed,
        in_slope, in_slope1, in_slope2, in_minor_slope, in_minor_negative,
        in_line_frac, in_pattern_opaque, in_bg_color, in_copy_dx, in_copy_dy,
        in_aux2, in_smask, in_logic_op, in_pixel_writemask, in_colorcomp,
        in_aux1, in_retire_x, in_retire_y, in_retire_done
    };

    wire head_rwaux_write;
    wire head_solid_fill;
    wire head_pattern_fill;
    wire head_pattern_tail;
    wire head_copy_fill;
    wire head_line_draw;
    wire head_shade;
    wire [31:0] head_command;
    wire [31:0] head_data;
    wire [19:0] head_lsmode;
    wire head_ls_reload;
    wire [3:0] head_lane_mask;
    wire [11:0] head_x;
    wire [11:0] head_y;
    wire [11:0] head_xstart;
    wire [11:0] head_xend;
    wire [11:0] head_yend;
    wire head_x_down;
    wire head_y_down;
    wire head_block;
    wire head_clip_xend;
    wire head_quadmode;
    wire [1:0] head_plane;
    wire [7:0] head_color;
    wire [19:0] head_color_fixed;
    wire [19:0] head_color1_fixed;
    wire [19:0] head_color2_fixed;
    wire [20:0] head_slope;
    wire [20:0] head_slope1;
    wire [20:0] head_slope2;
    wire [16:0] head_minor_slope;
    wire head_minor_negative;
    wire [14:0] head_line_frac;
    wire head_pattern_opaque;
    wire [7:0] head_bg_color;
    wire [11:0] head_copy_dx;
    wire [11:0] head_copy_dy;
    wire [31:0] head_aux2;
    wire [255:0] head_smask;
    wire [3:0] head_logic_op;
    wire [7:0] head_pixel_writemask;
    wire head_colorcomp;
    wire [9:0] head_aux1;
    wire [11:0] head_retire_x;
    wire [11:0] head_retire_y;
    wire head_retire_done;

    assign {
        head_rwaux_write, head_solid_fill, head_pattern_fill,
        head_pattern_tail, head_copy_fill, head_line_draw, head_shade,
        head_command, head_data, head_lsmode, head_ls_reload, head_lane_mask, head_x, head_y,
        head_xstart, head_xend, head_yend, head_x_down, head_y_down,
        head_block, head_clip_xend, head_quadmode, head_plane, head_color,
        head_color_fixed, head_color1_fixed, head_color2_fixed, head_slope,
        head_slope1, head_slope2, head_minor_slope, head_minor_negative,
        head_line_frac, head_pattern_opaque, head_bg_color, head_copy_dx,
        head_copy_dy, head_aux2, head_smask, head_logic_op,
        head_pixel_writemask, head_colorcomp, head_aux1, head_retire_x,
        head_retire_y, head_retire_done
    } = head_context;

    wire full = count == {1'b1, {FIFO_BITS{1'b0}}};
    wire push = in_valid && in_ready;
    wire pop = out_valid && out_ready;

    assign in_ready = !full;
    assign out_valid = head_valid;

    assign out_rwaux_write = out_valid ? head_rwaux_write : 1'b0;
    assign out_solid_fill = out_valid ? head_solid_fill : 1'b0;
    assign out_pattern_fill = out_valid ? head_pattern_fill : 1'b0;
    assign out_pattern_tail = out_valid ? head_pattern_tail : 1'b0;
    assign out_copy_fill = out_valid ? head_copy_fill : 1'b0;
    assign out_line_draw = out_valid ? head_line_draw : 1'b0;
    assign out_shade = out_valid ? head_shade : 1'b0;
    assign out_command = out_valid ? head_command : 32'h0;
    assign out_data = out_valid ? head_data : 32'h0;
    assign out_lsmode = out_valid ? head_lsmode : 20'h0;
    assign out_ls_reload = out_valid ? head_ls_reload : 1'b0;
    assign out_lane_mask = out_valid ? head_lane_mask : 4'h0;
    assign out_x = out_valid ? head_x : 12'h0;
    assign out_y = out_valid ? head_y : 12'h0;
    assign out_xstart = out_valid ? head_xstart : 12'h0;
    assign out_xend = out_valid ? head_xend : 12'h0;
    assign out_yend = out_valid ? head_yend : 12'h0;
    assign out_x_down = out_valid ? head_x_down : 1'b0;
    assign out_y_down = out_valid ? head_y_down : 1'b0;
    assign out_block = out_valid ? head_block : 1'b0;
    assign out_clip_xend = out_valid ? head_clip_xend : 1'b0;
    assign out_quadmode = out_valid ? head_quadmode : 1'b0;
    assign out_plane = out_valid ? head_plane : 2'h0;
    assign out_color = out_valid ? head_color : 8'h0;
    assign out_color_fixed = out_valid ? head_color_fixed : 20'h0;
    assign out_color1_fixed = out_valid ? head_color1_fixed : 20'h0;
    assign out_color2_fixed = out_valid ? head_color2_fixed : 20'h0;
    assign out_slope = out_valid ? head_slope : 21'h0;
    assign out_slope1 = out_valid ? head_slope1 : 21'h0;
    assign out_slope2 = out_valid ? head_slope2 : 21'h0;
    assign out_minor_slope = out_valid ? head_minor_slope : 17'h0;
    assign out_minor_negative = out_valid ? head_minor_negative : 1'b0;
    assign out_line_frac = out_valid ? head_line_frac : 15'h0;
    assign out_pattern_opaque = out_valid ? head_pattern_opaque : 1'b0;
    assign out_bg_color = out_valid ? head_bg_color : 8'h0;
    assign out_copy_dx = out_valid ? head_copy_dx : 12'h0;
    assign out_copy_dy = out_valid ? head_copy_dy : 12'h0;
    assign out_aux2 = out_valid ? head_aux2 : 32'h0;
    assign out_smask = out_valid ? head_smask : 256'h0;
    assign out_logic_op = out_valid ? head_logic_op : 4'h3;
    assign out_pixel_writemask = out_valid ? head_pixel_writemask : 8'hff;
    assign out_colorcomp = out_valid ? head_colorcomp : 1'b0;
    assign out_aux1 = out_valid ? head_aux1 : 10'h0;
    assign out_retire_x = out_valid ? head_retire_x : 12'h0;
    assign out_retire_y = out_valid ? head_retire_y : 12'h0;
    assign out_retire_done = out_valid ? head_retire_done : 1'b0;

    // Synchronous packed-memory write/read. The bypass cases make the FIFO
    // first-word fall-through without requiring an asynchronous RAM read.
    always @(posedge clk) begin
        if (push)
            context_mem[wr_ptr] <= in_context;

        if (!head_valid && push)
            head_context <= in_context;
        else if (pop) begin
            if (count > {{FIFO_BITS{1'b0}}, 1'b1})
                head_context <= context_mem[rd_ptr +
                    {{(FIFO_BITS - 1){1'b0}}, 1'b1}];
            else if (push)
                head_context <= in_context;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr <= {FIFO_BITS{1'b0}};
            wr_ptr <= {FIFO_BITS{1'b0}};
            count <= {(FIFO_BITS + 1){1'b0}};
            head_valid <= 1'b0;
        end else begin
            if (push)
                wr_ptr <= wr_ptr + {{(FIFO_BITS - 1){1'b0}}, 1'b1};

            if (pop)
                rd_ptr <= rd_ptr + {{(FIFO_BITS - 1){1'b0}}, 1'b1};

            if (!head_valid) begin
                if (push)
                    head_valid <= 1'b1;
            end else if (pop) begin
                if ((count > {{FIFO_BITS{1'b0}}, 1'b1}) || push)
                    head_valid <= 1'b1;
                else
                    head_valid <= 1'b0;
            end

            case ({push, pop})
                2'b10: count <= count + {{FIFO_BITS{1'b0}}, 1'b1};
                2'b01: count <= count - {{FIFO_BITS{1'b0}}, 1'b1};
                default: ;
            endcase
        end
    end
endmodule

`default_nettype wire
