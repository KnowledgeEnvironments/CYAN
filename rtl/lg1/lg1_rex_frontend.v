// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - LG1 clean REX-like frontend

`default_nettype none

module lg1_rex_frontend (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [14:0] addr,
    input  wire [63:0] wdata,
    output reg  [63:0] rdata,
    input  wire [ 7:0] wmask,
    input  wire        valid,
    input  wire        write,
    output wire        ready,
    input  wire        chip_busy,

    output wire        load_valid,
    input  wire        load_ready,
    output wire [11:0] load_x,
    output wire [11:0] load_y,
    output wire [11:0] load_xend,
    output wire        load_x_down,
    output wire        load_block,
    output wire        load_quadmode,
    output wire        load_exhausted,
    output wire [ 1:0] load_plane,
    output wire [ 9:0] load_aux1,
    output wire        load_is_pixel,
    input  wire        load_rsp_valid,
    input  wire [31:0] load_rsp_data,
    output wire        load_rsp_ready,

    output reg         ctx_valid,
    output reg         ctx_rwaux_write,
    output reg         ctx_solid_fill,
    output reg         ctx_pattern_fill,
    output reg         ctx_pattern_tail,
    output reg         ctx_copy_fill,
    output reg         ctx_line_draw,
    output reg         ctx_shade,
    output reg  [31:0] ctx_command,
    output reg  [31:0] ctx_data,
    output reg  [19:0] ctx_lsmode,
    output reg         ctx_ls_reload,
    output reg  [ 3:0] ctx_lane_mask,
    output reg  [11:0] ctx_x,
    output reg  [11:0] ctx_y,
    output reg  [11:0] ctx_xstart,
    output reg  [11:0] ctx_xend,
    output reg  [11:0] ctx_yend,
    output reg         ctx_x_down,
    output reg         ctx_y_down,
    output reg         ctx_block,
    output reg         ctx_clip_xend,
    output reg         ctx_quadmode,
    output reg  [ 1:0] ctx_plane,
    output reg  [ 7:0] ctx_color,
    output reg  [19:0] ctx_color_fixed,
    output reg  [19:0] ctx_color1_fixed,
    output reg  [19:0] ctx_color2_fixed,
    output reg  [20:0] ctx_slope,
    output reg  [20:0] ctx_slope1,
    output reg  [20:0] ctx_slope2,
    output reg  [16:0] ctx_minor_slope,
    output reg         ctx_minor_negative,
    output reg  [14:0] ctx_line_frac,
    output reg         ctx_pattern_opaque,
    output reg  [ 7:0] ctx_bg_color,
    output reg  [11:0] ctx_copy_dx,
    output reg  [11:0] ctx_copy_dy,
    output reg  [31:0] ctx_aux2,
    output reg  [255:0] ctx_smask,
    output reg  [ 3:0] ctx_logic_op,
    output reg  [ 7:0] ctx_pixel_writemask,
    output reg         ctx_colorcomp,
    output reg  [ 9:0] ctx_aux1,
    output reg  [11:0] ctx_retire_x,
    output reg  [11:0] ctx_retire_y,
    output reg         ctx_retire_done,
    input  wire        ctx_ready,

    input  wire        retire_valid,
    input  wire [11:0] retire_x,
    input  wire [11:0] retire_y,
    input  wire        retire_done,

    input  wire        ls_work_valid,
    input  wire [31:0] ls_work_pattern,
    input  wire [19:0] ls_work_mode
);
    localparam [31:0] CMD_DRAW       = 32'h00000001;
    localparam [31:0] CMD_BLOCK      = 32'h00000008;
    localparam [31:0] CMD_LENGTH32   = 32'h00000010;
    localparam [31:0] CMD_QUADMODE   = 32'h00000020;
    localparam [31:0] CMD_XMAJOR     = 32'h00000040;
    localparam [31:0] CMD_XYCONTINUE = 32'h00000080;
    localparam [31:0] CMD_STOPONX    = 32'h00000100;
    localparam [31:0] CMD_STOPONY    = 32'h00000200;
    localparam [31:0] CMD_ENZPAT     = 32'h00000400;
    localparam [31:0] CMD_ENLSPAT    = 32'h00000800;
    localparam [31:0] CMD_RGBMODE    = 32'h00004000;
    localparam [31:0] CMD_COLORCOMP  = 32'h00010000;
    localparam [31:0] CMD_SHADE      = 32'h00020000;
    localparam [31:0] CMD_INITFRAC   = 32'h00040000;
    localparam [31:0] CMD_LOGICSRC   = 32'h00080000;
    localparam [31:0] CMD_LRQPOLY    = 32'h02000000;
    localparam [31:0] CMD_COLORAUX   = 32'h00200000;
    localparam [31:0] CMD_LSOPAQUE   = 32'h00400000;
    localparam [31:0] CMD_ZOPAQUE    = 32'h00800000;
    localparam [31:0] XSTATE_COLORAUX = 32'h10000000;
    localparam [31:0] XSTATE_LSOPAQUE = 32'h40000000;
    localparam [31:0] XSTATE_ZOPAQUE = 32'h80000000;
    localparam        ENABLE_COPY = 1'b1;

    localparam [1:0] AUX2_PIXELPLANES   = 2'b01;
    localparam [1:0] AUX2_OVERLAYPLANES = 2'b10;
    localparam [1:0] AUX2_CIDPLANES     = 2'b11;

    reg [31:0] command;
    reg [31:0] aux1;
    reg [31:0] xstate;
    reg        xstate_written;
    reg        xstate_mask_active;
    reg [15:0] rwmask;
    reg [31:0] aux2;
    reg [31:0] smask [0:7];
    reg [31:0] xywin;
    reg [31:0] configmode;
    reg [31:0] togglectxt;
    reg [ 2:0] configsel;
    reg [31:0] rwaux1;
    reg [31:0] rwaux2;
    reg [19:0] color0;
    reg        color0_after_xstate;
    reg        xstate_after_command;
    reg        zero_go_5b9_xstate_source;
    reg [19:0] color1;
    reg [19:0] color2;
    reg signed [20:0] slope0;
    reg signed [20:0] slope1;
    reg signed [20:0] slope2;
    reg [16:0] minorslope;
    reg        minorslope_negative;
    reg [ 7:0] colorback;
    reg [31:0] zpattern;
    reg [31:0] lspattern;
    reg [19:0] lsmode;
    reg        ls_programmed_since_launch;
    reg [31:0] aweight;
    reg [11:0] xsave;
    reg [31:0] xymove;
    reg [26:0] xstart_fixed;
    reg [26:0] ystart_fixed;
    reg [26:0] xend_fixed;
    reg [26:0] yend_fixed;
    reg [11:0] xstarti;
    reg [11:0] ystarti;
    reg [11:0] xendi;
    reg [11:0] yendi;
    reg [11:0] xcurri;
    reg [11:0] ycurri;
    reg        rwaux_stream_done;
    // True after software explicitly reloads XSTART and until another
    // primitive consumes that coordinate setup. The draw-image path uses
    // this provenance to give its non-BLOCK 0x...a1
    // RWAUX stream a real scanline endpoint without changing ordinary
    // XYCONTINUE cursor semantics.
    reg        xstart_explicit_since_primitive;
    reg        load_pending;
    reg        load_command_committed;
    reg [24:0] load_next_latched;
    reg        load_cursor_superseded;

    // SET writes assemble programmable state. A GO access snapshots it for
    // coherent execution.  Only the ten fields read on both sides of
    // TOGGLECTXT are truly banked; the remaining
    // current_* values below are launch snapshots, not a second programmable
    // register bank.
    reg [31:0] current_command;
    reg [31:0] current_aux1;
    reg [31:0] current_xstate;
    reg        current_xstate_written;
    reg [15:0] current_rwmask;
    reg [31:0] current_aux2;
    reg [31:0] current_smask [0:7];
    reg [31:0] current_rwaux1;
    reg [31:0] current_rwaux2;
    reg [19:0] current_color0;
    reg        current_color0_after_xstate;
    reg        current_xstate_after_command;
    reg        current_zero_go_5b9_xstate_source;
    reg [19:0] current_color1;
    reg [19:0] current_color2;
    reg signed [20:0] current_slope0;
    reg signed [20:0] current_slope1;
    reg signed [20:0] current_slope2;
    reg [16:0] current_minorslope;
    reg        current_minorslope_negative;
    reg [ 7:0] current_colorback;
    reg [31:0] current_zpattern;
    reg [31:0] current_lspattern;
    reg [19:0] current_lsmode;
    reg [31:0] current_aweight;
    reg [11:0] current_xsave;
    reg [31:0] current_xymove;
    reg [26:0] current_xstart_fixed;
    reg [26:0] current_ystart_fixed;
    reg [26:0] current_xend_fixed;
    reg [26:0] current_yend_fixed;
    reg [11:0] current_xstarti;
    reg [11:0] current_ystarti;
    reg [11:0] current_xendi;
    reg [11:0] current_yendi;
    reg [11:0] current_xcurri;
    reg [11:0] current_ycurri;
    reg        current_rwaux_stream_done;
    reg        current_xstart_explicit_since_primitive;

    wire go = addr[11];
    wire config_region = (addr[14:12] == 3'b100);
    wire [7:0] dreg_offset = {addr[7:2], 2'b00};
    wire [10:0] cfg_offset = {addr[10:2], 2'b00};
    wire [31:0] wd = bus_wd(wdata, wmask);
    wire [3:0] wd_lane_mask = bus_wd_lane_mask(wmask);
    wire rwaux_go_write =
        valid && write && go && !config_region && (dreg_offset == 8'h70) &&
        command_draw(command) &&
        command_effective_coloraux(command, xstate, 1'b1) &&
        !rwaux_stream_done && (wd_lane_mask != 4'h0);
    wire solid_trigger_write =
        valid && write && go && !config_region &&
        ((dreg_offset == 8'h00) || (dreg_offset == 8'h0c) ||
         (dreg_offset == 8'h10) || (dreg_offset == 8'h14) ||
         (dreg_offset == 8'h18) || (dreg_offset == 8'h1c) ||
         (dreg_offset == 8'h20) || (dreg_offset == 8'h24) ||
         (dreg_offset == 8'h28) || (dreg_offset == 8'h30) ||
         (dreg_offset == 8'h38) ||
         (dreg_offset == 8'h3c) || (dreg_offset == 8'h40) ||
         (dreg_offset == 8'h44) || (dreg_offset == 8'h48) ||
         (dreg_offset == 8'h4c) || (dreg_offset == 8'h50) ||
         (dreg_offset == 8'h54) || (dreg_offset == 8'h58) ||
         (dreg_offset == 8'h84) || (dreg_offset == 8'h88));
    wire [31:0] solid_trigger_command =
        (dreg_offset == 8'h00) ? wd : command;
    wire [11:0] solid_trigger_xstart =
        (dreg_offset == 8'h0c) ? wd[11:0] :
        (dreg_offset == 8'h10) ? wd[22:11] :
        (dreg_offset == 8'h14) ? wd[26:15] : xstarti;
    wire [11:0] solid_trigger_ystart =
        (dreg_offset == 8'h1c) ? wd[11:0] :
        (dreg_offset == 8'h20) ? wd[22:11] :
        (dreg_offset == 8'h24) ? wd[26:15] : ystarti;
    wire [11:0] solid_trigger_xend =
        (dreg_offset == 8'h18) ? wd[22:11] :
        (dreg_offset == 8'h84) ? wd[11:0] : xendi;
    wire [11:0] solid_trigger_yend =
        (dreg_offset == 8'h28) ? wd[22:11] :
        (dreg_offset == 8'h88) ? wd[11:0] : yendi;
    wire [19:0] solid_trigger_color0 =
        (dreg_offset == 8'h38) ? {1'b0, wd[7:0], 11'h0} :
        (dreg_offset == 8'h3c) ? wd[19:0] : color0;
    wire solid_trigger_color0_after_xstate =
        (dreg_offset == 8'h08) ? 1'b0 :
        ((dreg_offset == 8'h38) || (dreg_offset == 8'h3c)) ? 1'b1 :
        color0_after_xstate;
    wire solid_trigger_xstate_after_command =
        (dreg_offset == 8'h00) ? 1'b0 :
        (dreg_offset == 8'h08) ? 1'b1 : xstate_after_command;
    wire signed [20:0] solid_trigger_slope0 =
        (dreg_offset == 8'h50) ? slope_from_word(wd) : slope0;
    wire [19:0] solid_trigger_color1 =
        (dreg_offset == 8'h40) ? {1'b0, wd[7:0], 11'h0} :
        (dreg_offset == 8'h44) ? wd[19:0] : color1;
    wire [19:0] solid_trigger_color2 =
        (dreg_offset == 8'h48) ? {1'b0, wd[7:0], 11'h0} :
        (dreg_offset == 8'h4c) ? wd[19:0] : color2;
    wire signed [20:0] solid_trigger_slope1 =
        (dreg_offset == 8'h54) ? slope_from_word(wd) : slope1;
    wire signed [20:0] solid_trigger_slope2 =
        (dreg_offset == 8'h58) ? slope_from_word(wd) : slope2;
    wire [16:0] line_trigger_minorslope =
        (dreg_offset == 8'h30) ? minorslope_from_word(wd) : minorslope;
    wire line_trigger_minor_negative =
        (dreg_offset == 8'h30) ? wd[31] : minorslope_negative;
    wire [26:0] line_trigger_xstart_fixed =
        (dreg_offset == 8'h0c) ? {wd[11:0], 15'h0} :
        (dreg_offset == 8'h10) ? {wd[22:0], 4'h0} :
        (dreg_offset == 8'h14) ? wd[26:0] : xstart_fixed;
    wire [26:0] line_trigger_ystart_fixed =
        (dreg_offset == 8'h1c) ? {wd[11:0], 15'h0} :
        (dreg_offset == 8'h20) ? {wd[22:0], 4'h0} :
        (dreg_offset == 8'h24) ? wd[26:0] : ystart_fixed;
    wire [14:0] line_trigger_raw_fraction =
        ((solid_trigger_command & CMD_XMAJOR) != 32'h0) ?
        line_trigger_ystart_fixed[14:0] :
        line_trigger_xstart_fixed[14:0];
    wire [14:0] line_trigger_fraction =
        ((solid_trigger_command & CMD_INITFRAC) != 32'h0) ? 15'h4000 :
        (line_trigger_minor_negative ?
         ((~line_trigger_raw_fraction) + 15'd1) :
         line_trigger_raw_fraction);
    // LRQPOLY asks the polygon rasterizer to reject an empty scanline whose
    // initial left endpoint lies to the right of its right endpoint. Software
    // deliberately emits those rows without a software
    // order check.  This applies only to the observed QUAD/STOPONX/INITFRAC
    // polygon topology; LRQPOLY alpha-point streams and ordinary explicit
    // right-to-left primitives retain their separate behavior.
    wire solid_trigger_lrqpoly_reversed_span =
        ((solid_trigger_command & CMD_LRQPOLY) != 32'h0) &&
        command_is_quad_initfrac_span(solid_trigger_command) &&
        (solid_trigger_xstart > solid_trigger_xend);
    wire solid_go_write =
        solid_trigger_write && should_start_solid(solid_trigger_command,
                                                  xstate,
                                                  solid_trigger_xstate_after_command) &&
        !solid_trigger_lrqpoly_reversed_span;
    wire line_go_write =
        solid_trigger_write && should_start_line(solid_trigger_command,
                                                 xstate,
                                                 solid_trigger_xstate_after_command);
    wire [31:0] pattern_trigger_command = solid_trigger_command;
    wire solid_trigger_lrqpoly_point =
        command_is_lrqpoly_point(solid_trigger_command) &&
        ((solid_trigger_command & CMD_ENZPAT) == 32'h0);
    wire pattern_trigger_lrqpoly_point =
        command_is_lrqpoly_point(pattern_trigger_command) &&
        ((pattern_trigger_command & CMD_ENZPAT) != 32'h0);
    wire pattern_data_go_write =
        valid && write && go && !config_region &&
        (((dreg_offset == 8'h60) &&
          ((command & CMD_ENZPAT) != 32'h0)) ||
         ((dreg_offset == 8'h64) &&
          ((command & CMD_ENLSPAT) != 32'h0))) &&
        should_start_pattern(command, xstate, xstate_after_command);
    wire pattern_alias_go_write =
        solid_trigger_write &&
        should_start_pattern(pattern_trigger_command, xstate,
                             solid_trigger_xstate_after_command) &&
        !should_start_line(pattern_trigger_command, xstate,
                           solid_trigger_xstate_after_command);
    wire pattern_go_write =
        (pattern_data_go_write || pattern_alias_go_write) &&
        !rwaux_stream_done;
    wire [31:0] pattern_trigger_word =
        ((dreg_offset == 8'h60) || (dreg_offset == 8'h64)) ? wd :
        (((pattern_trigger_command & CMD_ENZPAT) != 32'h0) ?
         zpattern : lspattern);
    wire pattern_trigger_x_down =
        solid_trigger_xstart > solid_trigger_xend;
    wire pattern_trigger_y_down =
        solid_trigger_ystart > solid_trigger_yend;
    wire pattern_trigger_block =
        (pattern_trigger_command & CMD_BLOCK) != 32'h0;
    wire pattern_trigger_xycontinue =
        (pattern_trigger_command & CMD_XYCONTINUE) != 32'h0;
    wire [11:0] pattern_trigger_base_x =
        pattern_trigger_xycontinue ? xcurri : solid_trigger_xstart;
    wire [11:0] pattern_trigger_base_y =
        pattern_trigger_xycontinue ? ycurri : solid_trigger_ystart;
    wire [11:0] lrqpoly_trigger_next_x = pattern_trigger_x_down ?
        (xcurri - 12'd1) : (xcurri + 12'd1);
    wire [31:0] copy_trigger_command = solid_trigger_command;
    wire [31:0] copy_trigger_xymove =
        (dreg_offset == 8'h34) ? (wd & 32'h07ff07ff) : xymove;
    wire copy_go_write =
        ENABLE_COPY && valid && write && go && !config_region &&
        ((solid_trigger_write &&
          should_start_copy(copy_trigger_command, xstate,
                            solid_trigger_xstate_after_command)) ||
         ((dreg_offset == 8'h34) &&
          should_start_copy(command, xstate, xstate_after_command)));
    // REX GO is an address attribute, not a write-only operation.  The PROM
    // uses a read of GO.COMMAND to launch each row of its welcome-screen
    // background, so accepting the read without queuing the current command
    // silently drops the entire primitive stream.
    wire command_go_read =
        valid && !write && go && !config_region && (dreg_offset == 8'h00);
    // A GO.COMMAND read launches the programmed primitive without supplying
    // RWAUX data. Diagnostic software can leave XSTATE.COLORAUX set after
    // its register test, then uses this read to launch rex_clear's solid
    // block.  Do not let that stale data-port selector suppress the launch.
    // Geometry GO writes use COMMAND/XSTATE write ordering; GO.RWAUX retains
    // the full OR rule required by host streams.
    wire command_lrqpoly_reversed_span =
        ((command & CMD_LRQPOLY) != 32'h0) &&
        command_is_quad_initfrac_span(command) &&
        (xstarti > xendi);
    wire solid_go_read =
        command_go_read &&
        should_start_solid(command, xstate & ~XSTATE_COLORAUX,
                           xstate_after_command) &&
        !command_lrqpoly_reversed_span;
    wire line_go_read =
        command_go_read &&
        should_start_line(command, xstate, xstate_after_command);
    wire pattern_go_read =
        command_go_read &&
        should_start_pattern(command, xstate, xstate_after_command) &&
        !should_start_line(command, xstate, xstate_after_command) &&
        !rwaux_stream_done;
    wire copy_go_read =
        ENABLE_COPY && command_go_read &&
        should_start_copy(command, xstate, xstate_after_command);
    wire ldpixel_go_read =
        valid && !write && go && !config_region &&
        (dreg_offset == 8'h70) && (command[2:0] == 3'h3);
    wire ldpixel_go_command_write =
        valid && write && go && !config_region &&
        (dreg_offset == 8'h00) && (wd[2:0] == 3'h3);
    wire rwaux_read_waits_for_load =
        valid && !write && !config_region && (dreg_offset == 8'h70) &&
        !ldpixel_go_read && load_pending;
    wire ldpixel_go_access =
        ldpixel_go_read || ldpixel_go_command_write;
    wire [31:0] command_go_value = write ? wd : command;
    wire lspatsave_go_access =
        valid && go && !config_region && (dreg_offset == 8'h00) &&
        (command_go_value[2:0] == 3'h2);
    wire needs_context =
        rwaux_go_write || solid_go_write || line_go_write ||
        pattern_go_write || copy_go_write ||
        solid_go_read || line_go_read || pattern_go_read || copy_go_read;
    wire context_slot_ready = !ctx_valid || ctx_ready;
    wire accept = valid &&
        (ldpixel_go_access ? load_command_committed :
         lspatsave_go_access ? !chip_busy :
         rwaux_read_waits_for_load ? !load_pending :
         (!needs_context || context_slot_ready));

    // A load response belongs to the cursor generation that launched it.
    // Software may reset the cursor for its alpha-point stream while the
    // final LDPIXEL prefetch is still outstanding.  Retain the returned data,
    // but never let that older response retire over a newer explicit cursor
    // mutation or primitive launch.
    wire load_cursor_superseding_access =
        accept && !config_region && !ldpixel_go_access &&
        (command_go_read || rwaux_go_write || solid_go_write ||
         line_go_write || pattern_go_write || copy_go_write ||
         (write && (((dreg_offset == 8'h00) && go) ||
                    (dreg_offset == 8'h0c) ||
                    (dreg_offset == 8'h10) ||
                    (dreg_offset == 8'h14) ||
                    (dreg_offset == 8'h1c) ||
                    (dreg_offset == 8'h20) ||
                    (dreg_offset == 8'h24))));

    assign ready = accept;
    assign load_valid =
        ldpixel_go_access && !load_pending && !load_command_committed;
    assign load_rsp_ready = load_pending;

    function [31:0] bus_wd;
        input [63:0] data;
        input [ 7:0] mask;
        begin
            bus_wd = (mask[7] || mask[6] || mask[5] || mask[4]) ?
                     data[63:32] : data[31:0];
        end
    endfunction

    function [3:0] bus_wd_lane_mask;
        input [7:0] mask;
        begin
            bus_wd_lane_mask = (mask[7] || mask[6] || mask[5] || mask[4]) ?
                               {mask[4], mask[5], mask[6], mask[7]} :
                               {mask[0], mask[1], mask[2], mask[3]};
        end
    endfunction

    // Register byte enables use architectural byte order, unlike RWAUX where
    // the same host enables select successive pixel lanes. Keep both decodes
    // explicit so a partial RWMASK write cannot erase the untouched byte.
    wire [3:0] wd_reg_mask =
        (wmask[7] || wmask[6] || wmask[5] || wmask[4]) ?
        {wmask[7], wmask[6], wmask[5], wmask[4]} :
        {wmask[3], wmask[2], wmask[1], wmask[0]};

    function [31:0] merge32;
        input [31:0] old_value;
        input [31:0] new_value;
        input [ 3:0] mask;
        begin
            merge32 = old_value;
            if (mask[3]) merge32[31:24] = new_value[31:24];
            if (mask[2]) merge32[23:16] = new_value[23:16];
            if (mask[1]) merge32[15:8] = new_value[15:8];
            if (mask[0]) merge32[7:0] = new_value[7:0];
        end
    endfunction

    function [15:0] merge_rwmask;
        input [15:0] old_value;
        input [31:0] new_value;
        input [ 3:0] mask;
        reg [31:0] merged;
        begin
            merged = merge32({16'h0, old_value}, new_value, mask);
            merge_rwmask = merged[15:0];
        end
    endfunction

    function command_draw;
        input [31:0] cmd;
        begin
            command_draw = (cmd[2:0] == 3'h1);
        end
    endfunction

    function command_effective_coloraux;
        input [31:0] cmd;
        input [31:0] xs;
        input        xs_after_cmd;
        begin
            command_effective_coloraux =
                ((cmd & CMD_COLORAUX) != 32'h0) ||
                (xs_after_cmd && ((xs & XSTATE_COLORAUX) != 32'h0));
        end
    endfunction

    function [1:0] plane_from_aux2;
        input [31:0] value;
        begin
            case (value[30:29])
                AUX2_PIXELPLANES:   plane_from_aux2 = 2'h0;
                AUX2_OVERLAYPLANES: plane_from_aux2 = 2'h2;
                AUX2_CIDPLANES:     plane_from_aux2 = 2'h3;
                default:            plane_from_aux2 = 2'h0;
            endcase
        end
    endfunction

    function [255:0] smask_pack;
        input unused;
        begin
            smask_pack = {
                smask[7], smask[6], smask[5], smask[4],
                smask[3], smask[2], smask[1], smask[0]
            };
        end
    endfunction

    function command_is_quad_initfrac_span;
        input [31:0] cmd;
        begin
            // The compatible polygon rasterizer launches each
            // horizontal row through GO.XSTARTI with this exact geometry
            // topology.  Keep INITFRAC line commands and unproved QUAD
            // combinations out of the solid path.
            command_is_quad_initfrac_span =
                ((cmd & (CMD_BLOCK | CMD_QUADMODE | CMD_XMAJOR |
                         CMD_XYCONTINUE | CMD_STOPONX | CMD_STOPONY |
                         CMD_INITFRAC)) ==
                 (CMD_QUADMODE | CMD_STOPONX | CMD_INITFRAC));
        end
    endfunction

    function command_is_initfrac_point;
        input [31:0] cmd;
        begin
            // The compatible point path launches GO.COMMAND 3004c001 after
            // writing the final X/Y float aliases. With no extent/iteration
            // flags this is one pixel; INITFRAC alone must not suppress it.
            // Keep stopped lines and unproved QUAD/continuation/length
            // combinations on their existing paths.
            command_is_initfrac_point =
                ((cmd & (CMD_BLOCK | CMD_LENGTH32 | CMD_QUADMODE |
                         CMD_XMAJOR | CMD_XYCONTINUE | CMD_STOPONX |
                         CMD_STOPONY | CMD_INITFRAC)) == CMD_INITFRAC);
        end
    endfunction

    function command_is_lrqpoly_point;
        input [31:0] cmd;
        begin
            // The compatible alpha-text path emits one command per
            // coverage sample.  LRQPOLY+XYCONTINUE advances exactly one X;
            // ENZPAT marks a transparent sample, while the otherwise
            // identical command emits one RGB point.
            command_is_lrqpoly_point =
                ((cmd & (CMD_BLOCK | CMD_LENGTH32 | CMD_QUADMODE |
                         CMD_XMAJOR | CMD_XYCONTINUE | CMD_STOPONX |
                         CMD_STOPONY | CMD_INITFRAC | CMD_LRQPOLY)) ==
                 (CMD_XMAJOR | CMD_XYCONTINUE | CMD_INITFRAC |
                  CMD_LRQPOLY));
        end
    endfunction

    function should_start_solid;
        input [31:0] cmd;
        input [31:0] xs;
        input        xs_after_cmd;
        begin
            should_start_solid =
                command_draw(cmd) &&
                !command_effective_coloraux(cmd, xs, xs_after_cmd) &&
                ((cmd & (CMD_ENZPAT | CMD_ENLSPAT)) == 32'h0) &&
                ((cmd & CMD_LOGICSRC) == 32'h0) &&
                (((cmd & CMD_BLOCK) != 32'h0) ||
                 ((cmd & (CMD_XMAJOR | CMD_INITFRAC)) == 32'h0) ||
                 command_is_quad_initfrac_span(cmd) ||
                 command_is_initfrac_point(cmd) ||
                 command_is_lrqpoly_point(cmd));
        end
    endfunction

    function should_start_line;
        input [31:0] cmd;
        input [31:0] xs;
        input        xs_after_cmd;
        begin
            should_start_line =
                command_draw(cmd) &&
                !command_effective_coloraux(cmd, xs, xs_after_cmd) &&
                ((cmd & (CMD_BLOCK | CMD_QUADMODE | CMD_XYCONTINUE |
                         CMD_LOGICSRC | CMD_ENZPAT)) == 32'h0) &&
                ((cmd & (CMD_STOPONX | CMD_STOPONY)) != 32'h0) &&
                ((cmd & (CMD_XMAJOR | CMD_INITFRAC | CMD_ENLSPAT)) !=
                 32'h0);
        end
    endfunction

    function should_start_copy;
        input [31:0] cmd;
        input [31:0] xs;
        input        xs_after_cmd;
        begin
            should_start_copy =
                command_draw(cmd) &&
                !command_effective_coloraux(cmd, xs, xs_after_cmd) &&
                ((cmd & CMD_LOGICSRC) != 32'h0) &&
                ((cmd & CMD_BLOCK) != 32'h0);
        end
    endfunction

    function [11:0] move_x_delta;
        input [31:0] value;
        begin
            move_x_delta = {value[26], value[26:16]};
        end
    endfunction

    function [11:0] move_y_delta;
        input [31:0] value;
        begin
            move_y_delta = {value[10], value[10:0]};
        end
    endfunction

    function signed [20:0] slope_from_word;
        input [31:0] raw;
        reg [31:0] encoded;
        reg [20:0] transformed;
        begin
            encoded = raw & 32'h801fffff;
            if (encoded[31]) begin
                encoded = encoded & 32'hffefffff;
                encoded = ~encoded + 32'd1;
            end
            transformed = encoded[20:0];
            transformed[20] = encoded[20] | encoded[19];
            slope_from_word = $signed(transformed);
        end
    endfunction

    function [16:0] minorslope_from_word;
        input [31:0] raw;
        reg [31:0] encoded;
        begin
            encoded = raw & 32'h8001ffff;
            if (encoded[31]) begin
                encoded = encoded & 32'hfffeffff;
                encoded = ~encoded + 32'd1;
            end
            minorslope_from_word = encoded[16:0];
        end
    endfunction

    // LIGHT supports both ways of supplying the 0x800 hardware coordinate
    // bias: integer/X streams put it in XYWIN, while GL float-to-fixed streams
    // may put it in the coordinate and leave XYWIN as the plain window offset.
    // Some software paths retain the bias in both. Remove only that
    // duplicated bias; preserving either single-bias form keeps their shared
    // framebuffer translation identical.
    function [11:0] coordinate_without_duplicate_bias;
        input [11:0] coord;
        input [11:0] origin;
        begin
            coordinate_without_duplicate_bias =
                (coord[11] && origin[11]) ? (coord - 12'h800) : coord;
        end
    endfunction

    function [11:0] visible_axis_coord;
        input [11:0] coord;
        input [11:0] origin;
        reg signed [13:0] translated;
        begin
            translated = $signed({2'b00,
                coordinate_without_duplicate_bias(coord, origin)}) +
                         $signed({2'b00, origin}) - 14'sd2048;
            visible_axis_coord = translated[11:0];
        end
    endfunction

    function [11:0] visible_x_coord;
        input [11:0] coord;
        begin
            visible_x_coord = visible_axis_coord(coord, xywin[27:16]);
        end
    endfunction

    function [11:0] visible_y_coord;
        input [11:0] coord;
        begin
            visible_y_coord = visible_axis_coord(coord, xywin[11:0]);
        end
    endfunction

    task enqueue_solid_context;
        input [31:0] cmd;
        input [11:0] sx;
        input [11:0] sy;
        input [11:0] ex;
        input [11:0] ey;
        input [19:0] fill_color;
        input        fill_color_after_xstate;
        input [19:0] fill_color1;
        input [19:0] fill_color2;
        input signed [20:0] fill_slope;
        input signed [20:0] fill_slope1;
        input signed [20:0] fill_slope2;
        begin
            ctx_valid <= 1'b1;
            ctx_rwaux_write <= 1'b0;
            ctx_solid_fill <= 1'b1;
            ctx_pattern_fill <= 1'b0;
            ctx_pattern_tail <= 1'b0;
            ctx_copy_fill <= 1'b0;
            ctx_line_draw <= 1'b0;
            ctx_shade <= (cmd & CMD_SHADE) != 32'h0;
            ctx_command <= cmd;
            ctx_data <= 32'h0;
            ctx_lsmode <= lsmode;
            ctx_ls_reload <= 1'b0;
            ctx_lane_mask <= 4'h0;
            ctx_x <= visible_x_coord(sx);
            ctx_y <= visible_y_coord(sy);
            ctx_xstart <= visible_x_coord(sx);
            ctx_xend <= (((cmd & CMD_BLOCK) != 32'h0) ||
                         ((cmd & CMD_STOPONX) != 32'h0)) ?
                        visible_x_coord(ex) : visible_x_coord(sx);
            ctx_yend <= (((cmd & CMD_BLOCK) != 32'h0) ||
                         ((cmd & CMD_STOPONY) != 32'h0)) ?
                        visible_y_coord(ey) : visible_y_coord(sy);
            ctx_x_down <= (((cmd & CMD_BLOCK) != 32'h0) ||
                           ((cmd & CMD_STOPONX) != 32'h0)) &&
                          (sx > ex);
            ctx_y_down <= (((cmd & CMD_BLOCK) != 32'h0) ||
                           ((cmd & CMD_STOPONY) != 32'h0)) &&
                          (sy > ey);
            ctx_block <= (cmd & CMD_BLOCK) != 32'h0;
            ctx_clip_xend <= 1'b0;
            ctx_quadmode <= (cmd & CMD_QUADMODE) != 32'h0;
            ctx_plane <= plane_from_aux2(aux2);
            ctx_color <= solid_foreground_color(
                cmd, xstate, fill_color, fill_color_after_xstate);
            ctx_color_fixed <= fill_color;
            ctx_color1_fixed <= fill_color1;
            ctx_color2_fixed <= fill_color2;
            ctx_slope <= fill_slope;
            ctx_slope1 <= fill_slope1;
            ctx_slope2 <= fill_slope2;
            ctx_minor_slope <= 17'h0;
            ctx_minor_negative <= 1'b0;
            ctx_line_frac <= 15'h0;
            ctx_pattern_opaque <= 1'b0;
            ctx_bg_color <= background_color(xstate, colorback);
            ctx_copy_dx <= 12'h0;
            ctx_copy_dy <= 12'h0;
            ctx_aux2 <= aux2;
            ctx_smask <= smask_pack(1'b0);
            ctx_logic_op <= effective_logic_op(cmd, xstate);
            ctx_pixel_writemask <=
                effective_pixel_writemask(xstate, rwmask, aux1, aux2);
            ctx_colorcomp <= (cmd & CMD_COLORCOMP) != 32'h0;
            ctx_aux1 <= aux1[9:0];
            ctx_retire_x <= (((cmd & CMD_BLOCK) != 32'h0) ||
                             ((cmd & CMD_STOPONX) != 32'h0)) ? ex : sx;
            ctx_retire_y <= (((cmd & CMD_BLOCK) != 32'h0) ||
                             ((cmd & CMD_STOPONY) != 32'h0)) ? ey : sy;
            ctx_retire_done <= 1'b1;
        end
    endtask

    task enqueue_line_context;
        input [31:0] cmd;
        input [11:0] sx;
        input [11:0] sy;
        input [11:0] ex;
        input [11:0] ey;
        input [19:0] line_color;
        input [19:0] line_color1;
        input [19:0] line_color2;
        input signed [20:0] line_slope;
        input signed [20:0] line_slope1;
        input signed [20:0] line_slope2;
        input [16:0] minor_step;
        input minor_negative;
        input [14:0] line_fraction;
        begin
            ctx_valid <= 1'b1;
            ctx_rwaux_write <= 1'b0;
            ctx_solid_fill <= 1'b0;
            ctx_pattern_fill <= 1'b0;
            ctx_pattern_tail <= 1'b0;
            ctx_copy_fill <= 1'b0;
            ctx_line_draw <= 1'b1;
            ctx_shade <= (cmd & CMD_SHADE) != 32'h0;
            ctx_command <= cmd;
            ctx_data <= lspattern;
            ctx_lsmode <= lsmode;
            ctx_ls_reload <= ((cmd & CMD_ENLSPAT) != 32'h0) &&
                             ls_programmed_since_launch;
            ctx_lane_mask <= 4'h0;
            ctx_x <= visible_x_coord(sx);
            ctx_y <= visible_y_coord(sy);
            ctx_xstart <= visible_x_coord(sx);
            ctx_xend <= visible_x_coord(ex);
            ctx_yend <= visible_y_coord(ey);
            ctx_x_down <= sx > ex;
            ctx_y_down <= sy > ey;
            ctx_block <= 1'b0;
            ctx_clip_xend <= 1'b0;
            ctx_quadmode <= 1'b0;
            ctx_plane <= plane_from_aux2(aux2);
            // Use the same host-versus-direct-LIGHT stipple source split as
            // block/pattern launches. Explicit-ROP 30000a01/mode-03ff line
            // stipple consumes COLOR0, while mode-00ff consumes XSTATE.
            ctx_color <= pattern_foreground_color(
                cmd, xstate, aux2, line_color, color0_after_xstate,
                xstate_after_command, 1'b0);
            ctx_color_fixed <= line_color;
            ctx_color1_fixed <= line_color1;
            ctx_color2_fixed <= line_color2;
            ctx_slope <= line_slope;
            ctx_slope1 <= line_slope1;
            ctx_slope2 <= line_slope2;
            ctx_minor_slope <= minor_step;
            ctx_minor_negative <= minor_negative;
            ctx_line_frac <= line_fraction;
            ctx_pattern_opaque <=
                ((cmd & CMD_LSOPAQUE) != 32'h0) ||
                ((xstate & XSTATE_LSOPAQUE) != 32'h0);
            ctx_bg_color <= background_color(xstate, colorback);
            ctx_copy_dx <= 12'h0;
            ctx_copy_dy <= 12'h0;
            ctx_aux2 <= aux2;
            ctx_smask <= smask_pack(1'b0);
            ctx_logic_op <= effective_logic_op(cmd, xstate);
            ctx_pixel_writemask <=
                effective_pixel_writemask(xstate, rwmask, aux1, aux2);
            ctx_colorcomp <= (cmd & CMD_COLORCOMP) != 32'h0;
            ctx_aux1 <= aux1[9:0];
            ctx_retire_x <= ex;
            ctx_retire_y <= ey;
            ctx_retire_done <= 1'b1;
        end
    endtask

    task enqueue_copy_context;
        input [31:0] cmd;
        input [11:0] sx;
        input [11:0] sy;
        input [11:0] ex;
        input [11:0] ey;
        input [31:0] move;
        begin
            ctx_valid <= 1'b1;
            ctx_rwaux_write <= 1'b0;
            ctx_solid_fill <= 1'b0;
            ctx_pattern_fill <= 1'b0;
            ctx_pattern_tail <= 1'b0;
            ctx_copy_fill <= 1'b1;
            ctx_line_draw <= 1'b0;
            ctx_shade <= 1'b0;
            ctx_command <= cmd;
            ctx_data <= 32'h0;
            ctx_lsmode <= lsmode;
            ctx_ls_reload <= 1'b0;
            ctx_lane_mask <= 4'h0;
            ctx_x <= visible_x_coord(sx);
            ctx_y <= visible_y_coord(sy);
            ctx_xstart <= visible_x_coord(sx);
            ctx_xend <= visible_x_coord(ex);
            ctx_yend <= visible_y_coord(ey);
            ctx_x_down <= sx > ex;
            ctx_y_down <= sy > ey;
            ctx_block <= 1'b1;
            ctx_clip_xend <= 1'b0;
            ctx_quadmode <= (cmd & CMD_QUADMODE) != 32'h0;
            ctx_plane <= plane_from_aux2(aux2);
            ctx_color <= 8'h0;
            ctx_color_fixed <= 20'h0;
            ctx_color1_fixed <= 20'h0;
            ctx_color2_fixed <= 20'h0;
            ctx_slope <= 21'h0;
            ctx_slope1 <= 21'h0;
            ctx_slope2 <= 21'h0;
            ctx_minor_slope <= 17'h0;
            ctx_minor_negative <= 1'b0;
            ctx_line_frac <= 15'h0;
            ctx_pattern_opaque <= 1'b0;
            ctx_bg_color <= 8'h0;
            ctx_copy_dx <= move_x_delta(move);
            ctx_copy_dy <= move_y_delta(move);
            ctx_aux2 <= aux2;
            ctx_smask <= smask_pack(1'b0);
            ctx_logic_op <= effective_logic_op(cmd, xstate);
            ctx_pixel_writemask <=
                effective_pixel_writemask(xstate, rwmask, aux1, aux2);
            ctx_colorcomp <= (cmd & CMD_COLORCOMP) != 32'h0;
            ctx_aux1 <= aux1[9:0];
            ctx_retire_x <= ex;
            ctx_retire_y <= ey;
            ctx_retire_done <= 1'b1;
        end
    endtask

    task enqueue_pattern_context;
        input [31:0] cmd;
        input [31:0] pattern_data;
        input [11:0] sx;
        input [11:0] sy;
        input [11:0] start_x;
        input [11:0] end_x;
        input [11:0] end_y;
        input        x_down;
        input        y_down;
        input        block_mode;
        input [ 1:0] plane;
        input [ 7:0] fg_color;
        input        opaque;
        input [ 7:0] bg_color;
        input [31:0] aux2_value;
        input [255:0] smask_value;
        input [ 3:0] logic_op;
        input [ 7:0] pixel_writemask;
        input        colorcomp;
        input [ 9:0] aux1_value;
        input        tail_repeat;
        begin
            ctx_valid <= 1'b1;
            ctx_rwaux_write <= 1'b0;
            ctx_solid_fill <= 1'b0;
            ctx_pattern_fill <= 1'b1;
            ctx_pattern_tail <= tail_repeat;
            ctx_copy_fill <= 1'b0;
            ctx_line_draw <= 1'b0;
            ctx_shade <= (cmd & CMD_SHADE) != 32'h0;
            ctx_command <= cmd;
            ctx_data <= pattern_data;
            ctx_lsmode <= lsmode;
            ctx_ls_reload <= ((cmd & CMD_ENLSPAT) != 32'h0) &&
                             ls_programmed_since_launch;
            ctx_lane_mask <= 4'h0;
            ctx_x <= visible_x_coord(sx);
            ctx_y <= visible_y_coord(sy);
            ctx_xstart <= visible_x_coord(start_x);
            ctx_xend <= visible_x_coord(end_x);
            ctx_yend <= visible_y_coord(end_y);
            ctx_x_down <= x_down;
            ctx_y_down <= y_down;
            ctx_block <= block_mode;
            ctx_clip_xend <= 1'b0;
            ctx_quadmode <= 1'b0;
            ctx_plane <= plane;
            ctx_color <= fg_color;
            ctx_color_fixed <= color0;
            ctx_color1_fixed <= color1;
            ctx_color2_fixed <= color2;
            ctx_slope <= slope0;
            ctx_slope1 <= slope1;
            ctx_slope2 <= slope2;
            ctx_minor_slope <= 17'h0;
            ctx_minor_negative <= 1'b0;
            ctx_line_frac <= 15'h0;
            ctx_pattern_opaque <= opaque;
            ctx_bg_color <= bg_color;
            ctx_copy_dx <= 12'h0;
            ctx_copy_dy <= 12'h0;
            ctx_aux2 <= aux2_value;
            ctx_smask <= smask_value;
            ctx_logic_op <= logic_op;
            ctx_pixel_writemask <= pixel_writemask;
            ctx_colorcomp <= colorcomp;
            ctx_aux1 <= aux1_value;
            ctx_retire_x <= sx;
            ctx_retire_y <= sy;
            ctx_retire_done <= 1'b0;
        end
    endtask

    function should_start_pattern;
        input [31:0] cmd;
        input [31:0] xs;
        input        xs_after_cmd;
        begin
            should_start_pattern =
                command_draw(cmd) &&
                !command_effective_coloraux(cmd, xs, xs_after_cmd) &&
                (((cmd & CMD_ENZPAT) != 32'h0) ||
                 ((cmd & CMD_ENLSPAT) != 32'h0));
        end
    endfunction

    function [7:0] solid_foreground_color;
        input [31:0] cmd;
        input [31:0] xs;
        input [19:0] c0;
        input c0_after_xs;
        reg xsgi_segment_span;
        reg xsgi_block_constant;
        reg xsgi_point_constant;
        begin
            // The compatible segment path emits the 0x121 solid
            // span primitive with an explicit command ROP but still carries
            // its CI8 foreground in XSTATE[7:0].  Match the primitive bits,
            // while allowing its observed high command variants 30000121 and
            // 32000121.  Full-mask traffic is structurally unambiguous at
            // 00ff; rexSolidZeroSeg proves that the same packet can carry an
            // arbitrary nonzero GC planemask, so absence of a later COLOR0
            // write is the partial-mask discriminator.  The demo also uses
            // 32000121 with 03ff and a later COLOR0; that direct form must not
            // be cleared through XSTATE.
            xsgi_segment_span =
                ((cmd & 32'h000fffff) ==
                 (CMD_DRAW | CMD_QUADMODE | CMD_STOPONX)) &&
                ((xs[25:16] == 10'h0ff) ||
                 ((xs[25:16] != 10'h0) && !c0_after_xs));

            // The companion Xsgi rectangle edge uses 30000329.  Its XSTATE
            // source form has the observed 00ff planemask; partial masks use
            // the same no-later-COLOR0 provenance as the span above.  Direct
            // LIGHT uses the same command with 03ff and supplies COLOR0.
            xsgi_block_constant =
                ((cmd & 32'h000fffff) ==
                 (CMD_DRAW | CMD_BLOCK | CMD_QUADMODE |
                  CMD_STOPONX | CMD_STOPONY)) &&
                ((xs[25:16] == 10'h0ff) ||
                 ((xs[25:16] != 10'h0) && !c0_after_xs));

            // The host path emits explicit-ROP point command 30000001 with
            // its CI8 color in XSTATE and COLOR0 left stale.  Full-mask 00ff
            // is structurally unambiguous; arbitrary nonzero planemasks use
            // the same write-provenance split.  Direct LIGHT supplies COLOR0
            // after XSTATE.
            xsgi_point_constant =
                ((cmd & 32'h000fffff) == CMD_DRAW) &&
                ((xs[25:16] == 10'h0ff) ||
                 ((xs[25:16] != 10'h0) && !c0_after_xs));

            // The host path also delegates zero command-ROP constant color
            // to XSTATE (for example command 00000329).  Keep both decisions
            // structural rather than inferring them from register values.
            if ((xs[25:16] != 10'h0) &&
                ((cmd[31:28] == 4'h0) || xsgi_segment_span ||
                 xsgi_block_constant || xsgi_point_constant) &&
                ((cmd & (CMD_RGBMODE | CMD_SHADE)) == 32'h0))
                solid_foreground_color = xs[7:0];
            else
                solid_foreground_color = c0[18:11];
        end
    endfunction

    function [7:0] background_color;
        input [31:0] xs;
        input [7:0] cb;
        begin
            background_color = (xs[25:16] != 10'h0) ? xs[15:8] : cb;
        end
    endfunction

    function [7:0] pattern_foreground_color;
        input [31:0] cmd;
        input [31:0] xs;
        input [31:0] a2;
        input [19:0] c0;
        input c0_after_xs;
        input xs_after_cmd;
        input zero_go_5b9_xs;
        reg xstate_color_active;
        reg xsgi_window_source;
        reg xstate_lspat_source;
        reg xsgi_mono_source;
        reg xsgi_poly_glyph_source;
        reg xsgi_draw_mono_source;
        reg xsgi_zero_poly_arc_source;
        begin
            xstate_color_active =
                (xs[25:16] != 10'h0) &&
                ((cmd & (CMD_RGBMODE | CMD_SHADE)) == 32'h0);
            // One glyph path writes the foreground to
            // COLOR0 (REX offset 0x38) after programming XSTATE at offset
            // 0x08 and before launching command 0x300005b9.  LSMODE=f0000
            // is retained stipple state, not a color-source discriminator.
            xsgi_window_source = (a2[11:0] == 12'hf11);
            // The later AUX2=f11 window-source path and line-stipple path
            // retain their separately observed XSTATE foreground behavior.
            // Ordinary ENZPAT glyphs consume COLOR0.
            // A zero command ROP also delegates the foreground to XSTATE.
            // The stippled-fill path constructs XSTATE from the GC's
            // arbitrary nonzero planemask and colors, then emits explicit-ROP
            // ENLSPAT without writing COLOR0.  Full-mask 00ff is structurally
            // unambiguous; partial masks use the same no-later-COLOR0
            // provenance as solid primitives.  Direct LIGHT/demo ENLSPAT
            // uses explicit ROP plus mode 03ff and carries its foreground in
            // a later COLOR0 write.
            xstate_lspat_source =
                ((cmd & CMD_ENLSPAT) != 32'h0) &&
                ((cmd[31:28] == 4'h0) ||
                 (xs[25:16] == 10'h0ff) ||
                 ((xs[25:16] != 10'h0) && !c0_after_xs));
            // The mono-image paths load the CI8 foreground into
            // XSTATE, then stream the unambiguous 0x5b1/0x5a1 full-word and
            // 0x5a9 final-row ENZPAT forms without writing COLOR0.  The
            // XSTATE planemask comes directly from the GC and is not limited
            // to ff; use write provenance for its partial-mask form.  Keep
            // Keep 0x5b9 out of this selector: the glyph path uses that otherwise
            // similar form and explicitly supplies its foreground through
            // COLOR0.
            xsgi_mono_source =
                ((cmd & CMD_ENZPAT) != 32'h0) &&
                ((xs[25:16] == 10'h0ff) ||
                 ((xs[25:16] != 10'h0) && !c0_after_xs)) &&
                (((cmd & 32'h000fffff) == 32'h000005b1) ||
                 ((cmd & 32'h000fffff) == 32'h000005a1) ||
                 ((cmd & 32'h000fffff) == 32'h000005a9));
            // The polygon-glyph path also uses 0x5b9, but it does not load
            // COLOR0 after XSTATE.
            // Its XSTATE planemask is the caller's arbitrary GC mask, not a
            // fixed ff.  Preserve the no-later-COLOR0 packet distinction so
            // cached/poly text cannot consume a stale COLOR0 foreground.
            xsgi_poly_glyph_source =
                ((cmd & CMD_ENZPAT) != 32'h0) &&
                (xs[25:16] != 10'h0) &&
                ((cmd & 32'h000fffff) == 32'h000005b9) &&
                !c0_after_xs;
            // The short mono-image path uses 0x5b9, but
            // writes command first and then loads ROP/mask/foreground into
            // XSTATE before streaming GO.ZPATTERN.  Direct LIGHT bitmap text
            // uses 0x5b9 without that post-command XSTATE write.
            xsgi_draw_mono_source =
                ((cmd & CMD_ENZPAT) != 32'h0) &&
                ((cmd & 32'h000fffff) == 32'h000005b9) &&
                xs_after_cmd && !c0_after_xs;
            // The zero-arc path uses the opposite 5b9 order from the
            // mono-image path: mode-03ff XSTATE, GO.COMMAND=0, then
            // SET.COMMAND=ROP|5b9 and GO.ZPATTERN, with no COLOR0 write.
            // The banked sequence marker is necessary because direct LIGHT
            // also uses explicit-ROP 5b9 but supplies COLOR0.
            xsgi_zero_poly_arc_source =
                zero_go_5b9_xs && !c0_after_xs &&
                (xs[25:16] == 10'h3ff) &&
                ((cmd & CMD_ENZPAT) != 32'h0) &&
                ((cmd & 32'h000fffff) == 32'h000005b9);
            if (xstate_color_active &&
                (xstate_lspat_source || xsgi_mono_source ||
                 xsgi_poly_glyph_source || xsgi_draw_mono_source ||
                 xsgi_zero_poly_arc_source ||
                 (((cmd & CMD_ENZPAT) != 32'h0) &&
                  ((((cmd & CMD_BLOCK) != 32'h0) &&
                    xsgi_window_source) ||
                   (cmd[31:28] == 4'h0)))))
                pattern_foreground_color = xs[7:0];
            else
                pattern_foreground_color = c0[18:11];
        end
    endfunction

    function [7:0] pattern_background_color;
        input [31:0] cmd;
        input [31:0] xs;
        input [7:0] cb;
        begin
            if ((xs[25:16] != 10'h0) &&
                ((cmd & (CMD_RGBMODE | CMD_SHADE)) == 32'h0))
                pattern_background_color = xs[15:8];
            else
                pattern_background_color = cb;
        end
    endfunction

    function [3:0] effective_logic_op;
        input [31:0] cmd;
        input [31:0] xs;
        begin
            if (cmd[31:28] != 4'h0)
                effective_logic_op = cmd[31:28];
            else if (xstate_written)
                effective_logic_op = xs[27:24];
            else
                effective_logic_op = 4'h3;
        end
    endfunction

    function [7:0] effective_pixel_writemask;
        input [31:0] xs;
        input [15:0] mask;
        input [31:0] a1;
        input [31:0] a2;
        reg [7:0] base_mask;
        reg [3:0] logical_mask;
        begin
            base_mask = xstate_mask_active ? xs[23:16] : mask[7:0];
            logical_mask = base_mask[3:0];
            if ((a2[30:29] == AUX2_PIXELPLANES) && a1[5]) begin
                // Native packed-byte contract: DBLDST0 is low, DBLDST1
                // is high. Keep this paired with DBLSRC and XMAP CI4.
                case (a1[3:2])
                    2'b01: effective_pixel_writemask =
                        {4'h0, logical_mask};
                    2'b10: effective_pixel_writemask =
                        {logical_mask, 4'h0};
                    2'b11: effective_pixel_writemask =
                        {logical_mask, logical_mask};
                    default: effective_pixel_writemask = 8'h00;
                endcase
            end else begin
                effective_pixel_writemask = base_mask;
            end
        end
    endfunction

    function pattern_opaque;
        input [31:0] cmd;
        input [31:0] xs;
        begin
            pattern_opaque =
                (((cmd & CMD_ENZPAT) != 32'h0) &&
                 (((cmd & CMD_ZOPAQUE) != 32'h0) ||
                  ((xs & XSTATE_ZOPAQUE) != 32'h0))) ||
                (((cmd & CMD_ENLSPAT) != 32'h0) &&
                 (((cmd & CMD_LSOPAQUE) != 32'h0) ||
                  ((xs & XSTATE_LSOPAQUE) != 32'h0)));
        end
    endfunction

    function [24:0] rwaux_advance_word;
        input [11:0] cur_x;
        input [11:0] cur_y;
        input [11:0] start_x;
        input [11:0] end_x;
        input [11:0] end_y;
        input        x_down;
        input        y_down;
        input        block_mode;
        input        xycontinue;
        input        quadmode;
        reg signed [12:0] next_x_signed;
        reg signed [12:0] step_signed;
        reg        done;
        reg [11:0] next_x;
        reg [11:0] next_y;
        begin
            done = 1'b0;
            next_x = cur_x;
            next_y = cur_y;

            if (xycontinue) begin
                step_signed = quadmode ? 13'sd4 : 13'sd1;
                if (x_down)
                    next_x_signed = $signed({1'b0, cur_x}) - step_signed;
                else
                    next_x_signed = $signed({1'b0, cur_x}) + step_signed;

                if (block_mode &&
                    ((x_down &&
                      (next_x_signed < $signed({1'b0, end_x}))) ||
                     (!x_down &&
                      (next_x_signed > $signed({1'b0, end_x}))))) begin
                    if (cur_y == end_y) begin
                        next_x = next_x_signed[11:0];
                        done = 1'b1;
                    end else if (y_down) begin
                        if (cur_y > end_y) begin
                            next_x = start_x;
                            next_y = cur_y - 12'd1;
                        end else begin
                            next_x = next_x_signed[11:0];
                            done = 1'b1;
                        end
                    end else if (cur_y < end_y) begin
                        next_x = start_x;
                        next_y = cur_y + 12'd1;
                    end else begin
                        next_x = next_x_signed[11:0];
                        done = 1'b1;
                    end
                end else begin
                    next_x = next_x_signed[11:0];
                end
            end

            rwaux_advance_word = {done, next_y, next_x};
        end
    endfunction

    function [24:0] pattern_advance_word;
        input [11:0] cur_x;
        input [11:0] cur_y;
        input [11:0] start_x;
        input [11:0] end_x;
        input [11:0] end_y;
        input        x_down;
        input        y_down;
        input        block_mode;
        input        xycontinue;
        input        stop_on_y;
        input        line_stipple;
        reg signed [12:0] next_x_signed;
        reg signed [12:0] pattern_step;
        reg [11:0] pattern_distance;
        reg        done;
        reg [11:0] next_x;
        reg [11:0] next_y;
        begin
            done = 1'b0;
            next_x = cur_x;
            next_y = cur_y;
            if (xycontinue) begin
                if (x_down)
                    pattern_distance = cur_x - end_x;
                else
                    pattern_distance = end_x - cur_x;

                // ENZPAT supplies at most one 32-pixel host bitmap word.
                // ENLSPAT is different: each GO word is a programmable
                // stipple for the entire remaining STOPONX span. The host
                // path queues one ENLSPAT word per row, so its issue
                // cursor must wrap to XSTART/Y+1 after the full span rather
                // than project a second word at X+32 on the same row.
                if (line_stipple)
                    pattern_step = $signed({1'b0, pattern_distance}) + 13'sd1;
                else if (pattern_distance < 12'd31)
                    pattern_step = $signed({1'b0, pattern_distance}) + 13'sd1;
                else
                    pattern_step = 13'sd32;
                if (x_down)
                    next_x_signed = $signed({1'b0, cur_x}) - pattern_step;
                else
                    next_x_signed = $signed({1'b0, cur_x}) + pattern_step;

                if (block_mode &&
                    ((x_down &&
                      (next_x_signed < $signed({1'b0, end_x}))) ||
                     (!x_down &&
                      (next_x_signed > $signed({1'b0, end_x}))))) begin
                    if (stop_on_y && (cur_y == end_y)) begin
                        next_x = next_x_signed[11:0];
                        done = 1'b1;
                    end else if (y_down) begin
                        next_x = start_x;
                        next_y = cur_y - 12'd1;
                    end else begin
                        next_x = start_x;
                        next_y = cur_y + 12'd1;
                    end
                end else begin
                    next_x = next_x_signed[11:0];
                end
            end
            pattern_advance_word = {done, next_y, next_x};
        end
    endfunction

    wire x_down_now = xstarti > xendi;
    wire y_down_now = ystarti > yendi;
    wire ctx_block_now = (command & CMD_BLOCK) != 32'h0;
    wire ctx_xycontinue_now = (command & CMD_XYCONTINUE) != 32'h0;
    wire ctx_quadmode_now = (command & CMD_QUADMODE) != 32'h0;
    wire [11:0] ctx_base_x_now = ctx_xycontinue_now ? xcurri : xstarti;
    wire [11:0] ctx_base_y_now = ctx_xycontinue_now ? ycurri : ystarti;
    wire [24:0] rwaux_next =
        rwaux_advance_word(xcurri, ycurri, xstarti, xendi, yendi,
                           x_down_now, y_down_now, ctx_block_now,
                           ctx_xycontinue_now, ctx_quadmode_now);
    wire [24:0] pattern_next =
        pattern_advance_word(xcurri, ycurri, xstarti, xendi, yendi,
                             x_down_now, y_down_now, ctx_block_now,
                             ctx_xycontinue_now,
                             (command & CMD_STOPONY) != 32'h0,
                             (command & CMD_ENLSPAT) != 32'h0);
    wire [24:0] pattern_trigger_word_next =
        pattern_advance_word(xcurri, ycurri, solid_trigger_xstart,
                             solid_trigger_xend, solid_trigger_yend,
                             pattern_trigger_x_down, pattern_trigger_y_down,
                             pattern_trigger_block,
                             pattern_trigger_xycontinue,
                             (pattern_trigger_command & CMD_STOPONY) != 32'h0,
                             (pattern_trigger_command & CMD_ENLSPAT) != 32'h0);
    wire [24:0] pattern_trigger_next = pattern_trigger_lrqpoly_point ?
        {1'b0, ycurri, lrqpoly_trigger_next_x} :
        pattern_trigger_word_next;
    wire [31:0] load_trigger_command =
        ldpixel_go_command_write ? wd : command;
    wire load_trigger_block =
        (load_trigger_command & CMD_BLOCK) != 32'h0;
    wire load_trigger_xycontinue =
        (load_trigger_command & CMD_XYCONTINUE) != 32'h0;
    wire load_trigger_quadmode =
        (load_trigger_command & CMD_QUADMODE) != 32'h0;
    wire [11:0] load_trigger_base_x =
        load_trigger_xycontinue ? xcurri : xstarti;
    wire [11:0] load_trigger_base_y =
        load_trigger_xycontinue ? ycurri : ystarti;
    wire [24:0] load_trigger_next =
        rwaux_advance_word(xcurri, ycurri, xstarti, xendi, yendi,
                           x_down_now, y_down_now, load_trigger_block,
                           load_trigger_xycontinue ||
                           ldpixel_go_command_write,
                           load_trigger_quadmode);
    assign load_x = visible_x_coord(load_trigger_base_x);
    assign load_y = visible_y_coord(load_trigger_base_y);
    assign load_xend = visible_x_coord(xendi);
    assign load_x_down = x_down_now;
    assign load_block = load_trigger_block;
    assign load_quadmode = load_trigger_quadmode;
    assign load_exhausted = rwaux_stream_done;
    assign load_plane = plane_from_aux2(aux2);
    assign load_aux1 = aux1[9:0];
    assign load_is_pixel = aux2[30:29] == AUX2_PIXELPLANES;

    always @(*) begin
        rdata = 64'h0;
        if (config_region) begin
            case (cfg_offset)
                11'h790: rdata = {smask[2], smask[2]};
                11'h794: rdata = {smask[3], smask[3]};
                11'h798: rdata = {smask[4], smask[4]};
                11'h79c: rdata = {smask[5], smask[5]};
                11'h7a0: rdata = {smask[6], smask[6]};
                11'h7a4: rdata = {smask[7], smask[7]};
                11'h7a8: rdata = {aux2, aux2};
                11'h7ec: rdata = {29'h0, configsel, 29'h0, configsel};
                11'h7f4: rdata = {togglectxt, togglectxt};
                // CHIPBUSY is live pipeline status, not writable storage.
                // LIGHT's RexWait polls bit zero after issuing a command.
                11'h7f8: rdata = {
                    (configmode & 32'hfffffffe) | {31'h0, chip_busy},
                    (configmode & 32'hfffffffe) | {31'h0, chip_busy}
                };
                11'h7fc: rdata = {xywin, xywin};
                default: rdata = 64'h0;
            endcase
        end else begin
            case (dreg_offset)
                // The ten architectural context fields select current/next
                // storage through SET/GO read pages.  Singleton programmable
                // state reads live on both pages; current_* remains available
                // internally only as the immutable GO launch snapshot.
                8'h00: rdata = {command, command};
                8'h04: rdata = {aux1, aux1};
                8'h08: rdata = {xstate, xstate};
                8'h0c: rdata = go ? {20'h0, xstarti, 20'h0, xstarti} :
                                      {20'h0, current_xstarti,
                                       20'h0, current_xstarti};
                8'h10: rdata = go ? {9'h0, xstart_fixed[26:4],
                                      9'h0, xstart_fixed[26:4]} :
                                     {9'h0, current_xstart_fixed[26:4],
                                      9'h0, current_xstart_fixed[26:4]};
                8'h14: rdata = go ? {5'h0, xstart_fixed,
                                      5'h0, xstart_fixed} :
                                     {5'h0, current_xstart_fixed,
                                      5'h0, current_xstart_fixed};
                // LIGHT exposes only the integer XEND field through XENDF;
                // fractional bits read as zero (mask 0x007ff800).
                8'h18: rdata = {9'h0, xendi, 11'h0,
                                9'h0, xendi, 11'h0};
                8'h1c: rdata = go ? {20'h0, ystarti, 20'h0, ystarti} :
                                      {20'h0, current_ystarti,
                                       20'h0, current_ystarti};
                8'h20: rdata = go ? {9'h0, ystart_fixed[26:4],
                                      9'h0, ystart_fixed[26:4]} :
                                     {9'h0, current_ystart_fixed[26:4],
                                      9'h0, current_ystart_fixed[26:4]};
                8'h24: rdata = go ? {5'h0, ystart_fixed,
                                      5'h0, ystart_fixed} :
                                     {5'h0, current_ystart_fixed,
                                      5'h0, current_ystart_fixed};
                8'h28: rdata = {9'h0, yendi, 11'h0,
                                9'h0, yendi, 11'h0};
                8'h2c: rdata = go ? {20'h0, xsave, 20'h0, xsave} :
                                      {20'h0, current_xsave,
                                       20'h0, current_xsave};
                8'h30: rdata = {15'h0, minorslope, 15'h0, minorslope};
                8'h38: rdata = go ? {12'h0, color0, 12'h0, color0} :
                                      {12'h0, current_color0,
                                       12'h0, current_color0};
                8'h3c: rdata = go ? {12'h0, color0, 12'h0, color0} :
                                      {12'h0, current_color0,
                                       12'h0, current_color0};
                8'h40: rdata = go ? {12'h0, color1, 12'h0, color1} :
                                      {12'h0, current_color1,
                                       12'h0, current_color1};
                8'h44: rdata = go ? {12'h0, color1, 12'h0, color1} :
                                      {12'h0, current_color1,
                                       12'h0, current_color1};
                8'h48: rdata = go ? {12'h0, color2, 12'h0, color2} :
                                      {12'h0, current_color2,
                                       12'h0, current_color2};
                8'h4c: rdata = go ? {12'h0, color2, 12'h0, color2} :
                                      {12'h0, current_color2,
                                       12'h0, current_color2};
                8'h50: rdata = {11'h0, slope0, 11'h0, slope0};
                8'h54: rdata = {11'h0, slope1, 11'h0, slope1};
                8'h58: rdata = {11'h0, slope2, 11'h0, slope2};
                8'h34: rdata = {xymove & 32'h07ff07ff,
                                xymove & 32'h07ff07ff};
                8'h5c: rdata = {24'h0, colorback, 24'h0, colorback};
                8'h60: rdata = go ? {zpattern, zpattern} :
                                      {current_zpattern, current_zpattern};
                8'h64: rdata = go ? {lspattern, lspattern} :
                                      {current_lspattern, current_lspattern};
                8'h68: rdata = go ? {12'h0, lsmode, 12'h0, lsmode} :
                                      {12'h0, current_lsmode,
                                       12'h0, current_lsmode};
                8'h6c: rdata = {aweight, aweight};
                8'h70: rdata = go ? {rwaux1, rwaux1} :
                                      {current_rwaux1, current_rwaux1};
                8'h74: rdata = go ? {rwaux2, rwaux2} :
                                      {current_rwaux2, current_rwaux2};
                8'h78: rdata = {16'h0, rwmask, 16'h0, rwmask};
                8'h7c: rdata = {smask[0], smask[0]};
                8'h80: rdata = {smask[1], smask[1]};
                8'h84: rdata = {20'h0, xendi, 20'h0, xendi};
                8'h88: rdata = {20'h0, yendi, 20'h0, yendi};
                default: rdata = 64'h0;
            endcase
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            command <= 32'h0;
            aux1 <= 32'h0;
            xstate <= 32'h0;
            xstate_written <= 1'b0;
            xstate_mask_active <= 1'b0;
            rwmask <= 16'hffff;
            aux2 <= 32'h0;
            smask[0] <= 32'h0;
            smask[1] <= 32'h0;
            smask[2] <= 32'h0;
            smask[3] <= 32'h0;
            smask[4] <= 32'h0;
            smask[5] <= 32'h0;
            smask[6] <= 32'h0;
            smask[7] <= 32'h0;
            xywin <= 32'h08000800;
            configmode <= 32'h0;
            togglectxt <= 32'h0;
            configsel <= 3'h0;
            rwaux1 <= 32'h0;
            rwaux2 <= 32'h0;
            color0 <= 20'h0;
            color0_after_xstate <= 1'b0;
            xstate_after_command <= 1'b0;
            zero_go_5b9_xstate_source <= 1'b0;
            color1 <= 20'h0;
            color2 <= 20'h0;
            slope0 <= 21'sh0;
            slope1 <= 21'sh0;
            slope2 <= 21'sh0;
            minorslope <= 17'h0;
            minorslope_negative <= 1'b0;
            colorback <= 8'h0;
            zpattern <= 32'h0;
            lspattern <= 32'h0;
            lsmode <= 20'h0;
            ls_programmed_since_launch <= 1'b0;
            aweight <= 32'h0;
            xstart_fixed <= 27'h0;
            ystart_fixed <= 27'h0;
            xend_fixed <= 27'h0;
            yend_fixed <= 27'h0;
            xstarti <= 12'h0;
            ystarti <= 12'h0;
            xendi <= 12'h0;
            yendi <= 12'h0;
            xcurri <= 12'h0;
            ycurri <= 12'h0;
            rwaux_stream_done <= 1'b0;
            xstart_explicit_since_primitive <= 1'b0;
            load_pending <= 1'b0;
            load_command_committed <= 1'b0;
            load_next_latched <= 25'h0;
            load_cursor_superseded <= 1'b0;
            current_command <= 32'h0;
            current_aux1 <= 32'h0;
            current_xstate <= 32'h0;
            current_xstate_written <= 1'b0;
            current_rwmask <= 16'hffff;
            current_aux2 <= 32'h0;
            current_smask[0] <= 32'h0;
            current_smask[1] <= 32'h0;
            current_smask[2] <= 32'h0;
            current_smask[3] <= 32'h0;
            current_smask[4] <= 32'h0;
            current_smask[5] <= 32'h0;
            current_smask[6] <= 32'h0;
            current_smask[7] <= 32'h0;
            current_rwaux1 <= 32'h0;
            current_rwaux2 <= 32'h0;
            current_color0 <= 20'h0;
            current_color0_after_xstate <= 1'b0;
            current_xstate_after_command <= 1'b0;
            current_zero_go_5b9_xstate_source <= 1'b0;
            current_color1 <= 20'h0;
            current_color2 <= 20'h0;
            current_slope0 <= 21'sh0;
            current_slope1 <= 21'sh0;
            current_slope2 <= 21'sh0;
            current_minorslope <= 17'h0;
            current_minorslope_negative <= 1'b0;
            current_colorback <= 8'h0;
            current_zpattern <= 32'h0;
            current_lspattern <= 32'h0;
            current_lsmode <= 20'h0;
            current_aweight <= 32'h0;
            current_xsave <= 12'h0;
            current_xymove <= 32'h0;
            current_xstart_fixed <= 27'h0;
            current_ystart_fixed <= 27'h0;
            current_xend_fixed <= 27'h0;
            current_yend_fixed <= 27'h0;
            current_xstarti <= 12'h0;
            current_ystarti <= 12'h0;
            current_xendi <= 12'h0;
            current_yendi <= 12'h0;
            current_xcurri <= 12'h0;
            current_ycurri <= 12'h0;
            current_rwaux_stream_done <= 1'b0;
            current_xstart_explicit_since_primitive <= 1'b0;
            ctx_valid <= 1'b0;
            ctx_rwaux_write <= 1'b0;
            ctx_solid_fill <= 1'b0;
            ctx_pattern_fill <= 1'b0;
            ctx_pattern_tail <= 1'b0;
            ctx_copy_fill <= 1'b0;
            ctx_line_draw <= 1'b0;
            ctx_shade <= 1'b0;
            ctx_command <= 32'h0;
            ctx_data <= 32'h0;
            ctx_lsmode <= 20'h0;
            ctx_ls_reload <= 1'b0;
            ctx_lane_mask <= 4'h0;
            ctx_x <= 12'h0;
            ctx_y <= 12'h0;
            ctx_xstart <= 12'h0;
            ctx_xend <= 12'h0;
            ctx_yend <= 12'h0;
            ctx_x_down <= 1'b0;
            ctx_y_down <= 1'b0;
            ctx_block <= 1'b0;
            ctx_clip_xend <= 1'b0;
            ctx_quadmode <= 1'b0;
            ctx_plane <= 2'h0;
            ctx_color <= 8'h0;
            ctx_color_fixed <= 20'h0;
            ctx_color1_fixed <= 20'h0;
            ctx_color2_fixed <= 20'h0;
            ctx_slope <= 21'h0;
            ctx_slope1 <= 21'h0;
            ctx_slope2 <= 21'h0;
            ctx_minor_slope <= 17'h0;
            ctx_minor_negative <= 1'b0;
            ctx_line_frac <= 15'h0;
            ctx_pattern_opaque <= 1'b0;
            ctx_bg_color <= 8'h0;
            ctx_copy_dx <= 12'h0;
            ctx_copy_dy <= 12'h0;
            ctx_aux2 <= 32'h0;
            ctx_smask <= 256'h0;
            ctx_logic_op <= 4'h3;
            ctx_pixel_writemask <= 8'hff;
            ctx_colorcomp <= 1'b0;
            ctx_aux1 <= 10'h0;
            ctx_retire_x <= 12'h0;
            ctx_retire_y <= 12'h0;
            ctx_retire_done <= 1'b0;
            xsave <= 12'h0;
            xymove <= 32'h0;
        end else begin
            if (load_valid && load_ready) begin
                load_pending <= 1'b1;
                load_command_committed <= 1'b1;
                load_next_latched <= load_trigger_next;
                load_cursor_superseded <= 1'b0;
            end
            if (load_pending && load_cursor_superseding_access)
                load_cursor_superseded <= 1'b1;
            if (load_command_committed && !ldpixel_go_access)
                load_command_committed <= 1'b0;
            if (load_pending && load_rsp_valid) begin
                load_pending <= 1'b0;
                load_cursor_superseded <= 1'b0;
            end

            if (ctx_valid && ctx_ready)
                ctx_valid <= 1'b0;

            // Current coordinates describe work that has crossed RB and the
            // memory handshake, not merely work accepted into the command
            // FIFO.  This is the state saved by LIGHT after REXWAIT.
            if (retire_valid) begin
                current_xstarti <= retire_x;
                current_ystarti <= retire_y;
                current_xstart_fixed <= {retire_x, 15'h0};
                current_ystart_fixed <= {retire_y, 15'h0};
                current_xcurri <= retire_x;
                current_ycurri <= retire_y;
                current_rwaux_stream_done <= retire_done;
            end

            if (accept && write && config_region &&
                (cfg_offset == 11'h7f4)) begin
                // TOGGLECTXT is a strobe for exactly the ten logical fields
                // saved on both context sides. All
                // other current_* registers are GO snapshots and must not be
                // restored into the single programmable state.
                rwaux1 <= current_rwaux1;
                color0 <= current_color0;
                color0_after_xstate <= current_color0_after_xstate;
                color1 <= current_color1;
                color2 <= current_color2;
                zpattern <= current_zpattern;
                lspattern <= current_lspattern;
                lsmode <= current_lsmode;
                xsave <= current_xsave;
                xstart_fixed <= current_xstart_fixed;
                ystart_fixed <= current_ystart_fixed;
                xstarti <= current_xstarti;
                ystarti <= current_ystarti;
                xcurri <= current_xcurri;
                ycurri <= current_ycurri;
                rwaux_stream_done <= current_rwaux_stream_done;
                xstart_explicit_since_primitive <=
                    current_xstart_explicit_since_primitive;

                current_rwaux1 <= rwaux1;
                current_color0 <= color0;
                current_color0_after_xstate <= color0_after_xstate;
                current_color1 <= color1;
                current_color2 <= color2;
                current_zpattern <= zpattern;
                current_lspattern <= lspattern;
                current_lsmode <= lsmode;
                current_xsave <= xsave;
                current_xstart_fixed <= xstart_fixed;
                current_ystart_fixed <= ystart_fixed;
                current_xstarti <= xstarti;
                current_ystarti <= ystarti;
                current_xcurri <= xcurri;
                current_ycurri <= ycurri;
                current_rwaux_stream_done <= rwaux_stream_done;
                current_xstart_explicit_since_primitive <=
                    xstart_explicit_since_primitive;
            end else if (accept && go && !config_region) begin
                // GO copies the fully assembled next bank into current. A GO
                // write contributes its data to the copied register in the
                // same transaction.
                current_command <= (write && (dreg_offset == 8'h00)) ?
                                   wd : command;
                current_aux1 <= (write && (dreg_offset == 8'h04)) ?
                                (wd & 32'h000003ff) : aux1;
                current_xstate <= (write && (dreg_offset == 8'h08)) ?
                                  wd : xstate;
                current_xstate_written <=
                    (write && (dreg_offset == 8'h08)) ? 1'b1 :
                    xstate_written;
                current_rwmask <= (write && (dreg_offset == 8'h78)) ?
                                  merge_rwmask(rwmask, wd, wd_reg_mask) :
                                  rwmask;
                current_aux2 <= aux2;
                current_smask[0] <=
                    (write && (dreg_offset == 8'h7c)) ?
                    (wd & 32'h03ff03ff) : smask[0];
                current_smask[1] <=
                    (write && (dreg_offset == 8'h80)) ?
                    (wd & 32'h03ff03ff) : smask[1];
                current_smask[2] <= smask[2];
                current_smask[3] <= smask[3];
                current_smask[4] <= smask[4];
                current_smask[5] <= smask[5];
                current_smask[6] <= smask[6];
                current_smask[7] <= smask[7];
                current_rwaux1 <= (write && (dreg_offset == 8'h70)) ?
                                  wd : rwaux1;
                current_rwaux2 <= (write && (dreg_offset == 8'h74)) ?
                                  wd : rwaux2;
                current_color0 <=
                    (write && (dreg_offset == 8'h38)) ?
                        {1'b0, wd[7:0], 11'h0} :
                    (write && (dreg_offset == 8'h3c)) ? wd[19:0] : color0;
                current_color0_after_xstate <=
                    (write && (dreg_offset == 8'h08)) ? 1'b0 :
                    (write && ((dreg_offset == 8'h38) ||
                               (dreg_offset == 8'h3c))) ? 1'b1 :
                    color0_after_xstate;
                current_xstate_after_command <=
                    (write && (dreg_offset == 8'h00)) ? 1'b0 :
                    (write && (dreg_offset == 8'h08)) ? 1'b1 :
                    xstate_after_command;
                current_zero_go_5b9_xstate_source <=
                    (write && (dreg_offset == 8'h00)) ?
                        ((wd == 32'h0) && !color0_after_xstate &&
                         (xstate[25:16] == 10'h3ff)) :
                    (write && ((dreg_offset == 8'h08) ||
                               (dreg_offset == 8'h38) ||
                               (dreg_offset == 8'h3c))) ? 1'b0 :
                    zero_go_5b9_xstate_source;
                current_color1 <=
                    (write && (dreg_offset == 8'h40)) ?
                        {1'b0, wd[7:0], 11'h0} :
                    (write && (dreg_offset == 8'h44)) ? wd[19:0] : color1;
                current_color2 <=
                    (write && (dreg_offset == 8'h48)) ?
                        {1'b0, wd[7:0], 11'h0} :
                    (write && (dreg_offset == 8'h4c)) ? wd[19:0] : color2;
                current_slope0 <= (write && (dreg_offset == 8'h50)) ?
                                  slope_from_word(wd) : slope0;
                current_slope1 <= (write && (dreg_offset == 8'h54)) ?
                                  slope_from_word(wd) : slope1;
                current_slope2 <= (write && (dreg_offset == 8'h58)) ?
                                  slope_from_word(wd) : slope2;
                current_minorslope <= (write && (dreg_offset == 8'h30)) ?
                                      minorslope_from_word(wd) : minorslope;
                current_minorslope_negative <=
                    (write && (dreg_offset == 8'h30)) ? wd[31] :
                                                       minorslope_negative;
                current_colorback <= (write && (dreg_offset == 8'h5c)) ?
                                     wd[7:0] : colorback;
                current_zpattern <= (write && (dreg_offset == 8'h60)) ?
                                    wd : zpattern;
                current_lspattern <= (write && (dreg_offset == 8'h64)) ?
                                     wd : lspattern;
                current_lsmode <= (write && (dreg_offset == 8'h68)) ?
                                  wd[19:0] : lsmode;
                current_aweight <= (write && (dreg_offset == 8'h6c)) ?
                                   wd : aweight;
                current_xsave <= (write && (dreg_offset == 8'h2c)) ?
                                 wd[11:0] : xsave;
                current_xymove <= (write && (dreg_offset == 8'h34)) ?
                                  (wd & 32'h07ff07ff) : xymove;
                current_xstart_fixed <=
                    (write && (dreg_offset == 8'h0c)) ?
                        {wd[11:0], 15'h0} :
                    (write && (dreg_offset == 8'h10)) ?
                        {wd[22:0], 4'h0} :
                    (write && (dreg_offset == 8'h14)) ? wd[26:0] :
                                                        xstart_fixed;
                current_ystart_fixed <=
                    (write && (dreg_offset == 8'h1c)) ?
                        {wd[11:0], 15'h0} :
                    (write && (dreg_offset == 8'h20)) ?
                        {wd[22:0], 4'h0} :
                    (write && (dreg_offset == 8'h24)) ? wd[26:0] :
                                                        ystart_fixed;
                current_xend_fixed <=
                    (write && (dreg_offset == 8'h18)) ?
                        {wd[22:0], 4'h0} :
                    (write && (dreg_offset == 8'h84)) ?
                        {wd[11:0], 15'h0} : xend_fixed;
                current_yend_fixed <=
                    (write && (dreg_offset == 8'h28)) ?
                        {wd[22:11], 15'h0} :
                    (write && (dreg_offset == 8'h88)) ?
                        {wd[11:0], 15'h0} : yend_fixed;
                current_xstarti <=
                    (write && (dreg_offset == 8'h0c)) ? wd[11:0] :
                    (write && (dreg_offset == 8'h10)) ? wd[22:11] :
                    (write && (dreg_offset == 8'h14)) ? wd[26:15] : xstarti;
                current_ystarti <=
                    (write && (dreg_offset == 8'h1c)) ? wd[11:0] :
                    (write && (dreg_offset == 8'h20)) ? wd[22:11] :
                    (write && (dreg_offset == 8'h24)) ? wd[26:15] : ystarti;
                current_xendi <=
                    (write && (dreg_offset == 8'h18)) ? wd[22:11] :
                    (write && (dreg_offset == 8'h84)) ? wd[11:0] : xendi;
                current_yendi <=
                    (write && (dreg_offset == 8'h28)) ? wd[22:11] :
                    (write && (dreg_offset == 8'h88)) ? wd[11:0] : yendi;
                current_xcurri <=
                    (write && (dreg_offset == 8'h0c)) ? wd[11:0] :
                    (write && (dreg_offset == 8'h10)) ? wd[22:11] :
                    (write && (dreg_offset == 8'h14)) ? wd[26:15] :
                    ((dreg_offset == 8'h00) ? xstarti : xcurri);
                current_ycurri <=
                    (write && (dreg_offset == 8'h1c)) ? wd[11:0] :
                    (write && (dreg_offset == 8'h20)) ? wd[22:11] :
                    (write && (dreg_offset == 8'h24)) ? wd[26:15] :
                    ((dreg_offset == 8'h00) ? ystarti : ycurri);
                current_rwaux_stream_done <= rwaux_stream_done;
                current_xstart_explicit_since_primitive <=
                    rwaux_go_write ? xstart_explicit_since_primitive : 1'b0;
            end

            // REX_LSPATSAVE copies the retired working stipple state into the
            // next/programmed bank.  Do not overwrite current_lspattern here:
            // lightPcxSwap relies on TOGGLECTXT preserving the other context.
            // Acceptance is held until CHIPBUSY is clear above, so this
            // feedback cannot overtake queued pixels.
            if (accept && lspatsave_go_access && ls_work_valid) begin
                lspattern <= ls_work_pattern;
                lsmode <= ls_work_mode;
            end

            if (accept && write) begin
                if (config_region) begin
                    case (cfg_offset)
                        11'h790: smask[2] <= wd & 32'h03ff03ff;
                        11'h794: smask[3] <= wd & 32'h03ff03ff;
                        11'h798: smask[4] <= wd & 32'h03ff03ff;
                        11'h79c: smask[5] <= wd & 32'h03ff03ff;
                        11'h7a0: smask[6] <= wd & 32'h03ff03ff;
                        11'h7a4: smask[7] <= wd & 32'h03ff03ff;
                        11'h7a8: aux2 <= wd & 32'h7fffffff;
                        11'h7ec: configsel <= wd[2:0];
                        11'h7f4: togglectxt <= wd;
                        11'h7f8: configmode <= wd;
                        11'h7fc: xywin <= wd & 32'h0fff0fff;
                        default: ;
                    endcase
                end else begin
                    if (solid_go_write) begin
                        enqueue_solid_context(
                            solid_trigger_command,
                            solid_trigger_lrqpoly_point ?
                                xcurri : solid_trigger_xstart,
                            solid_trigger_lrqpoly_point ?
                                ycurri : solid_trigger_ystart,
                            solid_trigger_xend,
                            solid_trigger_yend,
                            solid_trigger_color0,
                            solid_trigger_color0_after_xstate,
                            solid_trigger_color1,
                            solid_trigger_color2,
                            solid_trigger_slope0,
                            solid_trigger_slope1,
                            solid_trigger_slope2
                        );
                        if (solid_trigger_lrqpoly_point) begin
                            ctx_retire_x <= lrqpoly_trigger_next_x;
                            ctx_retire_y <= ycurri;
                            ctx_retire_done <= 1'b0;
                        end
                    end
                    if (line_go_write)
                        enqueue_line_context(
                            solid_trigger_command,
                            solid_trigger_xstart,
                            solid_trigger_ystart,
                            solid_trigger_xend,
                            solid_trigger_yend,
                            solid_trigger_color0,
                            solid_trigger_color1,
                            solid_trigger_color2,
                            solid_trigger_slope0,
                            solid_trigger_slope1,
                            solid_trigger_slope2,
                            line_trigger_minorslope,
                            line_trigger_minor_negative,
                            line_trigger_fraction
                        );
                    case (dreg_offset)
                        8'h00: begin
                            command <= wd;
                            xstate_after_command <= 1'b0;
                            if (go)
                                zero_go_5b9_xstate_source <=
                                    (wd == 32'h0) &&
                                    !color0_after_xstate &&
                                    (xstate[25:16] == 10'h3ff);
                            else if ((wd & 32'h000fffff) != 32'h000005b9)
                                zero_go_5b9_xstate_source <= 1'b0;
                            rwaux_stream_done <= 1'b0;
                            if (go) begin
                                xcurri <= xstarti;
                                ycurri <= ystarti;
                            end
                        end
                        8'h04: aux1 <= wd & 32'h000003ff;
                        8'h08: begin
                            xstate <= wd;
                            xstate_written <= 1'b1;
                            xstate_mask_active <= 1'b1;
                            color0_after_xstate <= 1'b0;
                            xstate_after_command <= 1'b1;
                            zero_go_5b9_xstate_source <= 1'b0;
                            rwaux_stream_done <= 1'b0;
                        end
                        8'h0c: begin
                            xstart_fixed <= {wd[11:0], 15'h0};
                            xstarti <= wd[11:0];
                            xcurri <= wd[11:0];
                            rwaux_stream_done <= 1'b0;
                            xstart_explicit_since_primitive <= 1'b1;
                        end
                        8'h10: begin
                            xstart_fixed <= {wd[22:0], 4'h0};
                            xstarti <= wd[22:11];
                            xcurri <= wd[22:11];
                            rwaux_stream_done <= 1'b0;
                            xstart_explicit_since_primitive <= 1'b1;
                        end
                        8'h14: begin
                            xstart_fixed <= wd[26:0];
                            xstarti <= wd[26:15];
                            xcurri <= wd[26:15];
                            rwaux_stream_done <= 1'b0;
                            xstart_explicit_since_primitive <= 1'b1;
                        end
                        8'h18: begin
                            xend_fixed <= {wd[22:0], 4'h0};
                            xendi <= wd[22:11];
                            rwaux_stream_done <= 1'b0;
                        end
                        8'h1c: begin
                            ystart_fixed <= {wd[11:0], 15'h0};
                            ystarti <= wd[11:0];
                            ycurri <= wd[11:0];
                            rwaux_stream_done <= 1'b0;
                        end
                        8'h20: begin
                            ystart_fixed <= {wd[22:0], 4'h0};
                            ystarti <= wd[22:11];
                            ycurri <= wd[22:11];
                            rwaux_stream_done <= 1'b0;
                        end
                        8'h24: begin
                            ystart_fixed <= wd[26:0];
                            ystarti <= wd[26:15];
                            ycurri <= wd[26:15];
                            rwaux_stream_done <= 1'b0;
                        end
                        8'h28: begin
                            yend_fixed <= {wd[22:11], 15'h0};
                            yendi <= wd[22:11];
                            rwaux_stream_done <= 1'b0;
                        end
                        8'h2c: xsave <= wd[11:0];
                        8'h30: begin
                            minorslope <= minorslope_from_word(wd);
                            minorslope_negative <= wd[31];
                        end
                        8'h38: begin
                            color0 <= {1'b0, wd[7:0], 11'h0};
                            color0_after_xstate <= 1'b1;
                            zero_go_5b9_xstate_source <= 1'b0;
                        end
                        8'h3c: begin
                            color0 <= wd[19:0];
                            color0_after_xstate <= 1'b1;
                            zero_go_5b9_xstate_source <= 1'b0;
                        end
                        8'h40: color1 <= {1'b0, wd[7:0], 11'h0};
                        8'h44: color1 <= wd[19:0];
                        8'h48: color2 <= {1'b0, wd[7:0], 11'h0};
                        8'h4c: color2 <= wd[19:0];
                        8'h50: slope0 <= slope_from_word(wd);
                        8'h54: slope1 <= slope_from_word(wd);
                        8'h58: slope2 <= slope_from_word(wd);
                        8'h34: begin
                            xymove <= wd & 32'h07ff07ff;
                        end
                        8'h5c: colorback <= wd[7:0];
                        8'h60: begin
                            zpattern <= wd;
                            if (go)
                                zero_go_5b9_xstate_source <= 1'b0;
                        end
                        8'h64: begin
                            lspattern <= wd;
                        end
                        8'h68: lsmode <= wd[19:0];
                        8'h6c: aweight <= wd;
                        8'h70: begin
                            rwaux1 <= wd;
                            if (go && command_draw(command) &&
                                command_effective_coloraux(
                                    command, xstate, 1'b1) &&
                                !rwaux_stream_done && (wd_lane_mask != 4'h0)) begin
                                ctx_valid <= 1'b1;
                                ctx_rwaux_write <= 1'b1;
                                ctx_solid_fill <= 1'b0;
                                ctx_pattern_fill <= 1'b0;
                                ctx_pattern_tail <= 1'b0;
                                ctx_copy_fill <= 1'b0;
                                ctx_line_draw <= 1'b0;
                                ctx_shade <= 1'b0;
                                ctx_command <= command;
                                ctx_data <= wd;
                                ctx_lsmode <= lsmode;
                                ctx_ls_reload <= 1'b0;
                                ctx_lane_mask <= wd_lane_mask;
                                ctx_x <= visible_x_coord(ctx_base_x_now);
                                ctx_y <= visible_y_coord(ctx_base_y_now);
                                ctx_xstart <= visible_x_coord(xstarti);
                                ctx_xend <= visible_x_coord(xendi);
                                ctx_yend <= visible_y_coord(yendi);
                                ctx_x_down <= xstarti > xendi;
                                ctx_y_down <= ystarti > yendi;
                                ctx_block <= ctx_block_now;
                                ctx_clip_xend <=
                                    xstart_explicit_since_primitive &&
                                    (command[8:0] == 9'h0a1);
                                ctx_quadmode <= ctx_quadmode_now;
                                ctx_plane <= plane_from_aux2(aux2);
                                ctx_color <= 8'h0;
                                ctx_color_fixed <= 20'h0;
                                ctx_color1_fixed <= 20'h0;
                                ctx_color2_fixed <= 20'h0;
                                ctx_slope <= 21'h0;
                                ctx_slope1 <= 21'h0;
                                ctx_slope2 <= 21'h0;
                                ctx_minor_slope <= 17'h0;
                                ctx_minor_negative <= 1'b0;
                                ctx_line_frac <= 15'h0;
                                ctx_pattern_opaque <= 1'b0;
                                ctx_bg_color <= background_color(xstate,
                                                                 colorback);
                                ctx_copy_dx <= 12'h0;
                                ctx_copy_dy <= 12'h0;
                                ctx_aux2 <= aux2;
                                ctx_smask <= smask_pack(1'b0);
                                ctx_logic_op <=
                                    effective_logic_op(command, xstate);
                                ctx_pixel_writemask <=
                                    effective_pixel_writemask(xstate, rwmask,
                                                              aux1, aux2);
                                ctx_colorcomp <=
                                    (command & CMD_COLORCOMP) != 32'h0;
                                ctx_aux1 <= aux1[9:0];
                                ctx_retire_x <= rwaux_next[11:0];
                                ctx_retire_y <= rwaux_next[23:12];
                                ctx_retire_done <= rwaux_next[24];
                                xcurri <= rwaux_next[11:0];
                                ycurri <= rwaux_next[23:12];
                                rwaux_stream_done <= rwaux_next[24];
                            end
                        end
                        8'h74: rwaux2 <= wd;
                        8'h78: begin
                            rwmask <= merge_rwmask(rwmask, wd, wd_reg_mask);
                            // XSTATE and RWMASK are alternate carriers only
                            // for the pixel mask.  Keep independent XSTATE
                            // logic-op provenance intact; a readmask-only
                            // upper-byte write does not change ownership.
                            if (wd_reg_mask[0])
                                xstate_mask_active <= 1'b0;
                        end
                        8'h7c: smask[0] <= wd & 32'h03ff03ff;
                        8'h80: smask[1] <= wd & 32'h03ff03ff;
                        8'h84: begin
                            xend_fixed <= {wd[11:0], 15'h0};
                            xendi <= wd[11:0];
                            rwaux_stream_done <= 1'b0;
                        end
                        8'h88: begin
                            yend_fixed <= {wd[11:0], 15'h0};
                            yendi <= wd[11:0];
                            rwaux_stream_done <= 1'b0;
                        end
                        default: ;
                    endcase

                    // GO-qualified pattern and endpoint writes all launch
                    // the active pattern command.  Endpoint aliases must use
                    // the value from this transaction rather than the prior
                    // register bank; the same context path keeps the launch
                    // and projected issue cursor coherent under backpressure.
                    if (pattern_go_write) begin
                        enqueue_pattern_context(
                            pattern_trigger_command,
                            pattern_trigger_word,
                            pattern_trigger_base_x,
                            pattern_trigger_base_y,
                            pattern_trigger_lrqpoly_point ?
                                pattern_trigger_base_x : solid_trigger_xstart,
                            pattern_trigger_lrqpoly_point ?
                                pattern_trigger_base_x : solid_trigger_xend,
                            solid_trigger_yend,
                            pattern_trigger_x_down,
                            pattern_trigger_y_down,
                            pattern_trigger_block,
                            plane_from_aux2(aux2),
                            pattern_foreground_color(pattern_trigger_command,
                                                     xstate,
                                                     aux2,
                                                     solid_trigger_color0,
                                                     solid_trigger_color0_after_xstate,
                                                     solid_trigger_xstate_after_command,
                                                     zero_go_5b9_xstate_source),
                            pattern_opaque(pattern_trigger_command, xstate),
                            pattern_background_color(pattern_trigger_command,
                                                     xstate,
                                                     colorback),
                            aux2,
                            smask_pack(1'b0),
                            effective_logic_op(pattern_trigger_command,
                                               xstate),
                            effective_pixel_writemask(
                                xstate, rwmask, aux1, aux2),
                            (pattern_trigger_command & CMD_COLORCOMP) != 32'h0,
                            aux1[9:0],
                            1'b0
                        );
                        // The task's default fixed-color state comes from the
                        // stored bank.  GO color/slope aliases use the value
                        // accepted in this transaction instead.
                        ctx_color_fixed <= solid_trigger_color0;
                        ctx_color1_fixed <= solid_trigger_color1;
                        ctx_color2_fixed <= solid_trigger_color2;
                        ctx_slope <= solid_trigger_slope0;
                        ctx_slope1 <= solid_trigger_slope1;
                        ctx_slope2 <= solid_trigger_slope2;
                        ctx_lsmode <= (dreg_offset == 8'h68) ?
                                      wd[19:0] : lsmode;
                        ctx_ls_reload <=
                            ((pattern_trigger_command & CMD_ENLSPAT) != 32'h0) &&
                            (ls_programmed_since_launch ||
                             (dreg_offset == 8'h64) ||
                             (dreg_offset == 8'h68));
                        ctx_retire_x <= pattern_trigger_next[11:0];
                        ctx_retire_y <= pattern_trigger_next[23:12];
                        ctx_retire_done <= pattern_trigger_next[24];
                        xcurri <= pattern_trigger_next[11:0];
                        ycurri <= pattern_trigger_next[23:12];
                        rwaux_stream_done <= pattern_trigger_next[24];
                    end
                    if (copy_go_write)
                        enqueue_copy_context(copy_trigger_command,
                                             solid_trigger_xstart,
                                             solid_trigger_ystart,
                                             solid_trigger_xend,
                                             solid_trigger_yend,
                                             copy_trigger_xymove);

                    // Keep the issue cursor ahead of queued work while the
                    // retirement-visible current bank remains gated by the
                    // final RB memory handshake.  Pattern and RWAUX launches
                    // compute their own word/row advance above; solid, line,
                    // and copy contexts retire at their programmed endpoint.
                    // Without this projected issue state, an XYCONTINUE
                    // command queued after one of these primitives restarts
                    // from the stale XSTART/YSTART coordinate.
                    if (solid_go_write) begin
                        if (solid_trigger_lrqpoly_point) begin
                            xcurri <= lrqpoly_trigger_next_x;
                        end else begin
                            xcurri <= ((solid_trigger_command &
                                       (CMD_BLOCK | CMD_STOPONX)) != 32'h0) ?
                                      solid_trigger_xend : solid_trigger_xstart;
                            ycurri <= ((solid_trigger_command &
                                       (CMD_BLOCK | CMD_STOPONY)) != 32'h0) ?
                                      solid_trigger_yend : solid_trigger_ystart;
                        end
                    end else if (line_go_write || copy_go_write) begin
                        xcurri <= solid_trigger_xend;
                        ycurri <= solid_trigger_yend;
                    end
                end
            end else if (accept && command_go_read) begin
                if (solid_go_read)
                    enqueue_solid_context(command, xstarti, ystarti,
                                          xendi, yendi, color0,
                                          color0_after_xstate, color1, color2,
                                          slope0, slope1, slope2);
                else if (line_go_read)
                    enqueue_line_context(command, xstarti, ystarti,
                                         xendi, yendi, color0, color1, color2,
                                         slope0, slope1, slope2,
                                         minorslope, minorslope_negative,
                                         ((command & CMD_INITFRAC) != 32'h0) ?
                                             15'h4000 :
                                             (minorslope_negative ?
                                              ((~(((command & CMD_XMAJOR) !=
                                                   32'h0) ?
                                                  ystart_fixed[14:0] :
                                                  xstart_fixed[14:0])) +
                                               15'd1) :
                                              (((command & CMD_XMAJOR) !=
                                                32'h0) ?
                                               ystart_fixed[14:0] :
                                               xstart_fixed[14:0])));
                else if (pattern_go_read) begin
                    enqueue_pattern_context(
                        command,
                        ((command & CMD_ENZPAT) != 32'h0) ?
                            zpattern : lspattern,
                        ctx_xycontinue_now ? xcurri : xstarti,
                        ctx_xycontinue_now ? ycurri : ystarti,
                        xstarti,
                        xendi,
                        yendi,
                        x_down_now,
                        y_down_now,
                        ctx_block_now,
                        plane_from_aux2(aux2),
                        pattern_foreground_color(command, xstate, aux2,
                                                 color0,
                                                 color0_after_xstate,
                                                 xstate_after_command,
                                                 zero_go_5b9_xstate_source),
                        pattern_opaque(command, xstate),
                        pattern_background_color(command, xstate, colorback),
                        aux2,
                        smask_pack(1'b0),
                        effective_logic_op(command, xstate),
                        effective_pixel_writemask(
                            xstate, rwmask, aux1, aux2),
                        (command & CMD_COLORCOMP) != 32'h0,
                        aux1[9:0],
                        1'b0
                    );
                    ctx_retire_x <= pattern_next[11:0];
                    ctx_retire_y <= pattern_next[23:12];
                    ctx_retire_done <= pattern_next[24];
                    xcurri <= pattern_next[11:0];
                    ycurri <= pattern_next[23:12];
                    rwaux_stream_done <= pattern_next[24];
                end else if (copy_go_read)
                    enqueue_copy_context(command, xstarti, ystarti,
                                         xendi, yendi, xymove);

                // A GO.COMMAND access primes the current coordinate even for
                // NOP and unsupported commands, matching the compatibility
                // model and the PROM's setup sequence.
                if (solid_go_read) begin
                    xcurri <= ((command &
                               (CMD_BLOCK | CMD_STOPONX)) != 32'h0) ?
                              xendi : xstarti;
                    ycurri <= ((command &
                               (CMD_BLOCK | CMD_STOPONY)) != 32'h0) ?
                              yendi : ystarti;
                end else if (line_go_read || copy_go_read) begin
                    xcurri <= xendi;
                    ycurri <= yendi;
                end else if (!pattern_go_read) begin
                    xcurri <= xstarti;
                    ycurri <= ystarti;
                end
            end

            if (accept && write && !config_region &&
                ((dreg_offset == 8'h64) || (dreg_offset == 8'h68)))
                ls_programmed_since_launch <= 1'b1;

            // TOGGLECTXT restores a different context's saved next state.
            // Its first continued primitive must load that state rather than
            // inherit the working phase of the context that was switched out.
            if (accept && write && config_region && (cfg_offset == 11'h7f4))
                ls_programmed_since_launch <= 1'b1;

            if (accept &&
                (((line_go_write || line_go_read) &&
                  ((write ? solid_trigger_command : command) &
                   CMD_ENLSPAT) != 32'h0) ||
                 ((pattern_go_write || pattern_go_read) &&
                  ((write ? pattern_trigger_command : command) &
                   CMD_ENLSPAT) != 32'h0)))
                ls_programmed_since_launch <= 1'b0;

            // A non-RWAUX primitive consumes the explicit coordinate setup.
            // RWAUX data words deliberately leave the marker set so every
            // padded word in the same rexRevADrawImage scanline sees XEND.
            if (accept &&
                (solid_go_write || line_go_write || pattern_go_write ||
                 copy_go_write || solid_go_read || line_go_read ||
                 pattern_go_read || copy_go_read))
                xstart_explicit_since_primitive <= 1'b0;

            if (load_pending && load_rsp_valid) begin
                rwaux1 <= load_rsp_data;
                current_rwaux1 <= load_rsp_data;
                if (!load_cursor_superseded &&
                    !load_cursor_superseding_access) begin
                    xcurri <= load_next_latched[11:0];
                    ycurri <= load_next_latched[23:12];
                    rwaux_stream_done <= load_next_latched[24];
                    current_xcurri <= load_next_latched[11:0];
                    current_ycurri <= load_next_latched[23:12];
                    current_rwaux_stream_done <= load_next_latched[24];
                end
            end

        end
    end
endmodule

`default_nettype wire
