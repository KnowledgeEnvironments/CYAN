// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - coherent two-entry LG1 64-byte line cache
//
// Bridges the ordered 32-bit RB stream and coherent read-only scan probes to a
// 512-bit logical memory channel.  Every RB miss fills a complete line before
// a pending write is merged, so later reads always observe backing memory plus
// all accepted dirty bytes.  Scan misses pass through without allocation.
// Each physical side permits one transaction at a time, independently; this
// is a bounded dual-port-VRAM abstraction, not a multi-outstanding scheduler.

`default_nettype none

module lg1_line_cache_64b (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         word_valid,
    output wire         word_ready,
    input  wire         word_write,
    input  wire [  1:0] word_plane,
    input  wire [ 17:0] word_addr,
    input  wire [ 31:0] word_wdata,
    input  wire [  3:0] word_wstrb,
    output reg          word_rsp_valid,
    input  wire         word_rsp_ready,
    output reg  [ 31:0] word_rsp_rdata,

    // Read-only coherent scan probe.  A resident line is returned directly,
    // including dirty bytes, without changing replacement state.  A miss is
    // forwarded to the native line channel without allocating a cache way.
    input  wire         scan_valid,
    output wire         scan_ready,
    input  wire [ 15:0] scan_addr,
    output reg          scan_rsp_valid,
    input  wire         scan_rsp_ready,
    output reg  [511:0] scan_rsp_rdata,

    input  wire         flush,
    output reg          flush_done,

    output wire         line_valid,
    input  wire         line_ready,
    input  wire         line_write_done,
    output wire         line_write,
    output wire [ 15:0] line_addr,
    output wire [511:0] line_wdata,
    output wire [ 63:0] line_wstrb,
    input  wire         line_rsp_valid,
    output wire         line_rsp_ready,
    input  wire [511:0] line_rsp_rdata,

    // Independent serial-port backing channel.  This mirrors the physical
    // VRAM split: RB/cache traffic and scan probes own separate request
    // mailboxes and may both be outstanding below this coherence boundary.
    output wire         scan_line_valid,
    input  wire         scan_line_ready,
    output wire [ 15:0] scan_line_addr,
    input  wire         scan_line_rsp_valid,
    output wire         scan_line_rsp_ready,
    input  wire [511:0] scan_line_rsp_rdata,

    output wire         dirty,
    output wire         busy
);
    localparam [2:0] ST_IDLE      = 3'd0;
    localparam [2:0] ST_WRITEBACK = 3'd1;
    localparam [2:0] ST_FILL_REQ  = 3'd2;
    localparam [2:0] ST_FILL_WAIT = 3'd3;
    localparam [2:0] ST_WRITEBACK_WAIT = 3'd4;
    reg [2:0] state;

    reg          way0_valid;
    reg          way1_valid;
    reg [ 15:0] way0_addr;
    reg [ 15:0] way1_addr;
    reg [511:0] way0_data;
    reg [511:0] way1_data;
    reg [ 63:0] way0_dirty;
    reg [ 63:0] way1_dirty;
    reg          lru_way;

    reg          pending_write;
    reg [ 15:0] pending_line_addr;
    reg [  3:0] pending_word_index;
    reg [ 31:0] pending_wdata;
    reg [  3:0] pending_wstrb;
    reg          pending_way;
    reg          miss_active;
    reg          scan_miss_busy;
    reg [ 15:0] scan_pending_addr;

    reg          flush_pending;
    reg          flush_seen;

    wire [15:0] request_line_addr = {word_plane, word_addr[17:4]};
    wire [ 3:0] request_word_index = word_addr[3:0];
    wire hit0 = way0_valid && (way0_addr == request_line_addr);
    wire hit1 = way1_valid && (way1_addr == request_line_addr);
    wire scan_hit0 = way0_valid && (way0_addr == scan_addr);
    wire scan_hit1 = way1_valid && (way1_addr == scan_addr);
    wire selected_victim =
        !way0_valid ? 1'b0 :
        !way1_valid ? 1'b1 :
                      lru_way;
    wire victim_dirty =
        selected_victim ? (way1_dirty != 64'h0) :
                          (way0_dirty != 64'h0);

    wire flush_way0 = way0_valid && (way0_dirty != 64'h0);
    wire flush_way1 = way1_valid && (way1_dirty != 64'h0);
    wire flush_victim = flush_way0 ? 1'b0 : 1'b1;

    wire idle_available =
        (state == ST_IDLE) && !flush && !flush_pending &&
        (!word_rsp_valid || word_rsp_ready) &&
        // Do not retire a scan response and accept an RB mutation on the
        // same edge: the scanout gearbox begins from that response epoch.
        !scan_rsp_valid;
    // Real LG1 VRAM scanout has an independent serial port.  At this shared
    // FPGA backing boundary, accept each pending serial segment immediately;
    // the fetcher's multi-cycle local deposit is the RB service window.
    // Delaying every one of 32 segments, even by a small fixed allowance,
    // compounds beyond the programmed 1320-clock raster-line deadline.
    wire scan_hit = scan_hit0 || scan_hit1;
    wire scan_rb_conflict = miss_active &&
                            (pending_line_addr == scan_addr);
    wire scan_port_available = !scan_rsp_valid && !scan_miss_busy &&
                               !scan_rb_conflict;
    assign word_ready = idle_available && !scan_valid &&
                        !(scan_miss_busy &&
                          (request_line_addr == scan_pending_addr));
    assign scan_ready = scan_port_available &&
                        (scan_hit || scan_line_ready);
    assign scan_line_valid = scan_valid && scan_port_available && !scan_hit;
    assign scan_line_addr = scan_addr;
    assign scan_line_rsp_ready = scan_miss_busy && !scan_rsp_valid;

    assign line_valid =
        (state == ST_WRITEBACK) || (state == ST_FILL_REQ);
    assign line_write = (state == ST_WRITEBACK);
    assign line_addr =
        (state == ST_WRITEBACK) ?
            (pending_way ? way1_addr : way0_addr) :
            pending_line_addr;
    assign line_wdata =
        (state == ST_WRITEBACK) ?
            (pending_way ? way1_data : way0_data) :
            512'h0;
    assign line_wstrb =
        (state == ST_WRITEBACK) ?
            (pending_way ? way1_dirty : way0_dirty) :
            64'h0;
    assign line_rsp_ready = (state == ST_FILL_WAIT);

    // A resident dirty line is coherent and does not mean the graphics engine
    // is still executing.  Consumers that require backing-memory visibility
    // must issue flush and wait for flush_done.  Keeping these indications
    // separate prevents a normal write-back cache residency from holding the
    // host-visible engine-busy status forever.
    assign dirty = (way0_dirty != 64'h0) || (way1_dirty != 64'h0);
    assign busy = (state != ST_IDLE) || word_rsp_valid || flush_pending;

    function [511:0] merge_word;
        input [511:0] old_data;
        input [  3:0] word_index;
        input [ 31:0] new_data;
        input [  3:0] byte_strobe;
        integer i;
        reg [511:0] value;
        begin
            value = old_data;
            for (i = 0; i < 4; i = i + 1)
                if (byte_strobe[i])
                    value[511 - (word_index * 32) - (i * 8) -: 8] =
                        new_data[31 - (i * 8) -: 8];
            merge_word = value;
        end
    endfunction

    function [63:0] merge_dirty;
        input [63:0] old_dirty;
        input [ 3:0] word_index;
        input [ 3:0] byte_strobe;
        integer i;
        reg [63:0] value;
        begin
            value = old_dirty;
            for (i = 0; i < 4; i = i + 1)
                if (byte_strobe[i])
                    value[63 - (word_index * 4) - i] = 1'b1;
            merge_dirty = value;
        end
    endfunction

    function [31:0] select_word;
        input [511:0] data;
        input [  3:0] word_index;
        begin
            select_word = data[511 - (word_index * 32) -: 32];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            way0_valid <= 1'b0;
            way1_valid <= 1'b0;
            way0_addr <= 16'h0;
            way1_addr <= 16'h0;
            way0_data <= 512'h0;
            way1_data <= 512'h0;
            way0_dirty <= 64'h0;
            way1_dirty <= 64'h0;
            lru_way <= 1'b0;
            pending_write <= 1'b0;
            pending_line_addr <= 16'h0;
            pending_word_index <= 4'h0;
            pending_wdata <= 32'h0;
            pending_wstrb <= 4'h0;
            pending_way <= 1'b0;
            miss_active <= 1'b0;
            scan_miss_busy <= 1'b0;
            scan_pending_addr <= 16'h0;
            flush_pending <= 1'b0;
            flush_seen <= 1'b0;
            flush_done <= 1'b0;
            word_rsp_valid <= 1'b0;
            word_rsp_rdata <= 32'h0;
            scan_rsp_valid <= 1'b0;
            scan_rsp_rdata <= 512'h0;
        end else begin
            flush_done <= 1'b0;

            if (!flush)
                flush_seen <= 1'b0;
            else if (!flush_seen) begin
                flush_seen <= 1'b1;
                flush_pending <= 1'b1;
            end

            if (word_rsp_valid && word_rsp_ready)
                word_rsp_valid <= 1'b0;
            if (scan_rsp_valid && scan_rsp_ready)
                scan_rsp_valid <= 1'b0;

            if (scan_valid && scan_ready) begin
                if (scan_hit) begin
                    scan_rsp_rdata <= scan_hit1 ? way1_data : way0_data;
                    scan_rsp_valid <= 1'b1;
                end else begin
                    scan_pending_addr <= scan_addr;
                    scan_miss_busy <= 1'b1;
                end
            end
            if (scan_line_rsp_valid && scan_line_rsp_ready) begin
                scan_rsp_rdata <= scan_line_rsp_rdata;
                scan_rsp_valid <= 1'b1;
                scan_miss_busy <= 1'b0;
            end

            case (state)
                ST_IDLE: begin
                    if (flush_pending || (flush && !flush_seen)) begin
                        if (flush_way0 || flush_way1) begin
                            pending_way <= flush_victim;
                            miss_active <= 1'b0;
                            state <= ST_WRITEBACK;
                        end else begin
                            flush_pending <= 1'b0;
                            flush_done <= 1'b1;
                        end
                    end else if (word_valid && word_ready) begin
                        if (hit0 || hit1) begin
                            if (hit1)
                                lru_way <= 1'b0;
                            else
                                lru_way <= 1'b1;

                            if (word_write) begin
                                if (hit1) begin
                                    way1_data <= merge_word(
                                        way1_data, request_word_index,
                                        word_wdata, word_wstrb);
                                    way1_dirty <= merge_dirty(
                                        way1_dirty, request_word_index,
                                        word_wstrb);
                                end else begin
                                    way0_data <= merge_word(
                                        way0_data, request_word_index,
                                        word_wdata, word_wstrb);
                                    way0_dirty <= merge_dirty(
                                        way0_dirty, request_word_index,
                                        word_wstrb);
                                end
                            end else begin
                                word_rsp_rdata <= select_word(
                                    hit1 ? way1_data : way0_data,
                                    request_word_index);
                                word_rsp_valid <= 1'b1;
                            end
                        end else begin
                            pending_write <= word_write;
                            pending_line_addr <= request_line_addr;
                            pending_word_index <= request_word_index;
                            pending_wdata <= word_wdata;
                            pending_wstrb <= word_wstrb;
                            pending_way <= selected_victim;
                            miss_active <= 1'b1;
                            if (victim_dirty)
                                state <= ST_WRITEBACK;
                            else
                                state <= ST_FILL_REQ;
                        end
                    end
                end

                ST_WRITEBACK: begin
                    // line_ready accepts the request into the downstream
                    // mailbox.  Dirty ownership remains here until the
                    // distinct native-write completion returns.
                    if (line_ready)
                        state <= ST_WRITEBACK_WAIT;
                end

                ST_WRITEBACK_WAIT: begin
                    if (line_write_done) begin
                        if (pending_way)
                            way1_dirty <= 64'h0;
                        else
                            way0_dirty <= 64'h0;
                        if (miss_active) begin
                            state <= ST_FILL_REQ;
                        end else if (flush_pending &&
                                     (pending_way ? flush_way0 :
                                                    flush_way1)) begin
                            // Preserve the old no-bubble handoff between two
                            // flush victims without releasing either dirty
                            // line before its own completion.
                            pending_way <= ~pending_way;
                            state <= ST_WRITEBACK;
                        end else begin
                            state <= ST_IDLE;
                        end
                    end
                end

                ST_FILL_REQ: begin
                    if (line_ready)
                        state <= ST_FILL_WAIT;
                end

                ST_FILL_WAIT: begin
                    if (line_rsp_valid && line_rsp_ready) begin
                        if (pending_way) begin
                            way1_valid <= 1'b1;
                            way1_addr <= pending_line_addr;
                            way1_data <= pending_write ?
                                merge_word(line_rsp_rdata, pending_word_index,
                                           pending_wdata, pending_wstrb) :
                                line_rsp_rdata;
                            way1_dirty <= pending_write ?
                                merge_dirty(64'h0, pending_word_index,
                                            pending_wstrb) :
                                64'h0;
                            lru_way <= 1'b0;
                        end else begin
                            way0_valid <= 1'b1;
                            way0_addr <= pending_line_addr;
                            way0_data <= pending_write ?
                                merge_word(line_rsp_rdata, pending_word_index,
                                           pending_wdata, pending_wstrb) :
                                line_rsp_rdata;
                            way0_dirty <= pending_write ?
                                merge_dirty(64'h0, pending_word_index,
                                            pending_wstrb) :
                                64'h0;
                            lru_way <= 1'b1;
                        end

                        if (!pending_write) begin
                            word_rsp_rdata <= select_word(
                                line_rsp_rdata, pending_word_index);
                            word_rsp_valid <= 1'b1;
                        end
                        miss_active <= 1'b0;
                        state <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
