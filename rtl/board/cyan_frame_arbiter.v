// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - completion-aware guest/management Ethernet TX arbitration.
//
// The HPC1 guest producer may insert an arbitrarily long legal gap while it
// waits for DMA. Buffer the guest frame through accepted EOF before making it
// eligible for the shared MAC. Management traffic therefore remains usable
// while a guest frame is incomplete, and a selected complete guest frame can
// be replayed atomically without timeout, truncation, or frame splicing.
// Management has deliberate priority before guest replay begins; it may
// overtake or starve a completed guest frame, but it can never splice one.
// The connected HPC1 producer supplies a legal SOF-first stream. A malformed
// non-SOF byte at an empty buffer is outside this module's interface contract.

`default_nettype none

module cyan_frame_arbiter #(
    parameter integer MAX_GUEST_FRAME_BYTES = 2048
) (
    input  wire       clk,
    input  wire       rst_n,

    input  wire [7:0] guest_data,
    input  wire       guest_valid,
    input  wire       guest_sof,
    input  wire       guest_eof,
    output wire       guest_ready,

    input  wire [7:0] mgmt_data,
    input  wire       mgmt_valid,
    input  wire       mgmt_sof,
    input  wire       mgmt_eof,
    output wire       mgmt_ready,

    output wire [7:0] out_data,
    output wire       out_valid,
    output wire       out_sof,
    output wire       out_eof,
    input  wire       out_ready
);
    localparam integer GUEST_PTR_W =
        (MAX_GUEST_FRAME_BYTES <= 2) ? 1 : $clog2(MAX_GUEST_FRAME_BYTES);
    localparam integer GUEST_LEN_W =
        (MAX_GUEST_FRAME_BYTES <= 1) ? 1 :
        $clog2(MAX_GUEST_FRAME_BYTES + 1);

    localparam [GUEST_PTR_W-1:0] GUEST_LAST_INDEX =
        GUEST_PTR_W'(MAX_GUEST_FRAME_BYTES - 1);

    localparam [1:0]
        STATE_IDLE           = 2'd0,
        STATE_GUEST_PREFETCH = 2'd1,
        STATE_GUEST_SEND     = 2'd2,
        STATE_MGMT_SEND      = 2'd3;

    reg [1:0] state;
    reg [GUEST_PTR_W-1:0] guest_write_index;
    reg [GUEST_PTR_W-1:0] guest_replay_index;
    reg [GUEST_LEN_W-1:0] guest_frame_length;
    reg                   guest_frame_complete;
    reg                   guest_drop_overlength;

    wire guest_capture_accept = guest_valid && guest_ready;
    wire guest_capture_restart = guest_capture_accept && guest_sof &&
        (guest_drop_overlength ||
         guest_write_index != {GUEST_PTR_W{1'b0}});
    wire guest_capture_write = guest_capture_accept &&
        (!guest_drop_overlength || guest_sof);
    wire [GUEST_PTR_W-1:0] guest_capture_waddr = guest_sof ?
        {GUEST_PTR_W{1'b0}} : guest_write_index;

    wire [GUEST_LEN_W-1:0] guest_replay_index_ext =
        {{(GUEST_LEN_W-GUEST_PTR_W){1'b0}}, guest_replay_index};
    wire guest_replay_last = (guest_replay_index_ext ==
                              guest_frame_length - 1'b1);
    wire guest_replay_accept = (state == STATE_GUEST_SEND) && out_ready;
    wire [GUEST_PTR_W-1:0] guest_replay_next =
        guest_replay_index + 1'b1;
    wire [GUEST_PTR_W-1:0] guest_ram_raddr =
        (guest_replay_accept && !guest_replay_last) ?
        guest_replay_next : guest_replay_index;
    wire [7:0] guest_ram_rdata;

    cyan_sync_byte_ram #(.ADDR_W(GUEST_PTR_W)) u_guest_frame_ram (
        .clk  (clk),
        .we   (guest_capture_write),
        .waddr(guest_capture_waddr),
        .wdata(guest_data),
        .raddr(guest_ram_raddr),
        .rdata(guest_ram_rdata)
    );

    wire choose_mgmt = (state == STATE_MGMT_SEND) ||
                       ((state == STATE_IDLE) && mgmt_valid);
    wire choose_guest = (state == STATE_GUEST_SEND);

    assign guest_ready = !guest_frame_complete;
    assign mgmt_ready = choose_mgmt && out_ready;

    assign out_data = choose_mgmt ? mgmt_data : guest_ram_rdata;
    assign out_valid = choose_mgmt ? mgmt_valid : choose_guest;
    assign out_sof = choose_mgmt ? mgmt_sof :
                     (choose_guest &&
                      guest_replay_index == {GUEST_PTR_W{1'b0}});
    assign out_eof = choose_mgmt ? mgmt_eof :
                     (choose_guest && guest_replay_last);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                 <= STATE_IDLE;
            guest_write_index     <= {GUEST_PTR_W{1'b0}};
            guest_replay_index    <= {GUEST_PTR_W{1'b0}};
            guest_frame_length    <= {{(GUEST_LEN_W-1){1'b0}}, 1'b1};
            guest_frame_complete  <= 1'b0;
            guest_drop_overlength <= 1'b0;
        end else begin
            // Capture is independent of output arbitration. A new SOF while
            // an incomplete/overlength guest frame exists is a definitive
            // producer resynchronization and restarts the private buffer.
            if (guest_capture_accept) begin
                if (guest_capture_restart) begin
                    guest_drop_overlength <= 1'b0;
                    if (guest_eof) begin
                        guest_write_index <= {GUEST_PTR_W{1'b0}};
                        guest_frame_length <=
                            {{(GUEST_LEN_W-1){1'b0}}, 1'b1};
                        guest_frame_complete <= 1'b1;
                    end else begin
                        guest_write_index <=
                            {{(GUEST_PTR_W-1){1'b0}}, 1'b1};
                    end
                end else if (guest_drop_overlength) begin
                    if (guest_eof) begin
                        guest_drop_overlength <= 1'b0;
                        guest_write_index <= {GUEST_PTR_W{1'b0}};
                    end
                end else if (guest_eof) begin
                    guest_frame_length <= guest_write_index + 1'b1;
                    guest_frame_complete <= 1'b1;
                end else if (guest_write_index == GUEST_LAST_INDEX) begin
                    guest_drop_overlength <= 1'b1;
                end else begin
                    guest_write_index <= guest_write_index + 1'b1;
                end
            end

            case (state)
                STATE_IDLE: begin
                    guest_replay_index <= {GUEST_PTR_W{1'b0}};
                    if (mgmt_valid) begin
                        if (!(out_ready && mgmt_eof))
                            state <= STATE_MGMT_SEND;
                    end else if (guest_frame_complete) begin
                        state <= STATE_GUEST_PREFETCH;
                    end
                end

                STATE_GUEST_PREFETCH: begin
                    // The inferred block RAM has one registered read cycle.
                    state <= STATE_GUEST_SEND;
                end

                STATE_GUEST_SEND: begin
                    if (guest_replay_accept) begin
                        if (guest_replay_last) begin
                            state <= STATE_IDLE;
                            guest_replay_index <= {GUEST_PTR_W{1'b0}};
                            guest_write_index <= {GUEST_PTR_W{1'b0}};
                            guest_frame_complete <= 1'b0;
                            guest_drop_overlength <= 1'b0;
                        end else begin
                            guest_replay_index <= guest_replay_next;
                        end
                    end
                end

                STATE_MGMT_SEND: begin
                    if (mgmt_valid && out_ready && mgmt_eof)
                        state <= STATE_IDLE;
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
