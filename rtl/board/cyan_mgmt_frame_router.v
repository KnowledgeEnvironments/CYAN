// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - bounded Ethernet management-frame diversion.
//
// A complete received Ethernet frame is buffered before it is classified.
// Frames addressed to CYAN1 with the reserved management EtherType are sent
// to the management endpoint. Every other legal-sized frame is replayed
// byte-for-byte when the guest can accept its SOF. Because Ethernet cannot
// backpressure the wire, an ordinary frame whose guest consumer remains busy
// is instead discarded before SOF after a bounded wait. This prevents a
// stopped guest from making the raw management endpoint permanently
// unreachable on a finite queued frame. GUEST_REPLAY_WAIT_CYCLES is measured
// in clk cycles; its 50,000-cycle default is 1 ms in the 50 MHz KCU105
// integration. guest_ready must be an eager capacity signal independent of
// guest_valid, which is true of the connected HPC1 Ethernet receiver. The
// first guest byte handshakes in the same cycle that eager ready is observed.

`default_nettype none

module cyan_mgmt_frame_router #(
    parameter integer MAX_FRAME_BYTES = 2048,
    parameter integer GUEST_REPLAY_WAIT_CYCLES = 50000
) (
    input  wire       clk,
    input  wire       rst_n,

    input  wire [7:0] in_data,
    input  wire       in_valid,
    input  wire       in_sof,
    input  wire       in_eof,
    output wire       in_ready,

    output wire [7:0] guest_data,
    output wire       guest_valid,
    output wire       guest_sof,
    output wire       guest_eof,
    input  wire       guest_ready,

    output wire [7:0] mgmt_data,
    output wire       mgmt_valid,
    output wire       mgmt_sof,
    output wire       mgmt_eof,
    input  wire       mgmt_ready,

    // Classification counters: an ordinary frame later dropped by the
    // pre-SOF timeout increments both guest_frame_count and dropped_frame_count.
    output reg [31:0] guest_frame_count,
    output reg [31:0] mgmt_frame_count,
    output reg [31:0] dropped_frame_count
);
    localparam integer PTR_W = (MAX_FRAME_BYTES <= 2) ? 1 : $clog2(MAX_FRAME_BYTES);
    localparam integer LEN_W = (MAX_FRAME_BYTES <= 1) ? 1 : $clog2(MAX_FRAME_BYTES + 1);
    localparam integer GUEST_WAIT_LIMIT =
        (GUEST_REPLAY_WAIT_CYCLES < 1) ? 1 : GUEST_REPLAY_WAIT_CYCLES;
    localparam integer GUEST_WAIT_W =
        (GUEST_WAIT_LIMIT <= 1) ? 1 : $clog2(GUEST_WAIT_LIMIT);
    localparam [31:0] GUEST_WAIT_LAST = GUEST_WAIT_LIMIT - 1;

    localparam [1:0]
        STATE_IDLE    = 2'd0,
        STATE_CAPTURE = 2'd1,
        STATE_REPLAY  = 2'd2,
        STATE_DROP    = 2'd3;

    reg [1:0] state;
    reg [7:0] frame_mem [0:MAX_FRAME_BYTES-1];
    reg [PTR_W-1:0] write_index;
    reg [PTR_W-1:0] replay_index;
    reg [LEN_W-1:0] frame_length;
    reg              header_matches;
    reg              replay_to_mgmt;
    reg              guest_offer_committed;
    reg [GUEST_WAIT_W-1:0] guest_replay_wait;

    function header_byte_matches;
        input [PTR_W-1:0] index;
        input [7:0] data;
        begin
            case (index)
                0:  header_byte_matches = (data == 8'h02);
                1:  header_byte_matches = (data == 8'h43);
                2:  header_byte_matches = (data == 8'h59);
                3:  header_byte_matches = (data == 8'h41);
                4:  header_byte_matches = (data == 8'h4e);
                5:  header_byte_matches = (data == 8'h01);
                12: header_byte_matches = (data == 8'h88);
                13: header_byte_matches = (data == 8'hb5);
                default: header_byte_matches = 1'b1;
            endcase
        end
    endfunction

    wire replay_valid = (state == STATE_REPLAY);
    wire replay_ready = replay_to_mgmt ? mgmt_ready : guest_ready;
    wire [LEN_W-1:0] replay_index_ext =
        {{(LEN_W-PTR_W){1'b0}}, replay_index};
    wire [31:0] write_index_ext = {{(32-PTR_W){1'b0}}, write_index};
    wire replay_last = replay_valid &&
                       (replay_index_ext == frame_length - 1'b1);

    // The upstream MAC already bounds legal frames to MAX_FRAME_BYTES.  A
    // malformed overlength frame is consumed and counted so it cannot wedge
    // the RX path.
    assign in_ready = (state == STATE_IDLE) ||
                      (state == STATE_CAPTURE) ||
                      (state == STATE_DROP);

    assign guest_data  = frame_mem[replay_index];
    assign guest_valid = replay_valid && !replay_to_mgmt &&
                         (guest_offer_committed || guest_ready);
    assign guest_sof   = guest_valid && (replay_index == {PTR_W{1'b0}});
    assign guest_eof   = guest_valid && replay_last;

    assign mgmt_data  = frame_mem[replay_index];
    assign mgmt_valid = replay_valid && replay_to_mgmt;
    assign mgmt_sof   = mgmt_valid && (replay_index == {PTR_W{1'b0}});
    assign mgmt_eof   = mgmt_valid && replay_last;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state               <= STATE_IDLE;
            write_index         <= {PTR_W{1'b0}};
            replay_index        <= {PTR_W{1'b0}};
            frame_length        <= {{(LEN_W-1){1'b0}}, 1'b1};
            header_matches      <= 1'b0;
            replay_to_mgmt      <= 1'b0;
            guest_offer_committed <= 1'b0;
            guest_replay_wait   <= {GUEST_WAIT_W{1'b0}};
            guest_frame_count   <= 32'd0;
            mgmt_frame_count    <= 32'd0;
            dropped_frame_count <= 32'd0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    replay_index <= {PTR_W{1'b0}};
                    guest_offer_committed <= 1'b0;
                    guest_replay_wait <= {GUEST_WAIT_W{1'b0}};
                    if (in_valid && in_ready && in_sof) begin
                        frame_mem[0] <= in_data;
                        write_index <= {{(PTR_W-1){1'b0}}, 1'b1};
                        header_matches <= header_byte_matches({PTR_W{1'b0}}, in_data);
                        if (in_eof) begin
                            frame_length <= {{(LEN_W-1){1'b0}}, 1'b1};
                            replay_to_mgmt <= 1'b0;
                            guest_frame_count <= guest_frame_count + 1'b1;
                            state <= STATE_REPLAY;
                        end else begin
                            state <= STATE_CAPTURE;
                        end
                    end
                end

                STATE_CAPTURE: begin
                    if (in_valid && in_ready) begin
                        frame_mem[write_index] <= in_data;
                        header_matches <= header_matches &&
                                          header_byte_matches(write_index, in_data);
                        if (in_eof) begin
                            frame_length <= write_index + 1'b1;
                            replay_index <= {PTR_W{1'b0}};
                            // A management frame must contain the complete
                            // 14-byte Ethernet header.
                            replay_to_mgmt <= header_matches &&
                                header_byte_matches(write_index, in_data) &&
                                (write_index >= 13);
                            if (header_matches &&
                                header_byte_matches(write_index, in_data) &&
                                (write_index >= 13))
                                mgmt_frame_count <= mgmt_frame_count + 1'b1;
                            else
                                guest_frame_count <= guest_frame_count + 1'b1;
                            state <= STATE_REPLAY;
                        end else if (write_index_ext == MAX_FRAME_BYTES - 1) begin
                            dropped_frame_count <= dropped_frame_count + 1'b1;
                            state <= STATE_DROP;
                        end else begin
                            write_index <= write_index + 1'b1;
                        end
                    end
                end

                STATE_REPLAY: begin
                    // Do not offer an ordinary SOF until its eager sink is
                    // ready. The first byte handshakes on that same cycle, so
                    // no next-cycle capacity reservation is assumed. Before
                    // that handshake the frame may be dropped after a bounded
                    // wait. Afterwards, normal ready/valid rules apply and the
                    // remaining frame stays stable through backpressure.
                    if (!replay_to_mgmt && !guest_offer_committed) begin
                        if (guest_ready) begin
                            guest_replay_wait <= {GUEST_WAIT_W{1'b0}};
                            if (replay_last) begin
                                replay_index <= {PTR_W{1'b0}};
                                state <= STATE_IDLE;
                            end else begin
                                guest_offer_committed <= 1'b1;
                                replay_index <= replay_index + 1'b1;
                            end
                        end else begin
                            if (guest_replay_wait ==
                                GUEST_WAIT_LAST[GUEST_WAIT_W-1:0]) begin
                                guest_replay_wait <= {GUEST_WAIT_W{1'b0}};
                                dropped_frame_count <=
                                    dropped_frame_count + 1'b1;
                                state <= STATE_IDLE;
                            end else begin
                                guest_replay_wait <= guest_replay_wait + 1'b1;
                            end
                        end
                    end else if (replay_valid && replay_ready) begin
                        guest_replay_wait <= {GUEST_WAIT_W{1'b0}};
                        if (replay_last) begin
                            replay_index <= {PTR_W{1'b0}};
                            guest_offer_committed <= 1'b0;
                            state <= STATE_IDLE;
                        end else begin
                            replay_index <= replay_index + 1'b1;
                        end
                    end
                end

                STATE_DROP: begin
                    if (in_valid && in_ready && in_eof)
                        state <= STATE_IDLE;
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
