// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - DSP56001 SSI word stream to IEC 60958 / S/PDIF.
//
// The Indigo audio glue clocks the DSP SSI externally at two 24-bit words per
// 48 kHz stereo sample: left, then right.  This block preserves that boundary.
// It requests one DSP word at each S/PDIF subframe boundary and serializes the
// accepted word directly as channel A/B PCM.  The encoded output is the
// biphase-mark stream consumed by the KCU105 ADV7511 S/PDIF input.

`default_nettype none

module cyan_spdif_tx #(
    parameter integer CLK_HZ = 50000000,
    parameter integer SAMPLE_RATE_HZ = 48000
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [23:0] sample_data,
    input  wire        sample_valid,
    input  wire [31:0] channel_status_data,
    input  wire        channel_status_valid,
    input  wire [31:0] user_data,
    input  wire        user_valid,
    input  wire        stream_active,
    input  wire [5:0]  sample_rate_select,
    output reg         sample_ready,
    output reg         spdif,
    output wire [127:0] status
);
    // Indigo HP1's audio Actel derives the SSI/AES sample clock from the
    // six-bit selector written to transmit-control bits 6:1. The
    // encoding chooses 48/44.1 kHz, optionally multiplies by 2/3, then divides
    // by one through eight.  The two external-source encodings retain the
    // parameterized board clock until those inputs are modeled.
    function [31:0] hp1_half_cell_hz;
        input [5:0] rate_select;
        reg [31:0] undivided;
        begin
            case (rate_select[2:1])
                2'b00: undivided = rate_select[0] ? 32'd4096000 :
                                                       32'd6144000;
                2'b01: undivided = rate_select[0] ? 32'd3763200 :
                                                       32'd5644800;
                default: undivided = rate_select[0] ?
                    (SAMPLE_RATE_HZ * 256) / 3 : SAMPLE_RATE_HZ * 128;
            endcase
            case (rate_select[5:3])
                3'd0: hp1_half_cell_hz = undivided;
                3'd1: hp1_half_cell_hz = undivided / 2;
                3'd2: hp1_half_cell_hz = undivided / 3;
                3'd3: hp1_half_cell_hz = undivided / 4;
                3'd4: hp1_half_cell_hz = undivided / 5;
                3'd5: hp1_half_cell_hz = undivided / 6;
                3'd6: hp1_half_cell_hz = undivided / 7;
                default: hp1_half_cell_hz = undivided / 8;
            endcase
        end
    endfunction

    wire [31:0] half_cell_hz = hp1_half_cell_hz(sample_rate_select);

    reg [31:0] phase;
    wire [32:0] phase_sum = {1'b0, phase} + half_cell_hz;
    wire half_cell_tick = phase_sum >= CLK_HZ;

    reg [5:0] half_cell;
    reg       channel_b;
    reg [7:0] frame_index;
    reg       preamble_invert;
    reg [23:0] current_sample;
    reg       sample_request_pending;
    reg       stream_active_d;
    reg       align_stream_start;

    // Passive physical-stream ledger.  Counters describe only the current or
    // most recently completed GO interval and never feed serializer control.
    // The rolling signature rotates left by one then XORs each accepted
    // 24-bit PCM word, preserving order as well as content.
    reg        telemetry_session_seen;
    reg        telemetry_session_completed;
    reg [5:0]  telemetry_rate_select;
    reg [23:0] telemetry_request_count;
    reg [23:0] telemetry_accept_count;
    reg [15:0] telemetry_underrun_count;
    reg [7:0]  telemetry_nonzero_count;
    reg [23:0] telemetry_sample_signature;
    reg        telemetry_request_saturated;
    reg        telemetry_accept_saturated;

    assign status = {
        8'h41, 8'd1,
        stream_active,
        telemetry_session_seen,
        telemetry_session_completed,
        sample_request_pending,
        channel_status_valid,
        user_valid,
        telemetry_request_saturated,
        telemetry_accept_saturated,
        telemetry_rate_select,
        2'b00,
        telemetry_request_count,
        telemetry_accept_count,
        telemetry_underrun_count,
        telemetry_nonzero_count,
        telemetry_sample_signature
    };

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            telemetry_session_seen <= 1'b0;
            telemetry_session_completed <= 1'b0;
            telemetry_rate_select <= 6'd0;
            telemetry_request_count <= 24'd0;
            telemetry_accept_count <= 24'd0;
            telemetry_underrun_count <= 16'd0;
            telemetry_nonzero_count <= 8'd0;
            telemetry_sample_signature <= 24'd0;
            telemetry_request_saturated <= 1'b0;
            telemetry_accept_saturated <= 1'b0;
        end else if (stream_active && !stream_active_d) begin
            telemetry_session_seen <= 1'b1;
            telemetry_session_completed <= 1'b0;
            telemetry_rate_select <= sample_rate_select;
            telemetry_request_count <= 24'd0;
            telemetry_accept_count <= 24'd0;
            telemetry_underrun_count <= 16'd0;
            telemetry_nonzero_count <= 8'd0;
            telemetry_sample_signature <= 24'd0;
            telemetry_request_saturated <= 1'b0;
            telemetry_accept_saturated <= 1'b0;
        end else begin
            if (stream_active) begin
                telemetry_session_seen <= 1'b1;
                telemetry_rate_select <= sample_rate_select;
            end
            if (!stream_active && stream_active_d && telemetry_session_seen)
                telemetry_session_completed <= 1'b1;

            if (stream_active && half_cell_tick && half_cell == 6'd63) begin
                if (telemetry_request_count != 24'hff_ffff)
                    telemetry_request_count <=
                        telemetry_request_count + 1'b1;
                else
                    telemetry_request_saturated <= 1'b1;
            end
            if (stream_active && sample_request_pending && sample_valid) begin
                if (telemetry_accept_count != 24'hff_ffff)
                    telemetry_accept_count <= telemetry_accept_count + 1'b1;
                else
                    telemetry_accept_saturated <= 1'b1;
                if (sample_data != 24'd0 &&
                    telemetry_nonzero_count != 8'hff)
                    telemetry_nonzero_count <=
                        telemetry_nonzero_count + 1'b1;
                telemetry_sample_signature <=
                    {telemetry_sample_signature[22:0],
                     telemetry_sample_signature[23]} ^ sample_data;
            end
            if (stream_active && half_cell_tick && half_cell == 6'd7 &&
                sample_request_pending && !sample_valid &&
                telemetry_underrun_count != 16'hffff)
                telemetry_underrun_count <=
                    telemetry_underrun_count + 1'b1;
        end
    end

    wire [7:0] preamble = channel_b ? 8'b11100100 :
                           (frame_index == 0 ? 8'b11101000 :
                                               8'b11100010);
    wire [4:0] data_bit_index = 5'd4 + ((half_cell - 6'd8) >> 1);
    // HP1 supplies the first four bytes of each metadata block in AES3 wire
    // order (least-significant bit first).  Preserve the historical 48 kHz
    // consumer indication until firmware has staged a complete C group, then
    // serialize the supplied bits on both channel subframes.  Frames beyond
    // the four-byte HP1 window are zero.  User data is emitted only after the
    // glue has both a complete U group and its explicit enable.
    wire channel_status_bit = channel_status_valid ?
        ((frame_index < 8'd32) ?
         channel_status_data[frame_index[4:0]] : 1'b0) :
        (frame_index == 8'd25);
    wire user_bit = user_valid && frame_index < 8'd32 ?
                    user_data[frame_index[4:0]] : 1'b0;
    wire parity_bit = ^{current_sample, user_bit, channel_status_bit};
    reg encoded_bit;

    always @(*) begin
        encoded_bit = 1'b0;
        if (data_bit_index >= 5'd4 && data_bit_index <= 5'd27)
            encoded_bit = current_sample[data_bit_index - 5'd4];
        else if (data_bit_index == 5'd29)
            encoded_bit = user_bit;
        else if (data_bit_index == 5'd30)
            encoded_bit = channel_status_bit;
        else if (data_bit_index == 5'd31)
            encoded_bit = parity_bit;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase <= 32'd0;
            half_cell <= 6'd0;
            channel_b <= 1'b0;
            frame_index <= 8'd0;
            preamble_invert <= 1'b0;
            current_sample <= 24'd0;
            sample_request_pending <= 1'b0;
            // A high level present at reset preserves the legacy continuous
            // stream.  A later low-to-high transition is a real HP1 GO edge.
            stream_active_d <= 1'b1;
            align_stream_start <= 1'b0;
            sample_ready <= 1'b0;
            spdif <= 1'b0;
        end else begin
            stream_active_d <= stream_active;
            if (!stream_active)
                align_stream_start <= 1'b0;
            else if (!stream_active_d)
                align_stream_start <= 1'b1;

            sample_ready <= 1'b0;
            if (sample_request_pending && sample_valid) begin
                current_sample <= sample_data;
                sample_request_pending <= 1'b0;
            end

            // A word request is issued at the preceding subframe boundary.
            // If it is still unanswered immediately before PCM bit 4 begins,
            // emit silence for this subframe instead of replaying stale PCM.
            // A same-cycle valid response wins through the guard above.
            if (half_cell_tick && half_cell == 6'd7 &&
                sample_request_pending && !sample_valid)
                current_sample <= 24'd0;

            if (half_cell_tick)
                phase <= phase_sum - CLK_HZ;
            else
                phase <= phase_sum[31:0];

            if (half_cell_tick) begin
                if (half_cell < 6'd8) begin
                    spdif <= preamble[7 - half_cell[2:0]] ^
                             preamble_invert;
                end else if (!half_cell[0]) begin
                    // Every data bit starts with a transition.
                    spdif <= ~spdif;
                end else if (encoded_bit) begin
                    // A one adds a second transition at mid-bit.
                    spdif <= ~spdif;
                end

                if (half_cell == 6'd63) begin
                    // The transfer completed at this subframe boundary.  The
                    // DSP responds one machine clock later, well before PCM
                    // data begins at half-cell eight of the new subframe.
                    sample_ready <= 1'b1;
                    sample_request_pending <= 1'b1;
                    preamble_invert <= encoded_bit ? ~spdif : spdif;
                    half_cell <= 6'd0;
                    // HP1 no-go resets the transmitter's subframe counter.
                    // Align the first requested DSP word after GO to channel
                    // A/frame zero while still emitting silence beforehand.
                    if (stream_active &&
                        (align_stream_start || !stream_active_d)) begin
                        channel_b <= 1'b0;
                        frame_index <= 8'd0;
                        align_stream_start <= 1'b0;
                    end else if (channel_b) begin
                        channel_b <= 1'b0;
                        if (frame_index == 8'd191)
                            frame_index <= 8'd0;
                        else
                            frame_index <= frame_index + 1'b1;
                    end else begin
                        channel_b <= 1'b1;
                    end
                end else begin
                    half_cell <= half_cell + 1'b1;
                end
            end
        end
    end
endmodule

`default_nettype wire
