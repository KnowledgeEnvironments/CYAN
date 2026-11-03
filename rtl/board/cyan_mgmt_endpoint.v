// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - raw-Ethernet management protocol endpoint.
//
// Input/output frames include the 14-byte Ethernet header but exclude FCS.
// The endpoint validates the CYAN message header and payload CRC before any
// guest-visible operation.  Keyboard/mouse payloads are then transferred to
// the paced SCC device queues with ordinary ready/valid backpressure.

`default_nettype none

module cyan_mgmt_endpoint #(
    parameter integer MAX_PAYLOAD_BYTES = 256,
    parameter [31:0] BUILD_ID = 32'h2026_0809
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  rx_data,
    input  wire        rx_valid,
    input  wire        rx_sof,
    input  wire        rx_eof,
    output wire        rx_ready,

    output wire [7:0]  tx_data,
    output wire        tx_valid,
    output wire        tx_sof,
    output wire        tx_eof,
    input  wire        tx_ready,

    output wire [7:0]  host_kbd_data,
    output wire        host_kbd_valid,
    input  wire        host_kbd_ready,
    output wire [7:0]  host_mouse_data,
    output wire        host_mouse_valid,
    input  wire        host_mouse_ready,

    input  wire [15:0] host_kbd_queued,
    input  wire [15:0] host_mouse_queued,
    input  wire [31:0] kbd_tx_event_overflow,
    input  wire [31:0] mouse_tx_event_overflow,

    input  wire [127:0] board_status,
    input  wire [127:0] hdmi_link_status,
    input  wire [127:0] video_status,
    input  wire [127:0] video_signature_status,
    input  wire [127:0] video_stage_signature_status,
    input  wire [127:0] resolver_status,
    input  wire [127:0] resolver_detail_status,
    input  wire [127:0] visual_stage_status,
    input  wire [127:0] scan_service_status,
    input  wire [127:0] scan_latency_status,
    input  wire [127:0] audio_status,
    input  wire [127:0] gfx_retrace_status,
    input  wire [127:0] lg1_trace_status,
    output reg  [1:0]   lg1_trace_command,
    output reg  [7:0]   lg1_trace_command_tag,
    output reg          lg1_trace_command_valid,
    input  wire         lg1_trace_command_ready,
    input  wire         lg1_trace_command_ack,
    input  wire         lg1_trace_command_ok,
    output reg  [15:0]  lg1_trace_read_ordinal,
    output reg          lg1_trace_read_request,
    input  wire [95:0]  lg1_trace_read_data,
    input  wire         lg1_trace_read_ack,
    input  wire         lg1_trace_read_valid,
    output reg          video_status_request_toggle,
    input  wire         video_status_ack_toggle,
    output reg  [1:0]   fan_duty_select,

    output reg  [1:0]   capture_command,
    output reg          capture_command_toggle,
    output reg  [15:0]  capture_word_index,
    input  wire         capture_ack_toggle,
    input  wire [127:0] capture_info,
    input  wire [511:0] capture_word_data,
    input  wire         capture_word_valid,

    input  wire [31:0] scc_access_event_data,
    input  wire        scc_access_event_valid,
    output wire        scc_access_event_ready,
    output reg  [1:0]  scc_trace_control,
    output reg         scc_trace_control_valid,

    input  wire [7:0]  kbd_tx_event_data,
    input  wire        kbd_tx_event_valid,
    output wire        kbd_tx_event_ready,
    input  wire [7:0]  mouse_tx_event_data,
    input  wire        mouse_tx_event_valid,
    output wire        mouse_tx_event_ready,
    input  wire [7:0]  kbd_rx_event_data,
    input  wire        kbd_rx_event_valid,
    output wire        kbd_rx_event_ready,
    input  wire [7:0]  mouse_rx_event_data,
    input  wire        mouse_rx_event_valid,
    output wire        mouse_rx_event_ready,

    output reg  [31:0] accepted_request_count,
    output reg  [31:0] rejected_request_count
);
    localparam integer HEADER_BYTES = 34;
    localparam integer INDEX_W = 16;
    localparam integer PAYLOAD_INDEX_W =
        (MAX_PAYLOAD_BYTES <= 2) ? 1 : $clog2(MAX_PAYLOAD_BYTES);

    localparam [3:0]
        STATE_IDLE       = 4'd0,
        STATE_CAPTURE    = 4'd1,
        STATE_VALIDATE   = 4'd2,
        STATE_EXECUTE    = 4'd3,
        STATE_PREP_CRC   = 4'd4,
        STATE_CALC_CRC   = 4'd5,
        STATE_SEND       = 4'd6,
        STATE_CAPTURE_WAIT = 4'd7,
        STATE_VIDEO_STATUS_WAIT = 4'd8,
        STATE_LG1_TRACE_COMMAND_WAIT = 4'd9,
        STATE_LG1_TRACE_READ_WAIT = 4'd10;

    localparam [7:0]
        OP_STATUS      = 8'h01,
        OP_FAN         = 8'h03,
        OP_BOARD_STATUS = 8'h04,
        OP_HDMI_LINK_STATUS = 8'h05,
        OP_KBD_BYTES   = 8'h10,
        OP_MOUSE_BYTES = 8'h11,
        OP_KBD_TX_EVENTS   = 8'h20,
        OP_MOUSE_TX_EVENTS = 8'h21,
        OP_KBD_RX_EVENTS   = 8'h22,
        OP_MOUSE_RX_EVENTS = 8'h23,
        OP_VIDEO_STATUS    = 8'h24,
        OP_SCC_ACCESS_EVENTS = 8'h25,
        OP_SCC_TRACE_CONTROL = 8'h26,
        OP_VIDEO_SIGNATURES = 8'h27,
        OP_VIDEO_STAGE_SIGNATURES = 8'h28,
        OP_RESOLVER_STATUS   = 8'h29,
        OP_RESOLVER_DETAIL_STATUS = 8'h2a,
        OP_LG1_TRACE_ARM      = 8'h2b,
        OP_LG1_TRACE_MARK     = 8'h2c,
        OP_LG1_TRACE_FREEZE   = 8'h2d,
        OP_LG1_TRACE_STATUS   = 8'h2e,
        OP_LG1_TRACE_READ     = 8'h2f,
        OP_CAPTURE_INFO      = 8'h30,
        OP_CAPTURE_READ      = 8'h31,
        OP_GFX_RETRACE_STATUS = 8'h32,
        OP_VISUAL_STAGE_STATUS = 8'h33,
        OP_SCAN_SERVICE_STATUS = 8'h34,
        OP_SCAN_LATENCY_STATUS = 8'h35,
        OP_AUDIO_STATUS        = 8'h36;

    reg [3:0] state;
    reg [7:0] header_mem [0:HEADER_BYTES-1];
    reg [7:0] request_payload_mem [0:MAX_PAYLOAD_BYTES-1];
    reg [7:0] response_payload_mem [0:15];
    reg [INDEX_W-1:0] capture_index;
    reg [INDEX_W-1:0] frame_length;
    reg [15:0] request_payload_length;
    reg [15:0] payload_received;
    reg [31:0] request_crc_work;
    reg [15:0] execute_index;
    reg [7:0] response_status;
    reg [7:0] response_opcode;
    reg [15:0] response_payload_length;
    reg [31:0] response_crc_work;
    reg [31:0] response_crc;
    reg [3:0] response_crc_index;
    reg [INDEX_W-1:0] transmit_index;
    reg [1:0] capture_chunk;
    (* ASYNC_REG = "TRUE" *) reg [1:0] capture_ack_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] video_status_ack_sync;

    function [7:0] capture_data_byte;
        input [511:0] word_data;
        input [31:0] byte_index;
        integer pixel_index;
        integer bit_index;
        begin
            if (byte_index >= 48) begin
                capture_data_byte =
                    word_data[384 + (byte_index - 48) * 8 +: 8];
            end else begin
                pixel_index = byte_index / 3;
                bit_index = pixel_index * 24;
                case (byte_index % 3)
                    0: capture_data_byte = word_data[bit_index + 16 +: 8];
                    1: capture_data_byte = word_data[bit_index + 8 +: 8];
                    default: capture_data_byte = word_data[bit_index +: 8];
                endcase
            end
        end
    endfunction

    function [31:0] crc32_next;
        input [31:0] crc;
        input [7:0] data;
        integer bit_index;
        reg [31:0] value;
        begin
            value = crc;
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                if ((value[0] ^ data[bit_index]) != 1'b0)
                    value = (value >> 1) ^ 32'hedb8_8320;
                else
                    value = value >> 1;
            end
            crc32_next = value;
        end
    endfunction

    function [7:0] response_byte;
        input [INDEX_W-1:0] index;
        reg [INDEX_W-1:0] payload_index;
        begin
            payload_index = index - 16'd34;
            if (index < 6)
                response_byte = header_mem[index[5:0] + 6'd6];
            else if (index == 6)
                response_byte = 8'h02;
            else if (index == 7)
                response_byte = 8'h43;
            else if (index == 8)
                response_byte = 8'h59;
            else if (index == 9)
                response_byte = 8'h41;
            else if (index == 10)
                response_byte = 8'h4e;
            else if (index == 11)
                response_byte = 8'h01;
            else if (index == 12)
                response_byte = 8'h88;
            else if (index == 13)
                response_byte = 8'hb5;
            else if (index == 14)
                response_byte = 8'h43;
            else if (index == 15)
                response_byte = 8'h59;
            else if (index == 16)
                response_byte = 8'h41;
            else if (index == 17)
                response_byte = 8'h4e;
            else if (index == 18)
                response_byte = 8'h01;
            else if (index == 19)
                response_byte = response_opcode;
            else if (index == 20)
                response_byte = 8'h01;
            else if (index == 21)
                response_byte = response_status;
            else if (index >= 22 && index <= 25)
                response_byte = header_mem[index[5:0]];
            else if (index == 26)
                response_byte = response_payload_length[15:8];
            else if (index == 27)
                response_byte = response_payload_length[7:0];
            else if (index == 28 || index == 29)
                response_byte = 8'h00;
            else if (index == 30)
                response_byte = response_crc[31:24];
            else if (index == 31)
                response_byte = response_crc[23:16];
            else if (index == 32)
                response_byte = response_crc[15:8];
            else if (index == 33)
                response_byte = response_crc[7:0];
            else
                response_byte = response_payload_mem[payload_index[3:0]];
        end
    endfunction

    wire [31:0] expected_request_crc = {
        header_mem[30], header_mem[31], header_mem[32], header_mem[33]
    };
    wire [31:0] completed_request_crc = ~request_crc_work;
    wire [31:0] response_crc_next = crc32_next(
        response_crc_work, response_payload_mem[response_crc_index[3:0]]
    );
    wire [INDEX_W-1:0] response_last_index =
        16'd34 + response_payload_length - 1'b1;
    wire [31:0] frame_length_ext = {16'd0, frame_length};
    wire [31:0] request_payload_length_ext =
        {16'd0, request_payload_length};
    wire [31:0] payload_received_ext = {16'd0, payload_received};
    wire [15:0] response_crc_index_ext = {12'd0, response_crc_index};

    wire request_header_valid =
        (frame_length_ext >= 32'd34) &&
        (header_mem[14] == 8'h43) && (header_mem[15] == 8'h59) &&
        (header_mem[16] == 8'h41) && (header_mem[17] == 8'h4e) &&
        (header_mem[18] == 8'h01) &&
        ((header_mem[20] & 8'h01) == 0) &&
        (header_mem[28] == 0) && (header_mem[29] == 0) &&
        (request_payload_length_ext <= MAX_PAYLOAD_BYTES) &&
        (frame_length_ext >= 32'd34 + request_payload_length_ext) &&
        (payload_received == request_payload_length) &&
        (expected_request_crc == completed_request_crc);

    assign rx_ready = (state == STATE_IDLE) || (state == STATE_CAPTURE);

    assign host_kbd_data =
        request_payload_mem[execute_index[PAYLOAD_INDEX_W-1:0]];
    assign host_kbd_valid = (state == STATE_EXECUTE) &&
                            (response_opcode == OP_KBD_BYTES) &&
                            (execute_index < request_payload_length);
    assign host_mouse_data =
        request_payload_mem[execute_index[PAYLOAD_INDEX_W-1:0]];
    assign host_mouse_valid = (state == STATE_EXECUTE) &&
                              (response_opcode == OP_MOUSE_BYTES) &&
                              (execute_index < request_payload_length);
    assign kbd_tx_event_ready = (state == STATE_EXECUTE) &&
                                (response_opcode == OP_KBD_TX_EVENTS) &&
                                (execute_index < 16);
    assign mouse_tx_event_ready = (state == STATE_EXECUTE) &&
                                  (response_opcode == OP_MOUSE_TX_EVENTS) &&
                                  (execute_index < 16);
    assign kbd_rx_event_ready = (state == STATE_EXECUTE) &&
                                (response_opcode == OP_KBD_RX_EVENTS) &&
                                (execute_index < 16);
    assign mouse_rx_event_ready = (state == STATE_EXECUTE) &&
                                  (response_opcode == OP_MOUSE_RX_EVENTS) &&
                                (execute_index < 16);
    assign scc_access_event_ready = (state == STATE_EXECUTE) &&
                                    (response_opcode == OP_SCC_ACCESS_EVENTS) &&
                                    (execute_index < 4);

    assign tx_data = response_byte(transmit_index);
    assign tx_valid = (state == STATE_SEND);
    assign tx_sof = tx_valid && (transmit_index == 0);
    assign tx_eof = tx_valid && (transmit_index == response_last_index);

    integer reset_index;
    integer response_index;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                    <= STATE_IDLE;
            capture_index            <= 0;
            frame_length             <= 0;
            request_payload_length   <= 0;
            payload_received         <= 0;
            request_crc_work         <= 32'hffff_ffff;
            execute_index            <= 0;
            response_status          <= 0;
            response_opcode          <= 0;
            response_payload_length  <= 0;
            response_crc_work        <= 32'hffff_ffff;
            response_crc             <= 0;
            response_crc_index       <= 0;
            transmit_index           <= 0;
            accepted_request_count   <= 0;
            rejected_request_count   <= 0;
            fan_duty_select           <= 2'b01;
            capture_command           <= 2'd0;
            capture_command_toggle    <= 1'b0;
            capture_word_index        <= 16'd0;
            capture_chunk             <= 2'd0;
            capture_ack_sync          <= 2'b00;
            video_status_request_toggle <= 1'b0;
            video_status_ack_sync      <= 2'b00;
            lg1_trace_command            <= 2'd0;
            lg1_trace_command_tag        <= 8'd0;
            lg1_trace_command_valid      <= 1'b0;
            lg1_trace_read_ordinal       <= 16'd0;
            lg1_trace_read_request       <= 1'b0;
            scc_trace_control           <= 2'd0;
            scc_trace_control_valid     <= 1'b0;
            for (reset_index = 0; reset_index < HEADER_BYTES; reset_index = reset_index + 1)
                header_mem[reset_index] <= 0;
        end else begin
            scc_trace_control_valid <= 1'b0;
            lg1_trace_read_request <= 1'b0;
            capture_ack_sync <= {capture_ack_sync[0], capture_ack_toggle};
            video_status_ack_sync <= {
                video_status_ack_sync[0], video_status_ack_toggle
            };
            case (state)
                STATE_IDLE: begin
                    if (rx_valid && rx_ready && rx_sof) begin
                        header_mem[0] <= rx_data;
                        capture_index <= 1;
                        frame_length <= rx_eof ? 1 : 0;
                        request_payload_length <= 0;
                        payload_received <= 0;
                        request_crc_work <= 32'hffff_ffff;
                        state <= rx_eof ? STATE_VALIDATE : STATE_CAPTURE;
                    end
                end

                STATE_CAPTURE: begin
                    if (rx_valid && rx_ready) begin
                        if (capture_index < 16'd34)
                            header_mem[capture_index[5:0]] <= rx_data;
                        if (capture_index == 26)
                            request_payload_length[15:8] <= rx_data;
                        if (capture_index == 27)
                            request_payload_length[7:0] <= rx_data;
                        if (capture_index >= 16'd34 &&
                            payload_received < request_payload_length) begin
                            if (payload_received_ext < MAX_PAYLOAD_BYTES)
                                request_payload_mem[payload_received[PAYLOAD_INDEX_W-1:0]] <= rx_data;
                            request_crc_work <= crc32_next(request_crc_work, rx_data);
                            payload_received <= payload_received + 1'b1;
                        end
                        if (rx_eof) begin
                            frame_length <= capture_index + 1'b1;
                            state <= STATE_VALIDATE;
                        end else begin
                            capture_index <= capture_index + 1'b1;
                        end
                    end
                end

                STATE_VALIDATE: begin
                    response_opcode <= header_mem[19];
                    execute_index <= 0;
                    response_payload_length <= 0;
                    if (!request_header_valid) begin
                        response_status <= 8'h02;
                        rejected_request_count <= rejected_request_count + 1'b1;
                        state <= STATE_PREP_CRC;
                    end else begin
                        accepted_request_count <= accepted_request_count + 1'b1;
                        case (header_mem[19])
                            OP_STATUS: begin
                                response_status <= 0;
                                response_payload_length <= 16;
                                response_payload_mem[0]  <= BUILD_ID[31:24];
                                response_payload_mem[1]  <= BUILD_ID[23:16];
                                response_payload_mem[2]  <= BUILD_ID[15:8];
                                response_payload_mem[3]  <= BUILD_ID[7:0];
                                response_payload_mem[4]  <= host_kbd_queued[15:8];
                                response_payload_mem[5]  <= host_kbd_queued[7:0];
                                response_payload_mem[6]  <= host_mouse_queued[15:8];
                                response_payload_mem[7]  <= host_mouse_queued[7:0];
                                response_payload_mem[8]  <= kbd_tx_event_overflow[31:24];
                                response_payload_mem[9]  <= kbd_tx_event_overflow[23:16];
                                response_payload_mem[10] <= kbd_tx_event_overflow[15:8];
                                response_payload_mem[11] <= kbd_tx_event_overflow[7:0];
                                response_payload_mem[12] <= mouse_tx_event_overflow[31:24];
                                response_payload_mem[13] <= mouse_tx_event_overflow[23:16];
                                response_payload_mem[14] <= mouse_tx_event_overflow[15:8];
                                response_payload_mem[15] <= mouse_tx_event_overflow[7:0];
                                state <= STATE_PREP_CRC;
                            end
                            OP_BOARD_STATUS: begin
                                response_status <= 0;
                                response_payload_length <= 16;
                                for (response_index = 0;
                                     response_index < 16;
                                     response_index = response_index + 1)
                                    response_payload_mem[response_index] <=
                                        board_status[127-response_index*8 -: 8];
                                state <= STATE_PREP_CRC;
                            end
                            OP_HDMI_LINK_STATUS: begin
                                response_status <= 0;
                                response_payload_length <= 16;
                                for (response_index = 0;
                                     response_index < 16;
                                     response_index = response_index + 1)
                                    response_payload_mem[response_index] <=
                                        hdmi_link_status[127-response_index*8 -: 8];
                                state <= STATE_PREP_CRC;
                            end
                            OP_GFX_RETRACE_STATUS: begin
                                if (request_payload_length != 0) begin
                                    response_status <= 8'h02;
                                end else begin
                                    response_status <= 0;
                                    response_payload_length <= 16;
                                    for (response_index = 0;
                                         response_index < 16;
                                         response_index = response_index + 1)
                                        response_payload_mem[response_index] <=
                                            gfx_retrace_status[
                                                127-response_index*8 -: 8];
                                end
                                state <= STATE_PREP_CRC;
                            end
                            OP_FAN: begin
                                if (request_payload_length != 1) begin
                                    response_status <= 8'h02;
                                end else begin
                                    case (request_payload_mem[0])
                                        8'd25: fan_duty_select <= 2'b01;
                                        8'd50: fan_duty_select <= 2'b00;
                                        8'd75: fan_duty_select <= 2'b10;
                                        8'd100: fan_duty_select <= 2'b11;
                                        default: response_status <= 8'h02;
                                    endcase
                                    if (request_payload_mem[0] == 8'd25 ||
                                        request_payload_mem[0] == 8'd50 ||
                                        request_payload_mem[0] == 8'd75 ||
                                        request_payload_mem[0] == 8'd100) begin
                                        response_status <= 0;
                                        response_payload_length <= 1;
                                        response_payload_mem[0] <= request_payload_mem[0];
                                    end
                                end
                                state <= STATE_PREP_CRC;
                            end
                            OP_KBD_BYTES, OP_MOUSE_BYTES: begin
                                response_status <= 0;
                                state <= STATE_EXECUTE;
                            end
                            OP_KBD_TX_EVENTS, OP_MOUSE_TX_EVENTS,
                            OP_KBD_RX_EVENTS, OP_MOUSE_RX_EVENTS,
                            OP_SCC_ACCESS_EVENTS: begin
                                response_status <= 0;
                                state <= STATE_EXECUTE;
                            end
                            OP_SCC_TRACE_CONTROL: begin
                                if (request_payload_length != 1 ||
                                    request_payload_mem[0] > 2) begin
                                    response_status <= 8'h02;
                                end else begin
                                    scc_trace_control <=
                                        request_payload_mem[0][1:0];
                                    scc_trace_control_valid <= 1'b1;
                                    response_status <= 0;
                                    response_payload_length <= 1;
                                    response_payload_mem[0] <=
                                        request_payload_mem[0];
                                end
                                state <= STATE_PREP_CRC;
                            end
                            OP_VIDEO_STATUS, OP_VIDEO_SIGNATURES,
                            OP_VIDEO_STAGE_SIGNATURES,
                            OP_RESOLVER_STATUS,
                            OP_RESOLVER_DETAIL_STATUS,
                            OP_VISUAL_STAGE_STATUS,
                            OP_SCAN_SERVICE_STATUS,
                            OP_SCAN_LATENCY_STATUS: begin
                                if (request_payload_length != 0) begin
                                    response_status <= 8'h02;
                                    state <= STATE_PREP_CRC;
                                end else begin
                                    response_status <= 0;
                                    video_status_request_toggle <=
                                        ~video_status_request_toggle;
                                    state <= STATE_VIDEO_STATUS_WAIT;
                                end
                            end
                            OP_AUDIO_STATUS: begin
                                if (request_payload_length != 0) begin
                                    response_status <= 8'h02;
                                    response_payload_length <= 0;
                                end else begin
                                    response_status <= 0;
                                    response_payload_length <= 16;
                                    for (response_index = 0;
                                         response_index < 16;
                                         response_index = response_index + 1)
                                        response_payload_mem[response_index] <=
                                            audio_status[
                                                127-response_index*8 -: 8];
                                end
                                state <= STATE_PREP_CRC;
                            end
                            OP_LG1_TRACE_ARM: begin
                                if (request_payload_length > 1 ||
                                    (request_payload_length == 1 &&
                                     request_payload_mem[0] > 3)) begin
                                    response_status <= 8'h02;
                                    state <= STATE_PREP_CRC;
                                end else begin
                                    // Empty/zero selects compatible config
                                    // capture, one selects config writes, and
                                    // two selects accepted write/GO events;
                                    // three selects every accepted access.
                                    lg1_trace_command <=
                                        (request_payload_length == 1 &&
                                         request_payload_mem[0] != 0) ?
                                        2'd3 : 2'd0;
                                    lg1_trace_command_tag <=
                                        (request_payload_length == 1 &&
                                         request_payload_mem[0] >= 2) ?
                                        request_payload_mem[0] : 8'h00;
                                    lg1_trace_command_valid <= 1'b1;
                                    response_status <= 0;
                                    state <= STATE_LG1_TRACE_COMMAND_WAIT;
                                end
                            end
                            OP_LG1_TRACE_MARK: begin
                                if (request_payload_length != 1) begin
                                    response_status <= 8'h02;
                                    state <= STATE_PREP_CRC;
                                end else begin
                                    lg1_trace_command <= 2'd1;
                                    lg1_trace_command_tag <=
                                        request_payload_mem[0];
                                    lg1_trace_command_valid <= 1'b1;
                                    response_status <= 0;
                                    state <= STATE_LG1_TRACE_COMMAND_WAIT;
                                end
                            end
                            OP_LG1_TRACE_FREEZE: begin
                                if (request_payload_length != 0) begin
                                    response_status <= 8'h02;
                                    state <= STATE_PREP_CRC;
                                end else begin
                                    lg1_trace_command <= 2'd2;
                                    lg1_trace_command_tag <= 8'hff;
                                    lg1_trace_command_valid <= 1'b1;
                                    response_status <= 0;
                                    state <= STATE_LG1_TRACE_COMMAND_WAIT;
                                end
                            end
                            OP_LG1_TRACE_STATUS: begin
                                if (request_payload_length != 0) begin
                                    response_status <= 8'h02;
                                end else begin
                                    response_status <= 0;
                                    response_payload_length <= 16;
                                    for (response_index = 0;
                                         response_index < 16;
                                         response_index = response_index + 1)
                                        response_payload_mem[response_index] <=
                                            lg1_trace_status[
                                                127-response_index*8 -: 8];
                                end
                                state <= STATE_PREP_CRC;
                            end
                            OP_LG1_TRACE_READ: begin
                                if (request_payload_length != 2) begin
                                    response_status <= 8'h02;
                                    state <= STATE_PREP_CRC;
                                end else begin
                                    lg1_trace_read_ordinal <= {
                                        request_payload_mem[0],
                                        request_payload_mem[1]
                                    };
                                    lg1_trace_read_request <= 1'b1;
                                    response_status <= 0;
                                    state <= STATE_LG1_TRACE_READ_WAIT;
                                end
                            end
                            OP_CAPTURE_INFO: begin
                                if (request_payload_length == 0) begin
                                    response_status <= 0;
                                    response_payload_length <= 16;
                                    for (response_index = 0;
                                         response_index < 16;
                                         response_index = response_index + 1)
                                        response_payload_mem[response_index] <=
                                            capture_info[127-response_index*8 -: 8];
                                    state <= STATE_PREP_CRC;
                                end else if (request_payload_length == 1 &&
                                             (request_payload_mem[0] == 8'd1 ||
                                              request_payload_mem[0] == 8'd2)) begin
                                    capture_command <= request_payload_mem[0][1:0];
                                    capture_command_toggle <=
                                        ~capture_command_toggle;
                                    response_status <= 0;
                                    state <= STATE_CAPTURE_WAIT;
                                end else begin
                                    response_status <= 8'h02;
                                    state <= STATE_PREP_CRC;
                                end
                            end
                            OP_CAPTURE_READ: begin
                                if (request_payload_length != 3 ||
                                    request_payload_mem[2] >= 4) begin
                                    response_status <= 8'h02;
                                    state <= STATE_PREP_CRC;
                                end else begin
                                    capture_word_index <= {
                                        request_payload_mem[0],
                                        request_payload_mem[1]
                                    };
                                    capture_chunk <= request_payload_mem[2][1:0];
                                    capture_command <= 2'd3;
                                    capture_command_toggle <=
                                        ~capture_command_toggle;
                                    response_status <= 0;
                                    state <= STATE_CAPTURE_WAIT;
                                end
                            end
                            default: begin
                                response_status <= 8'h01;
                                state <= STATE_PREP_CRC;
                            end
                        endcase
                    end
                end

                STATE_EXECUTE: begin
                    if (response_opcode == OP_KBD_BYTES ||
                        response_opcode == OP_MOUSE_BYTES) begin
                        if (request_payload_length == 0) begin
                            response_payload_length <= 0;
                            state <= STATE_PREP_CRC;
                        end else if ((host_kbd_valid && host_kbd_ready) ||
                                     (host_mouse_valid && host_mouse_ready)) begin
                            if (execute_index == request_payload_length - 1'b1) begin
                                response_payload_length <= 0;
                                state <= STATE_PREP_CRC;
                            end else begin
                                execute_index <= execute_index + 1'b1;
                            end
                        end
                    end else if (response_opcode == OP_KBD_TX_EVENTS) begin
                        if (execute_index == 16 || !kbd_tx_event_valid) begin
                            response_payload_length <= execute_index;
                            state <= STATE_PREP_CRC;
                        end else begin
                            response_payload_mem[execute_index[3:0]] <= kbd_tx_event_data;
                            execute_index <= execute_index + 1'b1;
                        end
                    end else if (response_opcode == OP_MOUSE_TX_EVENTS) begin
                        if (execute_index == 16 || !mouse_tx_event_valid) begin
                            response_payload_length <= execute_index;
                            state <= STATE_PREP_CRC;
                        end else begin
                            response_payload_mem[execute_index[3:0]] <= mouse_tx_event_data;
                            execute_index <= execute_index + 1'b1;
                        end
                    end else if (response_opcode == OP_KBD_RX_EVENTS) begin
                        if (execute_index == 16 || !kbd_rx_event_valid) begin
                            response_payload_length <= execute_index;
                            state <= STATE_PREP_CRC;
                        end else begin
                            response_payload_mem[execute_index[3:0]] <= kbd_rx_event_data;
                            execute_index <= execute_index + 1'b1;
                        end
                    end else if (response_opcode == OP_MOUSE_RX_EVENTS) begin
                        if (execute_index == 16 || !mouse_rx_event_valid) begin
                            response_payload_length <= execute_index;
                            state <= STATE_PREP_CRC;
                        end else begin
                            response_payload_mem[execute_index[3:0]] <= mouse_rx_event_data;
                            execute_index <= execute_index + 1'b1;
                        end
                    end else if (response_opcode == OP_SCC_ACCESS_EVENTS) begin
                        if (execute_index == 4 || !scc_access_event_valid) begin
                            response_payload_length <= execute_index << 2;
                            state <= STATE_PREP_CRC;
                        end else begin
                            response_payload_mem[{execute_index[1:0], 2'b00}] <=
                                scc_access_event_data[31:24];
                            response_payload_mem[{execute_index[1:0], 2'b00} + 4'd1] <=
                                scc_access_event_data[23:16];
                            response_payload_mem[{execute_index[1:0], 2'b00} + 4'd2] <=
                                scc_access_event_data[15:8];
                            response_payload_mem[{execute_index[1:0], 2'b00} + 4'd3] <=
                                scc_access_event_data[7:0];
                            execute_index <= execute_index + 1'b1;
                        end
                    end
                end

                STATE_CAPTURE_WAIT: begin
                    if (capture_ack_sync[1] == capture_command_toggle) begin
                        if (response_opcode == OP_CAPTURE_INFO) begin
                            response_payload_length <= 16;
                            for (response_index = 0;
                                 response_index < 16;
                                 response_index = response_index + 1)
                                response_payload_mem[response_index] <=
                                    capture_info[127-response_index*8 -: 8];
                        end else if (!capture_word_valid) begin
                            response_status <= 8'h03;
                            response_payload_length <= 0;
                        end else begin
                            response_payload_length <= 16;
                            for (response_index = 0;
                                 response_index < 16;
                                 response_index = response_index + 1)
                                response_payload_mem[response_index] <=
                                    capture_data_byte(
                                        capture_word_data,
                                        {26'd0, capture_chunk, 4'b0000} +
                                        response_index
                                    );
                        end
                        state <= STATE_PREP_CRC;
                    end
                end

                STATE_LG1_TRACE_COMMAND_WAIT: begin
                    if (lg1_trace_command_ack) begin
                        lg1_trace_command_valid <= 1'b0;
                        response_status <= lg1_trace_command_ok ?
                                           8'h00 : 8'h03;
                        response_payload_length <= 0;
                        state <= STATE_PREP_CRC;
                    end else if (lg1_trace_command_ready)
                        // Hold valid through the accepting edge; the trace
                        // acknowledges it on the following cycle.
                        lg1_trace_command_valid <= 1'b1;
                end

                STATE_LG1_TRACE_READ_WAIT: begin
                    if (lg1_trace_read_ack) begin
                        if (!lg1_trace_read_valid) begin
                            response_status <= 8'h03;
                            response_payload_length <= 0;
                        end else begin
                            response_status <= 0;
                            response_payload_length <= 12;
                            for (response_index = 0;
                                 response_index < 12;
                                 response_index = response_index + 1)
                                response_payload_mem[response_index] <=
                                    lg1_trace_read_data[
                                        95-response_index*8 -: 8];
                        end
                        state <= STATE_PREP_CRC;
                    end
                end

                STATE_VIDEO_STATUS_WAIT: begin
                    if (video_status_ack_sync[1] ==
                        video_status_request_toggle) begin
                        response_payload_length <= 16;
                        for (response_index = 0;
                             response_index < 16;
                             response_index = response_index + 1)
                            response_payload_mem[response_index] <=
                                (response_opcode == OP_VIDEO_SIGNATURES) ?
                                video_signature_status[
                                    127-response_index*8 -: 8] :
                                (response_opcode == OP_RESOLVER_STATUS) ?
                                resolver_status[
                                    127-response_index*8 -: 8] :
                                (response_opcode ==
                                 OP_RESOLVER_DETAIL_STATUS) ?
                                resolver_detail_status[
                                    127-response_index*8 -: 8] :
                                (response_opcode ==
                                 OP_VISUAL_STAGE_STATUS) ?
                                visual_stage_status[
                                    127-response_index*8 -: 8] :
                                (response_opcode ==
                                 OP_SCAN_SERVICE_STATUS) ?
                                scan_service_status[
                                    127-response_index*8 -: 8] :
                                (response_opcode ==
                                 OP_SCAN_LATENCY_STATUS) ?
                                scan_latency_status[
                                    127-response_index*8 -: 8] :
                                (response_opcode ==
                                 OP_VIDEO_STAGE_SIGNATURES) ?
                                video_stage_signature_status[
                                    127-response_index*8 -: 8] :
                                video_status[127-response_index*8 -: 8];
                        state <= STATE_PREP_CRC;
                    end
                end

                STATE_PREP_CRC: begin
                    response_crc_work <= 32'hffff_ffff;
                    response_crc_index <= 0;
                    transmit_index <= 0;
                    if (response_payload_length == 0) begin
                        response_crc <= 0;
                        state <= STATE_SEND;
                    end else begin
                        state <= STATE_CALC_CRC;
                    end
                end

                STATE_CALC_CRC: begin
                    response_crc_work <= response_crc_next;
                    if (response_crc_index_ext == response_payload_length - 1'b1) begin
                        response_crc <= ~response_crc_next;
                        transmit_index <= 0;
                        state <= STATE_SEND;
                    end else begin
                        response_crc_index <= response_crc_index + 1'b1;
                    end
                end

                STATE_SEND: begin
                    if (tx_valid && tx_ready) begin
                        if (tx_eof) begin
                            transmit_index <= 0;
                            state <= STATE_IDLE;
                        end else begin
                            transmit_index <= transmit_index + 1'b1;
                        end
                    end
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
