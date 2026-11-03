// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - LG1 independent serial-port scanline fetch
//
// The random/RB side runs in the 50 MHz machine domain and fills one of two
// coherent scanline banks from native 64-byte probes.  The VC1 side reads the
// other port at the dedicated 64 MHz serial clock.  A bundled-data ownership
// toggle publishes a row only after all 16 front and 16 overlay segments are
// resident; no per-pixel coordinate crosses clock domains.

`default_nettype none

module lg1_scanout_line_fetch (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        serial_clk,
    input  wire        serial_rst_n,

    // VC1 serial-domain raster contract.
    input  wire [ 9:0] timing_x,
    input  wire [ 9:0] timing_y,
    input  wire        timing_visible,
    // Machine-domain pause request.  It gates producer issue immediately and
    // is synchronized before it suppresses new serial-domain row requests.
    // A resident row remains readable while fetching is paused.
    input  wire        pause,

    output wire        sample_valid,
    output wire [ 9:0] sample_x,
    output wire [ 9:0] sample_y,
    output wire [ 7:0] sample_front,
    output wire [ 7:0] sample_back,
    output wire [ 1:0] sample_overlay,
    output reg  [15:0] front_response_low_signature,
    output reg  [15:0] front_response_high_signature,
    output reg  [ 7:0] front_response_signature_generation,
    output reg         front_response_signature_valid,
    output reg         underrun,
    output reg  [31:0] underrun_count,
    output reg  [31:0] protocol_error_count,

    // Machine-domain RB snoop/write port.
    input  wire        rb_snoop_valid,
    input  wire [ 1:0] rb_snoop_plane,
    input  wire [17:0] rb_snoop_addr,
    input  wire [31:0] rb_snoop_wdata,
    input  wire [ 3:0] rb_snoop_wstrb,
    output wire        rb_snoop_block,

    // Machine-domain coherent 64-byte probe port.
    output wire        mem_req_valid,
    input  wire        mem_req_ready,
    output wire [15:0] mem_req_addr,
    input  wire        mem_rsp_valid,
    input  wire [511:0] mem_rsp_rdata
);
    localparam [9:0] V_VISIBLE = 10'd768;

    // Each memory is an independent-random-write/serial-read dual-port VRAM
    // analogue.  The parity-selected row bank is the high address bit.
    (* ram_style = "block" *) reg [31:0] front_line_mem [0:511];
    (* ram_style = "block" *) reg [31:0] overlay_line_mem [0:511];

    function [15:0] crc16_byte;
        input [15:0] current;
        input [7:0] value;
        integer bit_i;
        reg [15:0] crc;
        begin
            crc = current ^ {value, 8'h00};
            for (bit_i = 0; bit_i < 8; bit_i = bit_i + 1)
                crc = crc[15] ? ((crc << 1) ^ 16'h1021) : (crc << 1);
            crc16_byte = crc;
        end
    endfunction

    function [15:0] crc16_word;
        input [15:0] current;
        input [31:0] value;
        reg [15:0] crc;
        begin
            crc = crc16_byte(current, value[31:24]);
            crc = crc16_byte(crc, value[23:16]);
            crc = crc16_byte(crc, value[15:8]);
            crc16_word = crc16_byte(crc, value[7:0]);
        end
    endfunction

    function [31:0] gray_to_binary;
        input [31:0] gray;
        integer bit_i;
        reg [31:0] value;
        begin
            value[31] = gray[31];
            for (bit_i = 30; bit_i >= 0; bit_i = bit_i - 1)
                value[bit_i] = value[bit_i + 1] ^ gray[bit_i];
            gray_to_binary = value;
        end
    endfunction

    // Producer-owned completion bundles are declared before the consumer so
    // the CDC intent is explicit even in single-pass Verilog tools.
    reg        row_publish_toggle;
    reg        row_publish_good_q;
    reg        row_publish_buffer_q;
    reg [ 9:0] row_publish_line_q;
    reg        signature_publish_toggle;
    reg [15:0] signature_low_q;
    reg [15:0] signature_high_q;
    reg [ 7:0] signature_generation_q;
    reg [31:0] protocol_error_count_producer;
    wire [31:0] protocol_error_gray =
        protocol_error_count_producer ^
        (protocol_error_count_producer >> 1);

    // ------------------------------------------------------------------
    // Serial consumer and row-request source.
    // ------------------------------------------------------------------
    reg        read_valid;
    reg [ 9:0] read_x;
    reg [ 9:0] read_y;
    reg [ 1:0] read_lane;
    reg [31:0] read_front_word;
    reg [31:0] read_overlay_word;
    reg        serial_even_valid;
    reg        serial_odd_valid;
    reg [ 9:0] serial_even_line;
    reg [ 9:0] serial_odd_line;
    // Stable internal observability names retained for the focused row
    // producer tests.  Their ownership has moved to the serial domain.
    wire       even_valid = serial_even_valid;
    wire       odd_valid = serial_odd_valid;
    wire [9:0] even_line = serial_even_line;
    wire [9:0] odd_line = serial_odd_line;
    reg        row_req_toggle;
    reg        row_req_pending;
    reg        row_req_buffer_q;
    reg [ 9:0] row_req_line_q;
    reg        publish_seen_serial;
    reg        underrun_active;

    (* ASYNC_REG = "TRUE" *) reg publish_sync1_serial;
    (* ASYNC_REG = "TRUE" *) reg publish_sync2_serial;
    (* ASYNC_REG = "TRUE" *) reg signature_sync1_serial;
    (* ASYNC_REG = "TRUE" *) reg signature_sync2_serial;
    (* ASYNC_REG = "TRUE" *) reg pause_sync1_serial;
    (* ASYNC_REG = "TRUE" *) reg pause_sync2_serial;
    (* ASYNC_REG = "TRUE" *) reg [31:0] protocol_gray_sync1_serial;
    (* ASYNC_REG = "TRUE" *) reg [31:0] protocol_gray_sync2_serial;
    reg signature_seen_serial;

    wire current_buffer_serial = timing_y[0];
    wire current_valid_serial = current_buffer_serial ?
        (serial_odd_valid && (serial_odd_line == timing_y)) :
        (serial_even_valid && (serial_even_line == timing_y));
    wire [9:0] target_line_serial = (timing_y < V_VISIBLE) ?
        ((timing_y == (V_VISIBLE - 10'd1)) ? 10'd0 : timing_y + 10'd1) :
        10'd0;
    wire target_buffer_serial = target_line_serial[0];
    wire target_valid_serial = target_buffer_serial ?
        (serial_odd_valid && (serial_odd_line == target_line_serial)) :
        (serial_even_valid && (serial_even_line == target_line_serial));
    wire publish_event_serial =
        publish_sync2_serial != publish_seen_serial;
    wire [8:0] line_read_addr = {
        current_buffer_serial, timing_x[9:2]
    };

    assign sample_valid = read_valid;
    assign sample_x = read_x;
    assign sample_y = read_y;
    assign sample_front =
        (read_lane == 2'd0) ? read_front_word[31:24] :
        (read_lane == 2'd1) ? read_front_word[23:16] :
        (read_lane == 2'd2) ? read_front_word[15:8] :
                              read_front_word[7:0];
    assign sample_back = 8'h00;
    assign sample_overlay =
        (read_lane == 2'd0) ? read_overlay_word[25:24] :
        (read_lane == 2'd1) ? read_overlay_word[17:16] :
        (read_lane == 2'd2) ? read_overlay_word[9:8] :
                              read_overlay_word[1:0];

    // Canonical independent-clock simple-dual-port read template.
    always @(posedge serial_clk) begin
        read_x <= timing_x;
        read_y <= timing_y;
        read_lane <= timing_x[1:0];
        read_front_word <= front_line_mem[line_read_addr];
        read_overlay_word <= overlay_line_mem[line_read_addr];
    end

    always @(posedge serial_clk or negedge serial_rst_n) begin
        if (!serial_rst_n) begin
            read_valid <= 1'b0;
            serial_even_valid <= 1'b0;
            serial_odd_valid <= 1'b0;
            serial_even_line <= 10'd0;
            serial_odd_line <= 10'd0;
            row_req_toggle <= 1'b0;
            row_req_pending <= 1'b0;
            row_req_buffer_q <= 1'b0;
            row_req_line_q <= 10'd0;
            publish_sync1_serial <= 1'b0;
            publish_sync2_serial <= 1'b0;
            publish_seen_serial <= 1'b0;
            signature_sync1_serial <= 1'b0;
            signature_sync2_serial <= 1'b0;
            signature_seen_serial <= 1'b0;
            pause_sync1_serial <= 1'b0;
            pause_sync2_serial <= 1'b0;
            protocol_gray_sync1_serial <= 32'd0;
            protocol_gray_sync2_serial <= 32'd0;
            front_response_low_signature <= 16'd0;
            front_response_high_signature <= 16'd0;
            front_response_signature_generation <= 8'd0;
            front_response_signature_valid <= 1'b0;
            underrun <= 1'b0;
            underrun_count <= 32'd0;
            underrun_active <= 1'b0;
            protocol_error_count <= 32'd0;
        end else begin
            publish_sync1_serial <= row_publish_toggle;
            publish_sync2_serial <= publish_sync1_serial;
            signature_sync1_serial <= signature_publish_toggle;
            signature_sync2_serial <= signature_sync1_serial;
            pause_sync1_serial <= pause;
            pause_sync2_serial <= pause_sync1_serial;
            protocol_gray_sync1_serial <= protocol_error_gray;
            protocol_gray_sync2_serial <= protocol_gray_sync1_serial;
            protocol_error_count <=
                gray_to_binary(protocol_gray_sync2_serial);

            read_valid <= timing_visible && current_valid_serial;
            if (timing_visible && !current_valid_serial) begin
                underrun <= 1'b1;
                if (!underrun_active)
                    underrun_count <= underrun_count + 32'd1;
                underrun_active <= 1'b1;
            end else begin
                underrun_active <= 1'b0;
            end

            if (publish_event_serial) begin
                publish_seen_serial <= publish_sync2_serial;
                row_req_pending <= 1'b0;
                if (row_publish_buffer_q) begin
                    serial_odd_valid <= row_publish_good_q;
                    serial_odd_line <= row_publish_line_q;
                end else begin
                    serial_even_valid <= row_publish_good_q;
                    serial_even_line <= row_publish_line_q;
                end
            end

            // Permit a few serial clocks after x=0 so a completion arriving
            // exactly on the line boundary can retire before the next request.
            if (!pause_sync2_serial && (timing_x < 10'd4) &&
                (!row_req_pending || publish_event_serial) &&
                !target_valid_serial &&
                !(publish_event_serial && row_publish_good_q &&
                  (row_publish_line_q == target_line_serial))) begin
                row_req_buffer_q <= target_buffer_serial;
                row_req_line_q <= target_line_serial;
                row_req_toggle <= ~row_req_toggle;
                row_req_pending <= 1'b1;
                if (target_buffer_serial)
                    serial_odd_valid <= 1'b0;
                else
                    serial_even_valid <= 1'b0;
            end

            if (signature_sync2_serial != signature_seen_serial) begin
                signature_seen_serial <= signature_sync2_serial;
                front_response_low_signature <= signature_low_q;
                front_response_high_signature <= signature_high_q;
                front_response_signature_generation <= signature_generation_q;
                front_response_signature_valid <= 1'b1;
            end
        end
    end

    // ------------------------------------------------------------------
    // 50 MHz producer, coherent probe gearbox, and RB write snoop.
    // ------------------------------------------------------------------
    reg        producer_even_valid;
    reg        producer_odd_valid;
    reg [ 9:0] producer_even_line;
    reg [ 9:0] producer_odd_line;
    reg        fetch_active;
    reg        fetch_buffer;
    reg [ 9:0] fetch_line;
    reg        fetch_request_toggle;
    reg [ 1:0] issue_plane;
    reg [ 3:0] issue_segment;
    reg        request_pending;
    reg        unpack_active;
    reg [ 3:0] unpack_index;
    reg [511:0] unpack_data;
    reg [ 2:0] unpack_snoop_cooldown;
    reg [31:0] received_segments;
    reg        row_req_seen_producer;
    (* ASYNC_REG = "TRUE" *) reg row_req_sync1_producer;
    (* ASYNC_REG = "TRUE" *) reg row_req_sync2_producer;

    reg [15:0] front_response_low_work;
    reg [15:0] front_response_high_work;
    reg        front_response_frame_active;
    reg        front_response_frame_good;
    reg [ 9:0] front_response_expected_line;
    reg [ 7:0] front_response_expected_word;
    reg [ 7:0] signature_generation_work;

    assign mem_req_valid = !pause && fetch_active && !request_pending &&
                           !unpack_active;
    assign mem_req_addr = {issue_plane, fetch_line, issue_segment};
    assign rb_snoop_block = mem_rsp_valid ||
                            (unpack_active &&
                             (unpack_snoop_cooldown != 3'd0));

    wire req_fire = mem_req_valid && mem_req_ready;
    wire rsp_fire = mem_rsp_valid && request_pending;
    wire [7:0] unpack_word = {issue_segment, unpack_index};
    wire unpack_front = unpack_active && (issue_plane == 2'd0);
    wire unpack_overlay = unpack_active && (issue_plane == 2'd2);
    wire unpack_last_word = unpack_active && (unpack_index == 4'hf);
    wire unpack_last_segment = unpack_last_word &&
        (issue_plane == 2'd2) && (issue_segment == 4'hf);
    wire [31:0] unpack_word_data =
        unpack_data[511 - (unpack_index * 32) -: 32];
    wire [8:0] line_write_addr = {
        fetch_buffer, issue_segment, unpack_index
    };
    wire [9:0] rb_snoop_line = rb_snoop_addr[17:8];
    wire rb_snoop_buffer = rb_snoop_addr[8];
    wire [8:0] rb_snoop_write_addr = {
        rb_snoop_buffer, rb_snoop_addr[7:0]
    };
    wire rb_snoop_resident = rb_snoop_buffer ?
        (producer_odd_valid && (producer_odd_line == rb_snoop_line)) :
        (producer_even_valid && (producer_even_line == rb_snoop_line));
    wire rb_snoop_fill = fetch_active && (rb_snoop_line == fetch_line);
    wire rb_snoop_front = rb_snoop_valid &&
                          (rb_snoop_resident || rb_snoop_fill) &&
                          (rb_snoop_plane == 2'd0);
    wire rb_snoop_overlay = rb_snoop_valid &&
                            (rb_snoop_resident || rb_snoop_fill) &&
                            (rb_snoop_plane == 2'd2);
    wire unpack_write_collision =
        (unpack_front && rb_snoop_front) ||
        (unpack_overlay && rb_snoop_overlay);
    wire unpack_fire = unpack_active && !unpack_write_collision;
    wire front_write = rb_snoop_front || unpack_front;
    wire [8:0] front_write_addr = rb_snoop_front ?
        rb_snoop_write_addr : line_write_addr;
    wire [31:0] front_write_data = rb_snoop_front ?
        rb_snoop_wdata : unpack_word_data;
    wire [3:0] front_write_strobe = rb_snoop_front ?
        rb_snoop_wstrb : 4'hf;
    wire [3:0] front_write_bram_strobe = {
        front_write_strobe[0], front_write_strobe[1],
        front_write_strobe[2], front_write_strobe[3]
    };
    wire overlay_write = rb_snoop_overlay || unpack_overlay;
    wire [8:0] overlay_write_addr = rb_snoop_overlay ?
        rb_snoop_write_addr : line_write_addr;
    wire [31:0] overlay_write_data = rb_snoop_overlay ?
        rb_snoop_wdata : unpack_word_data;
    wire [3:0] overlay_write_strobe = rb_snoop_overlay ?
        rb_snoop_wstrb : 4'hf;
    wire [3:0] overlay_write_bram_strobe = {
        overlay_write_strobe[0], overlay_write_strobe[1],
        overlay_write_strobe[2], overlay_write_strobe[3]
    };
    wire front_response_fire = unpack_fire && (issue_plane == 2'd0);
    wire front_response_frame_start = front_response_fire &&
        (fetch_line == 10'd0) && (unpack_word == 8'd0);
    wire [15:0] front_response_selected_work = unpack_word[1] ?
        front_response_high_work : front_response_low_work;
    wire [15:0] front_response_signature_next = crc16_word(
        front_response_frame_start ? 16'hffff :
                                     front_response_selected_work,
        unpack_word_data);

    // Canonical independent-clock byte-write port.
    integer front_byte;
    always @(posedge clk) begin
        for (front_byte = 0; front_byte < 4; front_byte = front_byte + 1)
            if (front_write && front_write_bram_strobe[front_byte])
                front_line_mem[front_write_addr][front_byte*8 +: 8] <=
                    front_write_data[front_byte*8 +: 8];
    end

    integer overlay_byte;
    always @(posedge clk) begin
        for (overlay_byte = 0; overlay_byte < 4;
             overlay_byte = overlay_byte + 1)
            if (overlay_write && overlay_write_bram_strobe[overlay_byte])
                overlay_line_mem[overlay_write_addr][overlay_byte*8 +: 8] <=
                    overlay_write_data[overlay_byte*8 +: 8];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            producer_even_valid <= 1'b0;
            producer_odd_valid <= 1'b0;
            producer_even_line <= 10'd0;
            producer_odd_line <= 10'd0;
            fetch_active <= 1'b0;
            fetch_buffer <= 1'b0;
            fetch_line <= 10'd0;
            fetch_request_toggle <= 1'b0;
            issue_plane <= 2'd0;
            issue_segment <= 4'd0;
            request_pending <= 1'b0;
            unpack_active <= 1'b0;
            unpack_index <= 4'd0;
            unpack_data <= 512'd0;
            unpack_snoop_cooldown <= 3'd0;
            received_segments <= 32'd0;
            row_req_sync1_producer <= 1'b0;
            row_req_sync2_producer <= 1'b0;
            row_req_seen_producer <= 1'b0;
            row_publish_toggle <= 1'b0;
            row_publish_good_q <= 1'b0;
            row_publish_buffer_q <= 1'b0;
            row_publish_line_q <= 10'd0;
            front_response_low_work <= 16'hffff;
            front_response_high_work <= 16'hffff;
            front_response_frame_active <= 1'b0;
            front_response_frame_good <= 1'b0;
            front_response_expected_line <= 10'd0;
            front_response_expected_word <= 8'd0;
            signature_generation_work <= 8'd0;
            signature_publish_toggle <= 1'b0;
            signature_low_q <= 16'd0;
            signature_high_q <= 16'd0;
            signature_generation_q <= 8'd0;
            protocol_error_count_producer <= 32'd0;
        end else begin
            row_req_sync1_producer <= row_req_toggle;
            row_req_sync2_producer <= row_req_sync1_producer;

            if (mem_rsp_valid && !request_pending)
                protocol_error_count_producer <=
                    protocol_error_count_producer + 32'd1;

            if (!pause && !fetch_active &&
                (row_req_sync2_producer != row_req_seen_producer)) begin
                row_req_seen_producer <= row_req_sync2_producer;
                fetch_request_toggle <= row_req_sync2_producer;
                fetch_active <= 1'b1;
                fetch_buffer <= row_req_buffer_q;
                fetch_line <= row_req_line_q;
                issue_plane <= 2'd0;
                issue_segment <= 4'd0;
                request_pending <= 1'b0;
                unpack_active <= 1'b0;
                unpack_index <= 4'd0;
                unpack_snoop_cooldown <= 3'd0;
                received_segments <= 32'd0;
                if (row_req_buffer_q)
                    producer_odd_valid <= 1'b0;
                else
                    producer_even_valid <= 1'b0;
            end

            if (req_fire)
                request_pending <= 1'b1;

            if (rsp_fire) begin
                request_pending <= 1'b0;
                unpack_active <= 1'b1;
                unpack_index <= 4'd0;
                unpack_data <= mem_rsp_rdata;
                unpack_snoop_cooldown <= 3'd0;
            end

            if (rb_snoop_valid && rb_snoop_fill && unpack_active &&
                (rb_snoop_plane == issue_plane) &&
                (rb_snoop_addr[7:4] == issue_segment)) begin : patch_unpack
                integer patch_byte;
                for (patch_byte = 0; patch_byte < 4;
                     patch_byte = patch_byte + 1)
                    if (rb_snoop_wstrb[patch_byte])
                        unpack_data[
                            511 - (rb_snoop_addr[3:0] * 32) -
                            (patch_byte * 8) -: 8
                        ] <= rb_snoop_wdata[31 - (patch_byte * 8) -: 8];
            end

            if (unpack_write_collision)
                unpack_snoop_cooldown <= 3'd7;

            if (front_response_frame_start) begin
                front_response_low_work <=
                    front_response_signature_next;
                front_response_high_work <= 16'hffff;
                front_response_frame_active <= 1'b1;
                front_response_frame_good <= 1'b1;
                front_response_expected_line <= 10'd0;
                front_response_expected_word <= 8'd1;
            end else if (front_response_fire &&
                         front_response_frame_active) begin
                if ((fetch_line != front_response_expected_line) ||
                    (unpack_word != front_response_expected_word))
                    front_response_frame_good <= 1'b0;
                if (unpack_word[1])
                    front_response_high_work <=
                        front_response_signature_next;
                else
                    front_response_low_work <=
                        front_response_signature_next;
                if (unpack_word == 8'hff) begin
                    front_response_expected_line <= fetch_line + 10'd1;
                    front_response_expected_word <= 8'd0;
                end else begin
                    front_response_expected_word <= unpack_word + 8'd1;
                end
                if ((fetch_line == (V_VISIBLE - 10'd1)) &&
                    (unpack_word == 8'hff)) begin
                    front_response_frame_active <= 1'b0;
                    if (front_response_frame_good &&
                        (front_response_expected_line ==
                         (V_VISIBLE - 10'd1)) &&
                        (front_response_expected_word == 8'hff)) begin
                        signature_generation_work <=
                            signature_generation_work + 8'd1;
                        signature_low_q <= front_response_low_work;
                        signature_high_q <=
                            front_response_signature_next;
                        signature_generation_q <=
                            signature_generation_work + 8'd1;
                        signature_publish_toggle <=
                            ~signature_publish_toggle;
                    end
                end
            end

            if (unpack_fire) begin
                if (unpack_last_word) begin
                    received_segments[
                        {(issue_plane == 2'd2), issue_segment}
                    ] <= 1'b1;
                    unpack_active <= 1'b0;
                    unpack_snoop_cooldown <= 3'd0;
                    if (unpack_last_segment) begin
                        fetch_active <= 1'b0;
                        row_publish_buffer_q <= fetch_buffer;
                        row_publish_line_q <= fetch_line;
                        row_publish_toggle <= fetch_request_toggle;
                        if (&received_segments[30:0]) begin
                            row_publish_good_q <= 1'b1;
                            if (fetch_buffer) begin
                                producer_odd_valid <= 1'b1;
                                producer_odd_line <= fetch_line;
                            end else begin
                                producer_even_valid <= 1'b1;
                                producer_even_line <= fetch_line;
                            end
                        end else begin
                            row_publish_good_q <= 1'b0;
                            protocol_error_count_producer <=
                                protocol_error_count_producer + 32'd1;
                        end
                    end else if (issue_segment == 4'hf) begin
                        issue_plane <= 2'd2;
                        issue_segment <= 4'd0;
                    end else begin
                        issue_segment <= issue_segment + 4'd1;
                    end
                end else begin
                    unpack_index <= unpack_index + 4'd1;
                    if (unpack_snoop_cooldown != 3'd0)
                        unpack_snoop_cooldown <=
                            unpack_snoop_cooldown - 3'd1;
                end
            end
        end
    end
endmodule

`default_nettype wire
