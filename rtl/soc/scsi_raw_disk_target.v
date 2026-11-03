// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - raw direct-access SCSI target front-end.
//
// This module sits behind wd33c93's synthesizable backend interface and turns
// IP20 disk CDBs into single-sector block requests. It intentionally presents
// a raw disk: the SD card contents are an SGI disk image written 1:1.

`default_nettype none

module scsi_raw_disk_target #(
    parameter [31:0] DISK_SECTORS = 32'd4_194_304, // 2 GiB default
    parameter READ_ONLY = 0
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        scsi_bus_reset,
    input  wire        media_ready,
    input  wire [31:0] media_sectors,

    input  wire        scsi_cmd_pending,
    output reg         scsi_cmd_done,
    output reg  [7:0]  scsi_cmd_status,
    output reg  [23:0] scsi_resp_len,
    output reg         scsi_status_valid,
    input  wire [2:0]  scsi_target_id,
    input  wire [95:0] scsi_cdb,
    input  wire [23:0] scsi_xfer_count,
    input  wire [23:0] scsi_data_index,
    output wire [7:0]  scsi_read_data,
    output wire        scsi_read_valid,
    input  wire        scsi_read_consume,
    input  wire        scsi_write_valid,
    input  wire [7:0]  scsi_write_data,
    output wire        scsi_write_ready,
    input  wire        scsi_data_out_done,

    output reg         block_req_valid,
    output reg         block_req_write,
    output reg  [31:0] block_req_lba,
    input  wire        block_req_ready,
    input  wire        block_done,
    input  wire        block_error,

    output reg  [8:0]  buf_addr,
    input  wire [7:0]  buf_rdata,
    output reg         buf_we,
    output reg  [7:0]  buf_wdata,

    output wire [255:0] debug_probe
);

    localparam [7:0] ST_SATOK = 8'h16;
    localparam [7:0] ST_ERROR = 8'h42;

    localparam [7:0] OP_TEST_UNIT_READY = 8'h00;
    localparam [7:0] OP_REZERO_UNIT     = 8'h01;
    localparam [7:0] OP_REQUEST_SENSE   = 8'h03;
    localparam [7:0] OP_READ_6          = 8'h08;
    localparam [7:0] OP_WRITE_6         = 8'h0a;
    localparam [7:0] OP_INQUIRY         = 8'h12;
    localparam [7:0] OP_RESERVE_6       = 8'h16;
    localparam [7:0] OP_RELEASE_6       = 8'h17;
    localparam [7:0] OP_MODE_SENSE_6    = 8'h1a;
    localparam [7:0] OP_START_STOP_UNIT = 8'h1b;
    localparam [7:0] OP_PREVENT_ALLOW   = 8'h1e;
    localparam [7:0] OP_READ_CAP_10     = 8'h25;
    localparam [7:0] OP_READ_10         = 8'h28;
    localparam [7:0] OP_WRITE_10        = 8'h2a;
    localparam [7:0] OP_VERIFY_10       = 8'h2f;
    localparam [7:0] OP_SYNC_CACHE_10   = 8'h35;

    localparam [3:0]
        T_IDLE            = 4'd0,
        T_WAIT_DROP       = 4'd1,
        T_READ_REQ        = 4'd2,
        T_READ_WAIT       = 4'd3,
        T_READ_STREAM     = 4'd4,
        T_WRITE_STREAM    = 4'd5,
        T_WRITE_REQ       = 4'd6,
        T_WRITE_WAIT      = 4'd7,
        T_STATUS_HELD     = 4'd8;

    reg [3:0]  state;
    reg [7:0]  resp_mem [0:255];
    reg [7:0]  sense_key;
    reg [7:0]  sense_asc;
    reg [7:0]  sense_ascq;
    reg [31:0] base_lba;
    reg [15:0] block_count;
    reg [31:0] loaded_lba;
    reg        loaded_valid;
    reg        active_response;
    reg        active_read;
    reg        read_command_accepted;
    reg        active_write;
    reg        final_write_block;
    reg        response_stream_seen;
    reg        read_stream_seen;
    reg [7:0]  last_opcode;
    reg [7:0]  last_error;
    reg [2:0]  last_target_id;
    reg [23:0] last_xfer_count;

    wire [7:0] cdb0  = scsi_cdb[95:88];
    wire [7:0] cdb1  = scsi_cdb[87:80];
    wire [7:0] cdb2  = scsi_cdb[79:72];
    wire [7:0] cdb3  = scsi_cdb[71:64];
    wire [7:0] cdb4  = scsi_cdb[63:56];
    wire [7:0] cdb5  = scsi_cdb[55:48];
    wire [7:0] cdb7  = scsi_cdb[39:32];
    wire [7:0] cdb8  = scsi_cdb[31:24];

    wire [31:0] capacity_sectors =
        (media_sectors != 32'd0) ? media_sectors : DISK_SECTORS;
    wire [31:0] last_lba =
        (capacity_sectors == 32'd0) ? 32'd0 : (capacity_sectors - 32'd1);

    wire [7:0] resp_data = resp_mem[scsi_data_index[7:0]];
    wire [31:0] desired_lba = base_lba + {17'd0, scsi_data_index[23:9]};
    wire        data_is_sector =
        active_read && (state == T_READ_STREAM) && loaded_valid &&
        (desired_lba == loaded_lba) && (scsi_data_index < scsi_resp_len);

    assign scsi_read_valid = active_read ? data_is_sector :
                             (active_response && scsi_data_index < scsi_resp_len);
    assign scsi_read_data = active_read ? buf_rdata : resp_data;
    assign scsi_write_ready = active_write && (state == T_WRITE_STREAM);

    wire [31:0] decoded_lba_6 =
        {11'd0, cdb1[4:0], cdb2, cdb3};
    wire [15:0] decoded_count_6 =
        (cdb4 == 8'd0) ? 16'd256 : {8'd0, cdb4};
    wire [31:0] decoded_lba_10 =
        {cdb2, cdb3, cdb4, cdb5};
    wire [15:0] decoded_count_10 =
        {cdb7, cdb8};

    wire [31:0] decoded_lba =
        (cdb0 == OP_READ_6 || cdb0 == OP_WRITE_6) ? decoded_lba_6 :
                                                     decoded_lba_10;
    wire [15:0] decoded_count =
        (cdb0 == OP_READ_6 || cdb0 == OP_WRITE_6) ? decoded_count_6 :
                                                     decoded_count_10;
    wire [23:0] decoded_bytes =
        (decoded_count > 16'd32767) ? 24'hff_fe00 :
                                      {decoded_count[14:0], 9'b0};
    wire [23:0] blocks_to_bytes =
        (block_count > 16'd32767) ? 24'hff_fe00 :
                                    {block_count[14:0], 9'b0};
    wire decoded_range_ok =
        (decoded_count == 16'd0) ||
        ((decoded_lba < capacity_sectors) &&
         ({16'd0, decoded_count} <= (capacity_sectors - decoded_lba)));

    task set_sense_good;
    begin
        sense_key <= 8'h00;
        sense_asc <= 8'h00;
        sense_ascq <= 8'h00;
        last_error <= 8'h00;
    end
    endtask

    task set_sense_not_ready;
    begin
        sense_key <= 8'h02;
        sense_asc <= 8'h3a;  // medium not present
        sense_ascq <= 8'h00;
    end
    endtask

    task set_sense_illegal;
    begin
        sense_key <= 8'h05;
        sense_asc <= 8'h20;  // invalid command operation code
        sense_ascq <= 8'h00;
    end
    endtask

    task set_sense_lba;
    begin
        sense_key <= 8'h05;
        sense_asc <= 8'h21;  // logical block address out of range
        sense_ascq <= 8'h00;
    end
    endtask

    task set_sense_medium_error;
    begin
        sense_key <= 8'h03;
        sense_asc <= 8'h11;  // unrecovered read/write error
        sense_ascq <= 8'h00;
    end
    endtask

    task finish_no_data;
        input [7:0] status;
    begin
        scsi_resp_len <= 24'd0;
        scsi_cmd_status <= status;
        scsi_cmd_done <= 1'b1;
        scsi_status_valid <= 1'b1;
        active_read <= 1'b0;
        active_response <= 1'b0;
        active_write <= 1'b0;
        response_stream_seen <= 1'b0;
        read_stream_seen <= 1'b0;
        state <= T_WAIT_DROP;
    end
    endtask

    task finish_response;
        input [23:0] len;
    begin
        scsi_resp_len <= len;
        scsi_cmd_status <= ST_SATOK;
        scsi_cmd_done <= 1'b1;
        scsi_status_valid <= 1'b1;
        active_read <= 1'b0;
        active_response <= 1'b1;
        active_write <= 1'b0;
        response_stream_seen <= 1'b0;
        read_stream_seen <= 1'b0;
        state <= T_WAIT_DROP;
    end
    endtask

    task fill_inquiry;
        input [7:0] alloc_len;
        reg [7:0] len;
    begin
        len = (alloc_len < 8'd36) ? alloc_len : 8'd36;
        resp_mem[0]  <= 8'h00; // direct-access block device
        resp_mem[1]  <= 8'h00;
        resp_mem[2]  <= 8'h02;
        resp_mem[3]  <= 8'h02;
        resp_mem[4]  <= 8'd31;
        resp_mem[5]  <= 8'h00;
        resp_mem[6]  <= 8'h00;
        resp_mem[7]  <= 8'h00;
        resp_mem[8]  <= "S"; resp_mem[9]  <= "G";
        resp_mem[10] <= "I"; resp_mem[11] <= " ";
        resp_mem[12] <= " "; resp_mem[13] <= " ";
        resp_mem[14] <= " "; resp_mem[15] <= " ";
        resp_mem[16] <= "S"; resp_mem[17] <= "D";
        resp_mem[18] <= " "; resp_mem[19] <= "S";
        resp_mem[20] <= "P"; resp_mem[21] <= "I";
        resp_mem[22] <= " "; resp_mem[23] <= "D";
        resp_mem[24] <= "I"; resp_mem[25] <= "S";
        resp_mem[26] <= "K"; resp_mem[27] <= " ";
        resp_mem[28] <= " "; resp_mem[29] <= " ";
        resp_mem[30] <= " "; resp_mem[31] <= " ";
        resp_mem[32] <= "0"; resp_mem[33] <= "0";
        resp_mem[34] <= "1"; resp_mem[35] <= " ";
        finish_response({16'd0, len});
    end
    endtask

    task fill_request_sense;
        input [7:0] alloc_len;
        reg [7:0] len;
    begin
        len = (alloc_len < 8'd18) ? alloc_len : 8'd18;
        resp_mem[0]  <= 8'h70;
        resp_mem[1]  <= 8'h00;
        resp_mem[2]  <= sense_key;
        resp_mem[3]  <= 8'h00;
        resp_mem[4]  <= 8'h00;
        resp_mem[5]  <= 8'h00;
        resp_mem[6]  <= 8'h00;
        resp_mem[7]  <= 8'd10;
        resp_mem[8]  <= 8'h00;
        resp_mem[9]  <= 8'h00;
        resp_mem[10] <= 8'h00;
        resp_mem[11] <= 8'h00;
        resp_mem[12] <= sense_asc;
        resp_mem[13] <= sense_ascq;
        resp_mem[14] <= 8'h00;
        resp_mem[15] <= 8'h00;
        resp_mem[16] <= 8'h00;
        resp_mem[17] <= 8'h00;
        finish_response({16'd0, len});
    end
    endtask

    task fill_read_capacity;
        reg [31:0] max_lba;
    begin
        max_lba = last_lba;
        resp_mem[0] <= max_lba[31:24];
        resp_mem[1] <= max_lba[23:16];
        resp_mem[2] <= max_lba[15:8];
        resp_mem[3] <= max_lba[7:0];
        resp_mem[4] <= 8'h00;
        resp_mem[5] <= 8'h00;
        resp_mem[6] <= 8'h02;
        resp_mem[7] <= 8'h00;
        finish_response(24'd8);
    end
    endtask

    task fill_mode_sense_6;
        input [7:0] alloc_len;
        reg [7:0] len;
        reg [31:0] blocks24;
    begin
        len = (alloc_len < 8'd12) ? alloc_len : 8'd12;
        blocks24 = (capacity_sectors > 32'h00ff_ffff) ?
                   32'h00ff_ffff : capacity_sectors;
        resp_mem[0]  <= 8'd11;
        resp_mem[1]  <= 8'h00;
        resp_mem[2]  <= READ_ONLY ? 8'h80 : 8'h00;
        resp_mem[3]  <= 8'h08;
        resp_mem[4]  <= 8'h00;
        resp_mem[5]  <= blocks24[23:16];
        resp_mem[6]  <= blocks24[15:8];
        resp_mem[7]  <= blocks24[7:0];
        resp_mem[8]  <= 8'h00;
        resp_mem[9]  <= 8'h00;
        resp_mem[10] <= 8'h02;
        resp_mem[11] <= 8'h00;
        finish_response({16'd0, len});
    end
    endtask

    always @(*) begin
        buf_addr = active_read ? scsi_data_index[8:0] : scsi_data_index[8:0];
        buf_wdata = scsi_write_data;
        buf_we = scsi_write_valid && active_write && (state == T_WRITE_STREAM);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= T_IDLE;
            scsi_cmd_done <= 1'b0;
            scsi_cmd_status <= ST_SATOK;
            scsi_resp_len <= 24'd0;
            scsi_status_valid <= 1'b0;
            block_req_valid <= 1'b0;
            block_req_write <= 1'b0;
            block_req_lba <= 32'd0;
            sense_key <= 8'h00;
            sense_asc <= 8'h00;
            sense_ascq <= 8'h00;
            base_lba <= 32'd0;
            block_count <= 16'd0;
            loaded_lba <= 32'd0;
            loaded_valid <= 1'b0;
            active_read <= 1'b0;
            active_response <= 1'b0;
            active_write <= 1'b0;
            read_command_accepted <= 1'b0;
            final_write_block <= 1'b0;
            response_stream_seen <= 1'b0;
            read_stream_seen <= 1'b0;
            last_opcode <= 8'h00;
            last_error <= 8'h00;
            last_target_id <= 3'd0;
            last_xfer_count <= 24'd0;
        end else if (scsi_bus_reset) begin
            state <= T_IDLE;
            scsi_cmd_done <= 1'b0;
            scsi_cmd_status <= ST_SATOK;
            scsi_resp_len <= 24'd0;
            scsi_status_valid <= 1'b0;
            block_req_valid <= 1'b0;
            block_req_write <= 1'b0;
            block_req_lba <= 32'd0;
            base_lba <= 32'd0;
            block_count <= 16'd0;
            loaded_lba <= 32'd0;
            loaded_valid <= 1'b0;
            active_read <= 1'b0;
            active_response <= 1'b0;
            active_write <= 1'b0;
            read_command_accepted <= 1'b0;
            final_write_block <= 1'b0;
            response_stream_seen <= 1'b0;
            read_stream_seen <= 1'b0;
            last_opcode <= 8'h00;
            last_error <= 8'h00;
            last_target_id <= 3'd0;
            last_xfer_count <= 24'd0;
        end else begin
            scsi_cmd_done <= 1'b0;
            block_req_valid <= 1'b0;

            case (state)
                T_IDLE: begin
                    scsi_status_valid <= 1'b0;
                    loaded_valid <= 1'b0;
                    active_read <= 1'b0;
                    active_response <= 1'b0;
                    active_write <= 1'b0;
                    read_command_accepted <= 1'b0;
                    final_write_block <= 1'b0;
                    response_stream_seen <= 1'b0;
                    read_stream_seen <= 1'b0;
                    if (scsi_cmd_pending) begin
                        last_opcode <= cdb0;
                        last_target_id <= scsi_target_id;
                        last_xfer_count <= scsi_xfer_count;
                        scsi_cmd_status <= ST_SATOK;
                        case (cdb0)
                            OP_TEST_UNIT_READY: begin
                                if (media_ready) begin
                                    set_sense_good();
                                    finish_no_data(ST_SATOK);
                                end else begin
                                    set_sense_not_ready();
                                    last_error <= 8'h01;
                                    finish_no_data(ST_ERROR);
                                end
                            end
                            OP_INQUIRY: begin
                                set_sense_good();
                                fill_inquiry(cdb4);
                            end
                            OP_REQUEST_SENSE: begin
                                fill_request_sense(cdb4);
                            end
                            OP_READ_CAP_10: begin
                                if (media_ready) begin
                                    set_sense_good();
                                    fill_read_capacity();
                                end else begin
                                    set_sense_not_ready();
                                    last_error <= 8'h01;
                                    finish_no_data(ST_ERROR);
                                end
                            end
                            OP_MODE_SENSE_6: begin
                                set_sense_good();
                                fill_mode_sense_6(cdb4);
                            end
                            OP_REZERO_UNIT,
                            OP_RESERVE_6,
                            OP_RELEASE_6,
                            OP_START_STOP_UNIT,
                            OP_PREVENT_ALLOW,
                            OP_VERIFY_10,
                            OP_SYNC_CACHE_10: begin
                                set_sense_good();
                                finish_no_data(ST_SATOK);
                            end
                            OP_READ_6, OP_READ_10: begin
                                if (!media_ready) begin
                                    set_sense_not_ready();
                                    last_error <= 8'h01;
                                    finish_no_data(ST_ERROR);
                                end else if (!decoded_range_ok) begin
                                    set_sense_lba();
                                    last_error <= 8'h02;
                                    finish_no_data(ST_ERROR);
                                end else if (decoded_count == 16'd0) begin
                                    set_sense_good();
                                    finish_no_data(ST_SATOK);
                                end else begin
                                    set_sense_good();
                                    base_lba <= decoded_lba;
                                    block_count <= decoded_count;
                                    scsi_resp_len <= decoded_bytes;
                                    scsi_cmd_status <= ST_SATOK;
                                    scsi_status_valid <= 1'b1;
                                    active_read <= 1'b1;
                                    read_stream_seen <= 1'b0;
                                    block_req_write <= 1'b0;
                                    block_req_lba <= decoded_lba;
                                    state <= T_READ_REQ;
                                end
                            end
                            OP_WRITE_6, OP_WRITE_10: begin
                                if (READ_ONLY) begin
                                    sense_key <= 8'h07;
                                    sense_asc <= 8'h27;
                                    sense_ascq <= 8'h00;
                                    last_error <= 8'h03;
                                    finish_no_data(ST_ERROR);
                                end else if (!media_ready) begin
                                    set_sense_not_ready();
                                    last_error <= 8'h01;
                                    finish_no_data(ST_ERROR);
                                end else if (!decoded_range_ok) begin
                                    set_sense_lba();
                                    last_error <= 8'h02;
                                    finish_no_data(ST_ERROR);
                                end else if (decoded_count == 16'd0) begin
                                    set_sense_good();
                                    finish_no_data(ST_SATOK);
                                end else begin
                                    set_sense_good();
                                    base_lba <= decoded_lba;
                                    block_count <= decoded_count;
                                    scsi_resp_len <= decoded_bytes;
                                    scsi_cmd_status <= ST_SATOK;
                                    scsi_status_valid <= 1'b0;
                                    active_write <= 1'b1;
                                    scsi_cmd_done <= 1'b1;
                                    state <= T_WRITE_STREAM;
                                end
                            end
                            default: begin
                                set_sense_illegal();
                                last_error <= cdb0;
                                finish_no_data(ST_ERROR);
                            end
                        endcase
                    end
                end

                T_WAIT_DROP: begin
                    if (scsi_read_consume)
                        response_stream_seen <= 1'b1;
                    if (active_response && response_stream_seen &&
                        scsi_data_index >= scsi_resp_len)
                        active_response <= 1'b0;
                    if (!scsi_cmd_pending &&
                        (!active_response ||
                         (response_stream_seen &&
                          scsi_data_index >= scsi_resp_len)))
                        state <= T_IDLE;
                end

                T_READ_REQ: begin
                    if (!scsi_cmd_pending && !read_command_accepted) begin
                        // The WD command was reset or aborted before its first
                        // backing sector completed.  Retire this target
                        // generation without waiting for a separate SCSI bus
                        // reset; a following command may already be queued.
                        active_read <= 1'b0;
                        loaded_valid <= 1'b0;
                        scsi_status_valid <= 1'b0;
                        state <= T_IDLE;
                    end else begin
                        block_req_valid <= 1'b1;
                        block_req_write <= 1'b0;
                        if (block_req_ready)
                            state <= T_READ_WAIT;
                    end
                end

                T_READ_WAIT: begin
                    if (!scsi_cmd_pending && !read_command_accepted) begin
                        // A delayed first-sector completion belongs to the
                        // abandoned command.  After the initial completion,
                        // WD deliberately drops cmd_pending for the complete
                        // data phase, including all later sector refills.
                        active_read <= 1'b0;
                        loaded_valid <= 1'b0;
                        scsi_status_valid <= 1'b0;
                        state <= T_IDLE;
                    end else if (block_done) begin
                        if (block_error) begin
                            set_sense_medium_error();
                            last_error <= 8'h04;
                            active_read <= 1'b0;
                            finish_no_data(ST_ERROR);
                        end else begin
                            loaded_lba <= block_req_lba;
                            loaded_valid <= 1'b1;
                            read_command_accepted <= 1'b1;
                            read_stream_seen <= 1'b0;
                            if (scsi_cmd_pending)
                                scsi_cmd_done <= 1'b1;
                            state <= T_READ_STREAM;
                        end
                    end
                end

                T_READ_STREAM: begin
                    if (scsi_read_consume)
                        read_stream_seen <= 1'b1;
                    if (!read_stream_seen && scsi_data_index != 24'd0) begin
                        state <= T_READ_STREAM;
                    end else if (read_stream_seen &&
                                 scsi_data_index >= scsi_resp_len) begin
                        active_read <= 1'b0;
                        state <= T_WAIT_DROP;
                    end else if (desired_lba != loaded_lba || !loaded_valid) begin
                        block_req_write <= 1'b0;
                        block_req_lba <= desired_lba;
                        loaded_valid <= 1'b0;
                        state <= T_READ_REQ;
                    end
                end

                T_WRITE_STREAM: begin
                    if (scsi_write_valid) begin
                        if ((scsi_data_index[8:0] == 9'd511) ||
                            ((scsi_data_index + 24'd1) >= scsi_resp_len)) begin
                            block_req_write <= 1'b1;
                            block_req_lba <= base_lba + {17'd0, scsi_data_index[23:9]};
                            final_write_block <=
                                ((scsi_data_index + 24'd1) >= scsi_resp_len);
                            state <= T_WRITE_REQ;
                        end
                    end
                end

                T_WRITE_REQ: begin
                    block_req_valid <= 1'b1;
                    block_req_write <= 1'b1;
                    if (block_req_ready)
                        state <= T_WRITE_WAIT;
                end

                T_WRITE_WAIT: begin
                    if (block_done) begin
                        if (block_error) begin
                            set_sense_medium_error();
                            last_error <= 8'h05;
                            scsi_cmd_status <= ST_ERROR;
                        end
                        if (final_write_block) begin
                            active_write <= 1'b0;
                            scsi_status_valid <= 1'b1;
                            state <= T_STATUS_HELD;
                        end else begin
                            state <= T_WRITE_STREAM;
                        end
                    end
                end

                T_STATUS_HELD: begin
                    if (!scsi_data_out_done) begin
                        scsi_status_valid <= 1'b0;
                        state <= T_WAIT_DROP;
                    end
                end

                default: state <= T_IDLE;
            endcase
        end
    end

    wire [31:0] debug_status_word = {
        last_opcode,
        last_error,
        last_target_id,
        media_ready,
        active_read,
        active_write,
        state,
        block_req_valid,
        block_req_write,
        block_req_lba[3:0]
    };

    assign debug_probe = {
        32'h5343_5447, // "SCTG"
        debug_status_word,
        {8'd0, last_xfer_count},
        {8'd0, scsi_resp_len},
        decoded_lba,
        {16'd0, decoded_count},
        {8'd0, scsi_data_index},
        cdb0,
        cdb1,
        cdb7,
        cdb8
    };

    wire unused_inputs = ^{scsi_read_consume, blocks_to_bytes, base_lba};

endmodule

`default_nettype wire
