// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// Bridge the IP20 MC's 64-bit RAM handshake to the Xilinx DDR4 MIG app port.
//
// The CPU/MC island may run slower than the MIG UI clock. Requests cross with a
// toggle handshake; each MC doubleword maps into one 64-bit beat of a 512-bit
// MIG burst. Xilinx's native app data is byte-lane interleaved across the
// eight DDR beats, so a CPU doubleword is scattered/gathered by byte.

`default_nettype none

module mc_mig_bridge #(
    parameter integer APP_ADDR_WIDTH = 28,
    parameter integer APP_DATA_WIDTH = 512,
    parameter integer APP_MASK_WIDTH = 64,
    parameter integer WRITE_SETTLE_CYCLES = 64,
    parameter [15:0] READ_TIMEOUT_CYCLES = 16'd1024
) (
    input  wire                       cpu_clk,
    input  wire                       cpu_rst_n,
    input  wire                       mig_clk,
    input  wire                       mig_rst,
    input  wire                       init_calib_complete,

    input  wire                       cpu_valid,
    input  wire                       cpu_wen,
    input  wire [25:0]                cpu_addr,
    input  wire [63:0]                cpu_wdata,
    input  wire [ 7:0]                cpu_wmask,
    output reg  [63:0]                cpu_rdata,
    output wire                       cpu_ready,

    output reg  [APP_ADDR_WIDTH-1:0]  app_addr,
    output reg  [2:0]                 app_cmd,
    output reg                        app_en,
    input  wire                       app_rdy,
    output wire                       app_hi_pri,
    output reg  [APP_DATA_WIDTH-1:0]  app_wdf_data,
    output reg                        app_wdf_end,
    output reg  [APP_MASK_WIDTH-1:0]  app_wdf_mask,
    output reg                        app_wdf_wren,
    input  wire                       app_wdf_rdy,
    input  wire [APP_DATA_WIDTH-1:0]  app_rd_data,
    input  wire                       app_rd_data_end,
    input  wire                       app_rd_data_valid,

    output wire [255:0]               debug_probe
);
    localparam [2:0] MIG_CMD_WRITE = 3'b000;
    localparam [2:0] MIG_CMD_READ  = 3'b001;

    assign app_hi_pri = 1'b0;

    reg        req_toggle_cpu;
    reg        cpu_busy;
    reg        cpu_ready_r;
    reg        cpu_resp_pending;
    reg        cpu_accept_armed;
    reg        req_wen_cpu;
    reg [25:0] req_addr_cpu;
    reg [63:0] req_wdata_cpu;
    reg [ 7:0] req_wmask_cpu;

    reg        ack_toggle_mig;
    reg [63:0] resp_rdata_mig;

    reg [2:0]  ack_sync_cpu;
    reg [63:0] resp_rdata_cpu_stage;
    reg        resp_rdata_cpu_stage_valid;

    wire ack_stage_cpu =
        cpu_busy && (ack_sync_cpu[1] == req_toggle_cpu);
    wire ack_retired_cpu =
        cpu_busy && resp_rdata_cpu_stage_valid &&
        (ack_sync_cpu[2] == req_toggle_cpu);
    wire cpu_req_changed =
        (cpu_addr != req_addr_cpu) ||
        (cpu_wen != req_wen_cpu) ||
        (cpu_wen && ((cpu_wdata != req_wdata_cpu) ||
                     (cpu_wmask != req_wmask_cpu)));

    assign cpu_ready = cpu_ready_r && !(cpu_valid && cpu_req_changed);

    always @(posedge cpu_clk or negedge cpu_rst_n) begin
        if (!cpu_rst_n) begin
            req_toggle_cpu <= 1'b0;
            cpu_busy <= 1'b0;
            cpu_ready_r <= 1'b0;
            cpu_resp_pending <= 1'b0;
            cpu_accept_armed <= 1'b1;
            req_wen_cpu <= 1'b0;
            req_addr_cpu <= 26'd0;
            req_wdata_cpu <= 64'd0;
            req_wmask_cpu <= 8'd0;
            ack_sync_cpu <= 3'b000;
            resp_rdata_cpu_stage <= 64'd0;
            resp_rdata_cpu_stage_valid <= 1'b0;
            cpu_rdata <= 64'd0;
        end else begin
            ack_sync_cpu <= {ack_sync_cpu[1:0], ack_toggle_mig};

            if (!cpu_valid) begin
                cpu_accept_armed <= 1'b1;
                cpu_resp_pending <= 1'b0;
                cpu_ready_r <= 1'b0;
            end else if (cpu_resp_pending && !cpu_req_changed) begin
                cpu_ready_r <= 1'b1;
            end else begin
                cpu_ready_r <= 1'b0;
            end

            if (ack_stage_cpu) begin
                resp_rdata_cpu_stage <= resp_rdata_mig;
                resp_rdata_cpu_stage_valid <= 1'b1;
            end

            if (ack_retired_cpu) begin
                cpu_rdata <= resp_rdata_cpu_stage;
                cpu_ready_r <= 1'b1;
                cpu_busy <= 1'b0;
                cpu_resp_pending <= 1'b1;
                resp_rdata_cpu_stage_valid <= 1'b0;
            end else if (cpu_valid && !cpu_busy &&
                         (cpu_accept_armed || cpu_req_changed)) begin
                req_wen_cpu <= cpu_wen;
                req_addr_cpu <= cpu_addr;
                req_wdata_cpu <= cpu_wdata;
                req_wmask_cpu <= cpu_wmask;
                req_toggle_cpu <= ~req_toggle_cpu;
                cpu_busy <= 1'b1;
                cpu_resp_pending <= 1'b0;
                resp_rdata_cpu_stage_valid <= 1'b0;
                cpu_accept_armed <= 1'b0;
                cpu_ready_r <= 1'b0;
            end
        end
    end

    reg [2:0]  req_sync_mig;
    reg        req_seen_mig;
    reg        op_wen;
    reg [25:0] op_addr;
    reg [63:0] op_wdata;
    reg [ 7:0] op_wmask;
    reg [15:0] read_count;
    reg [15:0] write_count;
    reg [15:0] stall_count;
    reg [15:0] read_wait_count;
    reg [15:0] read_timeout_count;
    reg [ 3:0] state;
    reg [ 3:0] last_beat;
    reg [25:0] last_addr;
    reg [63:0] last_rdata;
    reg [63:0] last_wdata;
    reg        write_cmd_accepted;
    reg        write_data_accepted;
    reg [APP_DATA_WIDTH-1:0] op_line_data;
    reg [ 7:0] verify_match_lanes;
    reg [ 7:0] verify_retry_count;
`ifdef KCU105_CYAN_SOC_DEBUG
    reg [APP_ADDR_WIDTH-1:0] dbg_first_prom_write_app_addr;
    reg [63:0] dbg_first_prom_write_wdata;
    reg [63:0] dbg_first_prom_read_rdata;
    reg [63:0] dbg_first_prom_bad_read_rdata;
    reg [APP_ADDR_WIDTH-1:0] dbg_first_prom_bad_read_app_addr;
    reg [ 7:0] dbg_first_prom_write_wmask;
    reg [ 2:0] dbg_first_prom_write_beat;
    reg [ 2:0] dbg_first_prom_read_beat;
    reg [ 2:0] dbg_first_prom_bad_read_beat;
    reg [15:0] dbg_first_prom_bad_read_count;
    reg        dbg_first_prom_write_cmd_valid;
    reg        dbg_first_prom_write_data_valid;
    reg        dbg_first_prom_read_valid;
    reg        dbg_first_prom_bad_read_valid;
    reg        dbg_first_prom_write_cmd_with_data;
    reg        dbg_first_prom_write_data_with_cmd;
`endif

    localparam [3:0]
        ST_IDLE        = 4'd0,
        ST_WRITE_SETUP = 4'd1,
        ST_WRITE_WAIT  = 4'd2,
        ST_READ_SETUP  = 4'd3,
        ST_READ_CMD    = 4'd4,
        ST_READ_WAIT   = 4'd5,
        ST_DONE        = 4'd6,
        ST_RMW_READ_SETUP    = 4'd7,
        ST_RMW_READ_CMD      = 4'd8,
        ST_RMW_READ_WAIT     = 4'd9,
        ST_WRITE_VERIFY_SETUP = 4'd10,
        ST_WRITE_VERIFY_CMD   = 4'd11,
        ST_WRITE_VERIFY_WAIT  = 4'd12,
        ST_WRITE_VERIFY_DECIDE = 4'd13;

    wire [2:0] op_beat = op_addr[2:0];
    wire [APP_ADDR_WIDTH-1:0] op_app_addr =
        {{(APP_ADDR_WIDTH-26){1'b0}}, op_addr[25:3], 3'b000};

    function [APP_DATA_WIDTH-1:0] beat_wdata;
        input [63:0] data;
        input [2:0]  beat;
        reg [APP_DATA_WIDTH-1:0] tmp;
        integer byte_idx;
        begin
            tmp = {APP_DATA_WIDTH{1'b0}};
            for (byte_idx = 0; byte_idx < 8; byte_idx = byte_idx + 1)
                tmp[byte_idx * 64 + beat * 8 +: 8] =
                    data[byte_idx * 8 +: 8];
            beat_wdata = tmp;
        end
    endfunction

    function [APP_MASK_WIDTH-1:0] beat_wmask;
        input [7:0] mask;
        input [2:0] beat;
        reg [APP_MASK_WIDTH-1:0] tmp;
        integer byte_idx;
        integer beat_idx;
        integer mask_idx;
        begin
            tmp = {APP_MASK_WIDTH{1'b1}};
            beat_idx = {29'd0, beat};
            for (byte_idx = 0; byte_idx < 8; byte_idx = byte_idx + 1) begin
                mask_idx = byte_idx * 8 + beat_idx;
                tmp[mask_idx] = ~mask[byte_idx];
            end
            beat_wmask = tmp;
        end
    endfunction

    function [63:0] beat_rdata;
        input [APP_DATA_WIDTH-1:0] data;
        input [2:0] beat;
        reg [63:0] tmp;
        integer byte_idx;
        begin
            tmp = 64'd0;
            for (byte_idx = 0; byte_idx < 8; byte_idx = byte_idx + 1)
                tmp[byte_idx * 8 +: 8] =
                    data[byte_idx * 64 + beat * 8 +: 8];
            beat_rdata = tmp;
        end
    endfunction

    function [APP_DATA_WIDTH-1:0] merge_beat_wdata;
        input [APP_DATA_WIDTH-1:0] line;
        input [63:0] data;
        input [7:0] mask;
        input [2:0] beat;
        reg [APP_DATA_WIDTH-1:0] tmp;
        integer byte_idx;
        begin
            tmp = line;
            for (byte_idx = 0; byte_idx < 8; byte_idx = byte_idx + 1) begin
                if (mask[byte_idx])
                    tmp[byte_idx * 64 + beat * 8 +: 8] =
                        data[byte_idx * 8 +: 8];
            end
            merge_beat_wdata = tmp;
        end
    endfunction

    function [7:0] beat_seen_wmask;
        input [APP_MASK_WIDTH-1:0] mask;
        input [2:0] beat;
        reg [7:0] tmp;
        integer byte_idx;
        integer beat_idx;
        integer mask_idx;
        begin
            tmp = 8'd0;
            beat_idx = {29'd0, beat};
            for (byte_idx = 0; byte_idx < 8; byte_idx = byte_idx + 1) begin
                mask_idx = byte_idx * 8 + beat_idx;
                tmp[byte_idx] = ~mask[mask_idx];
            end
            beat_seen_wmask = tmp;
        end
    endfunction

    function [7:0] line_match_lanes;
        input [APP_DATA_WIDTH-1:0] got;
        input [APP_DATA_WIDTH-1:0] expected;
        integer lane_idx;
        begin
            for (lane_idx = 0; lane_idx < 8; lane_idx = lane_idx + 1)
                line_match_lanes[lane_idx] =
                    got[lane_idx * 64 +: 64] ==
                    expected[lane_idx * 64 +: 64];
        end
    endfunction

`ifdef KCU105_CYAN_SOC_DEBUG
    wire [63:0] dbg_first_prom_expected_rdata =
        op_addr[0] ? 64'h4444_4444_8888_8888 :
                     64'h1111_1111_2222_2222;
`endif

    wire write_cmd_fire =
        !write_cmd_accepted && app_en && (app_cmd == MIG_CMD_WRITE) && app_rdy;
    wire write_data_fire =
        !write_data_accepted && app_wdf_wren && app_wdf_rdy;
    wire write_done =
        (write_cmd_accepted || write_cmd_fire) &&
        (write_data_accepted || write_data_fire);

    always @(posedge mig_clk) begin
        if (mig_rst || !init_calib_complete) begin
            req_sync_mig <= 3'b000;
            req_seen_mig <= 1'b0;
            ack_toggle_mig <= 1'b0;
            resp_rdata_mig <= 64'd0;
            app_addr <= {APP_ADDR_WIDTH{1'b0}};
            app_cmd <= MIG_CMD_READ;
            app_en <= 1'b0;
            app_wdf_data <= {APP_DATA_WIDTH{1'b0}};
            app_wdf_end <= 1'b0;
            app_wdf_mask <= {APP_MASK_WIDTH{1'b1}};
            app_wdf_wren <= 1'b0;
            op_wen <= 1'b0;
            op_addr <= 26'd0;
            op_wdata <= 64'd0;
            op_wmask <= 8'd0;
            read_count <= 16'd0;
            write_count <= 16'd0;
            stall_count <= 16'd0;
            read_wait_count <= 16'd0;
            read_timeout_count <= 16'd0;
            state <= ST_IDLE;
            last_beat <= 4'd0;
            last_addr <= 26'd0;
            last_rdata <= 64'd0;
            last_wdata <= 64'd0;
            write_cmd_accepted <= 1'b0;
            write_data_accepted <= 1'b0;
            op_line_data <= {APP_DATA_WIDTH{1'b0}};
            verify_match_lanes <= 8'd0;
            verify_retry_count <= 8'd0;
`ifdef KCU105_CYAN_SOC_DEBUG
            dbg_first_prom_write_app_addr <= {APP_ADDR_WIDTH{1'b0}};
            dbg_first_prom_write_wdata <= 64'd0;
            dbg_first_prom_read_rdata <= 64'd0;
            dbg_first_prom_bad_read_rdata <= 64'd0;
            dbg_first_prom_bad_read_app_addr <= {APP_ADDR_WIDTH{1'b0}};
            dbg_first_prom_write_wmask <= 8'd0;
            dbg_first_prom_write_beat <= 3'd0;
            dbg_first_prom_read_beat <= 3'd0;
            dbg_first_prom_bad_read_beat <= 3'd0;
            dbg_first_prom_bad_read_count <= 16'd0;
            dbg_first_prom_write_cmd_valid <= 1'b0;
            dbg_first_prom_write_data_valid <= 1'b0;
            dbg_first_prom_read_valid <= 1'b0;
            dbg_first_prom_bad_read_valid <= 1'b0;
            dbg_first_prom_write_cmd_with_data <= 1'b0;
            dbg_first_prom_write_data_with_cmd <= 1'b0;
`endif
        end else begin
            req_sync_mig <= {req_sync_mig[1:0], req_toggle_cpu};
            app_en <= 1'b0;
            app_wdf_wren <= 1'b0;
            app_wdf_end <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (req_sync_mig[2] != req_seen_mig) begin
                        req_seen_mig <= req_sync_mig[2];
                        op_wen <= req_wen_cpu;
                        op_addr <= req_addr_cpu;
                        op_wdata <= req_wdata_cpu;
                        op_wmask <= req_wmask_cpu;
                        write_cmd_accepted <= 1'b0;
                        write_data_accepted <= 1'b0;
                        if (req_wen_cpu && (req_wmask_cpu == 8'h00))
                            state <= ST_DONE;
                        else if (req_wen_cpu)
                            state <= ST_RMW_READ_SETUP;
                        else
                            state <= ST_READ_SETUP;
                    end
                end

                ST_RMW_READ_SETUP: begin
                    app_addr <= op_app_addr;
                    app_cmd <= MIG_CMD_READ;
                    app_en <= 1'b1;
                    state <= ST_RMW_READ_CMD;
                end

                ST_RMW_READ_CMD: begin
                    app_addr <= op_app_addr;
                    app_cmd <= MIG_CMD_READ;
                    app_en <= 1'b1;
                    if (app_rdy) begin
                        app_en <= 1'b0;
                        read_wait_count <= 16'd0;
                        state <= ST_RMW_READ_WAIT;
                    end else if (stall_count != 16'hffff) begin
                        stall_count <= stall_count + 16'd1;
                    end
                end

                ST_RMW_READ_WAIT: begin
                    if (app_rd_data_valid) begin
                        op_line_data <=
                            merge_beat_wdata(app_rd_data, op_wdata,
                                             op_wmask, op_beat);
                        verify_retry_count <= 8'd0;
                        read_wait_count <= 16'd0;
                        state <= ST_WRITE_SETUP;
                    end else if (read_wait_count >= READ_TIMEOUT_CYCLES) begin
                        read_wait_count <= 16'd0;
                        if (read_timeout_count != 16'hffff)
                            read_timeout_count <= read_timeout_count + 16'd1;
                        if (stall_count != 16'hffff)
                            stall_count <= stall_count + 16'd1;
                        // An accepted MIG read has exactly one ordered
                        // response.  Without transaction IDs, reissuing it
                        // here can leave a late duplicate to satisfy the next
                        // unrelated read.
                    end else if (stall_count != 16'hffff) begin
                        read_wait_count <= read_wait_count + 16'd1;
                        stall_count <= stall_count + 16'd1;
                    end else begin
                        read_wait_count <= read_wait_count + 16'd1;
                    end
                end

                ST_WRITE_SETUP: begin
                    app_addr <= op_app_addr;
                    app_cmd <= MIG_CMD_WRITE;
                    app_wdf_data <= op_line_data;
                    app_wdf_mask <= {APP_MASK_WIDTH{1'b0}};
                    app_en <= 1'b1;
                    app_wdf_wren <= 1'b1;
                    app_wdf_end <= 1'b1;
                    state <= ST_WRITE_WAIT;
                end

                ST_WRITE_WAIT: begin
                    app_addr <= op_app_addr;
                    app_cmd <= MIG_CMD_WRITE;
                    app_wdf_data <= op_line_data;
                    app_wdf_mask <= {APP_MASK_WIDTH{1'b0}};
                    app_en <= !(write_cmd_accepted || write_cmd_fire);
                    app_wdf_wren <= !(write_data_accepted || write_data_fire);
                    app_wdf_end <= !(write_data_accepted || write_data_fire);
`ifdef KCU105_CYAN_SOC_DEBUG
                    if (op_addr[25:1] == 25'h0800000) begin
                        if (write_cmd_fire && !dbg_first_prom_write_cmd_valid) begin
                            dbg_first_prom_write_cmd_valid <= 1'b1;
                            dbg_first_prom_write_app_addr <= op_app_addr;
                            dbg_first_prom_write_beat <= op_beat;
                            dbg_first_prom_write_cmd_with_data <= write_data_fire;
                        end
                        if (write_data_fire && !dbg_first_prom_write_data_valid) begin
                            dbg_first_prom_write_data_valid <= 1'b1;
                            dbg_first_prom_write_app_addr <= op_app_addr;
                            dbg_first_prom_write_wdata <=
                                beat_rdata(op_line_data, op_beat);
                            dbg_first_prom_write_wmask <= op_wmask;
                            dbg_first_prom_write_beat <= op_beat;
                            dbg_first_prom_write_data_with_cmd <= write_cmd_fire;
                        end
                    end
`endif
                    if (write_cmd_fire)
                        write_cmd_accepted <= 1'b1;
                    if (write_data_fire)
                        write_data_accepted <= 1'b1;
                    if (write_done) begin
                        app_en <= 1'b0;
                        app_wdf_wren <= 1'b0;
                        app_wdf_end <= 1'b0;
                        write_count <= write_count + 16'd1;
                        last_addr <= op_addr;
                        last_beat <= {1'b0, op_beat};
                        last_wdata <= beat_rdata(op_line_data, op_beat);
                        state <= ST_WRITE_VERIFY_SETUP;
                    end else if (stall_count != 16'hffff) begin
                        stall_count <= stall_count + 16'd1;
                    end
                end

                ST_WRITE_VERIFY_SETUP: begin
                    app_addr <= op_app_addr;
                    app_cmd <= MIG_CMD_READ;
                    app_en <= 1'b1;
                    state <= ST_WRITE_VERIFY_CMD;
                end

                ST_WRITE_VERIFY_CMD: begin
                    app_addr <= op_app_addr;
                    app_cmd <= MIG_CMD_READ;
                    app_en <= 1'b1;
                    if (app_rdy) begin
                        app_en <= 1'b0;
                        read_wait_count <= 16'd0;
                        state <= ST_WRITE_VERIFY_WAIT;
                    end else if (stall_count != 16'hffff) begin
                        stall_count <= stall_count + 16'd1;
                    end
                end

                ST_WRITE_VERIFY_WAIT: begin
                    if (app_rd_data_valid) begin
                        read_wait_count <= 16'd0;
                        verify_match_lanes <=
                            line_match_lanes(app_rd_data, op_line_data);
                        state <= ST_WRITE_VERIFY_DECIDE;
                    end else if (read_wait_count >= READ_TIMEOUT_CYCLES) begin
                        read_wait_count <= 16'd0;
                        if (read_timeout_count != 16'hffff)
                            read_timeout_count <= read_timeout_count + 16'd1;
                        if (stall_count != 16'hffff)
                            stall_count <= stall_count + 16'd1;
                        // Keep waiting for the one response corresponding to
                        // the accepted command; a retry cannot be correlated.
                    end else if (stall_count != 16'hffff) begin
                        read_wait_count <= read_wait_count + 16'd1;
                        stall_count <= stall_count + 16'd1;
                    end else begin
                        read_wait_count <= read_wait_count + 16'd1;
                    end
                end

                ST_WRITE_VERIFY_DECIDE: begin
                    if (&verify_match_lanes) begin
                        state <= ST_DONE;
                    end else if (verify_retry_count == 8'hff) begin
                        verify_retry_count <= 8'd0;
                        write_cmd_accepted <= 1'b0;
                        write_data_accepted <= 1'b0;
                        state <= ST_WRITE_SETUP;
                    end else begin
                        verify_retry_count <= verify_retry_count + 8'd1;
                        state <= ST_WRITE_VERIFY_SETUP;
                    end
                end

                ST_READ_SETUP: begin
                    app_addr <= op_app_addr;
                    app_cmd <= MIG_CMD_READ;
                    app_en <= 1'b1;
                    state <= ST_READ_CMD;
                end

                ST_READ_CMD: begin
                    app_addr <= op_app_addr;
                    app_cmd <= MIG_CMD_READ;
                    app_en <= 1'b1;
                    if (app_rdy) begin
                        app_en <= 1'b0;
                        read_wait_count <= 16'd0;
                        state <= ST_READ_WAIT;
                    end else if (stall_count != 16'hffff) begin
                        stall_count <= stall_count + 16'd1;
                    end
                end

                ST_READ_WAIT: begin
                    if (app_rd_data_valid) begin
                        resp_rdata_mig <= beat_rdata(app_rd_data, op_beat);
                        read_count <= read_count + 16'd1;
                        read_wait_count <= 16'd0;
                        last_addr <= op_addr;
                        last_beat <= {1'b0, op_beat};
                        last_rdata <= beat_rdata(app_rd_data, op_beat);
`ifdef KCU105_CYAN_SOC_DEBUG
                        if ((op_addr[25:1] == 25'h0800000) &&
                            !dbg_first_prom_read_valid) begin
                            dbg_first_prom_read_valid <= 1'b1;
                            dbg_first_prom_read_rdata <=
                                beat_rdata(app_rd_data, op_beat);
                            dbg_first_prom_read_beat <= op_beat;
                        end
                        if ((op_addr[25:1] == 25'h0800000) &&
                            (beat_rdata(app_rd_data, op_beat) !=
                             dbg_first_prom_expected_rdata)) begin
                            if (dbg_first_prom_bad_read_count != 16'hffff)
                                dbg_first_prom_bad_read_count <=
                                    dbg_first_prom_bad_read_count + 16'd1;
                            if (!dbg_first_prom_bad_read_valid) begin
                                dbg_first_prom_bad_read_valid <= 1'b1;
                                dbg_first_prom_bad_read_rdata <=
                                    beat_rdata(app_rd_data, op_beat);
                                dbg_first_prom_bad_read_app_addr <= op_app_addr;
                                dbg_first_prom_bad_read_beat <= op_beat;
                            end
                        end
`endif
                        state <= ST_DONE;
                    end else if (read_wait_count >= READ_TIMEOUT_CYCLES) begin
                        read_wait_count <= 16'd0;
                        if (read_timeout_count != 16'hffff)
                            read_timeout_count <= read_timeout_count + 16'd1;
                        if (stall_count != 16'hffff)
                            stall_count <= stall_count + 16'd1;
                        // Keep waiting for the accepted read.  Retrying a
                        // merely late response can corrupt a later request.
                    end else if (stall_count != 16'hffff) begin
                        read_wait_count <= read_wait_count + 16'd1;
                        stall_count <= stall_count + 16'd1;
                    end else begin
                        read_wait_count <= read_wait_count + 16'd1;
                    end
                end

                ST_DONE: begin
                    if (op_wen)
                        resp_rdata_mig <= 64'd0;
                    ack_toggle_mig <= ~ack_toggle_mig;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

`ifdef KCU105_CYAN_SOC_DEBUG
    wire [31:0] dbg_mig_live_status = {
        8'h05,
        state,
        app_cmd,
        init_calib_complete,
        mig_rst,
        app_en,
        app_rdy,
        app_rd_data_valid,
        app_rd_data_end,
        app_wdf_wren,
        app_wdf_rdy,
        write_cmd_accepted,
        write_data_accepted,
        req_sync_mig,
        req_seen_mig,
        ack_toggle_mig,
        2'd0
    };

    wire [31:0] dbg_cpu_live_status = {
        8'hc5,
        cpu_valid,
        cpu_wen,
        cpu_ready,
        cpu_ready_r,
        cpu_busy,
        cpu_resp_pending,
        cpu_accept_armed,
        cpu_req_changed,
        ack_stage_cpu,
        ack_retired_cpu,
        resp_rdata_cpu_stage_valid,
        ack_sync_cpu,
        req_toggle_cpu,
        ack_toggle_mig,
        cpu_wmask
    };

    assign debug_probe = {
        32'h4D49_4735, // "MIG5"
        dbg_mig_live_status,
        {read_wait_count, read_timeout_count},
        {read_count, stall_count},
        {op_wen, op_beat, 2'd0, op_addr},
        {4'd0, app_addr},
        dbg_cpu_live_status,
        {6'd0, req_addr_cpu}
    };
`else
    assign debug_probe = {
        32'h4D49_4730, // "MIG0"
        last_rdata,
        last_wdata,
        {6'd0, last_addr},
        {read_count, write_count},
        {stall_count, 8'd0, last_beat, state}
    };
`endif

    wire unused_app_rd_data_end = app_rd_data_end;
endmodule

`default_nettype wire
