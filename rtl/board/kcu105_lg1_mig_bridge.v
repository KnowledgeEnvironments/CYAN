// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - KCU105 LG1 coherent-line to DDR4 MIG bridge
//
// The V237 graphics core presents one ordered 64-byte transaction at a time
// in the 50 MHz CPU clock domain.  This bridge crosses that transaction into
// the MIG UI clock domain with a toggle handshake and maps the four 1 MiB LG1
// planes into a reserved 4 MiB window at the top of the 512 MiB DDR image.

`default_nettype none

module kcu105_lg1_mig_bridge #(
    parameter integer APP_ADDR_WIDTH = 28,
    parameter integer APP_DATA_WIDTH = 512,
    parameter integer APP_MASK_WIDTH = 64,
    parameter [25:0] LG1_BASE_DW_ADDR = 26'h3f80000,
    parameter integer ENABLE_READ_AHEAD = 1,
    parameter integer COLD_INIT_ENABLE = 0,
    parameter [15:0] COLD_INIT_START_LINE = 16'h8000,
    parameter [15:0] COLD_INIT_END_LINE = 16'hffff
) (
    input  wire                       cpu_clk,
    input  wire                       cpu_rst_n,
    input  wire                       line_valid,
    output wire                       line_ready,
    output wire                       line_write_done,
    input  wire                       line_write,
    input  wire                       line_scan,
    input  wire [15:0]                line_addr,
    input  wire [511:0]               line_wdata,
    input  wire [63:0]                line_wstrb,
    output reg                        line_rsp_valid,
    input  wire                       line_rsp_ready,
    output reg  [511:0]               line_rsp_rdata,

    input  wire                       mig_clk,
    input  wire                       mig_rst,
    input  wire                       init_calib_complete,
    output wire [APP_ADDR_WIDTH-1:0]  app_addr,
    output wire [2:0]                 app_cmd,
    output wire                       app_en,
    input  wire                       app_rdy,
    output wire                       app_hi_pri,
    output wire [APP_DATA_WIDTH-1:0]  app_wdf_data,
    output wire                       app_wdf_end,
    output wire [APP_MASK_WIDTH-1:0]  app_wdf_mask,
    output wire                       app_wdf_wren,
    input  wire                       app_wdf_rdy,
    input  wire [APP_DATA_WIDTH-1:0]  app_rd_data,
    input  wire                       app_rd_data_end,
    input  wire                       app_rd_data_valid,

    // Same-MIG-domain coherence sideband.  A peer bridge asserts this when
    // both halves of a native line write have been accepted.  The event is
    // used only to invalidate a matching speculative read; demand responses
    // and unrelated lookahead remain independent.
    input  wire                       peer_write_commit,
    input  wire [15:0]                peer_write_addr,
    output wire                       native_write_commit,
    output wire [15:0]                native_write_addr,

    output wire [127:0]               debug_probe,
    output reg                        cold_init_done
);
    localparam [2:0] MIG_CMD_WRITE = 3'b000;
    localparam [2:0] MIG_CMD_READ  = 3'b001;
    localparam [2:0] ST_IDLE           = 3'd0;
    localparam [2:0] ST_READ_CMD       = 3'd1;
    localparam [2:0] ST_READ_WAIT      = 3'd2;
    localparam [2:0] ST_WRITE          = 3'd3;
    localparam [2:0] ST_PREFETCH_CMD   = 3'd4;
    localparam [2:0] ST_PREFETCH_WAIT  = 3'd5;
    localparam [2:0] ST_COLD_INIT      = 3'd6;

    // CPU-domain request mailbox.  Its payload remains stable until the MIG
    // acknowledges the matching toggle.
    reg         req_toggle_cpu;
    reg         req_busy_cpu;
    reg         req_write_cpu;
    reg         req_scan_cpu;
    reg [15:0]  req_addr_cpu;
    reg [511:0] req_wdata_cpu;
    reg [63:0]  req_wstrb_cpu;
    (* ASYNC_REG = "TRUE" *) reg [2:0] ack_sync_cpu;
    reg [511:0] rsp_stage_cpu;
    reg         rsp_stage_valid_cpu;

    reg         ack_toggle_mig;
    reg [511:0] rsp_rdata_mig;

    assign line_ready = cpu_rst_n && !req_busy_cpu && !line_rsp_valid;

    wire ack_stage_cpu = req_busy_cpu &&
                         (ack_sync_cpu[1] == req_toggle_cpu);
    wire ack_retire_cpu = req_busy_cpu && rsp_stage_valid_cpu &&
                          (ack_sync_cpu[2] == req_toggle_cpu);
    wire write_ack_retire_cpu = req_busy_cpu && req_write_cpu &&
                                (ack_sync_cpu[2] == req_toggle_cpu);
    // This is a CPU-domain pulse for exactly one accepted line write.  The
    // MIG-domain toggle changes only after both write command and write data
    // have been accepted, so it is deliberately later than line_ready.
    assign line_write_done = cpu_rst_n && write_ack_retire_cpu;

    always @(posedge cpu_clk or negedge cpu_rst_n) begin
        if (!cpu_rst_n) begin
            req_toggle_cpu <= 1'b0;
            req_busy_cpu <= 1'b0;
            req_write_cpu <= 1'b0;
            req_scan_cpu <= 1'b0;
            req_addr_cpu <= 16'd0;
            req_wdata_cpu <= 512'd0;
            req_wstrb_cpu <= 64'd0;
            ack_sync_cpu <= 3'b000;
            rsp_stage_cpu <= 512'd0;
            rsp_stage_valid_cpu <= 1'b0;
            line_rsp_valid <= 1'b0;
            line_rsp_rdata <= 512'd0;
        end else begin
            ack_sync_cpu <= {ack_sync_cpu[1:0], ack_toggle_mig};

            if (line_rsp_valid && line_rsp_ready)
                line_rsp_valid <= 1'b0;

            if (ack_stage_cpu && !req_write_cpu) begin
                rsp_stage_cpu <= rsp_rdata_mig;
                rsp_stage_valid_cpu <= 1'b1;
            end

            if (ack_retire_cpu) begin
                req_busy_cpu <= 1'b0;
                rsp_stage_valid_cpu <= 1'b0;
                if (!req_write_cpu) begin
                    line_rsp_rdata <= rsp_stage_cpu;
                    line_rsp_valid <= 1'b1;
                end
            end else if (write_ack_retire_cpu) begin
                req_busy_cpu <= 1'b0;
            end

            if (line_valid && line_ready) begin
                req_write_cpu <= line_write;
                req_scan_cpu <= line_scan;
                req_addr_cpu <= line_addr;
                req_wdata_cpu <= line_wdata;
                req_wstrb_cpu <= line_wstrb;
                req_toggle_cpu <= ~req_toggle_cpu;
                req_busy_cpu <= 1'b1;
                rsp_stage_valid_cpu <= 1'b0;
            end
        end
    end

    // MIG-domain transaction engine.
    (* ASYNC_REG = "TRUE" *) reg [2:0] req_sync_mig;
    reg         req_seen_mig;
    reg [2:0]   state;
    reg         op_write;
    reg         op_scan;
    reg [15:0]  op_addr;
    reg [511:0] op_wdata;
    reg [63:0]  op_wstrb;
    reg         write_cmd_accepted;
    reg         write_data_accepted;
    reg [31:0]  read_count;
    reg [31:0]  write_count;
    reg         prefetch_valid;
    reg [15:0]  prefetch_addr;
    reg [15:0]  prefetch_target_addr;
    reg [511:0] prefetch_data;
    reg         prefetch_poisoned;
    reg [15:0]  cold_init_addr;

    function [511:0] logical_to_mig_data;
        input [511:0] logical;
        integer beat;
        integer byte_lane;
        begin
            logical_to_mig_data = 512'd0;
            for (beat = 0; beat < 8; beat = beat + 1)
                for (byte_lane = 0; byte_lane < 8;
                     byte_lane = byte_lane + 1)
                    logical_to_mig_data[
                        (byte_lane * 64) + (beat * 8) +: 8] =
                        logical[(beat * 64) + (byte_lane * 8) +: 8];
        end
    endfunction

    function [63:0] logical_to_mig_mask;
        input [63:0] strobe;
        integer beat;
        integer byte_lane;
        begin
            logical_to_mig_mask = 64'hffff_ffff_ffff_ffff;
            for (beat = 0; beat < 8; beat = beat + 1)
                for (byte_lane = 0; byte_lane < 8;
                     byte_lane = byte_lane + 1)
                    logical_to_mig_mask[(byte_lane * 8) + beat] =
                        ~strobe[(beat * 8) + byte_lane];
        end
    endfunction

    function [511:0] mig_to_logical_data;
        input [511:0] physical;
        integer beat;
        integer byte_lane;
        begin
            mig_to_logical_data = 512'd0;
            for (beat = 0; beat < 8; beat = beat + 1)
                for (byte_lane = 0; byte_lane < 8;
                     byte_lane = byte_lane + 1)
                    mig_to_logical_data[
                        (beat * 64) + (byte_lane * 8) +: 8] =
                        physical[(byte_lane * 64) + (beat * 8) +: 8];
        end
    endfunction

    // The scan producer reads front segments 0..15, then overlay segments
    // 0..15 for one row.  The next independent serial transfer starts at the
    // following row's front segment zero.  Carry this exact sequence into the
    // DDR lookahead rather than guessing address+1 across plane boundaries.
    function [15:0] scan_successor;
        input [15:0] address;
        reg [1:0] plane;
        reg [9:0] row;
        reg [3:0] segment;
        begin
            plane = address[15:14];
            row = address[13:4];
            segment = address[3:0];
            if ((plane == 2'd0) && (segment == 4'hf))
                scan_successor = {2'd2, row, 4'd0};
            else if ((plane == 2'd2) && (segment == 4'hf))
                scan_successor = {
                    2'd0, (row == 10'd767) ? 10'd0 : row + 10'd1, 4'd0
                };
            else
                scan_successor = address + 16'd1;
        end
    endfunction

    wire prefetch_state = (state == ST_PREFETCH_CMD) ||
                          (state == ST_PREFETCH_WAIT);
    wire cold_init_state = (state == ST_COLD_INIT);
    wire [15:0] active_addr = cold_init_state ? cold_init_addr :
                              prefetch_state ? prefetch_target_addr : op_addr;
    wire [25:0] op_dw_addr = LG1_BASE_DW_ADDR +
                             {7'd0, active_addr, 3'b000};
    assign app_addr = {{(APP_ADDR_WIDTH-26){1'b0}}, op_dw_addr};
    assign app_cmd = ((state == ST_WRITE) || cold_init_state) ?
                     MIG_CMD_WRITE : MIG_CMD_READ;
    assign app_en = ((state == ST_READ_CMD) ||
                     (state == ST_PREFETCH_CMD) ||
                     (((state == ST_WRITE) || cold_init_state) &&
                      !write_cmd_accepted));
    assign app_hi_pri = 1'b0;
    assign app_wdf_data = cold_init_state ? {APP_DATA_WIDTH{1'b0}} :
                          logical_to_mig_data(op_wdata);
    assign app_wdf_mask = cold_init_state ? {APP_MASK_WIDTH{1'b0}} :
                          logical_to_mig_mask(op_wstrb);
    assign app_wdf_wren = ((state == ST_WRITE) || cold_init_state) &&
                          !write_data_accepted;
    assign app_wdf_end = app_wdf_wren;

    wire write_cmd_fire = app_en && app_rdy;
    wire write_data_fire = app_wdf_wren && app_wdf_rdy;
    wire write_complete =
        (write_cmd_accepted || write_cmd_fire) &&
        (write_data_accepted || write_data_fire);
    assign native_write_commit = (state == ST_WRITE) && write_complete;
    assign native_write_addr = op_addr;

    wire peer_resident_conflict = peer_write_commit && prefetch_valid &&
                                  (peer_write_addr == prefetch_addr);
    wire peer_target_conflict = peer_write_commit &&
                                (peer_write_addr == prefetch_target_addr);

    always @(posedge mig_clk) begin
        if (mig_rst || !init_calib_complete) begin
            req_sync_mig <= 3'b000;
            req_seen_mig <= 1'b0;
            ack_toggle_mig <= 1'b0;
            rsp_rdata_mig <= 512'd0;
            state <= ST_IDLE;
            op_write <= 1'b0;
            op_scan <= 1'b0;
            op_addr <= 16'd0;
            op_wdata <= 512'd0;
            op_wstrb <= 64'd0;
            write_cmd_accepted <= 1'b0;
            write_data_accepted <= 1'b0;
            read_count <= 32'd0;
            write_count <= 32'd0;
            prefetch_valid <= 1'b0;
            prefetch_addr <= 16'd0;
            prefetch_target_addr <= 16'd0;
            prefetch_data <= 512'd0;
            prefetch_poisoned <= 1'b0;
            cold_init_addr <= COLD_INIT_START_LINE;
            cold_init_done <= (COLD_INIT_ENABLE == 0);
        end else begin
            req_sync_mig <= {req_sync_mig[1:0], req_toggle_cpu};

            // A resident speculative copy cannot survive a matching peer
            // write which has now linearized at the native MIG interface.
            if (peer_resident_conflict)
                prefetch_valid <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (!cold_init_done) begin
                        write_cmd_accepted <= 1'b0;
                        write_data_accepted <= 1'b0;
                        state <= ST_COLD_INIT;
                    end else if (req_sync_mig[2] != req_seen_mig) begin
                        req_seen_mig <= req_sync_mig[2];
                        op_write <= req_write_cpu;
                        op_scan <= req_scan_cpu;
                        op_addr <= req_addr_cpu;
                        op_wdata <= req_wdata_cpu;
                        op_wstrb <= req_wstrb_cpu;
                        write_cmd_accepted <= 1'b0;
                        write_data_accepted <= 1'b0;
                        if (req_write_cpu) begin
                            // A speculative line must never survive a write;
                            // global invalidation is cheap and unambiguous.
                            prefetch_valid <= 1'b0;
                            state <= ST_WRITE;
                        end else if ((ENABLE_READ_AHEAD != 0) &&
                                     prefetch_valid &&
                                     !peer_resident_conflict &&
                                     (prefetch_addr == req_addr_cpu)) begin
                            rsp_rdata_mig <= prefetch_data;
                            ack_toggle_mig <= req_sync_mig[2];
                            read_count <= read_count + 32'd1;
                            prefetch_valid <= 1'b0;
                            prefetch_target_addr <= req_scan_cpu ?
                                scan_successor(req_addr_cpu) :
                                req_addr_cpu + 16'd1;
                            prefetch_poisoned <= 1'b0;
                            state <= (ENABLE_READ_AHEAD != 0) ?
                                     ST_PREFETCH_CMD : ST_IDLE;
                        end else begin
                            prefetch_valid <= 1'b0;
                            state <= ST_READ_CMD;
                        end
                    end
                end

                ST_READ_CMD: begin
                    if (app_rdy)
                        state <= ST_READ_WAIT;
                end

                ST_READ_WAIT: begin
                    if (app_rd_data_valid) begin
                        rsp_rdata_mig <= mig_to_logical_data(app_rd_data);
                        ack_toggle_mig <= req_seen_mig;
                        read_count <= read_count + 32'd1;
                        prefetch_valid <= 1'b0;
                        prefetch_target_addr <= op_scan ?
                            scan_successor(op_addr) : op_addr + 16'd1;
                        prefetch_poisoned <= 1'b0;
                        state <= (ENABLE_READ_AHEAD != 0) ?
                                 ST_PREFETCH_CMD : ST_IDLE;
                    end
                end

                ST_PREFETCH_CMD: begin
                    // If a same-line peer write and read command linearize on
                    // this edge, the eventual older read response is poison.
                    // A write before command acceptance needs no poison: the
                    // later native read will observe the new line.
                    if (app_rdy) begin
                        if (peer_target_conflict)
                            prefetch_poisoned <= 1'b1;
                        state <= ST_PREFETCH_WAIT;
                    end
                end

                ST_PREFETCH_WAIT: begin
                    if (peer_target_conflict)
                        prefetch_poisoned <= 1'b1;
                    if (app_rd_data_valid) begin
                        prefetch_data <= mig_to_logical_data(app_rd_data);
                        prefetch_addr <= prefetch_target_addr;
                        prefetch_valid <= !(prefetch_poisoned ||
                                            peer_target_conflict);
                        prefetch_poisoned <= 1'b0;
                        state <= ST_IDLE;
                    end
                end

                ST_WRITE: begin
                    if (write_cmd_fire)
                        write_cmd_accepted <= 1'b1;
                    if (write_data_fire)
                        write_data_accepted <= 1'b1;
                    if (write_complete) begin
                        ack_toggle_mig <= req_seen_mig;
                        write_count <= write_count + 32'd1;
                        state <= ST_IDLE;
                    end
                end

                ST_COLD_INIT: begin
                    if (write_cmd_fire)
                        write_cmd_accepted <= 1'b1;
                    if (write_data_fire)
                        write_data_accepted <= 1'b1;
                    if (write_complete) begin
                        write_count <= write_count + 32'd1;
                        write_cmd_accepted <= 1'b0;
                        write_data_accepted <= 1'b0;
                        if (cold_init_addr == COLD_INIT_END_LINE) begin
                            cold_init_done <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            cold_init_addr <= cold_init_addr + 16'd1;
                        end
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    wire unused_app_rd_data_end = app_rd_data_end;
    assign debug_probe = {
        44'd0,
        cold_init_done,
        prefetch_addr,
        prefetch_valid,
        unused_app_rd_data_end,
        read_count[15:0],
        write_count[15:0],
        op_addr,
        op_write,
        write_data_accepted,
        write_cmd_accepted,
        state[1:0],
        app_wdf_rdy,
        app_rdy,
        app_rd_data_valid,
        app_wdf_wren,
        app_en,
        init_calib_complete,
        req_seen_mig,
        req_sync_mig[2],
        ack_toggle_mig,
        req_toggle_cpu,
        req_busy_cpu,
        line_rsp_valid
    };
endmodule

`default_nettype wire
