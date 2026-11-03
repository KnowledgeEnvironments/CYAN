// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - composable two-client LG1 MIG transaction arbiter
//
// Unlike the board's final physical MIG arbiter, this stage is itself an
// upstream client.  It must therefore present held valid signals without
// gating them by downstream ready.  Command/data acceptance is tracked per
// selected transaction, and an ordered owner FIFO routes pipelined reads.

`default_nettype none

module kcu105_lg1_port_arbiter #(
    parameter integer APP_ADDR_WIDTH = 28,
    parameter integer APP_DATA_WIDTH = 512,
    parameter integer APP_MASK_WIDTH = 64
) (
    input wire clk,
    input wire rst,

    input wire [APP_ADDR_WIDTH-1:0] rb_app_addr,
    input wire [2:0] rb_app_cmd,
    input wire rb_app_en,
    output wire rb_app_rdy,
    input wire rb_app_hi_pri,
    input wire [APP_DATA_WIDTH-1:0] rb_app_wdf_data,
    input wire rb_app_wdf_end,
    input wire [APP_MASK_WIDTH-1:0] rb_app_wdf_mask,
    input wire rb_app_wdf_wren,
    output wire rb_app_wdf_rdy,
    output wire [APP_DATA_WIDTH-1:0] rb_app_rd_data,
    output wire rb_app_rd_data_end,
    output wire rb_app_rd_data_valid,

    input wire [APP_ADDR_WIDTH-1:0] scan_app_addr,
    input wire [2:0] scan_app_cmd,
    input wire scan_app_en,
    output wire scan_app_rdy,
    input wire scan_app_hi_pri,
    input wire [APP_DATA_WIDTH-1:0] scan_app_wdf_data,
    input wire scan_app_wdf_end,
    input wire [APP_MASK_WIDTH-1:0] scan_app_wdf_mask,
    input wire scan_app_wdf_wren,
    output wire scan_app_wdf_rdy,
    output wire [APP_DATA_WIDTH-1:0] scan_app_rd_data,
    output wire scan_app_rd_data_end,
    output wire scan_app_rd_data_valid,

    output wire [APP_ADDR_WIDTH-1:0] app_addr,
    output wire [2:0] app_cmd,
    output wire app_en,
    input wire app_rdy,
    output wire app_hi_pri,
    output wire [APP_DATA_WIDTH-1:0] app_wdf_data,
    output wire app_wdf_end,
    output wire [APP_MASK_WIDTH-1:0] app_wdf_mask,
    output wire app_wdf_wren,
    input wire app_wdf_rdy,
    input wire [APP_DATA_WIDTH-1:0] app_rd_data,
    input wire app_rd_data_end,
    input wire app_rd_data_valid,

    output wire [63:0] debug_probe
);
    localparam [2:0] MIG_CMD_WRITE = 3'b000;
    localparam integer OWNER_DEPTH = 8;
    localparam [3:0] OWNER_DEPTH_COUNT = 4'd8;

    reg active;
    reg owner_scan;
    reg last_owner_scan;
    reg cmd_accepted;
    reg data_accepted;
    reg [7:0] owner_fifo;
    reg [3:0] owner_count;
    reg [15:0] rb_grants;
    reg [15:0] scan_grants;

    wire choose_scan = scan_app_en &&
                       (!rb_app_en || !last_owner_scan);
    wire selected_en = owner_scan ? scan_app_en : rb_app_en;
    wire selected_wdf_wren = owner_scan ? scan_app_wdf_wren :
                                          rb_app_wdf_wren;
    wire selected_write = owner_scan ?
        (scan_app_cmd == MIG_CMD_WRITE) : (rb_app_cmd == MIG_CMD_WRITE);
    wire owner_full = (owner_count == OWNER_DEPTH_COUNT);

    assign app_addr = owner_scan ? scan_app_addr : rb_app_addr;
    assign app_cmd = owner_scan ? scan_app_cmd : rb_app_cmd;
    assign app_hi_pri = owner_scan ? scan_app_hi_pri : rb_app_hi_pri;
    assign app_wdf_data = owner_scan ? scan_app_wdf_data : rb_app_wdf_data;
    assign app_wdf_end = owner_scan ? scan_app_wdf_end : rb_app_wdf_end;
    assign app_wdf_mask = owner_scan ? scan_app_wdf_mask : rb_app_wdf_mask;
    assign app_en = active && selected_en && !cmd_accepted &&
                    (selected_write || !owner_full);
    assign app_wdf_wren = active && selected_write && selected_wdf_wren &&
                          !data_accepted;

    assign rb_app_rdy = active && !owner_scan && app_rdy && app_en;
    assign scan_app_rdy = active && owner_scan && app_rdy && app_en;
    assign rb_app_wdf_rdy = active && !owner_scan && app_wdf_rdy &&
                            app_wdf_wren;
    assign scan_app_wdf_rdy = active && owner_scan && app_wdf_rdy &&
                              app_wdf_wren;

    assign rb_app_rd_data = app_rd_data;
    assign scan_app_rd_data = app_rd_data;
    assign rb_app_rd_data_end = app_rd_data_end;
    assign scan_app_rd_data_end = app_rd_data_end;
    assign rb_app_rd_data_valid = app_rd_data_valid &&
                                   (owner_count != 0) && !owner_fifo[0];
    assign scan_app_rd_data_valid = app_rd_data_valid &&
                                     (owner_count != 0) && owner_fifo[0];

    wire cmd_fire = app_en && app_rdy;
    wire data_fire = app_wdf_wren && app_wdf_rdy;
    wire write_complete = (cmd_accepted || cmd_fire) &&
                          (data_accepted || data_fire);
    wire read_fire = cmd_fire && !selected_write;
    wire response_fire = app_rd_data_valid && (owner_count != 0);

    always @(posedge clk) begin
        if (rst) begin
            active <= 1'b0;
            owner_scan <= 1'b0;
            last_owner_scan <= 1'b1;
            cmd_accepted <= 1'b0;
            data_accepted <= 1'b0;
            owner_fifo <= 8'd0;
            owner_count <= 4'd0;
            rb_grants <= 16'd0;
            scan_grants <= 16'd0;
        end else begin
            case ({read_fire, response_fire})
                2'b10: begin
                    owner_fifo[owner_count[2:0]] <= owner_scan;
                    owner_count <= owner_count + 4'd1;
                end
                2'b01: begin
                    owner_fifo <= {1'b0, owner_fifo[7:1]};
                    owner_count <= owner_count - 4'd1;
                end
                2'b11: begin
                    case (owner_count)
                        4'd1: owner_fifo <= {7'd0, owner_scan};
                        4'd2: owner_fifo <= {6'd0, owner_scan,
                                             owner_fifo[1]};
                        4'd3: owner_fifo <= {5'd0, owner_scan,
                                             owner_fifo[2:1]};
                        4'd4: owner_fifo <= {4'd0, owner_scan,
                                             owner_fifo[3:1]};
                        4'd5: owner_fifo <= {3'd0, owner_scan,
                                             owner_fifo[4:1]};
                        4'd6: owner_fifo <= {2'd0, owner_scan,
                                             owner_fifo[5:1]};
                        4'd7: owner_fifo <= {1'd0, owner_scan,
                                             owner_fifo[6:1]};
                        4'd8: owner_fifo <= {owner_scan,
                                             owner_fifo[7:1]};
                        default: owner_fifo <= owner_fifo;
                    endcase
                end
                default: begin end
            endcase

            if (!active) begin
                cmd_accepted <= 1'b0;
                data_accepted <= 1'b0;
                if ((rb_app_en || scan_app_en) &&
                    (!owner_full ||
                     (rb_app_en && (rb_app_cmd == MIG_CMD_WRITE)))) begin
                    owner_scan <= choose_scan;
                    if (choose_scan)
                        scan_grants <= scan_grants + 16'd1;
                    else
                        rb_grants <= rb_grants + 16'd1;
                    active <= 1'b1;
                end
            end else if (selected_write) begin
                if (cmd_fire)
                    cmd_accepted <= 1'b1;
                if (data_fire)
                    data_accepted <= 1'b1;
                if (write_complete) begin
                    last_owner_scan <= owner_scan;
                    active <= 1'b0;
                end
            end else if (read_fire) begin
                last_owner_scan <= owner_scan;
                active <= 1'b0;
            end
        end
    end

    assign debug_probe = {
        rb_grants, scan_grants, 20'd0, owner_count[2:0], owner_fifo[0],
        last_owner_scan, owner_scan, data_accepted, cmd_accepted, active,
        response_fire, data_fire, cmd_fire
    };
endmodule

`default_nettype wire
