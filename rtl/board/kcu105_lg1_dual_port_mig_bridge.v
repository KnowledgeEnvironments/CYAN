// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - independent LG1 RB and serial-port DDR bridge
//
// Physical LG1 VRAM has independent random and serial ports.  Keep that
// ownership boundary at the FPGA backing store by giving each port its own
// CPU/MIG mailbox, then compose their native requests with the proven ordered
// MIG arbiter.  A native transaction already accepted by DDR is never
// aborted; otherwise the two bounded clients make forward progress in round-
// robin order and read responses return through an explicit owner queue.

`default_nettype none

module kcu105_lg1_dual_port_mig_bridge #(
    parameter integer APP_ADDR_WIDTH = 28,
    parameter integer APP_DATA_WIDTH = 512,
    parameter integer APP_MASK_WIDTH = 64,
    parameter [25:0] LG1_BASE_DW_ADDR = 26'h3f80000,
    parameter integer RB_ENABLE_READ_AHEAD = 1,
    parameter integer SCAN_ENABLE_READ_AHEAD = 1,
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
    input  wire [15:0]                line_addr,
    input  wire [511:0]               line_wdata,
    input  wire [63:0]                line_wstrb,
    output wire                       line_rsp_valid,
    input  wire                       line_rsp_ready,
    output wire [511:0]               line_rsp_rdata,

    input  wire                       scan_line_valid,
    output wire                       scan_line_ready,
    input  wire [15:0]                scan_line_addr,
    output wire                       scan_line_rsp_valid,
    input  wire                       scan_line_rsp_ready,
    output wire [511:0]               scan_line_rsp_rdata,

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

    output wire [127:0]               debug_probe,
    output wire [127:0]               scan_debug_probe,
    output wire [63:0]                port_arbiter_debug,
    output wire                       cold_init_done
);
    wire [APP_ADDR_WIDTH-1:0] rb_app_addr;
    wire [2:0] rb_app_cmd;
    wire rb_app_en;
    wire rb_app_rdy;
    wire rb_app_hi_pri;
    wire [APP_DATA_WIDTH-1:0] rb_app_wdf_data;
    wire rb_app_wdf_end;
    wire [APP_MASK_WIDTH-1:0] rb_app_wdf_mask;
    wire rb_app_wdf_wren;
    wire rb_app_wdf_rdy;
    wire [APP_DATA_WIDTH-1:0] rb_app_rd_data;
    wire rb_app_rd_data_end;
    wire rb_app_rd_data_valid;
    wire rb_native_write_commit;
    wire [15:0] rb_native_write_addr;

    kcu105_lg1_mig_bridge #(
        .APP_ADDR_WIDTH(APP_ADDR_WIDTH),
        .APP_DATA_WIDTH(APP_DATA_WIDTH),
        .APP_MASK_WIDTH(APP_MASK_WIDTH),
        .LG1_BASE_DW_ADDR(LG1_BASE_DW_ADDR),
        .ENABLE_READ_AHEAD(RB_ENABLE_READ_AHEAD),
        .COLD_INIT_ENABLE(COLD_INIT_ENABLE),
        .COLD_INIT_START_LINE(COLD_INIT_START_LINE),
        .COLD_INIT_END_LINE(COLD_INIT_END_LINE)
    ) u_rb_bridge (
        .cpu_clk(cpu_clk), .cpu_rst_n(cpu_rst_n),
        .line_valid(line_valid), .line_ready(line_ready),
        .line_write_done(line_write_done), .line_write(line_write),
        .line_scan(1'b0), .line_addr(line_addr),
        .line_wdata(line_wdata), .line_wstrb(line_wstrb),
        .line_rsp_valid(line_rsp_valid),
        .line_rsp_ready(line_rsp_ready), .line_rsp_rdata(line_rsp_rdata),
        .mig_clk(mig_clk), .mig_rst(mig_rst),
        .init_calib_complete(init_calib_complete),
        .app_addr(rb_app_addr), .app_cmd(rb_app_cmd),
        .app_en(rb_app_en), .app_rdy(rb_app_rdy),
        .app_hi_pri(rb_app_hi_pri),
        .app_wdf_data(rb_app_wdf_data), .app_wdf_end(rb_app_wdf_end),
        .app_wdf_mask(rb_app_wdf_mask), .app_wdf_wren(rb_app_wdf_wren),
        .app_wdf_rdy(rb_app_wdf_rdy), .app_rd_data(rb_app_rd_data),
        .app_rd_data_end(rb_app_rd_data_end),
        .app_rd_data_valid(rb_app_rd_data_valid),
        .peer_write_commit(1'b0), .peer_write_addr(16'd0),
        .native_write_commit(rb_native_write_commit),
        .native_write_addr(rb_native_write_addr),
        .debug_probe(debug_probe), .cold_init_done(cold_init_done)
    );

    wire scan_write_done_unused;
    wire [APP_ADDR_WIDTH-1:0] scan_app_addr;
    wire [2:0] scan_app_cmd;
    wire scan_app_en;
    wire scan_app_rdy;
    wire scan_app_hi_pri;
    wire [APP_DATA_WIDTH-1:0] scan_app_wdf_data;
    wire scan_app_wdf_end;
    wire [APP_MASK_WIDTH-1:0] scan_app_wdf_mask;
    wire scan_app_wdf_wren;
    wire scan_app_wdf_rdy;
    wire [APP_DATA_WIDTH-1:0] scan_app_rd_data;
    wire scan_app_rd_data_end;
    wire scan_app_rd_data_valid;

    kcu105_lg1_mig_bridge #(
        .APP_ADDR_WIDTH(APP_ADDR_WIDTH),
        .APP_DATA_WIDTH(APP_DATA_WIDTH),
        .APP_MASK_WIDTH(APP_MASK_WIDTH),
        .LG1_BASE_DW_ADDR(LG1_BASE_DW_ADDR),
        .ENABLE_READ_AHEAD(SCAN_ENABLE_READ_AHEAD),
        .COLD_INIT_ENABLE(0)
    ) u_scan_bridge (
        .cpu_clk(cpu_clk), .cpu_rst_n(cpu_rst_n),
        .line_valid(scan_line_valid), .line_ready(scan_line_ready),
        .line_write_done(scan_write_done_unused), .line_write(1'b0),
        .line_scan(1'b1), .line_addr(scan_line_addr),
        .line_wdata(512'd0), .line_wstrb(64'd0),
        .line_rsp_valid(scan_line_rsp_valid),
        .line_rsp_ready(scan_line_rsp_ready),
        .line_rsp_rdata(scan_line_rsp_rdata),
        .mig_clk(mig_clk), .mig_rst(mig_rst),
        .init_calib_complete(init_calib_complete),
        .app_addr(scan_app_addr), .app_cmd(scan_app_cmd),
        .app_en(scan_app_en), .app_rdy(scan_app_rdy),
        .app_hi_pri(scan_app_hi_pri),
        .app_wdf_data(scan_app_wdf_data), .app_wdf_end(scan_app_wdf_end),
        .app_wdf_mask(scan_app_wdf_mask),
        .app_wdf_wren(scan_app_wdf_wren),
        .app_wdf_rdy(scan_app_wdf_rdy),
        .app_rd_data(scan_app_rd_data),
        .app_rd_data_end(scan_app_rd_data_end),
        .app_rd_data_valid(scan_app_rd_data_valid),
        .peer_write_commit(rb_native_write_commit),
        .peer_write_addr(rb_native_write_addr),
        .native_write_commit(), .native_write_addr(),
        .debug_probe(scan_debug_probe), .cold_init_done()
    );

    kcu105_lg1_port_arbiter #(
        .APP_ADDR_WIDTH(APP_ADDR_WIDTH),
        .APP_DATA_WIDTH(APP_DATA_WIDTH),
        .APP_MASK_WIDTH(APP_MASK_WIDTH)
    ) u_port_arbiter (
        .clk(mig_clk), .rst(mig_rst),
        .rb_app_addr(rb_app_addr), .rb_app_cmd(rb_app_cmd),
        .rb_app_en(rb_app_en), .rb_app_rdy(rb_app_rdy),
        .rb_app_hi_pri(rb_app_hi_pri),
        .rb_app_wdf_data(rb_app_wdf_data), .rb_app_wdf_end(rb_app_wdf_end),
        .rb_app_wdf_mask(rb_app_wdf_mask),
        .rb_app_wdf_wren(rb_app_wdf_wren),
        .rb_app_wdf_rdy(rb_app_wdf_rdy),
        .rb_app_rd_data(rb_app_rd_data),
        .rb_app_rd_data_end(rb_app_rd_data_end),
        .rb_app_rd_data_valid(rb_app_rd_data_valid),
        .scan_app_addr(scan_app_addr), .scan_app_cmd(scan_app_cmd),
        .scan_app_en(scan_app_en), .scan_app_rdy(scan_app_rdy),
        .scan_app_hi_pri(scan_app_hi_pri),
        .scan_app_wdf_data(scan_app_wdf_data),
        .scan_app_wdf_end(scan_app_wdf_end),
        .scan_app_wdf_mask(scan_app_wdf_mask),
        .scan_app_wdf_wren(scan_app_wdf_wren),
        .scan_app_wdf_rdy(scan_app_wdf_rdy),
        .scan_app_rd_data(scan_app_rd_data),
        .scan_app_rd_data_end(scan_app_rd_data_end),
        .scan_app_rd_data_valid(scan_app_rd_data_valid),
        .app_addr(app_addr), .app_cmd(app_cmd),
        .app_en(app_en), .app_rdy(app_rdy), .app_hi_pri(app_hi_pri),
        .app_wdf_data(app_wdf_data), .app_wdf_end(app_wdf_end),
        .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren),
        .app_wdf_rdy(app_wdf_rdy), .app_rd_data(app_rd_data),
        .app_rd_data_end(app_rd_data_end),
        .app_rd_data_valid(app_rd_data_valid),
        .debug_probe(port_arbiter_debug)
    );
endmodule

`default_nettype wire
