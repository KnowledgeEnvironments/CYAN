// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// KCU105 CYAN SoC target.
//
// This keeps the board-bound KCU105 DDR4 MIG instance and replaces the vendor
// traffic generator with the CYAN IP20 CPU/MC/HPC1/PROM/SCC cluster.

`timescale 1ps/1ps
`default_nettype none

module kcu105_cyan_soc_top #(
    parameter nCK_PER_CLK = 4,
    parameter APP_DATA_WIDTH = 512,
    parameter APP_MASK_WIDTH = 64,
    parameter SIMULATION = "FALSE",
    parameter PROM_INIT_FILE = "UNSPECIFIED",
    parameter PROM_INIT_B00 = "UNSPECIFIED",
    parameter PROM_INIT_B01 = "UNSPECIFIED",
    parameter PROM_INIT_B02 = "UNSPECIFIED",
    parameter PROM_INIT_B03 = "UNSPECIFIED",
    parameter PROM_INIT_B04 = "UNSPECIFIED",
    parameter PROM_INIT_B05 = "UNSPECIFIED",
    parameter PROM_INIT_B06 = "UNSPECIFIED",
    parameter PROM_INIT_B07 = "UNSPECIFIED",
    parameter PROM_INIT_B08 = "UNSPECIFIED",
    parameter PROM_INIT_B09 = "UNSPECIFIED",
    parameter PROM_INIT_B0A = "UNSPECIFIED",
    parameter PROM_INIT_B0B = "UNSPECIFIED",
    parameter PROM_INIT_B0C = "UNSPECIFIED",
    parameter PROM_INIT_B0D = "UNSPECIFIED",
    parameter PROM_INIT_B0E = "UNSPECIFIED",
    parameter PROM_INIT_B0F = "UNSPECIFIED",
    // The MIG UI clock is 300 MHz.  A divide of three gives the R4000/cache
    // island its routed 100 MHz target; six is the source-compatible 50 MHz
    // fallback.  The rest of the IP20 machine remains fixed at 50 MHz.
    parameter integer CPU_CLK_DIVIDE = 3,
    parameter EEPROM_INIT_FILE = ""
) (
    input  wire             sys_rst,

    output wire [7:0]       gpio_led,
    output wire             sm_fan_pwm,
    input  wire             c0_sys_clk_p,
    input  wire             c0_sys_clk_n,
    output wire             c0_ddr4_act_n,
    output wire [16:0]      c0_ddr4_adr,
    output wire [1:0]       c0_ddr4_ba,
    output wire [0:0]       c0_ddr4_bg,
    output wire [0:0]       c0_ddr4_cke,
    output wire [0:0]       c0_ddr4_odt,
    output wire [0:0]       c0_ddr4_cs_n,
    output wire [0:0]       c0_ddr4_ck_t,
    output wire [0:0]       c0_ddr4_ck_c,
    output wire             c0_ddr4_reset_n,
    inout  wire [7:0]       c0_ddr4_dm_dbi_n,
    inout  wire [63:0]      c0_ddr4_dq,
    inout  wire [7:0]       c0_ddr4_dqs_t,
    inout  wire [7:0]       c0_ddr4_dqs_c,

    input  wire             uart_rxd,
    output wire             uart_txd,
    input  wire             uart_cts,
    output wire             uart_rts,

    output wire             sgmii_txn,
    output wire             sgmii_txp,
    input  wire             sgmii_rxn,
    input  wire             sgmii_rxp,
    input  wire             sgmii_refclk_n,
    input  wire             sgmii_refclk_p,
    output wire             phy_mdc,
    inout  wire             phy_mdio,
    output wire             phy_rst_n,

    output wire             sdio_cmd_fpga,
    output wire             sdio_clk_fpga,
    input  wire             sdio_data0_fpga,
    inout  wire             sdio_data1_fpga,
    inout  wire             sdio_data2_fpga,
    output wire             sdio_data3_fpga,
    input  wire             sdio_cd_fpga,

    output wire             hdmi_out_clk,
    output wire             hdmi_hsync,
    output wire             hdmi_vsync,
    output wire             hdmi_data_e,
    output wire [15:0]      hdmi_data,
    output wire             hdmi_spdif,
    output wire             hdmi_iic_mux_reset_n,
    inout  wire             hdmi_iic_scl,
    inout  wire             hdmi_iic_sda
);
    localparam APP_ADDR_WIDTH = 28;

    wire [APP_ADDR_WIDTH-1:0] c0_ddr4_app_addr;
    wire [2:0]                c0_ddr4_app_cmd;
    wire                      c0_ddr4_app_en;
    wire                      c0_ddr4_app_hi_pri;
    wire [APP_DATA_WIDTH-1:0] c0_ddr4_app_wdf_data;
    wire                      c0_ddr4_app_wdf_end;
    wire [APP_MASK_WIDTH-1:0] c0_ddr4_app_wdf_mask;
    wire                      c0_ddr4_app_wdf_wren;
    wire [APP_DATA_WIDTH-1:0] c0_ddr4_app_rd_data;
    wire                      c0_ddr4_app_rd_data_end;
    wire                      c0_ddr4_app_rd_data_valid;
    wire                      c0_ddr4_app_rdy;
    wire                      c0_ddr4_app_wdf_rdy;
    wire                      c0_ddr4_clk;
    wire                      c0_ddr4_rst;
    wire                      dbg_clk;
    wire                      c0_init_calib_complete;
    wire [511:0]              dbg_bus;
    wire                      addn_ui_clkout1_unused;

    wire c0_ddr4_reset_n_int;
    assign c0_ddr4_reset_n = c0_ddr4_reset_n_int;

    kcu105_ddr4 u_kcu105_ddr4 (
        .sys_rst(sys_rst),
        .c0_sys_clk_p(c0_sys_clk_p),
        .c0_sys_clk_n(c0_sys_clk_n),
        .c0_init_calib_complete(c0_init_calib_complete),
        .c0_ddr4_act_n(c0_ddr4_act_n),
        .c0_ddr4_adr(c0_ddr4_adr),
        .c0_ddr4_ba(c0_ddr4_ba),
        .c0_ddr4_bg(c0_ddr4_bg),
        .c0_ddr4_cke(c0_ddr4_cke),
        .c0_ddr4_odt(c0_ddr4_odt),
        .c0_ddr4_cs_n(c0_ddr4_cs_n),
        .c0_ddr4_ck_t(c0_ddr4_ck_t),
        .c0_ddr4_ck_c(c0_ddr4_ck_c),
        .c0_ddr4_reset_n(c0_ddr4_reset_n_int),
        .c0_ddr4_dm_dbi_n(c0_ddr4_dm_dbi_n),
        .c0_ddr4_dq(c0_ddr4_dq),
        .c0_ddr4_dqs_c(c0_ddr4_dqs_c),
        .c0_ddr4_dqs_t(c0_ddr4_dqs_t),
        .c0_ddr4_ui_clk(c0_ddr4_clk),
        .c0_ddr4_ui_clk_sync_rst(c0_ddr4_rst),
        .addn_ui_clkout1(addn_ui_clkout1_unused),
        .dbg_clk(dbg_clk),
        .c0_ddr4_app_addr(c0_ddr4_app_addr),
        .c0_ddr4_app_cmd(c0_ddr4_app_cmd),
        .c0_ddr4_app_en(c0_ddr4_app_en),
        .c0_ddr4_app_hi_pri(c0_ddr4_app_hi_pri),
        .c0_ddr4_app_wdf_data(c0_ddr4_app_wdf_data),
        .c0_ddr4_app_wdf_end(c0_ddr4_app_wdf_end),
        .c0_ddr4_app_wdf_mask(c0_ddr4_app_wdf_mask),
        .c0_ddr4_app_wdf_wren(c0_ddr4_app_wdf_wren),
        .c0_ddr4_app_rd_data(c0_ddr4_app_rd_data),
        .c0_ddr4_app_rd_data_end(c0_ddr4_app_rd_data_end),
        .c0_ddr4_app_rd_data_valid(c0_ddr4_app_rd_data_valid),
        .c0_ddr4_app_rdy(c0_ddr4_app_rdy),
        .c0_ddr4_app_wdf_rdy(c0_ddr4_app_wdf_rdy),
        .dbg_bus(dbg_bus)
    );

    // Historical name retained for the 50 MHz whole-machine clock so the
    // existing board debug/CDC hierarchy remains stable.
    wire cpu_clk;
    wire cpu_island_clk;
    wire soc_rst_n;
    BUFGCE_DIV #(
        .BUFGCE_DIVIDE(6),
        .IS_CE_INVERTED(1'b0),
        .IS_CLR_INVERTED(1'b0),
        .IS_I_INVERTED(1'b0),
        .SIM_DEVICE("ULTRASCALE")
    ) u_machine_clk_div (
        .I(c0_ddr4_clk),
        .CE(1'b1),
        .CLR(c0_ddr4_rst),
        .O(cpu_clk)
    );

    BUFGCE_DIV #(
        .BUFGCE_DIVIDE(CPU_CLK_DIVIDE),
        .IS_CE_INVERTED(1'b0),
        .IS_CLR_INVERTED(1'b0),
        .IS_I_INVERTED(1'b0),
        .SIM_DEVICE("ULTRASCALE")
    ) u_cpu_island_clk_div (
        .I(c0_ddr4_clk),
        .CE(1'b1),
        .CLR(c0_ddr4_rst),
        .O(cpu_island_clk)
    );

    // The MIG UI clock is 300 MHz.  Keep the CYAN machine at 50 MHz and
    // derive the VESA 1024x768 pixel clock independently: 300*13/3/20=65 MHz.
    wire video_clk_mmcm;
    wire video_clk_fb;
    wire video_clk_fb_buf;
    wire video_clk;
    wire video_mmcm_locked;
    MMCME3_BASE #(
        .BANDWIDTH("OPTIMIZED"),
        .CLKIN1_PERIOD(3.333),
        .DIVCLK_DIVIDE(3),
        .CLKFBOUT_MULT_F(13.000),
        .CLKOUT0_DIVIDE_F(20.000),
        .STARTUP_WAIT("FALSE")
    ) u_video_mmcm (
        .CLKIN1(c0_ddr4_clk),
        .CLKFBIN(video_clk_fb_buf),
        .RST(c0_ddr4_rst),
        .PWRDWN(1'b0),
        .CLKFBOUT(video_clk_fb),
        .CLKOUT0(video_clk_mmcm),
        .LOCKED(video_mmcm_locked)
    );
    BUFG u_video_clk_buf (.I(video_clk_mmcm), .O(video_clk));
    BUFG u_video_clk_fb_buf (.I(video_clk_fb), .O(video_clk_fb_buf));

    // The Indigo LG1 serial section is a distinct 64 MHz clock domain.  Keep
    // REX, HPC, MC, and scanline production on the 50 MHz machine clock.  The
    // 300.120 MHz MIG UI reference gives 300.120*16/5/15 = 64.0256 MHz,
    // preserving the original machine's 64:50 serial-to-system ratio.
    wire lg1_serial_clk_mmcm;
    wire lg1_serial_clk_fb;
    wire lg1_serial_clk_fb_buf;
    wire lg1_serial_clk;
    wire lg1_serial_mmcm_locked;
    MMCME3_BASE #(
        .BANDWIDTH("OPTIMIZED"),
        .CLKIN1_PERIOD(3.332),
        .DIVCLK_DIVIDE(5),
        .CLKFBOUT_MULT_F(16.000),
        .CLKOUT0_DIVIDE_F(15.000),
        .STARTUP_WAIT("FALSE")
    ) u_lg1_serial_mmcm (
        .CLKIN1(c0_ddr4_clk),
        .CLKFBIN(lg1_serial_clk_fb_buf),
        .RST(c0_ddr4_rst),
        .PWRDWN(1'b0),
        .CLKFBOUT(lg1_serial_clk_fb),
        .CLKOUT0(lg1_serial_clk_mmcm),
        .LOCKED(lg1_serial_mmcm_locked)
    );
    BUFG u_lg1_serial_clk_buf (
        .I(lg1_serial_clk_mmcm), .O(lg1_serial_clk));
    BUFG u_lg1_serial_clk_fb_buf (
        .I(lg1_serial_clk_fb), .O(lg1_serial_clk_fb_buf));

    reg [7:0] cpu_reset_sync;
    wire mig_ready = c0_init_calib_complete && !c0_ddr4_rst;
    wire lg1_cold_init_done;
    reg [2:0] lg1_cold_init_sync;
    always @(posedge cpu_clk or negedge mig_ready) begin
        if (!mig_ready)
            lg1_cold_init_sync <= 3'b000;
        else
            lg1_cold_init_sync <= {lg1_cold_init_sync[1:0],
                                   lg1_cold_init_done};
    end
    wire cpu_release_ready = mig_ready && lg1_cold_init_sync[2];
    always @(posedge cpu_clk or negedge cpu_release_ready) begin
        if (!cpu_release_ready)
            cpu_reset_sync <= 8'h00;
        else
            cpu_reset_sync <= {cpu_reset_sync[6:0], 1'b1};
    end
    wire cpu_rst_n = &cpu_reset_sync;

    // Asynchronous assertion prevents any serial activity after either its
    // clock source or the machine is lost.  Deassertion is local to the
    // 64 MHz domain; ip20_kcu_soc_core applies its later EEPROM/SoC release
    // through a second local synchronizer.
    (* ASYNC_REG = "TRUE" *) reg [3:0] lg1_serial_reset_sync;
    wire lg1_serial_release = cpu_rst_n && lg1_serial_mmcm_locked;
    always @(posedge lg1_serial_clk or negedge lg1_serial_release) begin
        if (!lg1_serial_release)
            lg1_serial_reset_sync <= 4'b0000;
        else
            lg1_serial_reset_sync <= {lg1_serial_reset_sync[2:0], 1'b1};
    end
    wire lg1_serial_rst_n = &lg1_serial_reset_sync;

    reg [3:0] video_reset_sync;
    always @(posedge video_clk or negedge video_mmcm_locked) begin
        if (!video_mmcm_locked)
            video_reset_sync <= 4'b0000;
        else
            video_reset_sync <= {video_reset_sync[2:0], 1'b1};
    end
    wire video_rst_n = &video_reset_sync;

    wire [25:0] ram_addr;
    wire [63:0] ram_wdata;
    wire [ 7:0] ram_wmask;
    wire        ram_wen;
    wire        ram_valid;
    wire [63:0] ram_rdata;
    wire        ram_ready;
    wire [255:0] mig_bridge_debug;

    wire [APP_ADDR_WIDTH-1:0] cpu_app_addr;
    wire [2:0]                cpu_app_cmd;
    wire                      cpu_app_en;
    wire                      cpu_app_rdy;
    wire                      cpu_app_hi_pri;
    wire [APP_DATA_WIDTH-1:0] cpu_app_wdf_data;
    wire                      cpu_app_wdf_end;
    wire [APP_MASK_WIDTH-1:0] cpu_app_wdf_mask;
    wire                      cpu_app_wdf_wren;
    wire                      cpu_app_wdf_rdy;
    wire [APP_DATA_WIDTH-1:0] cpu_app_rd_data;
    wire                      cpu_app_rd_data_end;
    wire                      cpu_app_rd_data_valid;

    mc_mig_bridge #(
        .APP_ADDR_WIDTH(APP_ADDR_WIDTH),
        .APP_DATA_WIDTH(APP_DATA_WIDTH),
        .APP_MASK_WIDTH(APP_MASK_WIDTH)
    ) u_ram_bridge (
        .cpu_clk(cpu_clk),
        .cpu_rst_n(cpu_rst_n),
        .mig_clk(c0_ddr4_clk),
        .mig_rst(c0_ddr4_rst),
        .init_calib_complete(c0_init_calib_complete),
        .cpu_valid(ram_valid),
        .cpu_wen(ram_wen),
        .cpu_addr(ram_addr),
        .cpu_wdata(ram_wdata),
        .cpu_wmask(ram_wmask),
        .cpu_rdata(ram_rdata),
        .cpu_ready(ram_ready),
        .app_addr(cpu_app_addr),
        .app_cmd(cpu_app_cmd),
        .app_en(cpu_app_en),
        .app_rdy(cpu_app_rdy),
        .app_hi_pri(cpu_app_hi_pri),
        .app_wdf_data(cpu_app_wdf_data),
        .app_wdf_end(cpu_app_wdf_end),
        .app_wdf_mask(cpu_app_wdf_mask),
        .app_wdf_wren(cpu_app_wdf_wren),
        .app_wdf_rdy(cpu_app_wdf_rdy),
        .app_rd_data(cpu_app_rd_data),
        .app_rd_data_end(cpu_app_rd_data_end),
        .app_rd_data_valid(cpu_app_rd_data_valid),
        .debug_probe(mig_bridge_debug)
    );

    wire         lg1_line_valid;
    wire         lg1_line_ready;
    wire         lg1_line_write_done;
    wire         lg1_line_write;
    wire [15:0]  lg1_line_addr;
    wire [511:0] lg1_line_wdata;
    wire [63:0]  lg1_line_wstrb;
    wire         lg1_line_rsp_valid;
    wire         lg1_line_rsp_ready;
    wire [511:0] lg1_line_rsp_rdata;

    wire         soc_lg1_line_valid;
    wire         soc_lg1_line_ready;
    wire         soc_lg1_line_write_done;
    wire         soc_lg1_line_write;
    wire [15:0]  soc_lg1_line_addr;
    wire [511:0] soc_lg1_line_wdata;
    wire [63:0]  soc_lg1_line_wstrb;
    wire         soc_lg1_line_rsp_valid;
    wire         soc_lg1_line_rsp_ready;
    wire [511:0] soc_lg1_line_rsp_rdata;
    wire         soc_lg1_scan_mem_valid;
    wire         soc_lg1_scan_mem_ready;
    wire [15:0]  soc_lg1_scan_mem_addr;
    wire         soc_lg1_scan_mem_rsp_valid;
    wire         soc_lg1_scan_mem_rsp_ready;
    wire [511:0] soc_lg1_scan_mem_rsp_rdata;
    wire [511:0] lg1_capture_data;
    wire [127:0] lg1_capture_reader_status;
    wire [31:0]  lg1_capture_bist_status;
    wire         soc_video_de;
    wire         soc_video_hsync;
    wire         soc_video_vsync;
    wire [7:0]   soc_video_r;
    wire [7:0]   soc_video_g;
    wire [7:0]   soc_video_b;
    wire         soc_video_valid;
    wire         soc_video_frame_start;
    wire [9:0]   soc_video_x;
    wire [9:0]   soc_video_y;
    wire [7:0]   soc_video_raw_index;
    wire [4:0]   soc_video_did;
    wire [7:0]   soc_video_display_index;
    wire [15:0]  soc_video_xmap_mode;
    wire [1:0]   soc_video_palette_bank;
    wire [2:0]   soc_video_resolver_class;
    wire [15:0]  soc_video_fetch_low_signature;
    wire [15:0]  soc_video_fetch_high_signature;
    wire [7:0]   soc_video_fetch_signature_generation;
    wire         soc_video_fetch_signature_valid;
    wire [91:0]  soc_video_resolver_telemetry;
    wire [91:0]  soc_video_resolver_detail;
    wire         soc_video_resolver_frame_done;
    wire [31:0]  soc_video_underrun_count;
    wire [31:0]  soc_video_protocol_error_count;
    wire [127:0] soc_gfx_retrace_status;
    wire         soc_lg1_config_access_valid;
    wire [87:0]  soc_lg1_config_access_payload;
    wire [ 2:0]  soc_lg1_config_current_sel;
    wire         hdmi_video_de;
    wire         hdmi_video_hsync;
    wire         hdmi_video_vsync;
    wire [7:0]   hdmi_video_r;
    wire [7:0]   hdmi_video_g;
    wire [7:0]   hdmi_video_b;
    wire         hdmi_iic_scl_drive_low;
    wire         hdmi_iic_sda_drive_low;
    wire [127:0] hdmi_status;
    wire [23:0]  dsp_audio_sample_data;
    wire         dsp_audio_sample_valid;
    wire [31:0]  dsp_audio_channel_status;
    wire         dsp_audio_channel_status_valid;
    wire [31:0]  dsp_audio_user_data;
    wire         dsp_audio_user_valid;
    wire         dsp_audio_stream_active;
    wire [5:0]   dsp_audio_rate_select;
    wire         dsp_audio_sample_ready;
    wire [127:0] dsp_audio_status;

    wire [APP_ADDR_WIDTH-1:0] gfx_app_addr;
    wire [2:0]                gfx_app_cmd;
    wire                      gfx_app_en;
    wire                      gfx_app_rdy;
    wire                      gfx_app_hi_pri;
    wire [APP_DATA_WIDTH-1:0] gfx_app_wdf_data;
    wire                      gfx_app_wdf_end;
    wire [APP_MASK_WIDTH-1:0] gfx_app_wdf_mask;
    wire                      gfx_app_wdf_wren;
    wire                      gfx_app_wdf_rdy;
    wire [APP_DATA_WIDTH-1:0] gfx_app_rd_data;
    wire                      gfx_app_rd_data_end;
    wire                      gfx_app_rd_data_valid;
    wire [APP_ADDR_WIDTH-1:0] video_app_addr;
    wire [2:0]                video_app_cmd;
    wire                      video_app_en;
    wire                      video_app_rdy;
    wire                      video_app_hi_pri;
    wire [APP_DATA_WIDTH-1:0] video_app_wdf_data;
    wire                      video_app_wdf_end;
    wire [APP_MASK_WIDTH-1:0] video_app_wdf_mask;
    wire                      video_app_wdf_wren;
    wire                      video_app_wdf_rdy;
    wire [APP_DATA_WIDTH-1:0] video_app_rd_data;
    wire                      video_app_rd_data_end;
    wire                      video_app_rd_data_valid;
    wire [127:0]              video_resampler_status;
    wire [127:0]              video_resampler_signature_status;
    wire [127:0]              video_resampler_resolver_status;
    wire [127:0]              video_resampler_resolver_detail_status;
    wire [127:0]              video_resampler_visual_stage_status;
    wire [127:0]              video_resampler_stage_signature_status;
    wire [127:0]              video_resampler_scan_service_status;
    wire [127:0]              video_resampler_scan_latency_status;
    wire [1:0]                mgmt_capture_command;
    wire                      mgmt_capture_command_toggle;
    wire [15:0]               mgmt_capture_word_index;
    wire                      mgmt_capture_ack_toggle;
    wire [127:0]              mgmt_capture_info;
    wire [511:0]              mgmt_capture_word_data;
    wire                      mgmt_capture_word_valid;
    wire                      mgmt_video_status_request_toggle;
    wire                      mgmt_video_status_ack_toggle;
    wire [1:0]                mgmt_lg1_trace_command;
    wire [7:0]                mgmt_lg1_trace_command_tag;
    wire                      mgmt_lg1_trace_command_valid;
    wire                      mgmt_lg1_trace_command_ready;
    wire                      mgmt_lg1_trace_command_ack;
    wire                      mgmt_lg1_trace_command_ok;
    wire [127:0]              mgmt_lg1_trace_status;
    wire [15:0]               mgmt_lg1_trace_read_ordinal;
    wire                      mgmt_lg1_trace_read_request;
    wire [95:0]               mgmt_lg1_trace_read_data;
    wire                      mgmt_lg1_trace_read_ack;
    wire                      mgmt_lg1_trace_read_valid;
    wire [127:0]              lg1_mig_debug;
    wire [127:0]              lg1_scan_mig_debug;
    wire [63:0]               lg1_port_arbiter_debug;
    wire [63:0]               mig_arb_debug;

    kcu105_lg1_capture_reader u_lg1_capture_reader (
        .clk(cpu_clk),
        .rst_n(cpu_rst_n),
        .core_rst_n(soc_rst_n),
        .hold(soc_lg1_capture_hold),
        .reset_toggle(soc_lg1_capture_reset_toggle),
        .next_toggle(soc_lg1_capture_next_toggle),
        .core_valid(soc_lg1_line_valid),
        .core_ready(soc_lg1_line_ready),
        .core_write_done(soc_lg1_line_write_done),
        .core_write(soc_lg1_line_write),
        .core_addr(soc_lg1_line_addr),
        .core_wdata(soc_lg1_line_wdata),
        .core_wstrb(soc_lg1_line_wstrb),
        .core_rsp_valid(soc_lg1_line_rsp_valid),
        .core_rsp_ready(soc_lg1_line_rsp_ready),
        .core_rsp_rdata(soc_lg1_line_rsp_rdata),
        .mem_valid(lg1_line_valid),
        .mem_ready(lg1_line_ready),
        .mem_write_done(lg1_line_write_done),
        .mem_write(lg1_line_write),
        .mem_addr(lg1_line_addr),
        .mem_wdata(lg1_line_wdata),
        .mem_wstrb(lg1_line_wstrb),
        .mem_rsp_valid(lg1_line_rsp_valid),
        .mem_rsp_ready(lg1_line_rsp_ready),
        .mem_rsp_rdata(lg1_line_rsp_rdata),
        .capture_data(lg1_capture_data),
        .capture_status(lg1_capture_reader_status)
    );

    kcu105_lg1_dual_port_mig_bridge #(
        .APP_ADDR_WIDTH(APP_ADDR_WIDTH),
        .APP_DATA_WIDTH(APP_DATA_WIDTH),
        .APP_MASK_WIDTH(APP_MASK_WIDTH),
        .LG1_BASE_DW_ADDR(26'h3f80000),
        // The real PROM paints only the CI8 pixel plane and leaves popup/CID
        // storage untouched.  Make the FPGA cold-attach state deterministic
        // before releasing the MIPS, while preserving warm guest resets.
        .COLD_INIT_ENABLE(1),
        .COLD_INIT_START_LINE(16'h8000),
        .COLD_INIT_END_LINE(16'hffff)
    ) u_lg1_mig_bridge (
        .cpu_clk(cpu_clk),
        .cpu_rst_n(cpu_rst_n),
        .line_valid(lg1_line_valid),
        .line_ready(lg1_line_ready),
        .line_write_done(lg1_line_write_done),
        .line_write(lg1_line_write),
        .line_addr(lg1_line_addr),
        .line_wdata(lg1_line_wdata),
        .line_wstrb(lg1_line_wstrb),
        .line_rsp_valid(lg1_line_rsp_valid),
        .line_rsp_ready(lg1_line_rsp_ready),
        .line_rsp_rdata(lg1_line_rsp_rdata),
        .scan_line_valid(soc_lg1_scan_mem_valid),
        .scan_line_ready(soc_lg1_scan_mem_ready),
        .scan_line_addr(soc_lg1_scan_mem_addr),
        .scan_line_rsp_valid(soc_lg1_scan_mem_rsp_valid),
        .scan_line_rsp_ready(soc_lg1_scan_mem_rsp_ready),
        .scan_line_rsp_rdata(soc_lg1_scan_mem_rsp_rdata),
        .mig_clk(c0_ddr4_clk),
        .mig_rst(c0_ddr4_rst),
        .init_calib_complete(c0_init_calib_complete),
        .app_addr(gfx_app_addr),
        .app_cmd(gfx_app_cmd),
        .app_en(gfx_app_en),
        .app_rdy(gfx_app_rdy),
        .app_hi_pri(gfx_app_hi_pri),
        .app_wdf_data(gfx_app_wdf_data),
        .app_wdf_end(gfx_app_wdf_end),
        .app_wdf_mask(gfx_app_wdf_mask),
        .app_wdf_wren(gfx_app_wdf_wren),
        .app_wdf_rdy(gfx_app_wdf_rdy),
        .app_rd_data(gfx_app_rd_data),
        .app_rd_data_end(gfx_app_rd_data_end),
        .app_rd_data_valid(gfx_app_rd_data_valid),
        .debug_probe(lg1_mig_debug),
        .scan_debug_probe(lg1_scan_mig_debug),
        .port_arbiter_debug(lg1_port_arbiter_debug),
        .cold_init_done(lg1_cold_init_done)
    );

    kcu105_hdmi_frame_resampler #(
        .APP_ADDR_WIDTH(APP_ADDR_WIDTH),
        .APP_DATA_WIDTH(APP_DATA_WIDTH),
        .APP_MASK_WIDTH(APP_MASK_WIDTH),
        // Three 3 MiB RGB888 banks immediately below the LG1 4 MiB window.
        .FRAME_BASE_DW_ADDR(26'h3e60000),
        .FRAME_BANK_DW_STRIDE(26'h0060000)
    ) u_hdmi_frame_resampler (
        .src_clk(lg1_serial_clk),
        .src_rst_n(lg1_serial_rst_n),
        .src_valid(soc_video_valid),
        .src_frame_start(soc_video_frame_start),
        .src_x(soc_video_x),
        .src_y(soc_video_y),
        .src_raw_index(soc_video_raw_index),
        .src_did(soc_video_did),
        .src_xmap_mode(soc_video_xmap_mode),
        .src_palette_bank(soc_video_palette_bank),
        .src_resolver_class(soc_video_resolver_class),
        .src_display_index(soc_video_display_index),
        .src_fetch_low_signature(soc_video_fetch_low_signature),
        .src_fetch_high_signature(soc_video_fetch_high_signature),
        .src_fetch_signature_generation(
            soc_video_fetch_signature_generation),
        .src_fetch_signature_valid(soc_video_fetch_signature_valid),
        .src_resolver_telemetry(soc_video_resolver_telemetry),
        .src_resolver_detail(soc_video_resolver_detail),
        .src_resolver_frame_done(soc_video_resolver_frame_done),
        .src_r(soc_video_r),
        .src_g(soc_video_g),
        .src_b(soc_video_b),
        .src_underrun_count(soc_video_underrun_count),
        .src_protocol_error_count(soc_video_protocol_error_count),
        .video_clk(video_clk),
        .video_rst_n(video_rst_n),
        .video_de(hdmi_video_de),
        .video_hsync(hdmi_video_hsync),
        .video_vsync(hdmi_video_vsync),
        .video_r(hdmi_video_r),
        .video_g(hdmi_video_g),
        .video_b(hdmi_video_b),
        .mig_clk(c0_ddr4_clk),
        .mig_rst(c0_ddr4_rst),
        .init_calib_complete(c0_init_calib_complete),
        .app_addr(video_app_addr),
        .app_cmd(video_app_cmd),
        .app_en(video_app_en),
        .app_rdy(video_app_rdy),
        .app_hi_pri(video_app_hi_pri),
        .app_wdf_data(video_app_wdf_data),
        .app_wdf_end(video_app_wdf_end),
        .app_wdf_mask(video_app_wdf_mask),
        .app_wdf_wren(video_app_wdf_wren),
        .app_wdf_rdy(video_app_wdf_rdy),
        .app_rd_data(video_app_rd_data),
        .app_rd_data_end(video_app_rd_data_end),
        .app_rd_data_valid(video_app_rd_data_valid),
        .capture_command(mgmt_capture_command),
        .capture_command_toggle(mgmt_capture_command_toggle),
        .capture_word_index(mgmt_capture_word_index),
        .capture_ack_toggle(mgmt_capture_ack_toggle),
        .capture_info(mgmt_capture_info),
        .capture_word_data(mgmt_capture_word_data),
        .capture_word_valid(mgmt_capture_word_valid),
        .status_request_toggle(mgmt_video_status_request_toggle),
        .status_ack_toggle(mgmt_video_status_ack_toggle),
        .status(video_resampler_status),
        .signature_status(video_resampler_signature_status),
        .stage_signature_status(video_resampler_stage_signature_status),
        .resolver_status(video_resampler_resolver_status),
        .resolver_detail_status(video_resampler_resolver_detail_status),
        .visual_stage_status(video_resampler_visual_stage_status),
        .scan_service_status(video_resampler_scan_service_status),
        .scan_latency_status(video_resampler_scan_latency_status)
    );

    kcu105_mig_app_arbiter #(
        .APP_ADDR_WIDTH(APP_ADDR_WIDTH),
        .APP_DATA_WIDTH(APP_DATA_WIDTH),
        .APP_MASK_WIDTH(APP_MASK_WIDTH)
    ) u_mig_app_arbiter (
        .clk(c0_ddr4_clk),
        .rst(c0_ddr4_rst),
        .cpu_app_addr(cpu_app_addr),
        .cpu_app_cmd(cpu_app_cmd),
        .cpu_app_en(cpu_app_en),
        .cpu_app_rdy(cpu_app_rdy),
        .cpu_app_hi_pri(cpu_app_hi_pri),
        .cpu_app_wdf_data(cpu_app_wdf_data),
        .cpu_app_wdf_end(cpu_app_wdf_end),
        .cpu_app_wdf_mask(cpu_app_wdf_mask),
        .cpu_app_wdf_wren(cpu_app_wdf_wren),
        .cpu_app_wdf_rdy(cpu_app_wdf_rdy),
        .cpu_app_rd_data(cpu_app_rd_data),
        .cpu_app_rd_data_end(cpu_app_rd_data_end),
        .cpu_app_rd_data_valid(cpu_app_rd_data_valid),
        .gfx_app_addr(gfx_app_addr),
        .gfx_app_cmd(gfx_app_cmd),
        .gfx_app_en(gfx_app_en),
        .gfx_app_rdy(gfx_app_rdy),
        .gfx_app_hi_pri(gfx_app_hi_pri),
        .gfx_app_wdf_data(gfx_app_wdf_data),
        .gfx_app_wdf_end(gfx_app_wdf_end),
        .gfx_app_wdf_mask(gfx_app_wdf_mask),
        .gfx_app_wdf_wren(gfx_app_wdf_wren),
        .gfx_app_wdf_rdy(gfx_app_wdf_rdy),
        .gfx_app_rd_data(gfx_app_rd_data),
        .gfx_app_rd_data_end(gfx_app_rd_data_end),
        .gfx_app_rd_data_valid(gfx_app_rd_data_valid),
        .video_app_addr(video_app_addr),
        .video_app_cmd(video_app_cmd),
        .video_app_en(video_app_en),
        .video_app_rdy(video_app_rdy),
        .video_app_hi_pri(video_app_hi_pri),
        .video_app_wdf_data(video_app_wdf_data),
        .video_app_wdf_end(video_app_wdf_end),
        .video_app_wdf_mask(video_app_wdf_mask),
        .video_app_wdf_wren(video_app_wdf_wren),
        .video_app_wdf_rdy(video_app_wdf_rdy),
        .video_app_rd_data(video_app_rd_data),
        .video_app_rd_data_end(video_app_rd_data_end),
        .video_app_rd_data_valid(video_app_rd_data_valid),
        .mig_app_addr(c0_ddr4_app_addr),
        .mig_app_cmd(c0_ddr4_app_cmd),
        .mig_app_en(c0_ddr4_app_en),
        .mig_app_rdy(c0_ddr4_app_rdy),
        .mig_app_hi_pri(c0_ddr4_app_hi_pri),
        .mig_app_wdf_data(c0_ddr4_app_wdf_data),
        .mig_app_wdf_end(c0_ddr4_app_wdf_end),
        .mig_app_wdf_mask(c0_ddr4_app_wdf_mask),
        .mig_app_wdf_wren(c0_ddr4_app_wdf_wren),
        .mig_app_wdf_rdy(c0_ddr4_app_wdf_rdy),
        .mig_app_rd_data(c0_ddr4_app_rd_data),
        .mig_app_rd_data_end(c0_ddr4_app_rd_data_end),
        .mig_app_rd_data_valid(c0_ddr4_app_rd_data_valid),
        .debug_probe(mig_arb_debug)
    );

    wire [7:0]  enet_tx_data;
    wire        enet_tx_valid;
    wire        enet_tx_sof;
    wire        enet_tx_eof;
    wire        enet_tx_ready;
    wire [7:0]  enet_rx_data;
    wire        enet_rx_valid;
    wire        enet_rx_sof;
    wire        enet_rx_eof;
    wire        enet_rx_ready;
    wire [7:0]  phy_enet_tx_data;
    wire        phy_enet_tx_valid;
    wire        phy_enet_tx_sof;
    wire        phy_enet_tx_eof;
    wire        phy_enet_tx_ready;
    wire [7:0]  phy_enet_rx_data;
    wire        phy_enet_rx_valid;
    wire        phy_enet_rx_sof;
    wire        phy_enet_rx_eof;
    wire        phy_enet_rx_ready;
    wire [7:0]  mgmt_rx_data;
    wire        mgmt_rx_valid;
    wire        mgmt_rx_sof;
    wire        mgmt_rx_eof;
    wire        mgmt_rx_ready;
    wire [7:0]  mgmt_tx_data;
    wire        mgmt_tx_valid;
    wire        mgmt_tx_sof;
    wire        mgmt_tx_eof;
    wire        mgmt_tx_ready;
    wire        enet_link_up;
    wire        enet_pcs_config_done;
    wire [255:0] enet_board_debug;

    wire [3:0]  soc_phase;
    wire        soc_heartbeat;
    wire [31:0] soc_debug_pc;
    wire [31:0] soc_debug_mem_addr;
    wire [31:0] soc_status_flags;
    wire [127:0] soc_debug_probe;
    wire [127:0] soc_eeprom_debug_probe;
    wire [255:0] soc_cpu_flow_debug_probe;
    wire [255:0] soc_cpu_exception_debug_probe;
    wire [511:0] soc_cpu_gpr_debug_probe;
    wire [7:0]   host_kbd_data;
    wire         host_kbd_valid;
    wire         host_kbd_ready;
    wire [7:0]   host_mouse_data;
    wire         host_mouse_valid;
    wire         host_mouse_ready;
    wire [7:0]   kbd_tx_event_data;
    wire         kbd_tx_event_valid;
    wire         kbd_tx_event_ready;
    wire [7:0]   mouse_tx_event_data;
    wire         mouse_tx_event_valid;
    wire         mouse_tx_event_ready;
    wire [7:0]   kbd_rx_event_data;
    wire         kbd_rx_event_valid;
    wire         kbd_rx_event_ready;
    wire [7:0]   mouse_rx_event_data;
    wire         mouse_rx_event_valid;
    wire         mouse_rx_event_ready;
    wire [15:0]  host_kbd_queued;
    wire [15:0]  host_mouse_queued;
    wire [31:0]  kbd_tx_event_overflow;
    wire [31:0]  mouse_tx_event_overflow;
    wire [31:0]  mgmt_guest_frame_count;
    wire [31:0]  mgmt_frame_count;
    wire [31:0]  mgmt_dropped_frame_count;
    wire [31:0]  mgmt_accepted_request_count;
    wire [31:0]  mgmt_rejected_request_count;
    wire [1:0]   mgmt_fan_duty_select;
    wire [31:0]  scc0_access_event_source_data;
    wire         scc0_access_event_source_valid;
    wire [31:0]  scc0_access_event_data;
    wire         scc0_access_event_valid;
    wire         scc0_access_event_ready;
    wire         scc0_access_event_push_ready;
    wire [15:0]  scc0_access_event_count;
    reg  [31:0]  scc0_access_event_overflow;
    wire [1:0]   mgmt_scc_trace_control;
    wire         mgmt_scc_trace_control_valid;
    reg          scc0_trace_waiting_for_rx;
    wire sd_cs_n;
    wire sd_clk;
    wire sd_mosi;
    wire sd_miso;
    wire [31:0] soc_vio_control;
`ifndef KCU105_CYAN_SOC_DEBUG
    assign soc_vio_control = 32'd0;
`endif
    wire soc_debug_reset = soc_vio_control[0];
    wire soc_debug_rx_inject_toggle = soc_vio_control[1];
    wire [7:0] soc_debug_rx_inject_data = soc_vio_control[15:8];
    wire soc_sd_read_skip_first_byte = soc_vio_control[16];
    wire soc_sd_debug_read_trigger = soc_vio_control[17];
    wire [2:0] soc_sd_debug_lba_select = soc_vio_control[20:18];
    wire soc_sd_debug_probe_select = soc_vio_control[21];
    // Raw-Ethernet management owns the fan in both normal and debug images,
    // so routine operation does not depend on Vivado.  The endpoint resets
    // to 25% and accepts absolute 25/50/75/100 settings.
    wire [1:0] soc_fan_duty_select = mgmt_fan_duty_select;
    // Keep the SCC/storage profile as the default.  Bit 24 switches only the
    // live VIO payloads to CPU/cache-coherency probes; it does not reset or
    // otherwise perturb the running SoC.
    wire soc_cpu_debug_probe_select = soc_vio_control[24];
    // Bit 25 selects the read-only DSP/host-interface state snapshot.
    wire soc_dsp_debug_probe_select = soc_vio_control[25];
    // Bit 26 is a diagnostic-only I$/D$/S$ bypass.  It defaults low in both
    // debug and non-debug builds, must be selected before resetting software,
    // and must remain selected for the whole diagnostic run.
    wire soc_force_uncached = soc_vio_control[26];
    // Bit 27 selects LG1 pixel/scanout state on VIO probe 4.
    wire soc_gfx_debug_probe_select = soc_vio_control[27];
    // Bits 31:29 form the read-only guest-frame capture control plane.  A
    // retained Hardware Manager value must first return to zero after reset;
    // otherwise an old session could stop scanout during PROM startup.
    reg soc_lg1_capture_armed;
    always @(posedge cpu_clk or negedge cpu_rst_n) begin
        if (!cpu_rst_n)
            soc_lg1_capture_armed <= 1'b0;
        else if (soc_vio_control[31:29] == 3'b000)
            soc_lg1_capture_armed <= 1'b1;
    end
    wire soc_lg1_capture_next_toggle = soc_lg1_capture_armed && soc_vio_control[29];
    wire soc_lg1_capture_reset_toggle = soc_lg1_capture_armed && soc_vio_control[30];
    wire soc_lg1_capture_hold = soc_lg1_capture_armed && soc_vio_control[31];
    // Disable the synthetic board BIST in the PROM-observation image.  Bit 28
    // is deliberately ignored so only guest-generated pixels can be captured.
    wire soc_lg1_capture_trigger = 1'b0;
    assign soc_rst_n = cpu_rst_n && !soc_debug_reset;

    assign sdio_cmd_fpga = sd_mosi;
    assign sdio_clk_fpga = sd_clk;
    assign sd_miso = sdio_data0_fpga;
    assign sdio_data1_fpga = 1'bz;
    assign sdio_data2_fpga = 1'bz;
    assign sdio_data3_fpga = sd_cs_n;

    ip20_kcu_soc_core #(
        .CLK_HZ(50000000),
        .SCACHE_LINES(8192),
        // Match the uniprocessor R4000SC NoMPmode replacement policy.
        // Ordinary direct-mapped S-cache victims must not back-invalidate
        // I-cache lines: an I/D index alias can otherwise ping-pong forever.
        // Explicit cache operations and DMA snoops still invalidate I-cache.
        .FULL_ICACHE_INCLUSION(0),
        // Retain the registered-BRAM cache island and overlap the synchronous
        // primary instruction-cache lookup with instruction fetch.
        .CPU_ICACHE_LOOKAHEAD(1),
        .HAS_DSP(1),
        .PROM_INIT_FILE(PROM_INIT_FILE),
        .PROM_INIT_B00(PROM_INIT_B00),
        .PROM_INIT_B01(PROM_INIT_B01),
        .PROM_INIT_B02(PROM_INIT_B02),
        .PROM_INIT_B03(PROM_INIT_B03),
        .PROM_INIT_B04(PROM_INIT_B04),
        .PROM_INIT_B05(PROM_INIT_B05),
        .PROM_INIT_B06(PROM_INIT_B06),
        .PROM_INIT_B07(PROM_INIT_B07),
        .PROM_INIT_B08(PROM_INIT_B08),
        .PROM_INIT_B09(PROM_INIT_B09),
        .PROM_INIT_B0A(PROM_INIT_B0A),
        .PROM_INIT_B0B(PROM_INIT_B0B),
        .PROM_INIT_B0C(PROM_INIT_B0C),
        .PROM_INIT_B0D(PROM_INIT_B0D),
        .PROM_INIT_B0E(PROM_INIT_B0E),
        .PROM_INIT_B0F(PROM_INIT_B0F),
        .EEPROM_INIT_FILE(EEPROM_INIT_FILE)
    ) u_soc (
        .clk(cpu_clk),
        .cpu_clk(cpu_island_clk),
        .lg1_serial_clk(lg1_serial_clk),
        .lg1_serial_rst_n(lg1_serial_rst_n),
        .rst_n(soc_rst_n),
        .uart_rxd(uart_rxd),
        .uart_txd(uart_txd),
        .debug_rx_inject_data(soc_debug_rx_inject_data),
        .debug_rx_inject_toggle(soc_debug_rx_inject_toggle),
        .host_kbd_data(host_kbd_data),
        .host_kbd_valid(host_kbd_valid),
        .host_kbd_ready(host_kbd_ready),
        .host_mouse_data(host_mouse_data),
        .host_mouse_valid(host_mouse_valid),
        .host_mouse_ready(host_mouse_ready),
        .kbd_tx_event_data(kbd_tx_event_data),
        .kbd_tx_event_valid(kbd_tx_event_valid),
        .kbd_tx_event_ready(kbd_tx_event_ready),
        .mouse_tx_event_data(mouse_tx_event_data),
        .mouse_tx_event_valid(mouse_tx_event_valid),
        .mouse_tx_event_ready(mouse_tx_event_ready),
        .kbd_rx_event_data(kbd_rx_event_data),
        .kbd_rx_event_valid(kbd_rx_event_valid),
        .kbd_rx_event_ready(kbd_rx_event_ready),
        .mouse_rx_event_data(mouse_rx_event_data),
        .mouse_rx_event_valid(mouse_rx_event_valid),
        .mouse_rx_event_ready(mouse_rx_event_ready),
        .host_kbd_queued(host_kbd_queued),
        .host_mouse_queued(host_mouse_queued),
        .kbd_tx_event_overflow(kbd_tx_event_overflow),
        .mouse_tx_event_overflow(mouse_tx_event_overflow),
        .scc0_access_event_data(scc0_access_event_source_data),
        .scc0_access_event_valid(scc0_access_event_source_valid),
        .ram_addr(ram_addr),
        .ram_wdata(ram_wdata),
        .ram_wmask(ram_wmask),
        .ram_wen(ram_wen),
        .ram_valid(ram_valid),
        .ram_rdata(ram_rdata),
        .ram_ready(ram_ready),
        .enet_tx_data(enet_tx_data),
        .enet_tx_valid(enet_tx_valid),
        .enet_tx_sof(enet_tx_sof),
        .enet_tx_eof(enet_tx_eof),
        .enet_tx_ready(enet_tx_ready),
        .enet_rx_data(enet_rx_data),
        .enet_rx_valid(enet_rx_valid),
        .enet_rx_sof(enet_rx_sof),
        .enet_rx_eof(enet_rx_eof),
        .enet_rx_ready(enet_rx_ready),
        .sd_cs_n(sd_cs_n),
        .sd_clk(sd_clk),
        .sd_mosi(sd_mosi),
        .sd_miso(sd_miso),
        .sd_card_detect(sdio_cd_fpga),
        .sd_read_skip_first_byte(soc_sd_read_skip_first_byte),
        .sd_debug_read_trigger(soc_sd_debug_read_trigger),
        .sd_debug_lba_select(soc_sd_debug_lba_select),
        .sd_debug_probe_select(soc_sd_debug_probe_select),
        .cpu_debug_probe_select(soc_cpu_debug_probe_select),
        .dsp_debug_probe_select(soc_dsp_debug_probe_select),
        .gfx_debug_probe_select(soc_gfx_debug_probe_select),
        .lg1_capture_trigger(soc_lg1_capture_trigger),
        .lg1_capture_hold(soc_lg1_capture_hold),
        .force_uncached(soc_force_uncached),
        .lg1_line_valid(soc_lg1_line_valid),
        .lg1_line_ready(soc_lg1_line_ready),
        .lg1_line_write_done(soc_lg1_line_write_done),
        .lg1_line_write(soc_lg1_line_write),
        .lg1_line_addr(soc_lg1_line_addr),
        .lg1_line_wdata(soc_lg1_line_wdata),
        .lg1_line_wstrb(soc_lg1_line_wstrb),
        .lg1_line_rsp_valid(soc_lg1_line_rsp_valid),
        .lg1_line_rsp_ready(soc_lg1_line_rsp_ready),
        .lg1_line_rsp_rdata(soc_lg1_line_rsp_rdata),
        .lg1_scan_mem_valid(soc_lg1_scan_mem_valid),
        .lg1_scan_mem_ready(soc_lg1_scan_mem_ready),
        .lg1_scan_mem_addr(soc_lg1_scan_mem_addr),
        .lg1_scan_mem_rsp_valid(soc_lg1_scan_mem_rsp_valid),
        .lg1_scan_mem_rsp_ready(soc_lg1_scan_mem_rsp_ready),
        .lg1_scan_mem_rsp_rdata(soc_lg1_scan_mem_rsp_rdata),
        .lg1_capture_bist_status(lg1_capture_bist_status),
        .video_de(soc_video_de),
        .video_hsync(soc_video_hsync),
        .video_vsync(soc_video_vsync),
        .video_r(soc_video_r),
        .video_g(soc_video_g),
        .video_b(soc_video_b),
        .video_valid(soc_video_valid),
        .video_frame_start(soc_video_frame_start),
        .video_x(soc_video_x),
        .video_y(soc_video_y),
        .video_raw_index(soc_video_raw_index),
        .video_did(soc_video_did),
        .video_display_index(soc_video_display_index),
        .video_xmap_mode(soc_video_xmap_mode),
        .video_palette_bank(soc_video_palette_bank),
        .video_resolver_class(soc_video_resolver_class),
        .video_fetch_low_signature(soc_video_fetch_low_signature),
        .video_fetch_high_signature(soc_video_fetch_high_signature),
        .video_fetch_signature_generation(
            soc_video_fetch_signature_generation),
        .video_fetch_signature_valid(soc_video_fetch_signature_valid),
        .video_resolver_telemetry(soc_video_resolver_telemetry),
        .video_resolver_detail(soc_video_resolver_detail),
        .video_resolver_frame_done(soc_video_resolver_frame_done),
        .video_underrun_count(soc_video_underrun_count),
        .video_protocol_error_count(soc_video_protocol_error_count),
        .gfx_retrace_status(soc_gfx_retrace_status),
        .dsp_audio_sample_data(dsp_audio_sample_data),
        .dsp_audio_sample_valid(dsp_audio_sample_valid),
        .dsp_audio_channel_status(dsp_audio_channel_status),
        .dsp_audio_channel_status_valid(dsp_audio_channel_status_valid),
        .dsp_audio_user_data(dsp_audio_user_data),
        .dsp_audio_user_valid(dsp_audio_user_valid),
        .dsp_audio_stream_active(dsp_audio_stream_active),
        .dsp_audio_rate_select(dsp_audio_rate_select),
        .dsp_audio_sample_ready(dsp_audio_sample_ready),
        .lg1_config_access_valid(soc_lg1_config_access_valid),
        .lg1_config_access_payload(soc_lg1_config_access_payload),
        .lg1_config_current_sel(soc_lg1_config_current_sel),
        .bringup_phase(soc_phase),
        .heartbeat(soc_heartbeat),
        .cpu_debug_pc(soc_debug_pc),
        .cpu_debug_mem_addr(soc_debug_mem_addr),
        .status_flags(soc_status_flags),
        .debug_probe(soc_debug_probe),
        .eeprom_debug_probe(soc_eeprom_debug_probe),
        .cpu_flow_debug_probe(soc_cpu_flow_debug_probe),
        .cpu_exception_debug_probe(soc_cpu_exception_debug_probe),
        .cpu_gpr_debug_probe(soc_cpu_gpr_debug_probe)
    );

    // Sixteen thousand lossless records cover the measured bounded Xsgi
    // configuration and REX-interesting bursts. This observer has no
    // functional connection back to u_soc or the LG1 bus.
    cyan_lg1_config_trace #(.ADDR_W(14)) u_lg1_config_trace (
        .clk(cpu_clk),
        .rst_n(soc_rst_n),
        .access_valid(soc_lg1_config_access_valid),
        .access_payload(soc_lg1_config_access_payload),
        .current_configsel(soc_lg1_config_current_sel),
        .command(mgmt_lg1_trace_command),
        .command_tag(mgmt_lg1_trace_command_tag),
        .command_valid(mgmt_lg1_trace_command_valid),
        .command_ready(mgmt_lg1_trace_command_ready),
        .command_ack(mgmt_lg1_trace_command_ack),
        .command_ok(mgmt_lg1_trace_command_ok),
        .status(mgmt_lg1_trace_status),
        .read_ordinal(mgmt_lg1_trace_read_ordinal),
        .read_request(mgmt_lg1_trace_read_request),
        .read_data(mgmt_lg1_trace_read_data),
        .read_ack(mgmt_lg1_trace_read_ack),
        .read_valid(mgmt_lg1_trace_read_valid)
    );

    // Preserve 256 keyboard/mouse-controller CPU accesses without ever
    // backpressuring HPC1. Management can clear immediately or arm a clean
    // window that begins on the first CPU access observing RX-ready.
    wire scc0_trace_clear = mgmt_scc_trace_control_valid &&
                            (mgmt_scc_trace_control != 2'd0);
    wire scc0_trace_push_valid = scc0_access_event_source_valid &&
        (!scc0_trace_waiting_for_rx || scc0_access_event_source_data[25]);
    cyan_byte_fifo #(.DEPTH(256), .WIDTH(32)) u_scc0_access_event_fifo (
        .clk(cpu_clk), .rst_n(cpu_rst_n), .clear(scc0_trace_clear),
        .push_data(scc0_access_event_source_data),
        .push_valid(scc0_trace_push_valid),
        .push_ready(scc0_access_event_push_ready),
        .pop_data(scc0_access_event_data),
        .pop_valid(scc0_access_event_valid),
        .pop_ready(scc0_access_event_ready),
        .count(scc0_access_event_count)
    );
    always @(posedge cpu_clk or negedge cpu_rst_n) begin
        if (!cpu_rst_n) begin
            scc0_access_event_overflow <= 32'd0;
            scc0_trace_waiting_for_rx <= 1'b0;
        end else begin
            if (mgmt_scc_trace_control_valid) begin
                scc0_trace_waiting_for_rx <=
                    (mgmt_scc_trace_control == 2'd1);
                if (mgmt_scc_trace_control != 2'd0)
                    scc0_access_event_overflow <= 32'd0;
            end else if (scc0_trace_waiting_for_rx &&
                         scc0_access_event_source_valid &&
                         scc0_access_event_source_data[25]) begin
                scc0_trace_waiting_for_rx <= 1'b0;
            end
            if (!scc0_trace_clear && scc0_trace_push_valid &&
                 !scc0_access_event_push_ready)
                scc0_access_event_overflow <= scc0_access_event_overflow + 1'b1;
        end
    end
    wire [7:0] scc0_access_event_count_status =
        (scc0_access_event_count[15:8] != 0) ? 8'hff :
        scc0_access_event_count[7:0];

    assign hdmi_iic_scl = hdmi_iic_scl_drive_low ? 1'b0 : 1'bz;
    assign hdmi_iic_sda = hdmi_iic_sda_drive_low ? 1'b0 : 1'bz;
    cyan_spdif_tx #(
        .CLK_HZ(50000000),
        .SAMPLE_RATE_HZ(48000)
    ) u_spdif_tx (
        .clk(cpu_clk),
        .rst_n(cpu_rst_n),
        .sample_data(dsp_audio_sample_data),
        .sample_valid(dsp_audio_sample_valid),
        .channel_status_data(dsp_audio_channel_status),
        .channel_status_valid(dsp_audio_channel_status_valid),
        .user_data(dsp_audio_user_data),
        .user_valid(dsp_audio_user_valid),
        .stream_active(dsp_audio_stream_active),
        .sample_rate_select(dsp_audio_rate_select),
        .sample_ready(dsp_audio_sample_ready),
        .spdif(hdmi_spdif),
        .status(dsp_audio_status)
    );

    kcu105_adv7511_tx #(
        .CLK_HZ(50000000),
        .I2C_HZ(100000)
    ) u_adv7511 (
        .clk(cpu_clk),
        .rst_n(cpu_rst_n),
        .pixel_clk(video_clk),
        .pixel_rst_n(video_rst_n),
        .video_de(hdmi_video_de),
        .video_hsync(hdmi_video_hsync),
        .video_vsync(hdmi_video_vsync),
        .video_r(hdmi_video_r),
        .video_g(hdmi_video_g),
        .video_b(hdmi_video_b),
        .hdmi_out_clk(hdmi_out_clk),
        .hdmi_hsync(hdmi_hsync),
        .hdmi_vsync(hdmi_vsync),
        .hdmi_data_e(hdmi_data_e),
        .hdmi_data(hdmi_data),
        .iic_scl_i(hdmi_iic_scl),
        .iic_sda_i(hdmi_iic_sda),
        .iic_scl_drive_low(hdmi_iic_scl_drive_low),
        .iic_sda_drive_low(hdmi_iic_sda_drive_low),
        .iic_mux_reset_n(hdmi_iic_mux_reset_n),
        .status(hdmi_status)
    );

`ifdef KCU105_CYAN_SOC_DEBUG
    localparam integer ENET_DEBUG_TX_PROBES = 1;
`else
    localparam integer ENET_DEBUG_TX_PROBES = 0;
`endif

    kcu105_sgmii_ethernet #(
        .CLK_HZ(50000000),
        .DEBUG_TX_PROBES(ENET_DEBUG_TX_PROBES)
    ) u_enet (
        .clk(cpu_clk),
        .rst_n(cpu_rst_n),
        .stream_tx_data(phy_enet_tx_data),
        .stream_tx_valid(phy_enet_tx_valid),
        .stream_tx_sof(phy_enet_tx_sof),
        .stream_tx_eof(phy_enet_tx_eof),
        .stream_tx_ready(phy_enet_tx_ready),
        .stream_rx_data(phy_enet_rx_data),
        .stream_rx_valid(phy_enet_rx_valid),
        .stream_rx_sof(phy_enet_rx_sof),
        .stream_rx_eof(phy_enet_rx_eof),
        .stream_rx_ready(phy_enet_rx_ready),
        .sgmii_txn(sgmii_txn),
        .sgmii_txp(sgmii_txp),
        .sgmii_rxn(sgmii_rxn),
        .sgmii_rxp(sgmii_rxp),
        .sgmii_refclk_n(sgmii_refclk_n),
        .sgmii_refclk_p(sgmii_refclk_p),
        .phy_mdc(phy_mdc),
        .phy_mdio(phy_mdio),
        .phy_rst_n(phy_rst_n),
        .link_up(enet_link_up),
        .pcs_config_done(enet_pcs_config_done),
        .debug_probe(enet_board_debug)
    );

    cyan_mgmt_frame_router u_mgmt_rx_router (
        .clk(cpu_clk),
        .rst_n(cpu_rst_n),
        .in_data(phy_enet_rx_data),
        .in_valid(phy_enet_rx_valid),
        .in_sof(phy_enet_rx_sof),
        .in_eof(phy_enet_rx_eof),
        .in_ready(phy_enet_rx_ready),
        .guest_data(enet_rx_data),
        .guest_valid(enet_rx_valid),
        .guest_sof(enet_rx_sof),
        .guest_eof(enet_rx_eof),
        .guest_ready(enet_rx_ready),
        .mgmt_data(mgmt_rx_data),
        .mgmt_valid(mgmt_rx_valid),
        .mgmt_sof(mgmt_rx_sof),
        .mgmt_eof(mgmt_rx_eof),
        .mgmt_ready(mgmt_rx_ready),
        .guest_frame_count(mgmt_guest_frame_count),
        .mgmt_frame_count(mgmt_frame_count),
        .dropped_frame_count(mgmt_dropped_frame_count)
    );

    // Keep essential board/video telemetry in the normal image.  The first
    // stage is allowed to sample changing counters; no sampled bit feeds
    // functional logic, and the second stage is the management snapshot.
    (* ASYNC_REG = "TRUE" *) reg [127:0] mgmt_video_status_meta;
    (* ASYNC_REG = "TRUE" *) reg [127:0] mgmt_video_status;
    (* ASYNC_REG = "TRUE" *) reg [127:0] mgmt_video_signature_status_meta;
    (* ASYNC_REG = "TRUE" *) reg [127:0] mgmt_video_signature_status;
    (* ASYNC_REG = "TRUE" *) reg [127:0]
        mgmt_video_stage_signature_status_meta;
    (* ASYNC_REG = "TRUE" *) reg [127:0]
        mgmt_video_stage_signature_status;
    (* ASYNC_REG = "TRUE" *) reg [127:0]
        mgmt_video_resolver_status_meta;
    (* ASYNC_REG = "TRUE" *) reg [127:0]
        mgmt_video_resolver_status;
    (* ASYNC_REG = "TRUE" *) reg [127:0]
        mgmt_video_resolver_detail_status_meta;
    (* ASYNC_REG = "TRUE" *) reg [127:0]
        mgmt_video_resolver_detail_status;
    (* ASYNC_REG = "TRUE" *) reg [127:0]
        mgmt_video_visual_stage_status_meta;
    (* ASYNC_REG = "TRUE" *) reg [127:0]
        mgmt_video_visual_stage_status;
    (* ASYNC_REG = "TRUE" *) reg [127:0]
        mgmt_video_scan_service_status_meta;
    (* ASYNC_REG = "TRUE" *) reg [127:0]
        mgmt_video_scan_service_status;
    (* ASYNC_REG = "TRUE" *) reg [127:0]
        mgmt_video_scan_latency_status_meta;
    (* ASYNC_REG = "TRUE" *) reg [127:0]
        mgmt_video_scan_latency_status;
    always @(posedge cpu_clk or negedge cpu_rst_n) begin
        if (!cpu_rst_n) begin
            mgmt_video_status_meta <= 128'd0;
            mgmt_video_status <= 128'd0;
            mgmt_video_signature_status_meta <= 128'd0;
            mgmt_video_signature_status <= 128'd0;
            mgmt_video_stage_signature_status_meta <= 128'd0;
            mgmt_video_stage_signature_status <= 128'd0;
            mgmt_video_resolver_status_meta <= 128'd0;
            mgmt_video_resolver_status <= 128'd0;
            mgmt_video_resolver_detail_status_meta <= 128'd0;
            mgmt_video_resolver_detail_status <= 128'd0;
            mgmt_video_visual_stage_status_meta <= 128'd0;
            mgmt_video_visual_stage_status <= 128'd0;
            mgmt_video_scan_service_status_meta <= 128'd0;
            mgmt_video_scan_service_status <= 128'd0;
            mgmt_video_scan_latency_status_meta <= 128'd0;
            mgmt_video_scan_latency_status <= 128'd0;
        end else begin
            mgmt_video_status_meta <= video_resampler_status;
            mgmt_video_status <= mgmt_video_status_meta;
            mgmt_video_signature_status_meta <=
                video_resampler_signature_status;
            mgmt_video_signature_status <= mgmt_video_signature_status_meta;
            mgmt_video_stage_signature_status_meta <=
                video_resampler_stage_signature_status;
            mgmt_video_stage_signature_status <=
                mgmt_video_stage_signature_status_meta;
            mgmt_video_resolver_status_meta <=
                video_resampler_resolver_status;
            mgmt_video_resolver_status <= mgmt_video_resolver_status_meta;
            mgmt_video_resolver_detail_status_meta <=
                video_resampler_resolver_detail_status;
            mgmt_video_resolver_detail_status <=
                mgmt_video_resolver_detail_status_meta;
            mgmt_video_visual_stage_status_meta <=
                video_resampler_visual_stage_status;
            mgmt_video_visual_stage_status <=
                mgmt_video_visual_stage_status_meta;
            mgmt_video_scan_service_status_meta <=
                video_resampler_scan_service_status;
            mgmt_video_scan_service_status <=
                mgmt_video_scan_service_status_meta;
            mgmt_video_scan_latency_status_meta <=
                video_resampler_scan_latency_status;
            mgmt_video_scan_latency_status <=
                mgmt_video_scan_latency_status_meta;
        end
    end
    wire [127:0] mgmt_board_status = {
        hdmi_status[127:96],
        soc_status_flags,
        mgmt_video_status[127:96],
        scc0_access_event_count_status,
        scc0_access_event_overflow[7:0],
        6'd0, mgmt_fan_duty_select,
        gpio_led
    };

    cyan_mgmt_endpoint u_mgmt_endpoint (
        .clk(cpu_clk),
        .rst_n(cpu_rst_n),
        .rx_data(mgmt_rx_data),
        .rx_valid(mgmt_rx_valid),
        .rx_sof(mgmt_rx_sof),
        .rx_eof(mgmt_rx_eof),
        .rx_ready(mgmt_rx_ready),
        .tx_data(mgmt_tx_data),
        .tx_valid(mgmt_tx_valid),
        .tx_sof(mgmt_tx_sof),
        .tx_eof(mgmt_tx_eof),
        .tx_ready(mgmt_tx_ready),
        .host_kbd_data(host_kbd_data),
        .host_kbd_valid(host_kbd_valid),
        .host_kbd_ready(host_kbd_ready),
        .host_mouse_data(host_mouse_data),
        .host_mouse_valid(host_mouse_valid),
        .host_mouse_ready(host_mouse_ready),
        .host_kbd_queued(host_kbd_queued),
        .host_mouse_queued(host_mouse_queued),
        .kbd_tx_event_overflow(kbd_tx_event_overflow),
        .mouse_tx_event_overflow(mouse_tx_event_overflow),
        .board_status(mgmt_board_status),
        .hdmi_link_status(hdmi_status),
        .video_status(mgmt_video_status),
        .video_signature_status(mgmt_video_signature_status),
        .video_stage_signature_status(mgmt_video_stage_signature_status),
        .resolver_status(mgmt_video_resolver_status),
        .resolver_detail_status(mgmt_video_resolver_detail_status),
        .visual_stage_status(mgmt_video_visual_stage_status),
        .scan_service_status(mgmt_video_scan_service_status),
        .scan_latency_status(mgmt_video_scan_latency_status),
        .audio_status(dsp_audio_status),
        .gfx_retrace_status(soc_gfx_retrace_status),
        .lg1_trace_status(mgmt_lg1_trace_status),
        .lg1_trace_command(mgmt_lg1_trace_command),
        .lg1_trace_command_tag(mgmt_lg1_trace_command_tag),
        .lg1_trace_command_valid(mgmt_lg1_trace_command_valid),
        .lg1_trace_command_ready(mgmt_lg1_trace_command_ready),
        .lg1_trace_command_ack(mgmt_lg1_trace_command_ack),
        .lg1_trace_command_ok(mgmt_lg1_trace_command_ok),
        .lg1_trace_read_ordinal(mgmt_lg1_trace_read_ordinal),
        .lg1_trace_read_request(mgmt_lg1_trace_read_request),
        .lg1_trace_read_data(mgmt_lg1_trace_read_data),
        .lg1_trace_read_ack(mgmt_lg1_trace_read_ack),
        .lg1_trace_read_valid(mgmt_lg1_trace_read_valid),
        .video_status_request_toggle(mgmt_video_status_request_toggle),
        .video_status_ack_toggle(mgmt_video_status_ack_toggle),
        .fan_duty_select(mgmt_fan_duty_select),
        .capture_command(mgmt_capture_command),
        .capture_command_toggle(mgmt_capture_command_toggle),
        .capture_word_index(mgmt_capture_word_index),
        .capture_ack_toggle(mgmt_capture_ack_toggle),
        .capture_info(mgmt_capture_info),
        .capture_word_data(mgmt_capture_word_data),
        .capture_word_valid(mgmt_capture_word_valid),
        .scc_access_event_data(scc0_access_event_data),
        .scc_access_event_valid(scc0_access_event_valid),
        .scc_access_event_ready(scc0_access_event_ready),
        .scc_trace_control(mgmt_scc_trace_control),
        .scc_trace_control_valid(mgmt_scc_trace_control_valid),
        .kbd_tx_event_data(kbd_tx_event_data),
        .kbd_tx_event_valid(kbd_tx_event_valid),
        .kbd_tx_event_ready(kbd_tx_event_ready),
        .mouse_tx_event_data(mouse_tx_event_data),
        .mouse_tx_event_valid(mouse_tx_event_valid),
        .mouse_tx_event_ready(mouse_tx_event_ready),
        .kbd_rx_event_data(kbd_rx_event_data),
        .kbd_rx_event_valid(kbd_rx_event_valid),
        .kbd_rx_event_ready(kbd_rx_event_ready),
        .mouse_rx_event_data(mouse_rx_event_data),
        .mouse_rx_event_valid(mouse_rx_event_valid),
        .mouse_rx_event_ready(mouse_rx_event_ready),
        .accepted_request_count(mgmt_accepted_request_count),
        .rejected_request_count(mgmt_rejected_request_count)
    );

    cyan_frame_arbiter u_mgmt_tx_arbiter (
        .clk(cpu_clk),
        .rst_n(cpu_rst_n),
        .guest_data(enet_tx_data),
        .guest_valid(enet_tx_valid),
        .guest_sof(enet_tx_sof),
        .guest_eof(enet_tx_eof),
        .guest_ready(enet_tx_ready),
        .mgmt_data(mgmt_tx_data),
        .mgmt_valid(mgmt_tx_valid),
        .mgmt_sof(mgmt_tx_sof),
        .mgmt_eof(mgmt_tx_eof),
        .mgmt_ready(mgmt_tx_ready),
        .out_data(phy_enet_tx_data),
        .out_valid(phy_enet_tx_valid),
        .out_sof(phy_enet_tx_sof),
        .out_eof(phy_enet_tx_eof),
        .out_ready(phy_enet_tx_ready)
    );

    assign uart_rts = 1'b0;

    // J73 is a three-wire fan switched by Q18.  Drive its gate at about
    // 24.4 kHz from the 50 MHz CPU clock instead of leaving AJ9 pulled high
    // (the KCU105 power-on/full-speed default).
    reg [10:0] fan_pwm_counter;
    always @(posedge cpu_clk) begin
        if (!cpu_rst_n)
            fan_pwm_counter <= 11'd0;
        else
            fan_pwm_counter <= fan_pwm_counter + 11'd1;
    end

    reg fan_pwm_value;
    always @* begin
        case (soc_fan_duty_select)
            2'b00: fan_pwm_value = ~fan_pwm_counter[10];
            2'b01: fan_pwm_value = (fan_pwm_counter[10:9] == 2'b00);
            2'b10: fan_pwm_value = (fan_pwm_counter[10:9] != 2'b11);
            default: fan_pwm_value = 1'b1;
        endcase
    end
    assign sm_fan_pwm = fan_pwm_value;

    assign gpio_led[0] = c0_init_calib_complete;
    assign gpio_led[1] = soc_status_flags[30];
    assign gpio_led[2] = soc_status_flags[29];
    assign gpio_led[3] = soc_status_flags[28];
    assign gpio_led[4] = soc_status_flags[20];
    assign gpio_led[5] = soc_status_flags[5];
    assign gpio_led[6] = enet_link_up || enet_pcs_config_done ||
                          enet_tx_valid || soc_status_flags[1];
    assign gpio_led[7] = soc_heartbeat;

`ifdef KCU105_CYAN_SOC_DEBUG
    wire [31:0] board_debug_status = {
        gpio_led,
        hdmi_status[107:105],
        uart_rxd,
        uart_txd,
        uart_cts,
        uart_rts,
        enet_rx_valid,
        enet_rx_sof,
        enet_rx_eof,
        enet_tx_valid,
        enet_tx_sof,
        enet_tx_eof,
        enet_tx_ready,
        enet_link_up,
        enet_pcs_config_done,
        cpu_rst_n,
        mig_ready,
        c0_ddr4_rst,
        c0_init_calib_complete,
        soc_phase
    };

    reg [31:0] dbg_soc_status_flags;
    reg [31:0] dbg_soc_debug_pc;
    reg [31:0] dbg_soc_debug_mem_addr;
    reg [31:0] dbg_board_debug_status;
    reg [127:0] dbg_soc_debug_probe;
    reg [255:0] dbg_mig_bridge_debug;
    reg [255:0] dbg_enet_board_debug;
    reg [127:0] dbg_soc_eeprom_debug_probe;
    reg [255:0] dbg_soc_cpu_flow_debug_probe;
    reg [255:0] dbg_soc_cpu_exception_debug_probe;
    reg [511:0] dbg_soc_cpu_gpr_debug_probe;
    reg [63:0] dbg_video_resampler_status_meta;
    reg [63:0] dbg_video_resampler_status;

    // Keep the CPU-clocked VIO snapshot entirely in the CPU domain.  Raw
    // lg1_mig_debug and mig_arb_debug state is generated on the MIG UI clock;
    // sampling it here would add unsafe debug-only CDC fanout to the real
    // request/acknowledge mailbox.  The CPU-side line contract is sufficient
    // to diagnose request stalls without perturbing that synchronizer.
    wire [127:0] lg1_cpu_bridge_debug = {
        42'd0,
        lg1_line_wstrb,
        lg1_line_addr,
        lg1_line_write,
        lg1_line_valid,
        lg1_line_ready,
        lg1_line_rsp_valid,
        lg1_line_rsp_ready,
        soc_gfx_debug_probe_select
    };

    always @(posedge cpu_clk) begin
        if (!cpu_rst_n) begin
            dbg_soc_status_flags <= 32'd0;
            dbg_soc_debug_pc <= 32'd0;
            dbg_soc_debug_mem_addr <= 32'd0;
            dbg_board_debug_status <= 32'd0;
            dbg_soc_debug_probe <= 128'd0;
            dbg_mig_bridge_debug <= 256'd0;
            dbg_enet_board_debug <= 256'd0;
            dbg_soc_eeprom_debug_probe <= 128'd0;
            dbg_soc_cpu_flow_debug_probe <= 256'd0;
            dbg_soc_cpu_exception_debug_probe <= 256'd0;
            dbg_soc_cpu_gpr_debug_probe <= 512'd0;
            dbg_video_resampler_status_meta <= 64'd0;
            dbg_video_resampler_status <= 64'd0;
        end else begin
            dbg_soc_status_flags <= soc_status_flags;
            dbg_soc_debug_pc <= soc_debug_pc;
            dbg_soc_debug_mem_addr <= soc_debug_mem_addr;
            dbg_board_debug_status <= board_debug_status;
            dbg_soc_debug_probe <= soc_lg1_capture_hold ?
                {lg1_capture_reader_status[127:32],
                 lg1_capture_bist_status} : soc_debug_probe;
            // Debug-only two-stage snapshot. Individual counters may cross on
            // adjacent clocks, but no sampled bit feeds functional control.
            dbg_video_resampler_status_meta <= video_resampler_status[127:64];
            dbg_video_resampler_status <= dbg_video_resampler_status_meta;
            dbg_mig_bridge_debug <= {dbg_video_resampler_status,
                                     lg1_cpu_bridge_debug,
                                     mig_bridge_debug[63:0]};
            dbg_enet_board_debug <= enet_board_debug;
            dbg_soc_eeprom_debug_probe <= soc_eeprom_debug_probe;
            dbg_soc_cpu_flow_debug_probe <= soc_cpu_flow_debug_probe;
            dbg_soc_cpu_exception_debug_probe <= soc_cpu_exception_debug_probe;
            dbg_soc_cpu_gpr_debug_probe <= soc_lg1_capture_hold ?
                lg1_capture_data : soc_cpu_gpr_debug_probe;
        end
    end

    kcu105_cyan_soc_vio u_cyan_soc_vio (
        .clk(cpu_clk),
        .probe_in0(dbg_soc_status_flags),
        .probe_in1(dbg_soc_debug_pc),
        .probe_in2(dbg_soc_debug_mem_addr),
        .probe_in3(dbg_board_debug_status),
        .probe_in4(dbg_soc_debug_probe),
        .probe_in5(dbg_mig_bridge_debug),
        .probe_in6(dbg_enet_board_debug),
        .probe_in7(dbg_soc_eeprom_debug_probe),
        .probe_in8(dbg_soc_cpu_flow_debug_probe),
        .probe_in9(dbg_soc_cpu_exception_debug_probe),
        .probe_in10(dbg_soc_cpu_gpr_debug_probe[511:256]),
        .probe_in11(dbg_soc_cpu_gpr_debug_probe[255:0]),
        .probe_out0(soc_vio_control)
    );
`endif

    wire unused_uart_cts = uart_cts;
    wire unused_dbg_clk = dbg_clk;
    wire [511:0] unused_dbg_bus = dbg_bus;
    wire unused_addn_ui_clkout1 = addn_ui_clkout1_unused;
    wire [255:0] unused_mig_bridge_debug = mig_bridge_debug;
    wire [127:0] unused_lg1_mig_debug = lg1_mig_debug;
    wire [63:0] unused_mig_arb_debug = mig_arb_debug;
    wire [127:0] unused_video_resampler_status = video_resampler_status;
    wire unused_soc_video_de = soc_video_de;
    wire unused_soc_video_hsync = soc_video_hsync;
    wire unused_soc_video_vsync = soc_video_vsync;
    wire [3:0] unused_soc_phase = soc_phase;
    wire [31:0] unused_soc_debug_pc = soc_debug_pc;
    wire [31:0] unused_soc_debug_mem_addr = soc_debug_mem_addr;
    wire [127:0] unused_soc_debug_probe = soc_debug_probe;
    wire unused_soc_debug_reset = soc_debug_reset;
    wire [127:0] unused_soc_eeprom_debug_probe = soc_eeprom_debug_probe;
    wire [255:0] unused_soc_cpu_flow_debug_probe = soc_cpu_flow_debug_probe;
    wire [255:0] unused_soc_cpu_exception_debug_probe = soc_cpu_exception_debug_probe;
    wire [511:0] unused_soc_cpu_gpr_debug_probe = soc_cpu_gpr_debug_probe;
    wire [255:0] unused_enet_board_debug = enet_board_debug;
    wire [31:0] unused_nck = nCK_PER_CLK;
    wire unused_sim = (SIMULATION == "TRUE");
endmodule

`default_nettype wire
