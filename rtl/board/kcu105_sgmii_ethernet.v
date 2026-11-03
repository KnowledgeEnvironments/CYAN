// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - KCU105 SGMII Ethernet shell.
//
// Bridges the IP20/HPC1 byte-stream Ethernet interface to the KCU105 onboard
// Marvell 88E1111 through the Xilinx SGMII-over-LVDS PCS/PMA IP.

`default_nettype none

module kcu105_sgmii_ethernet #(
    parameter integer CLK_HZ = 50000000,
    parameter integer DEBUG_TX_PROBES = 0
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  stream_tx_data,
    input  wire        stream_tx_valid,
    input  wire        stream_tx_sof,
    input  wire        stream_tx_eof,
    output wire        stream_tx_ready,

    output wire [7:0]  stream_rx_data,
    output wire        stream_rx_valid,
    output wire        stream_rx_sof,
    output wire        stream_rx_eof,
    input  wire        stream_rx_ready,

    output wire        sgmii_txn,
    output wire        sgmii_txp,
    input  wire        sgmii_rxn,
    input  wire        sgmii_rxp,
    input  wire        sgmii_refclk_n,
    input  wire        sgmii_refclk_p,

    output wire        phy_mdc,
    inout  wire        phy_mdio,
    output wire        phy_rst_n,

    output wire        link_up,
    output wire        pcs_config_done,
    output wire [255:0] debug_probe
);
    localparam [15:0] PCS_CONTROL_WORD = 16'h1340;
    localparam [63:0] PCS_MDIO_CONTROL_FRAME = {
        32'hffff_ffff,     // preamble
        2'b01,             // start
        2'b01,             // write
        5'd1,              // PCS/PMA management PHY address from Xilinx demo
        5'd0,              // control register
        2'b10,             // turn-around
        // SGMII MAC-side AN enabled, restart AN, 1000 Mbps, full duplex.
        PCS_CONTROL_WORD
    };

    wire        pcs_clk125;
    wire        pcs_clk312_unused;
    wire        pcs_clk625_unused;
    wire        pcs_rst125;
    wire        pcs_mmcm_locked;
    wire        pcs_mdio_o_unused;
    wire        pcs_mdio_t_unused;
    wire        pcs_an_interrupt_unused;
    wire        gmii_isolate_unused;
    wire        idelay_rdy;
    wire [15:0] pcs_status_vector;

    wire [7:0]  gmii_txd_mac;
    wire        gmii_tx_en_mac;
    wire        gmii_tx_er_mac;
    reg  [7:0]  gmii_txd_pcs;
    reg         gmii_tx_en_pcs;
    reg         gmii_tx_er_pcs;
    wire [7:0]  gmii_rxd_pcs;
    wire        gmii_rx_dv_pcs;
    wire        gmii_rx_er_pcs;
    reg  [7:0]  gmii_rxd_mac;
    reg         gmii_rx_dv_mac;
    reg         gmii_rx_er_mac;

    reg         pcs_mdc;
    reg         pcs_mdio_i;
    reg         pcs_mdio_active;
    reg         pcs_mdio_done;
    reg  [6:0]  pcs_mdio_bit_index;
    reg  [7:0]  pcs_mdio_divider;
    reg  [15:0] pcs_config_wait;

    // The 50 MHz machine reset is asynchronous to the recovered 125 MHz PCS
    // clock.  Assert immediately when either source is lost, then release all
    // CYAN-owned PCS/GMII state only after four local clock edges.  The Xilinx
    // PCS itself continues to use the board reset because it produces the
    // clock required by this synchronizer.
    wire pcs_reset_release = rst_n && pcs_mmcm_locked && !pcs_rst125;
    (* ASYNC_REG = "TRUE" *) reg [3:0] pcs_reset_sync;
    always @(posedge pcs_clk125 or negedge pcs_reset_release) begin
        if (!pcs_reset_release)
            pcs_reset_sync <= 4'b0000;
        else
            pcs_reset_sync <= {pcs_reset_sync[2:0], 1'b1};
    end
    wire pcs_domain_rst_n = &pcs_reset_sync;
    wire pcs_ready = pcs_domain_rst_n;
    // The onboard Marvell PHY sources the 625 MHz SGMII reference clock. Keep
    // PHY reset/MDIO alive from the system reset so PCS lock is not circular.
    wire mac_rst_n = rst_n;

    always @(posedge pcs_clk125 or negedge pcs_domain_rst_n) begin
        if (!pcs_domain_rst_n) begin
            gmii_txd_pcs <= 8'h00;
            gmii_tx_en_pcs <= 1'b0;
            gmii_tx_er_pcs <= 1'b0;
            gmii_rxd_mac <= 8'h00;
            gmii_rx_dv_mac <= 1'b0;
            gmii_rx_er_mac <= 1'b0;
        end else begin
            gmii_txd_pcs <= gmii_txd_mac;
            gmii_tx_en_pcs <= gmii_tx_en_mac;
            gmii_tx_er_pcs <= gmii_tx_er_mac;
            gmii_rxd_mac <= gmii_rxd_pcs;
            gmii_rx_dv_mac <= gmii_rx_dv_pcs;
            gmii_rx_er_mac <= gmii_rx_er_pcs;
        end
    end

    always @(posedge pcs_clk125 or negedge pcs_domain_rst_n) begin
        if (!pcs_domain_rst_n) begin
            pcs_mdc <= 1'b0;
            pcs_mdio_i <= 1'b1;
            pcs_mdio_active <= 1'b0;
            pcs_mdio_done <= 1'b0;
            pcs_mdio_bit_index <= 7'd0;
            pcs_mdio_divider <= 8'd0;
            pcs_config_wait <= 16'd0;
        end else if (!pcs_ready) begin
            pcs_mdc <= 1'b0;
            pcs_mdio_i <= 1'b1;
            pcs_mdio_active <= 1'b0;
            pcs_mdio_done <= 1'b0;
            pcs_mdio_bit_index <= 7'd0;
            pcs_mdio_divider <= 8'd0;
            pcs_config_wait <= 16'd0;
        end else if (!pcs_mdio_done) begin
            if (!pcs_mdio_active) begin
                if (&pcs_config_wait) begin
                    pcs_mdio_active <= 1'b1;
                    pcs_mdio_i <= PCS_MDIO_CONTROL_FRAME[63];
                    pcs_mdio_bit_index <= 7'd0;
                    pcs_mdio_divider <= 8'd0;
                    pcs_mdc <= 1'b0;
                end else begin
                    pcs_config_wait <= pcs_config_wait + 16'd1;
                end
            end else if (pcs_mdio_divider == 8'd24) begin
                pcs_mdio_divider <= 8'd0;
                pcs_mdc <= !pcs_mdc;
                if (pcs_mdc) begin
                    if (pcs_mdio_bit_index == 7'd63) begin
                        pcs_mdio_active <= 1'b0;
                        pcs_mdio_done <= 1'b1;
                        pcs_mdio_i <= 1'b1;
                    end else begin
                        pcs_mdio_bit_index <= pcs_mdio_bit_index + 7'd1;
                        pcs_mdio_i <= PCS_MDIO_CONTROL_FRAME[
                            63 - (pcs_mdio_bit_index + 7'd1)
                        ];
                    end
                end
            end else begin
                pcs_mdio_divider <= pcs_mdio_divider + 8'd1;
            end
        end else begin
            pcs_mdc <= 1'b0;
            pcs_mdio_i <= 1'b1;
        end
    end

    cyan_sgmii_pcs_pma u_pcs_pma (
        .txn(sgmii_txn),
        .txp(sgmii_txp),
        .rxn(sgmii_rxn),
        .rxp(sgmii_rxp),
        .mmcm_locked_out(pcs_mmcm_locked),
        .sgmii_clk_r(),
        .sgmii_clk_f(),
        .sgmii_clk_en(),
        .clk125_out(pcs_clk125),
        .clk625_out(pcs_clk625_unused),
        .clk312_out(pcs_clk312_unused),
        .rst_125_out(pcs_rst125),
        .refclk625_n(sgmii_refclk_n),
        .refclk625_p(sgmii_refclk_p),
        .gmii_txd(gmii_txd_pcs),
        .gmii_tx_en(gmii_tx_en_pcs),
        .gmii_tx_er(gmii_tx_er_pcs),
        .gmii_rxd(gmii_rxd_pcs),
        .gmii_rx_dv(gmii_rx_dv_pcs),
        .gmii_rx_er(gmii_rx_er_pcs),
        .gmii_isolate(gmii_isolate_unused),
        .mdc(pcs_mdc),
        .mdio_i(pcs_mdio_i),
        .mdio_o(pcs_mdio_o_unused),
        .mdio_t(pcs_mdio_t_unused),
        .ext_mdc(),
        .ext_mdio_i(1'b0),
        .mdio_t_in(1'b0),
        .ext_mdio_o(),
        .ext_mdio_t(),
        .phyaddr(5'd1),
        .configuration_vector(5'b00000),
        .configuration_valid(1'b0),
        .an_interrupt(pcs_an_interrupt_unused),
        .an_adv_config_vector(16'h0021),
        .an_adv_config_val(1'b0),
        .an_restart_config(1'b0),
        .speed_is_10_100(1'b0),
        .speed_is_100(1'b0),
        .status_vector(pcs_status_vector),
        .reset(!rst_n),
        .signal_detect(1'b1),
        .idelay_rdy_out(idelay_rdy)
    );

    gmii_ethernet_mac #(
        .CLK_HZ(CLK_HZ),
        .TX_FIFO_ADDR_W(11),
        .RX_FIFO_ADDR_W(14),
        .DEBUG_TX_PROBES(DEBUG_TX_PROBES)
    ) u_mac (
        .clk(clk),
        .rst_n(mac_rst_n),
        .gmii_rst_n(pcs_domain_rst_n),
        .stream_tx_data(stream_tx_data),
        .stream_tx_valid(stream_tx_valid),
        .stream_tx_sof(stream_tx_sof),
        .stream_tx_eof(stream_tx_eof),
        .stream_tx_ready(stream_tx_ready),
        .stream_rx_data(stream_rx_data),
        .stream_rx_valid(stream_rx_valid),
        .stream_rx_sof(stream_rx_sof),
        .stream_rx_eof(stream_rx_eof),
        .stream_rx_ready(stream_rx_ready),
        .gmii_tx_clk(pcs_clk125),
        .gmii_txd(gmii_txd_mac),
        .gmii_tx_en(gmii_tx_en_mac),
        .gmii_tx_er(gmii_tx_er_mac),
        .gmii_rx_clk(pcs_clk125),
        .gmii_rxd(gmii_rxd_mac),
        .gmii_rx_dv(gmii_rx_dv_mac),
        .gmii_rx_er(gmii_rx_er_mac),
        .phy_gtx_clk(),
        .phy_mdc(phy_mdc),
        .phy_mdio(phy_mdio),
        .phy_rst_n(phy_rst_n),
        .debug_probe(debug_probe[223:0])
    );

    assign link_up = pcs_status_vector[0];
    assign pcs_config_done = pcs_mdio_done;
    assign debug_probe[255:224] = {
        pcs_status_vector,
        1'b0,
        pcs_ready,
        pcs_mdio_active,
        pcs_mdio_done,
        idelay_rdy,
        pcs_mmcm_locked,
        pcs_rst125,
        pcs_an_interrupt_unused,
        gmii_isolate_unused,
        pcs_mdio_bit_index
    };
endmodule

`default_nettype wire
