// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// cyan_hpc_dma_coordinator.v -- machine-clocked HPC DMA coherence ordering
//
// One HPC beat is captured at a time.  CPU/cache quiescence is established
// before the MC transaction, DMA writes are committed to memory before their
// cache snoop, and DMA reads probe the cache before accepting the reserved MC
// read.  The level hold/fence controls use a four-phase handshake: a new beat
// is not captured until both acknowledgements from the previous beat are low.

`default_nettype none

module cyan_hpc_dma_coordinator (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [31:0] hpc_addr,
    input  wire [63:0] hpc_wdata,
    input  wire [ 7:0] hpc_wmask,
    input  wire        hpc_valid,
    input  wire        hpc_write,
    output wire [63:0] hpc_rdata,
    output wire        hpc_ready,

    output reg  [31:0] txn_addr,
    output reg  [63:0] txn_wdata,
    output reg  [ 7:0] txn_wmask,
    output reg         txn_write,

    output wire        mc_request,
    output wire        mc_valid,
    output wire        mc_write,
    input  wire [63:0] mc_rdata,
    input  wire        mc_ready,

    output wire        core_hold,
    input  wire        core_hold_ack,
    output wire        dmem_fence,
    input  wire        dmem_fence_ack,

    output wire        snoop_wr_launch,
    input  wire        snoop_wr_ready,
    input  wire        snoop_wr_complete,

    output wire        snoop_rd_launch,
    input  wire        snoop_rd_ready,
    input  wire        snoop_rd_resp_valid,
    input  wire        snoop_rd_resp_hit,
    input  wire [63:0] snoop_rd_resp_data,

    output wire [ 3:0] debug_state,
    output wire        debug_probe_hit,
    output wire [63:0] debug_probe_data
);
    localparam [3:0] DMA_IDLE      = 4'd0;
    localparam [3:0] DMA_WR_HOLD   = 4'd1;
    localparam [3:0] DMA_WR_MC     = 4'd2;
    localparam [3:0] DMA_WR_SNOOP  = 4'd3;
    localparam [3:0] DMA_RD_FENCE  = 4'd4;
    localparam [3:0] DMA_RD_PROBE  = 4'd5;
    localparam [3:0] DMA_RD_MC     = 4'd6;
    localparam [3:0] DMA_COMPLETE  = 4'd7;

    reg [3:0] state;
    reg       snoop_wr_launched;
    reg       snoop_rd_launched;
    reg       probe_hit;
    reg [63:0] probe_data;
    reg [63:0] result_data;

    // A CPU transaction already captured by the cross-clock mailbox is an
    // indivisible machine-side request. Let that one beat drain while the
    // coherence hold/fence is taking effect; core_hold_ack proves the mailbox
    // empty. Only then reserve MC across the cache probe and resolved DMA
    // read. Reserving in IDLE/RD_FENCE creates a cycle: MC blocks the CPU beat
    // while hold acknowledgement waits for that same beat to complete.
    assign mc_request =
        (state == DMA_RD_PROBE) ||
        (state == DMA_RD_MC) ||
        (state == DMA_WR_MC);
    assign mc_valid = (state == DMA_WR_MC) || (state == DMA_RD_MC);
    assign mc_write = (state == DMA_WR_MC);

    assign core_hold =
        (state == DMA_WR_HOLD) ||
        (state == DMA_WR_MC) ||
        (state == DMA_WR_SNOOP) ||
        (state == DMA_RD_FENCE) ||
        (state == DMA_RD_PROBE) ||
        (state == DMA_RD_MC);
    assign dmem_fence =
        (state == DMA_RD_FENCE) ||
        (state == DMA_RD_PROBE) ||
        (state == DMA_RD_MC);

    // These are launch pulses, not held ready/valid sources.  The bridge
    // captures the payload on the pulse and reports independent completion.
    assign snoop_wr_launch = (state == DMA_WR_SNOOP) &&
                             !snoop_wr_launched && snoop_wr_ready;
    assign snoop_rd_launch = (state == DMA_RD_PROBE) &&
                             !snoop_rd_launched && snoop_rd_ready;

    assign hpc_rdata = result_data;
    assign hpc_ready = (state == DMA_COMPLETE);
    assign debug_state = state;
    assign debug_probe_hit = probe_hit;
    assign debug_probe_data = probe_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= DMA_IDLE;
            txn_addr <= 32'd0;
            txn_wdata <= 64'd0;
            txn_wmask <= 8'd0;
            txn_write <= 1'b0;
            snoop_wr_launched <= 1'b0;
            snoop_rd_launched <= 1'b0;
            probe_hit <= 1'b0;
            probe_data <= 64'd0;
            result_data <= 64'd0;
        end else begin
            case (state)
                DMA_IDLE: begin
                    snoop_wr_launched <= 1'b0;
                    snoop_rd_launched <= 1'b0;
                    // Complete the previous four-phase level handshake before
                    // accepting a zero-gap successor beat.  A stale high ack
                    // must never advance the next transaction.
                    if (hpc_valid && !core_hold_ack && !dmem_fence_ack) begin
                        txn_addr <= hpc_addr;
                        txn_wdata <= hpc_wdata;
                        txn_wmask <= hpc_wmask;
                        txn_write <= hpc_write;
                        state <= hpc_write ? DMA_WR_HOLD : DMA_RD_FENCE;
                    end
                end
                DMA_WR_HOLD: begin
                    if (core_hold_ack)
                        state <= DMA_WR_MC;
                end
                DMA_WR_MC: begin
                    if (mc_ready) begin
                        snoop_wr_launched <= 1'b0;
                        state <= DMA_WR_SNOOP;
                    end
                end
                DMA_WR_SNOOP: begin
                    if (snoop_wr_launch)
                        snoop_wr_launched <= 1'b1;
                    if (snoop_wr_complete)
                        state <= DMA_COMPLETE;
                end
                DMA_RD_FENCE: begin
                    if (core_hold_ack && dmem_fence_ack) begin
                        snoop_rd_launched <= 1'b0;
                        state <= DMA_RD_PROBE;
                    end
                end
                DMA_RD_PROBE: begin
                    if (snoop_rd_launch)
                        snoop_rd_launched <= 1'b1;
                    if (snoop_rd_resp_valid) begin
                        probe_hit <= snoop_rd_resp_hit;
                        probe_data <= snoop_rd_resp_data;
                        state <= DMA_RD_MC;
                    end
                end
                DMA_RD_MC: begin
                    if (mc_ready) begin
                        result_data <= probe_hit ? probe_data : mc_rdata;
                        state <= DMA_COMPLETE;
                    end
                end
                DMA_COMPLETE: state <= DMA_IDLE;
                default: state <= DMA_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
