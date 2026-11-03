// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// Ratio-independent transactional boundary between the R4000/cache island
// and the 50 MHz IP20 machine domain.
//
// Every bundled-data channel is one-outstanding.  A source captures its
// payload, toggles a request bit, and holds the payload until the returned
// acknowledge toggle has crossed two synchronizer stages.  The destination
// only samples the held payload after the request toggle has crossed those
// stages.  Correctness therefore does not depend on a fixed 2:1 clock phase.

`default_nettype none

module cyan_cpu_island_bridge (
    input  wire        cpu_clk,
    input  wire        cpu_rst_n,

    input  wire        cpu_dmem_ren,
    input  wire        cpu_dmem_wen,
    input  wire [31:0] cpu_dmem_paddr,
    input  wire [ 7:0] cpu_dmem_wmask,
    input  wire [63:0] cpu_dmem_wdata,
    input  wire        cpu_dmem_uncached_wr,
    input  wire        cpu_dmem_cache_owned,
    input  wire [31:0] cpu_dmem_pc,
    output reg  [63:0] cpu_dmem_rdata,
    output reg         cpu_dmem_ready,

    input  wire        machine_clk,
    input  wire        machine_rst_n,
    output wire        machine_dmem_ren,
    output wire        machine_dmem_wen,
    output wire [31:0] machine_dmem_paddr,
    output wire [ 7:0] machine_dmem_wmask,
    output wire [63:0] machine_dmem_wdata,
    output wire        machine_dmem_uncached_wr,
    output wire        machine_dmem_cache_owned,
    output wire [31:0] machine_dmem_pc,
    input  wire [63:0] machine_dmem_rdata,
    input  wire        machine_dmem_ready,

    input  wire        machine_snoop_wr_valid,
    input  wire [31:0] machine_snoop_wr_addr,
    input  wire [63:0] machine_snoop_wr_data,
    input  wire [ 7:0] machine_snoop_wr_mask,
    output wire        machine_snoop_wr_ready,
    output reg         cpu_snoop_wr_valid,
    output reg  [31:0] cpu_snoop_wr_addr,
    output reg  [63:0] cpu_snoop_wr_data,
    output reg  [ 7:0] cpu_snoop_wr_mask,
    input  wire        cpu_snoop_wr_ready,
    input  wire        cpu_snoop_wr_done,
    output reg         machine_snoop_wr_complete,

    input  wire        machine_snoop_rd_valid,
    input  wire [31:0] machine_snoop_rd_addr,
    output wire        machine_snoop_rd_ready,
    output wire        cpu_snoop_rd_valid,
    output wire [31:0] cpu_snoop_rd_addr,
    input  wire        cpu_snoop_rd_resp_valid,
    input  wire        cpu_snoop_rd_resp_hit,
    input  wire [63:0] cpu_snoop_rd_resp_data,
    output reg         machine_snoop_rd_resp_valid,
    output reg         machine_snoop_rd_resp_hit,
    output reg  [63:0] machine_snoop_rd_resp_data,

    input  wire [5:0]  machine_irq,
    output wire [5:0]  cpu_irq,
    input  wire        machine_force_uncached,
    output wire        cpu_force_uncached,
    input  wire        machine_core_hold,
    output wire        machine_core_hold_ack,
    output wire        cpu_core_hold,
    input  wire        cpu_core_hold_quiesced,
    input  wire        machine_dmem_fence,
    output wire        machine_dmem_fence_ack
);
    // CPU request -> machine response channel.
    reg        cpu_req_toggle;
    reg        cpu_req_busy;
    reg        cpu_req_ren_q;
    reg        cpu_req_wen_q;
    reg [31:0] cpu_req_paddr_q;
    reg [ 7:0] cpu_req_wmask_q;
    reg [63:0] cpu_req_wdata_q;
    reg        cpu_req_uncached_wr_q;
    reg        cpu_req_cache_owned_q;
    reg [31:0] cpu_req_pc_q;

    reg        machine_req_seen;
    reg        machine_req_active;
    reg        machine_req_ren_q;
    reg        machine_req_wen_q;
    reg [31:0] machine_req_paddr_q;
    reg [ 7:0] machine_req_wmask_q;
    reg [63:0] machine_req_wdata_q;
    reg        machine_req_uncached_wr_q;
    reg        machine_req_cache_owned_q;
    reg [31:0] machine_req_pc_q;
    reg [63:0] machine_resp_data_q;
    reg        machine_resp_toggle;

    (* ASYNC_REG = "TRUE" *) reg machine_req_sync1;
    (* ASYNC_REG = "TRUE" *) reg machine_req_sync2;
    (* ASYNC_REG = "TRUE" *) reg cpu_resp_sync1;
    (* ASYNC_REG = "TRUE" *) reg cpu_resp_sync2;
    (* ASYNC_REG = "TRUE" *) reg cpu_core_hold_sync1;
    (* ASYNC_REG = "TRUE" *) reg cpu_core_hold_sync2;
    (* ASYNC_REG = "TRUE" *) reg machine_core_hold_ack_sync1;
    (* ASYNC_REG = "TRUE" *) reg machine_core_hold_ack_sync2;
    (* ASYNC_REG = "TRUE" *) reg cpu_dmem_fence_sync1;
    (* ASYNC_REG = "TRUE" *) reg cpu_dmem_fence_sync2;
    (* ASYNC_REG = "TRUE" *) reg machine_dmem_fence_ack_sync1;
    (* ASYNC_REG = "TRUE" *) reg machine_dmem_fence_ack_sync2;
    reg cpu_core_quiescent_ack;

    wire cpu_req_valid = cpu_dmem_ren || cpu_dmem_wen;

    assign machine_dmem_ren = machine_req_active && machine_req_ren_q;
    assign machine_dmem_wen = machine_req_active && machine_req_wen_q;
    assign machine_dmem_paddr = machine_req_paddr_q;
    assign machine_dmem_wmask = machine_req_wmask_q;
    assign machine_dmem_wdata = machine_req_wdata_q;
    assign machine_dmem_uncached_wr = machine_req_uncached_wr_q;
    assign machine_dmem_cache_owned = machine_req_cache_owned_q;
    assign machine_dmem_pc = machine_req_pc_q;

    always @(posedge cpu_clk or negedge cpu_rst_n) begin
        if (!cpu_rst_n) begin
            cpu_req_toggle <= 1'b0;
            cpu_req_busy <= 1'b0;
            cpu_req_ren_q <= 1'b0;
            cpu_req_wen_q <= 1'b0;
            cpu_req_paddr_q <= 32'd0;
            cpu_req_wmask_q <= 8'd0;
            cpu_req_wdata_q <= 64'd0;
            cpu_req_uncached_wr_q <= 1'b0;
            cpu_req_cache_owned_q <= 1'b0;
            cpu_req_pc_q <= 32'd0;
            cpu_resp_sync1 <= 1'b0;
            cpu_resp_sync2 <= 1'b0;
            cpu_dmem_rdata <= 64'd0;
            cpu_dmem_ready <= 1'b0;
        end else begin
            cpu_resp_sync1 <= machine_resp_toggle;
            cpu_resp_sync2 <= cpu_resp_sync1;
            cpu_dmem_ready <= 1'b0;
            if (cpu_req_busy) begin
                if (cpu_resp_sync2 == cpu_req_toggle) begin
                    cpu_dmem_rdata <= machine_resp_data_q;
                    cpu_dmem_ready <= 1'b1;
                    cpu_req_busy <= 1'b0;
                end
            end else if (cpu_req_valid && !cpu_dmem_ready &&
                         (!cpu_dmem_fence_sync2 || cpu_core_hold_sync2)) begin
                // ready is registered high across the source's synchronous
                // consume edge.  Suppress capture for exactly that edge: this
                // prevents replay of the completed beat while still allowing
                // a continuous-valid S-cache burst to present its next address
                // on the following edge.
                // The architectural hold freezes IS..DS, but cache
                // controllers and the one boundary DS operation must still
                // drain.  Blocking mailbox capture on hold/fence deadlocks a
                // CACHE writeback that starts after the hold reaches WB.
                // A standalone fence still rejects genuinely new requests;
                // the product read path pairs fence with hold so the frozen
                // boundary operation can drain.  The registered quiescence
                // ack below proves this mailbox is empty before DMA commits.
                cpu_req_ren_q <= cpu_dmem_ren;
                cpu_req_wen_q <= cpu_dmem_wen;
                cpu_req_paddr_q <= cpu_dmem_paddr;
                cpu_req_wmask_q <= cpu_dmem_wmask;
                cpu_req_wdata_q <= cpu_dmem_wdata;
                cpu_req_uncached_wr_q <= cpu_dmem_uncached_wr;
                cpu_req_cache_owned_q <= cpu_dmem_cache_owned;
                cpu_req_pc_q <= cpu_dmem_pc;
                cpu_req_toggle <= ~cpu_req_toggle;
                cpu_req_busy <= 1'b1;
            end
        end
    end

    always @(posedge machine_clk or negedge machine_rst_n) begin
        if (!machine_rst_n) begin
            machine_req_sync1 <= 1'b0;
            machine_req_sync2 <= 1'b0;
            machine_req_seen <= 1'b0;
            machine_req_active <= 1'b0;
            machine_req_ren_q <= 1'b0;
            machine_req_wen_q <= 1'b0;
            machine_req_paddr_q <= 32'd0;
            machine_req_wmask_q <= 8'd0;
            machine_req_wdata_q <= 64'd0;
            machine_req_uncached_wr_q <= 1'b0;
            machine_req_cache_owned_q <= 1'b0;
            machine_req_pc_q <= 32'd0;
            machine_resp_data_q <= 64'd0;
            machine_resp_toggle <= 1'b0;
        end else begin
            machine_req_sync1 <= cpu_req_toggle;
            machine_req_sync2 <= machine_req_sync1;
            if (!machine_req_active &&
                (machine_req_sync2 != machine_req_seen)) begin
                machine_req_seen <= machine_req_sync2;
                machine_req_active <= 1'b1;
                machine_req_ren_q <= cpu_req_ren_q;
                machine_req_wen_q <= cpu_req_wen_q;
                machine_req_paddr_q <= cpu_req_paddr_q;
                machine_req_wmask_q <= cpu_req_wmask_q;
                machine_req_wdata_q <= cpu_req_wdata_q;
                machine_req_uncached_wr_q <= cpu_req_uncached_wr_q;
                machine_req_cache_owned_q <= cpu_req_cache_owned_q;
                machine_req_pc_q <= cpu_req_pc_q;
            end else if (machine_req_active && machine_dmem_ready) begin
                machine_resp_data_q <= machine_dmem_rdata;
                machine_resp_toggle <= machine_req_seen;
                machine_req_active <= 1'b0;
            end
        end
    end

    // Accepted machine DMA write -> CPU cache snoop channel.
    reg        machine_wr_toggle;
    reg        machine_wr_busy;
    reg [31:0] machine_wr_addr_q;
    reg [63:0] machine_wr_data_q;
    reg [ 7:0] machine_wr_mask_q;
    reg        cpu_wr_seen;
    reg        cpu_wr_ack_toggle;
    reg        cpu_wr_active;
    reg        cpu_wr_wait_done;
    (* ASYNC_REG = "TRUE" *) reg cpu_wr_req_sync1;
    (* ASYNC_REG = "TRUE" *) reg cpu_wr_req_sync2;
    (* ASYNC_REG = "TRUE" *) reg machine_wr_ack_sync1;
    (* ASYNC_REG = "TRUE" *) reg machine_wr_ack_sync2;

    assign machine_snoop_wr_ready = !machine_wr_busy;

    always @(posedge machine_clk or negedge machine_rst_n) begin
        if (!machine_rst_n) begin
            machine_wr_toggle <= 1'b0;
            machine_wr_busy <= 1'b0;
            machine_wr_addr_q <= 32'd0;
            machine_wr_data_q <= 64'd0;
            machine_wr_mask_q <= 8'd0;
            machine_wr_ack_sync1 <= 1'b0;
            machine_wr_ack_sync2 <= 1'b0;
            machine_snoop_wr_complete <= 1'b0;
        end else begin
            machine_wr_ack_sync1 <= cpu_wr_ack_toggle;
            machine_wr_ack_sync2 <= machine_wr_ack_sync1;
            machine_snoop_wr_complete <= 1'b0;
            if (machine_wr_busy) begin
                if (machine_wr_ack_sync2 == machine_wr_toggle) begin
                    machine_wr_busy <= 1'b0;
                    machine_snoop_wr_complete <= 1'b1;
                end
            end else if (machine_snoop_wr_valid) begin
                machine_wr_addr_q <= machine_snoop_wr_addr;
                machine_wr_data_q <= machine_snoop_wr_data;
                machine_wr_mask_q <= machine_snoop_wr_mask;
                machine_wr_toggle <= ~machine_wr_toggle;
                machine_wr_busy <= 1'b1;
            end
        end
    end

    always @(posedge cpu_clk or negedge cpu_rst_n) begin
        if (!cpu_rst_n) begin
            cpu_wr_req_sync1 <= 1'b0;
            cpu_wr_req_sync2 <= 1'b0;
            cpu_wr_seen <= 1'b0;
            cpu_wr_ack_toggle <= 1'b0;
            cpu_wr_active <= 1'b0;
            cpu_wr_wait_done <= 1'b0;
            cpu_snoop_wr_valid <= 1'b0;
            cpu_snoop_wr_addr <= 32'd0;
            cpu_snoop_wr_data <= 64'd0;
            cpu_snoop_wr_mask <= 8'd0;
        end else begin
            cpu_wr_req_sync1 <= machine_wr_toggle;
            cpu_wr_req_sync2 <= cpu_wr_req_sync1;
            if (!cpu_wr_active && (cpu_wr_req_sync2 != cpu_wr_seen)) begin
                cpu_wr_seen <= cpu_wr_req_sync2;
                cpu_wr_active <= 1'b1;
                cpu_wr_wait_done <= 1'b0;
                cpu_snoop_wr_valid <= 1'b1;
                cpu_snoop_wr_addr <= machine_wr_addr_q;
                cpu_snoop_wr_data <= machine_wr_data_q;
                cpu_snoop_wr_mask <= machine_wr_mask_q;
            end else if (cpu_wr_active && !cpu_wr_wait_done &&
                         cpu_snoop_wr_ready) begin
                cpu_snoop_wr_valid <= 1'b0;
                if (cpu_snoop_wr_done) begin
                    cpu_wr_ack_toggle <= cpu_wr_seen;
                    cpu_wr_active <= 1'b0;
                end else begin
                    cpu_wr_wait_done <= 1'b1;
                end
            end else if (cpu_wr_active && cpu_wr_wait_done &&
                         cpu_snoop_wr_done) begin
                cpu_wr_ack_toggle <= cpu_wr_seen;
                cpu_wr_active <= 1'b0;
                cpu_wr_wait_done <= 1'b0;
            end
        end
    end

    // Machine DMA read -> CPU cache lookup -> machine response channel.
    reg        machine_rd_toggle;
    reg        machine_rd_busy;
    reg [31:0] machine_rd_addr_q;
    reg        cpu_rd_seen;
    reg        cpu_rd_active;
    reg [31:0] cpu_rd_addr_q;
    reg        cpu_rd_resp_hit_q;
    reg [63:0] cpu_rd_resp_data_q;
    reg        cpu_rd_ack_toggle;
    (* ASYNC_REG = "TRUE" *) reg cpu_rd_req_sync1;
    (* ASYNC_REG = "TRUE" *) reg cpu_rd_req_sync2;
    (* ASYNC_REG = "TRUE" *) reg machine_rd_ack_sync1;
    (* ASYNC_REG = "TRUE" *) reg machine_rd_ack_sync2;

    assign machine_snoop_rd_ready = !machine_rd_busy;
    assign cpu_snoop_rd_valid = cpu_rd_active;
    assign cpu_snoop_rd_addr = cpu_rd_addr_q;

    always @(posedge machine_clk or negedge machine_rst_n) begin
        if (!machine_rst_n) begin
            machine_rd_toggle <= 1'b0;
            machine_rd_busy <= 1'b0;
            machine_rd_addr_q <= 32'd0;
            machine_rd_ack_sync1 <= 1'b0;
            machine_rd_ack_sync2 <= 1'b0;
            machine_snoop_rd_resp_valid <= 1'b0;
            machine_snoop_rd_resp_hit <= 1'b0;
            machine_snoop_rd_resp_data <= 64'd0;
        end else begin
            machine_rd_ack_sync1 <= cpu_rd_ack_toggle;
            machine_rd_ack_sync2 <= machine_rd_ack_sync1;
            machine_snoop_rd_resp_valid <= 1'b0;
            if (machine_rd_busy) begin
                if (machine_rd_ack_sync2 == machine_rd_toggle) begin
                    machine_snoop_rd_resp_valid <= 1'b1;
                    machine_snoop_rd_resp_hit <= cpu_rd_resp_hit_q;
                    machine_snoop_rd_resp_data <= cpu_rd_resp_data_q;
                    machine_rd_busy <= 1'b0;
                end
            end else if (machine_snoop_rd_valid) begin
                machine_rd_addr_q <= machine_snoop_rd_addr;
                machine_rd_toggle <= ~machine_rd_toggle;
                machine_rd_busy <= 1'b1;
            end
        end
    end

    always @(posedge cpu_clk or negedge cpu_rst_n) begin
        if (!cpu_rst_n) begin
            cpu_rd_req_sync1 <= 1'b0;
            cpu_rd_req_sync2 <= 1'b0;
            cpu_rd_seen <= 1'b0;
            cpu_rd_active <= 1'b0;
            cpu_rd_addr_q <= 32'd0;
            cpu_rd_resp_hit_q <= 1'b0;
            cpu_rd_resp_data_q <= 64'd0;
            cpu_rd_ack_toggle <= 1'b0;
        end else begin
            cpu_rd_req_sync1 <= machine_rd_toggle;
            cpu_rd_req_sync2 <= cpu_rd_req_sync1;
            if (!cpu_rd_active && (cpu_rd_req_sync2 != cpu_rd_seen)) begin
                cpu_rd_seen <= cpu_rd_req_sync2;
                cpu_rd_active <= 1'b1;
                cpu_rd_addr_q <= machine_rd_addr_q;
            end else if (cpu_rd_active && cpu_snoop_rd_resp_valid) begin
                cpu_rd_resp_hit_q <= cpu_snoop_rd_resp_hit;
                cpu_rd_resp_data_q <= cpu_snoop_rd_resp_data;
                cpu_rd_ack_toggle <= cpu_rd_seen;
                cpu_rd_active <= 1'b0;
            end
        end
    end

    // Level controls may change independently of CPU execution.  Two-stage
    // synchronization preserves their level semantics; no pulse is inferred.
    (* ASYNC_REG = "TRUE" *) reg [5:0] cpu_irq_sync1;
    (* ASYNC_REG = "TRUE" *) reg [5:0] cpu_irq_sync2;
    (* ASYNC_REG = "TRUE" *) reg cpu_force_uncached_sync1;
    (* ASYNC_REG = "TRUE" *) reg cpu_force_uncached_sync2;

    assign cpu_irq = cpu_irq_sync2;
    assign cpu_force_uncached = cpu_force_uncached_sync2;
    assign cpu_core_hold = cpu_core_hold_sync2;
    assign machine_core_hold_ack = machine_core_hold_ack_sync2;
    assign machine_dmem_fence_ack = machine_dmem_fence_ack_sync2;

    always @(posedge cpu_clk or negedge cpu_rst_n) begin
        if (!cpu_rst_n) begin
            cpu_irq_sync1 <= 6'd0;
            cpu_irq_sync2 <= 6'd0;
            cpu_force_uncached_sync1 <= 1'b0;
            cpu_force_uncached_sync2 <= 1'b0;
            cpu_core_hold_sync1 <= 1'b0;
            cpu_core_hold_sync2 <= 1'b0;
            cpu_dmem_fence_sync1 <= 1'b0;
            cpu_dmem_fence_sync2 <= 1'b0;
            cpu_core_quiescent_ack <= 1'b0;
        end else begin
            cpu_irq_sync1 <= machine_irq;
            cpu_irq_sync2 <= cpu_irq_sync1;
            cpu_force_uncached_sync1 <= machine_force_uncached;
            cpu_force_uncached_sync2 <= cpu_force_uncached_sync1;
            cpu_core_hold_sync1 <= machine_core_hold;
            cpu_core_hold_sync2 <= cpu_core_hold_sync1;
            cpu_dmem_fence_sync1 <= machine_dmem_fence;
            cpu_dmem_fence_sync2 <= cpu_dmem_fence_sync1;
            // Collapse the hold level and CPU-mailbox occupancy entirely in
            // the CPU domain.  Only this one registered result crosses back;
            // the machine domain must never reconverge two async sources.
            cpu_core_quiescent_ack <= cpu_core_hold_sync2 &&
                                      cpu_core_hold_quiesced &&
                                      !cpu_req_busy;
        end
    end

    always @(posedge machine_clk or negedge machine_rst_n) begin
        if (!machine_rst_n) begin
            machine_core_hold_ack_sync1 <= 1'b0;
            machine_core_hold_ack_sync2 <= 1'b0;
            machine_dmem_fence_ack_sync1 <= 1'b0;
            machine_dmem_fence_ack_sync2 <= 1'b0;
        end else begin
            // Hold acknowledgement also proves that an older CPU mailbox
            // transaction has completed.  The machine may then commit a DMA
            // write without a delayed cache-owned writeback overtaking it.
            machine_core_hold_ack_sync1 <= cpu_core_quiescent_ack;
            machine_core_hold_ack_sync2 <= machine_core_hold_ack_sync1;
            machine_dmem_fence_ack_sync1 <= cpu_dmem_fence_sync2;
            machine_dmem_fence_ack_sync2 <= machine_dmem_fence_ack_sync1;
        end
    end
endmodule

`default_nettype wire
