// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - HPC1 shared DMA-port arbiter
//
// The SCSI and Ethernet engines share one completion-style memory port.  A
// requester must keep its request stable until mem_ready returns.  Arbitration
// therefore has to remain locked for the whole outstanding transaction; a
// priority mux may not switch to a newly asserted request while the first
// request is waiting for its response.
//
// HPC is a real-time GIO requester, but the IP20 MC does not enforce the HPC
// tenure limit itself.  The aggregate HPC port must relinquish GIO after its
// service allowance even when the SCSI/Ethernet engines leave short gaps
// between requests.  Finish an already-presented memory transaction before
// yielding so a coherent read probe or DDR completion can never be withdrawn.

`default_nettype none

module hpc1_dma_arbiter #(
    parameter integer CLK_HZ = 50000000,
    parameter integer SERVICE_US = 10,
    parameter integer RELEASE_US = 20,
    parameter integer MAX_INTERBEAT_GAP_CYCLES = 1
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [31:0] scsi_addr,
    input  wire [63:0] scsi_wdata,
    input  wire [ 7:0] scsi_wmask,
    input  wire        scsi_valid,
    input  wire        scsi_write,
    output wire        scsi_ready,

    input  wire [31:0] enet_addr,
    input  wire [63:0] enet_wdata,
    input  wire [ 7:0] enet_wmask,
    input  wire        enet_valid,
    input  wire        enet_write,
    output wire        enet_ready,

    input  wire [31:0] dsp_addr,
    input  wire [63:0] dsp_wdata,
    input  wire [ 7:0] dsp_wmask,
    input  wire        dsp_valid,
    input  wire        dsp_write,
    output wire        dsp_ready,

    output wire [31:0] mem_addr,
    output wire [63:0] mem_wdata,
    output wire [ 7:0] mem_wmask,
    output wire        mem_valid,
    output wire        mem_write,
    input  wire        mem_ready
);
    localparam integer CYCLES_PER_US =
        ((CLK_HZ / 1000000) > 0) ? (CLK_HZ / 1000000) : 1;
    localparam integer SERVICE_CYCLES = SERVICE_US * CYCLES_PER_US;
    localparam integer EPOCH_CYCLES =
        (SERVICE_US + RELEASE_US) * CYCLES_PER_US;
    localparam integer SERVICE_COUNT_WIDTH =
        (SERVICE_CYCLES <= 1) ? 1 : $clog2(SERVICE_CYCLES);
    localparam integer EPOCH_COUNT_WIDTH =
        (EPOCH_CYCLES <= 1) ? 1 : $clog2(EPOCH_CYCLES);
    localparam integer GAP_COUNT_WIDTH =
        (MAX_INTERBEAT_GAP_CYCLES <= 0) ? 1 :
        $clog2(MAX_INTERBEAT_GAP_CYCLES + 1);
    /* verilator lint_off WIDTHTRUNC */
    localparam [SERVICE_COUNT_WIDTH-1:0] SERVICE_COUNT_LAST =
        SERVICE_CYCLES - 1;
    localparam [EPOCH_COUNT_WIDTH-1:0] EPOCH_COUNT_LAST =
        EPOCH_CYCLES - 1;
    localparam [GAP_COUNT_WIDTH-1:0] INTERBEAT_GAP_LAST =
        MAX_INTERBEAT_GAP_CYCLES;
    /* verilator lint_on WIDTHTRUNC */

    localparam [2:0] TENURE_IDLE      = 3'd0;
    localparam [2:0] TENURE_SERVICE   = 3'd1;
    localparam [2:0] TENURE_GAP_CHECK = 3'd2;
    localparam [2:0] TENURE_DRAIN     = 3'd3;
    localparam [2:0] TENURE_RELEASE   = 3'd4;

    reg owner_locked;
    localparam [1:0] OWNER_SCSI = 2'd0;
    localparam [1:0] OWNER_ENET = 2'd1;
    localparam [1:0] OWNER_DSP  = 2'd2;

    reg [1:0] owner;
    reg [2:0] tenure_state;
    reg [SERVICE_COUNT_WIDTH-1:0] service_count;
    reg [EPOCH_COUNT_WIDTH-1:0] epoch_count;
    reg [GAP_COUNT_WIDTH-1:0] gap_count;
    reg drain_epoch_elapsed;

    // SCSI then Ethernet retain the historical fixed priority when the port
    // is idle; DSP is the new third HPC client.  Once a requester presents a
    // delayed transaction, preserve that exact owner through completion.
    // Once either requester presents a transaction that does not complete in
    // the same cycle, preserve that owner until mem_ready returns.
    wire [1:0] selected_owner = owner_locked ? owner :
                                scsi_valid ? OWNER_SCSI :
                                enet_valid ? OWNER_ENET : OWNER_DSP;
    wire select_scsi = selected_owner == OWNER_SCSI;
    wire select_enet = selected_owner == OWNER_ENET;
    wire select_dsp = selected_owner == OWNER_DSP;
    wire selected_valid = select_scsi ? scsi_valid :
                          select_enet ? enet_valid : dsp_valid;
    wire service_open = (tenure_state != TENURE_GAP_CHECK) &&
                        (tenure_state != TENURE_RELEASE);

    assign mem_addr  = select_scsi ? scsi_addr :
                       select_enet ? enet_addr : dsp_addr;
    assign mem_wdata = select_scsi ? scsi_wdata :
                       select_enet ? enet_wdata : dsp_wdata;
    assign mem_wmask = select_scsi ? scsi_wmask :
                       select_enet ? enet_wmask : dsp_wmask;
    assign mem_valid = selected_valid && service_open;
    assign mem_write = select_scsi ? scsi_write :
                       select_enet ? enet_write : dsp_write;

    assign scsi_ready = mem_ready && mem_valid && select_scsi;
    assign enet_ready = mem_ready && mem_valid && select_enet;
    assign dsp_ready = mem_ready && mem_valid && select_dsp;
    wire selected_ready = mem_ready && mem_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            owner_locked <= 1'b0;
            owner <= OWNER_SCSI;
        end else if (owner_locked) begin
            if (selected_ready)
                owner_locked <= 1'b0;
        end else if (selected_valid && !selected_ready) begin
            owner_locked <= 1'b1;
            owner <= selected_owner;
        end
    end

    // Account one aggregate tenure within a fixed 30-us epoch.  The production
    // descriptor engines leave one low-valid clock while advancing between
    // beats, so bridge that bubble instead of treating it as a fresh grant.
    // A longer idle interval means HPC really has ceded GIO; replenish the
    // allowance rather than throttling later sparse device-side work.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tenure_state <= TENURE_IDLE;
            service_count <= {SERVICE_COUNT_WIDTH{1'b0}};
            epoch_count <= {EPOCH_COUNT_WIDTH{1'b0}};
            gap_count <= {GAP_COUNT_WIDTH{1'b0}};
            drain_epoch_elapsed <= 1'b0;
        end else begin
            case (tenure_state)
                TENURE_IDLE: begin
                    service_count <= {SERVICE_COUNT_WIDTH{1'b0}};
                    epoch_count <= {EPOCH_COUNT_WIDTH{1'b0}};
                    gap_count <= {GAP_COUNT_WIDTH{1'b0}};
                    drain_epoch_elapsed <= 1'b0;
                    if (selected_valid) begin
                        if (SERVICE_CYCLES <= 1) begin
                            if (selected_ready) begin
                                epoch_count <= {{(EPOCH_COUNT_WIDTH-1){1'b0}},
                                                1'b1};
                                tenure_state <= TENURE_RELEASE;
                            end else begin
                                tenure_state <= TENURE_DRAIN;
                            end
                        end else begin
                            service_count <= {{(SERVICE_COUNT_WIDTH-1){1'b0}},
                                              1'b1};
                            epoch_count <= {{(EPOCH_COUNT_WIDTH-1){1'b0}},
                                            1'b1};
                            tenure_state <= TENURE_SERVICE;
                        end
                    end
                end

                TENURE_SERVICE: begin
                    if (epoch_count == EPOCH_COUNT_LAST) begin
                        // A new accounting epoch replenishes unused service.
                        // The currently visible cycle belongs to the old one.
                        epoch_count <= {EPOCH_COUNT_WIDTH{1'b0}};
                        service_count <= {SERVICE_COUNT_WIDTH{1'b0}};
                        gap_count <= {GAP_COUNT_WIDTH{1'b0}};
                        if (!selected_valid)
                            tenure_state <= TENURE_IDLE;
                    end else if (!selected_valid &&
                                 (gap_count == INTERBEAT_GAP_LAST)) begin
                        // More than the known inter-beat bubble is a genuine
                        // aggregate cede, so a later request gets a new budget.
                        epoch_count <= {EPOCH_COUNT_WIDTH{1'b0}};
                        service_count <= {SERVICE_COUNT_WIDTH{1'b0}};
                        gap_count <= {GAP_COUNT_WIDTH{1'b0}};
                        tenure_state <= TENURE_IDLE;
                    end else begin
                        epoch_count <= epoch_count + 1'b1;
                        if (selected_valid)
                            gap_count <= {GAP_COUNT_WIDTH{1'b0}};
                        else
                            gap_count <= gap_count + 1'b1;

                        if (service_count == SERVICE_COUNT_LAST) begin
                            service_count <= {SERVICE_COUNT_WIDTH{1'b0}};
                            if (selected_valid && !selected_ready)
                                tenure_state <= TENURE_DRAIN;
                            else if (selected_valid)
                                tenure_state <= TENURE_RELEASE;
                            else
                                tenure_state <= TENURE_GAP_CHECK;
                        end else begin
                            service_count <= service_count + 1'b1;
                        end
                    end
                end

                TENURE_GAP_CHECK: begin
                    // The service deadline can coincide with the first known
                    // one-cycle descriptor bubble.  Wait one closed cycle to
                    // distinguish that bubble from a genuine aggregate cede.
                    // A returning request remains held for the release window;
                    // a second idle cycle replenishes the complete allowance.
                    if (selected_valid) begin
                        epoch_count <= epoch_count + 1'b1;
                        tenure_state <= TENURE_RELEASE;
                    end else begin
                        epoch_count <= {EPOCH_COUNT_WIDTH{1'b0}};
                        service_count <= {SERVICE_COUNT_WIDTH{1'b0}};
                        gap_count <= {GAP_COUNT_WIDTH{1'b0}};
                        tenure_state <= TENURE_IDLE;
                    end
                end

                TENURE_DRAIN: begin
                    // A raw request is already visible downstream.  Keep it
                    // presented until its exact completion, then start release.
                    if (epoch_count == EPOCH_COUNT_LAST) begin
                        drain_epoch_elapsed <= 1'b1;
                    end else if (!drain_epoch_elapsed) begin
                        epoch_count <= epoch_count + 1'b1;
                    end
                    if (selected_ready) begin
                        service_count <= {SERVICE_COUNT_WIDTH{1'b0}};
                        if (drain_epoch_elapsed ||
                            (epoch_count == EPOCH_COUNT_LAST)) begin
                            // Even an atomic beat that outlives the epoch must
                            // produce one observable closed cycle after it
                            // drains before a waiting child can start again.
                            epoch_count <= EPOCH_COUNT_LAST;
                            drain_epoch_elapsed <= 1'b0;
                            tenure_state <= TENURE_RELEASE;
                        end else begin
                            tenure_state <= TENURE_RELEASE;
                        end
                    end
                end

                TENURE_RELEASE: begin
                    if ((EPOCH_CYCLES <= 1) ||
                        (epoch_count == EPOCH_COUNT_LAST)) begin
                        epoch_count <= {EPOCH_COUNT_WIDTH{1'b0}};
                        service_count <= {SERVICE_COUNT_WIDTH{1'b0}};
                        gap_count <= {GAP_COUNT_WIDTH{1'b0}};
                        tenure_state <= TENURE_IDLE;
                    end else begin
                        epoch_count <= epoch_count + 1'b1;
                    end
                end

                default: begin
                    tenure_state <= TENURE_IDLE;
                    service_count <= {SERVICE_COUNT_WIDTH{1'b0}};
                    epoch_count <= {EPOCH_COUNT_WIDTH{1'b0}};
                    gap_count <= {GAP_COUNT_WIDTH{1'b0}};
                    drain_epoch_elapsed <= 1'b0;
                end
            endcase
        end
    end
endmodule

`default_nettype wire
