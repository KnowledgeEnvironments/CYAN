// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

//////////////////////////////////////////////////////////////////////////////
// DSP56001 HPC1 Host Interface
//
// Y-peripheral registers at Y:$FFE0-$FFED (DSP-side view).
// Shared registers between CPU and DSP, implementing handshake protocol.
//
// DSP Address | Register   | CPU Equivalent      | Notes
// ------------|------------|---------------------|------
// Y:$FFE0     | DMAWDCNT   | HPC1DMAWDCNT 0x180  | DMA word count
// Y:$FFE1     | GIOADDL    | HPC1GIOADDL  0x184  | GIO address low
// Y:$FFE2     | GIOADDM    | HPC1GIOADDM  0x188  | GIO address mid
// Y:$FFE3     | PBUSADD    | HPC1PBUSADD  0x18C  | PBUS (SRAM) address
// Y:$FFE4     | DMACTRL    | HPC1DMACTRL  0x190  | DMA control (GO/DIR/MODE)
// Y:$FFE5     | COUNT      | HPC1COUNTER  0x194  | 24-bit GIO clock (RO)
// Y:$FFE6     | HANDTX     | HPC1HANDTX   0x198  | DSP->CPU handshake
// Y:$FFE7     | HANDRX     | HPC1HANDRX   0x19C  | CPU->DSP handshake
// Y:$FFE8     | INTSTAT    | HPC1CINTSTAT 0x1A0  | Interrupt status
// Y:$FFE9     | INTMASK    | HPC1CINTMASK 0x1A4  | Interrupt mask
// Y:$FFEA     | INTCONF    | (no CPU equiv)      | Interrupt config
// Y:$FFEB     | INTPOL     | (no CPU equiv)      | Interrupt polarity
// Y:$FFEC     | MISCCSR    | HPC1MISCSR   0x1B0  | Misc control/status
// Y:$FFED     | BURSTCTL   | HPC1BURSTCTL 0x1B4  | DMA burst control
//////////////////////////////////////////////////////////////////////////////

module dsp56001_host #(
    // IP20 runs the GIO clock at two thirds of its 50 MHz SysAD clock.  Keep
    // the ratio explicit: this module is clocked by the CYAN machine clock,
    // but HPC1COUNTER is architecturally clocked by GIO.
    parameter integer GIO_COUNTER_NUMERATOR = 2,
    parameter integer GIO_COUNTER_DENOMINATOR = 3
) (
    input  wire        clk,
    input  wire        rst_n,

    // ========================================================================
    // DSP-Side Y-Peripheral Interface (Y:$FFE0-$FFED)
    // ========================================================================
    input  wire [4:0]  dsp_addr,          // 5-bit address (0x00-0x1F)
    input  wire [23:0] dsp_wdata,
    output reg  [23:0] dsp_rdata,
    input  wire        dsp_rd_en,
    input  wire        dsp_wr_en,

    // ========================================================================
    // CPU-Side Register Interface (from HPC1)
    // ========================================================================
    input  wire [4:0]  cpu_addr,          // Register select (offset from 0x180)
    input  wire [23:0] cpu_wdata,
    output reg  [23:0] cpu_rdata,
    input  wire        cpu_rd_en,
    input  wire        cpu_wr_en,

    // ========================================================================
    // Interrupt Outputs
    // ========================================================================
    output wire        irq_to_cpu,        // Interrupt to CPU (via INT2)
    output wire        irq_to_dsp,        // Host-event interrupt to DSP (IRQA)

    // ========================================================================
    // DMA Control Outputs
    // ========================================================================
    output wire        dma_go,            // DMA transfer active
    output wire        dma_dir,           // 0=CPU->DSP, 1=DSP->CPU
    output wire        dma_mode,          // 0=normal, 1=folded
    output wire [15:0] dma_count,         // Word count
    output wire [31:0] dma_gio_addr,      // GIO bus address
    output wire [15:0] dma_pbus_addr,     // PBUS (SRAM) address
    output wire [2:0]  dma_sign_mode,     // MISCSR[6:4] PBUS extension mode
    input  wire [15:0] dma_count_current, // Live engine words remaining
    input  wire        dma_done,          // DMA transfer complete

    // ========================================================================
    // Reset/Control
    // ========================================================================
    output wire        dsp_reset,         // DSP reset from MISCSR
    output wire        dsp_irqa,          // Force IRQA from MISCSR

    // Non-invasive live state for FPGA debug probes.
    output wire [55:0] debug_status
);

    // ========================================================================
    // Register addresses (offset from Y:$FFE0 base)
    // ========================================================================
    localparam [4:0] R_DMAWDCNT = 5'h00;  // Y:$FFE0
    localparam [4:0] R_GIOADDL  = 5'h01;  // Y:$FFE1
    localparam [4:0] R_GIOADDM  = 5'h02;  // Y:$FFE2
    localparam [4:0] R_PBUSADD  = 5'h03;  // Y:$FFE3
    localparam [4:0] R_DMACTRL  = 5'h04;  // Y:$FFE4
    localparam [4:0] R_COUNT    = 5'h05;  // Y:$FFE5
    localparam [4:0] R_HANDTX   = 5'h06;  // Y:$FFE6
    localparam [4:0] R_HANDRX   = 5'h07;  // Y:$FFE7
    localparam [4:0] R_INTSTAT  = 5'h08;  // Y:$FFE8
    localparam [4:0] R_INTMASK  = 5'h09;  // Y:$FFE9
    localparam [4:0] R_INTCONF  = 5'h0A;  // Y:$FFEA
    localparam [4:0] R_INTPOL   = 5'h0B;  // Y:$FFEB
    localparam [4:0] R_MISCCSR  = 5'h0C;  // Y:$FFEC
    localparam [4:0] R_BURSTCTL = 5'h0D;  // Y:$FFED

    // CPU-side register offsets (from HPC1 base 0x180, divided by 4)
    localparam [4:0] C_DMAWDCNT = 5'h00;  // 0x180
    localparam [4:0] C_GIOADDL  = 5'h01;  // 0x184
    localparam [4:0] C_GIOADDM  = 5'h02;  // 0x188
    localparam [4:0] C_PBUSADD  = 5'h03;  // 0x18C
    localparam [4:0] C_DMACTRL  = 5'h04;  // 0x190
    localparam [4:0] C_COUNTER  = 5'h05;  // 0x194
    localparam [4:0] C_HANDTX   = 5'h06;  // 0x198
    localparam [4:0] C_HANDRX   = 5'h07;  // 0x19C
    localparam [4:0] C_CINTSTAT = 5'h08;  // 0x1A0
    localparam [4:0] C_CINTMASK = 5'h09;  // 0x1A4
    localparam [4:0] C_MISCSR   = 5'h0C;  // 0x1B0
    localparam [4:0] C_BURSTCTL = 5'h0D;  // 0x1B4

    // ========================================================================
    // Shared Register Storage
    // ========================================================================
    reg [15:0] dmawdcnt;
    reg [15:0] gioaddl;
    reg [15:0] gioaddm;
    reg [15:0] pbusadd;
    reg [2:0]  dmactrl;       // GO, DIR, MODE
    reg [15:0] handtx;        // DSP -> CPU handshake
    reg [15:0] handrx;        // CPU -> DSP handshake
    reg [2:0]  intstat;       // DMA(0), TX(1), RX(2)
    reg [2:0]  intmask;
    // DSP peripheral-space interrupt state is distinct from the CPU-facing
    // Mongoose CINTSTAT/CINTMASK state.
    reg [7:0]  dsp_intstat;
    reg [7:0]  dsp_intstat_next;
    reg [7:0]  dsp_intmask;
    reg [23:0] intconf;
    reg [23:0] intpol;
    reg [7:0]  misccsr;       // RESET(0), IRQA(1), POL(2), 32K(3), sign-ext(6:4)
    reg [15:0] burstctl;
    reg [23:0] counter;
    reg [31:0] counter_phase;

    wire [31:0] counter_phase_sum =
        counter_phase + GIO_COUNTER_NUMERATOR;
    wire counter_tick =
        counter_phase_sum >= GIO_COUNTER_DENOMINATOR;

    // ========================================================================
    // Interrupt Status Bits
    // ========================================================================
    localparam CINT_DMA = 0;  // End of DMA
    localparam CINT_TX  = 1;  // DSP wrote HANDTX
    localparam CINT_RX  = 2;  // DSP read HANDRX

    // The DSP-side interrupt register is not the three-bit CPU CINTSTAT.
    // The interface assigns DMA_DONE=0, HANDTX=5, and HANDRX=6. Status
    // writes are write-zero-to-clear; a host event coincident with a clear
    // wins so a transaction cannot be lost.
    always @(*) begin
        dsp_intstat_next = dsp_intstat;
        if (dsp_wr_en && dsp_addr == R_INTSTAT)
            dsp_intstat_next = dsp_intstat_next & dsp_wdata[7:0];
        if (dma_done)
            dsp_intstat_next[0] = 1'b1;
        if (cpu_rd_en && cpu_addr == C_HANDTX)
            dsp_intstat_next[5] = 1'b1;
        if (cpu_wr_en && cpu_addr == C_HANDRX)
            dsp_intstat_next[6] = 1'b1;
    end

    // ========================================================================
    // Register Access Logic
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dmawdcnt <= 16'h0;
            gioaddl  <= 16'h0;
            gioaddm  <= 16'h0;
            pbusadd  <= 16'h0;
            dmactrl  <= 3'h0;
            handtx   <= 16'h0;
            handrx   <= 16'h0;
            intstat  <= 3'h0;
            intmask  <= 3'h0;
            dsp_intstat <= 8'h0;
            dsp_intmask <= 8'h0;
            intconf  <= 24'h0;
            intpol   <= 24'h0;
            misccsr  <= 8'h01;   // RESET asserted on power-up
            burstctl <= 16'h0;
            counter <= 24'h0;
            counter_phase <= 32'h0;
        end else begin
            if (counter_tick) begin
                counter <= counter + 24'd1;
                counter_phase <= counter_phase_sum - GIO_COUNTER_DENOMINATOR;
            end else begin
                counter_phase <= counter_phase_sum;
            end

            dsp_intstat <= dsp_intstat_next;

            // DMA completion sets status
            if (dma_done) begin
                intstat[CINT_DMA] <= 1'b1;
                dmactrl[0] <= 1'b0;  // Clear GO
            end

            // DSP-side writes
            if (dsp_wr_en) begin
                case (dsp_addr)
                    R_DMAWDCNT: dmawdcnt <= dsp_wdata[15:0];
                    R_GIOADDL:  gioaddl  <= dsp_wdata[15:0];
                    R_GIOADDM:  gioaddm  <= dsp_wdata[15:0];
                    R_PBUSADD:  pbusadd  <= dsp_wdata[15:0];
                    R_DMACTRL:  dmactrl  <= dsp_wdata[2:0];
                    R_HANDTX: begin
                        handtx <= dsp_wdata[15:0];
                        intstat[CINT_TX] <= 1'b1;  // Signal CPU
                    end
                    R_INTMASK:  dsp_intmask <= dsp_wdata[7:0];
                    R_INTCONF:  intconf  <= dsp_wdata;
                    R_INTPOL:   intpol   <= dsp_wdata;
                    R_BURSTCTL: burstctl <= dsp_wdata[15:0];
                    default: ;
                endcase
            end

            // DSP-side read of HANDRX sets RX interrupt
            if (dsp_rd_en && dsp_addr == R_HANDRX) begin
                intstat[CINT_RX] <= 1'b1;  // Signal CPU that DSP read HANDRX
            end

            // CPU-side writes
            if (cpu_wr_en) begin
                case (cpu_addr)
                    C_DMAWDCNT: dmawdcnt <= cpu_wdata[15:0];
                    C_GIOADDL:  gioaddl  <= cpu_wdata[15:0];
                    C_GIOADDM:  gioaddm  <= cpu_wdata[15:0];
                    C_PBUSADD:  pbusadd  <= cpu_wdata[15:0];
                    C_DMACTRL:  dmactrl  <= cpu_wdata[2:0];
                    C_HANDRX:   handrx   <= cpu_wdata[15:0];
                    // Software writes -3 to clear TX (bit 1): zero bits clear,
                    // while one bits preserve other pending status.  A DMA
                    // completion, HANDTX write, or HANDRX read on this same
                    // edge is a new event and wins over the write-zero-to-
                    // clear update.  Old causes still follow the CPU mask.
                    C_CINTSTAT: intstat  <=
                        (intstat & cpu_wdata[2:0]) |
                        {
                            dsp_rd_en && dsp_addr == R_HANDRX,
                            dsp_wr_en && dsp_addr == R_HANDTX,
                            dma_done
                        };
                    C_CINTMASK: intmask  <= cpu_wdata[2:0];
                    C_MISCSR:   misccsr  <= cpu_wdata[7:0];
                    C_BURSTCTL: burstctl <= cpu_wdata[15:0];
                    default: ;
                endcase
            end
        end
    end

    // ========================================================================
    // DSP-Side Read
    // ========================================================================
    always @(*) begin
        dsp_rdata = 24'h0;
        case (dsp_addr)
            R_DMAWDCNT: dsp_rdata = {8'h0, dmawdcnt};
            R_GIOADDL:  dsp_rdata = {8'h0, gioaddl};
            R_GIOADDM:  dsp_rdata = {8'h0, gioaddm};
            R_PBUSADD:  dsp_rdata = {8'h0, pbusadd};
            R_DMACTRL:  dsp_rdata = {21'h0, dmactrl};
            R_COUNT:    dsp_rdata = counter;
            R_HANDTX:   dsp_rdata = {8'h0, handtx};
            R_HANDRX:   dsp_rdata = {8'h0, handrx};
            R_INTSTAT:  dsp_rdata = {16'h0, dsp_intstat};
            R_INTMASK:  dsp_rdata = {16'h0, dsp_intmask};
            R_INTCONF:  dsp_rdata = intconf;
            R_INTPOL:   dsp_rdata = intpol;
            R_MISCCSR:  dsp_rdata = {16'h0, misccsr};
            R_BURSTCTL: dsp_rdata = {8'h0, burstctl};
            default:    dsp_rdata = 24'h0;
        endcase
    end

    // ========================================================================
    // CPU-Side Read
    // ========================================================================
    always @(*) begin
        cpu_rdata = 24'h0;
        case (cpu_addr)
            C_DMAWDCNT: cpu_rdata = {8'h0, dmawdcnt};
            C_GIOADDL:  cpu_rdata = {8'h0, gioaddl};
            C_GIOADDM:  cpu_rdata = {8'h0, gioaddm};
            C_PBUSADD:  cpu_rdata = {8'h0, pbusadd};
            C_DMACTRL:  cpu_rdata = {21'h0, dmactrl};
            C_COUNTER:  cpu_rdata = counter;
            C_HANDTX:   cpu_rdata = {8'h0, handtx};
            C_CINTSTAT: cpu_rdata = {21'h0, intstat};
            C_CINTMASK: cpu_rdata = {21'h0, intmask};
            C_MISCSR:   cpu_rdata = {16'h0, misccsr};
            C_BURSTCTL: cpu_rdata = {8'h0, burstctl};
            default:    cpu_rdata = 24'h0;
        endcase
    end

    // ========================================================================
    // Control Outputs
    // ========================================================================
    assign dsp_reset = misccsr[0];         // HPC1MISC_RESET
    // This output is the synchronized logical assertion consumed by the DSP
    // core, not the physical package-pin level.  MISCSR.POL selects which pin
    // level represents an asserted /IRQA; it must not invert FORCE at this
    // already-normalized boundary.
    assign dsp_irqa  = misccsr[1];

    assign irq_to_cpu = |(intstat & intmask);
    assign irq_to_dsp = |(dsp_intstat & dsp_intmask);

    assign dma_go       = dmactrl[0];
    assign dma_dir      = dmactrl[1];
    assign dma_mode     = dmactrl[2];
    assign dma_count    = dmawdcnt;
    assign dma_gio_addr = {gioaddm, gioaddl};
    assign dma_pbus_addr = pbusadd;
    assign dma_sign_mode = misccsr[6:4];

    assign debug_status = {
        10'd0, misccsr, intmask, intstat, handtx, handrx
    };

endmodule
