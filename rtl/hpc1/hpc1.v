// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - HPC1.5 Peripheral Controller
// SGI IP20 uses HPC1 (rev 1.5), NOT HPC3 (which is IP22/Indy)
// Register layout matches HPC1 offsets from IP20.h
//
// Address map (offset from HPC1 base 0x1FB80000):
//   0x000-0x0FF: SCSI DMA control
//   0x0A8-0x0BF: Parallel port
//   0x120-0x1F0: Misc control registers
//   0x1B8-0x1BF: CPU_AUX_CONTROL (EEPROM bit-bang, LED)
//   0xD00-0xD30: Z8530 SCC (DUART0/1)
//
// This module handles address decoding and register access.
// Sub-modules handle specific peripherals (EEPROM, SCC, etc.)

`default_nettype none

module hpc1 (
    input  wire        clk,
    input  wire        rst_n,

    // CPU bus interface (directly from MC)
    input  wire [11:0] addr,       // Offset within HPC1 address space
    input  wire [63:0] wdata,
    output wire [63:0] rdata,
    input  wire [ 7:0] wmask,
    input  wire        valid,
    input  wire        write,
    output wire        ready,

    // Serial port outputs (directly from SCC)
    output wire        scc_txd_a,  // Channel A TX
    output wire        scc_txd_b,  // Channel B TX (console)
    input  wire        scc_rxd_a,  // Channel A RX
    input  wire        scc_rxd_b,  // Channel B RX (console)

    // EEPROM interface (directly exposed for potential external chip)
    output wire        eeprom_cs,
    output wire        eeprom_clk,
    output wire        eeprom_di,
    input  wire        eeprom_do,

    // Status outputs
    output wire        console_led,
    output wire [7:0]  scc_console_tx_data,
    output wire        scc_console_tx_valid,

    // RX injection (for simulation) - Channel B console
    input  wire [7:0]  rx_inject_data,
    input  wire        rx_inject_valid,

    // Keyboard (SCC0 Channel A) - monitoring and injection
    output wire [7:0]  kbd_tx_data,
    output wire        kbd_tx_valid,
    input  wire [7:0]  kbd_rx_data,
    input  wire        kbd_rx_valid,
    output wire [7:0]  mouse_tx_data,
    output wire        mouse_tx_valid,
    input  wire [7:0]  mouse_rx_data,
    input  wire        mouse_rx_valid,
    output wire [31:0] scc0_access_event_data,
    output wire        scc0_access_event_valid,
    output wire [127:0] scc_debug_probe,

    // SCSI target interface (for C++ disk backend or synthesizable backend)
    input  wire [7:0]  scsi_target_present,  // Bitmask of present SCSI targets
    output wire        scsi_cmd_pending,      // Command ready for C++ processing
    input  wire        scsi_cmd_done,         // C++ completed the command
    input  wire [7:0]  scsi_cmd_status,       // Status from C++
    input  wire [23:0] scsi_resp_len,         // Total SCSI response bytes (from C++)
    input  wire        scsi_status_valid,     // External write backend status ready
    output wire [2:0]  scsi_target_id,
    output wire [95:0] scsi_cdb,
    output wire [23:0] scsi_xfer_count,
    output wire [23:0] scsi_data_index,
    output wire        scsi_bus_reset,
    input  wire [7:0]  scsi_read_data,
    input  wire        scsi_read_valid,
    output wire        scsi_read_consume,
    output wire        scsi_write_valid,
    output wire [7:0]  scsi_write_data,
    input  wire        scsi_write_ready,
    output wire        scsi_data_out_done,

    // DSP56001 register interface (passthrough to external DSP module)
    // HPC1 registers 0x180-0x1B4 are DSP control/handshake registers
    output wire [4:0]  dsp_reg_addr,
    output wire [23:0] dsp_reg_wdata,
    input  wire [23:0] dsp_reg_rdata,
    output wire        dsp_reg_rd_en,
    output wire        dsp_reg_wr_en,
    input  wire        dsp_irq,

    // DSP-side access to the same IP20 RTC reached by the CPU at 0xE00.
    // Its bit-7-selected RAM page follows the DP8572A-compatible path.
    // The IP20 DSP decodes this device at Y:$FFC0-$FFDF.
    input  wire [4:0]  dsp_rtc_addr,
    input  wire [23:0] dsp_rtc_wdata,
    output wire [23:0] dsp_rtc_rdata,
    input  wire        dsp_rtc_rd_en,
    input  wire        dsp_rtc_wr_en,

    // DSP DMA is a third child of the shared HPC real-time memory master.
    input  wire [31:0] dsp_dma_mem_addr,
    input  wire [63:0] dsp_dma_mem_wdata,
    input  wire [7:0]  dsp_dma_mem_wmask,
    input  wire        dsp_dma_mem_valid,
    input  wire        dsp_dma_mem_write,
    output wire        dsp_dma_mem_ready,
    input  wire        dsp_dma_active,

    // LG1/VC1 raw vertical-retrace event.  IP20 INT2 latches this one-clock
    // pulse into LIO1 bit 7 until PCON_CLEARVRI is driven low.
    input  wire        gfx_retrace_irq,

    // MC qualified virtual-DMA interrupt. IP20 exposes this level on LIO0
    // bit 4; DMA_CAUSE provides the sticky status and acknowledge behavior.
    input  wire        gfx_dma_irq,

    // SCSI DMA bus master interface
    output wire [31:0] dma_mem_addr,
    output wire [63:0] dma_mem_wdata,
    input  wire [63:0] dma_mem_rdata,
    output wire [ 7:0] dma_mem_wmask,
    output wire        dma_mem_valid,
    output wire        dma_mem_write,
    input  wire        dma_mem_ready,

    // DMA status
    output wire        dma_active,

    // Ethernet packet stream for Verilator harness or FPGA MAC/PHY glue
    output wire [7:0]  enet_tx_data,
    output wire        enet_tx_valid,
    output wire        enet_tx_sof,
    output wire        enet_tx_eof,
    input  wire        enet_tx_ready,
    input  wire [7:0]  enet_rx_data,
    input  wire        enet_rx_valid,
    input  wire        enet_rx_sof,
    input  wire        enet_rx_eof,
    output wire        enet_rx_ready,
    output wire        enet_irq,
    output wire [255:0] enet_debug_probe,
    output wire [767:0] enet_debug_snapshot,
    output wire [255:0] scsi_debug_probe,

    // Timer interrupt outputs (8254 counter 0 and 1)
    output reg         timer_irq_0,   // Counter 0 (scheduling clock) → CPU IP5
    output reg         timer_irq_1,   // Counter 1 (profiling clock) → CPU IP6

    // SCSI interrupt output (active high when WD93 has pending interrupt)
    output wire        scsi0_irq,

    // CPU interrupt output (active high, directly to CPU hw_int)
    // On IP20, INT2 is embedded in HPC1 — all interrupt routing is here
    output wire [5:0]  cpu_irq,

    // Retained observation of the vertical-retrace interrupt boundary.  No
    // bit feeds functional logic.  The four 24-bit counters cover more than
    // four days at the programmed ~45.8 Hz frame rate before wrapping.
    output wire [127:0] gfx_retrace_status,

    // IDE diagnostic mode: accelerates 8254 cascade for timer/RTC rate matching
    input  wire        ide_mode
);

    parameter HAS_DSP = 0;
    parameter SCSI_EXTERNAL_TARGET = 0;
    parameter FORCE_SCSI_IRQ_MASK = 0;
    parameter LATCH_SCSI_IRQ_STATUS = 0;
    parameter integer CLK_HZ = 50000000;

`ifndef SYNTHESIS
    reg trace_hpc1;
    reg trace_int2;
    reg trace_int2_raw_d;
    reg trace_int2_pending_d;
    reg trace_int2_cpu_irq1_d;
    reg trace_int2_clearvri_d;
    reg trace_int2_mask7_d;
    initial begin
        trace_hpc1 = 1'b0;
        trace_int2 = 1'b0;
        if ($test$plusargs("trace_hpc1"))
            trace_hpc1 = 1'b1;
        if (trace_hpc1 || $test$plusargs("trace_int2"))
            trace_int2 = 1'b1;
    end
`endif

    // =========================================================================
    // Register declarations
    // =========================================================================

    // CPU_AUX_CONTROL register (offset 0x1BC, byte access)
    // Bit 4: SER_TO_CPU (read) - Data from EEPROM
    // Bit 3: CPU_TO_SER (write) - Data to EEPROM
    // Bit 2: SERCLK - Serial clock
    // Bit 1: CONSOLE_CS - EEPROM chip select
    // Bit 0: CONSOLE_LED / NVRAM_PRE
    reg [7:0] cpu_aux_control;

    // Ethernet/SEEQ register windows are implemented by hpc1_enet.
    wire sel_enet = (addr < 12'h060);
    wire sel_hpc_endian = (addr >= 12'h0C0) && (addr <= 12'h0C3);
    wire sel_seeq = (addr >= 12'h100) && (addr <= 12'h11F);

    // SCSI DMA registers (offsets 0x088-0x09C)
    reg [31:0] scsi_bc;       // 0x088: SCSI byte count
    reg [31:0] scsi_cbp;      // 0x08C: SCSI current buffer pointer
    reg [31:0] scsi_nbdp;     // 0x090: SCSI next buffer descriptor pointer
    reg [31:0] scsi_ctrl;     // 0x094: SCSI control (bit 0 = chip reset)
    reg [31:0] scsi_pntr;     // 0x098: SCSI pointer register
    reg [31:0] scsi_fifo;     // 0x09C: SCSI FIFO data

    assign scsi_bus_reset = scsi_ctrl[0];

    // Parallel port DMA registers (0x0A8-0x0BF)
    reg [31:0] pbus_bc;        // 0x0A8: Parallel byte count
    reg [31:0] pbus_cbp;       // 0x0AC: Parallel current buffer pointer
    reg [31:0] pbus_nbdp;      // 0x0B0: Parallel next buffer desc pointer
    reg [31:0] pbus_ctrl;      // 0x0B4: Parallel control
    reg [31:0] pbus_pntr;      // 0x0B8: Parallel pointer register
    reg [31:0] pbus_fifo;      // 0x0BC: Parallel FIFO data

    // HPC1 FIFO RAMs (16 entries each, for IDE FIFO test)
    // Flags are latched on pointer write, committed on data write.
    // SCSI FIFO (pointer at 0x98, data at 0x9C, ctrl at 0x94)
    reg [31:0] scsi_fifo_ram [0:15];
    reg [3:0]  scsi_fifo_flags [0:15];
    reg [3:0]  scsi_fifo_flag_latch;
    // Parallel FIFO (pointer at 0xB8, data at 0xBC, ctrl at 0xB4)
    reg [31:0] pbus_fifo_ram [0:15];
    reg [3:0]  pbus_fifo_flags [0:15];
    reg [3:0]  pbus_fifo_flag_latch;

    // =========================================================================
    // IP20 INT2 registers (offsets 0x1C0-0x1FF)
    // On IP20, INT2 is within HPC1 PBUS at 0x1FB801C0-0x1FB801FF
    // =========================================================================

    // Mask registers (read/write)
    reg [7:0] lio_0_mask;      // 0x1C4-0x1C7: LIO 0 interrupt mask
    reg [7:0] lio_1_mask;      // 0x1CC-0x1CF: LIO 1 interrupt mask
    reg [7:0] vme_0_mask;      // 0x1D4-0x1D7: VME interrupt mask for LIO 0
    reg [7:0] vme_1_mask;      // 0x1D8-0x1DB: VME interrupt mask for LIO 1
    reg [7:0] int2_port_config; // 0x1DC-0x1DF: Port configuration
    reg       gfx_retrace_pending;
    reg [23:0] gfx_retrace_raw_count;
    reg [23:0] gfx_retrace_armed_raw_count;
    reg [23:0] gfx_retrace_clear_write_count;
    reg [23:0] gfx_retrace_arm_write_count;

    // 8254 Timer (minimal implementation for PROM test)
    // Control word format: [7:6]=counter select, [5:4]=RW, [3:1]=mode, [0]=BCD
    reg [15:0] pt_counter_0;
    reg [15:0] pt_counter_1;
    reg [15:0] pt_counter_2;
    reg [15:0] pt_latch_0;       // Latched value for counter 0
    reg [15:0] pt_latch_1;       // Latched value for counter 1
    reg [15:0] pt_latch_2;       // Latched value for counter 2
    reg [7:0]  pt_control;
    reg [2:0]  pt_rw_mode;       // RW mode per counter (0=latch pending, 1=LSB, 2=MSB, 3=LSB/MSB)
    reg [2:0]  pt_byte_sel;      // Which byte to read/write next (0=LSB, 1=MSB)
    reg [2:0]  pt_latched;       // Which counters have latched values pending
    reg [2:0]  pt_count_enabled; // Counter enabled (after both bytes loaded)

    // 8254 master input clock. IP20 software expects counter 2 to receive a
    // 1 MHz input; counter 2's output then clocks counters 0/1 at 5 kHz when
    // software programs counter 2 with MASTER_COUNT=200.
    localparam integer PT_INPUT_HZ = 1000000;
    localparam [31:0] PT_INPUT_MAX =
        ((CLK_HZ / PT_INPUT_HZ) > 0) ? ((CLK_HZ / PT_INPUT_HZ) - 1) : 0;
    reg [31:0] pt_prescale;
    wire [31:0] pt_input_max = ide_mode ? 32'd0 : PT_INPUT_MAX;
    wire pt_input_tick = (pt_prescale >= pt_input_max);

    // Per-counter mode (from control word bits [3:1])
    reg [2:0] pt_mode_0;  // Counter 0 mode
    reg [2:0] pt_mode_1;  // Counter 1 mode
    reg [2:0] pt_mode_2;  // Counter 2 mode

    // Reload values for auto-reload (saved when counter is programmed)
    reg [15:0] pt_reload_0;
    reg [15:0] pt_reload_1;
    reg [15:0] pt_reload_2;

    // Interrupt status (directly from peripheral signals)
    // LIO 0 status bits (from IP20.h):
    //   [0] FIFO/GIO-0     LIO_FIFO   0x01
    //   [1] Centronics      LIO_CENTR  0x02
    //   [2] SCSI            LIO_SCSI   0x04
    //   [3] Ethernet        LIO_ENET   0x08
    //   [4] Graphics DMA    LIO_GDMA   0x10
    //   [5] DUART           LIO_DUART  0x20
    //   [6] GIO-1/2nd HPC   LIO_GIO_1  0x40
    //   [7] VME-0           LIO_VME0   0x80
    // SCSI DMA completion is reported through the WD93/SCSI path on bit [2].
    // Do not alias it onto FIFO/GIO-0: LG1 polls raw LIO_FIFO as REX FIFO
    // state, so a stale SCSI DMA-done status here produces false REX FIFO
    // timeouts during graphics startup.

    wire scc0_irq;
    wire scc1_irq;
    reg scsi0_irq_latched;
    wire scsi0_irq_status =
        (LATCH_SCSI_IRQ_STATUS != 0) ? (scsi0_irq | scsi0_irq_latched) :
                                       scsi0_irq;

    wire [7:0] lio_0_isr = {
        1'b0,           // [7] VME-0
        1'b0,           // [6] GIO-1/2nd HPC
        scc0_irq | scc1_irq,  // [5] DUART
        gfx_dma_irq,    // [4] Qualified graphics DMA interrupt from MC
        enet_irq,       // [3] Ethernet
        scsi0_irq_status, // [2] SCSI
        1'b0,           // [1] Centronics
        1'b0            // [0] FIFO/GIO-0 (not driven by SCSI DMA done)
    };

    // LIO 1 status bits (from IP20.h):
    //   [0] unused
    //   [1] GR1 mode        LIO_GR1MODE 0x02
    //   [2] unused
    //   [3] VME-1           LIO_VME1    0x08
    //   [4] DSP             LIO_DSP     0x10
    //   [5] AC fail         LIO_AC      0x20
    //   [6] Video           LIO_VIDEO   0x40
    //   [7] Vert retrace    LIO_GIO_2   0x80
    wire [7:0] lio_1_isr = {
        gfx_retrace_pending, // [7] GIO-2 / vertical retrace
        2'b00,          // [6:5] video/AC fail
        (HAS_DSP != 0) && dsp_irq, // [4] DSP host-interface interrupt
        4'b0000         // [3:0] VME-1/unused/GR1 mode/unused
    };

    // VME status (IP20 has VME, but we don't have any VME devices)
    wire [7:0] vme_isr = 8'h00;

    // Masked interrupts → CPU IRQ lines
    wire [7:0] effective_lio_0_mask =
        lio_0_mask | (FORCE_SCSI_IRQ_MASK ? 8'h04 : 8'h00);
    wire [7:0] masked_lio0 = lio_0_isr & effective_lio_0_mask;
    wire [7:0] masked_lio1 = lio_1_isr & lio_1_mask;

    // IP20 CPU IRQ mapping (from IP20.c interrupt vector table):
    //   cpu_irq[0] → Cause IP[2] = SR_IBIT3 = LIO0 (SCSI, Ethernet, etc)
    //   cpu_irq[1] → Cause IP[3] = SR_IBIT4 = LIO1 (Power, HPC1, etc)
    //   cpu_irq[2] → Cause IP[4] = SR_IBIT5 = 8254 counter 0 (scheduling/RTCLOCK)
    //   cpu_irq[3] → Cause IP[5] = SR_IBIT6 = 8254 counter 1 (profiling/KGCLOCK)
    //   cpu_irq[4] → Cause IP[6] = SR_IBIT7 = bus error (MC)
    //   cpu_irq[5] → Cause IP[7] = SR_IBIT8 = R4000 Count/Compare (internal)
    assign cpu_irq = {
        1'b0,                   // [5] → IP[7] Count/Compare (internal to CPU)
        1'b0,                   // [4] → IP[6] Bus error (MC)
        timer_irq_1,            // [3] → IP[5] counter 1 (profiling/KGCLOCK)
        timer_irq_0,            // [2] → IP[4] counter 0 (scheduling/RTCLOCK)
        (|masked_lio1),         // [1] → IP[3] LIO1
        (|masked_lio0)          // [0] → IP[2] LIO0
    };

    // Byte order is intentionally network/tool friendly:
    //   0      signature ('R')
    //   1      current PORT_CONFIG
    //   2      current LIO1 mask
    //   3      flags: raw, pending, CPU IRQ1, armed, mask7
    //   4..6   raw LG1 retrace pulses
    //   7..9   raw pulses observed while PCON_CLEARVRI was armed
    //   10..12 accepted PORT_CONFIG writes with PCON_CLEARVRI low
    //   13..15 accepted PORT_CONFIG writes with PCON_CLEARVRI high
    assign gfx_retrace_status = {
        8'h52,
        int2_port_config,
        lio_1_mask,
        3'b000,
        lio_1_mask[7],
        int2_port_config[3],
        cpu_irq[1],
        gfx_retrace_pending,
        gfx_retrace_irq,
        gfx_retrace_raw_count,
        gfx_retrace_armed_raw_count,
        gfx_retrace_clear_write_count,
        gfx_retrace_arm_write_count
    };

`ifndef SYNTHESIS
    // Opt-in trace of the real IP20 interrupt path embedded in HPC1.  State
    // transitions are reported one clock after the transition so the printed
    // values reflect the settled nonblocking assignments from the functional
    // always block below.  This instrumentation is absent from synthesis.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            trace_int2_raw_d <= 1'b0;
            trace_int2_pending_d <= 1'b0;
            trace_int2_cpu_irq1_d <= 1'b0;
            trace_int2_clearvri_d <= 1'b0;
            trace_int2_mask7_d <= 1'b0;
        end else begin
            if (trace_int2) begin
                if (gfx_retrace_irq)
                    $display("IP20_INT2_RETRACE raw t=%0t armed=%0d pending=%0d mask7=%0d cpu_irq1=%0d",
                             $time, int2_port_config[3],
                             gfx_retrace_pending, lio_1_mask[7], cpu_irq[1]);
                if (gfx_retrace_pending != trace_int2_pending_d)
                    $display("IP20_INT2_RETRACE pending t=%0t value=%0d raw=%0d armed=%0d mask7=%0d cpu_irq1=%0d",
                             $time, gfx_retrace_pending, trace_int2_raw_d,
                             int2_port_config[3], lio_1_mask[7], cpu_irq[1]);
                if (int2_port_config[3] != trace_int2_clearvri_d)
                    $display("IP20_INT2_RETRACE clearvri t=%0t value=%0d port_config=%02x pending=%0d",
                             $time, int2_port_config[3], int2_port_config,
                             gfx_retrace_pending);
                if (lio_1_mask[7] != trace_int2_mask7_d)
                    $display("IP20_INT2_RETRACE mask7 t=%0t value=%0d lio1_mask=%02x pending=%0d cpu_irq1=%0d",
                             $time, lio_1_mask[7], lio_1_mask,
                             gfx_retrace_pending, cpu_irq[1]);
                if (cpu_irq[1] != trace_int2_cpu_irq1_d)
                    $display("IP20_INT2_RETRACE cpu_irq1 t=%0t value=%0d pending=%0d mask7=%0d",
                             $time, cpu_irq[1], gfx_retrace_pending,
                             lio_1_mask[7]);
            end
            trace_int2_raw_d <= gfx_retrace_irq;
            trace_int2_pending_d <= gfx_retrace_pending;
            trace_int2_cpu_irq1_d <= cpu_irq[1];
            trace_int2_clearvri_d <= int2_port_config[3];
            trace_int2_mask7_d <= lio_1_mask[7];
        end
    end
`endif

    // (timer debug removed)

    // =========================================================================
    // IP20 RTC registers (offset 0xE00-0xEFF)
    // =========================================================================
    // Register bank 0 (MSR_RS=0): offsets 1-4
    //   0x00: MSR (Main Status Register) - bit 6 = RS (bank select)
    //   0x01: RTMR - bit 3 = CSS (Clock Start/Stop)
    //   0x02: OMR
    //   0x03: Periodic Flag Register
    //   0x04: Time Save Control (TSCR) - bit 7 = TE
    // Register bank 1 (MSR_RS=1): offsets 1-4
    //   0x01: Real Time Mode (ck_rtime1) - bits [4:3] = RTC_RUN
    //   0x02: Output Mode
    //   0x03: Interrupt Control 0
    //   0x04: Interrupt Control 1
    // Offsets 5-14: always visible (time counters)
    //   0x05: hundredths (BCD), 0x06: seconds, 0x07: minutes, 0x08: hours
    //   0x09: day, 0x0A: month, 0x0B: year, 0x0E: day of week
    // Offsets 25-29: time save RAM

    reg [7:0]  rtc_regs[64];       // Bank 0 registers + counters + timesave
    reg [7:0]  rtc_bank1 [0:3];   // Bank 1 registers (rtime, outmode, int0ctl, int1ctl)
    reg [7:0]  rtc_page_ram [0:31]; // RAM page selected by MSR bit 7
    localparam integer RTC_HUNDREDTH_HZ = 100;
    localparam [31:0] RTC_TICK_MAX =
        ((CLK_HZ / RTC_HUNDREDTH_HZ) > 0) ?
        ((CLK_HZ / RTC_HUNDREDTH_HZ) - 1) : 0;
    reg [31:0] rtc_prescaler;     // RTC hundredths tick prescaler
    reg        rtc_te_prev;       // Previous TE bit for falling-edge timesave latch

    // =========================================================================
    // Address decoding
    // =========================================================================

    wire sel_aux_control = (addr >= 12'h1B8) && (addr <= 12'h1BF);
    wire sel_scc0        = (addr >= 12'hD00) && (addr <= 12'hD0F);  // DUART0
    wire sel_scc1        = (addr >= 12'hD10) && (addr <= 12'hD1F);  // DUART1 (console)
    wire sel_scc2        = (addr >= 12'hD20) && (addr <= 12'hD2F);  // DUART2
    wire sel_pbus        = (addr >= 12'h0A8) && (addr <= 12'h0BF);

    // SCSI DMA registers (0x088-0x09F)
    wire sel_scsi_dma    = (addr >= 12'h088) && (addr <= 12'h09F);
    // WD33C93 register ports (byte-addressed, big-endian IP20)
    wire sel_scsi_addr   = (addr == 12'h122);  // SCSI0A: WD93 address register
    wire sel_scsi_data   = (addr == 12'h126);  // SCSI0D: WD93 data register
    wire sel_scsi_wd     = sel_scsi_addr || sel_scsi_data;
    wire scc0_ready;
    wire scc1_ready;

    // IP20 INT2 register selects (0x1C0-0x1FF)
    wire sel_lio_0_isr    = (addr >= 12'h1C0) && (addr <= 12'h1C3);  // LIO 0 status (ro)
    wire sel_lio_0_mask   = (addr >= 12'h1C4) && (addr <= 12'h1C7);  // LIO 0 mask (rw)
    wire sel_lio_1_isr    = (addr >= 12'h1C8) && (addr <= 12'h1CB);  // LIO 1 status (ro)
    wire sel_lio_1_mask   = (addr >= 12'h1CC) && (addr <= 12'h1CF);  // LIO 1 mask (rw)
    wire sel_vme_isr      = (addr >= 12'h1D0) && (addr <= 12'h1D3);  // VME status (ro)
    wire sel_vme_0_mask   = (addr >= 12'h1D4) && (addr <= 12'h1D7);  // VME 0 mask (rw)
    wire sel_vme_1_mask   = (addr >= 12'h1D8) && (addr <= 12'h1DB);  // VME 1 mask (rw)
    wire sel_port_config  = (addr >= 12'h1DC) && (addr <= 12'h1DF);  // Port config (rw)
    wire sel_timer_ack    = (addr >= 12'h1E0) && (addr <= 12'h1E3);  // Timer ack (wo)
    wire sel_pt_clock     = (addr >= 12'h1F0) && (addr <= 12'h1FF);  // 8254 timer

    // DSP56001 HPC1 registers (0x180-0x1B7, 32-bit aligned)
    wire sel_dsp_reg      = HAS_DSP && (addr >= 12'h180) && (addr <= 12'h1B7);

    // IP20 RTC (0xE00-0xEFF), including DP8572A-compatible RAM paging
    wire sel_rtc          = (addr >= 12'hE00) && (addr <= 12'hEFF);

    // RTC byte extraction: the RTC is an 8-bit device at byte lane 3 of each
    // 4-byte slot.
    // For SW (word store), data byte is at the LSByte of the 32-bit half.
    // addr[2] selects upper (0) or lower (1) 32-bit half of 64-bit word.
    wire [7:0] rtc_wbyte = addr[2] ? wdata[7:0] : wdata[39:32];

    wire [4:0] rtc_idx = addr[6:2];
    wire       rtc_ram_page = rtc_regs[0][7];
    wire [7:0] dsp_rtc_rbyte =
        (dsp_rtc_addr != 5'd0 && rtc_ram_page) ?
            rtc_page_ram[dsp_rtc_addr] :
        (dsp_rtc_addr >= 5'd1 && dsp_rtc_addr <= 5'd4 && rtc_regs[0][6]) ?
            rtc_bank1[dsp_rtc_addr - 5'd1] : rtc_regs[dsp_rtc_addr];
    assign dsp_rtc_rdata = (HAS_DSP && dsp_rtc_rd_en) ?
                           {16'd0, dsp_rtc_rbyte} : 24'd0;

    // =========================================================================
    // Byte extraction for big-endian byte access
    // =========================================================================

    // Extract the active byte from wmask/wdata for byte-addressed registers
    wire [7:0] wdata_byte =
        wmask[7] ? wdata[63:56] :
        wmask[6] ? wdata[55:48] :
        wmask[5] ? wdata[47:40] :
        wmask[4] ? wdata[39:32] :
        wmask[3] ? wdata[31:24] :
        wmask[2] ? wdata[23:16] :
        wmask[1] ? wdata[15:8]  :
                   wdata[7:0];

    // Check if any byte is being written
    wire byte_write = |wmask;

    // The R4000 pipeline can retire one uncached/device access and present the
    // next access without dropping valid for an intervening cycle. Keep ready
    // as a one-cycle response pulse: repeated same-address device reads, such
    // as WD93 data-port auto-increment reads, are distinct bus transactions.
    // The accepted-request latch still masks stale ready/rdata in the cycle
    // where the CPU changes a no-idle request after seeing the previous ready.
    reg        ready_r;
    reg [11:0] accepted_addr;
    reg        accepted_write;
    reg [63:0] accepted_wdata;
    reg [ 7:0] accepted_wmask;
    reg        wd93_req_valid;
    reg        wd93_req_sel_addr;
    reg        wd93_req_write;
    reg [7:0]  wd93_req_wdata;
    reg [63:0] rdata_r;
    wire accepted_request_changed =
        (addr != accepted_addr) ||
        (write != accepted_write) ||
        (write && ((wdata != accepted_wdata) ||
                   (wmask != accepted_wmask)));
    // Ordinary registers respond from ready_r.  A buffered WD93 request
    // responds while it is being replayed, so the CPU samples ready/rdata on
    // the same edge that the WD consumes the access.  This preserves the
    // timing cut without adding a PIO wait state.
    assign ready = (ready_r || wd93_req_valid) &&
                   !(valid && accepted_request_changed);
    wire hpc1_sub_ready =
        !wd93_req_valid &&
        (!write ||
         ((!sel_scc0 || scc0_ready) && (!sel_scc1 || scc1_ready)));
    wire hpc1_accept = valid && !ready_r && hpc1_sub_ready;

    // WD33C93 PIO accesses are accepted into a timing-cut request register and
    // replayed to the WD state machine one cycle later.  ready and live read
    // data are exposed during that replay cycle, making the CPU completion and
    // indirect-register auto-increment one atomic clock edge.
    wire wd93_accept = sel_scsi_wd && hpc1_accept;
    wire       wd93_sel_addr = wd93_req_valid ? wd93_req_sel_addr : sel_scsi_addr;

    // =========================================================================
    // EEPROM interface - directly map AUX control bits
    // =========================================================================

    assign eeprom_cs  = cpu_aux_control[1];  // CONSOLE_CS
    assign eeprom_clk = cpu_aux_control[2];  // SERCLK
    assign eeprom_di  = cpu_aux_control[3];  // CPU_TO_SER
    assign console_led = cpu_aux_control[0]; // CONSOLE_LED

    // Build AUX_CONTROL read value with EEPROM data
    wire [7:0] aux_control_read = {3'b0, eeprom_do, cpu_aux_control[3:0]};

    // DSP register index: (offset - 0x180) >> 2 = addr[6:2] (when in DSP range)
    assign dsp_reg_addr = addr[6:2];

    // DSP reg interface signals (directly driven from always block when HAS_DSP=1)
    reg [23:0] dsp_reg_wdata_r;
    reg        dsp_reg_rd_en_r;
    reg        dsp_reg_wr_en_r;
    assign dsp_reg_wdata = dsp_reg_wdata_r;
    assign dsp_reg_rd_en = dsp_reg_rd_en_r;
    assign dsp_reg_wr_en = dsp_reg_wr_en_r;

    // =========================================================================
    // SCC Z8530 instances
    // =========================================================================

    // SCC0 (DUART0) - keyboard/mouse
    wire [63:0] scc0_rdata;
    wire        scc0_txd_a;

    scc_z8530 #(
        .INST_NAME("SCC0"),
        .CLK_HZ(CLK_HZ),
        .BRG_CLOCK_HZ(3672000),
        .TX_BAUD(9600),
        .ADVERTISE_TX_FIFO(1)
    ) scc0_inst (
        .clk     (clk),
        .rst_n   (rst_n),
        .addr    (addr[3:0]),
        .wdata   (wdata),
        .rdata   (scc0_rdata),
        .wmask   (wmask),
        .valid   (sel_scc0 && hpc1_accept),
        .write   (write),
        .ready   (scc0_ready),
        .txd_a   (scc0_txd_a),
        .txd_b   (),
        .rxd_a   (scc_rxd_a),
        .rxd_b   (1'b1),
        .rx_inject_data  (mouse_rx_data),
        .rx_inject_valid (mouse_rx_valid),
        .rx_inject_data_a  (kbd_rx_data),
        .rx_inject_valid_a (kbd_rx_valid),
        .tx_byte_out_a   (kbd_tx_data),
        .tx_byte_valid_a (kbd_tx_valid),
        .tx_byte_out_b   (mouse_tx_data),
        .tx_byte_valid_b (mouse_tx_valid),
        .access_event_data(scc0_access_event_data),
        .access_event_valid(scc0_access_event_valid),
        .debug_probe     (),
        .scc_irq         (scc0_irq)
    );

    // SCC1 (DUART1) - console serial
    wire [63:0] scc1_rdata;

    scc_z8530 #(
        .INST_NAME("SCC1"),
        .CLK_HZ(CLK_HZ),
        .BRG_CLOCK_HZ(3672000),
        .FIXED_TX_BAUD(0),
        .TX_BAUD(9600),
        .ADVERTISE_TX_FIFO(1)
    ) scc1_inst (
        .clk     (clk),
        .rst_n   (rst_n),
        .addr    (addr[3:0]),
        .wdata   (wdata),
        .rdata   (scc1_rdata),
        .wmask   (wmask),
        .valid   (sel_scc1 && hpc1_accept),
        .write   (write),
        .ready   (scc1_ready),
        .txd_a   (),          // Channel A not used for console
        .txd_b   (scc_txd_b), // Channel B = console TX
        .rxd_a   (1'b1),
        .rxd_b   (scc_rxd_b),
        .rx_inject_data  (rx_inject_data),
        .rx_inject_valid (rx_inject_valid),
        .rx_inject_data_a  (8'h0),
        .rx_inject_valid_a (1'b0),
        .tx_byte_out_a   (),
        .tx_byte_valid_a (),
        .tx_byte_out_b   (scc_console_tx_data),
        .tx_byte_valid_b (scc_console_tx_valid),
        .access_event_data(),
        .access_event_valid(),
        .debug_probe     (scc_debug_probe),
        .scc_irq         (scc1_irq)
    );

    // Channel A TX from DUART0. The board may leave this unconnected, but the
    // SCC model now drives the async bitstream instead of hard-idling it.
    assign scc_txd_a = scc0_txd_a;

    // =========================================================================
    // WD33C93 SCSI Controller
    // =========================================================================

    wire [7:0] wd93_rdata;
    wire [127:0] wd93_debug_probe;
    wire [7:0] wd93_reg_ptr = wd93_debug_probe[71:64];
    assign rdata = (wd93_req_valid && !wd93_req_write)
        ? {8{wd93_rdata}}
        : rdata_r;

    wd33c93 #(
        .SCSI_EXTERNAL_TARGET(SCSI_EXTERNAL_TARGET)
    ) scsi0_inst (
        .clk              (clk),
        .rst_n            (rst_n),
        .sel_addr         (wd93_sel_addr),
        .wdata            (wd93_req_wdata),
        .rdata            (wd93_rdata),
        .valid            (wd93_req_valid),
        .write            (wd93_req_write),
        .hpc_reset        (scsi_ctrl[0]),
        .dma_window_done  (dma_clear_startdma),
        .target_present   (scsi_target_present),
        .scsi_cmd_pending (scsi_cmd_pending),
        .scsi_cmd_done    (scsi_cmd_done),
        .scsi_cmd_status  (scsi_cmd_status),
        .scsi_resp_len    (scsi_resp_len),
        .scsi_status_valid(scsi_status_valid),
        .scsi_target_id   (scsi_target_id),
        .scsi_cdb         (scsi_cdb),
        .scsi_xfer_count  (scsi_xfer_count),
        .scsi_data_index  (scsi_data_index),
        .scsi_read_data   (scsi_read_data),
        .scsi_read_valid  (scsi_read_valid),
        .scsi_read_consume(scsi_read_consume),
        .scsi_write_valid (scsi_write_valid),
        .scsi_write_data  (scsi_write_data),
        .scsi_write_ready (scsi_write_ready),
        .scsi_data_out_done(scsi_data_out_done),
        .drq              (scsi_drq),
        .dack             (scsi_dack),
        .drq_data_out     (scsi_drq_byte),
        .dack_data_in     (scsi_dack_byte),
        .drq_to_mem       (scsi_drq_to_mem),
        .irq              (scsi0_irq),
        .debug_probe      (wd93_debug_probe)
    );

    // =========================================================================
    // SCSI DMA Engine
    // =========================================================================

    // DRQ/DACK wiring between WD33C93 and DMA engine
    wire        scsi_drq;
    wire        scsi_dack;
    wire [7:0]  scsi_drq_byte;
    wire [7:0]  scsi_dack_byte;
    wire        scsi_drq_to_mem;
    wire        dma_clear_startdma;

    wire [31:0] scsi_dma_mem_addr;
    wire [63:0] scsi_dma_mem_wdata;
    wire [ 7:0] scsi_dma_mem_wmask;
    wire        scsi_dma_mem_valid;
    wire        scsi_dma_mem_write;
    wire        scsi_dma_mem_ready;
    wire        scsi_dma_active;

    wire [31:0] enet_dma_mem_addr;
    wire [63:0] enet_dma_mem_wdata;
    wire [ 7:0] enet_dma_mem_wmask;
    wire        enet_dma_mem_valid;
    wire        enet_dma_mem_write;
    wire        enet_dma_mem_ready;
    wire        enet_dma_active;
    wire [63:0] enet_pio_rdata;
    wire        enet_pio_valid = hpc1_accept &&
                                (sel_enet || sel_hpc_endian || sel_seeq);

    hpc1_scsi_dma scsi_dma_inst (
        .clk              (clk),
        .rst_n            (rst_n),
        .nbdp             (scsi_nbdp),
        .startdma         (scsi_ctrl[7]),
        .flushdma         (scsi_ctrl[1]),
        .to_mem           (scsi_ctrl[4]),
        .clear_startdma   (dma_clear_startdma),
        .drq              (scsi_drq),
        .dack             (scsi_dack),
        .drq_byte         (scsi_drq_byte),
        .dack_byte        (scsi_dack_byte),
        .dma_active       (scsi_dma_active),
        .dma_addr         (scsi_dma_mem_addr),
        .dma_wdata        (scsi_dma_mem_wdata),
        .dma_rdata        (dma_mem_rdata),
        .dma_wmask        (scsi_dma_mem_wmask),
        .dma_valid        (scsi_dma_mem_valid),
        .dma_write        (scsi_dma_mem_write),
        .dma_ready        (scsi_dma_mem_ready)
    );

    hpc1_enet #(
        .CLK_HZ(CLK_HZ)
    ) enet_inst (
        .clk                (clk),
        .rst_n              (rst_n),
        .pio_addr           (addr),
        .pio_wdata          (wdata),
        .pio_rdata          (enet_pio_rdata),
        .pio_wmask          (wmask),
        .pio_valid          (enet_pio_valid),
        .pio_write          (write),
        .pio_sel_enet       (sel_enet),
        .pio_sel_hpc_endian (sel_hpc_endian),
        .pio_sel_seeq       (sel_seeq),
        .dma_addr           (enet_dma_mem_addr),
        .dma_wdata          (enet_dma_mem_wdata),
        .dma_rdata          (dma_mem_rdata),
        .dma_wmask          (enet_dma_mem_wmask),
        .dma_valid          (enet_dma_mem_valid),
        .dma_write          (enet_dma_mem_write),
        .dma_ready          (enet_dma_mem_ready),
        .dma_active         (enet_dma_active),
        .tx_data            (enet_tx_data),
        .tx_valid           (enet_tx_valid),
        .tx_sof             (enet_tx_sof),
        .tx_eof             (enet_tx_eof),
        .tx_ready           (enet_tx_ready),
        .rx_data            (enet_rx_data),
        .rx_valid           (enet_rx_valid),
        .rx_sof             (enet_rx_sof),
        .rx_eof             (enet_rx_eof),
        .rx_ready           (enet_rx_ready),
        .enet_irq           (enet_irq),
        .debug_probe        (enet_debug_probe),
        .debug_snapshot     (enet_debug_snapshot)
    );

    // The memory port returns completion after a potentially long DDR wait.
    // Keep ownership locked across that wait: a combinational SCSI-priority
    // mux can otherwise route an outstanding Ethernet response into a newly
    // asserted SCSI descriptor/payload request.
    hpc1_dma_arbiter #(
        .CLK_HZ(CLK_HZ)
    ) dma_arbiter_inst (
        .clk        (clk),
        .rst_n      (rst_n),
        .scsi_addr  (scsi_dma_mem_addr),
        .scsi_wdata (scsi_dma_mem_wdata),
        .scsi_wmask (scsi_dma_mem_wmask),
        .scsi_valid (scsi_dma_mem_valid),
        .scsi_write (scsi_dma_mem_write),
        .scsi_ready (scsi_dma_mem_ready),
        .enet_addr  (enet_dma_mem_addr),
        .enet_wdata (enet_dma_mem_wdata),
        .enet_wmask (enet_dma_mem_wmask),
        .enet_valid (enet_dma_mem_valid),
        .enet_write (enet_dma_mem_write),
        .enet_ready (enet_dma_mem_ready),
        .dsp_addr   (dsp_dma_mem_addr),
        .dsp_wdata  (dsp_dma_mem_wdata),
        .dsp_wmask  (dsp_dma_mem_wmask),
        .dsp_valid  (dsp_dma_mem_valid),
        .dsp_write  (dsp_dma_mem_write),
        .dsp_ready  (dsp_dma_mem_ready),
        .mem_addr   (dma_mem_addr),
        .mem_wdata  (dma_mem_wdata),
        .mem_wmask  (dma_mem_wmask),
        .mem_valid  (dma_mem_valid),
        .mem_write  (dma_mem_write),
        .mem_ready  (dma_mem_ready)
    );
    assign dma_active = scsi_dma_active | enet_dma_active | dsp_dma_active;

    wire [31:0] scsi_debug_flags = {
        effective_lio_0_mask,
        scsi_dma_mem_wmask,
        lio_0_isr,
        scsi0_irq,
        scsi_drq,
        scsi_dack,
        scsi_drq_to_mem,
        scsi_dma_mem_valid,
        scsi_dma_mem_write,
        scsi_dma_mem_ready,
        scsi_dma_active
    };

    // Persistent WD93 PIO flight recorder. The interrupt handler reads
    // LUN/PHASE and SRCID/STAT as two auto-incrementing pairs.  A data-port
    // read that reaches CMD (register 24) without an explicit selector write
    // for CMD means the STAT read skipped a register.  Preserve the eight
    // PIO operations leading to that event across SCSI bus resets so the VIO
    // probe can distinguish a duplicated strobe from a bad status update.
    reg [127:0] wdpio_history;
    reg [127:0] wdpio_anomaly_history;
    reg [15:0]  wdpio_event_count;
    reg [7:0]   wdpio_data_read_count;
    reg [7:0]   wdpio_anomaly_count;
    reg         wdpio_explicit_cmd_select;
    reg         wdpio_anomaly_seen;
    wire [1:0] wdpio_event_type =
        wd93_req_sel_addr ? (wd93_req_write ? 2'b01 : 2'b00) :
                            (wd93_req_write ? 2'b11 : 2'b10);
    wire [7:0] wdpio_event_value = wd93_req_write ? wd93_req_wdata : wd93_rdata;
    wire [15:0] wdpio_event = {
        wdpio_event_type, wd93_reg_ptr[5:0], wdpio_event_value
    };
    wire wdpio_unselected_cmd_read = wd93_req_valid &&
        !wd93_req_sel_addr && !wd93_req_write &&
        (wd93_reg_ptr == 8'd24) && !wdpio_explicit_cmd_select;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wdpio_history <= 128'd0;
            wdpio_anomaly_history <= 128'd0;
            wdpio_event_count <= 16'd0;
            wdpio_data_read_count <= 8'd0;
            wdpio_anomaly_count <= 8'd0;
            wdpio_explicit_cmd_select <= 1'b0;
            wdpio_anomaly_seen <= 1'b0;
        end else if (wd93_req_valid) begin
            wdpio_history <= {wdpio_history[111:0], wdpio_event};
            wdpio_event_count <= wdpio_event_count + 16'd1;
            if (!wd93_req_sel_addr && !wd93_req_write)
                wdpio_data_read_count <= wdpio_data_read_count + 8'd1;

            if (wd93_req_sel_addr && wd93_req_write)
                wdpio_explicit_cmd_select <= (wd93_req_wdata == 8'd24);
            else if (!wd93_req_sel_addr)
                wdpio_explicit_cmd_select <= 1'b0;

            if (wdpio_unselected_cmd_read) begin
                wdpio_anomaly_seen <= 1'b1;
                wdpio_anomaly_count <= wdpio_anomaly_count + 8'd1;
                wdpio_anomaly_history <= {wdpio_history[111:0], wdpio_event};
            end
        end
    end

    assign scsi_debug_probe = wdpio_anomaly_seen ? {
        32'h5750_494f, // "WPIO"
        wdpio_anomaly_count,
        wdpio_data_read_count,
        wdpio_event_count,
        wdpio_anomaly_history,
        wd93_debug_probe[95:64],
        wd93_debug_probe[63:32]
    } : {
        32'h5343_5349, // "SCSI"
        scsi_bc,
        scsi_dma_mem_addr,
        scsi_debug_flags,
        wd93_debug_probe
    };

    // =========================================================================
    // Bus response logic
    // =========================================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cpu_aux_control <= 8'h00;
            pbus_bc <= 32'h0;
            pbus_cbp <= 32'h0;
            pbus_nbdp <= 32'h0;
            pbus_ctrl <= 32'h0;
            pbus_pntr <= 32'h0;
            pbus_fifo <= 32'h0;
            // SCSI DMA registers
            scsi_bc   <= 32'h0;
            scsi_cbp  <= 32'h0;
            scsi_nbdp <= 32'h0;
            scsi_ctrl <= 32'h0;
            scsi_pntr <= 32'h0;
            scsi_fifo <= 32'h0;
            // INT2 registers
            lio_0_mask <= 8'h00;
            lio_1_mask <= 8'h00;
            vme_0_mask <= 8'h00;
            vme_1_mask <= 8'h00;
            int2_port_config <= 8'h00;
            gfx_retrace_pending <= 1'b0;
            gfx_retrace_raw_count <= 24'd0;
            gfx_retrace_armed_raw_count <= 24'd0;
            gfx_retrace_clear_write_count <= 24'd0;
            gfx_retrace_arm_write_count <= 24'd0;
            scsi0_irq_latched <= 1'b0;
            // 8254 timer registers
            pt_counter_0 <= 16'hFFFF;
            pt_counter_1 <= 16'hFFFF;
            pt_counter_2 <= 16'hFFFF;
            pt_latch_0 <= 16'h0;
            pt_latch_1 <= 16'h0;
            pt_latch_2 <= 16'h0;
            pt_reload_0 <= 16'hFFFF;
            pt_reload_1 <= 16'hFFFF;
            pt_reload_2 <= 16'hFFFF;
            pt_control <= 8'h0;
            pt_rw_mode <= 3'b0;
            pt_byte_sel <= 3'b0;
            pt_latched <= 3'b0;
            pt_count_enabled <= 3'b000;  // Disabled until programmed
            timer_irq_0 <= 1'b0;
            timer_irq_1 <= 1'b0;
            pt_prescale <= 32'd0;
            pt_mode_0 <= 3'd0;
            pt_mode_1 <= 3'd0;
            pt_mode_2 <= 3'd0;
            // IP20 RTC registers - initialize with sensible defaults
            // Reg 0: MSR (Main Status Register)
            // Reg 1: RTMR (Real-Time Mode Register) - CSS bit 3
            // Reg 2: OMR (Output Mode Register)
            // Reg 3: Periodic Flag / ICR0
            // Reg 4: Time Save Control / ICR1
            // Reg 5: hundredths (BCD)
            // Reg 6: seconds (BCD)
            // Reg 7: minutes (BCD)
            // Reg 8: hours (BCD)
            // Reg 9: day of month (BCD)
            // Reg 10: month (BCD)
            // Reg 11: year (BCD)
            // Reg 14: day of week
            rtc_regs[1] <= 8'h08;  // RTMR: CSS=1 (clock running by default)
            rtc_regs[0] <= 8'h00;  // MSR: register set 0, RAM page deselected
            rtc_page_ram[1] <= 8'h00;
            rtc_page_ram[2] <= 8'h00;
            // Initialize time registers with valid date (battery-backed in real HW)
            rtc_regs[5]  <= 8'h00;  // hundredths = 00
            rtc_regs[6]  <= 8'h00;  // seconds = 00
            rtc_regs[7]  <= 8'h00;  // minutes = 00
            rtc_regs[8]  <= 8'h12;  // hours = 12 (noon)
            rtc_regs[9]  <= 8'h15;  // day = 15
            rtc_regs[10] <= 8'h03;  // month = 03 (March)
            rtc_regs[11] <= 8'h56;  // year = 56 (1970+56=2026)
            rtc_regs[14] <= 8'h06;  // day of week = Saturday
            rtc_prescaler <= 14'd0;
            rtc_te_prev <= 1'b0;
            // Bank 1: RTC_RUN active from reset (bits [4:3] = 0x18)
            rtc_bank1[0] <= 8'h18;  // ck_rtime1: RTC_RUN active
            rtc_bank1[1] <= 8'h00;  // ck_outmode1
            rtc_bank1[2] <= 8'h00;  // ck_int0ctl1
            rtc_bank1[3] <= 8'h00;  // ck_int1ctl1
            rdata_r <= 64'h0;
            ready_r <= 1'b0;
            accepted_addr <= 12'd0;
            accepted_write <= 1'b0;
            accepted_wdata <= 64'd0;
            accepted_wmask <= 8'd0;
            dsp_reg_wdata_r <= 24'h0;
            dsp_reg_rd_en_r <= 1'b0;
            dsp_reg_wr_en_r <= 1'b0;
            wd93_req_valid <= 1'b0;

            wd93_req_sel_addr <= 1'b0;
            wd93_req_write <= 1'b0;
            wd93_req_wdata <= 8'h00;
        end else begin
            // SGI IP20.h defines PCON_CLEARVRI as active low for clearing and
            // disabling the vertical-retrace latch.  Once armed high, retain
            // a raw one-clock VC1 event until software pulses the bit low.
            if (!int2_port_config[3])
                gfx_retrace_pending <= 1'b0;
            else if (gfx_retrace_irq)
                gfx_retrace_pending <= 1'b1;
            if (gfx_retrace_irq) begin
                gfx_retrace_raw_count <= gfx_retrace_raw_count + 1'b1;
                if (int2_port_config[3])
                    gfx_retrace_armed_raw_count <=
                        gfx_retrace_armed_raw_count + 1'b1;
            end

            // Ready is a one-cycle response-valid pulse. If the CPU advances
            // to another request without dropping valid, the combinational
            // ready output is masked by accepted_request_changed until this
            // register drops.
            ready_r <= 1'b0;
            dsp_reg_rd_en_r <= 1'b0;
            dsp_reg_wr_en_r <= 1'b0;
            wd93_req_valid <= 1'b0;

            // The DSP and CPU share one physical IP20 RTC.  DSP writes use the
            // low byte of its 24-bit Y-space bus.  CPU writes below have
            // priority only in the vanishingly rare same-cycle collision.
            if (HAS_DSP && dsp_rtc_wr_en) begin
                if (dsp_rtc_addr != 5'd0 && rtc_ram_page)
                    rtc_page_ram[dsp_rtc_addr] <= dsp_rtc_wdata[7:0];
                else if (dsp_rtc_addr >= 5'd1 && dsp_rtc_addr <= 5'd4 &&
                         rtc_regs[0][6])
                    rtc_bank1[dsp_rtc_addr - 5'd1] <= dsp_rtc_wdata[7:0];
                else
                    rtc_regs[dsp_rtc_addr] <= dsp_rtc_wdata[7:0];
            end

            // 8254 timer cascade:
            //   CPU clock -> 1 MHz prescaler -> counter 2 -> counters 0/1.
            // Software programs counter 2 to 200 and counter 0 to 50, yielding
            // a 100 Hz scheduling interrupt instead of a CPU-rate storm.
            if (!(valid && write && sel_pt_clock)) begin
                if (pt_input_tick) begin
                    pt_prescale <= 32'd0;
                    if (pt_count_enabled[2] && pt_counter_2 != 16'h0) begin
                        if (pt_mode_2[1]) begin
                            if (pt_counter_2 == 16'h1)
                                pt_counter_2 <= pt_reload_2;
                            else
                                pt_counter_2 <= pt_counter_2 - 1'b1;
                        end else begin
                            pt_counter_2 <= pt_counter_2 - 1'b1;
                        end
                    end

                    if (pt_count_enabled[2] && pt_counter_2 == 16'h1) begin
                        // Counter 2 terminal count is the clock edge for 0/1.
                        if (pt_count_enabled[0] && pt_counter_0 != 16'h0) begin
                            if (pt_mode_0[1]) begin
                                if (pt_counter_0 == 16'h1) begin
                                    pt_counter_0 <= pt_reload_0;
                                    timer_irq_0 <= 1'b1;
                                end else
                                    pt_counter_0 <= pt_counter_0 - 1'b1;
                            end else begin
                                pt_counter_0 <= pt_counter_0 - 1'b1;
                                if (pt_counter_0 == 16'h1)
                                    timer_irq_0 <= 1'b1;
                            end
                        end
                        if (pt_count_enabled[1] && pt_counter_1 != 16'h0) begin
                            if (pt_mode_1[1]) begin
                                if (pt_counter_1 == 16'h1) begin
                                    pt_counter_1 <= pt_reload_1;
                                    timer_irq_1 <= 1'b1;
                                end else
                                    pt_counter_1 <= pt_counter_1 - 1'b1;
                            end else begin
                                pt_counter_1 <= pt_counter_1 - 1'b1;
                                if (pt_counter_1 == 16'h1)
                                    timer_irq_1 <= 1'b1;
                            end
                        end
                    end
                end else
                    pt_prescale <= pt_prescale + 32'd1;
            end

            // (timer state debug removed)

            // SCSI DMA engine auto-clears STARTDMA and byte count on completion
            // Real HPC1 decrements scsi_bc during DMA. When done, bc=0.
            // PROM checks scsi_bc after DMA to verify transfer completed.
            if (dma_clear_startdma) begin
                scsi_ctrl[7] <= 1'b0;
                scsi_bc <= 32'h0;
            end

            // RTC time tick (IP20 layout)
            // Gated by RTC_RUN (bank1[0] bits [4:3] both set). PROM SCSI
            // timeouts use get_tod(), so this must track wall time instead of
            // an accelerated diagnostic clock.
            if (rtc_bank1[0][4] && rtc_bank1[0][3]) begin
                if (rtc_prescaler >= RTC_TICK_MAX) begin
                    rtc_prescaler <= 32'd0;
                    // BCD increment hundredths (0x00-0x99)
                    if (rtc_regs[5] >= 8'h99) begin
                        rtc_regs[5] <= 8'h00;
                        // Increment seconds (BCD 0-59)
                        if (rtc_regs[6] == 8'h59) begin
                            rtc_regs[6] <= 8'h00;
                            // Increment minutes (BCD 0-59)
                            if (rtc_regs[7] == 8'h59) begin
                                rtc_regs[7] <= 8'h00;
                                // Increment hours (BCD 0-23)
                                if (rtc_regs[8] == 8'h23) begin
                                    rtc_regs[8] <= 8'h00;
                                    // Midnight: increment day-of-week (1-7)
                                    if (rtc_regs[14] >= 8'h07)
                                        rtc_regs[14] <= 8'h01;
                                    else
                                        rtc_regs[14] <= rtc_regs[14] + 8'd1;
                                    // Increment day-of-month (BCD, month-dependent max)
                                    begin : rtc_day_carry
                                        reg [7:0] max_day;
                                        case (rtc_regs[10])
                                            8'h01, 8'h03, 8'h05, 8'h07,
                                            8'h08, 8'h10, 8'h12: max_day = 8'h31;
                                            8'h04, 8'h06, 8'h09, 8'h11: max_day = 8'h30;
                                            8'h02: max_day = 8'h28;
                                            default: max_day = 8'h31;
                                        endcase
                                        if (rtc_regs[9] >= max_day) begin
                                            rtc_regs[9] <= 8'h01;
                                            // Increment month (BCD 1-12)
                                            if (rtc_regs[10] == 8'h12) begin
                                                rtc_regs[10] <= 8'h01;
                                                // Increment year (BCD 0-99)
                                                if (rtc_regs[11] == 8'h99)
                                                    rtc_regs[11] <= 8'h00;
                                                else if (rtc_regs[11][3:0] == 4'd9)
                                                    rtc_regs[11] <= {rtc_regs[11][7:4] + 4'd1, 4'd0};
                                                else
                                                    rtc_regs[11] <= rtc_regs[11] + 8'd1;
                                            end else if (rtc_regs[10][3:0] == 4'd9)
                                                rtc_regs[10] <= {rtc_regs[10][7:4] + 4'd1, 4'd0};
                                            else
                                                rtc_regs[10] <= rtc_regs[10] + 8'd1;
                                        end else if (rtc_regs[9][3:0] == 4'd9)
                                            rtc_regs[9] <= {rtc_regs[9][7:4] + 4'd1, 4'd0};
                                        else
                                            rtc_regs[9] <= rtc_regs[9] + 8'd1;
                                    end
                                end else if (rtc_regs[8][3:0] == 4'd9)
                                    rtc_regs[8] <= {rtc_regs[8][7:4] + 4'd1, 4'd0};
                                else
                                    rtc_regs[8] <= rtc_regs[8] + 8'd1;
                            end else if (rtc_regs[7][3:0] == 4'd9)
                                rtc_regs[7] <= {rtc_regs[7][7:4] + 4'd1, 4'd0};
                            else
                                rtc_regs[7] <= rtc_regs[7] + 8'd1;
                        end else if (rtc_regs[6][3:0] == 4'd9)
                            rtc_regs[6] <= {rtc_regs[6][7:4] + 4'd1, 4'd0};
                        else
                            rtc_regs[6] <= rtc_regs[6] + 8'd1;
                    end else if (rtc_regs[5][3:0] == 4'd9)
                        rtc_regs[5] <= {rtc_regs[5][7:4] + 4'd1, 4'd0};
                    else
                        rtc_regs[5] <= rtc_regs[5] + 8'd1;
                end else
                    rtc_prescaler <= rtc_prescaler + 14'd1;
            end

            if (LATCH_SCSI_IRQ_STATUS != 0) begin
                if (scsi_ctrl[0])
                    scsi0_irq_latched <= 1'b0;
                else if (scsi0_irq)
                    scsi0_irq_latched <= 1'b1;
                else if (hpc1_accept && !write && sel_lio_0_isr)
                    scsi0_irq_latched <= 1'b0;
            end else begin
                scsi0_irq_latched <= 1'b0;
            end

            if (hpc1_accept) begin
                // Ordinary HPC1 registers complete from ready_r.  A WD93
                // request completes through wd93_req_valid on its replay edge.
                ready_r <= !wd93_accept;
                accepted_addr <= addr;
                accepted_write <= write;
                accepted_wdata <= wdata;
                accepted_wmask <= wmask;

                if (wd93_accept) begin
                    wd93_req_valid <= 1'b1;
                    wd93_req_sel_addr <= sel_scsi_addr;
                    wd93_req_write <= write;
                    wd93_req_wdata <= wdata_byte;
                end

                if (write) begin
                    // Write operations
                    if (sel_dsp_reg) begin
                        // DSP register write: extract 24-bit data from 32-bit word
                        // addr[2]=0: upper 32 bits, addr[2]=1: lower 32 bits
                        dsp_reg_wdata_r <= addr[2] ? wdata[23:0] : wdata[55:32];
                        dsp_reg_wr_en_r <= 1'b1;
                    end
                    else if (sel_aux_control) begin
                        // CPU_AUX_CONTROL is a byte register repeated across
                        // the 0x1B8..0x1BF window.
                        if (byte_write)
                            cpu_aux_control <= wdata_byte;
                        // $display("HPC1: AUX_CONTROL write = %02x", wdata_byte);
                    end
                    else if (sel_enet || sel_hpc_endian || sel_seeq) begin
                        // Register writes are handled by hpc1_enet via enet_pio_valid.
                    end
                    else if (sel_scsi_wd) begin
                        // WD33C93 writes handled by scsi0_inst (via valid gating)
                    end
                    else if (sel_scsi_dma) begin
                        // SCSI DMA registers (32-bit, use addr[4:2] to select)
                        // addr[2]=0: upper half (wmask[7:4], wdata[63:32])
                        // addr[2]=1: lower half (wmask[3:0], wdata[31:0])
                        case (addr[4:2])
                            3'd2: begin // 0x088: scsi_bc (addr[2]=0)
                                if (wmask[7]) scsi_bc[31:24] <= wdata[63:56];
                                if (wmask[6]) scsi_bc[23:16] <= wdata[55:48];
                                if (wmask[5]) scsi_bc[15:8]  <= wdata[47:40];
                                if (wmask[4]) scsi_bc[7:0]   <= wdata[39:32];
                            end
                            3'd3: begin // 0x08C: scsi_cbp (addr[2]=1)
                                if (wmask[3]) scsi_cbp[31:24] <= wdata[31:24];
                                if (wmask[2]) scsi_cbp[23:16] <= wdata[23:16];
                                if (wmask[1]) scsi_cbp[15:8]  <= wdata[15:8];
                                if (wmask[0]) scsi_cbp[7:0]   <= wdata[7:0];
                            end
                            3'd4: begin // 0x090: scsi_nbdp (addr[2]=0)
                                if (wmask[7]) scsi_nbdp[31:24] <= wdata[63:56];
                                if (wmask[6]) scsi_nbdp[23:16] <= wdata[55:48];
                                if (wmask[5]) scsi_nbdp[15:8]  <= wdata[47:40];
                                if (wmask[4]) scsi_nbdp[7:0]   <= wdata[39:32];
                            end
                            3'd5: begin // 0x094: scsi_ctrl (addr[2]=1)
                                if (wmask[3]) scsi_ctrl[31:24] <= wdata[31:24];
                                if (wmask[2]) scsi_ctrl[23:16] <= wdata[23:16];
                                if (wmask[1]) scsi_ctrl[15:8]  <= wdata[15:8];
                                if (wmask[0]) scsi_ctrl[7:0]   <= wdata[7:0];
                                // SCSI_FLUSH (bit 1) is consumed by the DMA
                                // engine.  It clears STARTDMA only after all
                                // buffered SCSI->memory bytes are committed.
`ifndef SYNTHESIS
                                if (wdata[7]) // Only log STARTDMA
                                    $display("HPC1: DMA_START dir=%b nbdp=0x%08x",
                                        wdata[4], scsi_nbdp);
`endif
                            end
                            3'd6: begin // 0x098: scsi_pntr (addr[2]=0, upper half)
                                if (wmask[7]) scsi_pntr[31:24] <= wdata[63:56];
                                if (wmask[6]) scsi_pntr[23:16] <= wdata[55:48];
                                if (wmask[5]) scsi_pntr[15:8]  <= wdata[47:40];
                                if (wmask[4]) scsi_pntr[7:0]   <= wdata[39:32];
                                // Latch flags from bits [31:28] = bus [63:60]
                                scsi_fifo_flag_latch <= wdata[63:60];
                            end
                            3'd7: begin // 0x09C: scsi_fifo (addr[2]=1)
                                if (wmask[3]) scsi_fifo[31:24] <= wdata[31:24];
                                if (wmask[2]) scsi_fifo[23:16] <= wdata[23:16];
                                if (wmask[1]) scsi_fifo[15:8]  <= wdata[15:8];
                                if (wmask[0]) scsi_fifo[7:0]   <= wdata[7:0];
                                // Commit data + latched flags to FIFO RAM
                                begin : scsi_data_wr
                                    reg [3:0] sidx;
                                    sidx = scsi_pntr[5:2];
                                    scsi_fifo_ram[sidx] <= {wdata[31:24], wdata[23:16], wdata[15:8], wdata[7:0]};
                                    scsi_fifo_flags[sidx] <= scsi_fifo_flag_latch;
                                end
                            end
                            default: ;
                        endcase
                    end
                    else if (sel_pbus) begin
                        // Parallel port registers 0xA8-0xBF
                        // addr[4:2] selects register within this range
                        case (addr[4:2])
                            3'd2: begin // 0x0A8: pbus_bc (addr[2]=0)
                                if (wmask[7]) pbus_bc[31:24] <= wdata[63:56];
                                if (wmask[6]) pbus_bc[23:16] <= wdata[55:48];
                                if (wmask[5]) pbus_bc[15:8]  <= wdata[47:40];
                                if (wmask[4]) pbus_bc[7:0]   <= wdata[39:32];
                            end
                            3'd3: begin // 0x0AC: pbus_cbp (addr[2]=1)
                                if (wmask[3]) pbus_cbp[31:24] <= wdata[31:24];
                                if (wmask[2]) pbus_cbp[23:16] <= wdata[23:16];
                                if (wmask[1]) pbus_cbp[15:8]  <= wdata[15:8];
                                if (wmask[0]) pbus_cbp[7:0]   <= wdata[7:0];
                            end
                            3'd4: begin // 0x0B0: pbus_nbdp (addr[2]=0)
                                if (wmask[7]) pbus_nbdp[31:24] <= wdata[63:56];
                                if (wmask[6]) pbus_nbdp[23:16] <= wdata[55:48];
                                if (wmask[5]) pbus_nbdp[15:8]  <= wdata[47:40];
                                if (wmask[4]) pbus_nbdp[7:0]   <= wdata[39:32];
                            end
                            3'd5: begin // 0x0B4: pbus_ctrl (addr[2]=1)
                                if (wmask[3]) pbus_ctrl[31:24] <= wdata[31:24];
                                if (wmask[2]) pbus_ctrl[23:16] <= wdata[23:16];
                                if (wmask[1]) pbus_ctrl[15:8]  <= wdata[15:8];
                                if (wmask[0]) pbus_ctrl[7:0]   <= wdata[7:0];
                            end
                            3'd6: begin // 0x0B8: pbus_pntr (addr[2]=0, upper half)
                                if (wmask[7]) pbus_pntr[31:24] <= wdata[63:56];
                                if (wmask[6]) pbus_pntr[23:16] <= wdata[55:48];
                                if (wmask[5]) pbus_pntr[15:8]  <= wdata[47:40];
                                if (wmask[4]) pbus_pntr[7:0]   <= wdata[39:32];
                                // Latch flags from bits [31:28] = bus [63:60]
                                pbus_fifo_flag_latch <= wdata[63:60];
                            end
                            3'd7: begin // 0x0BC: pbus_fifo (addr[2]=1)
                                // Commit data + latched flags to FIFO RAM
                                begin : pbus_fifo_data_wr
                                    reg [3:0] pidx;
                                    pidx = pbus_pntr[5:2];
                                    pbus_fifo_ram[pidx] <= {wdata[31:24], wdata[23:16], wdata[15:8], wdata[7:0]};
                                    pbus_fifo_flags[pidx] <= pbus_fifo_flag_latch;
                                end
                            end
                            default: ;
                        endcase
                    end
                    // INT2 writes (byte access, use wdata_byte helper)
                    else if (sel_lio_0_mask) begin
                        lio_0_mask <= wdata_byte;
`ifndef SYNTHESIS
                        if (trace_int2)
                            $display("INT2: LIO_0_MASK write = %02x", wdata_byte);
`endif
                    end
                    else if (sel_lio_1_mask) begin
                        lio_1_mask <= wdata_byte;
`ifndef SYNTHESIS
                        if (trace_int2)
                            $display("IP20_INT2_ACCESS write t=%0t addr=1fb80%03x reg=LIO1_MASK value=%02x mask7=%0d pending=%0d cpu_irq1=%0d",
                                     $time, addr, wdata_byte, wdata_byte[7],
                                     gfx_retrace_pending, cpu_irq[1]);
`endif
                    end
                    else if (sel_vme_0_mask) begin
                        vme_0_mask <= wdata_byte;
                        // $display("INT2: VME_0_MASK write = %02x", wdata_byte);
                    end
                    else if (sel_vme_1_mask) begin
                        vme_1_mask <= wdata_byte;
                        // $display("INT2: VME_1_MASK write = %02x", wdata_byte);
                    end
                    else if (sel_port_config) begin
                        int2_port_config <= wdata_byte;
                        if (!wdata_byte[3]) begin
                            gfx_retrace_pending <= 1'b0;
                            gfx_retrace_clear_write_count <=
                                gfx_retrace_clear_write_count + 1'b1;
                        end else begin
                            gfx_retrace_arm_write_count <=
                                gfx_retrace_arm_write_count + 1'b1;
                        end
`ifndef SYNTHESIS
                        if (trace_int2)
                            $display("IP20_INT2_ACCESS write t=%0t addr=1fb80%03x reg=PORT_CONFIG value=%02x clearvri=%0d pending_before=%0d mask7=%0d cpu_irq1=%0d",
                                     $time, addr, wdata_byte, wdata_byte[3],
                                     gfx_retrace_pending, lio_1_mask[7],
                                     cpu_irq[1]);
`endif
                    end
                    else if (sel_timer_ack) begin
                        // Timer acknowledge - write-only, clears timer IRQ bits
                        if (wdata_byte[0])
                            timer_irq_0 <= 1'b0;
                        if (wdata_byte[1])
                            timer_irq_1 <= 1'b0;
                    end
                    else if (sel_pt_clock) begin
                        // 8254 timer - decode register from address
                        // 0x1F0-0x1F3 = Counter 0, 0x1F4-0x1F7 = Counter 1
                        // 0x1F8-0x1FB = Counter 2, 0x1FC-0x1FF = Control
                        case (addr[3:2])
                            2'b00: begin // Counter 0 (0x1F0-0x1F3)
                                if (pt_byte_sel[0]) begin
                                    pt_counter_0[15:8] <= wdata_byte;
                                    pt_reload_0[15:8] <= wdata_byte;
                                    pt_byte_sel[0] <= 1'b0;
                                    pt_count_enabled[0] <= 1'b1;
                                    // $display("PT_CLOCK: CNT0 = 0x%02x%02x (%0d) ENABLED", wdata_byte, pt_counter_0[7:0], {wdata_byte, pt_counter_0[7:0]});
                                end else begin
                                    pt_counter_0[7:0] <= wdata_byte;
                                    pt_reload_0[7:0] <= wdata_byte;
                                    pt_byte_sel[0] <= 1'b1;
                                end
                            end
                            2'b01: begin // Counter 1 (0x1F4-0x1F7)
                                if (pt_byte_sel[1]) begin
                                    pt_counter_1[15:8] <= wdata_byte;
                                    pt_reload_1[15:8] <= wdata_byte;
                                    pt_byte_sel[1] <= 1'b0;
                                    pt_count_enabled[1] <= 1'b1;
                                    // $display("PT_CLOCK: CNT1 = 0x%02x%02x (%0d) ENABLED", wdata_byte, pt_counter_1[7:0], {wdata_byte, pt_counter_1[7:0]});
                                end else begin
                                    pt_counter_1[7:0] <= wdata_byte;
                                    pt_reload_1[7:0] <= wdata_byte;
                                    pt_byte_sel[1] <= 1'b1;
                                end
                            end
                            2'b10: begin // Counter 2 (0x1F8-0x1FB)
                                if (pt_byte_sel[2]) begin
                                    pt_counter_2[15:8] <= wdata_byte;
                                    pt_reload_2[15:8] <= wdata_byte;
                                    pt_byte_sel[2] <= 1'b0;
                                    pt_count_enabled[2] <= 1'b1;
                                    // $display("PT_CLOCK: CNT2 = 0x%02x%02x (%0d) ENABLED", wdata_byte, pt_counter_2[7:0], {wdata_byte, pt_counter_2[7:0]});
                                end else begin
                                    pt_counter_2[7:0] <= wdata_byte;
                                    pt_reload_2[7:0] <= wdata_byte;
                                    pt_byte_sel[2] <= 1'b1;
                                end
                            end
                            2'b11: begin // Control (0x1FC-0x1FF)
                                // Control word: [7:6]=counter, [5:4]=RW, [3:1]=mode, [0]=BCD
                                pt_control <= wdata_byte;
                                case (wdata_byte[7:6])
                                    2'b00: begin // Counter 0
                                        if (wdata_byte[5:4] == 2'b00) begin
                                            pt_latch_0 <= pt_counter_0;
                                            pt_latched[0] <= 1'b1;
                                        end else begin
                                            pt_rw_mode[0] <= wdata_byte[5];
                                            pt_mode_0 <= wdata_byte[3:1];
                                            pt_byte_sel[0] <= 1'b0;
                                            pt_count_enabled[0] <= 1'b0;
                                            timer_irq_0 <= 1'b0;
                                        end
                                    end
                                    2'b01: begin // Counter 1
                                        if (wdata_byte[5:4] == 2'b00) begin
                                            pt_latch_1 <= pt_counter_1;
                                            pt_latched[1] <= 1'b1;
                                        end else begin
                                            pt_rw_mode[1] <= wdata_byte[5];
                                            pt_mode_1 <= wdata_byte[3:1];
                                            pt_byte_sel[1] <= 1'b0;
                                            pt_count_enabled[1] <= 1'b0;
                                            timer_irq_1 <= 1'b0;
                                        end
                                    end
                                    2'b10: begin // Counter 2
                                        if (wdata_byte[5:4] == 2'b00) begin
                                            pt_latch_2 <= pt_counter_2;
                                            pt_latched[2] <= 1'b1;
                                        end else begin
                                            pt_rw_mode[2] <= wdata_byte[5];
                                            pt_mode_2 <= wdata_byte[3:1];
                                            pt_byte_sel[2] <= 1'b0;
                                            pt_count_enabled[2] <= 1'b0;
                                        end
                                    end
                                    default: ; // Read-back command - ignore for now
                                endcase
                                // $display("PT_CLOCK: CTRL write %02x cnt=%0d mode=%0d rw=%0d",
                                //     wdata_byte, wdata_byte[7:6], wdata_byte[3:1], wdata_byte[5:4]);
                            end
                        endcase
                    end
                    // SCC0 writes handled by scc0_inst (via valid gating)
                    // SCC2 stub: ignore writes
                    else if (sel_rtc) begin
                        // IP20 RTC write with register banking
                        if (rtc_idx != 5'd0 && rtc_ram_page) begin
                            rtc_page_ram[rtc_idx] <= rtc_wbyte;
                        end
                        // MSR (offset 0) and counters (offset 5+) always go to rtc_regs
                        // Offsets 1-4 are banked: RS=0 → rtc_regs, RS=1 → rtc_bank1
                        else if (rtc_idx >= 5'd1 && rtc_idx <= 5'd4 && rtc_regs[0][6]) begin
                            // Bank 1 write (RS=1): rtime, outmode, int0ctl, int1ctl
                            rtc_bank1[rtc_idx - 5'd1] <= rtc_wbyte;
                        end else begin
                            // Bank 0 write (RS=0) or non-banked registers
                            rtc_regs[rtc_idx] <= rtc_wbyte;
                            // Time Save: latch on TE falling edge (1→0 transition)
                            // TSCR is bank 0 offset 4; TE is bit 7
                            if (rtc_idx == 5'd4 && rtc_te_prev && !rtc_wbyte[7]) begin
                                rtc_regs[25] <= rtc_regs[6];   // seconds save
                                rtc_regs[26] <= rtc_regs[7];   // minutes save
                                rtc_regs[27] <= rtc_regs[8];   // hours save
                                rtc_regs[28] <= rtc_regs[9];   // day save
                                rtc_regs[29] <= rtc_regs[10];  // months save
                            end
                            if (rtc_idx == 5'd4)
                                rtc_te_prev <= rtc_wbyte[7];
                        end
                    end
                end else begin
                    // Read operations
                    if (sel_dsp_reg) begin
                        // DSP register read: return 24-bit data in appropriate 32-bit half
                        dsp_reg_rd_en_r <= 1'b1;
                        if (addr[2])
                            rdata_r <= {32'h0, 8'h0, dsp_reg_rdata};
                        else
                            rdata_r <= {8'h0, dsp_reg_rdata, 32'h0};
                    end
                    else if (sel_aux_control) begin
                        // Return AUX_CONTROL with EEPROM read bit
                        rdata_r <= {aux_control_read, aux_control_read,
                                  aux_control_read, aux_control_read,
                                  aux_control_read, aux_control_read,
                                  aux_control_read, aux_control_read};
                        // $display("HPC1: AUX_CONTROL read = %02x", aux_control_read);
                    end
                    else if (sel_scc0) begin
                        rdata_r <= scc0_rdata;
                    end
                    else if (sel_scc2) begin
                        // DUART2 stub: TX empty, no RX
                        if (!addr[2])
                            rdata_r <= {8{8'h04}};  // RR0: TX empty
                        else
                            rdata_r <= 64'h0;
                    end
                    else if (sel_scc1) begin
                        rdata_r <= scc1_rdata;
                    end
                    else if (sel_enet || sel_hpc_endian || sel_seeq) begin
                        rdata_r <= enet_pio_rdata;
                    end
                    else if (sel_scsi_wd) begin
                        // WD33C93 register read (byte replicated)
                        rdata_r <= {8{wd93_rdata}};
                    end
                    else if (sel_scsi_dma) begin
                        // SCSI DMA register read
                        case (addr[4:2])
                            3'd2: begin
                                rdata_r <= {scsi_bc, scsi_bc};
`ifndef SYNTHESIS
                                if (trace_hpc1)
                                    $display("HPC1_RD: scsi_bc=0x%08x", scsi_bc);
`endif
                            end
                            3'd3: begin
                                rdata_r <= {scsi_cbp, scsi_cbp};
`ifndef SYNTHESIS
                                if (trace_hpc1)
                                    $display("HPC1_RD: scsi_cbp=0x%08x", scsi_cbp);
`endif
                            end
                            3'd4: rdata_r <= {scsi_nbdp, scsi_nbdp};
                            3'd5: begin
                                rdata_r <= {scsi_ctrl, scsi_ctrl};
`ifndef SYNTHESIS
                                if (trace_hpc1)
                                    $display("HPC1_RD: scsi_ctrl=0x%08x", scsi_ctrl);
`endif
                            end
                            3'd6: begin // 0x098: scsi_pntr — return flags from FIFO
                                begin : scsi_ptr_rd
                                    reg [3:0] sidx;
                                    reg [31:0] val;
                                    sidx = scsi_pntr[5:2];
                                    val = scsi_pntr;
                                    val[31:28] = scsi_fifo_flags[sidx];
                                    rdata_r <= {val, val};
                                end
                            end
                            3'd7: begin // 0x09C: scsi_fifo — read from FIFO RAM
                                begin : scsi_data_rd
                                    reg [3:0] sidx;
                                    sidx = scsi_pntr[5:2];
                                    rdata_r <= {scsi_fifo_ram[sidx], scsi_fifo_ram[sidx]};
                                end
                            end
                            default: rdata_r <= 64'h0;
                        endcase
                    end
                    else if (sel_pbus) begin
                        case (addr[4:2])
                            3'd2: rdata_r <= {pbus_bc, pbus_bc};
                            3'd3: rdata_r <= {pbus_cbp, pbus_cbp};
                            3'd4: rdata_r <= {pbus_nbdp, pbus_nbdp};
                            3'd5: rdata_r <= {pbus_ctrl, pbus_ctrl};
                            3'd6: begin // 0x0B8: pbus_pntr — return flags from FIFO
                                begin : pbus_ptr_rd
                                    reg [3:0] pidx;
                                    reg [31:0] val;
                                    pidx = pbus_pntr[5:2];
                                    val = pbus_pntr;
                                    val[31:28] = pbus_fifo_flags[pidx];
                                    rdata_r <= {val, val};
                                end
                            end
                            3'd7: begin // 0x0BC: pbus_fifo — read from FIFO RAM
                                begin : pbus_data_rd
                                    reg [3:0] pidx;
                                    pidx = pbus_pntr[5:2];
                                    rdata_r <= {pbus_fifo_ram[pidx], pbus_fifo_ram[pidx]};
                                end
                            end
                            default: rdata_r <= 64'h0;
                        endcase
                    end
                    // INT2 reads (replicate byte across all lanes for big-endian access)
                    else if (sel_lio_0_isr) begin
                        rdata_r <= {8{lio_0_isr}};
                    end
                    else if (sel_lio_0_mask) begin
                        rdata_r <= {8{lio_0_mask}};
                    end
                    else if (sel_lio_1_isr) begin
                        rdata_r <= {8{lio_1_isr}};
`ifndef SYNTHESIS
                        if (trace_int2)
                            $display("IP20_INT2_ACCESS read t=%0t addr=1fb80%03x reg=LIO1_ISR value=%02x pending=%0d mask7=%0d cpu_irq1=%0d",
                                     $time, addr, lio_1_isr,
                                     gfx_retrace_pending, lio_1_mask[7],
                                     cpu_irq[1]);
`endif
                    end
                    else if (sel_lio_1_mask) begin
                        rdata_r <= {8{lio_1_mask}};
                    end
                    else if (sel_vme_isr) begin
                        rdata_r <= {8{vme_isr}};
                        // $display("INT2: VME_ISR read = %02x", vme_isr);
                    end
                    else if (sel_vme_0_mask) begin
                        rdata_r <= {8{vme_0_mask}};
                        // $display("INT2: VME_0_MASK read = %02x", vme_0_mask);
                    end
                    else if (sel_vme_1_mask) begin
                        rdata_r <= {8{vme_1_mask}};
                        // $display("INT2: VME_1_MASK read = %02x", vme_1_mask);
                    end
                    else if (sel_port_config) begin
                        rdata_r <= {8{int2_port_config}};
`ifndef SYNTHESIS
                        if (trace_int2)
                            $display("IP20_INT2_ACCESS read t=%0t addr=1fb80%03x reg=PORT_CONFIG value=%02x clearvri=%0d pending=%0d",
                                     $time, addr, int2_port_config,
                                     int2_port_config[3], gfx_retrace_pending);
`endif
                    end
                    else if (sel_pt_clock) begin
                        // 8254 timer read - decode register from address
                        case (addr[3:2])
                            2'b00: begin // Counter 0 (0x1F0-0x1F3)
                                if (pt_latched[0]) begin
                                    if (pt_byte_sel[0]) begin
                                        rdata_r <= {8{pt_latch_0[15:8]}};
                                        pt_byte_sel[0] <= 1'b0;
                                        pt_latched[0] <= 1'b0;
                                    end else begin
                                        rdata_r <= {8{pt_latch_0[7:0]}};
                                        pt_byte_sel[0] <= 1'b1;
                                    end
                                end else begin
                                    if (pt_byte_sel[0]) begin
                                        rdata_r <= {8{pt_counter_0[15:8]}};
                                        pt_byte_sel[0] <= 1'b0;
                                    end else begin
                                        rdata_r <= {8{pt_counter_0[7:0]}};
                                        pt_byte_sel[0] <= 1'b1;
                                    end
                                end
                                // $display("PT_CLOCK: CNT0 read -> %04x (enabled=%b)", pt_counter_0, pt_count_enabled[0]);
                            end
                            2'b01: begin // Counter 1 (0x1F4-0x1F7)
                                if (pt_latched[1]) begin
                                    if (pt_byte_sel[1]) begin
                                        rdata_r <= {8{pt_latch_1[15:8]}};
                                        pt_byte_sel[1] <= 1'b0;
                                        pt_latched[1] <= 1'b0;
                                    end else begin
                                        rdata_r <= {8{pt_latch_1[7:0]}};
                                        pt_byte_sel[1] <= 1'b1;
                                    end
                                end else begin
                                    if (pt_byte_sel[1]) begin
                                        rdata_r <= {8{pt_counter_1[15:8]}};
                                        pt_byte_sel[1] <= 1'b0;
                                    end else begin
                                        rdata_r <= {8{pt_counter_1[7:0]}};
                                        pt_byte_sel[1] <= 1'b1;
                                    end
                                end
                                // $display("PT_CLOCK: CNT1 read -> %04x (enabled=%b)", pt_counter_1, pt_count_enabled[1]);
                            end
                            2'b10: begin // Counter 2 (0x1F8-0x1FB)
                                if (pt_latched[2]) begin
                                    if (pt_byte_sel[2]) begin
                                        rdata_r <= {8{pt_latch_2[15:8]}};
                                        pt_byte_sel[2] <= 1'b0;
                                        pt_latched[2] <= 1'b0;
                                        // $display("PT_CLOCK: CNT2 READ hi=%02x (latched=%04x)", pt_latch_2[15:8], pt_latch_2);
                                    end else begin
                                        rdata_r <= {8{pt_latch_2[7:0]}};
                                        pt_byte_sel[2] <= 1'b1;
                                        // $display("PT_CLOCK: CNT2 READ lo=%02x (latched=%04x)", pt_latch_2[7:0], pt_latch_2);
                                    end
                                end else begin
                                    if (pt_byte_sel[2]) begin
                                        rdata_r <= {8{pt_counter_2[15:8]}};
                                        pt_byte_sel[2] <= 1'b0;
                                        // $display("PT_CLOCK: CNT2 READ hi=%02x (live=%04x)", pt_counter_2[15:8], pt_counter_2);
                                    end else begin
                                        rdata_r <= {8{pt_counter_2[7:0]}};
                                        pt_byte_sel[2] <= 1'b1;
                                        // $display("PT_CLOCK: CNT2 READ lo=%02x (live=%04x)", pt_counter_2[7:0], pt_counter_2);
                                    end
                                end
                            end
                            2'b11: begin // Control (0x1FC-0x1FF) - write-only
                                rdata_r <= 64'h0;
                            end
                        endcase
                    end
                    else if (sel_rtc) begin
                        // IP20 RTC read with register banking
                        if (rtc_idx != 5'd0 && rtc_ram_page)
                            rdata_r <= {8{rtc_page_ram[rtc_idx]}};
                        else if (rtc_idx >= 5'd1 && rtc_idx <= 5'd4 && rtc_regs[0][6])
                            rdata_r <= {8{rtc_bank1[rtc_idx - 5'd1]}};  // Bank 1 (RS=1)
                        else
                            rdata_r <= {8{rtc_regs[rtc_idx]}};           // Bank 0 or non-banked
                    end
                    else begin
                        // Default - return 0
                        rdata_r <= 64'h0;
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
