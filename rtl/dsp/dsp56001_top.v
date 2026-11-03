// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

//////////////////////////////////////////////////////////////////////////////
// DSP56001 Top-Level Module
//
// Integrates all components for the SGI Indigo IP20 DSP subsystem:
//   - DSP56001 Core (FSM, registers, decode, ALU, AGU)
//   - Internal Memory (P-RAM 512x24, X/Y RAM+ROM 256x24 each)
//   - X-Peripheral Space (SCI, SSI, BCR, IPR at X:$FFC0-$FFFF)
//   - HPC1 Host Interface (Y:$FFE0-$FFED, handshake registers)
//   - External SRAM (32K x 24-bit dual-port, at 0x1FBE0000)
//
// CPU-side interface connects to HPC1 for register access and SRAM.
// The wrapper runs in CYAN's 50 MHz machine domain.  A clock enable advances
// the core at the physical IP20 DSP56001 rate of 20 MHz; host, DMA, and SRAM
// logic remain in the machine domain and see one-cycle core request strobes.
//////////////////////////////////////////////////////////////////////////////

module dsp56001_top (
    input  wire        clk,              // 50 MHz CYAN machine clock
    input  wire        rst_n,            // Active-low reset (from system)

    // ========================================================================
    // CPU-Side HPC1 Register Interface
    // ========================================================================
    // Directly from HPC1 for DSP control registers (0x180-0x1B4)
    input  wire [4:0]  cpu_reg_addr,     // Register select
    input  wire [23:0] cpu_reg_wdata,    // Write data
    output wire [23:0] cpu_reg_rdata,    // Read data
    input  wire        cpu_reg_rd_en,
    input  wire        cpu_reg_wr_en,

    // ========================================================================
    // CPU-Side SRAM Interface (0x1FBE0000-0x1FBE7FFF)
    // ========================================================================
    input  wire [14:0] cpu_sram_addr,    // 32K word address
    input  wire [23:0] cpu_sram_wdata,
    output wire [23:0] cpu_sram_rdata,
    input  wire        cpu_sram_wr_en,
    input  wire        cpu_sram_rd_en,
    output wire        cpu_sram_ready,

    // ========================================================================
    // Interrupt Output (active-high, active when masked interrupt pending)
    // ========================================================================
    output wire        irq_out,          // To INT2 / CPU interrupt

    // ========================================================================
    // SSI Audio Interface (directly exposed)
    // ========================================================================
    output wire        ssi_sclk_out,
    output wire        ssi_fss_out,
    output wire [23:0] ssi_tx_data,
    output wire        ssi_tx_en,
    output wire [31:0] ssi_tx_channel_status,
    output wire        ssi_tx_channel_status_valid,
    output wire [31:0] ssi_tx_user_data,
    output wire        ssi_tx_user_valid,
    output wire        ssi_tx_active,
    output wire [5:0]  ssi_tx_rate_select,
    input  wire        ssi_tx_ready,
    input  wire        ssi_rx_valid,
    input  wire [23:0] ssi_rx_data,

    // ========================================================================
    // Shared DP8573 RTC Window (DSP Y:$FFC0-$FFDF)
    // ========================================================================
    output wire [4:0]  rtc_addr,
    output wire [23:0] rtc_wdata,
    output wire        rtc_rd_en,
    output wire        rtc_wr_en,
    input  wire [23:0] rtc_rdata,

    // ========================================================================
    // HPC1 Shared DMA Memory Port
    // ========================================================================
    output wire [31:0] dma_mem_addr,
    output wire [63:0] dma_mem_wdata,
    input  wire [63:0] dma_mem_rdata,
    output wire [7:0]  dma_mem_wmask,
    output wire        dma_mem_valid,
    output wire        dma_mem_write,
    input  wire        dma_mem_ready,
    output wire        dma_mem_active,

    // ========================================================================
    // Debug Interface
    // ========================================================================
    input  wire        dbg_mem_wr_en,
    input  wire [1:0]  dbg_mem_space,    // 0=P, 1=X, 2=Y
    input  wire [15:0] dbg_mem_addr,
    input  wire [23:0] dbg_mem_wdata,
    input  wire        dbg_reg_wr_en,
    input  wire [5:0]  dbg_reg_wr_addr,
    input  wire [55:0] dbg_reg_wr_data,
    output wire [15:0] dbg_pc,
    output wire [23:0] dbg_sr,
    output wire [3:0]  dbg_state,
    output wire        dbg_busy,
    output wire [23:0] dbg_cur_inst,
    output wire [55:0] dbg_acc_a,
    output wire [55:0] dbg_acc_b,
    output wire [255:0] debug_probe
);

    // ========================================================================
    // Internal Reset (DSP reset from MISCSR + system reset)
    // ========================================================================
    wire host_dsp_reset;
    wire dsp_rst_n = rst_n && !host_dsp_reset;
    wire dsp_run   = !host_dsp_reset;

    // ========================================================================
    // Core <-> Memory Wires
    // ========================================================================
    wire [15:0] core_p_fetch_addr;
    wire        core_p_fetch_en;
    wire [23:0] core_p_fetch_data;
    wire        core_p_fetch_ready;

    wire [15:0] core_x_addr;
    wire        core_x_rd_en;
    wire        core_x_wr_en;
    wire [23:0] core_x_wdata;
    wire [23:0] core_x_rdata;
    wire        core_x_ready;

    wire [15:0] core_y_addr;
    wire        core_y_rd_en;
    wire        core_y_wr_en;
    wire [23:0] core_y_wdata;
    wire [23:0] core_y_rdata;
    wire        core_y_ready;
    wire        core_data_rom_enable;

    wire [15:0] core_p_data_addr;
    wire        core_p_data_rd_en;
    wire        core_p_data_wr_en;
    wire [23:0] core_p_data_wdata;
    wire [23:0] core_p_data_rdata;
    wire        core_software_reset;
    wire        core_stop_reset;

    // ========================================================================
    // Memory <-> External SRAM Wires
    // ========================================================================
    wire [14:0] mem_ext_addr;
    wire [23:0] mem_ext_wdata;
    wire        mem_ext_rd_en;
    wire        mem_ext_wr_en;
    wire [1:0]  mem_ext_space;
    wire [23:0] mem_ext_rdata;
    wire        mem_ext_ready;

    // ========================================================================
    // Memory <-> X-Peripheral Wires
    // ========================================================================
    wire [5:0]  xperiph_addr;
    wire [23:0] xperiph_wdata;
    wire        xperiph_rd_en;
    wire        xperiph_wr_en;
    wire [23:0] xperiph_rdata;
    wire [23:0] xperiph_internal_rdata;
    wire [23:0] audio_glue_pbd_rdata;
    wire        audio_glue_irqb;
    wire        audio_glue_tx_clock_active;

    // ========================================================================
    // Memory <-> Y-Peripheral (HPC1 Host) Wires
    // ========================================================================
    wire [4:0]  yperiph_addr;
    wire [23:0] yperiph_wdata;
    wire        yperiph_rd_en;
    wire        yperiph_wr_en;
    wire [23:0] yperiph_rdata;

    // ========================================================================
    // Host Interface Control Wires
    // ========================================================================
    wire        host_force_irqa;
    wire        host_event_irqa;
    wire [15:0] irq_priority;
    wire        irq_ssi_tx;
    wire        irq_ssi_rx;
    wire        irq_ssi_tx_exception;
    wire        irq_ssi_rx_exception;
    wire        irq_sci_tx;
    wire        irq_sci_rx;
    wire        irq_sci_rx_exception;
    wire        irq_sci_idle;
    wire        irq_sci_timer;
    wire        sci_idle_ack;
    wire        sci_timer_ack;
    wire [15:0] bcr_wait_states;
    wire        sci_tx_valid;
    wire [7:0]  sci_tx_data;
    wire        sci_tx_address;
    wire        dma_go, dma_dir, dma_mode, dma_done;
    wire [15:0] dma_count;
    wire [31:0] dma_gio_addr;
    wire [15:0] dma_pbus_addr;
    wire [2:0]  dma_sign_mode;
    wire [15:0] dma_count_current;
    wire        dma_folded_unsupported;
    wire [55:0] host_debug_status;

    // The primary HP1 schematic clocks the DSP56001 at 20 MHz.  Keep one
    // synchronous clock domain and advance the core on an exact-average 2/5
    // enable cadence rather than manufacturing a second FPGA clock domain.
    reg [2:0] core_phase;
    wire [2:0] core_phase_sum = core_phase + 3'd2;
    wire core_step = core_phase_sum >= 3'd5;

    always @(posedge clk or negedge dsp_rst_n) begin
        if (!dsp_rst_n)
            core_phase <= 3'd0;
        else if (core_step)
            core_phase <= core_phase_sum - 3'd5;
        else
            core_phase <= core_phase_sum;
    end

    // ========================================================================
    // DSP Core Instance
    // ========================================================================
    dsp56001_core core (
        .clk            (clk),
        .rst_n          (dsp_rst_n),
        .clock_enable   (core_step),
        .halt           (1'b0),
        .run            (dsp_run),
        .software_reset (core_software_reset),
        .stop_reset     (core_stop_reset),

        // P-space fetch
        .p_fetch_addr   (core_p_fetch_addr),
        .p_fetch_en     (core_p_fetch_en),
        .p_fetch_data   (core_p_fetch_data),
        .p_fetch_ready  (core_p_fetch_ready),

        // X-space data
        .x_addr         (core_x_addr),
        .x_rd_en        (core_x_rd_en),
        .x_wr_en        (core_x_wr_en),
        .x_wdata        (core_x_wdata),
        .x_rdata        (core_x_rdata),
        .x_ready        (core_x_ready),

        // Y-space data
        .y_addr         (core_y_addr),
        .y_rd_en        (core_y_rd_en),
        .y_wr_en        (core_y_wr_en),
        .y_wdata        (core_y_wdata),
        .y_rdata        (core_y_rdata),
        .y_ready        (core_y_ready),

        // P-space data (MOVEM)
        .p_data_addr    (core_p_data_addr),
        .p_data_rd_en   (core_p_data_rd_en),
        .p_data_wr_en   (core_p_data_wr_en),
        .p_data_wdata   (core_p_data_wdata),
        .p_data_rdata   (core_p_data_rdata),

        // Interrupts
        .irq_a          (host_force_irqa | host_event_irqa),
        .irq_b          (audio_glue_irqb),
        .irq_ssi_tx     (irq_ssi_tx),
        .irq_ssi_rx     (irq_ssi_rx),
        .irq_ssi_tx_exception(irq_ssi_tx_exception),
        .irq_ssi_rx_exception(irq_ssi_rx_exception),
        .irq_sci_tx     (irq_sci_tx),
        .irq_sci_rx     (irq_sci_rx),
        .irq_sci_rx_exception(irq_sci_rx_exception),
        .irq_sci_idle   (irq_sci_idle),
        .irq_sci_timer  (irq_sci_timer),
        .sci_idle_ack   (sci_idle_ack),
        .sci_timer_ack  (sci_timer_ack),
        .irq_priority   (irq_priority),

        // Debug
        .dbg_reg_wr_en  (dbg_reg_wr_en),
        .dbg_reg_wr_addr(dbg_reg_wr_addr),
        .dbg_reg_wr_data(dbg_reg_wr_data),
        .pc_out         (dbg_pc),
        .sr_out         (dbg_sr),
        .data_rom_enable(core_data_rom_enable),
        .state_out      (dbg_state),
        .busy           (dbg_busy),
        .cur_inst_out   (dbg_cur_inst),
        .acc_a_out      (dbg_acc_a),
        .acc_b_out      (dbg_acc_b)
    );

    // ========================================================================
    // Memory Subsystem Instance
    // ========================================================================
    // P-space: mux fetch and data ports
    // Fetch has priority (data access only during specific states)
    wire [15:0] p_addr  = core_p_fetch_en ? core_p_fetch_addr : core_p_data_addr;
    wire        p_rd_en = core_p_fetch_en || core_p_data_rd_en;
    wire        p_wr_en = !core_p_fetch_en && core_p_data_wr_en;
    wire [23:0] p_wdata = core_p_data_wdata;
    wire [23:0] p_rdata;
    wire        p_ready;

    assign core_p_fetch_data  = p_rdata;
    // Fetch and data accesses are mutually exclusive views of the same
    // physical P-space port.  Preserve ready for MOVEM/MOVEP data cycles too.
    assign core_p_fetch_ready = p_ready;
    assign core_p_data_rdata  = p_rdata;

    dsp56001_mem mem (
        .clk            (clk),
        .rst_n          (rst_n),
        .data_rom_enable(core_data_rom_enable),

        // P-space
        .p_addr         (p_addr),
        .p_rd_en        (p_rd_en),
        .p_wr_en        (p_wr_en),
        .p_wdata        (p_wdata),
        .p_rdata        (p_rdata),
        .p_ready        (p_ready),

        // X-space
        .x_addr         (core_x_addr),
        .x_rd_en        (core_x_rd_en),
        .x_wr_en        (core_x_wr_en),
        .x_wdata        (core_x_wdata),
        .x_rdata        (core_x_rdata),
        .x_ready        (core_x_ready),

        // Y-space
        .y_addr         (core_y_addr),
        .y_rd_en        (core_y_rd_en),
        .y_wr_en        (core_y_wr_en),
        .y_wdata        (core_y_wdata),
        .y_rdata        (core_y_rdata),
        .y_ready        (core_y_ready),

        // External SRAM
        .ext_addr       (mem_ext_addr),
        .ext_wdata      (mem_ext_wdata),
        .ext_rd_en      (mem_ext_rd_en),
        .ext_wr_en      (mem_ext_wr_en),
        .ext_space      (mem_ext_space),
        .ext_rdata      (mem_ext_rdata),
        .ext_ready      (mem_ext_ready),

        // X-Peripheral
        .xperiph_addr   (xperiph_addr),
        .xperiph_wdata  (xperiph_wdata),
        .xperiph_rd_en  (xperiph_rd_en),
        .xperiph_wr_en  (xperiph_wr_en),
        .xperiph_rdata  (xperiph_rdata),

        // Y-Peripheral
        .yperiph_addr   (yperiph_addr),
        .yperiph_wdata  (yperiph_wdata),
        .yperiph_rd_en  (yperiph_rd_en),
        .yperiph_wr_en  (yperiph_wr_en),
        .yperiph_rdata  (yperiph_rdata),

        // External DP8573 RTC window
        .yrtc_addr      (rtc_addr),
        .yrtc_wdata     (rtc_wdata),
        .yrtc_rd_en     (rtc_rd_en),
        .yrtc_wr_en     (rtc_wr_en),
        .yrtc_rdata     (rtc_rdata),

        // Debug
        .dbg_wr_en      (dbg_mem_wr_en),
        .dbg_space      (dbg_mem_space),
        .dbg_addr       (dbg_mem_addr),
        .dbg_wdata      (dbg_mem_wdata)
    );

    // ========================================================================
    // X-Peripheral Instance (SCI, SSI, BCR, IPR)
    // ========================================================================
    dsp56001_periph periph (
        .clk            (clk),
        .rst_n          (dsp_rst_n),
        .clock_enable   (core_step),
        .soft_reset     (core_software_reset),
        .stop_reset     (core_stop_reset),

        .addr           (xperiph_addr),
        .wdata          (xperiph_wdata),
        .rdata          (xperiph_internal_rdata),
        .rd_en          (xperiph_rd_en),
        .wr_en          (xperiph_wr_en),

        .ssi_tx_en      (ssi_tx_en),
        .ssi_tx_data    (ssi_tx_data),
        // HP1 register 10 gates the external SSI transmit clock/frame
        // presented to the DSP.  The board serializer may keep emitting
        // valid S/PDIF silence while stopped, but those subframe boundaries
        // are not DSP SSI time slots until the Actel transmitter is started.
        .ssi_tx_ready   (ssi_tx_ready && audio_glue_tx_clock_active),
        .ssi_rx_valid   (ssi_rx_valid),
        .ssi_rx_data    (ssi_rx_data),
        .ssi_sclk_out   (ssi_sclk_out),
        .ssi_fss_out    (ssi_fss_out),

        // The architectural SCI character engine is live.  IP20 does not
        // route its three SCI pins into the CYAN board top yet, so the future
        // pin shifter sees an idle receive side at this integration boundary.
        .sci_tx_valid   (sci_tx_valid),
        .sci_tx_data    (sci_tx_data),
        .sci_tx_address (sci_tx_address),
        .sci_tx_ready   (1'b0),
        .sci_tx_done    (1'b0),
        .sci_rx_valid   (1'b0),
        .sci_rx_data    (8'h00),
        .sci_rx_r8      (1'b0),
        .sci_rx_framing_error(1'b0),
        .sci_rx_parity_error(1'b0),
        .sci_rx_idle    (1'b0),

        .irq_ssi_tx     (irq_ssi_tx),
        .irq_ssi_rx     (irq_ssi_rx),
        .irq_ssi_tx_exception(irq_ssi_tx_exception),
        .irq_ssi_rx_exception(irq_ssi_rx_exception),
        .irq_sci_tx     (irq_sci_tx),
        .irq_sci_rx     (irq_sci_rx),
        .irq_sci_rx_exception(irq_sci_rx_exception),
        .irq_sci_idle   (irq_sci_idle),
        .irq_sci_timer  (irq_sci_timer),
        .sci_idle_ack   (sci_idle_ack),
        .sci_timer_ack  (sci_timer_ack),
        .irq_priority   (irq_priority),
        .bus_wait_states(bcr_wait_states)
    );

    // The original HP1 routes DSP56001 Port B to the audio Actel glue and
    // routes that device's interrupt output back to IRQB.  Keep the standard
    // on-chip peripheral register model separate from the external device:
    // the glue observes accepted PBD/PBDDR writes and supplies only the Port B
    // input pins during a PBD read.
    indigo_hp1_audio_glue audio_glue (
        .clk            (clk),
        // HP1 pages 35--36 put the DSP56001 and Actel on DSP.RST-.  The
        // RESET instruction is internal and resets only the DSP-owned pins.
        .rst_n          (dsp_rst_n),
        .dsp_software_reset(core_software_reset),
        .xperiph_addr   (xperiph_addr),
        .xperiph_wdata  (xperiph_wdata),
        .xperiph_wr_en  (xperiph_wr_en),
        .tx_subframe_accepted(ssi_tx_en),
        .rx_subframe_received(ssi_rx_valid),
        .tx_channel_status(ssi_tx_channel_status),
        .tx_channel_status_valid(ssi_tx_channel_status_valid),
        .tx_user_data   (ssi_tx_user_data),
        .tx_user_valid  (ssi_tx_user_valid),
        .tx_clock_active(audio_glue_tx_clock_active),
        .tx_rate_select (ssi_tx_rate_select),
        .pbd_rdata      (audio_glue_pbd_rdata),
        .irq_b          (audio_glue_irqb)
    );

    assign xperiph_rdata = (xperiph_addr == 6'h24) ?
                           audio_glue_pbd_rdata :
                           xperiph_internal_rdata;
    assign ssi_tx_active = audio_glue_tx_clock_active;

    // ========================================================================
    // HPC1 Host Interface Instance (Y-Peripheral)
    // ========================================================================
    dsp56001_host host (
        .clk            (clk),
        .rst_n          (rst_n),  // Host uses system reset, not DSP reset

        // DSP-side Y-peripheral
        .dsp_addr       (yperiph_addr),
        .dsp_wdata      (yperiph_wdata),
        .dsp_rdata      (yperiph_rdata),
        .dsp_rd_en      (yperiph_rd_en),
        .dsp_wr_en      (yperiph_wr_en),

        // CPU-side registers
        .cpu_addr       (cpu_reg_addr),
        .cpu_wdata      (cpu_reg_wdata),
        .cpu_rdata      (cpu_reg_rdata),
        .cpu_rd_en      (cpu_reg_rd_en),
        .cpu_wr_en      (cpu_reg_wr_en),

        // Interrupts
        .irq_to_cpu     (irq_out),
        .irq_to_dsp     (host_event_irqa),

        // DMA
        .dma_go         (dma_go),
        .dma_dir        (dma_dir),
        .dma_mode       (dma_mode),
        .dma_count      (dma_count),
        .dma_gio_addr   (dma_gio_addr),
        .dma_pbus_addr  (dma_pbus_addr),
        .dma_sign_mode  (dma_sign_mode),
        .dma_count_current(dma_count_current),
        .dma_done       (dma_done),

        // Control
        .dsp_reset      (host_dsp_reset),
        .dsp_irqa       (host_force_irqa),
        .debug_status   (host_debug_status)
    );

    // ========================================================================
    // External SRAM Instance (32K x 24-bit Dual-Port)
    // ========================================================================
    wire [23:0] sram_dsp_rdata;
    wire        sram_dsp_ready;
    wire        sram_dsp_rd_en;
    wire        sram_dsp_wr_en;
    wire [14:0] dma_sram_addr;
    wire [23:0] dma_sram_wdata;
    wire [23:0] dma_sram_rdata;
    wire        dma_sram_rd_en;
    wire        dma_sram_wr_en;
    wire        dma_sram_ready;

    wire cpu_sram_req = cpu_sram_rd_en | cpu_sram_wr_en;
    wire dma_sram_req = dma_sram_rd_en | dma_sram_wr_en;
    reg  sram_owner_locked;
    reg  sram_owner_dma;
    wire select_dma_sram = sram_owner_locked ? sram_owner_dma :
                           (!cpu_sram_req && dma_sram_req);
    wire selected_sram_req = select_dma_sram ? dma_sram_req : cpu_sram_req;
    wire shared_sram_ready;
    wire [23:0] shared_sram_rdata;

    // DSP56001UM09 sections 9.2.1--9.2.3 define four programmed BCR wait
    // fields.  The IP20 SRAM does not assert the external WT pin, so each
    // selected nonzero field is the exact number of oscillator-clock holds
    // before the one physical SRAM request is accepted.  Count only on the
    // architectural 20 MHz core enable; the surrounding machine domain stays
    // at 50 MHz and must not turn one DSP wait into several fabric writes.
    wire mem_ext_req = mem_ext_rd_en | mem_ext_wr_en;
    wire [3:0] mem_ext_programmed_wait =
        (mem_ext_space == 2'd0) ? bcr_wait_states[7:4] :
        (mem_ext_space == 2'd1) ? bcr_wait_states[15:12] :
                                  bcr_wait_states[11:8];
    reg        mem_wait_active;
    reg [3:0]  mem_wait_left;
    reg [14:0] mem_wait_addr;
    reg [23:0] mem_wait_wdata;
    reg [1:0]  mem_wait_space;
    reg        mem_wait_read;
    reg        mem_wait_write;
    wire mem_wait_match = mem_ext_addr == mem_wait_addr &&
                          (mem_wait_read ||
                           mem_ext_wdata == mem_wait_wdata) &&
                          mem_ext_space == mem_wait_space &&
                          mem_ext_rd_en == mem_wait_read &&
                          mem_ext_wr_en == mem_wait_write;
    wire mem_wait_grant = mem_ext_req &&
        ((!mem_wait_active && mem_ext_programmed_wait == 0) ||
         (core_step && mem_wait_active && mem_wait_left == 0 &&
          mem_wait_match));

    assign sram_dsp_rd_en = mem_ext_rd_en && mem_wait_grant;
    assign sram_dsp_wr_en = mem_ext_wr_en && mem_wait_grant;

    always @(posedge clk or negedge dsp_rst_n) begin
        if (!dsp_rst_n) begin
            mem_wait_active <= 1'b0;
            mem_wait_left <= 4'd0;
            mem_wait_addr <= 15'd0;
            mem_wait_wdata <= 24'd0;
            mem_wait_space <= 2'd0;
            mem_wait_read <= 1'b0;
            mem_wait_write <= 1'b0;
        end else if (core_software_reset) begin
            mem_wait_active <= 1'b0;
            mem_wait_left <= 4'd0;
        end else if (core_step) begin
            if (mem_wait_active) begin
                if (!mem_ext_req || !mem_wait_match) begin
                    // A legal stalled core retains the exact request.  If an
                    // aborted/replaced request reaches this boundary, discard
                    // its count and start the replacement fail-closed.
                    if (mem_ext_req && mem_ext_programmed_wait != 0) begin
                        mem_wait_active <= 1'b1;
                        mem_wait_left <= mem_ext_programmed_wait - 4'd1;
                        mem_wait_addr <= mem_ext_addr;
                        mem_wait_wdata <= mem_ext_wdata;
                        mem_wait_space <= mem_ext_space;
                        mem_wait_read <= mem_ext_rd_en;
                        mem_wait_write <= mem_ext_wr_en;
                    end else begin
                        mem_wait_active <= 1'b0;
                        mem_wait_left <= 4'd0;
                    end
                end else if (mem_wait_left != 0) begin
                    mem_wait_left <= mem_wait_left - 4'd1;
                end else if (sram_dsp_ready) begin
                    mem_wait_active <= 1'b0;
                end
            end else if (mem_ext_req && mem_ext_programmed_wait != 0) begin
                mem_wait_active <= 1'b1;
                mem_wait_left <= mem_ext_programmed_wait - 4'd1;
                mem_wait_addr <= mem_ext_addr;
                mem_wait_wdata <= mem_ext_wdata;
                mem_wait_space <= mem_ext_space;
                mem_wait_read <= mem_ext_rd_en;
                mem_wait_write <= mem_ext_wr_en;
            end
        end
    end

    // The SRAM is physically dual-port: DSP core on one port, HPC/PBUS on the
    // other.  CPU PIO and DSP DMA share the HPC/PBUS port.  Lock ownership
    // across its registered completion so a newly arriving request cannot
    // consume the previous request's response.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sram_owner_locked <= 1'b0;
            sram_owner_dma <= 1'b0;
        end else if (sram_owner_locked) begin
            if (shared_sram_ready)
                sram_owner_locked <= 1'b0;
        end else if (selected_sram_req && !shared_sram_ready) begin
            sram_owner_locked <= 1'b1;
            sram_owner_dma <= select_dma_sram;
        end
    end

    assign cpu_sram_rdata = shared_sram_rdata;
    assign cpu_sram_ready = shared_sram_ready && selected_sram_req &&
                            !select_dma_sram;
    assign dma_sram_rdata = shared_sram_rdata;
    assign dma_sram_ready = shared_sram_ready && selected_sram_req &&
                            select_dma_sram;

    dsp56001_sram sram (
        .clk            (clk),
        .rst_n          (rst_n),

        // CPU port
        .cpu_addr       (select_dma_sram ? dma_sram_addr : cpu_sram_addr),
        .cpu_wdata      (select_dma_sram ? dma_sram_wdata : cpu_sram_wdata),
        .cpu_rdata      (shared_sram_rdata),
        .cpu_wr_en      ((selected_sram_req && select_dma_sram &&
                          dma_sram_wr_en) ||
                         (selected_sram_req && !select_dma_sram &&
                          cpu_sram_wr_en)),
        .cpu_rd_en      ((selected_sram_req && select_dma_sram &&
                          dma_sram_rd_en) ||
                         (selected_sram_req && !select_dma_sram &&
                          cpu_sram_rd_en)),
        .cpu_ready      (shared_sram_ready),

        // DSP port
        .dsp_addr       (mem_ext_addr),
        .dsp_wdata      (mem_ext_wdata),
        .dsp_rdata      (sram_dsp_rdata),
        .dsp_wr_en      (sram_dsp_wr_en),
        .dsp_rd_en      (sram_dsp_rd_en),
        .dsp_ready      (sram_dsp_ready)
    );

    assign mem_ext_rdata = sram_dsp_rdata;
    assign mem_ext_ready = sram_dsp_ready;

    dsp56001_dma dma (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .dma_go                (dma_go),
        .dma_dir               (dma_dir),
        .dma_mode              (dma_mode),
        .dma_count             (dma_count),
        .dma_gio_addr          (dma_gio_addr),
        .dma_pbus_addr         (dma_pbus_addr),
        .dma_sign_mode         (dma_sign_mode),
        .dma_done              (dma_done),
        .dma_active            (dma_mem_active),
        .dma_count_current     (dma_count_current),
        .dma_folded_unsupported(dma_folded_unsupported),
        .sram_addr             (dma_sram_addr),
        .sram_wdata            (dma_sram_wdata),
        .sram_rdata            (dma_sram_rdata),
        .sram_rd_en            (dma_sram_rd_en),
        .sram_wr_en            (dma_sram_wr_en),
        .sram_ready            (dma_sram_ready),
        .mem_addr              (dma_mem_addr),
        .mem_wdata             (dma_mem_wdata),
        .mem_rdata             (dma_mem_rdata),
        .mem_wmask             (dma_mem_wmask),
        .mem_valid             (dma_mem_valid),
        .mem_write             (dma_mem_write),
        .mem_ready             (dma_mem_ready)
    );

    assign debug_probe = {
        32'h4453_5031, // "DSP1"
        {dbg_pc, dbg_state, dbg_busy, host_dsp_reset, irq_out, 9'd0},
        dbg_cur_inst,
        dbg_sr,
        dbg_acc_a,
        dbg_acc_b[55:24],
        host_debug_status
    };

endmodule

//////////////////////////////////////////////////////////////////////////////
// Indigo HP1 audio Actel Port-B boundary
//
// DSP Port B connects directly to the board audio glue and its active-low
// interrupt output connects to DSP IRQB. Bits 12:8 select the glue register, a falling
// PBD14 commits a write, and a falling PBD13 starts a read.  Register 5 returns
// the supported glue revision.  Register 7 is the active-low interrupt mask;
// register 8 is active-low pending status, write-one-to-clear, with an all-zero
// write causing the diagnostic soft source (bit 4).
//
// Block sources 0 and 2 advance from received and transmitted SSI subframes:
// one AES3 block is 192 two-subframe frames.  Transmit metadata source 3 is
// published at that same real block boundary after four staged bytes.
// Receive metadata source 1 drains the YM3436 13-byte microprocessor record.
// Consumer records contain C[31:0], U[31:0], then zeros.  Professional records
// contain C[31:0], the transmitter-generated local sample address, four zero
// reliability flags, a clear CRC-error flag, then U[31:0].
//////////////////////////////////////////////////////////////////////////////

module indigo_hp1_audio_glue (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        dsp_software_reset,
    input  wire [5:0]  xperiph_addr,
    input  wire [23:0] xperiph_wdata,
    input  wire        xperiph_wr_en,
    input  wire        tx_subframe_accepted,
    input  wire        rx_subframe_received,
    output wire [31:0] tx_channel_status,
    output wire        tx_channel_status_valid,
    output wire [31:0] tx_user_data,
    output wire        tx_user_valid,
    output wire        tx_clock_active,
    output wire [5:0]  tx_rate_select,
    output wire [23:0] pbd_rdata,
    output wire        irq_b
);
    localparam [5:0] A_PBC   = 6'h20;
    localparam [5:0] A_PBDDR = 6'h22;
    localparam [5:0] A_PBD   = 6'h24;
    localparam [4:0] R_REVISION = 5'h05;
    localparam [4:0] R_IRQ_MASK = 5'h07;
    localparam [4:0] R_IRQ_STATUS = 5'h08;
    localparam [4:0] R_RX_CONTROL = 5'h02;
    localparam [4:0] R_RX_NONAUDIO = 5'h03;
    localparam [4:0] R_TX_CONTROL = 5'h0a;
    localparam [4:0] R_TX_NONAUDIO = 5'h0b;
    localparam [4:0] R_TX_AUX_CONTROL = 5'h0c;
    localparam [8:0] TX_SUBFRAMES_PER_BLOCK = 9'd384;

    reg [23:0] pbd_latch;
    reg [23:0] pbddr_latch;
    reg        pbc_host_mode;
    reg [4:0]  irq_enable;
    reg [4:0]  irq_pending;
    reg [4:0]  read_register;
    reg        read_active;
    reg        tx_active;
    reg [5:0]  tx_rate;
    reg [8:0]  tx_subframe_count;
    reg [31:0] tx_local_sample_address;
    reg [31:0] tx_block_sample_address;
    reg [2:0]  tx_nonaudio_count;
    reg [1:0]  tx_nonaudio_type;
    reg [2:0]  tx_nonaudio_valid;
    reg [7:0]  tx_nonaudio_data [0:11];
    reg        rx_active;
    reg [8:0]  rx_subframe_count;
    reg        rx_metadata_ready;
    reg [3:0]  rx_metadata_index;
    reg [31:0] rx_metadata_sample_address;
    integer metadata_i;

    wire pbc_write = xperiph_wr_en && xperiph_addr == A_PBC;
    wire pbd_write = xperiph_wr_en && xperiph_addr == A_PBD;
    wire pbddr_write = xperiph_wr_en && xperiph_addr == A_PBDDR;
    // Only PBDDR-designated outputs reach the external pins.  The modeled
    // Actel supplies pull-high/read data on PB7:0 and no source on PB14:8.
    // PBD still retains every written output-latch bit while a pin is input.
    wire [14:0] pbd_input_pins = 15'h00ff;
    wire [14:0] pbd_pins_before =
        (pbd_latch[14:0] & pbddr_latch[14:0]) |
        (pbd_input_pins & ~pbddr_latch[14:0]);
    wire [14:0] pbd_pins_after =
        (xperiph_wdata[14:0] & pbddr_latch[14:0]) |
        (pbd_input_pins & ~pbddr_latch[14:0]);
    wire write_strobe_fall = pbd_write && !pbc_host_mode &&
                             pbd_pins_before[14] && !pbd_pins_after[14];
    wire read_strobe_fall = pbd_write && !pbc_host_mode &&
                            pbd_pins_before[13] && !pbd_pins_after[13];
    wire read_strobe_rise = pbd_write && !pbc_host_mode &&
                            !pbd_pins_before[13] && pbd_pins_after[13];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pbd_latch <= 24'h0;
            pbddr_latch <= 24'h0;
            pbc_host_mode <= 1'b0;
            irq_enable <= 5'h0;
            irq_pending <= 5'h0;
            read_register <= 5'h0;
            read_active <= 1'b0;
            tx_active <= 1'b0;
            tx_rate <= 6'd0;
            tx_subframe_count <= 9'h0;
            tx_local_sample_address <= 32'h0;
            tx_block_sample_address <= 32'h0;
            tx_nonaudio_count <= 3'h0;
            tx_nonaudio_type <= 2'h0;
            tx_nonaudio_valid <= 3'h0;
            for (metadata_i = 0; metadata_i < 12;
                 metadata_i = metadata_i + 1)
                tx_nonaudio_data[metadata_i] <= 8'h0;
            rx_active <= 1'b0;
            rx_subframe_count <= 9'h0;
            rx_metadata_ready <= 1'b0;
            rx_metadata_index <= 4'h0;
            rx_metadata_sample_address <= 32'h0;
        end else begin
            // The DSP56001 RESET instruction returns Port B to input state
            // without asserting the external DSP.RST- pin.  Clear only the
            // DSP-owned pin/interface shadow here; transmitter, receiver,
            // metadata and interrupt state survive this internal reset.
            if (dsp_software_reset) begin
                pbd_latch <= 24'h0;
                pbddr_latch <= 24'h0;
                pbc_host_mode <= 1'b0;
                read_register <= 5'h0;
                read_active <= 1'b0;
            end else begin
                if (pbc_write)
                    pbc_host_mode <= xperiph_wdata[0];
                if (pbddr_write)
                    pbddr_latch <= xperiph_wdata & 24'h007fff;

                if (pbd_write) begin
                    // The external Indigo glue observes the same fifteen-pin
                    // Port B boundary as the DSP56001 register model.  Keep
                    // reserved X-data-bus bits out of composed PBD readback.
                    pbd_latch <= xperiph_wdata & 24'h007fff;

                    if (read_strobe_fall) begin
                        read_register <= pbd_pins_after[12:8];
                        read_active <= 1'b1;
                    end else if (read_strobe_rise) begin
                        read_active <= 1'b0;
                    end

                    if (write_strobe_fall) begin
                        case (pbd_pins_after[12:8])
                        R_IRQ_MASK: begin
                            irq_enable <= ~pbd_pins_after[4:0];
                            if (!pbd_pins_after[1] && rx_metadata_ready)
                                irq_pending[1] <= 1'b1;
                        end
                        R_IRQ_STATUS: begin
                            if (pbd_pins_after[4:0] == 5'h00)
                                irq_pending[4] <= 1'b1;
                            else
                                irq_pending <= irq_pending &
                                               ~pbd_pins_after[4:0];
                        end
                        R_RX_CONTROL: begin
                            // Receiver control values carry GO in bit 4;
                            // clearing it selects reset/no-go.
                            rx_active <= pbd_pins_after[4];
                            if (!pbd_pins_after[4])
                                rx_subframe_count <= 9'h0;
                        end
                        R_TX_CONTROL: begin
                            tx_active <= pbd_pins_after[0];
                            // The programmed sample-rate field is shifted left
                            // once before register 10 is committed. Bits 6:1 are the complete
                            // audio-glue
                            // sample-rate selector; bit 0 is GO.
                            tx_rate <= pbd_pins_after[6:1];
                            if (!pbd_pins_after[0])
                                tx_subframe_count <= 9'h0;
                        end
                        R_TX_NONAUDIO: begin
                            if (tx_nonaudio_type < 2'd3 &&
                                tx_nonaudio_count < 3'd4) begin
                                tx_nonaudio_data[
                                    {tx_nonaudio_type,
                                     tx_nonaudio_count[1:0]}] <=
                                    pbd_pins_after[7:0];
                                if (tx_nonaudio_count == 3'd3)
                                    tx_nonaudio_valid[tx_nonaudio_type] <=
                                        1'b1;
                                tx_nonaudio_count <=
                                    tx_nonaudio_count + 1'b1;
                            end
                        end
                        R_TX_AUX_CONTROL: begin
                            // Bit 6 resets the AES3
                            // sample-address/block clock before C-bit writes.
                            if (pbd_pins_after[6]) begin
                                tx_subframe_count <= 9'h0;
                                tx_local_sample_address <= 32'h0;
                                tx_block_sample_address <= 32'h0;
                                tx_nonaudio_valid <= 3'h0;
                            end
                            tx_nonaudio_type <= pbd_pins_after[1:0];
                            tx_nonaudio_count <= 3'h0;
                        end
                            default: ;
                        endcase
                    end
                end
            end

            if (tx_subframe_accepted && tx_active) begin
                if (tx_subframe_count == 9'h0)
                    tx_block_sample_address <= tx_local_sample_address;
                if (tx_subframe_count[0])
                    tx_local_sample_address <=
                        tx_local_sample_address + 1'b1;
                if (tx_subframe_count == TX_SUBFRAMES_PER_BLOCK - 1'b1) begin
                    tx_subframe_count <= 9'h0;
                    irq_pending[2] <= 1'b1;
                    if (tx_nonaudio_count == 3'd4) begin
                        irq_pending[3] <= 1'b1;
                        tx_nonaudio_count <= 3'h0;
                    end
                end else begin
                    tx_subframe_count <= tx_subframe_count + 1'b1;
                end
            end

            if (rx_subframe_received && rx_active) begin
                if (rx_subframe_count == TX_SUBFRAMES_PER_BLOCK - 1'b1) begin
                    rx_subframe_count <= 9'h0;
                    irq_pending[0] <= 1'b1;
                    if (tx_nonaudio_valid == 3'b111) begin
                        rx_metadata_ready <= 1'b1;
                        rx_metadata_index <= 4'h0;
                        rx_metadata_sample_address <=
                            tx_block_sample_address;
                        if (irq_enable[1])
                            irq_pending[1] <= 1'b1;
                    end
                end else begin
                    rx_subframe_count <= rx_subframe_count + 1'b1;
                end
            end

            // Software acknowledges source 1 before reading register 3.
            // Completing that read advances exactly one byte and republishes
            // the next source-1 event until all 13 consumer bytes are drained.
            if (!dsp_software_reset && read_strobe_rise &&
                pbd_pins_after[12:8] == R_RX_NONAUDIO &&
                rx_metadata_ready) begin
                if (rx_metadata_index == 4'd12) begin
                    rx_metadata_ready <= 1'b0;
                end else begin
                    rx_metadata_index <= rx_metadata_index + 1'b1;
                    if (irq_enable[1])
                        irq_pending[1] <= 1'b1;
                end
            end
        end
    end

    reg [7:0] external_pbd_data;
    reg [7:0] rx_metadata_data;
    wire professional_mode = tx_nonaudio_data[0][0];
    // YM3436 professional output D64--D68 are reliability[3:0] and
    // CRC-error.  The clean internal loopback makes all five zero.  User data
    // therefore begins at D69, five bit positions into the returned byte 8.
    wire [39:0] professional_user_record =
        {3'b000, tx_nonaudio_data[7], tx_nonaudio_data[6],
         tx_nonaudio_data[5], tx_nonaudio_data[4], 5'b00000};
    always @(*) begin
        case (rx_metadata_index)
            4'd0: rx_metadata_data = tx_nonaudio_data[0];
            4'd1: rx_metadata_data = tx_nonaudio_data[1];
            4'd2: rx_metadata_data = tx_nonaudio_data[2];
            4'd3: rx_metadata_data = tx_nonaudio_data[3];
            4'd4: rx_metadata_data = professional_mode ?
                                           rx_metadata_sample_address[7:0] :
                                           tx_nonaudio_data[4];
            4'd5: rx_metadata_data = professional_mode ?
                                           rx_metadata_sample_address[15:8] :
                                           tx_nonaudio_data[5];
            4'd6: rx_metadata_data = professional_mode ?
                                           rx_metadata_sample_address[23:16] :
                                           tx_nonaudio_data[6];
            4'd7: rx_metadata_data = professional_mode ?
                                           rx_metadata_sample_address[31:24] :
                                           tx_nonaudio_data[7];
            4'd8: rx_metadata_data = professional_mode ?
                                           professional_user_record[7:0] :
                                           8'h00;
            4'd9: rx_metadata_data = professional_mode ?
                                           professional_user_record[15:8] :
                                           8'h00;
            4'd10: rx_metadata_data = professional_mode ?
                                            professional_user_record[23:16] :
                                            8'h00;
            4'd11: rx_metadata_data = professional_mode ?
                                            professional_user_record[31:24] :
                                            8'h00;
            4'd12: rx_metadata_data = professional_mode ?
                                            professional_user_record[39:32] :
                                            8'h00;
            default: rx_metadata_data = 8'h00;
        endcase
    end

    always @(*) begin
        external_pbd_data = 8'hff;
        if (read_active) begin
            case (read_register)
                R_REVISION: external_pbd_data = 8'h02;
                R_IRQ_STATUS:
                    external_pbd_data = {3'b111, ~irq_pending};
                R_RX_NONAUDIO:
                    external_pbd_data = rx_metadata_data;
                default: external_pbd_data = 8'hff;
            endcase
        end
    end

    // Output-configured Port B pins read back their latch.  Only the low byte
    // is driven by the external glue logic during read cycles.
    assign pbd_rdata = (pbd_latch & pbddr_latch) |
                       ({16'h0, external_pbd_data} & ~pbddr_latch);
    assign irq_b = |(irq_pending & irq_enable);

    // The HP1 transmitter accepts four little-bit-order bytes for each of
    // channel status and user data.  Publish the already-staged values to the
    // physical serializer; this boundary intentionally does not reinterpret
    // or synthesize metadata.
    assign tx_channel_status = {tx_nonaudio_data[3], tx_nonaudio_data[2],
                                tx_nonaudio_data[1], tx_nonaudio_data[0]};
    assign tx_channel_status_valid = tx_nonaudio_valid[0];
    assign tx_user_data = {tx_nonaudio_data[7], tx_nonaudio_data[6],
                           tx_nonaudio_data[5], tx_nonaudio_data[4]};
    assign tx_user_valid = tx_nonaudio_valid[1] && tx_nonaudio_valid[2] &&
                           tx_nonaudio_data[11][4];
    assign tx_clock_active = tx_active;
    assign tx_rate_select = tx_rate;

endmodule
