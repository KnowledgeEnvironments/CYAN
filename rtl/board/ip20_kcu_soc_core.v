// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// KCU105 IP20 SoC core.
//
// This is the functional DE2i IP20 cluster with board-specific SDRAM/MII/front
// panel code removed: R4000 + MC + external RAM handshake + HPC1/SCC/EEPROM +
// IP20 PROM ROM + minimal board status.

`default_nettype none

module ip20_kcu_soc_core #(
    parameter integer CLK_HZ = 50000000,
    parameter integer SCACHE_LINES = 1024,
    parameter FULL_ICACHE_INCLUSION = 0,
    // The registered lookahead is selectable so integrators can trade timing
    // closure against instruction-fetch overlap without changing cache
    // organization.
    parameter CPU_ICACHE_LOOKAHEAD = 0,
    parameter HAS_DSP = 0,
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
    parameter EEPROM_INIT_FILE = ""
) (
    input  wire        clk,
    input  wire        cpu_clk,
    input  wire        lg1_serial_clk,
    input  wire        lg1_serial_rst_n,
    input  wire        rst_n,

    input  wire        uart_rxd,
    output wire        uart_txd,
    input  wire [7:0]  debug_rx_inject_data,
    input  wire        debug_rx_inject_toggle,

    input  wire [7:0]  host_kbd_data,
    input  wire        host_kbd_valid,
    output wire        host_kbd_ready,
    input  wire [7:0]  host_mouse_data,
    input  wire        host_mouse_valid,
    output wire        host_mouse_ready,
    output wire [7:0]  kbd_tx_event_data,
    output wire        kbd_tx_event_valid,
    input  wire        kbd_tx_event_ready,
    output wire [7:0]  mouse_tx_event_data,
    output wire        mouse_tx_event_valid,
    input  wire        mouse_tx_event_ready,
    output wire [7:0]  kbd_rx_event_data,
    output wire        kbd_rx_event_valid,
    input  wire        kbd_rx_event_ready,
    output wire [7:0]  mouse_rx_event_data,
    output wire        mouse_rx_event_valid,
    input  wire        mouse_rx_event_ready,
    output wire [15:0] host_kbd_queued,
    output wire [15:0] host_mouse_queued,
    output wire [31:0] kbd_tx_event_overflow,
    output wire [31:0] mouse_tx_event_overflow,
    output wire [31:0] scc0_access_event_data,
    output wire        scc0_access_event_valid,

    output wire [25:0] ram_addr,
    output wire [63:0] ram_wdata,
    output wire [ 7:0] ram_wmask,
    output wire        ram_wen,
    output wire        ram_valid,
    input  wire [63:0] ram_rdata,
    input  wire        ram_ready,

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

    output wire        sd_cs_n,
    output wire        sd_clk,
    output wire        sd_mosi,
    input  wire        sd_miso,
    input  wire        sd_card_detect,
    input  wire        sd_read_skip_first_byte,
    input  wire        sd_debug_read_trigger,
    input  wire [2:0]  sd_debug_lba_select,
    input  wire        sd_debug_probe_select,
    input  wire        cpu_debug_probe_select,
    input  wire        dsp_debug_probe_select,
    input  wire        gfx_debug_probe_select,
    input  wire        lg1_capture_trigger,
    input  wire        lg1_capture_hold,
    input  wire        force_uncached,

    output wire         lg1_line_valid,
    input  wire         lg1_line_ready,
    input  wire         lg1_line_write_done,
    output wire         lg1_line_write,
    output wire [15:0]  lg1_line_addr,
    output wire [511:0] lg1_line_wdata,
    output wire [63:0]  lg1_line_wstrb,
    input  wire         lg1_line_rsp_valid,
    output wire         lg1_line_rsp_ready,
    input  wire [511:0] lg1_line_rsp_rdata,
    output wire         lg1_scan_mem_valid,
    input  wire         lg1_scan_mem_ready,
    output wire [15:0]  lg1_scan_mem_addr,
    input  wire         lg1_scan_mem_rsp_valid,
    output wire         lg1_scan_mem_rsp_ready,
    input  wire [511:0] lg1_scan_mem_rsp_rdata,
    output wire [31:0]  lg1_capture_bist_status,
    output wire         video_de,
    output wire         video_hsync,
    output wire         video_vsync,
    output wire [7:0]   video_r,
    output wire [7:0]   video_g,
    output wire [7:0]   video_b,
    output wire         video_valid,
    output wire         video_frame_start,
    output wire [9:0]   video_x,
    output wire [9:0]   video_y,
    output wire [7:0]   video_raw_index,
    output wire [4:0]   video_did,
    output wire [7:0]   video_display_index,
    output wire [15:0]  video_xmap_mode,
    output wire [1:0]   video_palette_bank,
    output wire [2:0]   video_resolver_class,
    output wire [91:0]  video_resolver_telemetry,
    output wire [91:0]  video_resolver_detail,
    output wire         video_resolver_frame_done,
    output wire [15:0]  video_fetch_low_signature,
    output wire [15:0]  video_fetch_high_signature,
    output wire [7:0]   video_fetch_signature_generation,
    output wire         video_fetch_signature_valid,
    output wire [31:0]  video_underrun_count,
    output wire [31:0]  video_protocol_error_count,
    output wire [127:0] gfx_retrace_status,

    output wire [23:0]  dsp_audio_sample_data,
    output wire         dsp_audio_sample_valid,
    output wire [31:0]  dsp_audio_channel_status,
    output wire         dsp_audio_channel_status_valid,
    output wire [31:0]  dsp_audio_user_data,
    output wire         dsp_audio_user_valid,
    output wire         dsp_audio_stream_active,
    output wire [5:0]   dsp_audio_rate_select,
    input  wire         dsp_audio_sample_ready,

    output wire         lg1_config_access_valid,
    output wire [87:0]  lg1_config_access_payload,
    output wire [ 2:0]  lg1_config_current_sel,

    output reg  [3:0]  bringup_phase,
    output reg         heartbeat,
    output wire [31:0] cpu_debug_pc,
    output wire [31:0] cpu_debug_mem_addr,
    output wire [31:0] status_flags,
    output wire [127:0] debug_probe,
    output wire [127:0] eeprom_debug_probe,
    output wire [255:0] cpu_flow_debug_probe,
    output wire [255:0] cpu_exception_debug_probe,
    output wire [511:0] cpu_gpr_debug_probe
);
    localparam PROM_FULL_CYCLE_READ = 1'b1;

    reg [15:0] eeprom_init_mem [0:127];
    integer eeprom_init_index;
    initial begin
        for (eeprom_init_index = 0; eeprom_init_index < 128;
             eeprom_init_index = eeprom_init_index + 1)
            eeprom_init_mem[eeprom_init_index] = 16'hffff;
        if (EEPROM_INIT_FILE != "")
            $readmemh(EEPROM_INIT_FILE, eeprom_init_mem);
    end

    reg [6:0] eeprom_init_addr;
    reg       eeprom_init_done;
    // Keep the write strobe combinational with the current address.  The old
    // registered strobe advanced init_addr on the same edge that asserted
    // init_we, so the EEPROM observed init_we=0 for address 0 and started at
    // word 1.  Word 0 is the PROM checksum/revision word; skipping it makes
    // every otherwise-valid NVRAM image fail its checksum on hardware.
    wire      eeprom_init_we = rst_n && !eeprom_init_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            eeprom_init_addr <= 7'd0;
            eeprom_init_done <= 1'b0;
        end else if (!eeprom_init_done) begin
            if (eeprom_init_addr == 7'd127)
                eeprom_init_done <= 1'b1;
            else
                eeprom_init_addr <= eeprom_init_addr + 7'd1;
        end
    end

    reg [7:0] reset_shift;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            reset_shift <= 8'h00;
        else if (eeprom_init_done)
            reset_shift <= {reset_shift[6:0], 1'b1};
    end

    wire soc_rst_n = &reset_shift;

    // The externally generated serial reset is already synchronized to the
    // 64 MHz clock.  Gate it with the later SoC/EEPROM release, asserting
    // asynchronously but releasing through serial-domain flops.
    (* ASYNC_REG = "TRUE" *) reg [3:0] lg1_serial_soc_reset_shift;
    wire lg1_serial_soc_release = soc_rst_n && lg1_serial_rst_n;
    always @(posedge lg1_serial_clk or negedge lg1_serial_soc_release) begin
        if (!lg1_serial_soc_release)
            lg1_serial_soc_reset_shift <= 4'b0000;
        else
            lg1_serial_soc_reset_shift <=
                {lg1_serial_soc_reset_shift[2:0], 1'b1};
    end
    wire lg1_serial_soc_rst_n = &lg1_serial_soc_reset_shift;

    // The complete R4000/cache hierarchy is an independent clock island.
    // Release its CDC endpoint first so level controls can settle, then hold
    // the architectural CPU/cache reset for four additional CPU edges.  This
    // prevents the first fetch from preceding synchronized force/IRQ state.
    (* ASYNC_REG = "TRUE" *) reg [2:0] cpu_cdc_reset_sync;
    always @(posedge cpu_clk or negedge soc_rst_n) begin
        if (!soc_rst_n)
            cpu_cdc_reset_sync <= 3'b000;
        else
            cpu_cdc_reset_sync <= {cpu_cdc_reset_sync[1:0], 1'b1};
    end
    wire cpu_cdc_rst_n = cpu_cdc_reset_sync[2];

    reg [3:0] cpu_core_reset_shift;
    always @(posedge cpu_clk or negedge cpu_cdc_rst_n) begin
        if (!cpu_cdc_rst_n)
            cpu_core_reset_shift <= 4'b0000;
        else
            cpu_core_reset_shift <= {cpu_core_reset_shift[2:0], 1'b1};
    end
    wire cpu_soc_rst_n = &cpu_core_reset_shift;

    wire [31:0] imem_paddr;
    wire        imem_ren;
    wire [31:0] imem_rdata;
    wire        island_dmem_ren;
    wire        island_dmem_wen;
    wire [31:0] island_dmem_paddr;
    wire [ 7:0] island_dmem_wmask;
    wire [63:0] island_dmem_wdata;
    wire [63:0] island_dmem_rdata;
    wire        island_dmem_ready;
    wire        island_dmem_uncached_wr;
    wire        island_dmem_cache_owned;
    wire        cpu_dmem_ren;
    wire        cpu_dmem_wen;
    wire [31:0] cpu_dmem_paddr;
    wire [ 7:0] cpu_dmem_wmask;
    wire [63:0] cpu_dmem_wdata;
    wire [63:0] cpu_dmem_rdata;
    wire        cpu_dmem_ready;
    wire        cpu_dmem_uncached_wr;
    wire        cpu_dmem_cache_owned;
    wire [31:0] cpu_dmem_pc;
    wire [31:0] cpu_debug_core_pc;
    wire [31:0] cpu_debug_core_is_pc;
    wire [31:0] cpu_debug_core_rf_pc;
    wire [31:0] cpu_debug_core_rf_insn;
    wire [31:0] cpu_debug_core_wb_pc;
    wire [31:0] cpu_debug_core_wb_insn;
    wire [31:0] cpu_debug_core_epc;
    wire [31:0] cpu_debug_core_cause;
    wire [31:0] cpu_debug_core_badvaddr;
    wire [31:0] cpu_debug_core_status;
    wire [31:0] cpu_debug_core_entryhi;
    wire [31:0] cpu_debug_core_flags;
    wire [31:0] cpu_debug_cache_status;
    wire [31:0] cpu_debug_cache_addr;
    wire [31:0] cpu_debug_core_gpr_v0;
    wire [31:0] cpu_debug_core_gpr_v1;
    wire [31:0] cpu_debug_core_gpr_a0;
    wire [31:0] cpu_debug_core_gpr_a1;
    wire [31:0] cpu_debug_core_gpr_a2;
    wire [31:0] cpu_debug_core_gpr_a3;
    wire [31:0] cpu_debug_core_gpr_s0;
    wire [31:0] cpu_debug_core_gpr_s1;
    wire [31:0] cpu_debug_core_gpr_s2;
    wire [31:0] cpu_debug_core_gpr_s3;
    wire [31:0] cpu_debug_core_gpr_s4;
    wire [31:0] cpu_debug_core_gpr_s5;
    wire [31:0] cpu_debug_core_gpr_s6;
    wire [31:0] cpu_debug_core_gpr_t2;
    wire [31:0] cpu_debug_core_gpr_t3;
    wire [31:0] cpu_debug_core_gpr_t4;
    wire [31:0] cpu_debug_core_gpr_t6;
    wire [31:0] cpu_debug_core_gpr_t7;
    wire [31:0] cpu_debug_core_gpr_t8;
    wire [31:0] cpu_debug_core_gpr_t9;
    wire [31:0] cpu_debug_core_gpr_sp;
    wire [31:0] cpu_debug_core_gpr_ra;
    wire [31:0] cpu_debug_core_last_cf_pc;
    wire [31:0] cpu_debug_core_last_cf_target;
    wire [31:0] cpu_debug_core_wb_eff_addr;
    wire [31:0] cpu_debug_core_wb_load_data;
    wire        cpu_debug_core_store_commit;
    wire [31:0] cpu_debug_core_store_pc;
    wire [31:0] cpu_debug_core_store_insn;
    wire [31:0] cpu_debug_core_store_paddr;
    wire [31:0] cpu_debug_core_store_vaddr;
    wire [ 7:0] cpu_debug_core_store_wmask;
    wire [63:0] cpu_debug_core_store_wdata;
    wire [31:0] cpu_debug_dtlb_state;
    wire [31:0] cpu_debug_dtlb_paddr;
    wire [31:0] cpu_debug_dtlb_lo;
    wire [31:0] cpu_debug_dtlb_mask;
    wire [31:0] cpu_debug_core_gpr_at;
    wire [31:0] cpu_debug_core_gpr_t0;
    wire [31:0] cpu_debug_core_gpr_t1;
    wire [5:0]  combined_cpu_irq;
    wire [5:0]  island_cpu_irq;
    wire        island_force_uncached;

    wire [31:0] hpc1_dma_addr;
    wire [63:0] hpc1_dma_wdata;
    wire [ 7:0] hpc1_dma_wmask;
    wire        hpc1_dma_valid;
    wire        hpc1_dma_write;
    wire        hpc1_dma_ready;
    wire [63:0] hpc1_dma_rdata_raw;
    wire        mc_hpc_dma_ready;
    wire [31:0] hpc_dma_slot_addr;
    wire [63:0] hpc_dma_slot_wdata;
    wire [ 7:0] hpc_dma_slot_wmask;
    wire        hpc_dma_slot_write;

    wire        dma_snoop_valid;
    wire [31:0] dma_snoop_addr;
    wire [63:0] dma_snoop_wdata;
    wire [ 7:0] dma_snoop_wmask;
    wire        mc_snoop_rd_valid;
    wire [31:0] mc_snoop_rd_addr;
    wire        mc_snoop_rd_resp_valid;
    wire        mc_snoop_rd_hit;
    wire [63:0] mc_snoop_rd_data;
    wire        machine_dma_snoop_valid;
    wire        machine_dma_snoop_ready;
    wire        machine_dma_snoop_complete;
    wire        island_snoop_wr_ready;
    wire        island_snoop_wr_done;
    wire        island_snoop_rd_valid;
    wire [31:0] island_snoop_rd_addr;
    wire        island_snoop_rd_resp_valid;
    wire        island_snoop_rd_hit;
    wire [63:0] island_snoop_rd_data;
    wire        machine_snoop_rd_ready;
    wire        machine_core_hold;
    wire        machine_core_hold_ack;
    wire        island_core_hold;
    wire        island_core_hold_quiesced;
    wire        machine_dmem_fence;
    wire        machine_dmem_fence_ack;
    wire        mc_hpc_dma_request;
    wire        mc_hpc_dma_valid;
    wire        mc_hpc_dma_write;
    wire [3:0]  hpc_dma_state;
    wire        hpc_dma_probe_hit;
    wire [63:0] hpc_dma_probe_data;
    wire [63:0] hpc1_dma_rdata_snooped;
    wire        hpc_dma_read_accept;

    cyan_cpu_island_bridge u_cpu_island_bridge (
        .cpu_clk(cpu_clk),
        .cpu_rst_n(cpu_cdc_rst_n),
        .cpu_dmem_ren(island_dmem_ren),
        .cpu_dmem_wen(island_dmem_wen),
        .cpu_dmem_paddr(island_dmem_paddr),
        .cpu_dmem_wmask(island_dmem_wmask),
        .cpu_dmem_wdata(island_dmem_wdata),
        .cpu_dmem_uncached_wr(island_dmem_uncached_wr),
        .cpu_dmem_cache_owned(island_dmem_cache_owned),
        .cpu_dmem_pc(cpu_debug_core_store_pc),
        .cpu_dmem_rdata(island_dmem_rdata),
        .cpu_dmem_ready(island_dmem_ready),
        .machine_clk(clk),
        .machine_rst_n(soc_rst_n),
        .machine_dmem_ren(cpu_dmem_ren),
        .machine_dmem_wen(cpu_dmem_wen),
        .machine_dmem_paddr(cpu_dmem_paddr),
        .machine_dmem_wmask(cpu_dmem_wmask),
        .machine_dmem_wdata(cpu_dmem_wdata),
        .machine_dmem_uncached_wr(cpu_dmem_uncached_wr),
        .machine_dmem_cache_owned(cpu_dmem_cache_owned),
        .machine_dmem_pc(cpu_dmem_pc),
        .machine_dmem_rdata(cpu_dmem_rdata),
        .machine_dmem_ready(cpu_dmem_ready),
        .machine_snoop_wr_valid(machine_dma_snoop_valid),
        .machine_snoop_wr_addr(hpc_dma_slot_addr),
        .machine_snoop_wr_data(hpc_dma_slot_wdata),
        .machine_snoop_wr_mask(hpc_dma_slot_wmask),
        .machine_snoop_wr_ready(machine_dma_snoop_ready),
        .cpu_snoop_wr_valid(dma_snoop_valid),
        .cpu_snoop_wr_addr(dma_snoop_addr),
        .cpu_snoop_wr_data(dma_snoop_wdata),
        .cpu_snoop_wr_mask(dma_snoop_wmask),
        .cpu_snoop_wr_ready(island_snoop_wr_ready),
        .cpu_snoop_wr_done(island_snoop_wr_done),
        .machine_snoop_wr_complete(machine_dma_snoop_complete),
        .machine_snoop_rd_valid(mc_snoop_rd_valid),
        .machine_snoop_rd_addr(mc_snoop_rd_addr),
        .machine_snoop_rd_ready(machine_snoop_rd_ready),
        .cpu_snoop_rd_valid(island_snoop_rd_valid),
        .cpu_snoop_rd_addr(island_snoop_rd_addr),
        .cpu_snoop_rd_resp_valid(island_snoop_rd_resp_valid),
        .cpu_snoop_rd_resp_hit(island_snoop_rd_hit),
        .cpu_snoop_rd_resp_data(island_snoop_rd_data),
        .machine_snoop_rd_resp_valid(mc_snoop_rd_resp_valid),
        .machine_snoop_rd_resp_hit(mc_snoop_rd_hit),
        .machine_snoop_rd_resp_data(mc_snoop_rd_data),
        .machine_irq(combined_cpu_irq),
        .cpu_irq(island_cpu_irq),
        .machine_force_uncached(force_uncached),
        .cpu_force_uncached(island_force_uncached),
        .machine_core_hold(machine_core_hold),
        .machine_core_hold_ack(machine_core_hold_ack),
        .cpu_core_hold(island_core_hold),
        .cpu_core_hold_quiesced(island_core_hold_quiesced),
        .machine_dmem_fence(machine_dmem_fence),
        .machine_dmem_fence_ack(machine_dmem_fence_ack)
    );

    r4000_top #(
        .SCACHE_LINES(SCACHE_LINES),
        .FULL_ICACHE_INCLUSION(FULL_ICACHE_INCLUSION),
        .ICACHE_LOOKAHEAD(CPU_ICACHE_LOOKAHEAD)
    ) cpu (
        .clk        (cpu_clk),
        .rst_n      (cpu_soc_rst_n),
        .imem_paddr (imem_paddr),
        .imem_ren   (imem_ren),
        .imem_rdata (imem_rdata),
        .dmem_ren   (island_dmem_ren),
        .dmem_wen   (island_dmem_wen),
        .dmem_paddr (island_dmem_paddr),
        .dmem_wmask (island_dmem_wmask),
        .dmem_wdata (island_dmem_wdata),
        .dmem_rdata (island_dmem_rdata),
        .dmem_ready (island_dmem_ready),
        .dmem_uncached_wr(island_dmem_uncached_wr),
        .dmem_cache_owned(island_dmem_cache_owned),
        .mfi_valid    (),
        .mfi_order    (),
        .mfi_pc       (),
        .mfi_insn     (),
        .mfi_trap     (),
        .mfi_rd_addr  (),
        .mfi_rd_wdata (),
        .mfi_hilo_wen (),
        .mfi_hi_wdata (),
        .mfi_lo_wdata (),
        .mfi_mem_addr (),
        .mfi_mem_rmask(),
        .mfi_mem_wmask(),
        .mfi_mem_rdata(),
        .mfi_mem_wdata(),
        .hw_int       (island_cpu_irq),
        .sim_idle_boost(1'b0),
        .force_uncached(island_force_uncached),
        .snoop_valid  (dma_snoop_valid),
        .snoop_addr   (dma_snoop_addr),
        .snoop_wdata  (dma_snoop_wdata),
        .snoop_wmask  (dma_snoop_wmask),
        .snoop_wr_ready(island_snoop_wr_ready),
        .snoop_wr_done(island_snoop_wr_done),
        .external_hold(island_core_hold),
        .external_hold_quiesced(island_core_hold_quiesced),
        .snoop_rd_valid(island_snoop_rd_valid),
        .snoop_rd_addr (island_snoop_rd_addr),
        .snoop_rd_resp_valid(island_snoop_rd_resp_valid),
        .snoop_rd_hit  (island_snoop_rd_hit),
        .snoop_rd_data (island_snoop_rd_data),
        .debug_core_pc      (cpu_debug_core_pc),
        .debug_core_is_pc   (cpu_debug_core_is_pc),
        .debug_core_rf_pc   (cpu_debug_core_rf_pc),
        .debug_core_rf_insn (cpu_debug_core_rf_insn),
        .debug_core_wb_pc   (cpu_debug_core_wb_pc),
        .debug_core_wb_insn (cpu_debug_core_wb_insn),
        .debug_core_epc     (cpu_debug_core_epc),
        .debug_core_cause   (cpu_debug_core_cause),
        .debug_core_badvaddr(cpu_debug_core_badvaddr),
        .debug_core_status  (cpu_debug_core_status),
        .debug_core_context (),
        .debug_core_entryhi (cpu_debug_core_entryhi),
        .debug_core_flags   (cpu_debug_core_flags),
        .debug_core_gpr_v0  (cpu_debug_core_gpr_v0),
        .debug_core_gpr_v1  (cpu_debug_core_gpr_v1),
        .debug_core_gpr_at  (cpu_debug_core_gpr_at),
        .debug_core_gpr_t0  (cpu_debug_core_gpr_t0),
        .debug_core_gpr_t1  (cpu_debug_core_gpr_t1),
        .debug_core_gpr_a0  (cpu_debug_core_gpr_a0),
        .debug_core_gpr_a1  (cpu_debug_core_gpr_a1),
        .debug_core_gpr_a2  (cpu_debug_core_gpr_a2),
        .debug_core_gpr_t8  (cpu_debug_core_gpr_t8),
        .debug_core_gpr_s4  (cpu_debug_core_gpr_s4),
        .debug_core_gpr_s5  (cpu_debug_core_gpr_s5),
        .debug_core_gpr_s6  (cpu_debug_core_gpr_s6),
        .debug_core_gpr_t2  (cpu_debug_core_gpr_t2),
        .debug_core_gpr_t3  (cpu_debug_core_gpr_t3),
        .debug_core_gpr_a3  (cpu_debug_core_gpr_a3),
        .debug_core_gpr_s0  (cpu_debug_core_gpr_s0),
        .debug_core_gpr_s1  (cpu_debug_core_gpr_s1),
        .debug_core_gpr_s2  (cpu_debug_core_gpr_s2),
        .debug_core_gpr_s3  (cpu_debug_core_gpr_s3),
        .debug_core_gpr_t4  (cpu_debug_core_gpr_t4),
        .debug_core_gpr_t6  (cpu_debug_core_gpr_t6),
        .debug_core_gpr_t7  (cpu_debug_core_gpr_t7),
        .debug_core_gpr_t9  (cpu_debug_core_gpr_t9),
        .debug_core_gpr_sp  (cpu_debug_core_gpr_sp),
        .debug_core_gpr_ra  (cpu_debug_core_gpr_ra),
        .debug_core_last_cf_pc     (cpu_debug_core_last_cf_pc),
        .debug_core_last_cf_target (cpu_debug_core_last_cf_target),
        .debug_core_wb_eff_addr(cpu_debug_core_wb_eff_addr),
        .debug_core_wb_load_data(cpu_debug_core_wb_load_data),
        .debug_cache_status (cpu_debug_cache_status),
        .debug_cache_addr   (cpu_debug_cache_addr),
        .debug_scache_probe (),
        .debug_icache_capture(),
        .debug_core_store_commit(cpu_debug_core_store_commit),
        .debug_core_store_pc(cpu_debug_core_store_pc),
        .debug_core_store_insn(cpu_debug_core_store_insn),
        .debug_core_store_paddr(cpu_debug_core_store_paddr),
        .debug_core_store_vaddr(cpu_debug_core_store_vaddr),
        .debug_core_store_wmask(cpu_debug_core_store_wmask),
        .debug_core_store_wdata(cpu_debug_core_store_wdata),
        .debug_dmem_vaddr(),
        .debug_dtlb_state(cpu_debug_dtlb_state),
        .debug_dtlb_paddr(cpu_debug_dtlb_paddr),
        .debug_dtlb_lo(cpu_debug_dtlb_lo),
        .debug_dtlb_mask(cpu_debug_dtlb_mask),
        .debug_tlb_idx  (6'd0),
        .debug_tlb_hi   (),
        .debug_tlb_lo0  (),
        .debug_tlb_lo1  (),
        .debug_tlb_mask (),
        .debug_tlbwr_meta(),
        .debug_tlbwr_hi(),
        .debug_tlbwr_lo0(),
        .debug_tlbwr_lo1(),
        .debug_tlbwr_mask()
    );

    assign cpu_debug_pc = cpu_debug_core_pc;
    assign cpu_debug_mem_addr =
        (island_dmem_ren || island_dmem_wen) ? island_dmem_paddr : imem_paddr;

    wire [16:0] prom_i_addr = imem_paddr[18:2];
    reg  [16:0] prom_d_addr;
    wire [31:0] prom_d_word;

    ip20_prom_rom #(
        .INIT_FILE(PROM_INIT_FILE),
        .INIT_HEX_B00(PROM_INIT_B00),
        .INIT_HEX_B01(PROM_INIT_B01),
        .INIT_HEX_B02(PROM_INIT_B02),
        .INIT_HEX_B03(PROM_INIT_B03),
        .INIT_HEX_B04(PROM_INIT_B04),
        .INIT_HEX_B05(PROM_INIT_B05),
        .INIT_HEX_B06(PROM_INIT_B06),
        .INIT_HEX_B07(PROM_INIT_B07),
        .INIT_HEX_B08(PROM_INIT_B08),
        .INIT_HEX_B09(PROM_INIT_B09),
        .INIT_HEX_B0A(PROM_INIT_B0A),
        .INIT_HEX_B0B(PROM_INIT_B0B),
        .INIT_HEX_B0C(PROM_INIT_B0C),
        .INIT_HEX_B0D(PROM_INIT_B0D),
        .INIT_HEX_B0E(PROM_INIT_B0E),
        .INIT_HEX_B0F(PROM_INIT_B0F)
    ) u_prom (
        .clk_a  (cpu_clk),
        .addr_a (prom_i_addr),
        .q_a    (imem_rdata),
        .clk_b  (clk),
        .addr_b (prom_d_addr),
        .q_b    (prom_d_word)
    );

    wire dmem_active = cpu_dmem_ren || cpu_dmem_wen;

    wire mc_reg_sel    = (cpu_dmem_paddr[31:17] == 15'h0FD0);
    wire ram_sel       = (cpu_dmem_paddr[31:29] == 3'h0) &&
                         (cpu_dmem_paddr[31:24] != 8'h1F);
    wire mc_sel        = mc_reg_sel || ram_sel;
    wire prom_sel      = (cpu_dmem_paddr[31:19] == 13'h3F8);
    wire hpc1_pbus_sel = (cpu_dmem_paddr[31:16] == 16'h1FB8);
    wire int2_sel      = (cpu_dmem_paddr[31:8]  == 24'h1FBD90);
    wire ip20_misc_sel = (cpu_dmem_paddr[31:3] == 29'h03F7_A000);
    // LG1 REX drawing/configuration aperture: 0x1f3f0000-0x1f3f7fff.
    wire lg1_sel       = (cpu_dmem_paddr[31:15] == 17'h3E7E);
    wire mc_cpu_sel    = mc_sel || lg1_sel;
    // Software board probes enumerate both 32 KiB halves of the 0x1f3f slot.
    // Keep the upper half
    // absent so it returns open-bus data, but do not turn that expected
    // nonmatching probe response into a fatal GIO timeout.
    wire lg1_probe_slot_sel = (cpu_dmem_paddr[31:16] == 16'h1F3F);
    // Optional Starter Video occupies the adjacent 0x1f3e slot.  This system
    // has no SVC1 hardware; retain faultless open-bus completion while making
    // the Philips I2C status read report NACK instead of a false device ACK.
    wire starter_video_slot_sel =
        (cpu_dmem_paddr[31:16] == 16'h1F3E);
    // Preserve the retained exact LG1 inventory contract outside the modeled
    // 32 KiB REX subrange. A graphics-option probe reads 0x1f06a07c and
    // recognizes another board only for DEADBEEF; the LG1-compatible path
    // completes the probe with zero. This fallback is a slot-
    // inventory response, not a claim that the LG1 core electrically owns the
    // entire aperture. Keep separately proved upper-LG1 and absent Starter
    // Video responses ahead of it.
    wire graphics_slot_fallback_sel =
        (cpu_dmem_paddr[31:22] == 10'h07C) &&
        !lg1_sel && !lg1_probe_slot_sel && !starter_video_slot_sel;
    wire [63:0] graphics_slot_fallback_rdata = 64'h0;
    wire gr2_sel       = 1'b0;
    wire dsp_sram_sel  = (cpu_dmem_paddr[31:17] == 15'h0FDF);
    wire [4:0]  dsp_reg_addr;
    wire [23:0] dsp_reg_wdata;
    wire [23:0] dsp_reg_rdata;
    wire        dsp_reg_rd_en;
    wire        dsp_reg_wr_en;
    wire        dsp_irq;
    wire [23:0] dsp_sram_word;
    wire        dsp_sram_ready;
    wire [63:0] dsp_sram_rdata;
    wire [255:0] dsp_debug_probe;
    wire [4:0]  dsp_rtc_addr;
    wire [23:0] dsp_rtc_wdata;
    wire [23:0] dsp_rtc_rdata;
    wire        dsp_rtc_rd_en;
    wire        dsp_rtc_wr_en;
    wire [31:0] dsp_dma_mem_addr;
    wire [63:0] dsp_dma_mem_wdata;
    wire [7:0]  dsp_dma_mem_wmask;
    wire        dsp_dma_mem_valid;
    wire        dsp_dma_mem_write;
    wire        dsp_dma_mem_ready;
    wire        dsp_dma_mem_active;

    assign dsp_sram_rdata = cpu_dmem_paddr[2]
        ? {32'h0, 8'h0, dsp_sram_word}
        : {8'h0, dsp_sram_word, 32'h0};

    generate if (HAS_DSP) begin : dsp_gen
        wire [14:0] sram_addr = cpu_dmem_paddr[16:2];
        wire [23:0] sram_wdata = cpu_dmem_paddr[2] ?
            cpu_dmem_wdata[23:0] : cpu_dmem_wdata[55:32];
        wire sram_active = dsp_sram_sel && dmem_active;

        dsp56001_top dsp_inst (
            .clk              (clk),
            .rst_n            (soc_rst_n),
            .cpu_reg_addr     (dsp_reg_addr),
            .cpu_reg_wdata    (dsp_reg_wdata),
            .cpu_reg_rdata    (dsp_reg_rdata),
            .cpu_reg_rd_en    (dsp_reg_rd_en),
            .cpu_reg_wr_en    (dsp_reg_wr_en),
            .cpu_sram_addr    (sram_addr),
            .cpu_sram_wdata   (sram_wdata),
            .cpu_sram_rdata   (dsp_sram_word),
            .cpu_sram_wr_en   (sram_active && cpu_dmem_wen),
            .cpu_sram_rd_en   (sram_active && cpu_dmem_ren),
            .cpu_sram_ready   (dsp_sram_ready),
            .irq_out          (dsp_irq),
            .ssi_sclk_out     (),
            .ssi_fss_out      (),
            .ssi_tx_data      (dsp_audio_sample_data),
            .ssi_tx_en        (dsp_audio_sample_valid),
            .ssi_tx_channel_status(dsp_audio_channel_status),
            .ssi_tx_channel_status_valid(dsp_audio_channel_status_valid),
            .ssi_tx_user_data (dsp_audio_user_data),
            .ssi_tx_user_valid(dsp_audio_user_valid),
            .ssi_tx_active    (dsp_audio_stream_active),
            .ssi_tx_rate_select(dsp_audio_rate_select),
            .ssi_tx_ready     (dsp_audio_sample_ready),
            .ssi_rx_valid     (1'b0),
            .ssi_rx_data      (24'd0),
            .rtc_addr         (dsp_rtc_addr),
            .rtc_wdata        (dsp_rtc_wdata),
            .rtc_rd_en        (dsp_rtc_rd_en),
            .rtc_wr_en        (dsp_rtc_wr_en),
            .rtc_rdata        (dsp_rtc_rdata),
            .dma_mem_addr     (dsp_dma_mem_addr),
            .dma_mem_wdata    (dsp_dma_mem_wdata),
            .dma_mem_rdata    (hpc1_dma_rdata_snooped),
            .dma_mem_wmask    (dsp_dma_mem_wmask),
            .dma_mem_valid    (dsp_dma_mem_valid),
            .dma_mem_write    (dsp_dma_mem_write),
            .dma_mem_ready    (dsp_dma_mem_ready),
            .dma_mem_active   (dsp_dma_mem_active),
            .dbg_mem_wr_en    (1'b0),
            .dbg_mem_space    (2'd0),
            .dbg_mem_addr     (16'd0),
            .dbg_mem_wdata    (24'd0),
            .dbg_reg_wr_en    (1'b0),
            .dbg_reg_wr_addr  (6'd0),
            .dbg_reg_wr_data  (56'd0),
            .dbg_pc           (),
            .dbg_sr           (),
            .dbg_state        (),
            .dbg_busy         (),
            .dbg_cur_inst     (),
            .dbg_acc_a        (),
            .dbg_acc_b        (),
            .debug_probe      (dsp_debug_probe)
        );
    end else begin : dsp_stub_gen
        reg stub_ready;
        always @(posedge clk or negedge soc_rst_n) begin
            if (!soc_rst_n)
                stub_ready <= 1'b0;
            else
                stub_ready <= dsp_sram_sel && dmem_active && !stub_ready;
        end

        assign dsp_reg_rdata = 24'd0;
        assign dsp_irq = 1'b0;
        assign dsp_audio_channel_status = 32'd0;
        assign dsp_audio_channel_status_valid = 1'b0;
        assign dsp_audio_user_data = 32'd0;
        assign dsp_audio_user_valid = 1'b0;
        assign dsp_audio_stream_active = 1'b0;
        assign dsp_audio_rate_select = 6'd0;
        assign dsp_sram_word =
            (cpu_dmem_paddr[16:2] == 15'd0) ? 24'h000001 : 24'h000000;
        assign dsp_sram_ready = stub_ready;
        assign dsp_debug_probe = 256'd0;
        assign dsp_rtc_addr = 5'd0;
        assign dsp_rtc_wdata = 24'd0;
        assign dsp_rtc_rd_en = 1'b0;
        assign dsp_rtc_wr_en = 1'b0;
        assign dsp_dma_mem_addr = 32'd0;
        assign dsp_dma_mem_wdata = 64'd0;
        assign dsp_dma_mem_wmask = 8'd0;
        assign dsp_dma_mem_valid = 1'b0;
        assign dsp_dma_mem_write = 1'b0;
        assign dsp_dma_mem_active = 1'b0;
    end endgenerate
    wire hpc1_region = (cpu_dmem_paddr[31:20] == 12'h1FB);
    wire gio_region = !lg1_probe_slot_sel && !starter_video_slot_sel &&
        !graphics_slot_fallback_sel && (
        cpu_dmem_paddr[31:20] == 12'h1F0 ||
        cpu_dmem_paddr[31:20] == 12'h1F1 ||
        cpu_dmem_paddr[31:20] == 12'h1F2 ||
        cpu_dmem_paddr[31:20] == 12'h1F3 ||
        cpu_dmem_paddr[31:20] == 12'h1F4 ||
        cpu_dmem_paddr[31:20] == 12'h1F5 ||
        cpu_dmem_paddr[31:20] == 12'h1F6 ||
        cpu_dmem_paddr[31:20] == 12'h1F7 ||
        cpu_dmem_paddr[31:20] == 12'h1F8 ||
        cpu_dmem_paddr[31:20] == 12'h1F9);
    wire hpc1_unhandled = hpc1_region && !hpc1_pbus_sel && !int2_sel &&
                           !ip20_misc_sel && !dsp_sram_sel;

    // Only a transaction with no modeled GIO/HPC responder reaches the MC's
    // timeout boundary. CPUCTRL1.ABORT_EN is owned by MC, not inferred from PC
    // history. Faultless inventory responses above are excluded by decode.
    wire mc_gio_error_access =
        (gio_region || hpc1_unhandled) && dmem_active;
    wire mc_gio_abort_en;

    localparam [2:0] PROM_IDLE        = 3'd0;
    localparam [2:0] PROM_WAIT0_SETUP = 3'd1;
    localparam [2:0] PROM_WAIT0       = 3'd2;
    localparam [2:0] PROM_WAIT1_SETUP = 3'd3;
    localparam [2:0] PROM_WAIT1       = 3'd4;
    localparam [2:0] PROM_DONE        = 3'd5;

    reg [2:0]  prom_state;
    reg [31:0] prom_hi;
    reg [63:0] prom_rdata;
    reg        prom_ready;
    wire       prom_req = prom_sel && dmem_active;
    wire [16:0] prom_even_idx = {cpu_dmem_paddr[18:3], 1'b0};
    wire [16:0] prom_odd_idx  = {cpu_dmem_paddr[18:3], 1'b1};

    always @(posedge clk or negedge soc_rst_n) begin
        if (!soc_rst_n) begin
            prom_state <= PROM_IDLE;
            prom_d_addr <= 17'd0;
            prom_hi <= 32'd0;
            prom_rdata <= 64'd0;
            prom_ready <= 1'b0;
        end else begin
            prom_ready <= 1'b0;
            case (prom_state)
                PROM_IDLE: begin
                    if (prom_req && cpu_dmem_wen) begin
                        prom_ready <= 1'b1;
                        prom_state <= PROM_DONE;
                    end else if (prom_req) begin
                        prom_d_addr <= prom_even_idx;
                        prom_state <= PROM_FULL_CYCLE_READ ?
                                      PROM_WAIT0_SETUP : PROM_WAIT0;
                    end
                end
                PROM_WAIT0_SETUP: prom_state <= PROM_WAIT0;
                PROM_WAIT0: begin
                    prom_hi <= prom_d_word;
                    prom_d_addr <= prom_odd_idx;
                    prom_state <= PROM_FULL_CYCLE_READ ?
                                  PROM_WAIT1_SETUP : PROM_WAIT1;
                end
                PROM_WAIT1_SETUP: prom_state <= PROM_WAIT1;
                PROM_WAIT1: begin
                    prom_rdata <= {prom_hi, prom_d_word};
                    prom_ready <= 1'b1;
                    prom_state <= PROM_DONE;
                end
                PROM_DONE: begin
                    prom_ready <= prom_req;
                    if (!prom_req)
                        prom_state <= PROM_IDLE;
                end
                default: prom_state <= PROM_IDLE;
            endcase
        end
    end

    wire [63:0] mc_rdata;
    wire        mc_ready;
    wire [31:0] mc_memcfg0;
    wire [479:0] mc_debug_probe_unused;
    assign hpc_dma_read_accept = hpc1_dma_valid && hpc1_dma_ready &&
                                 !hpc_dma_slot_write;
    assign mc_snoop_rd_addr = hpc_dma_slot_addr;

    cyan_hpc_dma_coordinator u_hpc_dma_coordinator (
        .clk(clk),
        .rst_n(soc_rst_n),
        .hpc_addr(hpc1_dma_addr),
        .hpc_wdata(hpc1_dma_wdata),
        .hpc_wmask(hpc1_dma_wmask),
        .hpc_valid(hpc1_dma_valid),
        .hpc_write(hpc1_dma_write),
        .hpc_rdata(hpc1_dma_rdata_snooped),
        .hpc_ready(hpc1_dma_ready),
        .txn_addr(hpc_dma_slot_addr),
        .txn_wdata(hpc_dma_slot_wdata),
        .txn_wmask(hpc_dma_slot_wmask),
        .txn_write(hpc_dma_slot_write),
        .mc_request(mc_hpc_dma_request),
        .mc_valid(mc_hpc_dma_valid),
        .mc_write(mc_hpc_dma_write),
        .mc_rdata(hpc1_dma_rdata_raw),
        .mc_ready(mc_hpc_dma_ready),
        .core_hold(machine_core_hold),
        .core_hold_ack(machine_core_hold_ack),
        .dmem_fence(machine_dmem_fence),
        .dmem_fence_ack(machine_dmem_fence_ack),
        .snoop_wr_launch(machine_dma_snoop_valid),
        .snoop_wr_ready(machine_dma_snoop_ready),
        .snoop_wr_complete(machine_dma_snoop_complete),
        .snoop_rd_launch(mc_snoop_rd_valid),
        .snoop_rd_ready(machine_snoop_rd_ready),
        .snoop_rd_resp_valid(mc_snoop_rd_resp_valid),
        .snoop_rd_resp_hit(mc_snoop_rd_hit),
        .snoop_rd_resp_data(mc_snoop_rd_data),
        .debug_state(hpc_dma_state),
        .debug_probe_hit(hpc_dma_probe_hit),
        .debug_probe_data(hpc_dma_probe_data)
    );
    wire [95:0] sd_scsi_cdb;
    wire [23:0] sd_scsi_data_index;

    wire [31:0] vdma_gio_addr;
    wire [63:0] vdma_gio_wdata;
    wire [ 7:0] vdma_gio_wmask;
    wire        vdma_gio_valid;
    wire        vdma_gio_write;
    wire        vdma_gio_source_vdma;
    wire [63:0] vdma_gio_rdata;
    wire        vdma_gio_ready;


`ifdef KCU105_CYAN_SOC_DEBUG
    reg [15:0] dma_snoop_rd_count;
    reg [15:0] dma_snoop_rd_hit_count;
    reg [31:0] dma_snoop_rd_last_accept_addr;
    reg [31:0] dma_snoop_rd_last_hit_addr;
    reg [63:0] dma_snoop_rd_last_raw_data;
    reg [63:0] dma_snoop_rd_last_hit_data;
    reg [31:0] dma_snoop_wr_count;
    reg [31:0] dma_snoop_wr_last_addr;
    reg [63:0] dma_snoop_wr_last_data;
    reg [ 7:0] dma_snoop_wr_last_mask;
    reg [ 7:0] inode1856_dma_beat_count;
    reg [23:0] inode1856_dma_first_index;
    reg [31:0] inode1856_dma_first_addr;
    reg [63:0] inode1856_dma_beat0;
    reg [63:0] inode1856_dma_beat1;
    reg [7:0]  volume_dma_beat_count;
    reg [23:0] volume_dma_first_index;
    reg [31:0] volume_dma_first_addr;
    reg [7:0]  volume_dma_first_mask;
    reg [63:0] volume_dma_beat0;
    reg [63:0] volume_dma_beat1;
    reg [63:0] volume_dma_beat2;
    reg [63:0] volume_dma_beat3;
    reg [7:0]  efs_dma_beat_count;
    reg [23:0] efs_dma_first_index;
    reg [31:0] efs_dma_first_addr;
    reg [7:0]  efs_dma_first_mask;
    reg [63:0] efs_dma_beat0;
    reg [63:0] efs_dma_beat1;
    reg [63:0] efs_dma_beat2;
    reg [63:0] efs_dma_beat3;

    wire [7:0] sd_scsi_cdb_opcode = sd_scsi_cdb[95:88];
    wire sd_scsi_cdb_is_read = (sd_scsi_cdb_opcode == 8'h08) ||
                               (sd_scsi_cdb_opcode == 8'h28);
    wire [31:0] sd_scsi_cdb_base_lba =
        (sd_scsi_cdb_opcode == 8'h08) ?
        {11'd0, sd_scsi_cdb[84:80], sd_scsi_cdb[79:72],
         sd_scsi_cdb[71:64]} :
        {sd_scsi_cdb[79:72], sd_scsi_cdb[71:64],
         sd_scsi_cdb[63:56], sd_scsi_cdb[55:48]};
    wire [31:0] sd_scsi_dma_stream_lba =
        sd_scsi_cdb_base_lba + {17'd0, sd_scsi_data_index[23:9]};
    wire inode1856_dma_accept = machine_dma_snoop_valid &&
                                sd_scsi_cdb_is_read &&
                                (sd_scsi_dma_stream_lba == 32'h0000_8133);
    wire volume_dma_accept = machine_dma_snoop_valid &&
                             sd_scsi_cdb_is_read &&
                             (sd_scsi_dma_stream_lba == 32'h0000_0000);
    wire efs_dma_accept = machine_dma_snoop_valid &&
                          sd_scsi_cdb_is_read &&
                          (sd_scsi_dma_stream_lba == 32'h0000_7d83);

    always @(posedge clk or negedge soc_rst_n) begin
        if (!soc_rst_n) begin
            dma_snoop_rd_count <= 16'd0;
            dma_snoop_rd_hit_count <= 16'd0;
            dma_snoop_rd_last_accept_addr <= 32'd0;
            dma_snoop_rd_last_hit_addr <= 32'd0;
            dma_snoop_rd_last_raw_data <= 64'd0;
            dma_snoop_rd_last_hit_data <= 64'd0;
            dma_snoop_wr_count <= 32'd0;
            dma_snoop_wr_last_addr <= 32'd0;
            dma_snoop_wr_last_data <= 64'd0;
            dma_snoop_wr_last_mask <= 8'd0;
            inode1856_dma_beat_count <= 8'd0;
            inode1856_dma_first_index <= 24'd0;
            inode1856_dma_first_addr <= 32'd0;
            inode1856_dma_beat0 <= 64'd0;
            inode1856_dma_beat1 <= 64'd0;
            volume_dma_beat_count <= 8'd0;
            volume_dma_first_index <= 24'd0;
            volume_dma_first_addr <= 32'd0;
            volume_dma_first_mask <= 8'd0;
            volume_dma_beat0 <= 64'd0;
            volume_dma_beat1 <= 64'd0;
            volume_dma_beat2 <= 64'd0;
            volume_dma_beat3 <= 64'd0;
            efs_dma_beat_count <= 8'd0;
            efs_dma_first_index <= 24'd0;
            efs_dma_first_addr <= 32'd0;
            efs_dma_first_mask <= 8'd0;
            efs_dma_beat0 <= 64'd0;
            efs_dma_beat1 <= 64'd0;
            efs_dma_beat2 <= 64'd0;
            efs_dma_beat3 <= 64'd0;
        end else begin
            if (hpc_dma_read_accept) begin
                dma_snoop_rd_count <= dma_snoop_rd_count + 16'd1;
                dma_snoop_rd_last_accept_addr <= hpc_dma_slot_addr & ~32'h7;
                dma_snoop_rd_last_raw_data <= hpc1_dma_rdata_raw;
                if (hpc_dma_probe_hit) begin
                    dma_snoop_rd_hit_count <= dma_snoop_rd_hit_count + 16'd1;
                    dma_snoop_rd_last_hit_addr <= hpc_dma_slot_addr & ~32'h7;
                    dma_snoop_rd_last_hit_data <= hpc_dma_probe_data;
                end
            end
            if (machine_dma_snoop_valid) begin
                dma_snoop_wr_count <= dma_snoop_wr_count + 32'd1;
                dma_snoop_wr_last_addr <= hpc_dma_slot_addr & ~32'h7;
                dma_snoop_wr_last_data <= hpc_dma_slot_wdata;
                dma_snoop_wr_last_mask <= hpc_dma_slot_wmask;
            end
            if (inode1856_dma_accept) begin
                if (inode1856_dma_beat_count == 8'd0) begin
                    inode1856_dma_first_index <= sd_scsi_data_index;
                    inode1856_dma_first_addr <= hpc_dma_slot_addr & ~32'h7;
                    inode1856_dma_beat0 <= hpc_dma_slot_wdata;
                end else if (inode1856_dma_beat_count == 8'd1) begin
                    inode1856_dma_beat1 <= hpc_dma_slot_wdata;
                end
                if (inode1856_dma_beat_count != 8'hff)
                    inode1856_dma_beat_count <=
                        inode1856_dma_beat_count + 8'd1;
            end
            if (volume_dma_accept) begin
                if (volume_dma_beat_count == 8'd0) begin
                    volume_dma_first_index <= sd_scsi_data_index;
                    volume_dma_first_addr <= hpc_dma_slot_addr & ~32'h7;
                    volume_dma_first_mask <= hpc_dma_slot_wmask;
                    volume_dma_beat0 <= hpc_dma_slot_wdata;
                end else if (volume_dma_beat_count == 8'd1) begin
                    volume_dma_beat1 <= hpc_dma_slot_wdata;
                end else if (volume_dma_beat_count == 8'd2) begin
                    volume_dma_beat2 <= hpc_dma_slot_wdata;
                end else if (volume_dma_beat_count == 8'd3) begin
                    volume_dma_beat3 <= hpc_dma_slot_wdata;
                end
                if (volume_dma_beat_count != 8'hff)
                    volume_dma_beat_count <= volume_dma_beat_count + 8'd1;
            end
            if (efs_dma_accept) begin
                if (efs_dma_beat_count == 8'd0) begin
                    efs_dma_first_index <= sd_scsi_data_index;
                    efs_dma_first_addr <= hpc_dma_slot_addr & ~32'h7;
                    efs_dma_first_mask <= hpc_dma_slot_wmask;
                    efs_dma_beat0 <= hpc_dma_slot_wdata;
                end else if (efs_dma_beat_count == 8'd1) begin
                    efs_dma_beat1 <= hpc_dma_slot_wdata;
                end else if (efs_dma_beat_count == 8'd2) begin
                    efs_dma_beat2 <= hpc_dma_slot_wdata;
                end else if (efs_dma_beat_count == 8'd3) begin
                    efs_dma_beat3 <= hpc_dma_slot_wdata;
                end
                if (efs_dma_beat_count != 8'hff)
                    efs_dma_beat_count <= efs_dma_beat_count + 8'd1;
            end
        end
    end
`endif

    wire mc_bus_err_irq;
    wire mc_gfx_dma_irq;
    wire mc_eeprom_cs;
    wire mc_eeprom_clk;
    wire mc_eeprom_di;

    mc #(
        .RAM_BYTES(128 * 1024 * 1024)
    ) mc_inst (
        .clk        (clk),
        .rst_n      (soc_rst_n),
        .cpu_addr   ({4'b0, cpu_dmem_paddr}),
        .cpu_wdata  (cpu_dmem_wdata),
        .cpu_rdata  (mc_rdata),
        .cpu_wmask  (cpu_dmem_wmask),
        .cpu_valid  (mc_cpu_sel && dmem_active),
        .cpu_write  (cpu_dmem_wen),
        .cpu_cache_owned(cpu_dmem_cache_owned),
        .cpu_ready  (mc_ready),
        .dma_addr   (hpc_dma_slot_addr),
        .dma_wdata  (hpc_dma_slot_wdata),
        .dma_rdata  (hpc1_dma_rdata_raw),
        .dma_wmask  (hpc_dma_slot_wmask),
        .dma_request(mc_hpc_dma_request),
        .dma_valid  (mc_hpc_dma_valid),
        .dma_write  (mc_hpc_dma_write),
        .dma_ready  (mc_hpc_dma_ready),
        .eeprom_cs  (mc_eeprom_cs),
        .eeprom_clk (mc_eeprom_clk),
        .eeprom_di  (mc_eeprom_di),
        .eeprom_do  (eeprom_do),
        .vdma_gio_addr  (vdma_gio_addr),
        .vdma_gio_wdata (vdma_gio_wdata),
        .vdma_gio_rdata (vdma_gio_rdata),
        .vdma_gio_wmask (vdma_gio_wmask),
        .vdma_gio_valid (vdma_gio_valid),
        .vdma_gio_write (vdma_gio_write),
        .vdma_gio_source_vdma(vdma_gio_source_vdma),
        .vdma_gio_ready (vdma_gio_ready),
        .ram_addr   (ram_addr),
        .ram_wdata  (ram_wdata),
        .ram_wmask  (ram_wmask),
        .ram_wen    (ram_wen),
        .ram_valid  (ram_valid),
        .ram_ready  (ram_ready),
        .ram_rdata  (ram_rdata),
        .gio_access (mc_gio_error_access),
        .gio_abort_en(mc_gio_abort_en),
        .memcfg0_out(mc_memcfg0),
        .bus_err_irq(mc_bus_err_irq),
        .gfx_dma_irq(mc_gfx_dma_irq),
        .debug_probe(mc_debug_probe_unused)
    );

    wire        eeprom_cs;
    wire        eeprom_clk;
    wire        eeprom_di;
    wire        eeprom_do;
    wire        hpc1_eeprom_cs;
    wire        hpc1_eeprom_clk;
    wire        hpc1_eeprom_di;
    wire [63:0] hpc1_rdata;
    wire        hpc1_ready;
    wire        console_led;
    wire        scc_txd_b;
    wire        dma_active;
    wire [255:0] enet_hpc_debug_probe;
    wire [767:0] enet_hpc_debug_snapshot;
    wire [255:0] scsi_hpc_debug_probe_unused;
    wire [255:0] sd_scsi_debug_probe;
    wire [31:0] inode1856_write_status;
    wire [127:0] scc_hpc_debug_probe;
    wire        hpc1_enet_irq;
    wire        hpc1_timer_irq_0;
    wire        hpc1_timer_irq_1;
    wire        hpc1_scsi0_irq;
    wire [5:0]  hpc1_cpu_irq;
    wire        lg1_irq;
    wire [7:0]  scc_console_tx_data;
    wire        scc_console_tx_valid;
    wire        sd_scsi_cmd_pending;
    wire        sd_scsi_cmd_done;
    wire [7:0]  sd_scsi_cmd_status;
    wire [23:0] sd_scsi_resp_len;
    wire        sd_scsi_status_valid;
    wire [2:0]  sd_scsi_target_id;
    wire [23:0] sd_scsi_xfer_count;
    wire        sd_scsi_bus_reset;
    wire [7:0]  sd_scsi_read_data;
    wire        sd_scsi_read_valid;
    wire        sd_scsi_read_consume;
    wire        sd_scsi_write_valid;
    wire [7:0]  sd_scsi_write_data;
    wire        sd_scsi_write_ready;
    wire        sd_scsi_data_out_done;
    reg         debug_rx_inject_toggle_d;
    wire        debug_rx_inject_valid;

    always @(posedge clk) begin
        if (!rst_n)
            debug_rx_inject_toggle_d <= debug_rx_inject_toggle;
        else
            debug_rx_inject_toggle_d <= debug_rx_inject_toggle;
    end

    assign debug_rx_inject_valid = rst_n &&
                                   (debug_rx_inject_toggle != debug_rx_inject_toggle_d);

    sd_scsi_bridge #(
        // 12.5 MHz SPI at the 50 MHz KCU SoC clock after SD init. The 25 MHz
        // setting caused corrupted sector reads on the KCU105 SD path.
        .RUN_CLK_DIV(16'd2)
    ) u_sd_scsi (
        .clk(clk),
        .rst_n(soc_rst_n),
        .scsi_bus_reset(sd_scsi_bus_reset),
        .scsi_cmd_pending(sd_scsi_cmd_pending),
        .scsi_cmd_done(sd_scsi_cmd_done),
        .scsi_cmd_status(sd_scsi_cmd_status),
        .scsi_resp_len(sd_scsi_resp_len),
        .scsi_status_valid(sd_scsi_status_valid),
        .scsi_target_id(sd_scsi_target_id),
        .scsi_cdb(sd_scsi_cdb),
        .scsi_xfer_count(sd_scsi_xfer_count),
        .scsi_data_index(sd_scsi_data_index),
        .scsi_read_data(sd_scsi_read_data),
        .scsi_read_valid(sd_scsi_read_valid),
        .scsi_read_consume(sd_scsi_read_consume),
        .scsi_write_valid(sd_scsi_write_valid),
        .scsi_write_data(sd_scsi_write_data),
        .scsi_write_ready(sd_scsi_write_ready),
        .scsi_data_out_done(sd_scsi_data_out_done),
        .sd_cs_n(sd_cs_n),
        .sd_clk(sd_clk),
        .sd_mosi(sd_mosi),
        .sd_miso(sd_miso),
        .sd_card_detect(sd_card_detect),
        .read_skip_first_byte(sd_read_skip_first_byte),
        .debug_read_trigger(sd_debug_read_trigger),
        .debug_lba_select(sd_debug_lba_select),
        .debug_probe_select(sd_debug_probe_select),
        .debug_inode1856_write_status(inode1856_write_status),
        .debug_probe(sd_scsi_debug_probe)
    );

    wire [7:0] kbd_tx_data;
    wire       kbd_tx_valid;
    wire [7:0] mouse_tx_data;
    wire       mouse_tx_valid;
    wire [7:0] kbd_rx_data;
    wire       kbd_rx_valid;
    wire [7:0] mouse_rx_data;
    wire       mouse_rx_valid;

    cyan_sgi_input_device #(
        .CLK_HZ(CLK_HZ),
        .FIFO_DEPTH(64)
    ) sgi_input_inst (
        .clk(clk),
        .rst_n(rst_n),
        .guest_active(soc_rst_n),
        .host_kbd_data(host_kbd_data),
        .host_kbd_valid(host_kbd_valid),
        .host_kbd_ready(host_kbd_ready),
        .host_mouse_data(host_mouse_data),
        .host_mouse_valid(host_mouse_valid),
        .host_mouse_ready(host_mouse_ready),
        .kbd_tx_data(kbd_tx_data),
        .kbd_tx_valid(kbd_tx_valid),
        .mouse_tx_data(mouse_tx_data),
        .mouse_tx_valid(mouse_tx_valid),
        .kbd_rx_data(kbd_rx_data),
        .kbd_rx_valid(kbd_rx_valid),
        .mouse_rx_data(mouse_rx_data),
        .mouse_rx_valid(mouse_rx_valid),
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
        .mouse_tx_event_overflow(mouse_tx_event_overflow)
    );

    hpc1 #(
        .HAS_DSP(HAS_DSP),
        .SCSI_EXTERNAL_TARGET(1),
        .FORCE_SCSI_IRQ_MASK(1),
        .LATCH_SCSI_IRQ_STATUS(0),
        .CLK_HZ(CLK_HZ)
    ) hpc1_inst (
        .clk        (clk),
        .rst_n      (soc_rst_n),
        .addr       (cpu_dmem_paddr[11:0]),
        .wdata      (cpu_dmem_wdata),
        .rdata      (hpc1_rdata),
        .wmask      (cpu_dmem_wmask),
        .valid      (hpc1_pbus_sel && dmem_active),
        .write      (cpu_dmem_wen),
        .ready      (hpc1_ready),
        .scc_txd_a  (),
        .scc_txd_b  (scc_txd_b),
        .scc_rxd_a  (1'b1),
        .scc_rxd_b  (uart_rxd),
        .eeprom_cs  (hpc1_eeprom_cs),
        .eeprom_clk (hpc1_eeprom_clk),
        .eeprom_di  (hpc1_eeprom_di),
        .eeprom_do  (eeprom_do),
        .console_led(console_led),
        .scc_console_tx_data(scc_console_tx_data),
        .scc_console_tx_valid(scc_console_tx_valid),
        .rx_inject_data  (debug_rx_inject_data),
        .rx_inject_valid (debug_rx_inject_valid),
        .kbd_tx_data  (kbd_tx_data),
        .kbd_tx_valid (kbd_tx_valid),
        .kbd_rx_data  (kbd_rx_data),
        .kbd_rx_valid (kbd_rx_valid),
        .mouse_tx_data  (mouse_tx_data),
        .mouse_tx_valid (mouse_tx_valid),
        .mouse_rx_data  (mouse_rx_data),
        .mouse_rx_valid (mouse_rx_valid),
        .scc0_access_event_data(scc0_access_event_data),
        .scc0_access_event_valid(scc0_access_event_valid),
        .scc_debug_probe(scc_hpc_debug_probe),
        .scsi_target_present (8'h02),
        .scsi_cmd_pending    (sd_scsi_cmd_pending),
        .scsi_cmd_done       (sd_scsi_cmd_done),
        .scsi_cmd_status     (sd_scsi_cmd_status),
        .scsi_resp_len       (sd_scsi_resp_len),
        .scsi_status_valid   (sd_scsi_status_valid),
        .scsi_target_id      (sd_scsi_target_id),
        .scsi_cdb            (sd_scsi_cdb),
        .scsi_xfer_count     (sd_scsi_xfer_count),
        .scsi_data_index     (sd_scsi_data_index),
        .scsi_bus_reset      (sd_scsi_bus_reset),
        .scsi_read_data      (sd_scsi_read_data),
        .scsi_read_valid     (sd_scsi_read_valid),
        .scsi_read_consume   (sd_scsi_read_consume),
        .scsi_write_valid    (sd_scsi_write_valid),
        .scsi_write_data     (sd_scsi_write_data),
        .scsi_write_ready    (sd_scsi_write_ready),
        .scsi_data_out_done  (sd_scsi_data_out_done),
        .dsp_reg_addr        (dsp_reg_addr),
        .dsp_reg_wdata       (dsp_reg_wdata),
        .dsp_reg_rdata       (dsp_reg_rdata),
        .dsp_reg_rd_en       (dsp_reg_rd_en),
        .dsp_reg_wr_en       (dsp_reg_wr_en),
        .dsp_irq             (dsp_irq),
        .dsp_rtc_addr        (dsp_rtc_addr),
        .dsp_rtc_wdata       (dsp_rtc_wdata),
        .dsp_rtc_rdata       (dsp_rtc_rdata),
        .dsp_rtc_rd_en       (dsp_rtc_rd_en),
        .dsp_rtc_wr_en       (dsp_rtc_wr_en),
        .dsp_dma_mem_addr    (dsp_dma_mem_addr),
        .dsp_dma_mem_wdata   (dsp_dma_mem_wdata),
        .dsp_dma_mem_wmask   (dsp_dma_mem_wmask),
        .dsp_dma_mem_valid   (dsp_dma_mem_valid),
        .dsp_dma_mem_write   (dsp_dma_mem_write),
        .dsp_dma_mem_ready   (dsp_dma_mem_ready),
        .dsp_dma_active      (dsp_dma_mem_active),
        .gfx_retrace_irq     (lg1_irq),
        .gfx_retrace_status  (gfx_retrace_status),
        .gfx_dma_irq         (mc_gfx_dma_irq),
        .dma_mem_addr        (hpc1_dma_addr),
        .dma_mem_wdata       (hpc1_dma_wdata),
        .dma_mem_rdata       (hpc1_dma_rdata_snooped),
        .dma_mem_wmask       (hpc1_dma_wmask),
        .dma_mem_valid       (hpc1_dma_valid),
        .dma_mem_write       (hpc1_dma_write),
        .dma_mem_ready       (hpc1_dma_ready),
        .dma_active          (dma_active),
        .enet_tx_data        (enet_tx_data),
        .enet_tx_valid       (enet_tx_valid),
        .enet_tx_sof         (enet_tx_sof),
        .enet_tx_eof         (enet_tx_eof),
        .enet_tx_ready       (enet_tx_ready),
        .enet_rx_data        (enet_rx_data),
        .enet_rx_valid       (enet_rx_valid),
        .enet_rx_sof         (enet_rx_sof),
        .enet_rx_eof         (enet_rx_eof),
        .enet_rx_ready       (enet_rx_ready),
        .enet_irq            (hpc1_enet_irq),
        .enet_debug_probe    (enet_hpc_debug_probe),
        .enet_debug_snapshot (enet_hpc_debug_snapshot),
        .scsi_debug_probe    (scsi_hpc_debug_probe_unused),
        .timer_irq_0         (hpc1_timer_irq_0),
        .timer_irq_1         (hpc1_timer_irq_1),
        .scsi0_irq           (hpc1_scsi0_irq),
        .cpu_irq             (hpc1_cpu_irq),
        .ide_mode            (1'b0)
    );

    assign uart_txd = scc_txd_b;
    assign eeprom_cs = mc_eeprom_cs || hpc1_eeprom_cs;
    assign eeprom_clk = mc_eeprom_cs ? mc_eeprom_clk : hpc1_eeprom_clk;
    assign eeprom_di = mc_eeprom_cs ? mc_eeprom_di : hpc1_eeprom_di;

    eeprom_93cs56 eeprom_inst (
        .clk       (clk),
        .rst_n     (soc_rst_n),
        .cs        (eeprom_cs),
        .sk        (eeprom_clk),
        .di        (eeprom_di),
        .do_out    (eeprom_do),
        .init_we   (eeprom_init_we),
        .init_addr (eeprom_init_addr),
        .init_data (eeprom_init_mem[eeprom_init_addr])
    );

    wire mc_eerom_reg_access = mc_sel && dmem_active && cpu_dmem_ready &&
                               (cpu_dmem_paddr[15:0] == 16'h0030);
    wire mc_eerom_reg_read = mc_eerom_reg_access && cpu_dmem_ren;
    wire mc_eerom_reg_write = mc_eerom_reg_access && cpu_dmem_wen;

    reg        mc_eeprom_clk_d;
    reg [15:0] mc_eeprom_rise_count;
    reg [15:0] mc_eerom_read_count;
    reg [15:0] mc_eeprom_do_shift;
    reg [15:0] mc_eeprom_di_shift;
    reg [31:0] mc_eerom_last_rdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mc_eeprom_clk_d <= 1'b0;
            mc_eeprom_rise_count <= 16'd0;
            mc_eerom_read_count <= 16'd0;
            mc_eeprom_do_shift <= 16'd0;
            mc_eeprom_di_shift <= 16'd0;
            mc_eerom_last_rdata <= 32'd0;
        end else begin
            mc_eeprom_clk_d <= mc_eeprom_clk;

            if (mc_eeprom_cs && !mc_eeprom_clk_d && mc_eeprom_clk) begin
                mc_eeprom_rise_count <= mc_eeprom_rise_count + 16'd1;
                mc_eeprom_do_shift <= {mc_eeprom_do_shift[14:0], eeprom_do};
                mc_eeprom_di_shift <= {mc_eeprom_di_shift[14:0], mc_eeprom_di};
            end

            if (mc_eerom_reg_read) begin
                mc_eerom_read_count <= mc_eerom_read_count + 16'd1;
                mc_eerom_last_rdata <= mc_rdata[31:0];
            end
        end
    end

    wire [63:0] ip20_misc_rdata;
    wire        ip20_misc_ready;
    ip20_misc ip20_misc_inst (
        .clk        (clk),
        .rst_n      (soc_rst_n),
        .addr       (cpu_dmem_paddr),
        .wdata      (cpu_dmem_wdata),
        .rdata      (ip20_misc_rdata),
        .wmask      (cpu_dmem_wmask),
        .valid      (ip20_misc_sel && dmem_active),
        .write      (cpu_dmem_wen),
        .ready      (ip20_misc_ready)
    );

    // The MC is the sole runtime GIO master. CPU LG1 PIO and virtual DMA have
    // already been ordered/arbitrated there; this boundary only selects the
    // external BIST/capture owners around that unified production path.
    wire [63:0] lg1_rdata;
    wire        lg1_ready;
    // Historical signal name retained for trace/test compatibility; this is
    // now the unified MC GIO stream and may be owned by CPU or VDMA.
    wire        lg1_vdma_sel = vdma_gio_valid &&
                               (vdma_gio_addr[31:15] == 17'h3E7E);
    wire        lg1_bus_valid;
    wire [14:0] lg1_bus_addr;
    wire [63:0] lg1_bus_wdata;
    wire [ 7:0] lg1_bus_wmask;
    wire        lg1_bus_write;

    wire        lg1_bist_bus_valid;
    wire [14:0] lg1_bist_bus_addr;
    wire [63:0] lg1_bist_bus_wdata;
    wire [ 7:0] lg1_bist_bus_wmask;
    wire        lg1_bist_flush;
    wire        lg1_bist_active;
    wire        lg1_bist_done_toggle;

    assign lg1_bus_valid = lg1_bist_active ? lg1_bist_bus_valid :
                           (lg1_capture_hold ? 1'b0 :
                            lg1_vdma_sel);
    assign lg1_bus_addr = lg1_bist_active ? lg1_bist_bus_addr :
                          vdma_gio_addr[14:0];
    assign lg1_bus_wdata = lg1_bist_active ? lg1_bist_bus_wdata :
                           vdma_gio_wdata;
    assign lg1_bus_wmask = lg1_bist_active ? lg1_bist_bus_wmask :
                           vdma_gio_wmask;
    assign lg1_bus_write = lg1_bist_active ? 1'b1 :
                           vdma_gio_write;
    assign vdma_gio_rdata = lg1_vdma_sel ? lg1_rdata :
                             {vdma_gio_addr, vdma_gio_addr};
    assign vdma_gio_ready = lg1_vdma_sel ?
                             (!lg1_bist_active && !lg1_capture_hold &&
                              lg1_ready) : vdma_gio_valid;

    // Observe the accepted public LG1 bus without feeding any signal back into
    // arbitration or LG1. The downstream passive ring selects its bounded
    // capture mode. The CPU debug store-PC signal is the core's data-side
    // memory PC for both loads and stores.
    wire [1:0] lg1_config_access_source = lg1_bist_active ? 2'd2 :
                                           (vdma_gio_source_vdma ? 2'd1 : 2'd0);
    cyan_lg1_config_event_encoder u_lg1_config_event_encoder (
        .clk(clk),
        .rst_n(soc_rst_n),
        .accepted(lg1_bus_valid && lg1_ready),
        .addr(lg1_bus_addr),
        .wdata(lg1_bus_wdata),
        .wmask(lg1_bus_wmask),
        .rdata(lg1_rdata),
        .write(lg1_bus_write),
        .source(lg1_config_access_source),
        .cpu_dmem_pc(cpu_dmem_pc),
        .access_valid(lg1_config_access_valid),
        .access_payload(lg1_config_access_payload),
        .current_configsel(lg1_config_current_sel)
    );

    wire        lg1_flush_done;
    wire        lg1_cache_dirty;
    wire        lg1_busy;
    wire        lg1_video_de;
    wire        lg1_video_hsync;
    wire        lg1_video_vsync;
    wire [ 7:0] lg1_video_r;
    wire [ 7:0] lg1_video_g;
    wire [ 7:0] lg1_video_b;
    wire        lg1_video_valid;
    wire        lg1_video_frame_start;
    wire [ 9:0] lg1_video_x;
    wire [ 9:0] lg1_video_y;
    wire [ 4:0] lg1_video_did;
    wire [ 7:0] lg1_video_raw_index;
    wire [ 7:0] lg1_video_index;
    wire [15:0] lg1_video_xmap_mode;
    wire [ 1:0] lg1_video_palette_bank;
    wire [ 2:0] lg1_video_resolver_class;
    wire [91:0] lg1_resolver_telemetry;
    wire [91:0] lg1_resolver_detail;
    wire        lg1_resolver_frame_done;
    wire [15:0] lg1_front_response_low_signature;
    wire [15:0] lg1_front_response_high_signature;
    wire [ 7:0] lg1_front_response_signature_generation;
    wire        lg1_front_response_signature_valid;
    wire        lg1_scanout_underrun;
    wire [31:0] lg1_scanout_underrun_count;
    wire [31:0] lg1_scanout_protocol_error_count;

    // Export the resolved LG1 scanout.  These are the pixels generated by
    // firmware and software writes to the device model, not a board pattern.
    assign video_de = lg1_video_de;
    assign video_hsync = lg1_video_hsync;
    assign video_vsync = lg1_video_vsync;
    assign video_r = lg1_video_r;
    assign video_g = lg1_video_g;
    assign video_b = lg1_video_b;
    assign video_valid = lg1_video_valid;
    assign video_frame_start = lg1_video_frame_start;
    assign video_x = lg1_video_x;
    assign video_y = lg1_video_y;
    assign video_raw_index = lg1_video_raw_index;
    assign video_did = lg1_video_did;
    assign video_display_index = lg1_video_index;
    assign video_xmap_mode = lg1_video_xmap_mode;
    assign video_palette_bank = lg1_video_palette_bank;
    assign video_resolver_class = lg1_video_resolver_class;
    assign video_resolver_telemetry = lg1_resolver_telemetry;
    assign video_resolver_detail = lg1_resolver_detail;
    assign video_resolver_frame_done = lg1_resolver_frame_done;
    assign video_fetch_low_signature = lg1_front_response_low_signature;
    assign video_fetch_high_signature = lg1_front_response_high_signature;
    assign video_fetch_signature_generation =
        lg1_front_response_signature_generation;
    assign video_fetch_signature_valid =
        lg1_front_response_signature_valid;
    assign video_underrun_count = lg1_scanout_underrun_count;
    assign video_protocol_error_count = lg1_scanout_protocol_error_count;

    kcu105_lg1_capture_bist u_lg1_capture_bist (
        .clk(clk),
        .rst_n(soc_rst_n),
        .trigger_toggle(lg1_capture_trigger),
        .bus_valid(lg1_bist_bus_valid),
        .bus_addr(lg1_bist_bus_addr),
        .bus_wdata(lg1_bist_bus_wdata),
        .bus_wmask(lg1_bist_bus_wmask),
        .bus_ready(lg1_ready),
        .lg1_busy(lg1_busy),
        .flush(lg1_bist_flush),
        .flush_done(lg1_flush_done),
        .active(lg1_bist_active),
        .done_toggle(lg1_bist_done_toggle),
        .status(lg1_capture_bist_status)
    );

    (* DONT_TOUCH = "yes" *) lg1_clean_line_core u_lg1 (
        .clk(clk),
        .rst_n(soc_rst_n),
        .serial_clk(lg1_serial_clk),
        .serial_rst_n(lg1_serial_soc_rst_n),
        .addr(lg1_bus_addr),
        .wdata(lg1_bus_wdata),
        .rdata(lg1_rdata),
        .wmask(lg1_bus_wmask),
        .valid(lg1_bus_valid),
        .write(lg1_bus_write),
        .ready(lg1_ready),
        .flush(lg1_bist_flush),
        .flush_done(lg1_flush_done),
        .cache_dirty(lg1_cache_dirty),
        .scanout_pause(lg1_bist_active || lg1_capture_hold),
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
        .scan_mem_valid(lg1_scan_mem_valid),
        .scan_mem_ready(lg1_scan_mem_ready),
        .scan_mem_addr(lg1_scan_mem_addr),
        .scan_mem_rsp_valid(lg1_scan_mem_rsp_valid),
        .scan_mem_rsp_ready(lg1_scan_mem_rsp_ready),
        .scan_mem_rsp_rdata(lg1_scan_mem_rsp_rdata),
        .video_de(lg1_video_de),
        .video_hsync(lg1_video_hsync),
        .video_vsync(lg1_video_vsync),
        .video_r(lg1_video_r),
        .video_g(lg1_video_g),
        .video_b(lg1_video_b),
        .video_valid(lg1_video_valid),
        .video_frame_start(lg1_video_frame_start),
        .video_x(lg1_video_x),
        .video_y(lg1_video_y),
        .video_did(lg1_video_did),
        .video_raw_index(lg1_video_raw_index),
        .video_index(lg1_video_index),
        .video_xmap_mode(lg1_video_xmap_mode),
        .video_palette_bank(lg1_video_palette_bank),
        .video_resolver_class(lg1_video_resolver_class),
        .resolver_telemetry(lg1_resolver_telemetry),
        .resolver_detail(lg1_resolver_detail),
        .resolver_frame_done(lg1_resolver_frame_done),
        .front_response_low_signature(
            lg1_front_response_low_signature),
        .front_response_high_signature(
            lg1_front_response_high_signature),
        .front_response_signature_generation(
            lg1_front_response_signature_generation),
        .front_response_signature_valid(
            lg1_front_response_signature_valid),
        .scanout_underrun(lg1_scanout_underrun),
        .scanout_underrun_count(lg1_scanout_underrun_count),
        .scanout_protocol_error_count(lg1_scanout_protocol_error_count),
        .irq(lg1_irq),
        .scan_sample_valid(1'b0),
        .scan_sample_x(10'd0),
        .scan_sample_y(10'd0),
        .scan_sample_raw_index(8'd0),
        .scan_sample_raw_back_index(8'd0),
        .scan_sample_overlay(2'd0),
        .scan_sample_de(),
        .scan_sample_did(),
        .scan_sample_mode(),
        .scan_sample_palette_bank(),
        .scan_sample_display_index(),
        .scan_sample_r(),
        .scan_sample_g(),
        .scan_sample_b(),
        .busy(lg1_busy)
    );

    wire [63:0] int2_rdata;
    wire        int2_ready;
    wire [5:0]  int2_cpu_irq;
    int2 int2_inst (
        .clk        (clk),
        .rst_n      (soc_rst_n),
        .addr       (cpu_dmem_paddr[7:0]),
        .wdata      (cpu_dmem_wdata),
        .rdata      (int2_rdata),
        .wmask      (cpu_dmem_wmask),
        .valid      (int2_sel && dmem_active),
        .write      (cpu_dmem_wen),
        .ready      (int2_ready),
        .scsi0_irq  (hpc1_scsi0_irq),
        .scsi1_irq  (1'b0),
        .enet_irq   (hpc1_enet_irq),
        .hpc1_irq   (1'b0),
        .timer_irq_0(hpc1_timer_irq_0),
        .timer_irq_1(hpc1_timer_irq_1),
        .cpu_irq    (int2_cpu_irq)
    );

    assign combined_cpu_irq = hpc1_cpu_irq | int2_cpu_irq |
                              {1'b0, mc_bus_err_irq, 4'b0};

    wire any_handled = mc_sel || prom_sel || hpc1_pbus_sel || int2_sel ||
                       ip20_misc_sel || lg1_sel || gr2_sel || dsp_sram_sel;
    wire unmapped_sel = dmem_active && !any_handled;
    wire unmapped_gio_timeout =
        unmapped_sel && (gio_region || hpc1_unhandled);

    reg unmapped_ready;
    always @(posedge clk or negedge soc_rst_n) begin
        if (!soc_rst_n)
            unmapped_ready <= 1'b0;
        else if (!unmapped_sel)
            unmapped_ready <= 1'b0;
        else if (!unmapped_ready &&
                 (!unmapped_gio_timeout || mc_gio_abort_en))
            unmapped_ready <= 1'b1;
    end

    wire [63:0] bus_float_data = {cpu_dmem_paddr, cpu_dmem_paddr};
    wire [63:0] starter_video_absent_rdata = 64'hffff_ffff_ffff_ffff;

    assign cpu_dmem_rdata = mc_cpu_sel     ? mc_rdata :
                            prom_sel       ? prom_rdata :
                            hpc1_pbus_sel  ? hpc1_rdata :
                            int2_sel       ? int2_rdata :
                            ip20_misc_sel  ? ip20_misc_rdata :
                            dsp_sram_sel   ? dsp_sram_rdata :
                            graphics_slot_fallback_sel ? graphics_slot_fallback_rdata :
                            starter_video_slot_sel ? starter_video_absent_rdata :
                            bus_float_data;

    assign cpu_dmem_ready = mc_cpu_sel     ? mc_ready :
                            prom_sel       ? prom_ready :
                            hpc1_pbus_sel  ? hpc1_ready :
                            int2_sel       ? int2_ready :
                            ip20_misc_sel  ? ip20_misc_ready :
                            dsp_sram_sel   ? dsp_sram_ready :
                            unmapped_ready;

    reg [31:0] heartbeat_count;
    reg [31:0] dmem_count;
    reg [31:0] scc_console_count;
    reg [31:0] scc_console_fa_count;
    reg [127:0] scc_console_tx_history;
    localparam integer SCC_TX_MON_CLKS_PER_BIT = CLK_HZ / 9600;
    localparam integer SCC_TX_MON_HALF_BIT = SCC_TX_MON_CLKS_PER_BIT / 2;
    localparam [1:0] SCC_TX_MON_IDLE  = 2'd0;
    localparam [1:0] SCC_TX_MON_START = 2'd1;
    localparam [1:0] SCC_TX_MON_DATA  = 2'd2;
    localparam [1:0] SCC_TX_MON_STOP  = 2'd3;
    reg [1:0] scc_tx_mon_state;
    reg [15:0] scc_tx_mon_clk_count;
    reg [2:0] scc_tx_mon_bit_index;
    reg [7:0] scc_tx_mon_shift;
    reg [7:0] scc_tx_mon_last_data;
    reg [15:0] scc_tx_mon_count;
    reg [15:0] scc_tx_mon_fa_count;
    reg [15:0] scc_tx_mon_framing_count;
    reg [15:0] scc_tx_mon_false_start_count;
    reg [127:0] scc_tx_mon_history;
    reg [31:0] enet_tx_count;
    reg        scc_console_seen;
    reg        enet_tx_seen;
    reg        ram_seen;
    reg [31:0] debug_mc_last_ready_addr;
    reg [31:0] debug_mc_last_ready_rdata_lo;
    reg [31:0] debug_ram_last_write_addr;
    reg [63:0] debug_ram_last_write_data;
    reg [ 7:0] debug_ram_last_write_mask;
    reg [31:0] debug_prom_probe_last_addr;
    reg [31:0] debug_prom_probe_last_data;
    reg [31:0] debug_prom_probe_last_expected;
    reg [31:0] debug_prom_probe_first_bad_addr;
    reg [31:0] debug_prom_probe_first_bad_data;
    reg [31:0] debug_prom_probe_first_bad_expected;
    reg [31:0] debug_prom_probe_first_write_addr;
    reg [63:0] debug_prom_probe_first_write_data;
    reg [ 7:0] debug_prom_probe_first_write_mask;
    reg [15:0] debug_prom_probe_read_count;
    reg [15:0] debug_prom_probe_bad_count;
    reg        debug_prom_probe_first_bad_valid;
    reg        debug_prom_probe_first_write_valid;
    reg        debug_prom_probe_read_d;

    wire        debug_prom_probe_read =
        mc_sel && dmem_active && cpu_dmem_ren && mc_ready &&
        (cpu_dmem_paddr[31:4] == 28'h0800000);
    wire [31:0] debug_ram_write_paddr = {3'b000, ram_addr, 3'b000};
    wire        debug_prom_probe_write =
        ram_valid && ram_wen && ram_ready &&
        (debug_ram_write_paddr[31:4] == 28'h0800000);
    wire        debug_prom_probe_read_pulse =
        debug_prom_probe_read && !debug_prom_probe_read_d;
    wire [31:0] debug_prom_probe_read_word =
        cpu_dmem_paddr[2] ? mc_rdata[31:0] : mc_rdata[63:32];
    reg [31:0] debug_prom_probe_expected_word;
    always @* begin
        case (cpu_dmem_paddr[3:2])
            2'd0: debug_prom_probe_expected_word = 32'h1111_1111;
            2'd1: debug_prom_probe_expected_word = 32'h2222_2222;
            2'd2: debug_prom_probe_expected_word = 32'h4444_4444;
            default: debug_prom_probe_expected_word = 32'h8888_8888;
        endcase
    end
    wire debug_prom_probe_bad =
        debug_prom_probe_first_write_valid && debug_prom_probe_read_pulse &&
        (debug_prom_probe_read_word != debug_prom_probe_expected_word);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            heartbeat_count <= 32'd0;
            heartbeat <= 1'b0;
            dmem_count <= 32'd0;
            scc_console_count <= 32'd0;
            scc_console_fa_count <= 32'd0;
            scc_console_tx_history <= 128'd0;
            enet_tx_count <= 32'd0;
            scc_console_seen <= 1'b0;
            enet_tx_seen <= 1'b0;
            ram_seen <= 1'b0;
            debug_mc_last_ready_addr <= 32'd0;
            debug_mc_last_ready_rdata_lo <= 32'd0;
            debug_ram_last_write_addr <= 32'd0;
            debug_ram_last_write_data <= 64'd0;
            debug_ram_last_write_mask <= 8'd0;
            debug_prom_probe_last_addr <= 32'd0;
            debug_prom_probe_last_data <= 32'd0;
            debug_prom_probe_last_expected <= 32'd0;
            debug_prom_probe_first_bad_addr <= 32'd0;
            debug_prom_probe_first_bad_data <= 32'd0;
            debug_prom_probe_first_bad_expected <= 32'd0;
            debug_prom_probe_first_write_addr <= 32'd0;
            debug_prom_probe_first_write_data <= 64'd0;
            debug_prom_probe_first_write_mask <= 8'd0;
            debug_prom_probe_read_count <= 16'd0;
            debug_prom_probe_bad_count <= 16'd0;
            debug_prom_probe_first_bad_valid <= 1'b0;
            debug_prom_probe_first_write_valid <= 1'b0;
            debug_prom_probe_read_d <= 1'b0;
            bringup_phase <= 4'h0;
        end else begin
            debug_prom_probe_read_d <= debug_prom_probe_read;
            heartbeat_count <= heartbeat_count + 32'd1;
            heartbeat <= heartbeat_count[25];

            if (dmem_active && cpu_dmem_ready)
                dmem_count <= dmem_count + 32'd1;
            if (mc_sel && dmem_active && cpu_dmem_ren && mc_ready) begin
                debug_mc_last_ready_addr <= cpu_dmem_paddr;
                debug_mc_last_ready_rdata_lo <= mc_rdata[31:0];
            end
            if (debug_prom_probe_read_pulse) begin
                debug_prom_probe_last_addr <= cpu_dmem_paddr;
                debug_prom_probe_last_data <= debug_prom_probe_read_word;
                debug_prom_probe_last_expected <= debug_prom_probe_expected_word;
                if (debug_prom_probe_read_count != 16'hffff)
                    debug_prom_probe_read_count <= debug_prom_probe_read_count + 16'd1;
            end
            if (debug_prom_probe_bad) begin
                if (debug_prom_probe_bad_count != 16'hffff)
                    debug_prom_probe_bad_count <= debug_prom_probe_bad_count + 16'd1;
                if (!debug_prom_probe_first_bad_valid) begin
                    debug_prom_probe_first_bad_valid <= 1'b1;
                    debug_prom_probe_first_bad_addr <= cpu_dmem_paddr;
                    debug_prom_probe_first_bad_data <= debug_prom_probe_read_word;
                    debug_prom_probe_first_bad_expected <= debug_prom_probe_expected_word;
                end
            end
            if (mc_sel && dmem_active && cpu_dmem_ready)
                ram_seen <= 1'b1;
            if (ram_valid && ram_wen && ram_ready) begin
                debug_ram_last_write_addr <= debug_ram_write_paddr;
                debug_ram_last_write_data <= ram_wdata;
                debug_ram_last_write_mask <= ram_wmask;
                if (debug_prom_probe_write && !debug_prom_probe_first_write_valid) begin
                    debug_prom_probe_first_write_valid <= 1'b1;
                    debug_prom_probe_first_write_addr <= debug_ram_write_paddr;
                    debug_prom_probe_first_write_data <= ram_wdata;
                    debug_prom_probe_first_write_mask <= ram_wmask;
                end
            end
            if (scc_console_tx_valid) begin
                scc_console_seen <= 1'b1;
                scc_console_count <= scc_console_count + 32'd1;
                scc_console_tx_history <= {
                    scc_console_tx_history[119:0], scc_console_tx_data
                };
                if (scc_console_tx_data == 8'hfa)
                    scc_console_fa_count <= scc_console_fa_count + 32'd1;
            end
            if (enet_tx_valid && enet_tx_ready && enet_tx_eof) begin
                enet_tx_seen <= 1'b1;
                enet_tx_count <= enet_tx_count + 32'd1;
            end

            if (!eeprom_init_done)
                bringup_phase <= 4'h1;
            else if (!soc_rst_n)
                bringup_phase <= 4'h2;
            else if (!imem_ren)
                bringup_phase <= 4'h3;
            else if (!ram_seen)
                bringup_phase <= 4'h4;
            else if (!scc_console_seen)
                bringup_phase <= 4'h5;
            else if (!enet_tx_seen)
                bringup_phase <= 4'h6;
            else
                bringup_phase <= 4'h7;
        end
    end

    // Decode the actual SCC console waveform inside the FPGA with the same
    // nominal 9600-baud timing as the USB UART.  This is intentionally
    // independent of the SCC's byte-valid monitor: comparing the two streams
    // distinguishes a malformed RTL waveform from corruption after the FPGA
    // output boundary.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scc_tx_mon_state <= SCC_TX_MON_IDLE;
            scc_tx_mon_clk_count <= 16'd0;
            scc_tx_mon_bit_index <= 3'd0;
            scc_tx_mon_shift <= 8'd0;
            scc_tx_mon_last_data <= 8'd0;
            scc_tx_mon_count <= 16'd0;
            scc_tx_mon_fa_count <= 16'd0;
            scc_tx_mon_framing_count <= 16'd0;
            scc_tx_mon_false_start_count <= 16'd0;
            scc_tx_mon_history <= 128'd0;
        end else begin
            case (scc_tx_mon_state)
                SCC_TX_MON_IDLE: begin
                    if (!scc_txd_b) begin
                        scc_tx_mon_state <= SCC_TX_MON_START;
                        scc_tx_mon_clk_count <= SCC_TX_MON_HALF_BIT - 1;
                    end
                end

                SCC_TX_MON_START: begin
                    if (scc_tx_mon_clk_count != 0) begin
                        scc_tx_mon_clk_count <= scc_tx_mon_clk_count - 16'd1;
                    end else if (!scc_txd_b) begin
                        scc_tx_mon_state <= SCC_TX_MON_DATA;
                        scc_tx_mon_clk_count <= SCC_TX_MON_CLKS_PER_BIT - 1;
                        scc_tx_mon_bit_index <= 3'd0;
                    end else begin
                        scc_tx_mon_state <= SCC_TX_MON_IDLE;
                        scc_tx_mon_false_start_count <=
                            scc_tx_mon_false_start_count + 16'd1;
                    end
                end

                SCC_TX_MON_DATA: begin
                    if (scc_tx_mon_clk_count != 0) begin
                        scc_tx_mon_clk_count <= scc_tx_mon_clk_count - 16'd1;
                    end else begin
                        scc_tx_mon_shift[scc_tx_mon_bit_index] <= scc_txd_b;
                        scc_tx_mon_clk_count <= SCC_TX_MON_CLKS_PER_BIT - 1;
                        if (scc_tx_mon_bit_index == 3'd7)
                            scc_tx_mon_state <= SCC_TX_MON_STOP;
                        else
                            scc_tx_mon_bit_index <= scc_tx_mon_bit_index + 3'd1;
                    end
                end

                SCC_TX_MON_STOP: begin
                    if (scc_tx_mon_clk_count != 0) begin
                        scc_tx_mon_clk_count <= scc_tx_mon_clk_count - 16'd1;
                    end else begin
                        scc_tx_mon_state <= SCC_TX_MON_IDLE;
                        if (scc_txd_b) begin
                            scc_tx_mon_count <= scc_tx_mon_count + 16'd1;
                            scc_tx_mon_last_data <= scc_tx_mon_shift;
                            scc_tx_mon_history <= {
                                scc_tx_mon_history[119:0], scc_tx_mon_shift
                            };
                            if (scc_tx_mon_shift == 8'hfa)
                                scc_tx_mon_fa_count <=
                                    scc_tx_mon_fa_count + 16'd1;
                        end else begin
                            scc_tx_mon_framing_count <=
                                scc_tx_mon_framing_count + 16'd1;
                        end
                    end
                end

                default: scc_tx_mon_state <= SCC_TX_MON_IDLE;
            endcase
        end
    end

    assign status_flags = {
        eeprom_init_done,
        soc_rst_n,
        imem_ren,
        dmem_active,
        cpu_dmem_ready,
        cpu_dmem_ren,
        cpu_dmem_wen,
        mc_sel,
        mc_ready,
        ram_valid,
        ram_wen,
        ram_ready,
        prom_sel,
        prom_ready,
        hpc1_pbus_sel,
        hpc1_ready,
        int2_sel,
        int2_ready,
        ip20_misc_sel,
        ip20_misc_ready,
        unmapped_sel,
        unmapped_ready,
        mc_bus_err_irq,
        hpc1_enet_irq,
        hpc1_scsi0_irq,
        hpc1_timer_irq_0,
        scc_console_tx_valid,
        scc_txd_b,
        console_led,
        dma_active,
        enet_tx_seen,
        enet_rx_valid
    };

`ifdef KCU105_CYAN_SOC_DEBUG
    wire [127:0] lg1_debug_probe = {
        32'h4c473144, // "LG1D"
        lg1_scanout_underrun_count[15:0],
        lg1_scanout_protocol_error_count[15:0],
        lg1_video_r,
        lg1_video_g,
        lg1_video_b,
        lg1_video_x,
        lg1_video_y,
        lg1_bus_addr,
        lg1_bus_valid,
        lg1_bus_write,
        lg1_busy,
        lg1_cache_dirty,
        lg1_irq
    };
    assign debug_probe = gfx_debug_probe_select ? lg1_debug_probe :
                          sd_scsi_debug_probe[255:128];
`else
    assign debug_probe = {
        debug_mc_last_ready_addr,
        debug_mc_last_ready_rdata_lo,
        cpu_debug_core_wb_eff_addr,
        cpu_debug_core_wb_load_data
    };
`endif

    assign eeprom_debug_probe = {
        {8'hEE, bringup_phase, 4'h0,
         eeprom_init_done, soc_rst_n, mc_eeprom_cs, mc_eeprom_clk,
         mc_eeprom_di, eeprom_do, hpc1_eeprom_cs, hpc1_eeprom_clk,
         hpc1_eeprom_di, mc_eerom_reg_read, mc_eerom_reg_write,
         mc_ready, 4'h0},
        {mc_eeprom_rise_count, mc_eerom_read_count},
        {mc_eeprom_do_shift, mc_eeprom_di_shift},
        mc_eerom_last_rdata
    };

`ifdef KCU105_CYAN_SOC_DEBUG
    wire [255:0] dma_snoop_debug_probe = {
        32'h534E4F50, // "SNOP"
        {dma_snoop_rd_count, dma_snoop_rd_hit_count},
        dma_snoop_rd_last_accept_addr,
        dma_snoop_rd_last_raw_data[63:32],
        dma_snoop_rd_last_raw_data[31:0],
        dma_snoop_rd_last_hit_addr,
        dma_snoop_rd_last_hit_data[63:32],
        dma_snoop_rd_last_hit_data[31:0]
    };

    wire [255:0] inode1856_dma_debug_probe = {
        32'h494E_4F44, // "INOD"
        inode1856_write_status,
        {inode1856_dma_beat_count, inode1856_dma_first_index},
        inode1856_dma_first_addr,
        inode1856_dma_beat0[63:32],
        inode1856_dma_beat0[31:0],
        inode1856_dma_beat1[63:32],
        inode1856_dma_beat1[31:0]
    };

    wire [255:0] volume_dma0_debug_probe = {
        32'h5648_4430, // "VHD0"
        {volume_dma_beat_count, volume_dma_first_index},
        volume_dma_first_addr,
        volume_dma_beat0[63:32],
        volume_dma_beat0[31:0],
        volume_dma_beat1[63:32],
        volume_dma_beat1[31:0],
        {24'd0, volume_dma_first_mask}
    };

    wire [255:0] volume_dma1_debug_probe = {
        32'h5648_4431, // "VHD1"
        volume_dma_beat2[63:32],
        volume_dma_beat2[31:0],
        volume_dma_beat3[63:32],
        volume_dma_beat3[31:0],
        96'd0
    };

    wire [255:0] efs_dma0_debug_probe = {
        32'h4546_4430, // "EFD0"
        {efs_dma_beat_count, efs_dma_first_index},
        efs_dma_first_addr,
        efs_dma_beat0[63:32],
        efs_dma_beat0[31:0],
        efs_dma_beat1[63:32],
        efs_dma_beat1[31:0],
        {24'd0, efs_dma_first_mask}
    };

    wire [255:0] efs_dma1_debug_probe = {
        32'h4546_4431, // "EFD1"
        efs_dma_beat2[63:32],
        efs_dma_beat2[31:0],
        efs_dma_beat3[63:32],
        efs_dma_beat3[31:0],
        96'd0
    };

    wire [255:0] cpu_irq_debug_probe = {
        32'h4350_5549, // "CPUI"
        cpu_debug_core_pc,
        cpu_debug_core_epc,
        cpu_debug_core_cause,
        cpu_debug_core_badvaddr,
        cpu_debug_core_status,
        {14'd0, combined_cpu_irq, hpc1_cpu_irq, int2_cpu_irq},
        {24'd0, mc_bus_err_irq, hpc1_enet_irq, hpc1_scsi0_irq,
        hpc1_timer_irq_0, hpc1_timer_irq_1, dma_active, cpu_dmem_ren,
         cpu_dmem_wen}
    };

    wire [255:0] cpu_post_remount_debug_probe = {
        32'h5052_4D32, // "PRM2"
        cpu_debug_core_flags,
        cpu_debug_cache_status,
        cpu_debug_core_rf_pc,
        cpu_debug_core_rf_insn,
        cpu_debug_core_wb_pc,
        cpu_debug_core_wb_insn,
        cpu_debug_dtlb_state
    };

    wire [255:0] cpu_live_debug_probe = {
        32'h4350_554C, // "CPUL"
        cpu_debug_core_pc,
        cpu_debug_core_rf_pc,
        cpu_debug_core_wb_pc,
        cpu_debug_core_wb_insn,
        cpu_debug_core_wb_eff_addr,
        cpu_debug_core_wb_load_data,
        cpu_debug_core_flags
    };

    // A local-root startup workload can stop with newly forked service
    // children asleep after its mapping allocator fails to find an otherwise
    // available K2 mapping.  Preserve the final allocator context before its
    // psema(&sptwaitlock) call.  This is observation-only and uses the
    // existing combined CPU+DSP VIO profile after a wait has been observed.
    wire cpu_sptalloc_wb =
        (cpu_debug_core_wb_pc >= 32'h8805_EB4C) &&
        (cpu_debug_core_wb_pc <  32'h8805_EF54);
    wire cpu_sptalloc_scan_wb =
        (cpu_debug_core_wb_pc == 32'h8805_ED88) ||
        (cpu_debug_core_wb_pc == 32'h8805_EE24);
    // Freeze on the JAL delay slot, after both the semaphore address in a0
    // and the call return address in ra are architecturally visible.
    wire cpu_sptalloc_wait_wb =
        (cpu_debug_core_wb_pc == 32'h8805_EF18);

    reg        spt_wait_seen;
    reg [15:0] spt_call_count;
    reg [15:0] spt_scan_count;
    reg [31:0] spt_last_pc;
    reg [31:0] spt_req_count;
    reg [31:0] spt_remaining;
    reg [31:0] spt_scan_start;
    reg [31:0] spt_retry_count;
    reg [31:0] spt_last_scan_result;
    reg [31:0] spt_a0;
    reg [31:0] spt_a1;
    reg [31:0] spt_a2;
    reg [31:0] spt_v0;
    reg [31:0] spt_s4;
    reg [31:0] spt_s5;
    reg [31:0] spt_s6;
    reg [31:0] spt_t0;
    reg [31:0] spt_t1;
    reg [31:0] spt_sp;
    reg [31:0] spt_ra;
    reg [31:0] spt_flags;

    // The observed software image places the allocator count at 0x8814a2b0.
    // Record every
    // committed write so the sleep snapshot says whether free K2 space was
    // already advertised when the allocator gave up, rather than only when a
    // later /dev/kmem inspection happened to read it.
    wire spt_count_store_commit =
        cpu_debug_core_store_commit &&
        (cpu_debug_core_store_vaddr[31:2] == 30'h2205_28ac);
    reg [23:0] spt_count_store_count;
    reg [7:0]  spt_count_store_wmask;
    reg [31:0] spt_count_last_value;

    always @(posedge clk or negedge soc_rst_n) begin
        if (!soc_rst_n) begin
            spt_count_store_count <= 24'd0;
            spt_count_store_wmask <= 8'd0;
            spt_count_last_value <= 32'd0;
        end else if (!spt_wait_seen && spt_count_store_commit) begin
            spt_count_store_count <= spt_count_store_count + 24'd1;
            spt_count_store_wmask <= cpu_debug_core_store_wmask;
            spt_count_last_value <= cpu_debug_core_store_vaddr[2] ?
                cpu_debug_core_store_wdata[31:0] :
                cpu_debug_core_store_wdata[63:32];
        end
    end

    always @(posedge clk or negedge soc_rst_n) begin
        if (!soc_rst_n) begin
            spt_wait_seen <= 1'b0;
            spt_call_count <= 16'd0;
            spt_scan_count <= 16'd0;
            spt_last_pc <= 32'd0;
            spt_req_count <= 32'd0;
            spt_remaining <= 32'd0;
            spt_scan_start <= 32'd0;
            spt_retry_count <= 32'd0;
            spt_last_scan_result <= 32'd0;
            spt_a0 <= 32'd0;
            spt_a1 <= 32'd0;
            spt_a2 <= 32'd0;
            spt_v0 <= 32'd0;
            spt_s4 <= 32'd0;
            spt_s5 <= 32'd0;
            spt_s6 <= 32'd0;
            spt_t0 <= 32'd0;
            spt_t1 <= 32'd0;
            spt_sp <= 32'd0;
            spt_ra <= 32'd0;
            spt_flags <= 32'd0;
        end else if (!spt_wait_seen && cpu_sptalloc_wb) begin
            if (cpu_debug_core_wb_pc == 32'h8805_EB4C)
                spt_call_count <= spt_call_count + 16'd1;
            if (cpu_sptalloc_scan_wb)
                spt_scan_count <= spt_scan_count + 16'd1;
            if (cpu_debug_core_wb_pc == 32'h8805_ED88)
                spt_last_scan_result <= cpu_debug_core_gpr_v0;
            if (cpu_sptalloc_wait_wb)
                spt_wait_seen <= 1'b1;
            spt_last_pc <= cpu_debug_core_wb_pc;
            spt_req_count <= cpu_debug_core_gpr_s3;
            spt_remaining <= cpu_debug_core_gpr_s0;
            spt_scan_start <= cpu_debug_core_gpr_s1;
            spt_retry_count <= cpu_debug_core_gpr_s2;
            spt_a0 <= cpu_debug_core_gpr_a0;
            spt_a1 <= cpu_debug_core_gpr_a1;
            spt_a2 <= cpu_debug_core_gpr_a2;
            spt_v0 <= cpu_debug_core_gpr_v0;
            spt_s4 <= cpu_debug_core_gpr_s4;
            spt_s5 <= cpu_debug_core_gpr_s5;
            spt_s6 <= cpu_debug_core_gpr_s6;
            spt_t0 <= cpu_debug_core_gpr_t0;
            spt_t1 <= cpu_debug_core_gpr_t1;
            spt_sp <= cpu_debug_core_gpr_sp;
            spt_ra <= cpu_debug_core_gpr_ra;
            spt_flags <= cpu_debug_core_flags;
        end
    end

    wire [255:0] cpu_spt_debug_probe0 = {
        32'h5350_5430, // "SPT0"
        {spt_call_count, spt_scan_count},
        spt_last_pc,
        spt_req_count,
        spt_remaining,
        spt_scan_start,
        spt_retry_count,
        spt_last_scan_result
    };

    wire [255:0] cpu_spt_debug_probe1 = {
        32'h5350_5431, // "SPT1"
        spt_a0,
        spt_a1,
        spt_a2,
        spt_v0,
        spt_s4,
        spt_s5,
        spt_s6
    };

    wire [255:0] cpu_spt_debug_probe2 = {
        32'h5350_5432, // "SPT2"
        spt_t0,
        spt_t1,
        {spt_count_store_count, spt_count_store_wmask},
        spt_count_last_value,
        spt_sp,
        spt_ra,
        spt_flags
    };

    wire cmp_loop_store_commit =
        cpu_debug_core_store_commit &&
        (cpu_debug_core_store_pc >= 32'h0040_1180) &&
        (cpu_debug_core_store_pc <  32'h0040_1280);
    reg [31:0] cmp_loop_store_count;
    reg [31:0] cmp_loop_last_store_pc;
    reg [31:0] cmp_loop_last_store_vaddr;
    reg [ 7:0] cmp_loop_last_store_wmask;
    reg [63:0] cmp_loop_last_store_wdata;
    always @(posedge clk or negedge soc_rst_n) begin
        if (!soc_rst_n) begin
            cmp_loop_store_count <= 32'd0;
            cmp_loop_last_store_pc <= 32'd0;
            cmp_loop_last_store_vaddr <= 32'd0;
            cmp_loop_last_store_wmask <= 8'd0;
            cmp_loop_last_store_wdata <= 64'd0;
        end else if (cmp_loop_store_commit) begin
            cmp_loop_store_count <= cmp_loop_store_count + 32'd1;
            cmp_loop_last_store_pc <= cpu_debug_core_store_pc;
            cmp_loop_last_store_vaddr <= cpu_debug_core_store_vaddr;
            cmp_loop_last_store_wmask <= cpu_debug_core_store_wmask;
            cmp_loop_last_store_wdata <= cpu_debug_core_store_wdata;
        end
    end

    wire [255:0] cmp_loop_regs_debug_probe = {
        32'h434D_5033, // "CMP3"
        cpu_debug_core_pc,
        cpu_debug_core_rf_pc,
        cpu_debug_core_wb_pc,
        cpu_debug_core_gpr_v0,
        cpu_debug_core_gpr_a0,
        cpu_debug_core_gpr_s0,
        cpu_debug_core_gpr_s5
    };

    wire [255:0] cmp_loop_store_debug_probe = {
        32'h434D_5053, // "CMPS"
        cmp_loop_store_count,
        cmp_loop_last_store_pc,
        cmp_loop_last_store_vaddr,
        cmp_loop_last_store_wdata[63:32],
        cmp_loop_last_store_wdata[31:0],
        {23'd0, cmp_loop_store_commit, cmp_loop_last_store_wmask},
        cpu_debug_dtlb_state
    };

    wire [31:0] scc_status_word = {
        uart_rxd,
        uart_txd,
        scc_txd_b,
        scc_console_tx_valid,
        debug_rx_inject_valid,
        debug_rx_inject_toggle,
        hpc1_enet_irq,
        hpc1_scsi0_irq,
        hpc1_timer_irq_0,
        hpc1_timer_irq_1,
        dma_active,
        cpu_dmem_ren,
        cpu_dmem_wen,
        hpc1_pbus_sel,
        hpc1_ready,
        console_led,
        10'd0,
        hpc1_cpu_irq
    };

    wire [255:0] scc_console_debug_probe = {
        32'h5343_4344, // "SCCD"
        scc_console_count,
        scc_status_word,
        {24'd0, debug_rx_inject_data},
        scc_hpc_debug_probe
    };

    // Retain the last sixteen bytes accepted by the SCC transmit holding
    // register.  Comparing this history with the bytes decoded at the USB
    // UART distinguishes a software write from a malformed serial waveform.
    wire [255:0] scc_tx_history_debug_probe = {
        32'h5343_4348, // "SCCH"
        scc_console_count,
        scc_console_fa_count,
        {24'd0, scc_console_tx_data},
        scc_console_tx_history
    };

    wire [255:0] scc_tx_waveform_debug_probe = {
        32'h5343_5458, // "SCTX"
        {scc_tx_mon_count, scc_tx_mon_fa_count},
        {scc_tx_mon_framing_count, scc_tx_mon_false_start_count},
        {scc_tx_mon_last_data, 2'd0, scc_tx_mon_state,
         1'd0, scc_tx_mon_bit_index, scc_tx_mon_clk_count},
        scc_tx_mon_history
    };

    // The filesystem checker caches the first disk block represented by each
    // inode buffer in a per-worker word. A bad value in that word leads it to
    // derive a wild pointer and
    // cylpass1 faults while dereferencing it.  Learn every worker's word from
    // the three legitimate store sites, and retain any other CPU or DMA write
    // to the same physical word.  This is observation-only.
    wire fsck_cache_init_store =
        cpu_debug_core_store_commit &&
        (cpu_debug_core_store_pc == 32'h0040_8408);
    wire fsck_cache_fallback_store =
        cpu_debug_core_store_commit &&
        (cpu_debug_core_store_pc == 32'h0040_8984);
    wire fsck_cache_refill_store =
        cpu_debug_core_store_commit &&
        (cpu_debug_core_store_pc == 32'h0040_89a4);
    wire fsck_cache_control_store =
        fsck_cache_init_store ||
        fsck_cache_fallback_store ||
        fsck_cache_refill_store;
    wire [31:0] fsck_cache_store_word = cpu_debug_core_store_paddr[2] ?
        cpu_debug_core_store_wdata[31:0] :
        cpu_debug_core_store_wdata[63:32];
    wire [3:0] fsck_cache_store_word_mask = cpu_debug_core_store_paddr[2] ?
        cpu_debug_core_store_wmask[3:0] :
        cpu_debug_core_store_wmask[7:4];

    reg [3:0]  fsck_cache_target_count;
    reg [31:0] fsck_cache_target_paddr0;
    reg [31:0] fsck_cache_target_paddr1;
    reg [31:0] fsck_cache_target_paddr2;
    reg [31:0] fsck_cache_target_paddr3;
    reg [31:0] fsck_cache_target_paddr4;
    reg [31:0] fsck_cache_target_paddr5;
    reg [31:0] fsck_cache_target_paddr6;
    reg [31:0] fsck_cache_target_paddr7;
    reg [31:0] fsck_cache_last_target_vaddr;
    reg [31:0] fsck_cache_last_target_paddr;
    reg [31:0] fsck_cache_init_word;
    reg [31:0] fsck_cache_last_control_pc;
    reg [31:0] fsck_cache_last_control_word;
    reg [15:0] fsck_cache_control_count;
    reg [15:0] fsck_cache_refill_count;

    wire fsck_cache_target_hit =
        ((fsck_cache_target_count > 4'd0) &&
         (cpu_debug_core_store_paddr[31:2] == fsck_cache_target_paddr0[31:2])) ||
        ((fsck_cache_target_count > 4'd1) &&
         (cpu_debug_core_store_paddr[31:2] == fsck_cache_target_paddr1[31:2])) ||
        ((fsck_cache_target_count > 4'd2) &&
         (cpu_debug_core_store_paddr[31:2] == fsck_cache_target_paddr2[31:2])) ||
        ((fsck_cache_target_count > 4'd3) &&
         (cpu_debug_core_store_paddr[31:2] == fsck_cache_target_paddr3[31:2])) ||
        ((fsck_cache_target_count > 4'd4) &&
         (cpu_debug_core_store_paddr[31:2] == fsck_cache_target_paddr4[31:2])) ||
        ((fsck_cache_target_count > 4'd5) &&
         (cpu_debug_core_store_paddr[31:2] == fsck_cache_target_paddr5[31:2])) ||
        ((fsck_cache_target_count > 4'd6) &&
         (cpu_debug_core_store_paddr[31:2] == fsck_cache_target_paddr6[31:2])) ||
        ((fsck_cache_target_count > 4'd7) &&
         (cpu_debug_core_store_paddr[31:2] == fsck_cache_target_paddr7[31:2]));
    wire fsck_cache_target_known = fsck_cache_target_hit;
    wire fsck_cache_control_new_target =
        fsck_cache_control_store && !fsck_cache_target_known &&
        (fsck_cache_target_count < 4'd8);

    reg        fsck_cache_bad_cpu_seen;
    reg [31:0] fsck_cache_bad_cpu_pc;
    reg [31:0] fsck_cache_bad_cpu_vaddr;
    reg [31:0] fsck_cache_bad_cpu_paddr;
    reg [31:0] fsck_cache_bad_cpu_word;
    reg [7:0]  fsck_cache_bad_cpu_wmask;
    reg [15:0] fsck_cache_target_store_count;
    reg [31:0] fsck_cache_last_store_pc;
    reg [31:0] fsck_cache_last_store_vaddr;
    reg [31:0] fsck_cache_last_store_paddr;
    reg [31:0] fsck_cache_last_store_word;
    reg [7:0]  fsck_cache_last_store_wmask;

    // A legitimate cached block number is below the filesystem's sentinel
    // value (0x001de7d5 in this run), and all three writes are full words.
    // Also flag any write to a learned field from an instruction other than
    // the three store sites above.
    wire fsck_cache_bad_cpu_store =
        cpu_debug_core_store_commit &&
        ((fsck_cache_control_store &&
          ((fsck_cache_store_word[31:21] != 11'd0) ||
           (fsck_cache_store_word_mask != 4'hf))) ||
         (fsck_cache_target_hit && !fsck_cache_control_store));

    wire fsck_cache_dma_target_hit = dma_snoop_valid &&
        (((fsck_cache_target_count > 4'd0) &&
          (dma_snoop_addr[31:3] == fsck_cache_target_paddr0[31:3])) ||
         ((fsck_cache_target_count > 4'd1) &&
          (dma_snoop_addr[31:3] == fsck_cache_target_paddr1[31:3])) ||
         ((fsck_cache_target_count > 4'd2) &&
          (dma_snoop_addr[31:3] == fsck_cache_target_paddr2[31:3])) ||
         ((fsck_cache_target_count > 4'd3) &&
          (dma_snoop_addr[31:3] == fsck_cache_target_paddr3[31:3])) ||
         ((fsck_cache_target_count > 4'd4) &&
          (dma_snoop_addr[31:3] == fsck_cache_target_paddr4[31:3])) ||
         ((fsck_cache_target_count > 4'd5) &&
          (dma_snoop_addr[31:3] == fsck_cache_target_paddr5[31:3])) ||
         ((fsck_cache_target_count > 4'd6) &&
          (dma_snoop_addr[31:3] == fsck_cache_target_paddr6[31:3])) ||
         ((fsck_cache_target_count > 4'd7) &&
          (dma_snoop_addr[31:3] == fsck_cache_target_paddr7[31:3])));
    reg        fsck_cache_dma_seen;
    reg [15:0] fsck_cache_dma_count;
    reg [31:0] fsck_cache_dma_addr;
    reg [63:0] fsck_cache_dma_data;
    reg [7:0]  fsck_cache_dma_wmask;

    wire fsck_cache_word_load_wb =
        (cpu_debug_core_wb_pc == 32'h0040_88a8);
    wire fsck_cache_bad_load_wb =
        fsck_cache_word_load_wb &&
        (cpu_debug_core_wb_load_data[31:21] != 11'd0);
    reg [15:0] fsck_cache_load_count;
    reg [31:0] fsck_cache_last_load_vaddr;
    reg [31:0] fsck_cache_last_load_word;
    reg        fsck_cache_bad_load_seen;
    reg [31:0] fsck_cache_bad_load_vaddr;
    reg [31:0] fsck_cache_bad_load_word;

    always @(posedge clk or negedge soc_rst_n) begin
        if (!soc_rst_n) begin
            fsck_cache_target_count <= 4'd0;
            fsck_cache_target_paddr0 <= 32'd0;
            fsck_cache_target_paddr1 <= 32'd0;
            fsck_cache_target_paddr2 <= 32'd0;
            fsck_cache_target_paddr3 <= 32'd0;
            fsck_cache_target_paddr4 <= 32'd0;
            fsck_cache_target_paddr5 <= 32'd0;
            fsck_cache_target_paddr6 <= 32'd0;
            fsck_cache_target_paddr7 <= 32'd0;
            fsck_cache_last_target_vaddr <= 32'd0;
            fsck_cache_last_target_paddr <= 32'd0;
            fsck_cache_init_word <= 32'd0;
            fsck_cache_last_control_pc <= 32'd0;
            fsck_cache_last_control_word <= 32'd0;
            fsck_cache_control_count <= 16'd0;
            fsck_cache_refill_count <= 16'd0;
            fsck_cache_bad_cpu_seen <= 1'b0;
            fsck_cache_bad_cpu_pc <= 32'd0;
            fsck_cache_bad_cpu_vaddr <= 32'd0;
            fsck_cache_bad_cpu_paddr <= 32'd0;
            fsck_cache_bad_cpu_word <= 32'd0;
            fsck_cache_bad_cpu_wmask <= 8'd0;
            fsck_cache_target_store_count <= 16'd0;
            fsck_cache_last_store_pc <= 32'd0;
            fsck_cache_last_store_vaddr <= 32'd0;
            fsck_cache_last_store_paddr <= 32'd0;
            fsck_cache_last_store_word <= 32'd0;
            fsck_cache_last_store_wmask <= 8'd0;
            fsck_cache_dma_seen <= 1'b0;
            fsck_cache_dma_count <= 16'd0;
            fsck_cache_dma_addr <= 32'd0;
            fsck_cache_dma_data <= 64'd0;
            fsck_cache_dma_wmask <= 8'd0;
            fsck_cache_load_count <= 16'd0;
            fsck_cache_last_load_vaddr <= 32'd0;
            fsck_cache_last_load_word <= 32'd0;
            fsck_cache_bad_load_seen <= 1'b0;
            fsck_cache_bad_load_vaddr <= 32'd0;
            fsck_cache_bad_load_word <= 32'd0;
        end else begin
            if (fsck_cache_control_store) begin
                fsck_cache_control_count <= fsck_cache_control_count + 16'd1;
                fsck_cache_last_target_vaddr <= cpu_debug_core_store_vaddr;
                fsck_cache_last_target_paddr <= cpu_debug_core_store_paddr;
                fsck_cache_last_control_pc <= cpu_debug_core_store_pc;
                fsck_cache_last_control_word <= fsck_cache_store_word;
                if (fsck_cache_init_store)
                    fsck_cache_init_word <= fsck_cache_store_word;
                if (fsck_cache_refill_store)
                    fsck_cache_refill_count <= fsck_cache_refill_count + 16'd1;
                if (fsck_cache_control_new_target) begin
                    case (fsck_cache_target_count)
                        4'd0: fsck_cache_target_paddr0 <= cpu_debug_core_store_paddr;
                        4'd1: fsck_cache_target_paddr1 <= cpu_debug_core_store_paddr;
                        4'd2: fsck_cache_target_paddr2 <= cpu_debug_core_store_paddr;
                        4'd3: fsck_cache_target_paddr3 <= cpu_debug_core_store_paddr;
                        4'd4: fsck_cache_target_paddr4 <= cpu_debug_core_store_paddr;
                        4'd5: fsck_cache_target_paddr5 <= cpu_debug_core_store_paddr;
                        4'd6: fsck_cache_target_paddr6 <= cpu_debug_core_store_paddr;
                        4'd7: fsck_cache_target_paddr7 <= cpu_debug_core_store_paddr;
                        default: begin end
                    endcase
                    fsck_cache_target_count <= fsck_cache_target_count + 4'd1;
                end
            end
            if (cpu_debug_core_store_commit &&
                (fsck_cache_target_hit || fsck_cache_control_store)) begin
                fsck_cache_target_store_count <=
                    fsck_cache_target_store_count + 16'd1;
                fsck_cache_last_store_pc <= cpu_debug_core_store_pc;
                fsck_cache_last_store_vaddr <= cpu_debug_core_store_vaddr;
                fsck_cache_last_store_paddr <= cpu_debug_core_store_paddr;
                fsck_cache_last_store_word <= fsck_cache_store_word;
                fsck_cache_last_store_wmask <= cpu_debug_core_store_wmask;
            end
            if (fsck_cache_bad_cpu_store && !fsck_cache_bad_cpu_seen) begin
                fsck_cache_bad_cpu_seen <= 1'b1;
                fsck_cache_bad_cpu_pc <= cpu_debug_core_store_pc;
                fsck_cache_bad_cpu_vaddr <= cpu_debug_core_store_vaddr;
                fsck_cache_bad_cpu_paddr <= cpu_debug_core_store_paddr;
                fsck_cache_bad_cpu_word <= fsck_cache_store_word;
                fsck_cache_bad_cpu_wmask <= cpu_debug_core_store_wmask;
            end
            if (fsck_cache_dma_target_hit) begin
                fsck_cache_dma_seen <= 1'b1;
                fsck_cache_dma_count <= fsck_cache_dma_count + 16'd1;
                fsck_cache_dma_addr <= dma_snoop_addr;
                fsck_cache_dma_data <= dma_snoop_wdata;
                fsck_cache_dma_wmask <= dma_snoop_wmask;
            end
            if (fsck_cache_word_load_wb) begin
                fsck_cache_load_count <= fsck_cache_load_count + 16'd1;
                fsck_cache_last_load_vaddr <= cpu_debug_core_wb_eff_addr;
                fsck_cache_last_load_word <= cpu_debug_core_wb_load_data;
            end
            if (fsck_cache_bad_load_wb && !fsck_cache_bad_load_seen) begin
                fsck_cache_bad_load_seen <= 1'b1;
                fsck_cache_bad_load_vaddr <= cpu_debug_core_wb_eff_addr;
                fsck_cache_bad_load_word <= cpu_debug_core_wb_load_data;
            end
        end
    end

    wire [255:0] fsck_cache_trace_meta_debug_probe = {
        32'h4653_5730, // "FSW0"
        {fsck_cache_target_count, 4'd0,
         fsck_cache_bad_cpu_seen, fsck_cache_dma_seen,
         fsck_cache_bad_load_seen, 5'd0,
         fsck_cache_control_count},
        fsck_cache_last_target_vaddr,
        fsck_cache_last_target_paddr,
        fsck_cache_init_word,
        fsck_cache_last_control_pc,
        fsck_cache_last_control_word,
        {fsck_cache_refill_count, fsck_cache_target_store_count}
    };

    wire [255:0] fsck_cache_trace_cpu_debug_probe = {
        32'h4653_5731, // "FSW1"
        fsck_cache_bad_cpu_pc,
        fsck_cache_bad_cpu_vaddr,
        fsck_cache_bad_cpu_paddr,
        fsck_cache_bad_cpu_word,
        {24'd0, fsck_cache_bad_cpu_wmask},
        fsck_cache_last_store_pc,
        fsck_cache_last_store_word
    };

    wire [255:0] fsck_cache_trace_dma_debug_probe = {
        32'h4653_5732, // "FSW2"
        {15'd0, fsck_cache_dma_seen, fsck_cache_dma_count},
        fsck_cache_dma_addr,
        fsck_cache_dma_data[63:32],
        fsck_cache_dma_data[31:0],
        {24'd0, fsck_cache_dma_wmask},
        fsck_cache_target_paddr0,
        fsck_cache_target_paddr1
    };

    wire [255:0] fsck_cache_trace_load_debug_probe = {
        32'h4653_5733, // "FSW3"
        {15'd0, fsck_cache_bad_load_seen, fsck_cache_load_count},
        fsck_cache_last_load_vaddr,
        fsck_cache_last_load_word,
        fsck_cache_bad_load_vaddr,
        fsck_cache_bad_load_word,
        fsck_cache_last_store_vaddr,
        fsck_cache_last_store_paddr
    };

    reg       fsck_process_seen;
    reg [7:0] fsck_process_asid;

    // Arm on the fsck Phase-1 inode-walk/ckinode code itself.  Shared libc is
    // also used by the parent shell after fsck dies, so PC range alone cannot
    // keep a persistent "latest TLB fault" latch tied to the failed process.
    // EntryHi[7:0] is the R4000 ASID and survives while handling that process.
    wire fsck_phase1_pc_live =
        (cpu_debug_core_store_pc == 32'h0040_eaf0) ||
        (cpu_debug_core_store_pc == 32'h0040_f904) ||
        (cpu_debug_core_is_pc == 32'h0040_8a60) ||
        (cpu_debug_core_is_pc == 32'h0040_8a64) ||
        (cpu_debug_core_is_pc == 32'h0040_eac8) ||
        (cpu_debug_core_is_pc == 32'h0040_eacc) ||
        (cpu_debug_core_is_pc == 32'h0040_eaf0) ||
        (cpu_debug_core_rf_pc == 32'h0040_8a60) ||
        (cpu_debug_core_rf_pc == 32'h0040_8a64) ||
        (cpu_debug_core_rf_pc == 32'h0040_eac8) ||
        (cpu_debug_core_rf_pc == 32'h0040_eacc) ||
        (cpu_debug_core_rf_pc == 32'h0040_eaf0) ||
        (cpu_debug_core_wb_pc == 32'h0040_8a60) ||
        (cpu_debug_core_wb_pc == 32'h0040_8a64) ||
        (cpu_debug_core_wb_pc == 32'h0040_eac8) ||
        (cpu_debug_core_wb_pc == 32'h0040_eacc) ||
        (cpu_debug_core_wb_pc == 32'h0040_eaf0);
    wire [4:0] cpu_debug_core_exc_code = cpu_debug_core_cause[6:2];
    wire user_high_tlbl_live =
        (cpu_debug_core_exc_code == 5'h02) &&
        (cpu_debug_core_epc[31:28] == 4'h0) &&
        (cpu_debug_core_badvaddr[31:28] == 4'hf);
    wire fsck_badptr_tlbl_live =
        user_high_tlbl_live &&
        (cpu_debug_core_epc >= 32'h0040_0000) &&
        (cpu_debug_core_epc < 32'h0042_0000) &&
        (cpu_debug_core_badvaddr == 32'hffb6_99b0);
    wire fsck_or_libc_user_epc =
        ((cpu_debug_core_epc >= 32'h0040_0000) &&
         (cpu_debug_core_epc <  32'h0042_0000)) ||
        ((cpu_debug_core_epc >= 32'h0fab_0000) &&
         (cpu_debug_core_epc <  32'h0fb6_0000));
    wire fsck_user_bus_fault_live =
        fsck_or_libc_user_epc && fsck_process_seen &&
        ((cpu_debug_core_exc_code == 5'h04) ||  // AdEL -> SIGBUS
         (cpu_debug_core_exc_code == 5'h05) ||  // AdES -> SIGBUS
         (cpu_debug_core_exc_code == 5'h06) ||  // IBE
         (cpu_debug_core_exc_code == 5'h07) ||  // DBE
         (cpu_debug_core_exc_code == 5'h0a) ||  // RI
         (cpu_debug_core_exc_code == 5'h0c) ||  // overflow
         (cpu_debug_core_exc_code == 5'h0d));   // trap
    wire fsck_or_libc_data_pc =
        ((cpu_debug_core_store_pc >= 32'h0040_0000) &&
         (cpu_debug_core_store_pc <  32'h0042_0000)) ||
        ((cpu_debug_core_store_pc >= 32'h0fab_0000) &&
         (cpu_debug_core_store_pc <  32'h0fb6_0000));
    // Keep the most recent data-TLB fault issued by any fsck sproc worker or
    // its libc.  A single ASID is insufficient here: Phase 1 uses several
    // workers, and a normal demand-refill miss in the first worker must not
    // permanently bind the capture to that worker.
    wire fsck_data_tlb_fault_live =
        cpu_debug_dtlb_state[27] &&          // data-stage valid
        cpu_debug_dtlb_state[22] &&          // DTLB exception
        fsck_or_libc_data_pc &&
        (fsck_process_seen || fsck_phase1_pc_live);
    wire fsck_data_tlbl_live =
        fsck_data_tlb_fault_live && cpu_debug_dtlb_state[26];
    wire fsck_data_tlbl_cylpass1 =
        fsck_data_tlbl_live &&
        (cpu_debug_core_store_pc == 32'h0040_eaf0);
    wire fsck_data_tlbl_ckinode =
        fsck_data_tlbl_live &&
        (cpu_debug_core_store_pc == 32'h0040_f904);

    wire fsck_ginode_ret_is =
        (cpu_debug_core_is_pc == 32'h0040_8a60) ||
        (cpu_debug_core_is_pc == 32'h0040_8a64);
    wire fsck_cylpass1_ret_is =
        (cpu_debug_core_is_pc == 32'h0040_eac8) ||
        (cpu_debug_core_is_pc == 32'h0040_eacc);
    wire fsck_cylpass1_deref_is =
        (cpu_debug_core_is_pc == 32'h0040_eaf0);
    wire fsck_ginode_ret_rf =
        (cpu_debug_core_rf_pc == 32'h0040_8a60) ||
        (cpu_debug_core_rf_pc == 32'h0040_8a64);
    wire fsck_cylpass1_ret_rf =
        (cpu_debug_core_rf_pc == 32'h0040_eac8) ||
        (cpu_debug_core_rf_pc == 32'h0040_eacc);
    wire fsck_cylpass1_deref_rf =
        (cpu_debug_core_rf_pc == 32'h0040_eaf0);
    wire fsck_ginode_ret_wb =
        (cpu_debug_core_wb_pc == 32'h0040_8a60) ||
        (cpu_debug_core_wb_pc == 32'h0040_8a64);
    wire fsck_cylpass1_ret_wb =
        (cpu_debug_core_wb_pc == 32'h0040_eac8) ||
        (cpu_debug_core_wb_pc == 32'h0040_eacc);
    wire fsck_cylpass1_deref_wb =
        (cpu_debug_core_wb_pc == 32'h0040_eaf0);
    wire fsck_ginode_pc_live =
        fsck_ginode_ret_is ||
        fsck_cylpass1_ret_is ||
        fsck_cylpass1_deref_is ||
        fsck_ginode_ret_rf ||
        fsck_cylpass1_ret_rf ||
        fsck_cylpass1_deref_rf ||
        fsck_ginode_ret_wb ||
        fsck_cylpass1_ret_wb ||
        fsck_cylpass1_deref_wb;
    wire fsck_ginode_fault_live =
        fsck_data_tlb_fault_live || fsck_user_bus_fault_live;

    // The refill path reaches 0x8800c9ac only when the mapping routine returns
    // nonzero. The exception path then turns the failed
    // user mapping into a signal.  This is the useful discriminator between
    // an ordinary demand-page refill and the miss which actually kills fsck.
    // CP0 EPC/BadVAddr still describe the original user access at this point.
    wire fsck_tlb_fatal_pc_live =
        (cpu_debug_core_is_pc == 32'h8800_c9ac) ||
        (cpu_debug_core_is_pc == 32'h8800_c9b0) ||
        (cpu_debug_core_is_pc == 32'h8800_c9b4) ||
        (cpu_debug_core_rf_pc == 32'h8800_c9ac) ||
        (cpu_debug_core_rf_pc == 32'h8800_c9b0) ||
        (cpu_debug_core_rf_pc == 32'h8800_c9b4) ||
        (cpu_debug_core_wb_pc == 32'h8800_c9ac) ||
        (cpu_debug_core_wb_pc == 32'h8800_c9b0) ||
        (cpu_debug_core_wb_pc == 32'h8800_c9b4);
    wire fsck_tlb_fatal_live =
        fsck_process_seen &&
        fsck_tlb_fatal_pc_live &&
        fsck_or_libc_user_epc &&
        ((cpu_debug_core_exc_code == 5'h02) ||
         (cpu_debug_core_exc_code == 5'h03));
    wire fsck_ginode_capture_live =
        fsck_tlb_fatal_live || fsck_ginode_fault_live || fsck_ginode_pc_live;

    wire [5:0] fsck_ginode_live_event =
        fsck_data_tlbl_cylpass1 ? 6'd22 :
        fsck_data_tlbl_ckinode  ? 6'd23 :
        fsck_data_tlbl_live     ? 6'd24 :
        (fsck_user_bus_fault_live &&
         (cpu_debug_core_exc_code == 5'h04)) ? 6'd25 :
        (fsck_user_bus_fault_live &&
         (cpu_debug_core_exc_code == 5'h05)) ? 6'd26 :
        (fsck_user_bus_fault_live &&
         (cpu_debug_core_exc_code == 5'h06)) ? 6'd27 :
        (fsck_user_bus_fault_live &&
         (cpu_debug_core_exc_code == 5'h07)) ? 6'd28 :
        (fsck_user_bus_fault_live &&
         (cpu_debug_core_exc_code == 5'h0a)) ? 6'd29 :
        (fsck_user_bus_fault_live &&
         (cpu_debug_core_exc_code == 5'h0c)) ? 6'd30 :
        (fsck_user_bus_fault_live &&
         (cpu_debug_core_exc_code == 5'h0d)) ? 6'd31 :
        (fsck_data_tlb_fault_live &&
         cpu_debug_dtlb_state[25]) ? 6'd32 :
        fsck_data_tlb_fault_live    ? 6'd33 :
        (cpu_debug_core_wb_pc == 32'h0040_8a60) ? 6'd1 :
        (cpu_debug_core_wb_pc == 32'h0040_8a64) ? 6'd2 :
        (cpu_debug_core_wb_pc == 32'h0040_eac8) ? 6'd3 :
        (cpu_debug_core_wb_pc == 32'h0040_eacc) ? 6'd4 :
        (cpu_debug_core_wb_pc == 32'h0040_eaf0) ? 6'd5 :
        (cpu_debug_core_rf_pc == 32'h0040_8a60) ? 6'd9 :
        (cpu_debug_core_rf_pc == 32'h0040_8a64) ? 6'd10 :
        (cpu_debug_core_rf_pc == 32'h0040_eac8) ? 6'd11 :
        (cpu_debug_core_rf_pc == 32'h0040_eacc) ? 6'd12 :
        (cpu_debug_core_rf_pc == 32'h0040_eaf0) ? 6'd13 :
        (cpu_debug_core_is_pc == 32'h0040_8a60) ? 6'd17 :
        (cpu_debug_core_is_pc == 32'h0040_8a64) ? 6'd18 :
        (cpu_debug_core_is_pc == 32'h0040_eac8) ? 6'd19 :
        (cpu_debug_core_is_pc == 32'h0040_eacc) ? 6'd20 :
        (cpu_debug_core_is_pc == 32'h0040_eaf0) ? 6'd21 :
        fsck_badptr_tlbl_live    ? 6'h3f :
                                                  6'd0;
    wire [31:0] fsck_ginode_live_pc =
        fsck_data_tlb_fault_live ? cpu_debug_core_store_pc :
        fsck_user_bus_fault_live ? cpu_debug_core_epc :
        fsck_cylpass1_deref_wb || fsck_cylpass1_ret_wb ||
        fsck_ginode_ret_wb ? cpu_debug_core_wb_pc :
        fsck_cylpass1_deref_rf || fsck_cylpass1_ret_rf ||
        fsck_ginode_ret_rf ? cpu_debug_core_rf_pc :
        fsck_cylpass1_deref_is || fsck_cylpass1_ret_is ||
        fsck_ginode_ret_is ? cpu_debug_core_is_pc :
                             cpu_debug_core_epc;
    wire [31:0] fsck_ginode_live_vaddr =
        fsck_data_tlb_fault_live ? cpu_debug_core_store_vaddr :
                              cpu_debug_core_badvaddr;
    wire [31:0] fsck_ginode_live_insn =
        fsck_data_tlb_fault_live ? cpu_debug_core_store_insn :
        fsck_user_bus_fault_live ? 32'd0 :
        fsck_cylpass1_deref_wb || fsck_cylpass1_ret_wb ||
        fsck_ginode_ret_wb ? cpu_debug_core_wb_insn :
        fsck_cylpass1_deref_rf || fsck_cylpass1_ret_rf ||
        fsck_ginode_ret_rf ? cpu_debug_core_rf_insn :
                             32'd0;

    reg        fsck_ginode_seen;
    reg        fsck_ginode_fault_seen;
    reg [5:0]  fsck_ginode_event;
    reg [31:0] fsck_ginode_pc;
    reg [31:0] fsck_ginode_vaddr;
    reg [31:0] fsck_ginode_insn;
    reg [31:0] fsck_ginode_v0;
    reg [31:0] fsck_ginode_v1;
    reg [31:0] fsck_ginode_at;
    reg [31:0] fsck_ginode_t0;
    reg [31:0] fsck_ginode_t1;
    reg [31:0] fsck_ginode_t2;
    reg [31:0] fsck_ginode_t3;
    reg [31:0] fsck_ginode_t4;
    reg [31:0] fsck_ginode_t6;
    reg [31:0] fsck_ginode_a0;
    reg [31:0] fsck_ginode_a1;
    reg [31:0] fsck_ginode_a2;
    reg [31:0] fsck_ginode_a3;
    reg [31:0] fsck_ginode_s0;
    reg [31:0] fsck_ginode_s1;
    reg [31:0] fsck_ginode_t7;
    reg [31:0] fsck_ginode_t8;
    reg [31:0] fsck_ginode_t9;
    reg [31:0] fsck_ginode_sp;
    reg [31:0] fsck_ginode_ra;
    reg [31:0] fsck_ginode_last_cf_pc;
    reg [31:0] fsck_ginode_last_cf_target;
    reg [31:0] fsck_ginode_cause;
    reg [23:0] fsck_ginode_dtlb_state;
    reg [7:0]  fsck_ginode_asid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsck_process_seen <= 1'b0;
            fsck_process_asid <= 8'd0;
            fsck_ginode_seen <= 1'b0;
            fsck_ginode_fault_seen <= 1'b0;
            fsck_ginode_event <= 6'd0;
            fsck_ginode_pc <= 32'd0;
            fsck_ginode_vaddr <= 32'd0;
            fsck_ginode_insn <= 32'd0;
            fsck_ginode_v0 <= 32'd0;
            fsck_ginode_v1 <= 32'd0;
            fsck_ginode_at <= 32'd0;
            fsck_ginode_t0 <= 32'd0;
            fsck_ginode_t1 <= 32'd0;
            fsck_ginode_t2 <= 32'd0;
            fsck_ginode_t3 <= 32'd0;
            fsck_ginode_t4 <= 32'd0;
            fsck_ginode_t6 <= 32'd0;
            fsck_ginode_a0 <= 32'd0;
            fsck_ginode_a1 <= 32'd0;
            fsck_ginode_a2 <= 32'd0;
            fsck_ginode_a3 <= 32'd0;
            fsck_ginode_s0 <= 32'd0;
            fsck_ginode_s1 <= 32'd0;
            fsck_ginode_t7 <= 32'd0;
            fsck_ginode_t8 <= 32'd0;
            fsck_ginode_t9 <= 32'd0;
            fsck_ginode_sp <= 32'd0;
            fsck_ginode_ra <= 32'd0;
            fsck_ginode_last_cf_pc <= 32'd0;
            fsck_ginode_last_cf_target <= 32'd0;
            fsck_ginode_cause <= 32'd0;
            fsck_ginode_dtlb_state <= 24'd0;
            fsck_ginode_asid <= 8'd0;
        end else if (!soc_rst_n) begin
            fsck_process_seen <= 1'b0;
            fsck_process_asid <= 8'd0;
            fsck_ginode_seen <= 1'b0;
            fsck_ginode_fault_seen <= 1'b0;
            fsck_ginode_event <= 6'd0;
            fsck_ginode_pc <= 32'd0;
            fsck_ginode_vaddr <= 32'd0;
            fsck_ginode_insn <= 32'd0;
            fsck_ginode_v0 <= 32'd0;
            fsck_ginode_v1 <= 32'd0;
            fsck_ginode_at <= 32'd0;
            fsck_ginode_t0 <= 32'd0;
            fsck_ginode_t1 <= 32'd0;
            fsck_ginode_t2 <= 32'd0;
            fsck_ginode_t3 <= 32'd0;
            fsck_ginode_t4 <= 32'd0;
            fsck_ginode_t6 <= 32'd0;
            fsck_ginode_a0 <= 32'd0;
            fsck_ginode_a1 <= 32'd0;
            fsck_ginode_a2 <= 32'd0;
            fsck_ginode_a3 <= 32'd0;
            fsck_ginode_s0 <= 32'd0;
            fsck_ginode_s1 <= 32'd0;
            fsck_ginode_t7 <= 32'd0;
            fsck_ginode_t8 <= 32'd0;
            fsck_ginode_t9 <= 32'd0;
            fsck_ginode_sp <= 32'd0;
            fsck_ginode_ra <= 32'd0;
            fsck_ginode_last_cf_pc <= 32'd0;
            fsck_ginode_last_cf_target <= 32'd0;
            fsck_ginode_cause <= 32'd0;
            fsck_ginode_dtlb_state <= 24'd0;
            fsck_ginode_asid <= 8'd0;
        end else begin
            if (fsck_phase1_pc_live && !fsck_ginode_fault_seen) begin
                fsck_process_seen <= 1'b1;
                fsck_process_asid <= cpu_debug_core_entryhi[7:0];
            end
            if (fsck_ginode_capture_live &&
                !fsck_ginode_fault_seen && !fsck_tlb_fatal_live) begin
                fsck_ginode_seen <= 1'b1;
                fsck_ginode_event <= fsck_ginode_live_event;
                fsck_ginode_pc <= fsck_ginode_live_pc;
                fsck_ginode_vaddr <= fsck_ginode_live_vaddr;
                fsck_ginode_insn <= fsck_ginode_live_insn;
                fsck_ginode_v0 <= cpu_debug_core_gpr_v0;
                fsck_ginode_v1 <= cpu_debug_core_gpr_v1;
                fsck_ginode_at <= cpu_debug_core_gpr_at;
                fsck_ginode_t0 <= cpu_debug_core_gpr_t0;
                fsck_ginode_t1 <= cpu_debug_core_gpr_t1;
                fsck_ginode_t2 <= cpu_debug_core_gpr_t2;
                fsck_ginode_t3 <= cpu_debug_core_gpr_t3;
                fsck_ginode_t4 <= cpu_debug_core_gpr_t4;
                fsck_ginode_t6 <= cpu_debug_core_gpr_t6;
                fsck_ginode_a0 <= cpu_debug_core_gpr_a0;
                fsck_ginode_a1 <= cpu_debug_core_gpr_a1;
                fsck_ginode_a2 <= cpu_debug_core_gpr_a2;
                fsck_ginode_a3 <= cpu_debug_core_gpr_a3;
                fsck_ginode_s0 <= cpu_debug_core_gpr_s0;
                fsck_ginode_s1 <= cpu_debug_core_gpr_s1;
                fsck_ginode_t7 <= cpu_debug_core_gpr_t7;
                fsck_ginode_t8 <= cpu_debug_core_gpr_t8;
                fsck_ginode_t9 <= cpu_debug_core_gpr_t9;
                fsck_ginode_sp <= cpu_debug_core_gpr_sp;
                fsck_ginode_ra <= cpu_debug_core_gpr_ra;
                fsck_ginode_last_cf_pc <= cpu_debug_core_last_cf_pc;
                fsck_ginode_last_cf_target <= cpu_debug_core_last_cf_target;
                fsck_ginode_cause <= cpu_debug_core_cause;
                fsck_ginode_dtlb_state <= cpu_debug_dtlb_state[23:0];
                fsck_ginode_asid <= cpu_debug_core_entryhi[7:0];
            end
            if (fsck_tlb_fatal_live && !fsck_ginode_fault_seen) begin
                // Freeze the immediately preceding user DTLB capture.  Do not
                // replace its user registers with the exception trampoline's
                // registers; only mark it as the fatal miss.
                fsck_ginode_seen <= 1'b1;
                fsck_ginode_fault_seen <= 1'b1;
                fsck_ginode_event <= 6'd34;
            end
            if (fsck_badptr_tlbl_live) begin
                if (!fsck_ginode_seen && !fsck_ginode_capture_live &&
                    !fsck_ginode_fault_seen) begin
                    fsck_ginode_seen <= 1'b1;
                    fsck_ginode_event <= 6'h3f;
                    fsck_ginode_pc <= cpu_debug_core_epc;
                    fsck_ginode_vaddr <= cpu_debug_core_badvaddr;
                    fsck_ginode_insn <= 32'd0;
                    fsck_ginode_v0 <= cpu_debug_core_gpr_v0;
                    fsck_ginode_v1 <= cpu_debug_core_gpr_v1;
                    fsck_ginode_at <= cpu_debug_core_gpr_at;
                    fsck_ginode_t0 <= cpu_debug_core_gpr_t0;
                    fsck_ginode_t1 <= cpu_debug_core_gpr_t1;
                    fsck_ginode_t2 <= cpu_debug_core_gpr_t2;
                    fsck_ginode_t3 <= cpu_debug_core_gpr_t3;
                    fsck_ginode_t4 <= cpu_debug_core_gpr_t4;
                    fsck_ginode_t6 <= cpu_debug_core_gpr_t6;
                    fsck_ginode_a0 <= cpu_debug_core_gpr_a0;
                    fsck_ginode_a1 <= cpu_debug_core_gpr_a1;
                    fsck_ginode_a2 <= cpu_debug_core_gpr_a2;
                    fsck_ginode_a3 <= cpu_debug_core_gpr_a3;
                    fsck_ginode_s0 <= cpu_debug_core_gpr_s0;
                    fsck_ginode_s1 <= cpu_debug_core_gpr_s1;
                    fsck_ginode_t7 <= cpu_debug_core_gpr_t7;
                    fsck_ginode_t8 <= cpu_debug_core_gpr_t8;
                    fsck_ginode_t9 <= cpu_debug_core_gpr_t9;
                    fsck_ginode_sp <= cpu_debug_core_gpr_sp;
                    fsck_ginode_ra <= cpu_debug_core_gpr_ra;
                    fsck_ginode_last_cf_pc <= cpu_debug_core_last_cf_pc;
                    fsck_ginode_last_cf_target <= cpu_debug_core_last_cf_target;
                    fsck_ginode_cause <= cpu_debug_core_cause;
                    fsck_ginode_dtlb_state <= cpu_debug_dtlb_state[23:0];
                    fsck_ginode_asid <= cpu_debug_core_entryhi[7:0];
                end
            end
        end
    end

    wire [255:0] cpu_gpr_arg_meta_debug_probe = {
        32'h4732_4130, // "G2A0"
        {fsck_ginode_seen, fsck_ginode_fault_seen, fsck_ginode_capture_live,
         fsck_ginode_seen ? fsck_ginode_event : fsck_ginode_live_event,
         fsck_ginode_seen ? fsck_ginode_dtlb_state[22:0] :
                            cpu_debug_dtlb_state[22:0]},
        fsck_ginode_seen ? fsck_ginode_pc : fsck_ginode_live_pc,
        fsck_ginode_seen ? fsck_ginode_vaddr : fsck_ginode_live_vaddr,
        fsck_ginode_seen ? fsck_ginode_insn : fsck_ginode_live_insn,
        fsck_ginode_seen ? fsck_ginode_last_cf_pc : cpu_debug_core_last_cf_pc,
        fsck_ginode_seen ? fsck_ginode_last_cf_target : cpu_debug_core_last_cf_target,
        fsck_ginode_seen ? fsck_ginode_cause : cpu_debug_core_cause
    };

    wire [255:0] cpu_gpr_arg_regs_debug_probe = {
        32'h4732_4330, // "G2C0"
        fsck_ginode_seen ? fsck_ginode_v0 : cpu_debug_core_gpr_v0,
        fsck_ginode_seen ? fsck_ginode_v1 : cpu_debug_core_gpr_v1,
        fsck_ginode_seen ? fsck_ginode_a0 : cpu_debug_core_gpr_a0,
        fsck_ginode_seen ? fsck_ginode_a1 : cpu_debug_core_gpr_a1,
        fsck_ginode_seen ? fsck_ginode_a2 : cpu_debug_core_gpr_a2,
        fsck_ginode_seen ? fsck_ginode_a3 : cpu_debug_core_gpr_a3,
        fsck_ginode_seen ? fsck_ginode_s0 : cpu_debug_core_gpr_s0
    };

    wire [255:0] cpu_gpr_temp_regs_debug_probe = {
        32'h4732_4430, // "G2D0"
        fsck_ginode_seen ? fsck_ginode_at : cpu_debug_core_gpr_at,
        fsck_ginode_seen ? fsck_ginode_t0 : cpu_debug_core_gpr_t0,
        fsck_ginode_seen ? fsck_ginode_t1 : cpu_debug_core_gpr_t1,
        fsck_ginode_seen ? fsck_ginode_t2 : cpu_debug_core_gpr_t2,
        fsck_ginode_seen ? fsck_ginode_t3 : cpu_debug_core_gpr_t3,
        fsck_ginode_seen ? fsck_ginode_t4 : cpu_debug_core_gpr_t4,
        fsck_ginode_seen ? fsck_ginode_t6 : cpu_debug_core_gpr_t6
    };

    wire [255:0] cpu_gpr_saved_regs_debug_probe = {
        32'h4732_4530, // "G2E0"
        fsck_ginode_seen ? fsck_ginode_t7 : cpu_debug_core_gpr_t7,
        fsck_ginode_seen ? fsck_ginode_t8 : cpu_debug_core_gpr_t8,
        fsck_ginode_seen ? fsck_ginode_t9 : cpu_debug_core_gpr_t9,
        fsck_ginode_seen ? fsck_ginode_s1 : cpu_debug_core_gpr_s1,
        {24'd0, fsck_ginode_seen ? fsck_ginode_asid :
                                  cpu_debug_core_entryhi[7:0]},
        fsck_ginode_seen ? fsck_ginode_sp : cpu_debug_core_gpr_sp,
        fsck_ginode_seen ? fsck_ginode_ra : cpu_debug_core_gpr_ra
    };

    // Live userland context for the post-remount pure-user runaway class.
    // Keep this combinational and observation-only: the two VIO payloads let
    // hardware debug distinguish a genuine control-flow loop from stale
    // cache/TLB state without changing CPU-visible behavior.
    wire [255:0] cpu_make_live_meta_debug_probe = {
        32'h4D4B_5330, // "MKS0"
        cpu_debug_core_pc,
        cpu_debug_core_rf_pc,
        cpu_debug_core_rf_insn,
        cpu_debug_core_wb_pc,
        cpu_debug_core_wb_insn,
        cpu_debug_core_last_cf_pc,
        cpu_debug_core_last_cf_target
    };

    wire [255:0] cpu_make_live_regs_debug_probe = {
        32'h4D4B_5331, // "MKS1"
        cpu_debug_core_gpr_v0,
        cpu_debug_core_gpr_v1,
        cpu_debug_core_gpr_a0,
        cpu_debug_core_gpr_a1,
        cpu_debug_core_gpr_t9,
        cpu_debug_core_gpr_sp,
        cpu_debug_core_gpr_ra
    };

    wire [255:0] cpu_make_live_temps_debug_probe = {
        32'h4D4B_5332, // "MKS2"
        cpu_debug_core_gpr_at,
        cpu_debug_core_gpr_t0,
        cpu_debug_core_gpr_t1,
        cpu_debug_core_gpr_t2,
        cpu_debug_core_gpr_t3,
        cpu_debug_core_gpr_t4,
        cpu_debug_core_gpr_t6
    };

    // The default profile keeps SCSI/SD plus SCC waveform/history visibility.
    // VIO bit 24 switches the payloads to CPU/cache state.  Bit 25 alone adds
    // DSP state; when both are set, expose the persistent fsck fault/provenance
    // latch so a short-lived user fault remains visible after signal delivery.
    assign cpu_flow_debug_probe =
        (cpu_debug_probe_select && dsp_debug_probe_select) ?
            fsck_cache_trace_meta_debug_probe :
        cpu_debug_probe_select ?
        cpu_irq_debug_probe : scsi_hpc_debug_probe_unused;
    assign cpu_exception_debug_probe =
        (cpu_debug_probe_select && dsp_debug_probe_select) ?
            fsck_cache_trace_cpu_debug_probe :
        cpu_debug_probe_select ?
        cpu_post_remount_debug_probe : sd_scsi_debug_probe;
    assign cpu_gpr_debug_probe =
        (cpu_debug_probe_select && dsp_debug_probe_select) ?
            {fsck_cache_trace_dma_debug_probe,
             fsck_cache_trace_load_debug_probe} :
        dsp_debug_probe_select ?
            {dsp_debug_probe, cpu_live_debug_probe} :
        cpu_debug_probe_select ?
            {cpu_live_debug_probe, cpu_post_remount_debug_probe} :
            {scc_tx_history_debug_probe, scc_tx_waveform_debug_probe};
`else
    assign cpu_flow_debug_probe = {
        cpu_debug_core_pc,
        cpu_debug_core_is_pc,
        cpu_debug_core_rf_pc,
        cpu_debug_core_rf_insn,
        cpu_debug_core_wb_pc,
        cpu_debug_core_wb_insn,
        cpu_debug_core_last_cf_pc,
        cpu_debug_core_last_cf_target
    };

    assign cpu_exception_debug_probe = {
        cpu_debug_core_epc,
        cpu_debug_core_cause,
        cpu_debug_core_status,
        cpu_debug_core_badvaddr,
        cpu_debug_core_gpr_a0,
        cpu_debug_core_gpr_a1,
        cpu_debug_core_gpr_t9,
        cpu_debug_core_gpr_ra
    };

    assign cpu_gpr_debug_probe = {
        cpu_debug_core_gpr_v0,
        cpu_debug_core_gpr_v1,
        cpu_debug_core_gpr_a0,
        cpu_debug_core_gpr_a1,
        cpu_debug_core_gpr_a2,
        cpu_debug_core_gpr_a3,
        cpu_debug_core_gpr_t2,
        cpu_debug_core_gpr_t3,
        cpu_debug_core_gpr_t4,
        cpu_debug_core_gpr_t6,
        cpu_debug_core_gpr_t7,
        cpu_debug_core_gpr_t8,
        cpu_debug_core_gpr_t9,
        cpu_debug_core_gpr_sp,
        cpu_debug_core_gpr_ra,
        cpu_dmem_paddr
    };
`endif

    wire [31:0] unused_mc_memcfg0 = mc_memcfg0;
    wire [31:0] unused_cpu_debug_dtlb_state = cpu_debug_dtlb_state;
    wire [31:0] unused_cpu_debug_dtlb_paddr = cpu_debug_dtlb_paddr;
    wire [31:0] unused_cpu_debug_dtlb_lo = cpu_debug_dtlb_lo;
    wire [31:0] unused_cpu_debug_dtlb_mask = cpu_debug_dtlb_mask;
    wire [7:0]  unused_scc_console_tx_data = scc_console_tx_data;
    wire [255:0] unused_enet_hpc_debug_probe = enet_hpc_debug_probe;
endmodule

`default_nettype wire
