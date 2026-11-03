// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - SGI Memory Controller (MC)
// IP20 compatible - big-endian addresses from mc.h
//
// Physical address range: 0x1FA00000 - 0x1FA1FFFF (MC registers)
// Also arbitrates CPU and DMA access to RAM.
//
// Register offsets (big-endian byte addresses):
//   0x04 CPUCTRL0    - CPU control 0 (rw)
//   0x0C CPUCTRL1    - CPU control 1 (rw)
//   0x1C SYSID       - System ID (ro)
//   0x84 GIO64_ARB    - GIO64 device attributes (rw)
//   0x8C CPU_TIME     - CPU long-burst arbitration tenure (rw)
//   0x9C LB_TIME      - non-CPU long-burst arbitration tenure (rw)
//   0xC4 MEMCFG0     - Memory config bank 0-1 (rw)
//   0xCC MEMCFG1     - Memory config bank 2-3 (rw)
//   0xD4 CPU_MEMACC  - CPU memory access timing (rw)
//   0xDC GIO_MEMACC  - GIO memory access timing (rw)
//   0xEC CPU_ERR_STAT - CPU error status (rw)
//   0xF4 GIO_ERROR_ADDR - physical address of first pending GIO error (ro)
//   0xFC GIO_ERR_STAT - GIO error status (rw)
//   0x1004 RPSS_CTR  - Free-running 100ns counter (ro)

`default_nettype none

module mc #(
    parameter RAM_BYTES = 512 * 1024 * 1024, // external RAM backing
    parameter [31:0] LOW_ALIAS_BYTES = 32'h0008_0000 // IP20 aliases low 512KB at SEG0
) (
    input  wire        clk,
    input  wire        rst_n,

    // CPU port (from bus mux — MC regs + RAM accesses)
    input  wire [35:0] cpu_addr,
    input  wire [63:0] cpu_wdata,
    output wire [63:0] cpu_rdata,
    input  wire [ 7:0] cpu_wmask,
    input  wire        cpu_valid,
    input  wire        cpu_write,
    // The S-cache has already serialized this external fill/writeback ahead
    // of a coherence probe. Such work must drain so a raw DMA-read probe can
    // resolve; ordinary CPU requests leave this low and cannot enter first.
    input  wire        cpu_cache_owned,
    output wire        cpu_ready,

    // DMA port (from HPC1 SCSI DMA engine)
    input  wire [31:0] dma_addr,
    input  wire [63:0] dma_wdata,
    output reg  [63:0] dma_rdata,
    input  wire [ 7:0] dma_wmask,
    // Raw real-time request, asserted before any external coherence lookup.
    // dma_valid qualifies the resolved transfer presented for service.
    input  wire        dma_request,
    input  wire        dma_valid,
    input  wire        dma_write,
    output reg         dma_ready,

    // IP20 MC EEROM serial register at 0x1FA00030.
    output wire        eeprom_cs,
    output wire        eeprom_clk,
    output wire        eeprom_di,
    input  wire        eeprom_do,

    // Unified MC GIO master port. CPU programmed I/O and MC virtual DMA
    // share this physical boundary; source_vdma identifies the owner for
    // observability only and never changes while valid is asserted.
    output reg  [31:0] vdma_gio_addr,
    output reg  [63:0] vdma_gio_wdata,
    input  wire [63:0] vdma_gio_rdata,
    output reg  [ 7:0] vdma_gio_wmask,
    output reg         vdma_gio_valid,
    output reg         vdma_gio_write,
    output reg         vdma_gio_source_vdma,
    input  wire        vdma_gio_ready,

    // RAM port (to Verilog RAM array in testbench / SDRAM on FPGA)
    output reg  [25:0] ram_addr,     // doubleword address (26 bits = 64M DWs = 512MB)
    output reg  [63:0] ram_wdata,
    output reg  [ 7:0] ram_wmask,
    output reg         ram_wen,
    output reg         ram_valid,
    input  wire        ram_ready,
    input  wire [63:0] ram_rdata,    // combinational read

    // Unacknowledged GIO transaction notification. CPUCTRL1.ABORT_EN decides
    // whether the transaction becomes a TIME error.
    input  wire        gio_access,

    // Architectural GIO timeout enable for the system completion boundary.
    // When clear, a genuinely unacknowledged GIO transaction does not
    // complete; when set, the system may return the timeout response.
    output wire        gio_abort_en,

    // MEMCFG output (for external visibility / SIMM probe logic)
    output wire [31:0] memcfg0_out,

    // Bus error interrupt (asserted while GIO/CPU error bits are set)
    output wire        bus_err_irq,

    // Qualified virtual-DMA interrupt. IP20 routes this to INT2 LIO0 bit 4;
    // completion is enabled by DMA_CTL.IntMask while fault causes are
    // unconditional. Software acknowledges the level by writing DMA_CAUSE.
    output wire        gfx_dma_irq,

    // Debug snapshot of RAM response attribution.
    output wire [479:0] debug_probe
);

    // ===========================================================================
    // Address Constants
    // ===========================================================================

    localparam MC_BASE = 36'h1FA00000;
    localparam MC_END  = 36'h1FA20000;

    localparam OFF_CPUCTRL0     = 16'h0004;
    localparam OFF_CPUCTRL1     = 16'h000C;
    localparam OFF_SYSID        = 16'h001C;
    localparam OFF_GIO64_ARB    = 16'h0080;
    localparam OFF_CPU_TIME     = 16'h0088;
    localparam OFF_LB_TIME      = 16'h0098;
    localparam OFF_MEMCFG0      = 16'h00C4;
    localparam OFF_MEMCFG1      = 16'h00CC;
    localparam OFF_CPU_MEMACC   = 16'h00D4;
    localparam OFF_GIO_MEMACC   = 16'h00DC;
    localparam OFF_CPU_ERR_STAT = 16'h00EC;
    localparam OFF_GIO_ERR_ADDR = 16'h00F4;
    localparam OFF_GIO_ERR_STAT = 16'h00FC;
    localparam OFF_RPSS_CTR     = 16'h1004;
    // The CPU uses big-endian word addresses (+4 within each 64-bit slot).
    // aligned_offset strips addr[2:0], so constants below are 64-bit slots.
    localparam OFF_DMA_GIO_MASK = 16'h0150;
    localparam OFF_DMA_GIO_SUB  = 16'h0158;
    localparam OFF_DMA_CAUSE    = 16'h0160;
    localparam OFF_DMA_CTL      = 16'h0168;
    localparam OFF_DMA_TLB_HI_0 = 16'h0180;
    localparam OFF_DMA_TLB_LO_0 = 16'h0188;
    localparam OFF_DMA_TLB_HI_1 = 16'h0190;
    localparam OFF_DMA_TLB_LO_1 = 16'h0198;
    localparam OFF_DMA_TLB_HI_2 = 16'h01A0;
    localparam OFF_DMA_TLB_LO_2 = 16'h01A8;
    localparam OFF_DMA_TLB_HI_3 = 16'h01B0;
    localparam OFF_DMA_TLB_LO_3 = 16'h01B8;
    localparam OFF_DMA_MEMADR   = 16'h2000;
    localparam OFF_DMA_MEMADRD  = 16'h2008;
    localparam OFF_DMA_SIZE     = 16'h2010;
    localparam OFF_DMA_STRIDE   = 16'h2018;
    localparam OFF_DMA_GIO_ADR  = 16'h2020;
    localparam OFF_DMA_GIO_ADRS = 16'h2028;
    localparam OFF_DMA_MODE     = 16'h2030;
    localparam OFF_DMA_COUNT    = 16'h2038;
    localparam OFF_DMA_STDMA    = 16'h2040;
    localparam OFF_DMA_RUN      = 16'h2048;
    localparam OFF_DMA_MEMADRDS = 16'h2070;

    localparam VDMA_M_GTOH      = 32'h0000_0002;
    localparam VDMA_M_FILL      = 32'h0000_0008;
    localparam VDMA_M_INCA      = 32'h0000_0010;
    localparam VDMA_C_XLATE     = 32'h0000_0100;
    localparam VDMA_R_PAGEFAULT = 32'h0000_0001;
    localparam VDMA_R_UTLBMISS  = 32'h0000_0002;
    localparam VDMA_R_CLEAN     = 32'h0000_0004;
    localparam VDMA_R_COMPLETE  = 32'h0000_0008;
    localparam VDMA_R_BPTE      = 32'h0000_0020;
    localparam VDMA_R_RUNNING   = 32'h0000_0040;
    // ===========================================================================
    // Registers
    // ===========================================================================

    reg [31:0] cpuctrl0;
    reg [31:0] cpuctrl1;
    reg [31:0] gio64_arb;
    reg [15:0] cpu_time;
    reg [15:0] lb_time;
    reg [31:0] memcfg0;
    reg [31:0] memcfg1;
    reg [31:0] cpu_memacc;
    reg [31:0] gio_memacc;
    reg [31:0] cpu_err_stat;
    reg [31:0] gio_err_addr;
    reg [31:0] gio_err_stat;
    reg [31:0] rpss_counter;
    reg [1:0]  rpss_divider_cnt;
    reg [31:0] eerom_reg;
    reg [63:0] cpu_rdata_q;
    reg        cpu_ready_r;
    reg [35:0] cpu_req_addr_q;
    reg [63:0] cpu_req_wdata_q;
    reg [ 7:0] cpu_req_wmask_q;
    reg        cpu_req_write_q;

    // The original MC/MUX boundary contains a 15-entry CPU command/address
    // FIFO and a write-data FIFO. CPU GIO writes are posted into this queue;
    // reads and later CPU operations cannot pass an older queued write.
    reg [31:0] cpu_gio_fifo_addr [0:14];
    reg [63:0] cpu_gio_fifo_wdata [0:14];
    reg [ 7:0] cpu_gio_fifo_wmask [0:14];
    reg [ 3:0] cpu_gio_fifo_wr_ptr;
    reg [ 3:0] cpu_gio_fifo_rd_ptr;
    reg [ 4:0] cpu_gio_fifo_count;

    reg [31:0] dma_gio_mask;
    reg [31:0] dma_gio_sub;
    reg [31:0] dma_cause;
    reg [31:0] dma_ctl;
    reg [31:0] dma_tlb_hi [0:3];
    reg [31:0] dma_tlb_lo [0:3];
    reg [31:0] vdma_memadr;
    reg [31:0] vdma_size;
    reg [31:0] vdma_stride;
    reg [31:0] vdma_gio_adr;
    reg [31:0] vdma_mode;
    reg [31:0] vdma_count;
    reg        vdma_running;
    reg [31:0] vdma_cur_mem;
    reg [31:0] vdma_phys_mem;
    reg [31:0] vdma_line_start_mem;
    reg [15:0] vdma_width_rem;
    reg [15:0] vdma_lines_rem;
    reg [15:0] vdma_zoom_rem;
    reg [31:0] vdma_pack_data;
    reg [3:0]  vdma_pack_mask;
    reg        vdma_packet_line_done;
    reg [31:0] vdma_read_word;
    reg [1:0]  vdma_read_lane;
    reg [3:0]  vdma_fill_step;
    reg [4:0]  vdma_after_xlate;
    reg        vdma_xlate_write;
    reg [31:0] vdma_xlate_pte_addr;
    reg        vdma_xlate_page_valid;
    reg [31:0] vdma_xlate_vpage;
    reg [31:0] vdma_xlate_ppage;
    reg        vdma_xlate_page_writable;
    reg        vdma_fault_latched;

    // IP20 has one memory/GIO arbiter.  HPC is an always-real-time requester;
    // CPU and the MC graphics-DMA master alternate programmable long-burst
    // windows.  Real-time service pauses, rather than consumes, the active
    // long-burst tenure.
    reg        arb_owner_vdma;
    reg [15:0] arb_time_left;
    reg [1:0]  arb_gio_phase;
    reg        vdma_resume_pending;
    reg [4:0]  vdma_resume_state;

`ifndef SYNTHESIS
    reg trace_vdma;
    reg [31:0] vdma_trace_count;
    reg [31:0] vdma_trace_limit;
    initial begin
        trace_vdma = 1'b0;
        vdma_trace_limit = 32'd256;
        if ($test$plusargs("trace_vdma"))
            trace_vdma = 1'b1;
        void'($value$plusargs("trace_vdma_limit=%d", vdma_trace_limit));
    end
`endif

    localparam EEROM_SI = 32'h00000010;
    localparam [31:0] SYSID_VALUE = 32'h00000003;  // IP20

    // Default: single-bank 128 MB — matches DE2i-150 FPGA target.
    // Multi-bank infrastructure below is kept intact (correctness for the
    // PROM walking-bit probe); extra banks can be re-enabled by changing
    // BANK1/2_SIMM_BYTES + MEMCFG0/1_DEFAULT.
    // Per-bank: [13]=VLD, [12:8]=size_mask, [7:0]=base_4MB
    // 128MB = 4MB * (1 + 31) → size_mask=0x1F, VLD=1 → 0x3Fxx (xx=base)
    //   bank 0: 0x3F20  (base 0x20 = 128MB phys offset)
    //   bank 1: 0x0000  (absent)
    //   bank 2: 0x0000  (absent)
    //   bank 3: 0x0000  (absent — IP20 max is 3 banks anyway)
    // PROM still probes during POST; this default gives the FPGA-accurate
    // pre-probe state and matches the layout software sees post-probe.
    localparam [31:0] MEMCFG0_DEFAULT = 32'h3F200000;
    localparam [31:0] MEMCFG1_DEFAULT = 32'h00000000;

    assign memcfg0_out = memcfg0;
    assign gio_abort_en = cpuctrl1[4];

    // Bus error interrupt: asserted while any error status bits are set
    // On real IP20 MC, GIO timeout sets GIO_ERR_STAT_TIME and asserts bus error IRQ
    // IRQ stays asserted until software clears the error status register.
    assign bus_err_irq = |gio_err_stat[15:8] | |cpu_err_stat[15:8];
    // DMA_CAUSE.COMPLETE is always poll-visible, but only interrupts when
    // DMA_CTL.IntMask (bit 4) is set. Translation/PTE faults interrupt
    // independently of that mask. RUNNING is synthesized in DMA_RUN and is
    // deliberately not an interrupt source.
    assign gfx_dma_irq = |(dma_cause & 32'h0000_0027) |
                         (dma_cause[3] & dma_ctl[4]);

    // ===========================================================================
    // Address Decode
    // ===========================================================================

    wire mc_select = (cpu_addr >= MC_BASE) && (cpu_addr < MC_END);
    wire [15:0] reg_offset = cpu_addr[15:0];
    wire [15:0] aligned_offset = {reg_offset[15:3], 3'b000};
    wire [31:0] reg_wdata32 = cpu_addr[2] ? cpu_wdata[31:0] : cpu_wdata[63:32];
    wire [3:0]  reg_wmask32 = cpu_addr[2] ? cpu_wmask[3:0] : cpu_wmask[7:4];
    wire [31:0] dma_run_status = dma_cause | (vdma_running ? VDMA_R_RUNNING : 32'h0);
    assign eeprom_cs  = eerom_reg[1];
    assign eeprom_clk = eerom_reg[2];
    assign eeprom_di  = eerom_reg[3];

    wire [31:0] eerom_read_word =
        (eerom_reg & ~EEROM_SI) | (eeprom_do ? EEROM_SI : 32'h0);
    wire [63:0] eerom_rdata = {eerom_read_word, eerom_read_word};
    wire        mc_reg_read_bypass = cpu_valid && mc_select && !cpu_write;

    function [31:0] merge32;
        input [31:0] old_value;
        input [31:0] new_value;
        input [3:0] mask;
        begin
            merge32 = old_value;
            if (mask[3]) merge32[31:24] = new_value[31:24];
            if (mask[2]) merge32[23:16] = new_value[23:16];
            if (mask[1]) merge32[15:8]  = new_value[15:8];
            if (mask[0]) merge32[7:0]   = new_value[7:0];
        end
    endfunction

    function [31:0] merge32_either_lane;
        input [31:0] old_value;
        input [63:0] new_value;
        input [7:0]  mask;
        reg [31:0] merged;
        begin
            merged = merge32(old_value, new_value[63:32], mask[7:4]);
            merge32_either_lane = merge32(merged, new_value[31:0], mask[3:0]);
        end
    endfunction

    wire [31:0] vdma_size_wdata = merge32(vdma_size, reg_wdata32, reg_wmask32);
    wire [31:0] vdma_stride_wdata = merge32(vdma_stride, reg_wdata32, reg_wmask32);
    wire [31:0] vdma_gio_adrs_wdata = merge32(vdma_gio_adr, reg_wdata32, reg_wmask32);
    wire [31:0] gio64_arb_wdata =
        merge32_either_lane(gio64_arb, cpu_wdata, cpu_wmask);
    wire [31:0] cpu_time_wdata =
        merge32_either_lane({16'h0, cpu_time}, cpu_wdata, cpu_wmask);
    wire [31:0] lb_time_wdata =
        merge32_either_lane({16'h0, lb_time}, cpu_wdata, cpu_wmask);

    function [7:0] ram_byte_at;
        input [63:0] data;
        input [2:0]  offset;
        begin
            case (offset)
                3'd0: ram_byte_at = data[63:56];
                3'd1: ram_byte_at = data[55:48];
                3'd2: ram_byte_at = data[47:40];
                3'd3: ram_byte_at = data[39:32];
                3'd4: ram_byte_at = data[31:24];
                3'd5: ram_byte_at = data[23:16];
                3'd6: ram_byte_at = data[15:8];
                default: ram_byte_at = data[7:0];
            endcase
        end
    endfunction

    function [63:0] ram_byte_wdata;
        input [7:0] data;
        input [2:0] offset;
        begin
            case (offset)
                3'd0: ram_byte_wdata = {data, 56'h0};
                3'd1: ram_byte_wdata = {8'h0, data, 48'h0};
                3'd2: ram_byte_wdata = {16'h0, data, 40'h0};
                3'd3: ram_byte_wdata = {24'h0, data, 32'h0};
                3'd4: ram_byte_wdata = {32'h0, data, 24'h0};
                3'd5: ram_byte_wdata = {40'h0, data, 16'h0};
                3'd6: ram_byte_wdata = {48'h0, data, 8'h0};
                default: ram_byte_wdata = {56'h0, data};
            endcase
        end
    endfunction

    function [7:0] ram_byte_wmask;
        input [2:0] offset;
        begin
            ram_byte_wmask = 8'h80 >> offset;
        end
    endfunction

    function [31:0] ram_word_at;
        input [63:0] data;
        input        upper_word;
        begin
            ram_word_at = upper_word ? data[63:32] : data[31:0];
        end
    endfunction

    function [7:0] vdma_lanes_to_wmask;
        input [3:0] lane_mask;
        begin
            vdma_lanes_to_wmask = {4'h0, lane_mask[0], lane_mask[1],
                                   lane_mask[2], lane_mask[3]};
        end
    endfunction

    function [7:0] vdma_word_lane;
        input [31:0] data;
        input [1:0] lane;
        begin
            case (lane)
                2'd0: vdma_word_lane = data[31:24];
                2'd1: vdma_word_lane = data[23:16];
                2'd2: vdma_word_lane = data[15:8];
                default: vdma_word_lane = data[7:0];
            endcase
        end
    endfunction

    function [31:0] vdma_set_word_lane;
        input [31:0] word;
        input [1:0]  lane;
        input [7:0]  data;
        begin
            case (lane)
                2'd0: vdma_set_word_lane = {data, word[23:0]};
                2'd1: vdma_set_word_lane = {word[31:24], data, word[15:0]};
                2'd2: vdma_set_word_lane = {word[31:16], data, word[7:0]};
                default: vdma_set_word_lane = {word[31:8], data};
            endcase
        end
    endfunction

    function [31:0] vdma_next_byte_addr;
        input [31:0] addr;
        input [31:0] mode;
        begin
            vdma_next_byte_addr = ((mode & VDMA_M_INCA) != 32'h0) ?
                                  (addr + 32'd1) : (addr - 32'd1);
        end
    endfunction

    function [31:0] vdma_advance_addr;
        input [31:0] addr;
        input [31:0] mode;
        input [3:0]  step;
        begin
            vdma_advance_addr = ((mode & VDMA_M_INCA) != 32'h0) ?
                                (addr + {28'h0, step}) :
                                (addr - {28'h0, step});
        end
    endfunction

    function [31:0] vdma_next_line_addr;
        input [31:0] addr;
        input [31:0] stride;
        begin
            vdma_next_line_addr = addr + {{16{stride[15]}}, stride[15:0]};
        end
    endfunction

    function [15:0] vdma_nonzero_zoom;
        input [15:0] zoom;
        begin
            vdma_nonzero_zoom = (zoom == 16'h0) ? 16'd1 : zoom;
        end
    endfunction

    wire [15:0] vdma_yzoom = vdma_nonzero_zoom({6'h0, vdma_stride[25:16]});
    wire [15:0] vdma_count_yzoom = vdma_nonzero_zoom({6'h0, vdma_count[25:16]});
    wire [31:0] vdma_live_count = {6'h0, vdma_zoom_rem[9:0], vdma_width_rem};
    wire [31:0] vdma_count_read = (vdma_running || vdma_fault_latched) ?
                                  vdma_live_count : vdma_count;

    // HTOG presents one 32-bit REX word at a time.  All remaining bytes in
    // that word are necessarily contained in the current aligned 64-bit RAM
    // response: a four-byte boundary cannot cross an eight-byte boundary.
    // Consume them together so pixel-DMA does not issue one DDR read per byte.
    wire        vdma_htog_inc = (vdma_mode & VDMA_M_INCA) != 32'h0;
    wire [2:0]  vdma_htog_word_bytes = vdma_htog_inc ?
                  (3'd4 - {1'b0, vdma_phys_mem[1:0]}) :
                  ({1'b0, vdma_phys_mem[1:0]} + 3'd1);
    wire [2:0]  vdma_htog_step =
                  (vdma_width_rem <= {13'h0, vdma_htog_word_bytes}) ?
                  vdma_width_rem[2:0] : vdma_htog_word_bytes;
    wire [2:0]  vdma_htog_src1 = vdma_htog_inc ?
                  (vdma_phys_mem[2:0] + 3'd1) :
                  (vdma_phys_mem[2:0] - 3'd1);
    wire [2:0]  vdma_htog_src2 = vdma_htog_inc ?
                  (vdma_phys_mem[2:0] + 3'd2) :
                  (vdma_phys_mem[2:0] - 3'd2);
    wire [2:0]  vdma_htog_src3 = vdma_htog_inc ?
                  (vdma_phys_mem[2:0] + 3'd3) :
                  (vdma_phys_mem[2:0] - 3'd3);
    wire [1:0]  vdma_htog_lane1 = vdma_htog_inc ?
                  (vdma_phys_mem[1:0] + 2'd1) :
                  (vdma_phys_mem[1:0] - 2'd1);
    wire [1:0]  vdma_htog_lane2 = vdma_htog_inc ?
                  (vdma_phys_mem[1:0] + 2'd2) :
                  (vdma_phys_mem[1:0] - 2'd2);
    wire [1:0]  vdma_htog_lane3 = vdma_htog_inc ?
                  (vdma_phys_mem[1:0] + 2'd3) :
                  (vdma_phys_mem[1:0] - 2'd3);
    wire [31:0] vdma_htog_pack0 = vdma_set_word_lane(
                  vdma_pack_data, vdma_phys_mem[1:0],
                  ram_byte_at(ram_rdata, vdma_phys_mem[2:0]));
    wire [31:0] vdma_htog_pack1 = (vdma_htog_step >= 3'd2) ?
                  vdma_set_word_lane(vdma_htog_pack0, vdma_htog_lane1,
                                     ram_byte_at(ram_rdata, vdma_htog_src1)) :
                  vdma_htog_pack0;
    wire [31:0] vdma_htog_pack2 = (vdma_htog_step >= 3'd3) ?
                  vdma_set_word_lane(vdma_htog_pack1, vdma_htog_lane2,
                                     ram_byte_at(ram_rdata, vdma_htog_src2)) :
                  vdma_htog_pack1;
    wire [31:0] vdma_htog_pack3 = (vdma_htog_step >= 3'd4) ?
                  vdma_set_word_lane(vdma_htog_pack2, vdma_htog_lane3,
                                     ram_byte_at(ram_rdata, vdma_htog_src3)) :
                  vdma_htog_pack2;
    wire [3:0]  vdma_htog_mask0 =
                  vdma_pack_mask | (4'b0001 << vdma_phys_mem[1:0]);
    wire [3:0]  vdma_htog_mask1 = (vdma_htog_step >= 3'd2) ?
                  (vdma_htog_mask0 | (4'b0001 << vdma_htog_lane1)) :
                  vdma_htog_mask0;
    wire [3:0]  vdma_htog_mask2 = (vdma_htog_step >= 3'd3) ?
                  (vdma_htog_mask1 | (4'b0001 << vdma_htog_lane2)) :
                  vdma_htog_mask1;
    wire [3:0]  vdma_htog_mask3 = (vdma_htog_step >= 3'd4) ?
                  (vdma_htog_mask2 | (4'b0001 << vdma_htog_lane3)) :
                  vdma_htog_mask2;

    reg [63:0] mc_reg_rdata;
    always @* begin
        case (aligned_offset)
            16'h0000: mc_reg_rdata = {cpuctrl0, cpuctrl0};
            16'h0008: mc_reg_rdata = {cpuctrl1, cpuctrl1};
            16'h0018: mc_reg_rdata = {SYSID_VALUE, SYSID_VALUE};
            OFF_GIO64_ARB: mc_reg_rdata = {gio64_arb, gio64_arb};
            OFF_CPU_TIME:  mc_reg_rdata = {16'h0, cpu_time, 16'h0, cpu_time};
            OFF_LB_TIME:   mc_reg_rdata = {16'h0, lb_time, 16'h0, lb_time};
            16'h00C0: mc_reg_rdata = {memcfg0, memcfg0};
            16'h00C8: mc_reg_rdata = {memcfg1, memcfg1};
            16'h00D0: mc_reg_rdata = {cpu_memacc, cpu_memacc};
            16'h00D8: mc_reg_rdata = {gio_memacc, gio_memacc};
            16'h00E8: mc_reg_rdata = {cpu_err_stat, cpu_err_stat};
            16'h00F0: mc_reg_rdata = {gio_err_addr, gio_err_addr};
            16'h00F8: mc_reg_rdata = {gio_err_stat, gio_err_stat};
            16'h0030: mc_reg_rdata = eerom_rdata;
            16'h1000: mc_reg_rdata = {rpss_counter, rpss_counter};
            OFF_DMA_GIO_MASK: mc_reg_rdata = {dma_gio_mask, dma_gio_mask};
            OFF_DMA_GIO_SUB:  mc_reg_rdata = {dma_gio_sub, dma_gio_sub};
            OFF_DMA_CAUSE:    mc_reg_rdata = {dma_cause, dma_cause};
            OFF_DMA_CTL:      mc_reg_rdata = {dma_ctl, dma_ctl};
            OFF_DMA_TLB_HI_0: mc_reg_rdata = {dma_tlb_hi[0], dma_tlb_hi[0]};
            OFF_DMA_TLB_LO_0: mc_reg_rdata = {dma_tlb_lo[0], dma_tlb_lo[0]};
            OFF_DMA_TLB_HI_1: mc_reg_rdata = {dma_tlb_hi[1], dma_tlb_hi[1]};
            OFF_DMA_TLB_LO_1: mc_reg_rdata = {dma_tlb_lo[1], dma_tlb_lo[1]};
            OFF_DMA_TLB_HI_2: mc_reg_rdata = {dma_tlb_hi[2], dma_tlb_hi[2]};
            OFF_DMA_TLB_LO_2: mc_reg_rdata = {dma_tlb_lo[2], dma_tlb_lo[2]};
            OFF_DMA_TLB_HI_3: mc_reg_rdata = {dma_tlb_hi[3], dma_tlb_hi[3]};
            OFF_DMA_TLB_LO_3: mc_reg_rdata = {dma_tlb_lo[3], dma_tlb_lo[3]};
            OFF_DMA_MEMADR:   mc_reg_rdata = {vdma_cur_mem, vdma_cur_mem};
            OFF_DMA_MEMADRD:  mc_reg_rdata = {vdma_memadr, vdma_memadr};
            OFF_DMA_SIZE:     mc_reg_rdata = {vdma_size, vdma_size};
            OFF_DMA_STRIDE:   mc_reg_rdata = {vdma_stride, vdma_stride};
            OFF_DMA_GIO_ADR:  mc_reg_rdata = {vdma_gio_adr, vdma_gio_adr};
            OFF_DMA_GIO_ADRS: mc_reg_rdata = {vdma_gio_adr, vdma_gio_adr};
            OFF_DMA_MODE:     mc_reg_rdata = {vdma_mode, vdma_mode};
            OFF_DMA_COUNT:    mc_reg_rdata = {vdma_count_read, vdma_count_read};
            OFF_DMA_STDMA:    mc_reg_rdata = {dma_run_status, dma_run_status};
            OFF_DMA_RUN:      mc_reg_rdata = {dma_run_status, dma_run_status};
            OFF_DMA_MEMADRDS: mc_reg_rdata = {vdma_memadr, vdma_memadr};
            default:          mc_reg_rdata = 64'h0;
        endcase
    end

    wire cpu_ready_req_changed =
        (cpu_addr != cpu_req_addr_q) ||
        (cpu_write != cpu_req_write_q) ||
        (cpu_write && ((cpu_wdata != cpu_req_wdata_q) ||
                       (cpu_wmask != cpu_req_wmask_q)));

    assign cpu_rdata = mc_reg_read_bypass ? mc_reg_rdata : cpu_rdata_q;

    wire [31:0] vdma_effective_gio_addr =
        (vdma_gio_adr & ~dma_gio_mask) | (dma_gio_sub & dma_gio_mask);

    // LG1 occupies the CPU-visible GIO graphics aperture. It must be decoded
    // before RAM: zero-extended 0x1f3fxxxx addresses otherwise also satisfy
    // the broad 512 MB physical-memory comparison.
    wire cpu_gio_select = (cpu_addr[35:15] == 21'h03E7E);

    // RAM region: physical 0x00000000-0x1FFFFFFF (512MB address space, 4 banks)
    wire cpu_is_ram = (cpu_addr[35:29] == 7'h00) && !cpu_gio_select;

    // CPUCTRL1.MC_HWM is encoded as (17 - maximum admitted commands). The
    // physical FIFO has 15 slots, so small encodings saturate at its depth.
    // Target software programs D, admitting exactly four commands.
    wire [4:0] cpu_gio_hwm_raw = 5'd17 - {1'b0, cpuctrl1[3:0]};
    wire [4:0] cpu_gio_hwm = (cpu_gio_hwm_raw > 5'd15) ?
                             5'd15 : cpu_gio_hwm_raw;

    // MEMCFG-based SIMM slot selection + low-memory alias
    //
    // IP20 physical memory map:
    //   0x00000000-0x0007FFFF: alias of SEG0 base, used for exception vectors,
    //                          SPB, and firmware/private vectors.
    //   0x00080000-0x07FFFFFF: EISA/IO hole
    //   0x08000000-0x0FFFFFFF: SEG0 — 128MB SIMM (bank 0, MEMCFG base=0x20)
    //
    // During SIMM probe, PROM sets bank 0 at base=0x20 and bank 1 at base=0x20
    // separately.  Only bank 0 routes to physical SIMM — bank 1/2/3 bus-float.
    // The first 512KB low-memory alias stays accessible regardless of MEMCFG so
    // exception handlers and early firmware data match the documented IP20 map.
    //
    // SIMM ADDRESS ALIASING: on real IP20 MC, the BNK/rank bit changes the
    // address wrap the PROM observes during SIMM probing. A 128 MB dual-rank
    // SIMM aliases every 32 MB while configured as rank 0 (0x3f20), then uses
    // the full 128 MB span when configured as both ranks (0x7f20).
    // Per-bank physical SIMM sizes. 0 means bank is absent.
    // Default: single 128 MB bank (FPGA target). Set other banks non-zero
    // (and update MEMCFG0/1_DEFAULT above) to extend to 256/384 MB for sim.
    parameter BANK0_SIMM_BYTES = 32'h08000000; // 128 MB
    parameter BANK1_SIMM_BYTES = 32'h00000000; // absent (FPGA default)
    parameter BANK2_SIMM_BYTES = 32'h00000000; // absent (FPGA default)
    parameter BANK3_SIMM_BYTES = 32'h00000000; // absent (IP20 supports 3 banks max)

    function simm_dual_rank;
        input [31:0] simm_bytes;
        begin
            case (simm_bytes)
                32'h00800000,  // 8 MB
                32'h02000000,  // 32 MB
                32'h08000000:  // 128 MB
                    simm_dual_rank = 1'b1;
                default:
                    simm_dual_rank = 1'b0;
            endcase
        end
    endfunction

    function [28:0] simm_alias_mask;
        input [31:0] simm_bytes;
        input        conf_rank;
        reg [31:0] alias_bytes;
        begin
            if (simm_bytes == 32'h00000000) begin
                simm_alias_mask = 29'h0;
            end else begin
                alias_bytes = (simm_dual_rank(simm_bytes) && !conf_rank) ?
                              (simm_bytes >> 2) : simm_bytes;
                simm_alias_mask = alias_bytes[28:0] - 29'd1;
            end
        end
    endfunction

    wire b0_present = (BANK0_SIMM_BYTES != 0);
    wire b1_present = (BANK1_SIMM_BYTES != 0);
    wire b2_present = (BANK2_SIMM_BYTES != 0);
    wire b3_present = (BANK3_SIMM_BYTES != 0);

    wire [6:0] cpu_addr_4mb = cpu_addr[28:22]; // physical address / 4MB
    wire [6:0] dma_addr_4mb = dma_addr[28:22]; // physical address / 4MB

    // MEMCFG layout (per IP20 mc.h):
    //   MEMCFG0 bits 31:16 = bank 0 entry, bits 15:0 = bank 1 entry
    //   MEMCFG1 bits 31:16 = bank 2 entry, bits 15:0 = bank 3 entry
    // Each entry: bit 14 = BNK/rank, bit 13 = VLD,
    //             bits 12:8 = size_mask, bits 7:0 = base_4MB
    //             (we use low 7 bits of base for 512MB addressable range)
    wire        b0_rank = memcfg0[30];
    wire        b0_vld  = memcfg0[29];
    wire [4:0]  b0_mask = memcfg0[28:24];
    wire [6:0]  b0_base = memcfg0[22:16];
    wire        b1_rank = memcfg0[14];
    wire        b1_vld  = memcfg0[13];
    wire [4:0]  b1_mask = memcfg0[12:8];
    wire [6:0]  b1_base = memcfg0[6:0];
    wire        b2_rank = memcfg1[30];
    wire        b2_vld  = memcfg1[29];
    wire [4:0]  b2_mask = memcfg1[28:24];
    wire [6:0]  b2_base = memcfg1[22:16];
    wire        b3_rank = memcfg1[14];
    wire        b3_vld  = memcfg1[13];
    wire [4:0]  b3_mask = memcfg1[12:8];
    wire [6:0]  b3_base = memcfg1[6:0];

    wire [28:0] b0_simm_mask = simm_alias_mask(BANK0_SIMM_BYTES, b0_rank);
    wire [28:0] b1_simm_mask = simm_alias_mask(BANK1_SIMM_BYTES, b1_rank);
    wire [28:0] b2_simm_mask = simm_alias_mask(BANK2_SIMM_BYTES, b2_rank);
    wire [28:0] b3_simm_mask = simm_alias_mask(BANK3_SIMM_BYTES, b3_rank);

    wire b0_hit = b0_present && b0_vld && (cpu_addr_4mb >= b0_base) &&
                   (cpu_addr_4mb <= (b0_base + {2'b0, b0_mask}));
    wire b1_hit = b1_present && b1_vld && (cpu_addr_4mb >= b1_base) &&
                   (cpu_addr_4mb <= (b1_base + {2'b0, b1_mask}));
    wire b2_hit = b2_present && b2_vld && (cpu_addr_4mb >= b2_base) &&
                   (cpu_addr_4mb <= (b2_base + {2'b0, b2_mask}));
    wire b3_hit = b3_present && b3_vld && (cpu_addr_4mb >= b3_base) &&
                   (cpu_addr_4mb <= (b3_base + {2'b0, b3_mask}));

    wire dma_b0_hit = b0_present && b0_vld && (dma_addr_4mb >= b0_base) &&
                      (dma_addr_4mb <= (b0_base + {2'b0, b0_mask}));
    wire dma_b1_hit = b1_present && b1_vld && (dma_addr_4mb >= b1_base) &&
                      (dma_addr_4mb <= (b1_base + {2'b0, b1_mask}));
    wire dma_b2_hit = b2_present && b2_vld && (dma_addr_4mb >= b2_base) &&
                      (dma_addr_4mb <= (b2_base + {2'b0, b2_mask}));
    wire dma_b3_hit = b3_present && b3_vld && (dma_addr_4mb >= b3_base) &&
                      (dma_addr_4mb <= (b3_base + {2'b0, b3_mask}));

    // Per-bank in-bank offset with physical-SIMM wrap. 29-bit arithmetic.
    // in_bank_raw is the unaliased offset; & simm_mask implements the wrap
    // (aliases accesses beyond the physical SIMM back into the SIMM).
    // phys_idx is the effective physical address used to index backing RAM.
    wire [28:0] b0_bank_base_byte = {b0_base, 22'b0};
    wire [28:0] b0_in_bank        = (cpu_addr[28:0] - b0_bank_base_byte) & b0_simm_mask;
    wire [28:0] b0_phys_idx       = b0_bank_base_byte + b0_in_bank;
    wire [28:0] b1_bank_base_byte = {b1_base, 22'b0};
    wire [28:0] b1_in_bank        = (cpu_addr[28:0] - b1_bank_base_byte) & b1_simm_mask;
    wire [28:0] b1_phys_idx       = b1_bank_base_byte + b1_in_bank;
    wire [28:0] b2_bank_base_byte = {b2_base, 22'b0};
    wire [28:0] b2_in_bank        = (cpu_addr[28:0] - b2_bank_base_byte) & b2_simm_mask;
    wire [28:0] b2_phys_idx       = b2_bank_base_byte + b2_in_bank;
    wire [28:0] b3_bank_base_byte = {b3_base, 22'b0};
    wire [28:0] b3_in_bank        = (cpu_addr[28:0] - b3_bank_base_byte) & b3_simm_mask;
    wire [28:0] b3_phys_idx       = b3_bank_base_byte + b3_in_bank;

    wire [28:0] dma_b0_in_bank    = (dma_addr[28:0] - b0_bank_base_byte) & b0_simm_mask;
    wire [28:0] dma_b0_phys_idx   = b0_bank_base_byte + dma_b0_in_bank;
    wire [28:0] dma_b1_in_bank    = (dma_addr[28:0] - b1_bank_base_byte) & b1_simm_mask;
    wire [28:0] dma_b1_phys_idx   = b1_bank_base_byte + dma_b1_in_bank;
    wire [28:0] dma_b2_in_bank    = (dma_addr[28:0] - b2_bank_base_byte) & b2_simm_mask;
    wire [28:0] dma_b2_phys_idx   = b2_bank_base_byte + dma_b2_in_bank;
    wire [28:0] dma_b3_in_bank    = (dma_addr[28:0] - b3_bank_base_byte) & b3_simm_mask;
    wire [28:0] dma_b3_phys_idx   = b3_bank_base_byte + dma_b3_in_bank;

    // Low-memory alias: IP20addrs documents physical memory 0-512KB aliased at
    // address 0 and at 128MB. Map low alias accesses onto the SEG0 bank index.
    wire cpu_low_ram = (LOW_ALIAS_BYTES != 0) &&
                       (cpu_addr[28:0] < LOW_ALIAS_BYTES[28:0]);
    wire [28:0] cpu_low_alias_idx = 29'h0800_0000 + cpu_addr[28:0];
    wire any_bank_hit = b0_hit || b1_hit || b2_hit || b3_hit;

    wire cpu_ram_valid = cpu_is_ram && (any_bank_hit || cpu_low_ram);

    wire vdma_xlate_enabled = (dma_ctl & VDMA_C_XLATE) != 32'h0;
    wire vdma_page_16k = dma_ctl[1];
    wire vdma_pte_8byte = dma_ctl[0];
    wire [31:0] vdma_page_mask = vdma_page_16k ? 32'h0000_3fff : 32'h0000_0fff;
    wire [31:0] vdma_page_base_mask = ~vdma_page_mask;
    wire [4:0]  vdma_page_shift = vdma_page_16k ? 5'd14 : 5'd12;
    wire [2:0]  vdma_pte_shift = vdma_pte_8byte ? 3'd3 : 3'd2;
    wire [31:0] vdma_tlb_tag = vdma_cur_mem & 32'hffc0_0000;
    wire vdma_tlb_match0 = ((dma_tlb_hi[0] & 32'hffc0_0000) == vdma_tlb_tag);
    wire vdma_tlb_match1 = ((dma_tlb_hi[1] & 32'hffc0_0000) == vdma_tlb_tag);
    wire vdma_tlb_match2 = ((dma_tlb_hi[2] & 32'hffc0_0000) == vdma_tlb_tag);
    wire vdma_tlb_match3 = ((dma_tlb_hi[3] & 32'hffc0_0000) == vdma_tlb_tag);
    wire vdma_tlb_hit = vdma_tlb_match0 || vdma_tlb_match1 ||
                        vdma_tlb_match2 || vdma_tlb_match3;
    wire [31:0] vdma_tlb_lo_sel = vdma_tlb_match0 ? dma_tlb_lo[0] :
                                  vdma_tlb_match1 ? dma_tlb_lo[1] :
                                  vdma_tlb_match2 ? dma_tlb_lo[2] :
                                                     dma_tlb_lo[3];
    wire [31:0] vdma_pte_base = (vdma_tlb_lo_sel & 32'h03ff_ffc0) << 6;
    wire [31:0] vdma_pte_index = (vdma_cur_mem & 32'h003f_ffff) >> vdma_page_shift;
    wire [31:0] vdma_pte_addr_calc = vdma_pte_base +
                                      (vdma_pte_index << vdma_pte_shift);
    // Software can install a valid-looking zero-base uTLB entry as
    // a negative-stride segment-boundary lookahead sentinel. It must never be
    // consumed as a real page-table base at physical zero.
    wire vdma_dummy_tlb_entry = vdma_tlb_hit && vdma_tlb_lo_sel[1] &&
                                ((vdma_tlb_lo_sel & 32'h03ff_ffc0) == 32'h0);
    wire [31:0] vdma_pte_word = vdma_pte_8byte ? ram_rdata[31:0] :
                                 ram_word_at(ram_rdata, !vdma_xlate_pte_addr[2]);
    wire vdma_xlate_page_hit = vdma_xlate_page_valid &&
        ((vdma_cur_mem & vdma_page_base_mask) == vdma_xlate_vpage);

    // Select RAM index: first-matching bank's wrapped index; low-mem
    // fallback uses phys addr directly. Banks are mutually exclusive in
    // practice (PROM configures non-overlapping bases).
    wire [25:0] cpu_ram_idx = b0_hit ? b0_phys_idx[28:3] :
                              b1_hit ? b1_phys_idx[28:3] :
                              b2_hit ? b2_phys_idx[28:3] :
                              b3_hit ? b3_phys_idx[28:3] :
                              cpu_low_ram ? cpu_low_alias_idx[28:3] :
                                            26'h0;

    // DMA masters use the same physical RAM decode as the CPU.  IP20 PROM
    // transmit descriptors live in the bank-0 SIMM window at 0x08000000+;
    // rejecting that range makes HPC1 Ethernet fetch zero descriptors even
    // though the CPU can build them there.
    wire dma_is_ram = (dma_addr[31:29] == 3'h0);
    wire dma_low_ram = (LOW_ALIAS_BYTES != 0) &&
                       (dma_addr[28:0] < LOW_ALIAS_BYTES[28:0]);
    wire [28:0] dma_low_alias_idx = 29'h0800_0000 + dma_addr[28:0];
    wire dma_any_bank_hit = dma_b0_hit || dma_b1_hit || dma_b2_hit || dma_b3_hit;
    wire dma_ram_valid = dma_is_ram && (dma_any_bank_hit || dma_low_ram);
    wire [25:0] dma_ram_idx = dma_b0_hit ? dma_b0_phys_idx[28:3] :
                              dma_b1_hit ? dma_b1_phys_idx[28:3] :
                              dma_b2_hit ? dma_b2_phys_idx[28:3] :
                              dma_b3_hit ? dma_b3_phys_idx[28:3] :
                              dma_low_ram ? dma_low_alias_idx[28:3] :
                                            26'h0;

    // Bus-float: for RAM addresses beyond physical size, return address as data
    wire [63:0] bus_float_data = {cpu_addr[31:0], cpu_addr[31:0]};

    // GIO region (for timeout error flag)
    wire lg1_region = cpu_gio_select;
    wire gio_region = !lg1_region && (
                      (cpu_addr[35:24] >= 12'h1F0) &&
                      (cpu_addr[35:24] <= 12'h1F9));

    // ===========================================================================
    // State Machine
    // ===========================================================================

    localparam S_IDLE             = 5'd0;
    localparam S_MC_REG           = 5'd1;
    localparam S_CPU_RAM          = 5'd2;
    localparam S_DMA_RAM          = 5'd3;
    localparam S_WAIT             = 5'd4;
    localparam S_BUS_FLOAT        = 5'd5;
    localparam S_VDMA_HTOG_ADDR   = 5'd6;
    localparam S_VDMA_HTOG_DATA   = 5'd7;
    localparam S_VDMA_GIO_WRITE   = 5'd8;
    localparam S_VDMA_GIO_READ    = 5'd9;
    localparam S_VDMA_GTOH_WRITE  = 5'd10;
    localparam S_VDMA_GTOH_WAIT   = 5'd11;
    localparam S_VDMA_FILL_WRITE  = 5'd12;
    localparam S_VDMA_FILL_WAIT   = 5'd13;
    localparam S_VDMA_COMPLETE    = 5'd14;
    localparam S_VDMA_XLATE       = 5'd15;
    localparam S_VDMA_XLATE_PTE   = 5'd16;
    localparam S_VDMA_FAULT       = 5'd17;
    localparam S_CPU_GIO_WRITE    = 5'd18;
    localparam S_CPU_GIO_READ     = 5'd19;

    reg [4:0] state;

    // The admitted SoC clock is the 50 MHz R4000 bus clock; GIO64 runs at
    // 33 1/3 MHz.  Advance tenure counters on two of every three SoC clocks.
    wire arb_gio_tick = (arb_gio_phase != 2'd2);
    wire dma_rt_request = dma_request || dma_valid;
    wire arb_rt_active = dma_rt_request || (state == S_DMA_RAM);
    wire arb_grant_vdma = vdma_running &&
        (arb_owner_vdma ? (arb_time_left != 16'h0) :
                          (arb_time_left == 16'h0));

    wire cpu_gio_fifo_pop = (state == S_CPU_GIO_WRITE) &&
                            vdma_gio_valid && vdma_gio_ready;
    wire cpu_gio_write_ready = cpu_valid && cpu_gio_select && cpu_write &&
                               ((cpu_gio_fifo_count < cpu_gio_hwm) ||
                                cpu_gio_fifo_pop);
    wire cpu_gio_write_admit = cpu_gio_write_ready;

    // Posted writes use a normal ready/valid handshake.  Payload identity is
    // not transaction identity: two adjacent writes may legally carry the
    // same address, data, and mask, and each valid && ready cycle is one FIFO
    // admission.  The production S-cache drops valid between CPU requests,
    // while this boundary also remains correct for a continuous-valid master.
    assign cpu_ready = cpu_gio_write_ready ||
                       (cpu_ready_r &&
                        !(cpu_valid && cpu_ready_req_changed));

    localparam [3:0] DBG_RESP_NONE = 4'd0;
    localparam [3:0] DBG_RESP_CPU  = 4'd1;
    localparam [3:0] DBG_RESP_DMA  = 4'd2;
    localparam [3:0] DBG_RESP_VDMA = 4'd3;

    localparam [25:0] DBG_WATCH_DW_ADDR = 26'h01013AE0; // 0x0809d700 >> 3
    localparam [31:0] DBG_WATCH_HI_WORD = 32'h3179_FFBF;
    localparam [31:0] DBG_WATCH_LO_WORD = 32'h358F_0080;

    reg [3:0]  debug_last_resp_src;
    reg [4:0]  debug_last_resp_state;
    reg        debug_last_resp_wen;
    reg        debug_last_resp_watch;
    reg        debug_last_resp_bad;
    reg [25:0] debug_last_resp_addr;
    reg [63:0] debug_last_resp_data;
    reg [15:0] debug_watch_cpu_count;
    reg [15:0] debug_watch_cpu_bad_count;
    reg [63:0] debug_watch_cpu_last_data;
    reg [63:0] debug_watch_cpu_bad_data;
    reg [15:0] debug_cpu_ram_req_count;
    reg [15:0] debug_cpu_ram_resp_count;
    reg [15:0] debug_idle_cpu_valid_count;
    reg [15:0] debug_bus_float_count;

    wire [31:0] debug_ram_paddr = {3'b000, ram_addr, 3'b000};
    wire [31:0] debug_last_resp_paddr = {3'b000, debug_last_resp_addr, 3'b000};
    wire        debug_cpu_watch_resp =
        (state == S_CPU_RAM) && ram_ready && !cpu_write &&
        (ram_addr == DBG_WATCH_DW_ADDR);
    wire        debug_cpu_watch_bad =
        debug_cpu_watch_resp &&
        (ram_rdata[63:32] != DBG_WATCH_HI_WORD ||
         ram_rdata[31:0]  != DBG_WATCH_LO_WORD);

    wire [31:0] debug_live_flags = {
        8'h4C, // "L"
        state,
        ram_valid,
        ram_wen,
        ram_ready,
        cpu_ready,
        dma_ready,
        cpu_valid,
        cpu_write,
        dma_valid,
        dma_write,
        vdma_running,
        vdma_fault_latched,
        8'd0
    };
    wire [31:0] debug_last_flags = {
        8'h52, // "R"
        debug_last_resp_src,
        debug_last_resp_state,
        debug_last_resp_wen,
        debug_last_resp_watch,
        debug_last_resp_bad,
        12'd0
    };
    wire [31:0] debug_progress_counts = {
        debug_cpu_ram_req_count,
        debug_cpu_ram_resp_count
    };
    wire [31:0] debug_progress_flags = {
        8'h05,
        state,
        cpu_valid,
        mc_select,
        cpu_is_ram,
        cpu_ram_valid,
        any_bank_hit,
        cpu_low_ram,
        dma_valid,
        dma_ram_valid,
        vdma_running,
        ram_valid,
        ram_ready,
        cpu_ready,
        dma_ready,
        ram_wen,
        cpu_write,
        cpu_valid && mc_select,
        cpu_valid && cpu_is_ram,
        cpu_valid && cpu_ram_valid,
        ram_valid && !ram_ready
    };

    assign debug_probe = {
        32'h4D43_5253, // "MCRS"
        debug_live_flags,
        cpu_addr[31:0],
        dma_addr,
        debug_ram_paddr,
        ram_rdata[63:32],
        debug_last_flags,
        debug_last_resp_paddr,
        debug_last_resp_data[63:32],
        debug_last_resp_data[31:0],
        debug_progress_counts,
        debug_progress_flags,
        memcfg0,
        memcfg1,
        {debug_idle_cpu_valid_count, debug_bus_float_count}
    };

    // ===========================================================================
    // Main Logic
    // ===========================================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            cpu_ready_r <= 1'b0;
            cpu_req_addr_q <= 36'd0;
            cpu_req_wdata_q <= 64'd0;
            cpu_req_wmask_q <= 8'd0;
            cpu_req_write_q <= 1'b0;
            cpu_rdata_q <= 64'h0;
            dma_ready <= 1'b0;
            dma_rdata <= 64'h0;
            vdma_gio_addr  <= 32'h0;
            vdma_gio_wdata <= 64'h0;
            vdma_gio_wmask <= 8'h0;
            vdma_gio_valid <= 1'b0;
            vdma_gio_write <= 1'b0;
            vdma_gio_source_vdma <= 1'b0;
            cpu_gio_fifo_wr_ptr <= 4'd0;
            cpu_gio_fifo_rd_ptr <= 4'd0;
            cpu_gio_fifo_count <= 5'd0;
            ram_addr  <= 26'h0;
            ram_wdata <= 64'h0;
            ram_wmask <= 8'h0;
            ram_wen   <= 1'b0;
            ram_valid <= 1'b0;

            cpuctrl0     <= 32'h0;
            cpuctrl1     <= 32'h0;
            gio64_arb    <= 32'h0000_0400;
            cpu_time     <= 16'h0100;
            lb_time      <= 16'h0200;
            memcfg0      <= MEMCFG0_DEFAULT;
            memcfg1      <= MEMCFG1_DEFAULT;
            cpu_memacc   <= 32'h0;
            gio_memacc   <= 32'h0;
            cpu_err_stat <= 32'h0;
            gio_err_addr <= 32'h0;
            gio_err_stat <= 32'h0;
            rpss_counter <= 32'h0;
            rpss_divider_cnt <= 2'd0;
            eerom_reg    <= 32'h0;
            dma_gio_mask <= 32'h0;
            dma_gio_sub  <= 32'h0;
            dma_cause    <= 32'h0;
            dma_ctl      <= 32'h0;
            dma_tlb_hi[0] <= 32'h0;
            dma_tlb_hi[1] <= 32'h0;
            dma_tlb_hi[2] <= 32'h0;
            dma_tlb_hi[3] <= 32'h0;
            dma_tlb_lo[0] <= 32'h0;
            dma_tlb_lo[1] <= 32'h0;
            dma_tlb_lo[2] <= 32'h0;
            dma_tlb_lo[3] <= 32'h0;
            vdma_memadr <= 32'h0;
            vdma_size <= 32'h0001_0000;
            vdma_stride <= 32'h0001_0000;
            vdma_gio_adr <= 32'h0;
            vdma_mode <= 32'h0;
            vdma_count <= 32'h0;
            vdma_running <= 1'b0;
            vdma_cur_mem <= 32'h0;
            vdma_phys_mem <= 32'h0;
            vdma_line_start_mem <= 32'h0;
            vdma_width_rem <= 16'h0;
            vdma_lines_rem <= 16'h0;
            vdma_zoom_rem <= 16'h1;
            vdma_pack_data <= 32'h0;
            vdma_pack_mask <= 4'h0;
            vdma_packet_line_done <= 1'b0;
            vdma_read_word <= 32'h0;
            vdma_read_lane <= 2'h0;
            vdma_fill_step <= 4'h1;
            vdma_after_xlate <= S_IDLE;
            vdma_xlate_write <= 1'b0;
            vdma_xlate_pte_addr <= 32'h0;
            vdma_xlate_page_valid <= 1'b0;
            vdma_xlate_vpage <= 32'h0;
            vdma_xlate_ppage <= 32'h0;
            vdma_xlate_page_writable <= 1'b0;
            vdma_fault_latched <= 1'b0;
            arb_owner_vdma <= 1'b0;
            arb_time_left <= 16'h0100;
            arb_gio_phase <= 2'd0;
            vdma_resume_pending <= 1'b0;
            vdma_resume_state <= S_IDLE;
            debug_last_resp_src <= DBG_RESP_NONE;
            debug_last_resp_state <= S_IDLE;
            debug_last_resp_wen <= 1'b0;
            debug_last_resp_watch <= 1'b0;
            debug_last_resp_bad <= 1'b0;
            debug_last_resp_addr <= 26'd0;
            debug_last_resp_data <= 64'd0;
            debug_watch_cpu_count <= 16'd0;
            debug_watch_cpu_bad_count <= 16'd0;
            debug_watch_cpu_last_data <= 64'd0;
            debug_watch_cpu_bad_data <= 64'd0;
            debug_cpu_ram_req_count <= 16'd0;
            debug_cpu_ram_resp_count <= 16'd0;
            debug_idle_cpu_valid_count <= 16'd0;
            debug_bus_float_count <= 16'd0;
`ifndef SYNTHESIS
            vdma_trace_count <= 32'h0;
`endif
        end else begin
            // CPU writes are acknowledged when captured, not when the GIO
            // target eventually accepts them. This is the MC/MUX posted-write
            // contract that keeps the R4000 running through graphics stalls.
            if (cpu_gio_write_admit) begin
                cpu_gio_fifo_addr[cpu_gio_fifo_wr_ptr] <= cpu_addr[31:0];
                cpu_gio_fifo_wdata[cpu_gio_fifo_wr_ptr] <= cpu_wdata;
                cpu_gio_fifo_wmask[cpu_gio_fifo_wr_ptr] <= cpu_wmask;
                cpu_gio_fifo_wr_ptr <= (cpu_gio_fifo_wr_ptr == 4'd14) ?
                                       4'd0 : cpu_gio_fifo_wr_ptr + 4'd1;
            end

            if (cpu_gio_fifo_pop)
                cpu_gio_fifo_rd_ptr <= (cpu_gio_fifo_rd_ptr == 4'd14) ?
                                       4'd0 : cpu_gio_fifo_rd_ptr + 4'd1;

            case ({cpu_gio_write_admit, cpu_gio_fifo_pop})
                2'b10: cpu_gio_fifo_count <= cpu_gio_fifo_count + 5'd1;
                2'b01: cpu_gio_fifo_count <= cpu_gio_fifo_count - 5'd1;
                default: ;
            endcase

            if (arb_gio_phase == 2'd2)
                arb_gio_phase <= 2'd0;
            else
                arb_gio_phase <= arb_gio_phase + 2'd1;

            if (arb_gio_tick && !arb_rt_active && arb_time_left != 16'h0)
                arb_time_left <= arb_time_left - 16'd1;

            // RPSS counter — increment every 4th cycle (25MHz at 100MHz sim clock)
            // to approximate real IP20 timing (50MHz bus / RPSS divider 2).
            rpss_divider_cnt <= rpss_divider_cnt + 2'd1;
            if (rpss_divider_cnt == 2'd3)
                rpss_counter <= rpss_counter + 32'd1;

            // GIO timeout. Preserve the address of the first pending error;
            // software clears status before a later probe can replace it.
            if (gio_access && cpuctrl1[4] && !gio_err_stat[10]) begin
                gio_err_stat <= gio_err_stat | 32'h00000400;
                gio_err_addr <= cpu_addr[31:0];
`ifndef SYNTHESIS
                $display("MC: GIO timeout at addr=0x%08x (bus error asserted)", cpu_addr[31:0]);
`endif
            end

            // Default: deassert RAM write
            if (!ram_valid)
                ram_wen <= 1'b0;

            case (state)
                S_IDLE: begin
                    cpu_ready_r <= 1'b0;
                    dma_ready <= 1'b0;
                    vdma_gio_valid <= 1'b0;
                    vdma_gio_write <= 1'b0;
                    vdma_gio_source_vdma <= 1'b0;
                    if (cpu_valid && debug_idle_cpu_valid_count != 16'hffff)
                        debug_idle_cpu_valid_count <= debug_idle_cpu_valid_count + 16'd1;

                    // HPC is an always-real-time requester and preempts either
                    // long-burst owner.  Its service pauses the active tenure.
                    if (dma_valid) begin
                        if (dma_ram_valid) begin
                            ram_addr  <= dma_ram_idx;
                            ram_wdata <= dma_wdata;
                            ram_wmask <= dma_wmask;
                            ram_wen   <= dma_write;
                            ram_valid <= 1'b1;
                            state     <= S_DMA_RAM;
                        end else begin
                            // DMA to non-existent RAM — return zeros, complete
                            dma_rdata <= 64'h0;
                            dma_ready <= 1'b1;
                            state     <= S_WAIT;
                        end
                    // Raw HPC demand reserves MC while the tag-first S-cache
                    // lookup resolves. No new CPU or VDMA external transfer
                    // may enter in that interval. A fill/writeback already
                    // serialized by the S-cache must drain, however, because
                    // the coherence lookup can depend on its completion.
                    end else if (arb_grant_vdma && !dma_rt_request) begin
                        if (!arb_owner_vdma) begin
                            arb_owner_vdma <= 1'b1;
                            arb_time_left <= lb_time;
                        end
                        if (vdma_resume_pending) begin
                            vdma_resume_pending <= 1'b0;
                            state <= vdma_resume_state;
                        end else if (vdma_lines_rem == 16'h0 || vdma_width_rem == 16'h0) begin
                            state <= S_VDMA_COMPLETE;
                        end else if ((vdma_mode & VDMA_M_FILL) != 32'h0) begin
                            vdma_after_xlate <= S_VDMA_FILL_WRITE;
                            vdma_xlate_write <= 1'b1;
                            state <= S_VDMA_XLATE;
                        end else if ((vdma_mode & VDMA_M_GTOH) != 32'h0) begin
                            if (vdma_read_lane == 2'h0) begin
                                state <= S_VDMA_GIO_READ;
                            end else begin
                                vdma_after_xlate <= S_VDMA_GTOH_WRITE;
                                vdma_xlate_write <= 1'b1;
                                state <= S_VDMA_XLATE;
                            end
                        end else begin
                            vdma_after_xlate <= S_VDMA_HTOG_ADDR;
                            vdma_xlate_write <= 1'b0;
                            state <= S_VDMA_XLATE;
                        end
                    end else if (cpu_gio_fifo_count != 5'd0 &&
                                 !dma_rt_request) begin
                        if (arb_owner_vdma) begin
                            arb_owner_vdma <= 1'b0;
                            arb_time_left <= cpu_time;
                        end
                        state <= S_CPU_GIO_WRITE;
                    end else if (cpu_valid && mc_select &&
                                 !dma_rt_request) begin
                        if (arb_owner_vdma) begin
                            arb_owner_vdma <= 1'b0;
                            arb_time_left <= cpu_time;
                        end
                        cpu_req_addr_q <= cpu_addr;
                        cpu_req_wdata_q <= cpu_wdata;
                        cpu_req_wmask_q <= cpu_wmask;
                        cpu_req_write_q <= cpu_write;
                        state <= S_MC_REG;
                    end else if (cpu_valid && cpu_gio_select && !cpu_write &&
                                 !dma_rt_request) begin
                        if (arb_owner_vdma) begin
                            arb_owner_vdma <= 1'b0;
                            arb_time_left <= cpu_time;
                        end
                        cpu_req_addr_q <= cpu_addr;
                        cpu_req_wdata_q <= cpu_wdata;
                        cpu_req_wmask_q <= cpu_wmask;
                        cpu_req_write_q <= 1'b0;
                        state <= S_CPU_GIO_READ;
                    end else if (cpu_valid && !cpu_gio_select &&
                                 (!dma_rt_request || cpu_cache_owned)) begin
                        if (arb_owner_vdma) begin
                            arb_owner_vdma <= 1'b0;
                            arb_time_left <= cpu_time;
                        end
                        if (cpu_is_ram) begin
                            if (cpu_ram_valid) begin
                                cpu_req_addr_q <= cpu_addr;
                                cpu_req_wdata_q <= cpu_wdata;
                                cpu_req_wmask_q <= cpu_wmask;
                                cpu_req_write_q <= cpu_write;
                                // Use aliased index so bank 0 over-configs wrap
                                // within the physical SIMM, matching real MC.
                                ram_addr  <= cpu_ram_idx;
                                ram_wdata <= cpu_wdata;
                                ram_wmask <= cpu_wmask;
                                ram_wen   <= cpu_write;
                                ram_valid <= 1'b1;
                                state     <= S_CPU_RAM;
                                if (debug_cpu_ram_req_count != 16'hffff)
                                    debug_cpu_ram_req_count <= debug_cpu_ram_req_count + 16'd1;
                            end else begin
                                cpu_req_addr_q <= cpu_addr;
                                cpu_req_wdata_q <= cpu_wdata;
                                cpu_req_wmask_q <= cpu_wmask;
                                cpu_req_write_q <= cpu_write;
                                // Beyond physical RAM — bus-float
                                state <= S_BUS_FLOAT;
                                if (debug_bus_float_count != 16'hffff)
                                    debug_bus_float_count <= debug_bus_float_count + 16'd1;
                            end
                        end
                    end else if (arb_owner_vdma && !dma_rt_request) begin
                        // A long-burst device returns early when it becomes
                        // idle.  CPU still receives its complete CPU_TIME
                        // reservation even when it has no current request.
                        arb_owner_vdma <= 1'b0;
                        arb_time_left <= cpu_time;
                    end
                end

                S_CPU_GIO_WRITE: begin
                    // A queued write may be deferred before it is presented
                    // so the always-real-time HPC requester can run. Once
                    // valid rises, the transaction is indivisible through
                    // ready because GIO has no cancellation handshake.
                    if (vdma_gio_valid) begin
                        if (vdma_gio_ready) begin
                            vdma_gio_valid <= 1'b0;
                            vdma_gio_write <= 1'b0;
                            vdma_gio_source_vdma <= 1'b0;
                            state <= S_IDLE;
                        end
                    end else if (dma_rt_request) begin
                        state <= S_IDLE;
                    end else begin
                        vdma_gio_addr <= cpu_gio_fifo_addr[cpu_gio_fifo_rd_ptr];
                        vdma_gio_wdata <= cpu_gio_fifo_wdata[cpu_gio_fifo_rd_ptr];
                        vdma_gio_wmask <= cpu_gio_fifo_wmask[cpu_gio_fifo_rd_ptr];
                        vdma_gio_write <= 1'b1;
                        vdma_gio_source_vdma <= 1'b0;
                        vdma_gio_valid <= 1'b1;
                    end
                end

                S_CPU_GIO_READ: begin
                    if (vdma_gio_valid) begin
                        if (vdma_gio_ready) begin
                            cpu_rdata_q <= vdma_gio_rdata;
                            cpu_ready_r <= 1'b1;
                            vdma_gio_valid <= 1'b0;
                            vdma_gio_source_vdma <= 1'b0;
                            state <= S_WAIT;
                        end
                    end else if (dma_rt_request) begin
                        // CPU continues to hold the read request; retry it
                        // after real-time service without presenting a beat.
                        state <= S_IDLE;
                    end else begin
                        vdma_gio_addr <= cpu_req_addr_q[31:0];
                        vdma_gio_wdata <= 64'h0;
                        vdma_gio_wmask <= 8'h0;
                        vdma_gio_write <= 1'b0;
                        vdma_gio_source_vdma <= 1'b0;
                        vdma_gio_valid <= 1'b1;
                    end
                end

                S_VDMA_XLATE: begin
                    if (!vdma_xlate_enabled) begin
                        vdma_phys_mem <= vdma_cur_mem & 32'h1fff_ffff;
                        state <= vdma_after_xlate;
                    end else if (vdma_xlate_page_hit) begin
                        if (vdma_xlate_write &&
                            !vdma_xlate_page_writable) begin
                            dma_cause <= dma_cause | VDMA_R_CLEAN;
                            state <= S_VDMA_FAULT;
                        end else begin
                            vdma_phys_mem <= vdma_xlate_ppage |
                                             (vdma_cur_mem & vdma_page_mask);
                            state <= vdma_after_xlate;
                        end
                    end else if (!vdma_tlb_hit || !vdma_tlb_lo_sel[1]) begin
                        dma_cause <= dma_cause | VDMA_R_UTLBMISS;
                        state <= S_VDMA_FAULT;
                    end else if (vdma_dummy_tlb_entry) begin
                        dma_cause <= dma_cause | VDMA_R_BPTE;
                        state <= S_VDMA_FAULT;
                    end else begin
                        vdma_xlate_pte_addr <= vdma_pte_addr_calc;
                        ram_addr <= {1'b0, vdma_pte_addr_calc[27:3]};
                        ram_wen <= 1'b0;
                        ram_valid <= 1'b1;
                        state <= S_VDMA_XLATE_PTE;
                    end
                end

                S_VDMA_XLATE_PTE: begin
                    if (ram_ready) begin
                        ram_valid <= 1'b0;
                        ram_wen <= 1'b0;
                        debug_last_resp_src <= DBG_RESP_VDMA;
                        debug_last_resp_state <= state;
                        debug_last_resp_wen <= 1'b0;
                        debug_last_resp_watch <= 1'b0;
                        debug_last_resp_bad <= 1'b0;
                        debug_last_resp_addr <= ram_addr;
                        debug_last_resp_data <= ram_rdata;
                        if (!vdma_pte_word[1]) begin
                            dma_cause <= dma_cause | VDMA_R_PAGEFAULT;
                            state <= S_VDMA_FAULT;
                        end else if (vdma_xlate_write && !vdma_pte_word[2]) begin
                            dma_cause <= dma_cause | VDMA_R_CLEAN;
                            state <= S_VDMA_FAULT;
                        end else begin
                            vdma_phys_mem <= ((vdma_pte_word & 32'h03ff_ffc0) << 6) |
                                             (vdma_cur_mem & vdma_page_mask);
                            // The hardware page-table lookup is a per-page
                            // cost.  Retain the translated page for following
                            // bytes until the virtual page, context controls,
                            // or DMA lifecycle changes.
                            vdma_xlate_page_valid <= 1'b1;
                            vdma_xlate_vpage <= vdma_cur_mem &
                                                vdma_page_base_mask;
                            vdma_xlate_ppage <=
                                ((vdma_pte_word & 32'h03ff_ffc0) << 6) &
                                vdma_page_base_mask;
                            vdma_xlate_page_writable <= vdma_pte_word[2];
                            state <= vdma_after_xlate;
                        end
                    end
                end

                S_VDMA_FAULT: begin
                    vdma_running <= 1'b0;
                    vdma_fault_latched <= 1'b1;
                    vdma_xlate_page_valid <= 1'b0;
                    vdma_resume_pending <= 1'b0;
                    vdma_gio_valid <= 1'b0;
                    vdma_gio_write <= 1'b0;
`ifndef SYNTHESIS
                    if (trace_vdma)
                        $display("VDMA_FAULT mem=%08x ctl=%08x cause=%08x",
                                 vdma_cur_mem, dma_ctl, dma_cause);
`endif
                    state <= S_IDLE;
                end

                S_CPU_RAM: begin
                    if (ram_ready) begin
                        cpu_rdata_q <= ram_rdata;
                        cpu_ready_r <= 1'b1;
                        ram_wen   <= 1'b0;
                        ram_valid <= 1'b0;
                        state     <= S_WAIT;
                        debug_last_resp_src <= DBG_RESP_CPU;
                        debug_last_resp_state <= state;
                        debug_last_resp_wen <= cpu_write;
                        debug_last_resp_watch <= debug_cpu_watch_resp;
                        debug_last_resp_bad <= debug_cpu_watch_bad;
                        debug_last_resp_addr <= ram_addr;
                        debug_last_resp_data <= ram_rdata;
                        if (debug_cpu_ram_resp_count != 16'hffff)
                            debug_cpu_ram_resp_count <= debug_cpu_ram_resp_count + 16'd1;
                        if (debug_cpu_watch_resp) begin
                            debug_watch_cpu_count <= debug_watch_cpu_count + 16'd1;
                            debug_watch_cpu_last_data <= ram_rdata;
                        end
                        if (debug_cpu_watch_bad) begin
                            debug_watch_cpu_bad_count <= debug_watch_cpu_bad_count + 16'd1;
                            debug_watch_cpu_bad_data <= ram_rdata;
                        end

                        // GIO timeout for GIO region (passed through MC for PROM/other)
                        if (gio_region)
                            gio_err_stat <= gio_err_stat | 32'h00000400;
                    end
                end

                S_DMA_RAM: begin
                    if (ram_ready) begin
                        dma_rdata <= ram_rdata;
                        dma_ready <= 1'b1;
                        ram_wen   <= 1'b0;
                        ram_valid <= 1'b0;
                        state     <= S_WAIT;
                        debug_last_resp_src <= DBG_RESP_DMA;
                        debug_last_resp_state <= state;
                        debug_last_resp_wen <= dma_write;
                        debug_last_resp_watch <= 1'b0;
                        debug_last_resp_bad <= 1'b0;
                        debug_last_resp_addr <= ram_addr;
                        debug_last_resp_data <= ram_rdata;
                    end
                end

                S_BUS_FLOAT: begin
                    // Return address as data (SIMM not present)
                    cpu_rdata_q <= bus_float_data;
                    cpu_ready_r <= 1'b1;
                    state     <= S_WAIT;
                end

                S_VDMA_HTOG_ADDR: begin
                    ram_addr <= {1'b0, vdma_phys_mem[27:3]};
                    ram_wen <= 1'b0;
                    ram_valid <= 1'b1;
                    state    <= S_VDMA_HTOG_DATA;
                end

                S_VDMA_HTOG_DATA: begin
                    if (ram_ready) begin
                        ram_valid <= 1'b0;
                        ram_wen <= 1'b0;
                        debug_last_resp_src <= DBG_RESP_VDMA;
                        debug_last_resp_state <= state;
                        debug_last_resp_wen <= 1'b0;
                        debug_last_resp_watch <= 1'b0;
                        debug_last_resp_bad <= 1'b0;
                        debug_last_resp_addr <= ram_addr;
                        debug_last_resp_data <= ram_rdata;
                        // Preserve the memory byte lanes on GIO writes. LIGHT's
                        // LG1 VDMA diagnostics bias xstart for unaligned buffers
                        // and rely on inactive leading byte lanes reaching REX.
                        // The response also contains every following byte up to
                        // this four-byte packet boundary, so retire the complete
                        // packet without rereading the same 64-bit RAM word.
                        vdma_pack_data <= vdma_htog_pack3;
                        vdma_pack_mask <= vdma_htog_mask3;
                        vdma_cur_mem <= vdma_advance_addr(
                            vdma_cur_mem, vdma_mode, {1'b0, vdma_htog_step});
                        vdma_width_rem <= vdma_width_rem -
                                          {13'h0, vdma_htog_step};
                        vdma_packet_line_done <=
                            (vdma_width_rem <= {13'h0, vdma_htog_step});
                        state <= S_VDMA_GIO_WRITE;
                    end
                end

                S_VDMA_GIO_WRITE: begin
                    // Once presented, a GIO operation is atomic.  LG1 can
                    // begin a multicycle read before asserting ready and this
                    // interface has no GIO READ/P_READ cancellation signal,
                    // so owner and payload remain stable through acceptance.
                    // A pending real-time request or expired tenure prevents
                    // only the next beat from being launched.
                    if (vdma_gio_valid) begin
                        if (vdma_gio_ready) begin
                            vdma_gio_valid <= 1'b0;
                            vdma_gio_write <= 1'b0;
                            vdma_pack_data <= 32'h0;
                            vdma_pack_mask <= 4'h0;

                            if (vdma_packet_line_done) begin
                                if (vdma_zoom_rem > 16'd1) begin
                                    vdma_zoom_rem <= vdma_zoom_rem - 16'd1;
                                    vdma_width_rem <= vdma_size[15:0];
                                    vdma_cur_mem <= vdma_line_start_mem;
                                    state <= S_IDLE;
                                end else if (vdma_lines_rem <= 16'd1) begin
                                    state <= S_VDMA_COMPLETE;
                                end else begin
                                    vdma_lines_rem <= vdma_lines_rem - 16'd1;
                                    vdma_width_rem <= vdma_size[15:0];
                                    vdma_cur_mem <= vdma_next_line_addr(vdma_cur_mem, vdma_stride);
                                    vdma_line_start_mem <= vdma_next_line_addr(vdma_cur_mem, vdma_stride);
                                    vdma_zoom_rem <= vdma_yzoom;
                                    state <= S_IDLE;
                                end
                            end else begin
                                state <= S_IDLE;
                            end
                        end
                    end else if (dma_rt_request ||
                                 (arb_owner_vdma &&
                                  arb_time_left == 16'h0)) begin
                        vdma_gio_write <= 1'b0;
                        vdma_resume_pending <= 1'b1;
                        vdma_resume_state <= S_VDMA_GIO_WRITE;
                        state <= S_IDLE;
                    end else if (!vdma_gio_valid) begin
                        vdma_gio_addr  <= vdma_effective_gio_addr;
                        vdma_gio_wdata <= {vdma_pack_data, vdma_pack_data};
                        vdma_gio_wmask <= vdma_lanes_to_wmask(vdma_pack_mask);
                        vdma_gio_write <= 1'b1;
                        vdma_gio_source_vdma <= 1'b1;
                        vdma_gio_valid <= 1'b1;
                    end
                end

                S_VDMA_GIO_READ: begin
                    if (vdma_gio_valid) begin
                        if (vdma_gio_ready) begin
                            vdma_read_word <= vdma_gio_rdata[31:0];
                            vdma_read_lane <= 2'h0;
                            vdma_gio_valid <= 1'b0;
                            vdma_after_xlate <= S_VDMA_GTOH_WRITE;
                            vdma_xlate_write <= 1'b1;
                            state <= S_VDMA_XLATE;
                        end
                    end else if (dma_rt_request ||
                                 (arb_owner_vdma &&
                                  arb_time_left == 16'h0)) begin
                        vdma_resume_pending <= 1'b1;
                        vdma_resume_state <= S_VDMA_GIO_READ;
                        state <= S_IDLE;
                    end else if (!vdma_gio_valid) begin
                        vdma_gio_addr  <= vdma_effective_gio_addr;
                        vdma_gio_wdata <= 64'h0;
                        vdma_gio_wmask <= 8'h0;
                        vdma_gio_write <= 1'b0;
                        vdma_gio_source_vdma <= 1'b1;
                        vdma_gio_valid <= 1'b1;
                    end
                end

                S_VDMA_GTOH_WRITE: begin
                    ram_addr  <= {1'b0, vdma_phys_mem[27:3]};
                    ram_wdata <= ram_byte_wdata(vdma_word_lane(vdma_read_word, vdma_read_lane),
                                                vdma_phys_mem[2:0]);
                    ram_wmask <= ram_byte_wmask(vdma_phys_mem[2:0]);
                    ram_wen   <= 1'b1;
                    ram_valid <= 1'b1;
                    state     <= S_VDMA_GTOH_WAIT;
                end

                S_VDMA_GTOH_WAIT: begin
                    if (ram_ready) begin
                        ram_wen <= 1'b0;
                        ram_valid <= 1'b0;
                        debug_last_resp_src <= DBG_RESP_VDMA;
                        debug_last_resp_state <= state;
                        debug_last_resp_wen <= 1'b1;
                        debug_last_resp_watch <= 1'b0;
                        debug_last_resp_bad <= 1'b0;
                        debug_last_resp_addr <= ram_addr;
                        debug_last_resp_data <= ram_rdata;
                        vdma_cur_mem <= vdma_next_byte_addr(vdma_cur_mem, vdma_mode);

                        if (vdma_width_rem <= 16'd1) begin
                            if (vdma_lines_rem <= 16'd1) begin
                                state <= S_VDMA_COMPLETE;
                            end else begin
                                vdma_lines_rem <= vdma_lines_rem - 16'd1;
                                vdma_width_rem <= vdma_size[15:0];
                                vdma_read_lane <= 2'h0;
                                vdma_cur_mem <= vdma_next_line_addr(
                                    vdma_next_byte_addr(vdma_cur_mem, vdma_mode),
                                    vdma_stride);
                                state <= S_IDLE;
                            end
                        end else begin
                            vdma_width_rem <= vdma_width_rem - 16'd1;
                            if (vdma_read_lane == 2'd3) begin
                                vdma_read_lane <= 2'h0;
                                state <= S_IDLE;
                            end else begin
                                vdma_read_lane <= vdma_read_lane + 2'd1;
                                state <= S_IDLE;
                            end
                        end
                    end
                end

                S_VDMA_FILL_WRITE: begin
                    ram_addr  <= {1'b0, vdma_phys_mem[27:3]};
                    if (((vdma_mode & VDMA_M_INCA) != 32'h0) &&
                        !vdma_xlate_enabled &&
                        vdma_phys_mem[2:0] == 3'b000 &&
                        vdma_width_rem >= 16'd8) begin
                        ram_wdata <= {vdma_gio_adr, vdma_gio_adr};
                        ram_wmask <= 8'hff;
                        vdma_fill_step <= 4'd8;
                    end else begin
                        ram_wdata <= ram_byte_wdata(vdma_word_lane(vdma_gio_adr, vdma_read_lane),
                                                    vdma_phys_mem[2:0]);
                        ram_wmask <= ram_byte_wmask(vdma_phys_mem[2:0]);
                        vdma_fill_step <= 4'd1;
                    end
                    ram_wen   <= 1'b1;
                    ram_valid <= 1'b1;
                    state     <= S_VDMA_FILL_WAIT;
                end

                S_VDMA_FILL_WAIT: begin
                    if (ram_ready) begin
                        ram_wen <= 1'b0;
                        ram_valid <= 1'b0;
                        debug_last_resp_src <= DBG_RESP_VDMA;
                        debug_last_resp_state <= state;
                        debug_last_resp_wen <= 1'b1;
                        debug_last_resp_watch <= 1'b0;
                        debug_last_resp_bad <= 1'b0;
                        debug_last_resp_addr <= ram_addr;
                        debug_last_resp_data <= ram_rdata;
                        vdma_cur_mem <= vdma_advance_addr(vdma_cur_mem, vdma_mode, vdma_fill_step);

                        if (vdma_width_rem <= {12'h0, vdma_fill_step}) begin
                            if (vdma_lines_rem <= 16'd1) begin
                                state <= S_VDMA_COMPLETE;
                            end else begin
                                vdma_lines_rem <= vdma_lines_rem - 16'd1;
                                vdma_width_rem <= vdma_size[15:0];
                                vdma_read_lane <= 2'h0;
                                vdma_cur_mem <= vdma_next_line_addr(
                                    vdma_advance_addr(vdma_cur_mem, vdma_mode, vdma_fill_step),
                                    vdma_stride);
                                state <= S_IDLE;
                            end
                        end else begin
                            vdma_width_rem <= vdma_width_rem - {12'h0, vdma_fill_step};
                            if (vdma_fill_step == 4'd1)
                                vdma_read_lane <= vdma_read_lane + 2'd1;
                            state <= S_IDLE;
                        end
                    end
                end

                S_VDMA_COMPLETE: begin
                    vdma_running <= 1'b0;
                    vdma_fault_latched <= 1'b0;
                    vdma_resume_pending <= 1'b0;
                    dma_cause <= dma_cause | VDMA_R_COMPLETE;
                    vdma_gio_valid <= 1'b0;
                    vdma_gio_write <= 1'b0;
`ifndef SYNTHESIS
                    if (trace_vdma)
                        $display("VDMA_DONE mem=%08x gio=%08x mode=%08x cause=%08x",
                                 vdma_cur_mem, vdma_gio_adr, vdma_mode,
                                 dma_cause | VDMA_R_COMPLETE);
`endif
                    state <= S_IDLE;
                end

                S_MC_REG: begin
                    cpu_ready_r <= 1'b1;
                    state     <= S_WAIT;

                    if (cpu_write) begin
                        case (aligned_offset)
                            16'h0000: cpuctrl0   <= merge32_either_lane(cpuctrl0,   cpu_wdata, cpu_wmask);
                            16'h0008: cpuctrl1   <= merge32_either_lane(cpuctrl1,   cpu_wdata, cpu_wmask);
                            OFF_GIO64_ARB: gio64_arb <= gio64_arb_wdata;
                            OFF_CPU_TIME: begin
                                cpu_time <= cpu_time_wdata[15:0];
                                if (!arb_owner_vdma)
                                    arb_time_left <= cpu_time_wdata[15:0];
                            end
                            OFF_LB_TIME: begin
                                lb_time <= lb_time_wdata[15:0];
                                if (arb_owner_vdma)
                                    arb_time_left <= lb_time_wdata[15:0];
                            end
                            16'h00C0: memcfg0    <= merge32_either_lane(memcfg0,    cpu_wdata, cpu_wmask);
                            16'h00C8: memcfg1    <= merge32_either_lane(memcfg1,    cpu_wdata, cpu_wmask);
                            16'h00D0: cpu_memacc <= merge32_either_lane(cpu_memacc, cpu_wdata, cpu_wmask);
                            16'h00D8: gio_memacc <= merge32_either_lane(gio_memacc, cpu_wdata, cpu_wmask);
                            16'h00E8: begin // Any write clears.
                                cpu_err_stat <= 32'h0;
                            end
                            16'h00F8: begin // Any write clears.
                                gio_err_stat <= 32'h0;
                            end
                            16'h0030: eerom_reg <=
                                merge32_either_lane(eerom_reg, cpu_wdata, cpu_wmask) &
                                ~EEROM_SI;
                            OFF_DMA_GIO_MASK: dma_gio_mask <= merge32(dma_gio_mask, reg_wdata32, reg_wmask32);
                            OFF_DMA_GIO_SUB:  dma_gio_sub  <= merge32(dma_gio_sub,  reg_wdata32, reg_wmask32);
                            OFF_DMA_CAUSE:    dma_cause    <= 32'h0;
                            OFF_DMA_CTL: begin
                                dma_ctl <= merge32(dma_ctl, reg_wdata32, reg_wmask32);
                                vdma_xlate_page_valid <= 1'b0;
                            end
                            OFF_DMA_TLB_HI_0: begin
                                dma_tlb_hi[0] <= merge32(dma_tlb_hi[0], reg_wdata32, reg_wmask32);
                                vdma_xlate_page_valid <= 1'b0;
                            end
                            OFF_DMA_TLB_LO_0: begin
                                dma_tlb_lo[0] <= merge32(dma_tlb_lo[0], reg_wdata32, reg_wmask32);
                                vdma_xlate_page_valid <= 1'b0;
                            end
                            OFF_DMA_TLB_HI_1: begin
                                dma_tlb_hi[1] <= merge32(dma_tlb_hi[1], reg_wdata32, reg_wmask32);
                                vdma_xlate_page_valid <= 1'b0;
                            end
                            OFF_DMA_TLB_LO_1: begin
                                dma_tlb_lo[1] <= merge32(dma_tlb_lo[1], reg_wdata32, reg_wmask32);
                                vdma_xlate_page_valid <= 1'b0;
                            end
                            OFF_DMA_TLB_HI_2: begin
                                dma_tlb_hi[2] <= merge32(dma_tlb_hi[2], reg_wdata32, reg_wmask32);
                                vdma_xlate_page_valid <= 1'b0;
                            end
                            OFF_DMA_TLB_LO_2: begin
                                dma_tlb_lo[2] <= merge32(dma_tlb_lo[2], reg_wdata32, reg_wmask32);
                                vdma_xlate_page_valid <= 1'b0;
                            end
                            OFF_DMA_TLB_HI_3: begin
                                dma_tlb_hi[3] <= merge32(dma_tlb_hi[3], reg_wdata32, reg_wmask32);
                                vdma_xlate_page_valid <= 1'b0;
                            end
                            OFF_DMA_TLB_LO_3: begin
                                dma_tlb_lo[3] <= merge32(dma_tlb_lo[3], reg_wdata32, reg_wmask32);
                                vdma_xlate_page_valid <= 1'b0;
                            end
                            OFF_DMA_MEMADR: begin
                                vdma_memadr <= merge32(vdma_memadr, reg_wdata32, reg_wmask32);
                            end
                            OFF_DMA_MEMADRD: begin
                                vdma_memadr <= merge32(vdma_memadr, reg_wdata32, reg_wmask32);
                                vdma_stride <= 32'h0001_0000;
                                vdma_mode <= VDMA_M_INCA | 32'h0000_0040;
                            end
                            OFF_DMA_SIZE: begin
                                vdma_size <= vdma_size_wdata;
                                vdma_count[15:0] <= vdma_size_wdata[15:0];
                            end
                            OFF_DMA_STRIDE: begin
                                vdma_stride <= vdma_stride_wdata;
                                vdma_count[31:16] <= {6'h0, vdma_stride_wdata[25:16]};
                            end
                            OFF_DMA_GIO_ADR: begin
                                vdma_gio_adr <= merge32(vdma_gio_adr, reg_wdata32, reg_wmask32);
                            end
                            OFF_DMA_GIO_ADRS: begin
                                vdma_gio_adr <= vdma_gio_adrs_wdata;
                                vdma_running <= 1'b1;
                                vdma_fault_latched <= 1'b0;
                                dma_cause <= 32'h0;
                                vdma_cur_mem <= vdma_memadr;
                                vdma_line_start_mem <= vdma_memadr;
                                vdma_width_rem <= vdma_count[15:0];
                                vdma_lines_rem <= vdma_size[31:16];
                                vdma_zoom_rem <= vdma_count_yzoom;
                                vdma_pack_data <= 32'h0;
                                vdma_pack_mask <= 4'h0;
                                vdma_packet_line_done <= 1'b0;
                                vdma_read_lane <= 2'h0;
                                vdma_resume_pending <= 1'b0;
                                vdma_xlate_page_valid <= 1'b0;
`ifndef SYNTHESIS
                                if (trace_vdma && vdma_trace_count < vdma_trace_limit) begin
                                    vdma_trace_count <= vdma_trace_count + 32'd1;
                                    $display("VDMA_START gioadrs mem=%08x gio=%08x size=%08x stride=%08x mode=%08x ctl=%08x",
                                             vdma_memadr,
                                             vdma_gio_adrs_wdata,
                                             vdma_size, vdma_stride, vdma_mode, dma_ctl);
                                end
`endif
                            end
                            OFF_DMA_MODE: begin
                                vdma_mode <= merge32(vdma_mode, reg_wdata32, reg_wmask32);
                            end
                            OFF_DMA_COUNT: begin
                                vdma_count <= merge32(vdma_count, reg_wdata32, reg_wmask32);
                            end
                            OFF_DMA_STDMA: begin
                                if (reg_wdata32[0]) begin
                                    vdma_running <= 1'b1;
                                    dma_cause <= 32'h0;
                                    if (vdma_fault_latched) begin
                                        vdma_fault_latched <= 1'b0;
                                    end else begin
                                        vdma_cur_mem <= vdma_memadr;
                                        vdma_line_start_mem <= vdma_memadr;
                                        vdma_width_rem <= vdma_count[15:0];
                                        vdma_lines_rem <= vdma_size[31:16];
                                        vdma_zoom_rem <= vdma_count_yzoom;
                                        vdma_pack_data <= 32'h0;
                                        vdma_pack_mask <= 4'h0;
                                        vdma_packet_line_done <= 1'b0;
                                        vdma_read_lane <= 2'h0;
                                    end
                                    vdma_resume_pending <= 1'b0;
                                    vdma_xlate_page_valid <= 1'b0;
`ifndef SYNTHESIS
                                    if (trace_vdma && vdma_trace_count < vdma_trace_limit) begin
                                        vdma_trace_count <= vdma_trace_count + 32'd1;
                                        $display("VDMA_START stdma mem=%08x gio=%08x size=%08x stride=%08x mode=%08x ctl=%08x",
                                                 vdma_fault_latched ? vdma_cur_mem : vdma_memadr,
                                                 vdma_gio_adr, vdma_size,
                                                 vdma_stride, vdma_mode, dma_ctl);
                                    end
`endif
                                end else begin
                                    vdma_running <= 1'b0;
                                    vdma_fault_latched <= 1'b0;
                                end
                            end
                            OFF_DMA_MEMADRDS: begin
                                vdma_memadr <= merge32(vdma_memadr, reg_wdata32, reg_wmask32);
                                vdma_running <= 1'b1;
                                vdma_fault_latched <= 1'b0;
                                dma_cause <= 32'h0;
                                vdma_cur_mem <= merge32(vdma_memadr, reg_wdata32, reg_wmask32);
                                vdma_line_start_mem <= merge32(vdma_memadr, reg_wdata32, reg_wmask32);
                                vdma_width_rem <= vdma_count[15:0];
                                vdma_lines_rem <= vdma_size[31:16];
                                vdma_zoom_rem <= vdma_count_yzoom;
                                vdma_pack_data <= 32'h0;
                                vdma_pack_mask <= 4'h0;
                                vdma_packet_line_done <= 1'b0;
                                vdma_read_lane <= 2'h0;
                                vdma_resume_pending <= 1'b0;
                                vdma_xlate_page_valid <= 1'b0;
`ifndef SYNTHESIS
                                if (trace_vdma && vdma_trace_count < vdma_trace_limit) begin
                                    vdma_trace_count <= vdma_trace_count + 32'd1;
                                    $display("VDMA_START memadrds mem=%08x gio=%08x size=%08x stride=%08x mode=%08x ctl=%08x",
                                             merge32(vdma_memadr, reg_wdata32, reg_wmask32),
                                             vdma_gio_adr, vdma_size, vdma_stride,
                                             vdma_mode, dma_ctl);
                                end
`endif
                            end
                            default: ;
                        endcase
                    end else begin
                        case (aligned_offset)
                            16'h0000, 16'h0008, 16'h0018,
                            OFF_GIO64_ARB, OFF_CPU_TIME, OFF_LB_TIME, 16'h00C0,
                            16'h00C8, 16'h00D0, 16'h00D8, 16'h00E8,
                            16'h00F8, 16'h0030, 16'h1000, OFF_DMA_GIO_MASK,
                            OFF_DMA_GIO_SUB, OFF_DMA_CAUSE, OFF_DMA_CTL,
                            OFF_DMA_TLB_HI_0, OFF_DMA_TLB_LO_0,
                            OFF_DMA_TLB_HI_1, OFF_DMA_TLB_LO_1,
                            OFF_DMA_TLB_HI_2, OFF_DMA_TLB_LO_2,
                            OFF_DMA_TLB_HI_3, OFF_DMA_TLB_LO_3,
                            OFF_DMA_MEMADR, OFF_DMA_MEMADRD, OFF_DMA_SIZE,
                            OFF_DMA_STRIDE, OFF_DMA_GIO_ADR, OFF_DMA_GIO_ADRS,
                            OFF_DMA_MODE, OFF_DMA_COUNT, OFF_DMA_STDMA,
                            OFF_DMA_RUN, OFF_DMA_MEMADRDS:
                                cpu_rdata_q <= mc_reg_rdata;
                            default:
                                cpu_rdata_q <= 64'h0;
                        endcase
                    end
                end

                S_WAIT: begin
                    cpu_ready_r <= 1'b0;
                    dma_ready <= 1'b0;
                    state     <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
