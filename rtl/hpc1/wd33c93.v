// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - WD33C93 SCSI Controller
// Western Digital WD33C93 (AIC-33C93) SCSI bus interface controller
//
// Indirect register access via two byte ports:
//   Address port (SCSI0A): write sets reg_ptr, read returns aux status
//   Data port (SCSI0D): read/write the register selected by reg_ptr
//
// Read data is combinatorial (same pattern as scc_z8530.v)
// Side effects (aux_int clear, state transitions) are registered
//
// Data path: WD93 ←DRQ/DACK→ HPC1 DMA engine ←bus master→ memory
//   For SCSI→mem: WD93 asserts DRQ with byte, DMA engine writes to memory
//   For mem→SCSI: DMA engine reads from memory, provides byte on DACK
//
// Cosim: C++ fills scsi_buf[] via Verilator hierarchy (represents SCSI
//   target data), then asserts scsi_cmd_done. WD93 enters data phase
//   and drives DRQ from the buffer. Transfer count regs are decremented
//   per DACK. When count reaches 0, WD93 completes autonomously.
//
// Multi-pass transfers: The PROM's DMA descriptor chain is capped at
//   256KB (64 pages). For larger reads (e.g., IDE .text = 652KB), the
//   WD93 programmed count expires mid-transfer. The SCSI target still
//   has data, so the WD93 signals an unexpected-data status: the target
//   is still in a data phase. The PROM remaps DMA descriptors, reprograms
//   WD93 transfer count, and re-issues C93SELATNTR. The WD93 resumes from
//   scsi_buf_idx (not reset).

`default_nettype none

module wd33c93 #(
    parameter SCSI_EXTERNAL_TARGET = 0
) (
    input  wire        clk,
    input  wire        rst_n,

    // Byte-level register access (from HPC1)
    input  wire        sel_addr,    // 1=address port (0x122), 0=data port (0x126)
    input  wire [7:0]  wdata,       // Write data byte
    output wire [7:0]  rdata,       // Read data byte (combinatorial)
    input  wire        valid,       // Access strobe
    input  wire        write,       // 1=write, 0=read

    // Hardware reset from HPC1 SCSI_CTRL bit 0
    input  wire        hpc_reset,
    input  wire        dma_window_done,

    // Target presence (bitmask: bit N = target ID N present)
    input  wire [7:0]  target_present,

    // Target backend command interface (cosim C++ or synthesizable target)
    output wire        scsi_cmd_pending,  // Command ready for target processing
    input  wire        scsi_cmd_done,     // Target accepted command / read data is ready
    input  wire [7:0]  scsi_cmd_status,   // WD93 status to report (e.g., 0x16=ST_SATOK)
    input  wire [23:0] scsi_resp_len,     // Total SCSI data-phase bytes
    input  wire        scsi_status_valid, // For external writes: final status is ready
    output wire [2:0]  scsi_target_id,
    output wire [95:0] scsi_cdb,
    output wire [23:0] scsi_xfer_count,
    output wire [23:0] scsi_data_index,
    input  wire [7:0]  scsi_read_data,
    input  wire        scsi_read_valid,
    output wire        scsi_read_consume,
    output wire        scsi_write_valid,
    output wire [7:0]  scsi_write_data,
    input  wire        scsi_write_ready,
    output wire        scsi_data_out_done,

    // DRQ/DACK interface to HPC1 DMA engine
    output wire        drq,               // Data request: byte ready (to_mem) or byte needed (from_mem)
    input  wire        dack,              // Data acknowledge from DMA engine
    output wire [7:0]  drq_data_out,      // Byte from WD93 to DMA (SCSI→mem)
    input  wire [7:0]  dack_data_in,      // Byte from DMA to WD93 (mem→SCSI)
    output wire        drq_to_mem,        // Direction: 1=SCSI→mem, 0=mem→SCSI

    // Interrupt output (active high when aux_int is set)
    output wire        irq,

    // Debug snapshot for FPGA bring-up
    output wire [127:0] debug_probe
);

    // =========================================================================
    // Register file (WD93 registers 0x00-0x1A)
    // =========================================================================

    // WD93 register indices (from wd93.h)
    localparam WD93ID     = 5'd0;   // Own ID (+ frequency bits)
    localparam WD93CTRL   = 5'd1;   // Control register
    localparam WD93TOUT   = 5'd2;   // Timeout period
    // 3-14: CDB bytes (C1-C12)
    localparam WD93LUN    = 5'd15;  // Target LUN
    localparam WD93PHASE  = 5'd16;  // Command phase
    localparam WD93SYNC   = 5'd17;  // Synchronous transfer
    localparam WD93CNTHI  = 5'd18;  // Transfer count MSB
    localparam WD93CNTMD  = 5'd19;  // Transfer count
    localparam WD93CNTLO  = 5'd20;  // Transfer count LSB
    localparam WD93DESTID = 5'd21;  // Destination SCSI ID
    localparam WD93SRCID  = 5'd22;  // Source ID (+ enable reselection)
    localparam WD93STAT   = 5'd23;  // SCSI status (read-only in real HW)
    localparam WD93CMD    = 5'd24;  // Command register
    localparam WD93DATA   = 5'd25;  // Data register
    localparam WD93QTAG   = 5'd26;  // Queue tag (93B only)

    reg [7:0] regs [0:26];

    // Register pointer (set by writing to address port)
    reg [7:0] reg_ptr;

    // =========================================================================
    // SCSI target data buffer (cosim: C++ fills via Verilator hierarchy)
    // Represents bytes received from (or to be sent to) the SCSI target device.
    // In real hardware this would be the SCSI bus data lines + REQ/ACK handshake.
    // =========================================================================

`ifndef SYNTHESIS
    reg [7:0] scsi_buf [0:4194303];  // 4MB (cosim staging)
`endif
    reg [23:0] scsi_buf_idx;          // Current byte index

    // Transfer count from registers (24-bit)
    wire [23:0] xfer_count = {regs[WD93CNTHI], regs[WD93CNTMD], regs[WD93CNTLO]};

    // =========================================================================
    // Auxiliary Status Register
    // =========================================================================

    reg aux_int;   // Bit 7 (0x80): Interrupt pending
    reg aux_lci;   // Bit 6 (0x40): Last command ignored
    reg aux_cip;   // Bit 4 (0x10): Command in progress

    // WD33C93 Auxiliary Status Register bit layout:
    // Bit 7: INT  (0x80) — interrupt pending
    // Bit 6: LCI  (0x40) — last command ignored
    // Bit 5: BSY  (0x20) — SCSI bus busy (not modeled)
    // Bit 4: CIP  (0x10) — command in progress
    // Bit 3: (reserved)
    // Bit 2: (reserved)
    // Bit 1: PE   (0x02) — parity error (not modeled)
    // Bit 0: DBR  (0x01) — data buffer ready for programmed I/O
    wire aux_dbr;
    wire [7:0] aux_status = {aux_int, aux_lci, 1'b0, aux_cip, 3'b000, aux_dbr};
    // Drive the board interrupt whenever the WD93 has a pending event.
    // The firmware/software baseline that reaches a diskless root depends on
    // these interrupts even when OWN_ID.IDI has not been programmed.
    assign irq = aux_int;

    reg trace_scsi;
    initial begin
        trace_scsi = 1'b0;
`ifndef SYNTHESIS
        if ($test$plusargs("trace_scsi") || $test$plusargs("scsi_debug"))
            trace_scsi = 1'b1;
`endif
    end

    // =========================================================================
    // Command state machine
    // =========================================================================

    localparam S_IDLE           = 3'd0;
    localparam S_RESET_PENDING  = 3'd1;
    localparam S_SELECT_PENDING = 3'd2;  // Absent target: timeout
    localparam S_DATA_PENDING   = 3'd3;  // Present target: wait for C++ to fill buffer
    localparam S_DATA_PHASE     = 3'd4;  // DRQ/DACK data transfer in progress
    localparam S_DATA_PAUSE     = 3'd5;  // Paused: programmed count expired, target has more data
    localparam S_STATUS_PENDING = 3'd6;  // External write target is committing data
    localparam S_STATUS_PAUSE   = 3'd7;  // Paused: target ended data before host count

    reg [2:0]  state;
    reg [31:0] delay_cnt;
    reg [7:0]  pending_status;  // Status to set on command completion
    reg        cmd_to_mem;      // Direction latched from SCSI command context
    reg        scsi_connected;  // Target is connected (scsi_buf valid, multi-pass)
    reg [2:0]  pending_dest_id; // Target ID for delayed selection timeout
    reg        aux_int_d;
    reg        dbg_irq_seen;
    reg        dbg_cmd_seen;
    reg        dbg_cmd_done_seen;
    reg        dbg_data_phase_seen;
    reg        dbg_data_xfer_seen;
    reg        dbg_dack_seen;
    reg        dbg_stat_read_seen;
    reg        dbg_aux_read_seen;
    reg [3:0]  dbg_cmd_count;
    reg [3:0]  dbg_cmd_done_count;
    reg [3:0]  dbg_data_xfer_count;
    reg [3:0]  dbg_stat_read_count;

    // Target selection check
    wire [2:0] dest_id = regs[WD93DESTID][2:0];
    wire       dest_present = target_present[dest_id];

    // Target command export. Keep the CDB byte order as programmed in WD93C1..C12.
    assign scsi_cmd_pending = (state == S_DATA_PENDING);
    assign scsi_target_id = dest_id;
    assign scsi_cdb = {
        regs[5'd3],  regs[5'd4],  regs[5'd5],  regs[5'd6],
        regs[5'd7],  regs[5'd8],  regs[5'd9],  regs[5'd10],
        regs[5'd11], regs[5'd12], regs[5'd13], regs[5'd14]
    };
    assign scsi_xfer_count = xfer_count;
    assign scsi_data_index = scsi_buf_idx;

    // DRQ interface: assert when in data phase and transfer count > 0.
    // Cap at scsi_resp_len (if non-zero) for both reads and writes:
    // - Reads: prevents overrun past SCSI response buffer
    // - Writes: terminates when target has received all expected data
    //   (models SCSI bus phase change from Data-Out to Status)
    wire use_external_target = (SCSI_EXTERNAL_TARGET != 0);
    wire target_byte_ready = cmd_to_mem
        ? (!use_external_target || scsi_read_valid)
        : (!use_external_target || scsi_write_ready);

    wire data_phase_ready = (state == S_DATA_PHASE) && (xfer_count != 24'h0) &&
                            (scsi_resp_len == 24'd0 || scsi_buf_idx < scsi_resp_len) &&
                            target_byte_ready;
    wire pio_data_reg_selected = !sel_addr && (reg_ptr <= 8'd26) &&
                                 (reg_ptr[4:0] == WD93DATA);
    wire pio_data_read_access = valid && !write && pio_data_reg_selected &&
                                (state == S_DATA_PHASE) && cmd_to_mem;
    wire pio_data_write_access = valid && write && pio_data_reg_selected &&
                                 (state == S_DATA_PHASE) && !cmd_to_mem;
    wire pio_data_read_xfer = pio_data_read_access && data_phase_ready;
    wire pio_data_write_xfer = pio_data_write_access && data_phase_ready;

    assign aux_dbr = data_phase_ready;
    assign drq = data_phase_ready;
`ifndef SYNTHESIS
    assign drq_data_out = use_external_target ? scsi_read_data :
                           scsi_buf[scsi_buf_idx];
`else
    assign drq_data_out = use_external_target ? scsi_read_data : 8'h00;
`endif
    assign drq_to_mem = cmd_to_mem;
    wire dack_xfer = dack && drq;
    wire data_phase_xfer = dack_xfer || pio_data_read_xfer || pio_data_write_xfer;
    wire [7:0] data_phase_write_data = pio_data_write_xfer ? wdata : dack_data_in;
    assign scsi_read_consume = use_external_target &&
                               (dack_xfer || pio_data_read_xfer) && cmd_to_mem;
    assign scsi_write_valid = use_external_target &&
                              (dack_xfer || pio_data_write_xfer) && !cmd_to_mem;
    assign scsi_write_data = data_phase_write_data;
    assign scsi_data_out_done = use_external_target &&
                                (state == S_STATUS_PENDING);

    // WD93 command codes (from wd93.h)
    localparam C93RESET       = 8'h00;
    localparam C93ABORT       = 8'h01;
    localparam C93SETATN      = 8'h02;
    localparam C93NEGATE_ACK  = 8'h03;
    localparam C93DISC        = 8'h04;
    localparam C93SELECT      = 8'h06;  // Select without ATN
    localparam C93SELECT_ATN  = 8'h07;  // Select with ATN
    localparam C93SELECT_XFER = 8'h08;  // Select and transfer
    localparam C93SELNATNTR   = 8'h09;  // Select without ATN and transfer

    localparam ST_UNEX_RDATA  = 8'h48;
    localparam ST_UNEX_SDATA  = 8'h49;
    localparam ST_UNEX_SSTATUS = 8'h4b;
    localparam PH_IDENTRECV   = 8'h45;
    localparam PH_DATA        = 8'h46;
    localparam PH_COMPLETE    = 8'h60;

    // Compatible firmware/software treats ST_UNEX_SDATA as the long-read continuation
    // case and ST_UNEX_RDATA as the long-write continuation case.
    wire [7:0] unexpected_data_status = cmd_to_mem ? ST_UNEX_SDATA : ST_UNEX_RDATA;

`ifdef WD93_FAST_TIMEOUT
    localparam [31:0] SELECT_TIMEOUT_CYCLES = 32'd500;
`else
    // The programmed WD33C93 selection timeout is in the hundreds of
    // milliseconds. Firing this in a few microseconds can interrupt the driver
    // before its active-command bookkeeping has settled.
    localparam [31:0] SELECT_TIMEOUT_CYCLES = 32'd6_250_000; // 250 ms @ 25 MHz
`endif

    wire [31:0] debug_word0 = 32'h5744_3933; // "WD93"
    wire [7:0] debug_events = {
        dbg_irq_seen,
        dbg_cmd_seen,
        dbg_cmd_done_seen,
        dbg_data_phase_seen,
        dbg_data_xfer_seen,
        dbg_dack_seen,
        dbg_stat_read_seen,
        dbg_aux_read_seen
    };
    wire [31:0] debug_word1 = {
        debug_events,
        state,
        aux_int,
        aux_lci,
        aux_cip,
        irq,
        scsi_connected,
        cmd_to_mem,
        dest_present,
        pending_dest_id,
        dest_id,
        reg_ptr
    };
    wire [31:0] debug_word2 = {
        regs[WD93LUN],
        regs[WD93PHASE],
        regs[WD93SRCID],
        regs[WD93STAT]
    };
    wire [31:0] debug_word3 = {
        regs[WD93DESTID],
        xfer_count
    };
    assign debug_probe = {debug_word0, debug_word1, debug_word2, debug_word3};

    // =========================================================================
    // Combinatorial read data
    // =========================================================================

    reg [7:0] rdata_comb;
    assign rdata = rdata_comb;

    always @(*) begin
        if (sel_addr) begin
            // Reading address port returns auxiliary status register
            rdata_comb = aux_status;
        end else begin
            // Reading data port returns the register at reg_ptr
            if (pio_data_read_xfer)
                rdata_comb = drq_data_out;
            else if (reg_ptr <= 8'd26)
                rdata_comb = regs[reg_ptr[4:0]];
            else
                rdata_comb = 8'h00;
        end
    end

    // =========================================================================
    // Registered logic: state machine, register writes, side effects
    // =========================================================================

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_ptr <= 8'd0;
            aux_int <= 1'b0;
            aux_lci <= 1'b0;
            aux_cip <= 1'b0;
            state   <= S_IDLE;
            delay_cnt <= 32'd0;
            scsi_buf_idx <= 24'h0;
            pending_status <= 8'h0;
            cmd_to_mem <= 1'b0;
            scsi_connected <= 1'b0;
            pending_dest_id <= 3'd0;
            aux_int_d <= 1'b0;
            dbg_irq_seen <= 1'b0;
            dbg_cmd_seen <= 1'b0;
            dbg_cmd_done_seen <= 1'b0;
            dbg_data_phase_seen <= 1'b0;
            dbg_data_xfer_seen <= 1'b0;
            dbg_dack_seen <= 1'b0;
            dbg_stat_read_seen <= 1'b0;
            dbg_aux_read_seen <= 1'b0;
            dbg_cmd_count <= 4'h0;
            dbg_cmd_done_count <= 4'h0;
            dbg_data_xfer_count <= 4'h0;
            dbg_stat_read_count <= 4'h0;
            for (i = 0; i <= 26; i = i + 1)
                regs[i] <= 8'h00;
        end else if (hpc_reset) begin
            // HPC-level hardware reset
            reg_ptr <= 8'd0;
            aux_int <= 1'b0;
            aux_lci <= 1'b0;
            aux_cip <= 1'b0;
            state   <= S_IDLE;
            delay_cnt <= 32'd0;
            scsi_buf_idx <= 24'h0;
            pending_status <= 8'h0;
            cmd_to_mem <= 1'b0;
            scsi_connected <= 1'b0;
            pending_dest_id <= 3'd0;
            aux_int_d <= 1'b0;
            dbg_irq_seen <= 1'b0;
            dbg_cmd_seen <= 1'b0;
            dbg_cmd_done_seen <= 1'b0;
            dbg_data_phase_seen <= 1'b0;
            dbg_data_xfer_seen <= 1'b0;
            dbg_dack_seen <= 1'b0;
            dbg_stat_read_seen <= 1'b0;
            dbg_aux_read_seen <= 1'b0;
            dbg_cmd_count <= 4'h0;
            dbg_cmd_done_count <= 4'h0;
            dbg_data_xfer_count <= 4'h0;
            dbg_stat_read_count <= 4'h0;
            for (i = 0; i <= 26; i = i + 1)
                regs[i] <= 8'h00;
        end else begin
            aux_int_d <= aux_int;
            if (aux_int && !aux_int_d) begin
                dbg_irq_seen <= 1'b1;
            end

            // State machine
            case (state)
                S_RESET_PENDING: begin
                    if (delay_cnt == 32'd0) begin
                        aux_cip <= 1'b0;
                        aux_int <= 1'b1;
                        // WD33C93A: return 0x01 when EAF (bit 3) set in OWN_ID
                        // Real IP20 has WD33C93A. Kernel checks this to enable
                        // advanced features (burst DMA, SELATNTR command mode).
                        regs[WD93STAT] <= regs[WD93ID][3] ? 8'h01 : 8'h00;
                        state <= S_IDLE;
                    end else begin
                        delay_cnt <= delay_cnt - 32'd1;
                    end
                end
                S_SELECT_PENDING: begin
                    if (delay_cnt == 32'd0) begin
                        aux_cip <= 1'b0;
                        aux_int <= 1'b1;
                        regs[WD93STAT] <= 8'h42;  // ST_TIMEOUT
                        regs[WD93PHASE] <= 8'h00; // PH_NOSELECT / bus free
                        regs[WD93LUN] <= 8'h00;
                        regs[WD93SRCID] <= {5'b00000, pending_dest_id};
                        scsi_connected <= 1'b0;
                        state <= S_IDLE;
                        if (trace_scsi)
                            $display("WD93: Select timeout for ID %0d, STAT=0x42",
                                pending_dest_id);
                    end else begin
                        delay_cnt <= delay_cnt - 32'd1;
                    end
                end
                S_DATA_PENDING: begin
                    // Wait for C++ to process command and fill scsi_buf
                    if (scsi_cmd_done) begin
                        dbg_cmd_done_seen <= 1'b1;
                        if (dbg_cmd_done_count != 4'hf)
                            dbg_cmd_done_count <= dbg_cmd_done_count + 4'd1;
                        pending_status <= scsi_cmd_status;
                        scsi_buf_idx <= 24'h0;
                        scsi_connected <= 1'b1;
                        if (xfer_count != 24'h0) begin
                            // Data phase: DRQ will assert on next cycle
                            state <= S_DATA_PHASE;
                            dbg_data_phase_seen <= 1'b1;
                            if (trace_scsi)
                                $display("WD93: Entering data phase, xfer_count=%0d to_mem=%0d resp_len=%0d",
                                    xfer_count, cmd_to_mem, scsi_resp_len);
                        end else begin
                            // No data phase (e.g. TEST UNIT READY)
                            aux_cip <= 1'b0;
                            aux_int <= 1'b1;
                            regs[WD93STAT] <= scsi_cmd_status;
                            regs[WD93PHASE] <= PH_COMPLETE;
                            regs[WD93LUN] <= 8'h00;    // SCSI status = GOOD
                            state <= S_IDLE;
                            scsi_connected <= 1'b0;
                            if (trace_scsi)
                                $display("WD93: Command complete (no data), STAT=0x%02x",
                                    scsi_cmd_status);
                        end
                    end
                end
                S_DATA_PHASE: begin
                    // DRQ/DACK data transfer. When the HPC DMA descriptor
                    // window ends before the target data phase is done, report
                    // the unexpected-data condition the PROM handles for
                    // remap. Preserve the WD count registers: save_datap()
                    // uses the residual count to decide whether there is more
                    // host buffer left to map.
                    if (dma_window_done && scsi_resp_len != 24'd0 &&
                        scsi_buf_idx < scsi_resp_len && xfer_count != 24'd0) begin
                        aux_cip <= 1'b0;
                        aux_int <= 1'b1;
                        regs[WD93STAT] <= unexpected_data_status;
                        regs[WD93PHASE] <= PH_DATA;
                        state <= S_DATA_PAUSE;
                        if (trace_scsi)
                            $display("WD93: DMA window ended, %0d/%0d bytes, resid=%0d STAT=0x%02x (more data)",
                                scsi_buf_idx, scsi_resp_len, xfer_count,
                                unexpected_data_status);
                    end else
                    if (data_phase_xfer) begin
                        dbg_data_xfer_seen <= 1'b1;
                        if (dbg_data_xfer_count != 4'hf)
                            dbg_data_xfer_count <= dbg_data_xfer_count + 4'd1;
                        if (dack_xfer)
                            dbg_dack_seen <= 1'b1;
                        if (cmd_to_mem) begin
                            // SCSI→mem: byte consumed from scsi_buf by DMA engine
                            scsi_buf_idx <= scsi_buf_idx + 24'd1;
                        end else begin
                            // mem→SCSI: byte provided by DMA engine into scsi_buf
`ifndef SYNTHESIS
                            scsi_buf[scsi_buf_idx] <= data_phase_write_data;
`endif
                            scsi_buf_idx <= scsi_buf_idx + 24'd1;
                        end
                        // Check for completion on programmed count expiry
                        if (xfer_count == 24'd1) begin
                            regs[WD93CNTHI] <= 8'h00;
                            regs[WD93CNTMD] <= 8'h00;
                            regs[WD93CNTLO] <= 8'h00;
                            aux_cip <= 1'b0;
                            aux_int <= 1'b1;
                            if ((scsi_buf_idx + 24'd1) < scsi_resp_len) begin
                                // Target has more data than programmed count
                                regs[WD93STAT] <= unexpected_data_status;
                                regs[WD93PHASE] <= PH_DATA;
                                state <= S_DATA_PAUSE;
                                if (trace_scsi)
                                    $display("WD93: Programmed count expired, %0d/%0d bytes, STAT=0x%02x (more data)",
                                        scsi_buf_idx + 24'd1, scsi_resp_len,
                                        unexpected_data_status);
                            end else if (use_external_target && !cmd_to_mem) begin
                                aux_cip <= 1'b1;
                                aux_int <= 1'b0;
                                regs[WD93STAT] <= pending_status;
                                regs[WD93PHASE] <= PH_DATA;  // while target commits write
                                state <= S_STATUS_PENDING;
                                if (trace_scsi)
                                    $display("WD93: Data-out received, waiting for target status (%0d bytes)",
                                        scsi_buf_idx + 24'd1);
                            end else begin
                                regs[WD93STAT] <= pending_status;
                                regs[WD93PHASE] <= PH_COMPLETE;
                                regs[WD93LUN] <= 8'h00;    // SCSI status = GOOD
                                state <= S_IDLE;
                                scsi_connected <= 1'b0;
                                if (trace_scsi)
                                    $display("WD93: Data phase complete, %0d bytes, STAT=0x%02x",
                                        scsi_buf_idx + 24'd1, pending_status);
                            end
                        end else begin
                            {regs[WD93CNTHI], regs[WD93CNTMD], regs[WD93CNTLO]} <=
                                xfer_count - 24'd1;
                        end
                    end
                    // SCSI target ran out of data before programmed count.
                    // Real WD33C93 reports unexpected status phase here; SGI's
                    // wd93 driver saves the residual count, forces count to 0,
                    // and resumes select-and-transfer to collect final status.
                    else if (scsi_buf_idx >= scsi_resp_len && scsi_resp_len != 24'd0) begin
                        regs[WD93STAT] <= ST_UNEX_SSTATUS;
                        regs[WD93PHASE] <= PH_IDENTRECV;
                        aux_cip <= 1'b0;
                        aux_int <= 1'b1;
                        state <= S_STATUS_PAUSE;
                        if (trace_scsi)
                            $display("WD93: Target ended data early, %0d/%0d bytes, resid=%0d STAT=0x%02x",
                                scsi_buf_idx, scsi_resp_len, xfer_count,
                                ST_UNEX_SSTATUS);
                    end
                end
                S_STATUS_PENDING: begin
                    if (scsi_status_valid) begin
                        aux_cip <= 1'b0;
                        aux_int <= 1'b1;
                        regs[WD93STAT] <= scsi_cmd_status;
                        regs[WD93PHASE] <= PH_COMPLETE;
                        regs[WD93LUN] <= 8'h00;    // SCSI status = GOOD
                        state <= S_IDLE;
                        scsi_connected <= 1'b0;
                        if (trace_scsi)
                            $display("WD93: Target write status ready, STAT=0x%02x",
                                scsi_cmd_status);
                    end
                end
                S_DATA_PAUSE: begin
                    // Waiting for PROM to handle the unexpected-data interrupt,
                    // remap DMA descriptors, reprogram transfer count, and
                    // re-issue C93SELATNTR. The command handler below will
                    // detect scsi_connected and resume S_DATA_PHASE.
                end
                S_STATUS_PAUSE: begin
                    // Waiting for the driver to resume after ST_UNEX_SSTATUS.
                    // It should have saved the residual and programmed count 0.
                end
                default: ;  // S_IDLE
            endcase

            // Bus access
            if (valid) begin
                if (write) begin
                    if (sel_addr) begin
                        // Write to address port: set register pointer
                        reg_ptr <= wdata;
                    end else begin
                        // Write to data port: write to selected register. During
                        // WD93 DATA-register PIO, the DATA register remains
                        // selected so repeated host byte writes stream data.
                        if (reg_ptr <= 8'd26 && !pio_data_write_access)
                            regs[reg_ptr[4:0]] <= wdata;

                        // Log CTRL writes; 0x2d selects 93A burst DMA.
`ifndef SYNTHESIS
                        if (trace_scsi && reg_ptr == WD93CTRL)
                            $display("WD93: CTRL write=0x%02x", wdata);
`endif

                        // Auto-increment address register after data port write,
                        // except for programmed-I/O DATA transfers.
                        if (!pio_data_write_access)
                            reg_ptr <= reg_ptr + 8'd1;

                        // Command register write triggers state machine
                        if (reg_ptr == WD93CMD) begin
                            case (wdata)
                                C93RESET: begin
                                    aux_cip   <= 1'b1;
                                    aux_int   <= 1'b0;
                                    state     <= S_RESET_PENDING;
                                    delay_cnt <= 32'd100;
                                    pending_dest_id <= 3'd0;
                                    scsi_connected <= 1'b0;
                                    // Clear all registers on reset (real WD33C93 behavior).
                                    // Critical: reg 3 (WD93C1) must be 0 so software reads
                                    // microcode rev as 0 → detects D_WD93A (not D_WD93B).
                                    // Stale CDB data in reg 3 (e.g. 0x12 from INQUIRY)
                                    // would cause false D_WD93B detection.
                                    for (i = 0; i <= 26; i = i + 1)
                                        if (i != WD93ID)  // preserve OWN_ID (has EAF bit)
                                            regs[i] <= 8'h00;
                                    if (trace_scsi)
                                        $display("WD93: RESET command issued");
                                end
                                C93SELECT, C93SELECT_ATN: begin
                                    // C93SELATN (0x06) = select with ATN. Real WD33C93
                                    // generates two interrupts: first ST_SELECT (0x11)
                                    // on selection complete, then ST_REQ_SMESGOUT (0x8e)
                                    // when target requests message out phase.
                                    //
                                    // Bug history: we previously returned ST_SATOK (0x16)
                                    // which is only valid for SELATNTR (select-and-transfer).
                                    // The initiator saw non-ST_SELECT and returned
                                    // SC_HARDERR, repeatedly retrying sync negotiation.
                                    //
                                    // Now: return ST_SELECT (0x11) on first interrupt but
                                    // DO NOT generate the second interrupt. The initiator's
                                    // wait_scintr 5ms timeout triggers SC_TIMEOUT, which
                                    // sets S_CANTSYNC for this target, and subsequent
                                    // commands skip sync negotiation.
                                    //
                                    // This is a compromise: proper implementation would
                                    // model two-interrupt SELATN + message-out transfer.
                                    aux_cip <= 1'b0;
                                    aux_int <= 1'b1;
                                    if (dest_present) begin
                                        regs[WD93STAT] <= 8'h11;  // ST_SELECT (selection done)
                                        regs[WD93PHASE] <= 8'h10; // PH_SELECT (selection successful)
                                        regs[WD93SRCID] <= {5'b00000, dest_id};
                                        if (trace_scsi)
                                            $display("WD93: SELECT cmd=0x%02x to ID %0d (ST_SELECT, 2nd intr will timeout)",
                                                wdata, dest_id);
                                    end else begin
                                        pending_dest_id <= dest_id;
                                        regs[WD93SRCID] <= {5'b00000, dest_id};
                                        scsi_connected <= 1'b0;
                                        state     <= S_SELECT_PENDING;
                                        delay_cnt <= SELECT_TIMEOUT_CYCLES;
                                        aux_cip <= 1'b1;
                                        aux_int <= 1'b0;
                                        if (trace_scsi)
                                            $display("WD93: SELECT cmd=0x%02x to ID %0d (timeout)",
                                                wdata, dest_id);
                                    end
                                end
                                C93SELECT_XFER, C93SELNATNTR: begin
                                    dbg_cmd_seen <= 1'b1;
                                    if (dbg_cmd_count != 4'hf)
                                        dbg_cmd_count <= dbg_cmd_count + 4'd1;
                                    aux_cip <= 1'b1;
                                    aux_int <= 1'b0;
                                    if (scsi_connected) begin
                                        if (state == S_STATUS_PAUSE) begin
                                            aux_cip <= 1'b0;
                                            aux_int <= 1'b1;
                                            regs[WD93STAT] <= pending_status;
                                            regs[WD93PHASE] <= PH_COMPLETE;
                                            regs[WD93LUN] <= 8'h00;    // SCSI status = GOOD
                                            state <= S_IDLE;
                                            scsi_connected <= 1'b0;
                                            if (trace_scsi)
                                                $display("WD93: RESUME status complete, STAT=0x%02x",
                                                    pending_status);
                                        end else begin
                                            // Resume: target already connected, scsi_buf valid.
                                            // Driver remapped DMA and reprogrammed transfer count.
                                            // Go directly to data phase (scsi_buf_idx preserved).
                                            state <= S_DATA_PHASE;
                                            dbg_data_phase_seen <= 1'b1;
                                            if (trace_scsi)
                                                $display("WD93: RESUME data phase from byte %0d, xfer_count=%0d",
                                                    scsi_buf_idx, xfer_count);
                                        end
                                    end else begin
                                        // New command: determine direction from CDB opcode
                                        case (regs[5'd3])
                                            8'h0A, 8'h2A: cmd_to_mem <= 1'b0;  // WRITE(6), WRITE(10)
                                            default:      cmd_to_mem <= 1'b1;  // Everything else reads to memory
                                        endcase
                                        if (dest_present) begin
                                            regs[WD93SRCID] <= {5'b00000, dest_id};
                                            state <= S_DATA_PENDING;
                                            if (trace_scsi)
                                                $display("WD93: SELATNTR cmd=0x%02x to ID %0d (PRESENT) CDB=%02x%02x%02x%02x%02x%02x xfer=%0d",
                                                    wdata, dest_id,
                                                    regs[5'd3], regs[5'd4], regs[5'd5],
                                                    regs[5'd6], regs[5'd7], regs[5'd8],
                                                    xfer_count);
                                        end else begin
                                            pending_dest_id <= dest_id;
                                            regs[WD93SRCID] <= {5'b00000, dest_id};
                                            scsi_connected <= 1'b0;
                                            state     <= S_SELECT_PENDING;
                                            delay_cnt <= SELECT_TIMEOUT_CYCLES;
                                            if (trace_scsi)
                                                $display("WD93: SELATNTR cmd=0x%02x to ID %0d (timeout)",
                                                    wdata, dest_id);
                                        end
                                    end
                                end
                                C93ABORT: begin
                                    aux_int <= 1'b1;
                                    regs[WD93STAT] <= 8'h42;
                                    state <= S_IDLE;
                                    if (trace_scsi)
                                        $display("WD93: ABORT command");
                                end
                                C93DISC: begin
                                    aux_cip <= 1'b0;
                                    // A host-issued Disconnect releases the bus
                                    // immediately and does not generate a completion
                                    // interrupt.  ST_DISCONNECT is reserved for a
                                    // target-side disconnect event.
                                    aux_int <= 1'b0;
                                    regs[WD93PHASE] <= 8'h00; // PH_NOSELECT (bus free)
                                    scsi_connected <= 1'b0;
                                    state <= S_IDLE;
                                    if (trace_scsi)
                                        $display("WD93: DISCONNECT command");
                                end
                                default: begin
                                    aux_int <= 1'b1;
                                    regs[WD93STAT] <= 8'h42;
                                    $display("WD93: unhandled cmd=0x%02x", wdata);
                                end
                            endcase
                        end
                    end
                end else begin
                    // Read side effects
                    if (!sel_addr) begin
                        // Log OWN_ID readback for driver probe test
`ifndef SYNTHESIS
                        if (trace_scsi && reg_ptr == WD93ID)
                            $display("WD93: OWN_ID read=0x%02x", regs[WD93ID]);
`endif
                        // Reading status register clears interrupt
                        if (reg_ptr == WD93STAT) begin
                            dbg_stat_read_seen <= 1'b1;
                            if (dbg_stat_read_count != 4'hf)
                                dbg_stat_read_count <= dbg_stat_read_count + 4'd1;
                            aux_int <= 1'b0;
`ifndef SYNTHESIS
                            if (trace_scsi)
                                $display("WD93: STAT read=0x%02x PHASE=0x%02x LUN=0x%02x",
                                    regs[WD93STAT], regs[WD93PHASE], regs[WD93LUN]);
`endif
                        end
                        // Auto-increment address register after data port read,
                        // except for programmed-I/O DATA transfers.
                        if (!pio_data_read_access)
                            reg_ptr <= reg_ptr + 8'd1;
                    end else begin
                        dbg_aux_read_seen <= 1'b1;
                        // Trace AUX_STATUS polling with the controller state.
`ifndef SYNTHESIS
                        if (trace_scsi && sel_addr)
                            $display("WD93_AUX: aux=%02x CIP=%b BSY=%b INT=%b state=%0d",
                                aux_status, aux_cip, 1'b0, aux_int, state);
`endif
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
