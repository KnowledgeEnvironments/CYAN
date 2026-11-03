// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - HPC1 SCSI DMA Engine
// Walks HPC1 descriptor chain, transfers data between WD33C93 and system
// memory via bus-master transactions.
//
// Data path: WD33C93 asserts DRQ with a byte, DMA engine writes it to
// memory at the current descriptor's buffer pointer. For mem→SCSI,
// DMA reads from memory and provides the byte back via DACK.
//
// Transfers use doubleword (8-byte) bus transactions. Bytes are
// accumulated until a doubleword boundary is crossed or the descriptor
// byte count is exhausted, then flushed as a single bus write.
// Reads fetch a full doubleword and extract bytes as needed.
//
// Descriptor format (3 x 32-bit words, 12 bytes):
//   Word 0: bcnt[12:0] = byte count, fill[31:13]
//   Word 1: cbp[27:0] = buffer pointer, efill[30:28], eox[31] = end of chain
//   Word 2: nbp[27:0] = next descriptor pointer, nfill[31:28]

`default_nettype none

module hpc1_scsi_dma (
    input  wire        clk,
    input  wire        rst_n,

    // Control (from HPC1 SCSI DMA registers)
    input  wire [31:0] nbdp,           // Next buffer descriptor pointer register
    input  wire        startdma,       // SCSI_STARTDMA bit (scsi_ctrl[7])
    input  wire        flushdma,       // SCSI_FLUSH bit (scsi_ctrl[1])
    input  wire        to_mem,         // SCSI_TO_MEM bit (scsi_ctrl[4])
    output reg         clear_startdma, // Pulse: tells HPC1 to clear STARTDMA

    // DRQ/DACK interface (from/to WD33C93)
    input  wire        drq,            // WD93 has byte ready (to_mem) or needs byte (from_mem)
    output reg         dack,           // Byte transferred
    input  wire [7:0]  drq_byte,       // Byte from WD93 (SCSI→mem)
    output reg  [7:0]  dack_byte,      // Byte to WD93 (mem→SCSI)

    // Status
    output wire        dma_active,     // DMA engine is busy

    // Bus master interface (to testbench → C++ memory)
    output reg  [31:0] dma_addr,
    output reg  [63:0] dma_wdata,
    input  wire [63:0] dma_rdata,
    output reg  [ 7:0] dma_wmask,
    output reg         dma_valid,
    output reg         dma_write,
    input  wire        dma_ready
);

    // State machine
    localparam [3:0]
        S_IDLE        = 4'd0,
        S_FETCH_W0    = 4'd1,   // Issue read for descriptor word 0
        S_FETCH_W0_W  = 4'd2,   // Wait for word 0 response
        S_FETCH_W1    = 4'd3,   // Issue read for descriptor word 1
        S_FETCH_W1_W  = 4'd4,   // Wait for word 1 response
        S_FETCH_W2    = 4'd5,   // Issue read for descriptor word 2
        S_FETCH_W2_W  = 4'd6,   // Wait for word 2 response
        S_XFER        = 4'd7,   // Wait for DRQ, accumulate/extract bytes
        S_FLUSH       = 4'd8,   // Write accumulated doubleword to memory
        S_FLUSH_WAIT  = 4'd9,   // Wait for write response
        S_READ        = 4'd10,  // Issue doubleword read from memory
        S_READ_WAIT   = 4'd11,  // Wait for read response
        S_DONE        = 4'd12,
        S_DACK_WAIT   = 4'd13,  // Wait 1 cycle after DACK for WD93 to advance byte
        S_ABORT_WAIT  = 4'd14;  // Retire an accepted read before stopping/restarting

    reg [3:0]  state;
    reg [31:0] desc_addr;       // Physical address of current descriptor
    reg [12:0] desc_bcnt;       // Byte count remaining in current descriptor
    reg [27:0] desc_cbp;        // Current buffer pointer (destination/source in memory)
    reg        desc_eox;        // End-of-transfer flag
    reg [27:0] desc_nbp;        // Next descriptor pointer
    reg        desc_zero_count; // SGI chains terminate on a fetched zero-byte descriptor

    // Doubleword write accumulator
    reg [63:0] wr_buf;          // Accumulated bytes for next bus write
    reg [7:0]  wr_mask;         // Byte enable mask
    reg [31:0] wr_addr;         // Aligned address for pending write
    reg        abort_after_flush; // Complete a software abort after draining writes

    // Doubleword read cache
    reg [63:0] rd_buf;          // Cached doubleword from last read
    reg [31:0] rd_buf_addr;     // Aligned address of cached doubleword
    reg        rd_buf_valid;    // Cache contains valid data

`ifndef SYNTHESIS
    reg trace_dma;
    initial begin
        trace_dma = $test$plusargs("trace_dma") || $test$plusargs("trace_hpc1");
    end
`endif

    // STARTDMA edge detect
    reg        startdma_prev;
    wire       startdma_edge = startdma && !startdma_prev;

    assign dma_active = (state != S_IDLE);

    // Extract 32-bit word from 64-bit bus read based on address alignment
    wire [31:0] fetched_upper = dma_rdata[63:32];
    wire [31:0] fetched_lower = dma_rdata[31:0];

    // Current doubleword-aligned address for desc_cbp
    wire [31:0] cbp_aligned = {4'b0, desc_cbp} & ~32'h7;

    // Extract byte from cached read buffer based on address
    reg [7:0] rd_byte;
    always @(*) begin
        case (desc_cbp[2:0])
            3'd0: rd_byte = rd_buf[63:56];
            3'd1: rd_byte = rd_buf[55:48];
            3'd2: rd_byte = rd_buf[47:40];
            3'd3: rd_byte = rd_buf[39:32];
            3'd4: rd_byte = rd_buf[31:24];
            3'd5: rd_byte = rd_buf[23:16];
            3'd6: rd_byte = rd_buf[15:8];
            3'd7: rd_byte = rd_buf[7:0];
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            dma_valid <= 1'b0;
            dma_write <= 1'b0;
            dma_addr <= 32'h0;
            dma_wdata <= 64'h0;
            dma_wmask <= 8'h0;
            dack <= 1'b0;
            dack_byte <= 8'h0;
            clear_startdma <= 1'b0;
            desc_addr <= 32'h0;
            desc_bcnt <= 13'h0;
            desc_cbp <= 28'h0;
            desc_eox <= 1'b0;
            desc_nbp <= 28'h0;
            desc_zero_count <= 1'b0;
            wr_buf <= 64'h0;
            wr_mask <= 8'h0;
            wr_addr <= 32'h0;
            abort_after_flush <= 1'b0;
            rd_buf <= 64'h0;
            rd_buf_addr <= 32'h0;
            rd_buf_valid <= 1'b0;
            startdma_prev <= 1'b0;
        end else begin
            clear_startdma <= 1'b0;
            dma_valid <= 1'b0;
            dack <= 1'b0;
            startdma_prev <= startdma;

            // Stop if software clears STARTDMA, or drain on SCSI_FLUSH.
            // On real HPC1, FLUSH does not itself clear STARTDMA: software sets
            // FLUSH and polls STARTDMA as a completion fence before using the
            // received bytes.  Keep STARTDMA asserted until the final bus
            // write has actually been accepted.
            if (state != S_IDLE && state != S_FLUSH &&
                    state != S_FLUSH_WAIT &&
                    state != S_ABORT_WAIT &&
                    (!startdma || flushdma) && !abort_after_flush) begin
`ifndef SYNTHESIS
                if (trace_dma)
                    $display("DMA_ABORT: state=%0d bcnt=%0d eox=%b drq=%b",
                        state, desc_bcnt, desc_eox, drq);
`endif
                // The MC/MIG handshake completes accepted reads even after
                // dma_valid drops.  Do not let a late payload/descriptor
                // response become the response to a replacement DMA's first
                // descriptor fetch.  Hold the old request until it retires.
                if (state == S_FETCH_W0_W || state == S_FETCH_W1_W ||
                        state == S_FETCH_W2_W || state == S_READ_WAIT) begin
                    if (dma_ready) begin
                        state <= S_IDLE;
                        // A software stop already cleared STARTDMA.  Do not
                        // emit a delayed auto-clear that could erase a new
                        // wd93_go() written on this same clock edge.
                        if (flushdma)
                            clear_startdma <= 1'b1;
                    end else begin
                        dma_valid <= 1'b1;
                        dma_write <= 1'b0;
                        state <= S_ABORT_WAIT;
                    end
                end else if (to_mem && |wr_mask) begin
                    abort_after_flush <= 1'b1;
                    state <= S_FLUSH;
                end else begin
                    state <= S_IDLE;
                    if (flushdma)
                        clear_startdma <= 1'b1;
                end
                rd_buf_valid <= 1'b0;
                // $display("DMA: Aborted (STARTDMA cleared) state=%0d desc_addr=0x%08x", state, desc_addr);
            end else case (state)
                S_IDLE: begin
                    if (startdma_edge) begin
                        desc_addr <= nbdp;
                        wr_buf <= 64'h0;
                        wr_mask <= 8'h0;
                        abort_after_flush <= 1'b0;
                        rd_buf_valid <= 1'b0;
                        desc_zero_count <= 1'b0;
                        state <= S_FETCH_W0;
                    end
                end

                // --- Descriptor fetch: 3 words, 3 bus reads ---

                S_FETCH_W0: begin
                    dma_addr <= desc_addr & ~32'h7;
                    dma_valid <= 1'b1;
                    dma_write <= 1'b0;
                    state <= S_FETCH_W0_W;
                end

                S_FETCH_W0_W: begin
                    if (dma_ready) begin
                        desc_bcnt <= desc_addr[2] ? fetched_lower[12:0] : fetched_upper[12:0];
                        desc_zero_count <= (desc_addr[2] ? fetched_lower[12:0] : fetched_upper[12:0]) == 13'h0;
                        state <= S_FETCH_W1;
                        // $display("DMA: Fetch W0 done, bcnt=%0d data=0x%016x",
                        //     desc_addr[2] ? fetched_lower[12:0] : fetched_upper[12:0], dma_rdata);
                    end else begin
                        dma_valid <= 1'b1;  // Hold valid until MC responds
                    end
                end

                S_FETCH_W1: begin
                    dma_addr <= (desc_addr + 32'd4) & ~32'h7;
                    dma_valid <= 1'b1;
                    dma_write <= 1'b0;
                    state <= S_FETCH_W1_W;
                end

                S_FETCH_W1_W: begin
                    if (dma_ready) begin
                        if (desc_addr[2]) begin
                            desc_cbp <= fetched_upper[27:0];
                            desc_eox <= fetched_upper[31];
                        end else begin
                            desc_cbp <= fetched_lower[27:0];
                            desc_eox <= fetched_lower[31];
                        end
                        state <= S_FETCH_W2;
                    end else begin
                        dma_valid <= 1'b1;  // Hold valid until MC responds
                    end
                end

                S_FETCH_W2: begin
                    dma_addr <= (desc_addr + 32'd8) & ~32'h7;
                    dma_valid <= 1'b1;
                    dma_write <= 1'b0;
                    state <= S_FETCH_W2_W;
                end

                S_FETCH_W2_W: begin
                    if (dma_ready) begin
                        if (desc_addr[2])
                            desc_nbp <= fetched_lower[27:0];
                        else
                            desc_nbp <= fetched_upper[27:0];
                        state <= S_XFER;
                    end else begin
                        dma_valid <= 1'b1;  // Hold valid until MC responds
                    end
                end

                // --- Data transfer ---

                S_XFER: begin
                    if (desc_bcnt == 13'h0) begin
                        // Descriptor exhausted — flush any pending write, then next descriptor or done
                        if (|wr_mask) begin
                            state <= S_FLUSH;
                        end else if (desc_zero_count) begin
                            state <= S_DONE;
                        end else if (desc_eox) begin
                            state <= S_DONE;
                        end else begin
                            desc_addr <= {4'b0, desc_nbp};
                            state <= S_FETCH_W0;
                        end
                    end else if (drq && to_mem) begin
                        // SCSI→Memory: accept byte from WD93, accumulate in write buffer
                        dack <= 1'b1;
                        case (desc_cbp[2:0])
                            3'd0: begin wr_buf[63:56] <= drq_byte; wr_mask[7] <= 1'b1; end
                            3'd1: begin wr_buf[55:48] <= drq_byte; wr_mask[6] <= 1'b1; end
                            3'd2: begin wr_buf[47:40] <= drq_byte; wr_mask[5] <= 1'b1; end
                            3'd3: begin wr_buf[39:32] <= drq_byte; wr_mask[4] <= 1'b1; end
                            3'd4: begin wr_buf[31:24] <= drq_byte; wr_mask[3] <= 1'b1; end
                            3'd5: begin wr_buf[23:16] <= drq_byte; wr_mask[2] <= 1'b1; end
                            3'd6: begin wr_buf[15:8]  <= drq_byte; wr_mask[1] <= 1'b1; end
                            3'd7: begin wr_buf[7:0]   <= drq_byte; wr_mask[0] <= 1'b1; end
                        endcase
                        wr_addr <= cbp_aligned;
                        desc_cbp <= desc_cbp + 28'd1;
                        desc_bcnt <= desc_bcnt - 13'd1;
                        // Flush when we fill last byte of doubleword or exhaust descriptor
                        if (desc_cbp[2:0] == 3'd7 || desc_bcnt == 13'd1) begin
                            state <= S_FLUSH;
                        end else begin
                            // Wait 1 cycle for WD93 to advance scsi_buf_idx
                            state <= S_DACK_WAIT;
                        end
                    end else if (drq && !to_mem) begin
                        // Memory→SCSI: provide byte from memory to WD93
                        if (rd_buf_valid && rd_buf_addr == cbp_aligned) begin
                            // Have cached doubleword — extract byte
                            dack <= 1'b1;
                            dack_byte <= rd_byte;
                            desc_cbp <= desc_cbp + 28'd1;
                            desc_bcnt <= desc_bcnt - 13'd1;
                            // Invalidate cache if crossing to next doubleword
                            if (desc_cbp[2:0] == 3'd7)
                                rd_buf_valid <= 1'b0;
                            // Wait 1 cycle for WD93 to advance scsi_buf_idx
                            state <= S_DACK_WAIT;
                        end else begin
                            // Need to read doubleword from memory
                            state <= S_READ;
                        end
                    end
                    // DRQ may drop while a target backend refills data, for
                    // example while the SD SPI path reads the next sector.
                    // Real HPC DMA keeps the descriptor active through these
                    // gaps and completes from descriptor exhaustion or a
                    // software STARTDMA clear.
                end

                // --- Doubleword write flush ---

                S_FLUSH: begin
                    dma_addr <= wr_addr;
                    dma_wdata <= wr_buf;
                    dma_wmask <= wr_mask;
                    dma_valid <= 1'b1;
                    dma_write <= 1'b1;
                    if (!startdma)
                        abort_after_flush <= 1'b1;
                    state <= S_FLUSH_WAIT;
                end

                S_FLUSH_WAIT: begin
                    if (!startdma)
                        abort_after_flush <= 1'b1;
                    if (dma_ready) begin
                        wr_buf <= 64'h0;
                        wr_mask <= 8'h0;
                        if (flushdma) begin
                            abort_after_flush <= 1'b0;
                            clear_startdma <= 1'b1;
                            state <= S_IDLE;
                        end else if (abort_after_flush || !startdma) begin
                            abort_after_flush <= 1'b0;
                            if (startdma) begin
                                // A replacement wd93_go() arrived while a
                                // software-stopped inbound write was draining.
                                // The old write is now durable; fetch the new
                                // descriptor without generating a stale clear.
                                desc_addr <= nbdp;
                                wr_buf <= 64'h0;
                                wr_mask <= 8'h0;
                                rd_buf_valid <= 1'b0;
                                desc_zero_count <= 1'b0;
                                state <= S_FETCH_W0;
                            end else begin
                                state <= S_IDLE;
                            end
                        end else if (!startdma) begin
                            // Software already cleared STARTDMA while this
                            // ordinary descriptor write was in flight.
                            state <= S_IDLE;
                        end else if (desc_bcnt == 13'h0) begin
                            if (desc_eox) begin
                                state <= S_DONE;
                            end else if (desc_zero_count) begin
                                state <= S_DONE;
                            end else begin
                                desc_addr <= {4'b0, desc_nbp};
                                state <= S_FETCH_W0;
                            end
                        end else begin
                            state <= S_XFER;
                        end
                    end else begin
                        dma_valid <= 1'b1;  // Hold valid until MC responds
                        dma_write <= 1'b1;
                    end
                end

                // --- Doubleword read (for mem→SCSI) ---

                S_READ: begin
                    dma_addr <= cbp_aligned;
                    dma_valid <= 1'b1;
                    dma_write <= 1'b0;
                    state <= S_READ_WAIT;
                end

                S_READ_WAIT: begin
                    if (dma_ready) begin
                        rd_buf <= dma_rdata;
                        rd_buf_addr <= cbp_aligned;
                        rd_buf_valid <= 1'b1;
                        state <= S_XFER;
                    end else begin
                        dma_valid <= 1'b1;  // Hold valid until MC responds
                    end
                end

                // --- Wait for WD93 to advance after DACK ---

                S_DACK_WAIT: begin
                    // 1-cycle delay: WD93 sees DACK and advances scsi_buf_idx
                    state <= S_XFER;
                end

                // --- Retire an accepted read after software abort ---

                S_ABORT_WAIT: begin
                    dma_valid <= 1'b1;
                    dma_write <= 1'b0;
                    if (dma_ready) begin
                        dma_valid <= 1'b0;
                        rd_buf_valid <= 1'b0;
                        if (startdma && !flushdma) begin
                            // IP20 does not poll completion after stopping a
                            // memory-to-SCSI DMA.  Preserve an immediate
                            // reconnect/new wd93_go() that arrived while the
                            // old DDR response was draining.
                            desc_addr <= nbdp;
                            wr_buf <= 64'h0;
                            wr_mask <= 8'h0;
                            abort_after_flush <= 1'b0;
                            desc_zero_count <= 1'b0;
                            state <= S_FETCH_W0;
                        end else begin
                            // FLUSH is a completion fence and must auto-clear
                            // STARTDMA.  A plain software stop already cleared
                            // the bit and must not leave a stale clear pulse
                            // that can kill a replacement DMA.
                            if (flushdma)
                                clear_startdma <= 1'b1;
                            state <= S_IDLE;
                        end
                    end
                end

                // --- Transfer complete ---

                S_DONE: begin
                    clear_startdma <= 1'b1;
                    state <= S_IDLE;
`ifndef SYNTHESIS
                    if (trace_dma)
                        $display("DMA_DONE: normal completion");
`endif
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
