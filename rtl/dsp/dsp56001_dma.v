// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - HPC1 DSP DMA engine
//
// The IP20 DSP interface moves one 24-bit PBUS SRAM word for each 32-bit
// GIO memory word in normal mode.  Folded mode moves a pair of left-justified
// 16-bit samples between two 24-bit DSP words and one 32-bit GIO word.
// DMACTRL[1] selects the direction (0 = GIO to DSP, 1 = DSP to GIO), while
// DMACTRL[2] selects folded mode.  DMAWDCNT always counts DSP SRAM words.
//
// Both downstream interfaces use completion handshakes.  A request and all
// of its payload remain stable until the corresponding ready input is high.

`default_nettype none

module dsp56001_dma (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        dma_go,
    input  wire        dma_dir,
    input  wire        dma_mode,
    input  wire [15:0] dma_count,
    input  wire [31:0] dma_gio_addr,
    input  wire [15:0] dma_pbus_addr,
    input  wire [2:0]  dma_sign_mode,

    output reg         dma_done,
    output wire        dma_active,
    output wire [15:0] dma_count_current,
    output wire        dma_folded_unsupported,

    output wire [14:0] sram_addr,
    output wire [23:0] sram_wdata,
    input  wire [23:0] sram_rdata,
    output wire        sram_rd_en,
    output wire        sram_wr_en,
    input  wire        sram_ready,

    output wire [31:0] mem_addr,
    output wire [63:0] mem_wdata,
    input  wire [63:0] mem_rdata,
    output wire [7:0]  mem_wmask,
    output wire        mem_valid,
    output wire        mem_write,
    input  wire        mem_ready
);
    localparam [2:0] S_IDLE       = 3'd0;
    localparam [2:0] S_SRAM_READ  = 3'd1;
    localparam [2:0] S_MEM_WRITE  = 3'd2;
    localparam [2:0] S_MEM_READ   = 3'd3;
    localparam [2:0] S_SRAM_WRITE = 3'd4;
    localparam [2:0] S_WAIT_GO_LO = 3'd5;
    localparam [2:0] S_FOLD_READ2 = 3'd6;
    localparam [2:0] S_FOLD_WRITE2 = 3'd7;

    reg [2:0]  state;
    reg        dir_r;
    reg        mode_r;
    reg [2:0]  sign_mode_r;
    reg [15:0] remaining;
    reg [31:0] gio_addr_r;
    reg [15:0] pbus_addr_r;
    reg [23:0] sram_word_r;
    reg [23:0] folded_second_word_r;
    reg [31:0] gio_word_r;

    function [31:0] extend_pbus_word;
        input [23:0] word;
        input [2:0]  sign_mode;
        begin
            case (sign_mode)
                3'b111: extend_pbus_word = {{24{word[7]}}, word[7:0]};
                3'b110: extend_pbus_word = {{16{word[15]}}, word[15:0]};
                3'b100: extend_pbus_word = {{8{word[23]}}, word};
                default: extend_pbus_word = {8'd0, word};
            endcase
        end
    endfunction

    wire [31:0] extended_sram_word =
        extend_pbus_word(sram_word_r, sign_mode_r);

    assign dma_active = (state != S_IDLE) && (state != S_WAIT_GO_LO);
    assign dma_count_current = remaining;
    assign dma_folded_unsupported = 1'b0;

    // PBUSADD is the DSP/Y-space address. The SRAM occupies 32K
    // words, so its physical port is the low 15 bits (Y:$C000 -> SRAM $4000).
    assign sram_addr = pbus_addr_r[14:0];
    assign sram_wdata = mode_r ?
                        ((state == S_FOLD_WRITE2) ?
                         {gio_word_r[15:0], 8'd0} :
                         {gio_word_r[31:16], 8'd0}) :
                        gio_word_r[23:0];
    assign sram_rd_en = (state == S_SRAM_READ) ||
                        (state == S_FOLD_READ2);
    assign sram_wr_en = (state == S_SRAM_WRITE) ||
                        (state == S_FOLD_WRITE2);

    assign mem_addr = {gio_addr_r[31:3], 3'b000};
    wire [31:0] folded_mem_word =
        {sram_word_r[23:8], folded_second_word_r[23:8]};
    wire [31:0] selected_write_word =
        mode_r ? folded_mem_word : extended_sram_word;
    assign mem_wdata = gio_addr_r[2] ?
                       {32'd0, selected_write_word} :
                       {selected_write_word, 32'd0};
    assign mem_wmask = (state == S_MEM_WRITE) ?
                       (mode_r && remaining == 16'd1 ?
                        (gio_addr_r[2] ? 8'h0c : 8'hc0) :
                        (gio_addr_r[2] ? 8'h0f : 8'hf0)) : 8'h00;
    assign mem_valid = (state == S_MEM_WRITE) || (state == S_MEM_READ);
    assign mem_write = (state == S_MEM_WRITE);

    wire [31:0] selected_mem_word =
        gio_addr_r[2] ? mem_rdata[31:0] : mem_rdata[63:32];

    task advance_or_complete;
        begin
            if (remaining == 16'd1) begin
                remaining <= 16'd0;
                dma_done <= 1'b1;
                state <= S_WAIT_GO_LO;
            end else begin
                remaining <= remaining - 16'd1;
                gio_addr_r <= gio_addr_r + 32'd4;
                pbus_addr_r <= pbus_addr_r + 16'd1;
                state <= dir_r ? S_SRAM_READ : S_MEM_READ;
            end
        end
    endtask

    task advance_folded_or_complete;
        begin
            if (remaining <= 16'd2) begin
                remaining <= 16'd0;
                dma_done <= 1'b1;
                state <= S_WAIT_GO_LO;
            end else begin
                remaining <= remaining - 16'd2;
                gio_addr_r <= gio_addr_r + 32'd4;
                // The first word of a full pair advanced PBUSADD before the
                // second access.  Advance once more to the next pair here.
                pbus_addr_r <= pbus_addr_r + 16'd1;
                state <= dir_r ? S_SRAM_READ : S_MEM_READ;
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            dir_r <= 1'b0;
            mode_r <= 1'b0;
            sign_mode_r <= 3'd0;
            remaining <= 16'd0;
            gio_addr_r <= 32'd0;
            pbus_addr_r <= 16'd0;
            sram_word_r <= 24'd0;
            folded_second_word_r <= 24'd0;
            gio_word_r <= 32'd0;
            dma_done <= 1'b0;
        end else begin
            dma_done <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (dma_go) begin
                        dir_r <= dma_dir;
                        mode_r <= dma_mode;
                        sign_mode_r <= dma_sign_mode;
                        remaining <= dma_count;
                        gio_addr_r <= dma_gio_addr;
                        pbus_addr_r <= dma_pbus_addr;
                        if (dma_count == 16'd0) begin
                            dma_done <= 1'b1;
                            state <= S_WAIT_GO_LO;
                        end else begin
                            state <= dma_dir ? S_SRAM_READ : S_MEM_READ;
                        end
                    end
                end

                S_SRAM_READ: begin
                    if (sram_ready) begin
                        sram_word_r <= sram_rdata;
                        if (mode_r && remaining > 16'd1) begin
                            pbus_addr_r <= pbus_addr_r + 16'd1;
                            state <= S_FOLD_READ2;
                        end else begin
                            folded_second_word_r <= 24'd0;
                            state <= S_MEM_WRITE;
                        end
                    end
                end

                S_FOLD_READ2: begin
                    if (sram_ready) begin
                        folded_second_word_r <= sram_rdata;
                        state <= S_MEM_WRITE;
                    end
                end

                S_MEM_WRITE: begin
                    if (mem_ready) begin
                        if (mode_r)
                            advance_folded_or_complete();
                        else
                            advance_or_complete();
                    end
                end

                S_MEM_READ: begin
                    if (mem_ready) begin
                        gio_word_r <= selected_mem_word;
                        state <= S_SRAM_WRITE;
                    end
                end

                S_SRAM_WRITE: begin
                    if (sram_ready) begin
                        if (mode_r && remaining > 16'd1) begin
                            pbus_addr_r <= pbus_addr_r + 16'd1;
                            state <= S_FOLD_WRITE2;
                        end else if (mode_r) begin
                            advance_folded_or_complete();
                        end else begin
                            advance_or_complete();
                        end
                    end
                end

                S_FOLD_WRITE2: begin
                    if (sram_ready)
                        advance_folded_or_complete();
                end

                S_WAIT_GO_LO: begin
                    if (!dma_go)
                        state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
