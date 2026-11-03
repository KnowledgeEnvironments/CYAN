// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

//////////////////////////////////////////////////////////////////////////////
// DSP56001 Memory Subsystem
//
// Internal Harvard architecture memories:
//   P-RAM:  512 x 24-bit (addresses $0000-$01FF)
//   X-RAM:  256 x 24-bit (addresses $0000-$00FF)
//   X-ROM:  256 x 24-bit (addresses $0100-$01FF, mu-law/A-law tables)
//   Y-RAM:  256 x 24-bit (addresses $0000-$00FF)
//   Y-ROM:  256 x 24-bit (addresses $0100-$01FF, sine table)
//
// Address routing:
//   P: $0000-$01FF -> internal RAM
//      $0200-$FFFF -> external SRAM
//   X: $0000-$00FF -> internal RAM
//      $0100-$01FF -> internal ROM
//      $0200-$FFBF -> external SRAM
//      $FFC0-$FFFF -> X-peripheral space
//   Y: $0000-$00FF -> internal RAM
//      $0100-$01FF -> internal ROM
//      $0200-$FFBF -> external SRAM
//      $FFC0-$FFDF -> external DP8573 RTC window
//      $FFE0-$FFFF -> Y-peripheral space (HPC1 registers)
//////////////////////////////////////////////////////////////////////////////

module dsp56001_mem (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        data_rom_enable,

    // ========================================================================
    // P-space Interface (instruction fetch + MOVEM)
    // ========================================================================
    input  wire [15:0] p_addr,
    input  wire        p_rd_en,
    input  wire        p_wr_en,
    input  wire [23:0] p_wdata,
    output wire [23:0] p_rdata,
    output wire        p_ready,

    // ========================================================================
    // X-space Interface
    // ========================================================================
    input  wire [15:0] x_addr,
    input  wire        x_rd_en,
    input  wire        x_wr_en,
    input  wire [23:0] x_wdata,
    output wire [23:0] x_rdata,
    output wire        x_ready,

    // ========================================================================
    // Y-space Interface
    // ========================================================================
    input  wire [15:0] y_addr,
    input  wire        y_rd_en,
    input  wire        y_wr_en,
    input  wire [23:0] y_wdata,
    output wire [23:0] y_rdata,
    output wire        y_ready,

    // ========================================================================
    // External SRAM Interface (active for external addresses)
    // ========================================================================
    output wire [14:0] ext_addr,          // 32K address
    output wire [23:0] ext_wdata,
    output wire        ext_rd_en,
    output wire        ext_wr_en,
    output wire [1:0]  ext_space,         // 0=P, 1=X, 2=Y
    input  wire [23:0] ext_rdata,
    input  wire        ext_ready,

    // ========================================================================
    // X-Peripheral Interface ($FFC0-$FFFF)
    // ========================================================================
    output wire [5:0]  xperiph_addr,      // 6-bit peripheral address
    output wire [23:0] xperiph_wdata,
    output wire        xperiph_rd_en,
    output wire        xperiph_wr_en,
    input  wire [23:0] xperiph_rdata,

    // ========================================================================
    // Y-Peripheral Interface ($FFE0-$FFFF)
    // ========================================================================
    output wire [4:0]  yperiph_addr,      // 5-bit peripheral address
    output wire [23:0] yperiph_wdata,
    output wire        yperiph_rd_en,
    output wire        yperiph_wr_en,
    input  wire [23:0] yperiph_rdata,

    // ========================================================================
    // Y-Space RTC Interface ($FFC0-$FFDF)
    // ========================================================================
    output wire [4:0]  yrtc_addr,
    output wire [23:0] yrtc_wdata,
    output wire        yrtc_rd_en,
    output wire        yrtc_wr_en,
    input  wire [23:0] yrtc_rdata,

    // ========================================================================
    // Debug/Bootstrap Write Interface
    // ========================================================================
    input  wire        dbg_wr_en,
    input  wire [1:0]  dbg_space,         // 0=P, 1=X, 2=Y
    input  wire [15:0] dbg_addr,
    input  wire [23:0] dbg_wdata
);

    // ========================================================================
    // Internal Memory Storage
    // ========================================================================
    reg [23:0] p_ram [0:511];    // 512 x 24
    reg [23:0] x_ram [0:255];    // 256 x 24
    reg [23:0] x_rom [0:255];    // 256 x 24 (mu-law/A-law)
    reg [23:0] y_ram [0:255];    // 256 x 24
    reg [23:0] y_rom [0:255];    // 256 x 24 (sine table)

    // Motorola DSP56001/D Appendix C. The stored table contains the positive
    // Mu-law half followed by the positive A-law half, each expanded to the
    // DSP's 24-bit fractional data format.
    function [23:0] mulaw_rom;
        input [6:0] code;
        reg [6:0] decoded;
        integer segment;
        integer quantum;
        integer linear;
        begin
            decoded = ~code;
            segment = (decoded >> 4) & 7;
            quantum = decoded & 15;
            linear = (((quantum << 3) + 132) << segment) - 132;
            mulaw_rom = linear << 8;
        end
    endfunction

    function [23:0] alaw_rom;
        input [7:0] code;
        reg [7:0] decoded;
        integer segment;
        integer quantum;
        integer linear;
        begin
            decoded = code ^ 8'h55;
            segment = (decoded >> 4) & 7;
            quantum = decoded & 15;
            linear = quantum << 4;
            if (segment == 0)
                linear = linear + 8;
            else begin
                linear = linear + 264;
                if (segment > 1)
                    linear = linear << (segment - 1);
            end
            alaw_rom = linear << 8;
        end
    endfunction

    // Appendix D has exact, rounded 24-bit fractional values. Keep the first
    // quadrant literal and derive the other three only by table symmetry so
    // this is bit-identical to the factory ROM, including -1.0 at index C0.
    function [23:0] sine_quarter;
        input [6:0] index;
        begin
            case (index)
                7'h00: sine_quarter = 24'h000000;
                7'h01: sine_quarter = 24'h03242B;
                7'h02: sine_quarter = 24'h0647D9;
                7'h03: sine_quarter = 24'h096A90;
                7'h04: sine_quarter = 24'h0C8BD3;
                7'h05: sine_quarter = 24'h0FAB27;
                7'h06: sine_quarter = 24'h12C810;
                7'h07: sine_quarter = 24'h15E214;
                7'h08: sine_quarter = 24'h18F8B8;
                7'h09: sine_quarter = 24'h1C0B82;
                7'h0A: sine_quarter = 24'h1F19F9;
                7'h0B: sine_quarter = 24'h2223A5;
                7'h0C: sine_quarter = 24'h25280C;
                7'h0D: sine_quarter = 24'h2826B9;
                7'h0E: sine_quarter = 24'h2B1F35;
                7'h0F: sine_quarter = 24'h2E110A;
                7'h10: sine_quarter = 24'h30FBC5;
                7'h11: sine_quarter = 24'h33DEF3;
                7'h12: sine_quarter = 24'h36BA20;
                7'h13: sine_quarter = 24'h398CDD;
                7'h14: sine_quarter = 24'h3C56BA;
                7'h15: sine_quarter = 24'h3F174A;
                7'h16: sine_quarter = 24'h41CE1E;
                7'h17: sine_quarter = 24'h447ACD;
                7'h18: sine_quarter = 24'h471CED;
                7'h19: sine_quarter = 24'h49B415;
                7'h1A: sine_quarter = 24'h4C3FE0;
                7'h1B: sine_quarter = 24'h4EBFE9;
                7'h1C: sine_quarter = 24'h5133CD;
                7'h1D: sine_quarter = 24'h539B2B;
                7'h1E: sine_quarter = 24'h55F5A5;
                7'h1F: sine_quarter = 24'h5842DD;
                7'h20: sine_quarter = 24'h5A827A;
                7'h21: sine_quarter = 24'h5CB421;
                7'h22: sine_quarter = 24'h5ED77D;
                7'h23: sine_quarter = 24'h60EC38;
                7'h24: sine_quarter = 24'h62F202;
                7'h25: sine_quarter = 24'h64E889;
                7'h26: sine_quarter = 24'h66CF81;
                7'h27: sine_quarter = 24'h68A69F;
                7'h28: sine_quarter = 24'h6A6D99;
                7'h29: sine_quarter = 24'h6C2429;
                7'h2A: sine_quarter = 24'h6DCA0D;
                7'h2B: sine_quarter = 24'h6F5F03;
                7'h2C: sine_quarter = 24'h70E2CC;
                7'h2D: sine_quarter = 24'h72552D;
                7'h2E: sine_quarter = 24'h73B5EC;
                7'h2F: sine_quarter = 24'h7504D3;
                7'h30: sine_quarter = 24'h7641AF;
                7'h31: sine_quarter = 24'h776C4F;
                7'h32: sine_quarter = 24'h788484;
                7'h33: sine_quarter = 24'h798A24;
                7'h34: sine_quarter = 24'h7A7D05;
                7'h35: sine_quarter = 24'h7B5D04;
                7'h36: sine_quarter = 24'h7C29FC;
                7'h37: sine_quarter = 24'h7CE3CF;
                7'h38: sine_quarter = 24'h7D8A5F;
                7'h39: sine_quarter = 24'h7E1D94;
                7'h3A: sine_quarter = 24'h7E9D56;
                7'h3B: sine_quarter = 24'h7F0992;
                7'h3C: sine_quarter = 24'h7F6237;
                7'h3D: sine_quarter = 24'h7FA737;
                7'h3E: sine_quarter = 24'h7FD888;
                7'h3F: sine_quarter = 24'h7FF622;
                7'h40: sine_quarter = 24'h7FFFFF;
                default: sine_quarter = 24'h000000;
            endcase
        end
    endfunction

    function [23:0] sine_rom;
        input [7:0] index;
        begin
            if (index <= 8'h40)
                sine_rom = sine_quarter(index[6:0]);
            else if (index < 8'h80)
                sine_rom = sine_quarter(8'h80 - index);
            else if (index == 8'h80)
                sine_rom = 24'h000000;
            else if (index < 8'hC0)
                sine_rom = -sine_quarter(index - 8'h80);
            else if (index == 8'hC0)
                sine_rom = 24'h800000;
            else
                sine_rom = -sine_quarter(9'h100 - index);
        end
    endfunction

    integer i;
    initial begin
        for (i = 0; i < 512; i = i + 1) p_ram[i] = 24'h0;
        for (i = 0; i < 256; i = i + 1) x_ram[i] = 24'h0;
        for (i = 0; i < 128; i = i + 1) x_rom[i] = mulaw_rom(i[6:0]);
        for (i = 128; i < 256; i = i + 1) x_rom[i] = alaw_rom(i[7:0]);
        for (i = 0; i < 256; i = i + 1) y_ram[i] = 24'h0;
        for (i = 0; i < 256; i = i + 1) y_rom[i] = sine_rom(i[7:0]);
        // The Indigo audio board's reset/bootstrap path enters the program
        // image loaded at P:$E000. Model the resulting two-word
        // bootstrap explicitly so the synthesised core follows the same path
        // as the cosimulation and the real board.
        p_ram[0] = 24'h0AF080; // JMP absolute
        p_ram[1] = 24'h00E000;
    end

    // ========================================================================
    // Address Decode: P-space
    // ========================================================================
    wire p_internal = (p_addr < 16'h0200);        // $0000-$01FF
    wire p_external = !p_internal;

    // ========================================================================
    // Address Decode: X-space
    // ========================================================================
    wire x_int_ram   = (x_addr < 16'h0100);        // $0000-$00FF
    wire x_int_rom   = data_rom_enable &&
                       (x_addr >= 16'h0100) && (x_addr < 16'h0200);  // $0100-$01FF
    wire x_periph    = (x_addr >= 16'hFFC0);        // $FFC0-$FFFF
    wire x_external  = !x_int_ram && !x_int_rom && !x_periph;

    // ========================================================================
    // Address Decode: Y-space
    // ========================================================================
    wire y_int_ram   = (y_addr < 16'h0100);        // $0000-$00FF
    wire y_int_rom   = data_rom_enable &&
                       (y_addr >= 16'h0100) && (y_addr < 16'h0200);  // $0100-$01FF
    wire y_rtc       = (y_addr >= 16'hFFC0) && (y_addr < 16'hFFE0);
    wire y_periph    = (y_addr >= 16'hFFE0);        // $FFE0-$FFFF
    wire y_external  = !y_int_ram && !y_int_rom && !y_rtc && !y_periph;

    reg        deferred_p_fetch;
    reg [14:0] deferred_p_fetch_addr;
    reg        deferred_data_is_y;
    reg        deferred_p_response_ready;

    // ========================================================================
    // P-space Read/Write
    // ========================================================================
    reg [23:0] p_rdata_int;
    reg        p_was_external;

    always @(posedge clk) begin
        // Debug/bootstrap write
        if (dbg_wr_en && dbg_space == 2'd0 && dbg_addr < 16'h0200)
            p_ram[dbg_addr[8:0]] <= dbg_wdata;
        // Normal write
        else if (p_wr_en && p_internal)
            p_ram[p_addr[8:0]] <= p_wdata;

        // Internal read (registered)
        if (p_rd_en && p_internal)
            p_rdata_int <= p_ram[p_addr[8:0]];

        // Retain the source of the last completed response until another read
        // replaces it.  The 20 MHz core enable may leave idle 50 MHz wrapper
        // clocks between request and consumption; clearing this selector on
        // those idle clocks would substitute stale internal RAM data.
        // A denied external MOVEM must leave the preceding P-fetch response
        // selected.  Absolute P-data addressing consumes that extension word
        // again on every held core step; selecting external data before the
        // request is accepted replaces the retained address with stale SRAM
        // data and makes the request disappear.  Internal P reads are always
        // complete, while an external read changes the response source only
        // on its real accepting edge.
        if (p_rd_en &&
            (p_internal ||
             (p_external &&
              ((deferred_p_fetch && deferred_p_response_ready) ||
               (ext_ready && ext_space == 2'd0)))))
            p_was_external <= p_external;
    end

    // External reads: combinational from ext_rdata (already registered in SRAM)
    // Internal reads: from registered p_rdata_int
    assign p_rdata = p_was_external ? ext_rdata : p_rdata_int;

    // ========================================================================
    // X-space Read/Write
    // ========================================================================
    reg [23:0] x_rdata_int;
    reg        x_was_external;
    reg [1:0]  x_rd_src;  // 0=ram, 1=rom, 2=periph

    always @(posedge clk) begin
        // Debug write
        if (dbg_wr_en && dbg_space == 2'd1 && dbg_addr < 16'h0100)
            x_ram[dbg_addr[7:0]] <= dbg_wdata;
        // Normal write
        else if (x_wr_en && x_int_ram)
            x_ram[x_addr[7:0]] <= x_wdata;

        // Internal read (registered)
        if (x_rd_en) begin
            if (x_int_ram)
                x_rdata_int <= x_ram[x_addr[7:0]];
            else if (x_int_rom)
                x_rdata_int <= x_rom[x_addr[7:0]];
            else if (x_periph)
                x_rdata_int <= xperiph_rdata;
        end

        if (x_rd_en)
            x_was_external <= x_external;
    end

    assign x_rdata = x_was_external ? ext_rdata : x_rdata_int;

    // ========================================================================
    // Y-space Read/Write
    // ========================================================================
    reg [23:0] y_rdata_int;
    reg        y_was_external;

    always @(posedge clk) begin
        // Debug write
        if (dbg_wr_en && dbg_space == 2'd2 && dbg_addr < 16'h0100)
            y_ram[dbg_addr[7:0]] <= dbg_wdata;
        // Normal write
        else if (y_wr_en && y_int_ram)
            y_ram[y_addr[7:0]] <= y_wdata;

        // Internal read (registered)
        if (y_rd_en) begin
            if (y_int_ram)
                y_rdata_int <= y_ram[y_addr[7:0]];
            else if (y_int_rom)
                y_rdata_int <= y_rom[y_addr[7:0]];
            else if (y_rtc)
                y_rdata_int <= yrtc_rdata;
            else if (y_periph)
                y_rdata_int <= yperiph_rdata;
        end

        if (y_rd_en)
            y_was_external <= y_external;
    end

    assign y_rdata = y_was_external ? ext_rdata : y_rdata_int;

    // ========================================================================
    // External SRAM Address Mapping
    //
    // P-space and X-space share the same SRAM view. Y-space selects the
    // complementary A14 mapping used by the host interface.
    //   P:$E000 -> SRAM 0x2000, Y:$C000 -> SRAM 0x4000, Y:$8000 -> SRAM 0x0000
    //
    // Address formula:
    //   P/X-space: sram_addr = {addr[14] ^ 1, addr[13:0]}  (XOR bit 14)
    //   Y-space:   sram_addr = addr[14:0]                   (direct)
    // ========================================================================
    reg [14:0] ext_addr_r;
    reg [23:0] ext_wdata_r;
    reg        ext_rd_en_r;
    reg        ext_wr_en_r;
    reg [1:0]  ext_space_r;

    // A modifying external X/Y bit operation writes its read-modify-write
    // result in S_WB_FETCH, on the same architectural edge that requests the
    // next external P word.  The SRAM has one DSP-side port, but the 50 MHz
    // wrapper has idle clocks between 20 MHz core steps.  Commit the data
    // write first, then retain the P request until its own accepting edge.
    // BCR-programmed waits can extend either half across several core clocks;
    // each requester therefore receives ready only for its own response.
    wire external_x_write = x_external && x_wr_en;
    wire external_y_write = y_external && y_wr_en;
    wire p_fetch_write_collision = p_external && p_rd_en &&
                                   (external_x_write || external_y_write);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            deferred_p_fetch <= 1'b0;
            deferred_p_fetch_addr <= 15'h0;
            deferred_data_is_y <= 1'b0;
            deferred_p_response_ready <= 1'b0;
        end else begin
            if (deferred_p_fetch) begin
                if (!deferred_p_response_ready && ext_ready &&
                    ext_space_r == 2'd0) begin
                    if (p_rd_en) begin
                        deferred_p_fetch <= 1'b0;
                        deferred_p_response_ready <= 1'b0;
                    end else begin
                        // A zero-wait SRAM response can arrive on an idle
                        // 50 MHz wrapper edge.  Retain it until the 20 MHz
                        // core presents the held fetch on its accepting edge.
                        deferred_p_response_ready <= 1'b1;
                    end
                end else if (deferred_p_response_ready && p_rd_en) begin
                    deferred_p_fetch <= 1'b0;
                    deferred_p_response_ready <= 1'b0;
                end
            end else if (p_fetch_write_collision && ext_ready) begin
                // The selected X/Y write has completed.  Remember that
                // acknowledgement while its companion P fetch waits.
                deferred_p_fetch <= 1'b1;
                deferred_p_fetch_addr <=
                    {p_addr[14] ^ 1'b1, p_addr[13:0]};
                deferred_data_is_y <= external_y_write;
                deferred_p_response_ready <= 1'b0;
            end
        end
    end

    always @(*) begin
        ext_addr_r  = 15'h0;
        ext_wdata_r = 24'h0;
        ext_rd_en_r = 1'b0;
        ext_wr_en_r = 1'b0;
        ext_space_r = 2'd0;

        if (deferred_p_fetch) begin
            ext_addr_r = deferred_p_fetch_addr;
            ext_rd_en_r = !deferred_p_response_ready;
            ext_space_r = 2'd0;
        end else if (p_fetch_write_collision && external_x_write) begin
            ext_addr_r  = {x_addr[14] ^ 1'b1, x_addr[13:0]};
            ext_wdata_r = x_wdata;
            ext_wr_en_r = 1'b1;
            ext_space_r = 2'd1;
        end else if (p_fetch_write_collision && external_y_write) begin
            ext_addr_r  = y_addr[14:0];
            ext_wdata_r = y_wdata;
            ext_wr_en_r = 1'b1;
            ext_space_r = 2'd2;
        end else if (p_external && (p_rd_en || p_wr_en)) begin
            // P-space: XOR bit 14
            ext_addr_r  = {p_addr[14] ^ 1'b1, p_addr[13:0]};
            ext_wdata_r = p_wdata;
            ext_rd_en_r = p_rd_en;
            ext_wr_en_r = p_wr_en;
            ext_space_r = 2'd0;  // P
        end else if (x_external && (x_rd_en || x_wr_en)) begin
            // X-space: XOR bit 14 (same as P-space)
            ext_addr_r  = {x_addr[14] ^ 1'b1, x_addr[13:0]};
            ext_wdata_r = x_wdata;
            ext_rd_en_r = x_rd_en;
            ext_wr_en_r = x_wr_en;
            ext_space_r = 2'd1;  // X
        end else if (y_external && (y_rd_en || y_wr_en)) begin
            // Y-space: direct (no XOR)
            ext_addr_r  = y_addr[14:0];
            ext_wdata_r = y_wdata;
            ext_rd_en_r = y_rd_en;
            ext_wr_en_r = y_wr_en;
            ext_space_r = 2'd2;  // Y
        end
    end

    assign ext_addr  = ext_addr_r;
    assign ext_wdata = ext_wdata_r;
    assign ext_rd_en = ext_rd_en_r;
    assign ext_wr_en = ext_wr_en_r;
    assign ext_space = ext_space_r;

    // ========================================================================
    // Peripheral Interface Routing
    // ========================================================================
    assign xperiph_addr  = x_addr[5:0];
    assign xperiph_wdata = x_wdata;
    assign xperiph_rd_en = x_rd_en && x_periph;
    assign xperiph_wr_en = x_wr_en && x_periph;

    assign yperiph_addr  = y_addr[4:0];
    assign yperiph_wdata = y_wdata;
    assign yperiph_rd_en = y_rd_en && y_periph;
    assign yperiph_wr_en = y_wr_en && y_periph;

    assign yrtc_addr  = y_addr[4:0];
    assign yrtc_wdata = y_wdata;
    assign yrtc_rd_en = y_rd_en && y_rtc;
    assign yrtc_wr_en = y_wr_en && y_rtc;

    // Debug: removed Y-peripheral trace

    // ========================================================================
    // Ready Signals
    // ========================================================================
    // Internal memory: 1-cycle latency.  The external SRAM is shared, so an
    // acknowledgement belongs only to the selected space.  During a deferred
    // collision the X/Y write is already complete and remains acknowledged
    // while the retained P fetch receives its independently delayed response.
    wire deferred_x_ready = deferred_p_fetch && !deferred_data_is_y;
    wire deferred_y_ready = deferred_p_fetch && deferred_data_is_y;
    assign p_ready = p_internal ||
                     (p_external &&
                      ((deferred_p_fetch && deferred_p_response_ready) ||
                       (ext_ready && ext_space_r == 2'd0)));
    assign x_ready = (x_int_ram || x_int_rom || x_periph) ||
                     (x_external &&
                      ((ext_ready && ext_space_r == 2'd1) ||
                       deferred_x_ready));
    assign y_ready = (y_int_ram || y_int_rom || y_rtc || y_periph) ||
                     (y_external &&
                      ((ext_ready && ext_space_r == 2'd2) ||
                       deferred_y_ready));

endmodule
