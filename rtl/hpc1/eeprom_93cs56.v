// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - 93CS56 Serial EEPROM
// 2Kbit serial EEPROM with Microwire interface
//
// The 93CS56 supports both x8 (256x8) and x16 (128x16) modes via ORG pin.
// IP20 PROM uses x8 addressing (8-bit address, 11-bit command) but reads
// 16 bits sequentially. We emulate this by accepting 8-bit addresses and
// returning 16-bit words, treating addr as word address (not byte address).
//
// Pin interface:
//   CS  - Chip select (active high)
//   SK  - Serial clock
//   DI  - Data in (from CPU)
//   DO  - Data out (to CPU)
//
// Commands (IP20 uses 8-bit address format = 11 bits total):
//   READ  : 10 + A7-A0 + (data out 16 bits)
//   WRITE : 01 + A7-A0 + D15-D0
//   EWEN  : 00 11 XXXXXX (enable write)
//   EWDS  : 00 00 XXXXXX (disable write)
//   ERASE : 11 + A7-A0
//   ERAL  : 00 10 XXXXXX (erase all)
//   WRAL  : 00 01 XXXXXX + D15-D0 (write all)
//
// Firmware-visible NVRAM convention:
//   Word 0 high byte: checksum
//   Word 0 low byte: revision (6)
//   Word 1: console ('g' or 'd')

`default_nettype none

module eeprom_93cs56 (
    input  wire        clk,
    input  wire        rst_n,

    // Microwire interface
    input  wire        cs,
    input  wire        sk,
    input  wire        di,
    output reg         do_out,

    // Memory initialization (directly wire to memory array)
    // Used by testbench to pre-load NVRAM contents
    input  wire        init_we,
    input  wire [6:0]  init_addr,
    input  wire [15:0] init_data
);

    // =========================================================================
    // Memory array - 128 x 16-bit words (x16 mode)
    // =========================================================================

    reg [15:0] mem [0:127];

    // =========================================================================
    // State machine
    // =========================================================================

    localparam S_IDLE      = 4'd0;
    localparam S_START_BIT = 4'd1;
    localparam S_OPCODE    = 4'd2;
    localparam S_ADDRESS   = 4'd3;
    localparam S_READ_DATA = 4'd4;
    localparam S_WRITE_DATA= 4'd5;
    localparam S_WRITE_BUSY= 4'd6;

    reg [3:0] state;
    reg [4:0] bit_count;
    reg [1:0] opcode;
    reg [7:0] address;      // 8-bit address (x8 mode command format)
    reg [15:0] shift_reg;   // 16-bit data read/write
    reg write_enabled;

    // Clock edge detection
    reg sk_prev;
    wire sk_rising  = sk && !sk_prev;
    wire sk_falling = !sk && sk_prev;

    // CS edge detection
    reg cs_prev;
    wire cs_rising = cs && !cs_prev;

    // =========================================================================
    // Memory write port — single always block so Quartus infers BRAM (M9K)
    // instead of flops. fsm_we/fsm_waddr/fsm_wdata are set by the main FSM
    // below when a 16-bit WRITE command completes; init_we is from the
    // board-level boot loader.
    // =========================================================================

    reg fsm_we;
    reg [6:0] fsm_waddr;
    reg [15:0] fsm_wdata;

    always @(posedge clk) begin
        if (init_we)
            mem[init_addr] <= init_data;
        else if (fsm_we)
            mem[fsm_waddr] <= fsm_wdata;
    end

    // =========================================================================
    // Main state machine
    // =========================================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            bit_count <= 5'd0;
            opcode <= 2'b0;
            address <= 8'b0;
            shift_reg <= 16'b0;
            do_out <= 1'b1;  // High-Z state (pulled high)
            write_enabled <= 1'b0;
            sk_prev <= 1'b0;
            cs_prev <= 1'b0;
            fsm_we <= 1'b0;
            fsm_waddr <= 7'd0;
            fsm_wdata <= 16'd0;
        end else begin
            // Default: no FSM-driven mem write this cycle. Set by S_WRITE_DATA.
            fsm_we <= 1'b0;
            sk_prev <= sk;
            cs_prev <= cs;

            // CS going low resets state machine
            if (!cs) begin
                state <= S_IDLE;
                do_out <= 1'b1;
            end
            // Handle S_WRITE_BUSY independently - set DO high after 1 cycle
            // (PROM polls DO to detect write completion, may not toggle SK)
            else if (state == S_WRITE_BUSY) begin
                do_out <= 1'b1;  // Write complete, ready for next command
            end
            else if (cs_rising) begin
                // CS rising edge - start new command
                state <= S_START_BIT;
                bit_count <= 5'd0;
            end
            else if (sk_rising && cs) begin
                // Clock in data on rising edge of SK
                case (state)
                    S_IDLE: begin
                        // Allow starting a new command from IDLE without CS toggle
                        // This is needed because after commands complete, CS may stay high
                        // and PROM just clocks in a new start bit
                        if (di) begin
                            state <= S_OPCODE;
                            bit_count <= 5'd0;
                        end
                    end

                    S_START_BIT: begin
                        if (di) begin
                            state <= S_OPCODE;
                            bit_count <= 5'd0;
                        end
                    end

                    S_OPCODE: begin
                        opcode <= {opcode[0], di};
                        bit_count <= bit_count + 1'b1;
                        if (bit_count == 5'd1) begin
                            state <= S_ADDRESS;
                            bit_count <= 5'd0;
                        end
                    end

                    S_ADDRESS: begin
                        address <= {address[6:0], di};
                        bit_count <= bit_count + 1'b1;
                        if (bit_count == 5'd7) begin
                            // Have full 8-bit address (x8 command format)
                            // Treat address as word address (addr[6:0] selects word)
                            // Ignore addr[7] as it would address beyond 128 words
                            case (opcode)
                                2'b10: begin
                                    // READ command
                                    state <= S_READ_DATA;
                                    shift_reg <= mem[{address[5:0], di}];
                                    bit_count <= 5'd0;
                                    do_out <= 1'b0;  // Dummy 0 bit
                                    // $display("EEPROM: READ addr=%02x data=%04x",
                                    //     {address[6:0], di}, mem[{address[5:0], di}]);
                                end
                                2'b01: begin
                                    // WRITE command
                                    if (write_enabled) begin
                                        state <= S_WRITE_DATA;
                                        bit_count <= 5'd0;
                                        shift_reg <= 16'd0;  // Clear shift reg for new data
                                        // $display("EEPROM: WRITE cmd to addr=%02x",
                                        //     {address[6:0], di});
                                    end else begin
                                        state <= S_IDLE;
                                        // $display("EEPROM: WRITE rejected (not enabled)");
                                    end
                                end
                                2'b00: begin
                                    // Special commands based on address bits [7:6]
                                    // EWDS = 00xxxxxx, EWEN = 11xxxxxx
                                    if (address[6:5] == 2'b11) begin
                                        // EWEN - enable write
                                        write_enabled <= 1'b1;
                                        // $display("EEPROM: EWEN (write enabled)");
                                    end
                                    else if (address[6:5] == 2'b00) begin
                                        // EWDS - disable write
                                        write_enabled <= 1'b0;
                                        // $display("EEPROM: EWDS (write disabled)");
                                    end
                                    state <= S_IDLE;
                                end
                                default: state <= S_IDLE;
                            endcase
                        end
                    end

                    S_READ_DATA: begin
                        // Shift out data MSB first (16 bits)
                        // Each rising edge: output current MSB, then shift
                        do_out <= shift_reg[15];
                        shift_reg <= {shift_reg[14:0], 1'b0};
                        bit_count <= bit_count + 1'b1;
                        if (bit_count == 5'd15) begin
                            // All 16 bits sent, auto-increment if CS still high
                            address <= address + 1'b1;
                            shift_reg <= mem[address + 1'b1];
                            bit_count <= 5'd0;
                        end
                    end

                    S_WRITE_DATA: begin
                        shift_reg <= {shift_reg[14:0], di};
                        bit_count <= bit_count + 1'b1;
                        if (bit_count == 5'd15) begin
                            // All 16 bits received (x16 mode). Routed
                            // through fsm_we/waddr/wdata so a single always
                            // block drives mem (so Quartus infers BRAM).
                            fsm_we <= 1'b1;
                            fsm_waddr <= address[6:0];
                            fsm_wdata <= {shift_reg[14:0], di};
                            state <= S_WRITE_BUSY;
                            do_out <= 1'b0;  // Busy indicator
                        end
                    end

                    S_WRITE_BUSY: begin
                        // Write complete immediately (no real write delay in sim)
                        do_out <= 1'b1;  // Ready
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

    // =========================================================================
    // Initialize memory with default NVRAM values at reset
    // Optional file format: one 16-bit word per line in hexadecimal.
    // =========================================================================

`ifndef SYNTHESIS
    reg [1023:0] eeprom_filename;
    integer eeprom_init_idx;

    initial begin
        for (eeprom_init_idx = 0; eeprom_init_idx < 128;
             eeprom_init_idx = eeprom_init_idx + 1)
            mem[eeprom_init_idx] = 16'hffff;
        eeprom_filename = 0;
        if ($value$plusargs("eeprom_file=%s", eeprom_filename)) begin
            $readmemh(eeprom_filename, mem);
            $display("EEPROM: loaded explicit image %0s", eeprom_filename);
        end
    end
`endif
    // Hardware builds initialize the array through the board-level loader.

endmodule

`default_nettype wire
