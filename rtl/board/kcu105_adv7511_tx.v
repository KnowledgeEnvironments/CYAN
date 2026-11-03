// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - KCU105 ADV7511 bridge for the real LG1 scanout.
//
// The KCU105 routes a 16-bit, 1.8-V pixel bus to its ADV7511.  This follows
// the data convention used by Analog Devices' hdl_2017_r1 KCU105 project:
// separate syncs and SDR YCbCr 4:2:2, with CbY/CrY on alternate pixels.

`default_nettype none

module kcu105_adv7511_tx #(
    parameter integer CLK_HZ = 50000000,
    parameter integer I2C_HZ = 100000
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        pixel_clk,
    input  wire        pixel_rst_n,
    input  wire        video_de,
    input  wire        video_hsync,
    input  wire        video_vsync,
    input  wire [7:0]  video_r,
    input  wire [7:0]  video_g,
    input  wire [7:0]  video_b,

    output wire        hdmi_out_clk,
    output wire        hdmi_hsync,
    output wire        hdmi_vsync,
    output wire        hdmi_data_e,
    output wire [15:0] hdmi_data,

    input  wire        iic_scl_i,
    input  wire        iic_sda_i,
    output wire        iic_scl_drive_low,
    output wire        iic_sda_drive_low,
    output wire        iic_mux_reset_n,
    output wire [127:0] status
);
    // Center the ADV7511 rising sampling edge in the output data eye.  The
    // data/control registers launch on pixel_clk rising; the forwarded clock
    // must therefore rise on pixel_clk falling, not on that same launch edge.
    ODDRE1 #(.SRVAL(1'b0), .SIM_DEVICE("ULTRASCALE")) u_hdmi_clk_oddr (
        .SR(1'b0), .D1(1'b0), .D2(1'b1), .C(pixel_clk), .Q(hdmi_out_clk)
    );

    // BT.601 studio-range conversion, matching ADI's RGB2CrYCb coefficients.
    reg signed [18:0] y_sum;
    reg signed [18:0] cb_sum;
    reg signed [18:0] cr_sum;
    reg               de_d;
    reg               hs_d;
    reg               vs_d;
    reg               phase_d;
    reg               pixel_phase;

    function [7:0] clip8;
        input signed [18:0] value;
        reg signed [18:0] rounded;
        begin
            rounded = (value + 19'sd128) >>> 8;
            if (rounded < 0)
                clip8 = 8'h00;
            else if (rounded > 255)
                clip8 = 8'hff;
            else
                clip8 = rounded[7:0];
        end
    endfunction

    // Keep the complete SDR payload in the UltraScale output bit slices.
    // This final stage preserves the pre-edge values consumed by the former
    // hdmi_* nonblocking assignments, but removes fabric-to-pad skew from the
    // parallel ADV7511 bus.  The forwarded clock rises half a pixel later.
    localparam [18:0] ADV_BLANK = {1'b0, 1'b0, 1'b0, 16'h8010};
    wire [15:0] adv_data_next = de_d ?
        {(phase_d ? clip8(cr_sum) : clip8(cb_sum)), clip8(y_sum)} :
        16'h8010;
    wire [18:0] adv_next = {hs_d, vs_d, de_d, adv_data_next};
    (* IOB = "TRUE" *) reg [18:0] adv_q;

    always @(posedge pixel_clk) begin
        if (!pixel_rst_n)
            adv_q <= ADV_BLANK;
        else
            adv_q <= adv_next;
    end

    assign hdmi_hsync = adv_q[18];
    assign hdmi_vsync = adv_q[17];
    assign hdmi_data_e = adv_q[16];
    assign hdmi_data = adv_q[15:0];

    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            y_sum <= 19'sd0;
            cb_sum <= 19'sd0;
            cr_sum <= 19'sd0;
            de_d <= 1'b0;
            hs_d <= 1'b0;
            vs_d <= 1'b0;
            phase_d <= 1'b0;
            pixel_phase <= 1'b0;
        end else begin
            y_sum <= 19'sd4096 +
                     $signed({1'b0, video_r}) * 19'sd66 +
                     $signed({1'b0, video_g}) * 19'sd129 +
                     $signed({1'b0, video_b}) * 19'sd25;
            cb_sum <= 19'sd32768 -
                      $signed({1'b0, video_r}) * 19'sd38 -
                      $signed({1'b0, video_g}) * 19'sd74 +
                      $signed({1'b0, video_b}) * 19'sd112;
            cr_sum <= 19'sd32768 +
                      $signed({1'b0, video_r}) * 19'sd112 -
                      $signed({1'b0, video_g}) * 19'sd94 -
                      $signed({1'b0, video_b}) * 19'sd18;
            de_d <= video_de;
            hs_d <= video_hsync;
            vs_d <= video_vsync;
            if (!video_de) begin
                pixel_phase <= 1'b0;
                phase_d <= 1'b0;
            end else begin
                phase_d <= pixel_phase;
                pixel_phase <= ~pixel_phase;
            end
        end
    end

    // The KCU105 routes the ADV7511 through TCA9548 U28 channel 5.  U28's
    // active-low reset is an FPGA pin, so release it before the startup delay
    // and select channel 5 (address 0x74, value 0x20) before programming the
    // ADV7511 main map at address 0x39.
    assign iic_mux_reset_n = rst_n;

    // ADV7511 main-map setup at 7-bit address 0x39.  These values combine
    // ADI's required fixed-register patch with 8-bit 4:2:2, style 2,
    // separate sync, YCbCr-to-RGB CSC, HDMI mode and power-up.
    localparam integer ADV_CFG_COUNT = 58;
    localparam integer CFG_COUNT = ADV_CFG_COUNT + 1;
    function [15:0] cfg_word;
        input [5:0] index;
        begin
            case (index)
                6'd0:  cfg_word = 16'h9803;
                6'd1:  cfg_word = 16'h9ae0;
                6'd2:  cfg_word = 16'h9c30;
                6'd3:  cfg_word = 16'h9d61;
                6'd4:  cfg_word = 16'ha2a4;
                6'd5:  cfg_word = 16'ha3a4;
                6'd6:  cfg_word = 16'he0d0;
                6'd7:  cfg_word = 16'hf900;
                6'd8:  cfg_word = 16'h5502;
                6'd9:  cfg_word = 16'h1521;
                6'd10: cfg_word = 16'h1634;
                6'd11: cfg_word = 16'h4808;
                6'd12: cfg_word = 16'hd00c;
                6'd13: cfg_word = 16'hba60;
                6'd14: cfg_word = 16'h18c7;
                6'd15: cfg_word = 16'h1934;
                6'd16: cfg_word = 16'h1a04;
                6'd17: cfg_word = 16'h1bad;
                6'd18: cfg_word = 16'h1c00;
                6'd19: cfg_word = 16'h1d00;
                6'd20: cfg_word = 16'h1e1c;
                6'd21: cfg_word = 16'h1f1b;
                6'd22: cfg_word = 16'h201d;
                6'd23: cfg_word = 16'h21dc;
                6'd24: cfg_word = 16'h2204;
                6'd25: cfg_word = 16'h23ad;
                6'd26: cfg_word = 16'h241f;
                6'd27: cfg_word = 16'h2524;
                6'd28: cfg_word = 16'h2601;
                6'd29: cfg_word = 16'h2735;
                6'd30: cfg_word = 16'h2800;
                6'd31: cfg_word = 16'h2900;
                6'd32: cfg_word = 16'h2a04;
                6'd33: cfg_word = 16'h2bad;
                6'd34: cfg_word = 16'h2c08;
                6'd35: cfg_word = 16'h2d7c;
                6'd36: cfg_word = 16'h2e1b;
                6'd37: cfg_word = 16'h2f77;
                6'd38: cfg_word = 16'haf02;
                6'd39: cfg_word = 16'hd6c0;
                6'd40: cfg_word = 16'h4000;
                // 48 kHz stereo S/PDIF.  N=6144 and CTS=65000 match the
                // board's exact 65 MHz TMDS clock.  The remaining values
                // select and enable the S/PDIF receiver, describe 24-bit PCM,
                // publish a two-channel Audio InfoFrame, enable audio/N-CTS
                // packets, and release AV mute before the final power-up.
                6'd41: cfg_word = 16'h0100;
                6'd42: cfg_word = 16'h0218;
                6'd43: cfg_word = 16'h0300;
                6'd44: cfg_word = 16'h0700;
                6'd45: cfg_word = 16'h08fd;
                6'd46: cfg_word = 16'h09e8;
                6'd47: cfg_word = 16'h0a11;
                6'd48: cfg_word = 16'h0b8e;
                6'd49: cfg_word = 16'h1220;
                6'd50: cfg_word = 16'h140b;
                6'd51: cfg_word = 16'h4aa0;
                6'd52: cfg_word = 16'h7270;
                6'd53: cfg_word = 16'h7301;
                6'd54: cfg_word = 16'h4a80;
                6'd55: cfg_word = 16'h4479;
                6'd56: cfg_word = 16'h4b80;
                6'd57: cfg_word = 16'h4110;
                default: cfg_word = 16'h4110;
            endcase
        end
    endfunction

    localparam [8:0] HALF_DIV = CLK_HZ / (I2C_HZ * 2);
    localparam [20:0] STARTUP_COUNT = CLK_HZ / 25;
    localparam [3:0] I_WAIT       = 4'd0;
    localparam [3:0] I_START      = 4'd1;
    localparam [3:0] I_START_LOW  = 4'd2;
    localparam [3:0] I_BIT_LOW    = 4'd3;
    localparam [3:0] I_BIT_HIGH   = 4'd4;
    localparam [3:0] I_ACK_LOW    = 4'd5;
    localparam [3:0] I_ACK_HIGH   = 4'd6;
    localparam [3:0] I_STOP_LOW   = 4'd7;
    localparam [3:0] I_STOP_HIGH  = 4'd8;
    localparam [3:0] I_STOP_FREE  = 4'd9;
    localparam [3:0] I_DONE       = 4'd10;
    localparam [3:0] I_READ_LOW   = 4'd11;
    localparam [3:0] I_READ_HIGH  = 4'd12;
    localparam [3:0] I_NACK_LOW   = 4'd13;
    localparam [3:0] I_NACK_HIGH  = 4'd14;
    localparam [3:0] I_RESTART_LOW = 4'd15;

    reg [3:0] i_state;
    reg [8:0] i_div;
    reg [20:0] startup_wait;
    reg [5:0] cfg_index;
    reg [1:0] byte_index;
    reg [2:0] bit_index;
    reg [7:0] tx_byte;
    reg scl_low;
    reg sda_low;
    reg nack_seen;
    reg [7:0] nack_count;
    reg cfg_done;
    reg read_phase;
    reg [2:0] read_index;
    reg [7:0] read_byte;
    reg [7:0] read_reg_42;
    reg [7:0] read_reg_9e;
    reg [7:0] read_reg_41;
    reg [7:0] read_reg_15;
    reg [7:0] read_reg_16;
    reg [7:0] read_reg_48;
    reg readback_valid;
    reg [15:0] readback_generation;
    wire mux_select = !cfg_done && (cfg_index == 6'd0);
    wire [15:0] current_cfg_word = cfg_word(cfg_index - 6'd1);

    function [7:0] readback_register;
        input [2:0] index;
        begin
            case (index)
                3'd0: readback_register = 8'h42; // HPD and monitor sense
                3'd1: readback_register = 8'h9e; // PLL lock
                3'd2: readback_register = 8'h41; // power state
                3'd3: readback_register = 8'h15; // input ID
                3'd4: readback_register = 8'h16; // input format/style
                default: readback_register = 8'h48; // data alignment
            endcase
        end
    endfunction

    wire i_tick = (i_div == HALF_DIV - 1);
    assign iic_scl_drive_low = scl_low;
    assign iic_sda_drive_low = sda_low;
    wire [31:0] legacy_status = {8'h48, i_state, cfg_index, 2'b00,
                                cfg_done, nack_seen,
                                iic_scl_i, iic_sda_i, nack_count};
    assign status = {
        legacy_status,
        readback_valid,
        read_reg_42[6], read_reg_42[5], read_reg_9e[4], read_reg_41[6],
        read_reg_42, read_reg_9e, read_reg_41,
        read_reg_15, read_reg_16, read_reg_48,
        readback_generation,
        27'd0
    };

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_state <= I_WAIT;
            i_div <= 9'd0;
            startup_wait <= 21'd0;
            cfg_index <= 6'd0;
            byte_index <= 2'd0;
            bit_index <= 3'd7;
            tx_byte <= 8'he8;
            scl_low <= 1'b0;
            sda_low <= 1'b0;
            nack_seen <= 1'b0;
            nack_count <= 8'd0;
            cfg_done <= 1'b0;
            read_phase <= 1'b0;
            read_index <= 3'd0;
            read_byte <= 8'd0;
            read_reg_42 <= 8'd0;
            read_reg_9e <= 8'd0;
            read_reg_41 <= 8'd0;
            read_reg_15 <= 8'd0;
            read_reg_16 <= 8'd0;
            read_reg_48 <= 8'd0;
            readback_valid <= 1'b0;
            readback_generation <= 16'd0;
        end else begin
            if (i_state == I_WAIT && startup_wait != STARTUP_COUNT)
                startup_wait <= startup_wait + 21'd1;
            if (i_tick)
                i_div <= 9'd0;
            else
                i_div <= i_div + 9'd1;

            if (i_tick) begin
                case (i_state)
                    I_WAIT: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                        if (startup_wait == STARTUP_COUNT)
                            i_state <= I_START;
                    end
                    I_START: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b1;
                        i_state <= I_START_LOW;
                    end
                    I_START_LOW: begin
                        scl_low <= 1'b1;
                        sda_low <= 1'b1;
                        byte_index <= 2'd0;
                        bit_index <= 3'd7;
                        tx_byte <= mux_select ? 8'he8 :
                                   (cfg_done && read_phase) ? 8'h73 : 8'h72;
                        i_state <= I_BIT_LOW;
                    end
                    I_BIT_LOW: begin
                        scl_low <= 1'b1;
                        sda_low <= ~tx_byte[bit_index];
                        i_state <= I_BIT_HIGH;
                    end
                    I_BIT_HIGH: begin
                        scl_low <= 1'b0;
                        if (bit_index == 3'd0)
                            i_state <= I_ACK_LOW;
                        else begin
                            bit_index <= bit_index - 3'd1;
                            i_state <= I_BIT_LOW;
                        end
                    end
                    I_ACK_LOW: begin
                        scl_low <= 1'b1;
                        sda_low <= 1'b0;
                        i_state <= I_ACK_HIGH;
                    end
                    I_ACK_HIGH: begin
                        scl_low <= 1'b0;
                        if (iic_sda_i) begin
                            nack_seen <= 1'b1;
                            nack_count <= nack_count + 8'd1;
                        end
                        if (cfg_done && read_phase) begin
                            bit_index <= 3'd7;
                            read_byte <= 8'd0;
                            i_state <= I_READ_LOW;
                        end else if (byte_index ==
                                     (mux_select ? 2'd1 :
                                      cfg_done ? 2'd1 : 2'd2))
                            i_state <= cfg_done ? I_RESTART_LOW : I_STOP_LOW;
                        else begin
                            byte_index <= byte_index + 2'd1;
                            bit_index <= 3'd7;
                            if (mux_select)
                                tx_byte <= 8'h20;
                            else if (cfg_done)
                                tx_byte <= readback_register(read_index);
                            else
                                tx_byte <= (byte_index == 2'd0) ?
                                    current_cfg_word[15:8] : current_cfg_word[7:0];
                            i_state <= I_BIT_LOW;
                        end
                    end
                    I_READ_LOW: begin
                        scl_low <= 1'b1;
                        sda_low <= 1'b0;
                        i_state <= I_READ_HIGH;
                    end
                    I_READ_HIGH: begin
                        scl_low <= 1'b0;
                        read_byte[bit_index] <= iic_sda_i;
                        if (bit_index == 3'd0) begin
                            case (read_index)
                                3'd0: read_reg_42 <= {read_byte[7:1], iic_sda_i};
                                3'd1: read_reg_9e <= {read_byte[7:1], iic_sda_i};
                                3'd2: read_reg_41 <= {read_byte[7:1], iic_sda_i};
                                3'd3: read_reg_15 <= {read_byte[7:1], iic_sda_i};
                                3'd4: read_reg_16 <= {read_byte[7:1], iic_sda_i};
                                default: read_reg_48 <= {read_byte[7:1], iic_sda_i};
                            endcase
                            i_state <= I_NACK_LOW;
                        end else begin
                            bit_index <= bit_index - 3'd1;
                            i_state <= I_READ_LOW;
                        end
                    end
                    I_NACK_LOW: begin
                        scl_low <= 1'b1;
                        sda_low <= 1'b0;
                        i_state <= I_NACK_HIGH;
                    end
                    I_NACK_HIGH: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                        i_state <= I_STOP_LOW;
                    end
                    I_RESTART_LOW: begin
                        // ADV7511 STOP resets the selected register address.
                        // Release SDA during a full SCL-low interval; FREE
                        // then raises SCL before START lowers SDA again.
                        // This preserves the pointer without generating STOP.
                        scl_low <= 1'b1;
                        sda_low <= 1'b0;
                        i_state <= I_STOP_FREE;
                    end
                    I_STOP_LOW: begin
                        scl_low <= 1'b1;
                        sda_low <= 1'b1;
                        i_state <= I_STOP_HIGH;
                    end
                    I_STOP_HIGH: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b1;
                        i_state <= I_STOP_FREE;
                    end
                    I_STOP_FREE: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                        if (!cfg_done && cfg_index == CFG_COUNT - 1) begin
                            cfg_done <= 1'b1;
                            i_state <= I_DONE;
                        end else if (!cfg_done) begin
                            cfg_index <= cfg_index + 6'd1;
                            i_state <= I_START;
                        end else if (!read_phase) begin
                            read_phase <= 1'b1;
                            i_state <= I_START;
                        end else begin
                            read_phase <= 1'b0;
                            if (read_index == 3'd5) begin
                                read_index <= 3'd0;
                                readback_valid <= 1'b1;
                                readback_generation <= readback_generation + 16'd1;
                            end else begin
                                read_index <= read_index + 3'd1;
                            end
                            i_state <= I_START;
                        end
                    end
                    I_DONE: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                        read_phase <= 1'b0;
                        read_index <= 3'd0;
                        i_state <= I_START;
                    end
                    default: begin
                        scl_low <= 1'b0;
                        sda_low <= 1'b0;
                        i_state <= I_DONE;
                    end
                endcase
            end
        end
    end
endmodule

`default_nettype wire
