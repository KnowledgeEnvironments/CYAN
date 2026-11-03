// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - SGI keyboard/mouse device behavior and managed injection.
//
// Host input is queued and delivered through the real HPC1/Z8530 receive
// injection ports at the equivalent serial-wire pace.  Device-directed SCC
// bytes are captured and CONFIG_REQUEST is answered like the original SGI
// keyboard and mouse; this is a device model, not a PROM diagnostic bypass.

`default_nettype none

module cyan_sgi_input_device #(
    parameter integer CLK_HZ = 50000000,
    parameter integer FIFO_DEPTH = 64,
    parameter integer RESPONSE_DELAY_CYCLES = CLK_HZ / 1000,
    parameter integer KBD_BYTE_CYCLES = (CLK_HZ * 11) / 600,
    parameter integer MOUSE_BYTE_CYCLES = (CLK_HZ * 10) / 4800
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       guest_active,

    input  wire [7:0] host_kbd_data,
    input  wire       host_kbd_valid,
    output wire       host_kbd_ready,
    input  wire [7:0] host_mouse_data,
    input  wire       host_mouse_valid,
    output wire       host_mouse_ready,

    input  wire [7:0] kbd_tx_data,
    input  wire       kbd_tx_valid,
    input  wire [7:0] mouse_tx_data,
    input  wire       mouse_tx_valid,

    output reg  [7:0] kbd_rx_data,
    output reg        kbd_rx_valid,
    output reg  [7:0] mouse_rx_data,
    output reg        mouse_rx_valid,

    output wire [7:0] kbd_tx_event_data,
    output wire       kbd_tx_event_valid,
    input  wire       kbd_tx_event_ready,
    output wire [7:0] mouse_tx_event_data,
    output wire       mouse_tx_event_valid,
    input  wire       mouse_tx_event_ready,
    output wire [7:0] kbd_rx_event_data,
    output wire       kbd_rx_event_valid,
    input  wire       kbd_rx_event_ready,
    output wire [7:0] mouse_rx_event_data,
    output wire       mouse_rx_event_valid,
    input  wire       mouse_rx_event_ready,

    output wire [15:0] host_kbd_queued,
    output wire [15:0] host_mouse_queued,
    output reg  [31:0] kbd_tx_event_overflow,
    output reg  [31:0] mouse_tx_event_overflow
);
    localparam [2:0]
        KRESP_IDLE     = 3'd0,
        KRESP_WAIT_ID  = 3'd1,
        KRESP_WAIT_DIP = 3'd2;
    localparam [1:0]
        MRESP_IDLE     = 2'd0,
        MRESP_WAIT_ID  = 2'd1;

    wire [7:0] host_kbd_head;
    wire       host_kbd_head_valid;
    reg        host_kbd_pop;
    wire [7:0] host_mouse_head;
    wire       host_mouse_head_valid;
    reg        host_mouse_pop;
    wire [15:0] host_kbd_count;
    wire [15:0] host_mouse_count;

    wire kbd_event_push_ready;
    wire mouse_event_push_ready;
    wire kbd_rx_event_push_ready;
    wire mouse_rx_event_push_ready;

    reg [2:0] kbd_response_state;
    reg [1:0] mouse_response_state;
    reg [31:0] kbd_response_timer;
    reg [31:0] mouse_response_timer;
    reg [31:0] kbd_host_gap;
    reg [31:0] mouse_host_gap;

    assign host_kbd_queued = host_kbd_count;
    assign host_mouse_queued = host_mouse_count;

    cyan_byte_fifo #(.DEPTH(FIFO_DEPTH)) u_host_kbd_fifo (
        .clk(clk), .rst_n(rst_n), .clear(1'b0),
        .push_data(host_kbd_data), .push_valid(host_kbd_valid),
        .push_ready(host_kbd_ready),
        .pop_data(host_kbd_head), .pop_valid(host_kbd_head_valid),
        .pop_ready(host_kbd_pop), .count(host_kbd_count)
    );

    cyan_byte_fifo #(.DEPTH(FIFO_DEPTH)) u_host_mouse_fifo (
        .clk(clk), .rst_n(rst_n), .clear(1'b0),
        .push_data(host_mouse_data), .push_valid(host_mouse_valid),
        .push_ready(host_mouse_ready),
        .pop_data(host_mouse_head), .pop_valid(host_mouse_head_valid),
        .pop_ready(host_mouse_pop), .count(host_mouse_count)
    );

    cyan_byte_fifo #(.DEPTH(FIFO_DEPTH)) u_kbd_tx_event_fifo (
        .clk(clk), .rst_n(rst_n), .clear(1'b0),
        .push_data(kbd_tx_data), .push_valid(kbd_tx_valid),
        .push_ready(kbd_event_push_ready),
        .pop_data(kbd_tx_event_data), .pop_valid(kbd_tx_event_valid),
        .pop_ready(kbd_tx_event_ready), .count()
    );

    cyan_byte_fifo #(.DEPTH(FIFO_DEPTH)) u_mouse_tx_event_fifo (
        .clk(clk), .rst_n(rst_n), .clear(1'b0),
        .push_data(mouse_tx_data), .push_valid(mouse_tx_valid),
        .push_ready(mouse_event_push_ready),
        .pop_data(mouse_tx_event_data), .pop_valid(mouse_tx_event_valid),
        .pop_ready(mouse_tx_event_ready), .count()
    );

    cyan_byte_fifo #(.DEPTH(FIFO_DEPTH)) u_kbd_rx_event_fifo (
        .clk(clk), .rst_n(rst_n), .clear(1'b0),
        .push_data(kbd_rx_data), .push_valid(kbd_rx_valid),
        .push_ready(kbd_rx_event_push_ready),
        .pop_data(kbd_rx_event_data), .pop_valid(kbd_rx_event_valid),
        .pop_ready(kbd_rx_event_ready), .count()
    );

    cyan_byte_fifo #(.DEPTH(FIFO_DEPTH)) u_mouse_rx_event_fifo (
        .clk(clk), .rst_n(rst_n), .clear(1'b0),
        .push_data(mouse_rx_data), .push_valid(mouse_rx_valid),
        .push_ready(mouse_rx_event_push_ready),
        .pop_data(mouse_rx_event_data), .pop_valid(mouse_rx_event_valid),
        .pop_ready(mouse_rx_event_ready), .count()
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kbd_rx_data             <= 8'd0;
            kbd_rx_valid            <= 1'b0;
            mouse_rx_data           <= 8'd0;
            mouse_rx_valid          <= 1'b0;
            host_kbd_pop            <= 1'b0;
            host_mouse_pop          <= 1'b0;
            kbd_response_state      <= KRESP_IDLE;
            mouse_response_state    <= MRESP_IDLE;
            kbd_response_timer      <= 32'd0;
            mouse_response_timer    <= 32'd0;
            kbd_host_gap            <= 32'd0;
            mouse_host_gap          <= 32'd0;
            kbd_tx_event_overflow   <= 32'd0;
            mouse_tx_event_overflow <= 32'd0;
        end else begin
            kbd_rx_valid   <= 1'b0;
            mouse_rx_valid <= 1'b0;
            host_kbd_pop   <= 1'b0;
            host_mouse_pop <= 1'b0;

            if (kbd_tx_valid && !kbd_event_push_ready)
                kbd_tx_event_overflow <= kbd_tx_event_overflow + 1'b1;
            if (mouse_tx_valid && !mouse_event_push_ready)
                mouse_tx_event_overflow <= mouse_tx_event_overflow + 1'b1;

            if (!guest_active) begin
                // The always-on host queues survive a SoC reset, but a partial
                // device reply does not.  PROM will issue a fresh probe after
                // reset and must receive a reply that starts at byte one.
                kbd_response_state   <= KRESP_IDLE;
                mouse_response_state <= MRESP_IDLE;
                kbd_response_timer   <= 32'd0;
                mouse_response_timer <= 32'd0;
                kbd_host_gap         <= 32'd0;
                mouse_host_gap       <= 32'd0;
            end else begin
                if (kbd_host_gap != 0)
                    kbd_host_gap <= kbd_host_gap - 1'b1;
                if (mouse_host_gap != 0)
                    mouse_host_gap <= mouse_host_gap - 1'b1;

                // A fresh CONFIG_REQUEST takes priority over queued operator input.
                if (kbd_tx_valid && kbd_tx_data == 8'h10 &&
                    kbd_response_state == KRESP_IDLE) begin
                    kbd_response_state <= KRESP_WAIT_ID;
                    // tx_valid reports the CPU loading the SCC transmitter,
                    // not a byte already received by the external keyboard.
                    // A real device cannot answer until the complete 600-baud
                    // command has crossed the wire, plus its turnaround time.
                    kbd_response_timer <= KBD_BYTE_CYCLES +
                                          RESPONSE_DELAY_CYCLES - 1;
                end else begin
                case (kbd_response_state)
                    KRESP_WAIT_ID: begin
                        if (kbd_response_timer == 0) begin
                            kbd_rx_data <= 8'h6e;
                            kbd_rx_valid <= 1'b1;
                            kbd_response_state <= KRESP_WAIT_DIP;
                            kbd_response_timer <= KBD_BYTE_CYCLES - 1;
                        end else begin
                            kbd_response_timer <= kbd_response_timer - 1'b1;
                        end
                    end
                    KRESP_WAIT_DIP: begin
                        if (kbd_response_timer == 0) begin
                            kbd_rx_data <= 8'h00;
                            kbd_rx_valid <= 1'b1;
                            kbd_response_state <= KRESP_IDLE;
                            kbd_host_gap <= KBD_BYTE_CYCLES - 1;
                        end else begin
                            kbd_response_timer <= kbd_response_timer - 1'b1;
                        end
                    end
                    default: begin
                        if (host_kbd_head_valid && kbd_host_gap == 0) begin
                            kbd_rx_data <= host_kbd_head;
                            kbd_rx_valid <= 1'b1;
                            host_kbd_pop <= 1'b1;
                            kbd_host_gap <= KBD_BYTE_CYCLES - 1;
                        end
                    end
                endcase
                end

                if (mouse_tx_valid && mouse_tx_data == 8'h10 &&
                    mouse_response_state == MRESP_IDLE) begin
                    mouse_response_state <= MRESP_WAIT_ID;
                    mouse_response_timer <= MOUSE_BYTE_CYCLES +
                                            RESPONSE_DELAY_CYCLES - 1;
                end else begin
                case (mouse_response_state)
                    MRESP_WAIT_ID: begin
                        if (mouse_response_timer == 0) begin
                            mouse_rx_data <= 8'h20;
                            mouse_rx_valid <= 1'b1;
                            mouse_response_state <= MRESP_IDLE;
                            mouse_host_gap <= MOUSE_BYTE_CYCLES - 1;
                        end else begin
                            mouse_response_timer <= mouse_response_timer - 1'b1;
                        end
                    end
                    default: begin
                        if (host_mouse_head_valid && mouse_host_gap == 0) begin
                            mouse_rx_data <= host_mouse_head;
                            mouse_rx_valid <= 1'b1;
                            host_mouse_pop <= 1'b1;
                            mouse_host_gap <= MOUSE_BYTE_CYCLES - 1;
                        end
                    end
                endcase
                end
            end
        end
    end
endmodule

module cyan_byte_fifo #(
    parameter integer DEPTH = 64,
    parameter integer WIDTH = 8
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clear,
    input  wire [WIDTH-1:0] push_data,
    input  wire        push_valid,
    output wire        push_ready,
    output wire [WIDTH-1:0] pop_data,
    output wire        pop_valid,
    input  wire        pop_ready,
    output wire [15:0] count
);
    localparam integer ADDR_W = (DEPTH <= 2) ? 1 : $clog2(DEPTH);
    localparam integer COUNT_W = $clog2(DEPTH + 1);

    reg [WIDTH-1:0] storage [0:DEPTH-1];
    reg [ADDR_W-1:0] write_pointer;
    reg [ADDR_W-1:0] read_pointer;
    reg [COUNT_W-1:0] item_count;

    wire push_fire = push_valid && push_ready;
    wire pop_fire = pop_valid && pop_ready;
    wire [31:0] item_count_ext = {{(32-COUNT_W){1'b0}}, item_count};
    wire [31:0] write_pointer_ext = {{(32-ADDR_W){1'b0}}, write_pointer};
    wire [31:0] read_pointer_ext = {{(32-ADDR_W){1'b0}}, read_pointer};

    assign push_ready = (item_count_ext != DEPTH);
    assign pop_valid = (item_count != 0);
    assign pop_data = storage[read_pointer];
    assign count = {{(16-COUNT_W){1'b0}}, item_count};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            write_pointer <= {ADDR_W{1'b0}};
            read_pointer <= {ADDR_W{1'b0}};
            item_count <= {COUNT_W{1'b0}};
        end else if (clear) begin
            write_pointer <= {ADDR_W{1'b0}};
            read_pointer <= {ADDR_W{1'b0}};
            item_count <= {COUNT_W{1'b0}};
        end else begin
            if (push_fire) begin
                storage[write_pointer] <= push_data;
                if (write_pointer_ext == DEPTH - 1)
                    write_pointer <= {ADDR_W{1'b0}};
                else
                    write_pointer <= write_pointer + 1'b1;
            end
            if (pop_fire) begin
                if (read_pointer_ext == DEPTH - 1)
                    read_pointer <= {ADDR_W{1'b0}};
                else
                    read_pointer <= read_pointer + 1'b1;
            end
            case ({push_fire, pop_fire})
                2'b10: item_count <= item_count + 1'b1;
                2'b01: item_count <= item_count - 1'b1;
                default: item_count <= item_count;
            endcase
        end
    end
endmodule

`default_nettype wire
