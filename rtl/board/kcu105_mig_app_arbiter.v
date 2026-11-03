// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - conservative three-client KCU105 DDR4 MIG app arbiter
//
// Each client already limits itself to one native transaction.  Keep writes
// conservative until both independent handshakes complete, but allow reads
// from the other clients to be issued while earlier reads await ordered MIG
// returns.  A bounded owner FIFO routes those returns without sampling a
// changing grant owner.

`default_nettype none

module kcu105_mig_app_arbiter #(
    parameter integer APP_ADDR_WIDTH = 28,
    parameter integer APP_DATA_WIDTH = 512,
    parameter integer APP_MASK_WIDTH = 64
) (
    input  wire                       clk,
    input  wire                       rst,

    input  wire [APP_ADDR_WIDTH-1:0]  cpu_app_addr,
    input  wire [2:0]                 cpu_app_cmd,
    input  wire                       cpu_app_en,
    output wire                       cpu_app_rdy,
    input  wire                       cpu_app_hi_pri,
    input  wire [APP_DATA_WIDTH-1:0]  cpu_app_wdf_data,
    input  wire                       cpu_app_wdf_end,
    input  wire [APP_MASK_WIDTH-1:0]  cpu_app_wdf_mask,
    input  wire                       cpu_app_wdf_wren,
    output wire                       cpu_app_wdf_rdy,
    output wire [APP_DATA_WIDTH-1:0]  cpu_app_rd_data,
    output wire                       cpu_app_rd_data_end,
    output wire                       cpu_app_rd_data_valid,

    input  wire [APP_ADDR_WIDTH-1:0]  gfx_app_addr,
    input  wire [2:0]                 gfx_app_cmd,
    input  wire                       gfx_app_en,
    output wire                       gfx_app_rdy,
    input  wire                       gfx_app_hi_pri,
    input  wire [APP_DATA_WIDTH-1:0]  gfx_app_wdf_data,
    input  wire                       gfx_app_wdf_end,
    input  wire [APP_MASK_WIDTH-1:0]  gfx_app_wdf_mask,
    input  wire                       gfx_app_wdf_wren,
    output wire                       gfx_app_wdf_rdy,
    output wire [APP_DATA_WIDTH-1:0]  gfx_app_rd_data,
    output wire                       gfx_app_rd_data_end,
    output wire                       gfx_app_rd_data_valid,

    input  wire [APP_ADDR_WIDTH-1:0]  video_app_addr,
    input  wire [2:0]                 video_app_cmd,
    input  wire                       video_app_en,
    output wire                       video_app_rdy,
    input  wire                       video_app_hi_pri,
    input  wire [APP_DATA_WIDTH-1:0]  video_app_wdf_data,
    input  wire                       video_app_wdf_end,
    input  wire [APP_MASK_WIDTH-1:0]  video_app_wdf_mask,
    input  wire                       video_app_wdf_wren,
    output wire                       video_app_wdf_rdy,
    output wire [APP_DATA_WIDTH-1:0]  video_app_rd_data,
    output wire                       video_app_rd_data_end,
    output wire                       video_app_rd_data_valid,

    output wire [APP_ADDR_WIDTH-1:0]  mig_app_addr,
    output wire [2:0]                 mig_app_cmd,
    output wire                       mig_app_en,
    input  wire                       mig_app_rdy,
    output wire                       mig_app_hi_pri,
    output wire [APP_DATA_WIDTH-1:0]  mig_app_wdf_data,
    output wire                       mig_app_wdf_end,
    output wire [APP_MASK_WIDTH-1:0]  mig_app_wdf_mask,
    output wire                       mig_app_wdf_wren,
    input  wire                       mig_app_wdf_rdy,
    input  wire [APP_DATA_WIDTH-1:0]  mig_app_rd_data,
    input  wire                       mig_app_rd_data_end,
    input  wire                       mig_app_rd_data_valid,

    output wire [63:0]                debug_probe
);
    localparam [2:0] MIG_CMD_WRITE = 3'b000;
    localparam [1:0] ST_IDLE       = 2'd0;
    localparam [1:0] ST_READ_CMD   = 2'd1;
    localparam [1:0] ST_ACK        = 2'd2;
    localparam [1:0] ST_WRITE      = 2'd3;
    localparam integer READ_OWNER_DEPTH = 8;
    localparam [3:0] READ_OWNER_DEPTH_COUNT = 4'd8;

    reg [1:0] state;
    reg [1:0] owner;
    reg [1:0] last_owner;
    reg       write_cmd_accepted;
    reg       write_data_accepted;
    reg [2:0] client_cmd_ready_q;
    reg [2:0] client_wdf_ready_q;
    reg [15:0] cpu_grant_count;
    reg [15:0] gfx_grant_count;
    reg [15:0] video_grant_count;
    // CPU and graphics permit at most one native read each.  The video
    // resampler may pipeline four scan reads so DDR return latency cannot
    // block its independent source-write stream.  Eight entries cover all
    // six legal requests plus reserve capacity. Keep the ordered owner queue
    // as explicit registers. The previous pointer-indexed
    // array inferred an asynchronous distributed-RAM read on the response
    // path; at the 300.12 MHz MIG clock that placed a RAMD32 lookup between a
    // pointer flop and each client's response-valid endpoint.  An explicit
    // registered head preserves the queue contract without putting storage
    // decoding in the response datapath.
    reg [1:0] read_owner_0;
    reg [1:0] read_owner_1;
    reg [1:0] read_owner_2;
    reg [1:0] read_owner_3;
    reg [1:0] read_owner_4;
    reg [1:0] read_owner_5;
    reg [1:0] read_owner_6;
    reg [1:0] read_owner_7;
    reg [3:0] read_owner_count;
    reg [APP_DATA_WIDTH-1:0] read_response_data_q;
    reg                       read_response_end_q;
    (* max_fanout = 32 *) reg [2:0] read_response_valid_q;

    reg [1:0] next_owner;
    always @* begin
        // A low-water HDMI scan request has a finite display deadline.  It
        // may bypass ordinary round robin before native acceptance, but never
        // preempts the transaction already held in a non-idle state.  The
        // resampler bounds urgent bursts with FIFO hysteresis.
        if (video_app_hi_pri) begin
            next_owner = 2'd2;
        end else begin
            next_owner = 2'd0;
            case (last_owner)
                2'd0: begin
                    if (gfx_app_en) next_owner = 2'd1;
                    else if (video_app_en) next_owner = 2'd2;
                    else next_owner = 2'd0;
                end
                2'd1: begin
                    if (video_app_en) next_owner = 2'd2;
                    else if (cpu_app_en) next_owner = 2'd0;
                    else next_owner = 2'd1;
                end
                default: begin
                    if (cpu_app_en) next_owner = 2'd0;
                    else if (gfx_app_en) next_owner = 2'd1;
                    else next_owner = 2'd2;
                end
            endcase
        end
    end

    wire next_owner_write = (next_owner == 2'd0) ?
        (cpu_app_cmd == MIG_CMD_WRITE) :
        (next_owner == 2'd1) ? (gfx_app_cmd == MIG_CMD_WRITE) :
                               (video_app_cmd == MIG_CMD_WRITE);
    wire read_owner_full = (read_owner_count == READ_OWNER_DEPTH_COUNT);

    wire owner_en = (owner == 2'd0) ? cpu_app_en :
                    (owner == 2'd1) ? gfx_app_en : video_app_en;
    wire owner_wdf_wren = (owner == 2'd0) ? cpu_app_wdf_wren :
                          (owner == 2'd1) ? gfx_app_wdf_wren :
                                             video_app_wdf_wren;
    // The generated Xilinx DDR4 native-interface example submits a complete
    // one-cycle BL8 write only when both the command FIFO and write-data FIFO
    // are ready.  Keep the clients' independent valid/ready interfaces, but
    // join them at the physical MIG boundary so commands and data cannot enter
    // separate native queues on different cycles.
    wire native_write_fire = (state == ST_WRITE) &&
                             !write_cmd_accepted &&
                             !write_data_accepted &&
                             owner_en && owner_wdf_wren &&
                             mig_app_rdy && mig_app_wdf_rdy;

    assign mig_app_addr = (owner == 2'd0) ? cpu_app_addr :
                          (owner == 2'd1) ? gfx_app_addr : video_app_addr;
    assign mig_app_cmd = (owner == 2'd0) ? cpu_app_cmd :
                         (owner == 2'd1) ? gfx_app_cmd : video_app_cmd;
    assign mig_app_hi_pri = (owner == 2'd0) ? cpu_app_hi_pri :
                            (owner == 2'd1) ? gfx_app_hi_pri :
                                               video_app_hi_pri;
    assign mig_app_wdf_data = (owner == 2'd0) ? cpu_app_wdf_data :
                              (owner == 2'd1) ? gfx_app_wdf_data :
                                                 video_app_wdf_data;
    assign mig_app_wdf_end = (owner == 2'd0) ? cpu_app_wdf_end :
                             (owner == 2'd1) ? gfx_app_wdf_end :
                                                video_app_wdf_end;
    assign mig_app_wdf_mask = (owner == 2'd0) ? cpu_app_wdf_mask :
                              (owner == 2'd1) ? gfx_app_wdf_mask :
                                                 video_app_wdf_mask;

    assign mig_app_en = ((state == ST_READ_CMD) && owner_en) ||
                        native_write_fire;
    assign mig_app_wdf_wren = native_write_fire;

    // A physical MIG acceptance is acknowledged to the selected client from
    // registers on the following cycle.  ST_ACK retains ownership until that
    // client samples the pulse, preventing a held request from being issued a
    // second time.  This is a real one-entry transaction boundary: no client
    // state or write payload lies on MIG-ready combinational feedback at the
    // 300 MHz UI clock.
    assign cpu_app_rdy = client_cmd_ready_q[0];
    assign gfx_app_rdy = client_cmd_ready_q[1];
    assign video_app_rdy = client_cmd_ready_q[2];
    assign cpu_app_wdf_rdy = client_wdf_ready_q[0];
    assign gfx_app_wdf_rdy = client_wdf_ready_q[1];
    assign video_app_wdf_rdy = client_wdf_ready_q[2];

    // MIG read returns are likewise captured before fanout.  The registered
    // data/end bundle and one-hot valid pulse remain aligned for consecutive
    // responses while the owner queue is retired at the physical interface.
    assign cpu_app_rd_data = read_response_data_q;
    assign gfx_app_rd_data = read_response_data_q;
    assign video_app_rd_data = read_response_data_q;
    assign cpu_app_rd_data_end = read_response_end_q;
    assign gfx_app_rd_data_end = read_response_end_q;
    assign video_app_rd_data_end = read_response_end_q;
    wire [1:0] read_response_owner = read_owner_0;
    wire read_response_fire = mig_app_rd_data_valid &&
                              (read_owner_count != 0);
    assign cpu_app_rd_data_valid = read_response_valid_q[0];
    assign gfx_app_rd_data_valid = read_response_valid_q[1];
    assign video_app_rd_data_valid = read_response_valid_q[2];

    wire write_cmd_fire = mig_app_en && mig_app_rdy;
    wire read_cmd_fire = (state == ST_READ_CMD) && write_cmd_fire;
    wire write_data_fire = mig_app_wdf_wren && mig_app_wdf_rdy;
    wire write_complete =
        (write_cmd_accepted || write_cmd_fire) &&
        (write_data_accepted || write_data_fire);

    always @(posedge clk) begin
        if (rst) begin
            state <= ST_IDLE;
            owner <= 2'd0;
            last_owner <= 2'd2;
            write_cmd_accepted <= 1'b0;
            write_data_accepted <= 1'b0;
            client_cmd_ready_q <= 3'b000;
            client_wdf_ready_q <= 3'b000;
            cpu_grant_count <= 16'd0;
            gfx_grant_count <= 16'd0;
            video_grant_count <= 16'd0;
            read_owner_0 <= 2'd0;
            read_owner_1 <= 2'd0;
            read_owner_2 <= 2'd0;
            read_owner_3 <= 2'd0;
            read_owner_4 <= 2'd0;
            read_owner_5 <= 2'd0;
            read_owner_6 <= 2'd0;
            read_owner_7 <= 2'd0;
            read_owner_count <= 4'd0;
            read_response_data_q <= {APP_DATA_WIDTH{1'b0}};
            read_response_end_q <= 1'b0;
            read_response_valid_q <= 3'b000;
        end else begin
            client_cmd_ready_q <= 3'b000;
            client_wdf_ready_q <= 3'b000;
            read_response_valid_q <= 3'b000;
            if (read_response_fire) begin
                read_response_data_q <= mig_app_rd_data;
                read_response_end_q <= mig_app_rd_data_end;
                case (read_response_owner)
                    2'd0: read_response_valid_q[0] <= 1'b1;
                    2'd1: read_response_valid_q[1] <= 1'b1;
                    2'd2: read_response_valid_q[2] <= 1'b1;
                    default: begin end
                endcase
            end

            case ({read_cmd_fire, read_response_fire})
                2'b10: begin
                    if (!read_owner_full) begin
                        case (read_owner_count)
                            4'd0: read_owner_0 <= owner;
                            4'd1: read_owner_1 <= owner;
                            4'd2: read_owner_2 <= owner;
                            4'd3: read_owner_3 <= owner;
                            4'd4: read_owner_4 <= owner;
                            4'd5: read_owner_5 <= owner;
                            4'd6: read_owner_6 <= owner;
                            4'd7: read_owner_7 <= owner;
                            default: begin end
                        endcase
                        read_owner_count <= read_owner_count + 4'd1;
                    end
                end
                2'b01: begin
                    read_owner_0 <= read_owner_1;
                    read_owner_1 <= read_owner_2;
                    read_owner_2 <= read_owner_3;
                    read_owner_3 <= read_owner_4;
                    read_owner_4 <= read_owner_5;
                    read_owner_5 <= read_owner_6;
                    read_owner_6 <= read_owner_7;
                    read_owner_count <= read_owner_count - 4'd1;
                end
                2'b11: begin
                    // Retire the registered head and append the newly issued
                    // request in one edge.  Count and order remain unchanged.
                    read_owner_0 <= read_owner_1;
                    read_owner_1 <= read_owner_2;
                    read_owner_2 <= read_owner_3;
                    read_owner_3 <= read_owner_4;
                    read_owner_4 <= read_owner_5;
                    read_owner_5 <= read_owner_6;
                    read_owner_6 <= read_owner_7;
                    case (read_owner_count)
                        4'd1: read_owner_0 <= owner;
                        4'd2: read_owner_1 <= owner;
                        4'd3: read_owner_2 <= owner;
                        4'd4: read_owner_3 <= owner;
                        4'd5: read_owner_4 <= owner;
                        4'd6: read_owner_5 <= owner;
                        4'd7: read_owner_6 <= owner;
                        4'd8: read_owner_7 <= owner;
                        default: begin end
                    endcase
                    read_owner_count <= read_owner_count;
                end
                default: read_owner_count <= read_owner_count;
            endcase

            case (state)
                ST_IDLE: begin
                    write_cmd_accepted <= 1'b0;
                    write_data_accepted <= 1'b0;
                    // An urgent video client has a registered acknowledgement
                    // turnaround cycle between native operations.  Preserve
                    // that finite-deadline reservation across the bubble;
                    // otherwise a normal CPU request is admitted first and
                    // cuts available video service nearly in half.  The
                    // video client clears hi_pri if no successor is ready.
                    if (((video_app_hi_pri && video_app_en) ||
                         (!video_app_hi_pri &&
                          (cpu_app_en || gfx_app_en || video_app_en))) &&
                        (next_owner_write || !read_owner_full)) begin
                        owner <= next_owner;
                        if (next_owner == 2'd1) begin
                            gfx_grant_count <= gfx_grant_count + 16'd1;
                            state <= (gfx_app_cmd == MIG_CMD_WRITE) ?
                                     ST_WRITE : ST_READ_CMD;
                        end else if (next_owner == 2'd2) begin
                            video_grant_count <= video_grant_count + 16'd1;
                            state <= (video_app_cmd == MIG_CMD_WRITE) ?
                                     ST_WRITE : ST_READ_CMD;
                        end else begin
                            cpu_grant_count <= cpu_grant_count + 16'd1;
                            state <= (cpu_app_cmd == MIG_CMD_WRITE) ?
                                     ST_WRITE : ST_READ_CMD;
                        end
                    end
                end

                ST_READ_CMD: begin
                    if (mig_app_en && mig_app_rdy) begin
                        last_owner <= owner;
                        case (owner)
                            2'd0: client_cmd_ready_q[0] <= 1'b1;
                            2'd1: client_cmd_ready_q[1] <= 1'b1;
                            2'd2: client_cmd_ready_q[2] <= 1'b1;
                            default: begin end
                        endcase
                        state <= ST_ACK;
                    end
                end

                ST_ACK: state <= ST_IDLE;

                ST_WRITE: begin
                    if (write_cmd_fire)
                        write_cmd_accepted <= 1'b1;
                    if (write_data_fire)
                        write_data_accepted <= 1'b1;
                    if (write_complete) begin
                        last_owner <= owner;
                        case (owner)
                            2'd0: begin
                                client_cmd_ready_q[0] <= 1'b1;
                                client_wdf_ready_q[0] <= 1'b1;
                            end
                            2'd1: begin
                                client_cmd_ready_q[1] <= 1'b1;
                                client_wdf_ready_q[1] <= 1'b1;
                            end
                            2'd2: begin
                                client_cmd_ready_q[2] <= 1'b1;
                                client_wdf_ready_q[2] <= 1'b1;
                            end
                            default: begin end
                        endcase
                        state <= ST_ACK;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    assign debug_probe = {
        cpu_grant_count,
        gfx_grant_count,
        video_grant_count,
        (read_owner_count >= 4'd7 ? 3'd7 : read_owner_count[2:0]),
        read_response_owner,
        owner,
        state,
        write_data_accepted,
        write_cmd_accepted,
        mig_app_rd_data_valid,
        mig_app_wdf_rdy,
        mig_app_rdy,
        mig_app_wdf_wren,
        mig_app_en
    };
endmodule

`default_nettype wire
