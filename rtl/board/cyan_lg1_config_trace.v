// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - passive circular accepted-LG1 trace.
//
// The trace consumes observational access records and management commands but
// has no output into the LG1 datapath. Accepted guest accesses always take
// write-port priority. Management commands wait for an idle observer cycle;
// the guest never waits for the trace.

`default_nettype none

module cyan_lg1_config_trace #(
    parameter integer ADDR_W = 14
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        access_valid,
    input  wire [87:0] access_payload,
    input  wire [ 2:0] current_configsel,

    // command 0: arm compatible configuration capture, 1: mark(tag),
    // command 2: freeze, command 3: arm a tag-selected bounded mode.
    input  wire [ 1:0] command,
    input  wire [ 7:0] command_tag,
    input  wire        command_valid,
    output wire        command_ready,
    output reg         command_ack,
    output reg         command_ok,
    output wire [127:0] status,

    input  wire [15:0] read_ordinal,
    input  wire        read_request,
    output wire [95:0] read_data,
    output reg         read_ack,
    output reg         read_valid
);
    localparam [ADDR_W:0] DEPTH_COUNT = {1'b1, {ADDR_W{1'b0}}};
    localparam [31:0] ADDR_WIDTH_VALUE = ADDR_W;
    wire [7:0] addr_width_status = ADDR_WIDTH_VALUE[7:0];

    reg [ADDR_W-1:0] write_ptr;
    reg [ADDR_W-1:0] oldest_ptr;
    reg [ADDR_W:0] retained_count;
    reg [18:0] stored_sequence;
    reg [15:0] overwrite_count;
    reg [15:0] timing_saturation_count;
    reg [63:0] cycle_count;
    reg [63:0] last_record_cycle;
    reg        armed;
    reg        frozen;
    reg        writes_only;
    reg        rex_interesting;
    reg        all_accesses;
    reg        command_seen;

    wire command_fire = command_valid && command_ready;
    assign command_ready = !access_valid && !command_seen;

    wire [63:0] access_delta_full = cycle_count - last_record_cycle;
    wire [ 7:0] access_delta = (access_delta_full > 64'd255) ?
                               8'hff : access_delta_full[7:0];
    wire [95:0] access_record = {access_payload, access_delta};
    // ARM begins a new logical trace, so its marker sequence is zero even on
    // a re-arm after an earlier capture. Other markers retain the running
    // sequence number from the current trace.
    wire arm_command = (command == 2'd0) || (command == 2'd3);
    wire [18:0] marker_sequence = arm_command ?
                                  19'd0 : stored_sequence;
    wire [7:0] marker_tag = arm_command ? 8'h00 : command_tag;
    wire [95:0] marker_record = {
        2'b01, marker_tag, cycle_count, current_configsel, marker_sequence
    };

    // access_payload[0] is the event encoder's accepted-access write bit.
    // Its address occupies payload bits [53:39], so bit 11 is payload[50].
    // The bounded REX predicate matches the exact simulator: every accepted
    // write or GO-alias access, without high-rate ordinary SET polling reads.
    wire access_config_region =
        (access_payload[53:51] == 3'b100);
    wire access_rex_interesting = access_payload[0] || access_payload[50];
    wire access_selected = all_accesses ? 1'b1 :
                           rex_interesting ? access_rex_interesting :
                           (access_config_region &&
                            (!writes_only || access_payload[0]));
    wire access_store = access_valid && armed &&
                        access_selected;
    wire command_store = command_fire &&
                         (arm_command ||
                          ((command == 2'd1 || command == 2'd2) && armed));
    wire ram_we = access_store || command_store;
    wire [ADDR_W-1:0] ram_waddr =
        (command_fire && arm_command) ? {ADDR_W{1'b0}} : write_ptr;
    wire [95:0] ram_wdata = access_store ? access_record : marker_record;

    wire [15:0] retained_status =
        {{(15-ADDR_W){1'b0}}, retained_count};
    wire read_in_range = frozen && (read_ordinal < retained_status);
    wire [ADDR_W-1:0] read_physical_addr =
        oldest_ptr + read_ordinal[ADDR_W-1:0];

    cyan_simple_dual_port_ram #(
        .ADDR_W(ADDR_W),
        .DATA_W(96)
    ) u_trace_ram (
        .clk(clk),
        .raddr(read_physical_addr),
        .rdata(read_data),
        .we(ram_we),
        .waddr(ram_waddr),
        .wdata(ram_wdata)
    );

    wire wrapped = (overwrite_count != 0);
    wire timing_saturated = (timing_saturation_count != 0);
    wire command_pending = command_valid && !command_ready;
    wire [7:0] flags = {
        armed, frozen, wrapped, command_pending, timing_saturated,
        writes_only, rex_interesting, all_accesses
    };
    wire [15:0] oldest_status = {{(16-ADDR_W){1'b0}}, oldest_ptr};
    wire [15:0] write_status = {{(16-ADDR_W){1'b0}}, write_ptr};
    assign status = {
        8'h4c, 8'h47, 8'h01, 8'h0c,
        flags, addr_width_status, retained_status,
        oldest_status, write_status, overwrite_count,
        timing_saturation_count
    };

    task automatic advance_ring;
        begin
            stored_sequence <= stored_sequence + 19'd1;
            write_ptr <= write_ptr + 1'b1;
            if (retained_count < DEPTH_COUNT)
                retained_count <= retained_count + 1'b1;
            else begin
                oldest_ptr <= oldest_ptr + 1'b1;
                if (overwrite_count != 16'hffff)
                    overwrite_count <= overwrite_count + 16'd1;
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            write_ptr <= 0;
            oldest_ptr <= 0;
            retained_count <= 0;
            stored_sequence <= 0;
            overwrite_count <= 0;
            timing_saturation_count <= 0;
            cycle_count <= 0;
            last_record_cycle <= 0;
            armed <= 1'b0;
            frozen <= 1'b1;
            writes_only <= 1'b0;
            rex_interesting <= 1'b0;
            all_accesses <= 1'b0;
            command_seen <= 1'b0;
            command_ack <= 1'b0;
            command_ok <= 1'b0;
            read_ack <= 1'b0;
            read_valid <= 1'b0;
        end else begin
            cycle_count <= cycle_count + 64'd1;
            command_ack <= 1'b0;
            command_ok <= 1'b0;
            read_ack <= read_request;
            read_valid <= read_request && read_in_range;

            if (!command_valid)
                command_seen <= 1'b0;

            if (access_store) begin
                advance_ring();
                last_record_cycle <= cycle_count;
                if ((access_delta_full > 64'd255) &&
                    (timing_saturation_count != 16'hffff))
                    timing_saturation_count <=
                        timing_saturation_count + 16'd1;
            end

            if (command_fire) begin
                command_seen <= 1'b1;
                command_ack <= 1'b1;
                case (command)
                    2'd0: begin
                        // The ARM marker is retained as logical record zero.
                        write_ptr <= {{(ADDR_W-1){1'b0}}, 1'b1};
                        oldest_ptr <= 0;
                        retained_count <= 1;
                        stored_sequence <= 1;
                        overwrite_count <= 0;
                        timing_saturation_count <= 0;
                        last_record_cycle <= cycle_count;
                        armed <= 1'b1;
                        frozen <= 1'b0;
                        writes_only <= 1'b0;
                        rex_interesting <= 1'b0;
                        all_accesses <= 1'b0;
                        command_ok <= 1'b1;
                    end
                    2'd1: begin
                        if (armed) begin
                            advance_ring();
                            last_record_cycle <= cycle_count;
                            command_ok <= 1'b1;
                        end
                    end
                    2'd2: begin
                        if (armed) begin
                            advance_ring();
                            last_record_cycle <= cycle_count;
                            armed <= 1'b0;
                            frozen <= 1'b1;
                            command_ok <= 1'b1;
                        end
                    end
                    2'd3: begin
                        // Command tag 2 selects the bounded REX predicate;
                        // tag 3 retains every accepted public LG1 access for
                        // short latency-discriminator intervals. Other tags
                        // retain compatible config-write-only behavior.
                        write_ptr <= {{(ADDR_W-1){1'b0}}, 1'b1};
                        oldest_ptr <= 0;
                        retained_count <= 1;
                        stored_sequence <= 1;
                        overwrite_count <= 0;
                        timing_saturation_count <= 0;
                        last_record_cycle <= cycle_count;
                        armed <= 1'b1;
                        frozen <= 1'b0;
                        writes_only <= (command_tag != 8'h02) &&
                                       (command_tag != 8'h03);
                        rex_interesting <= (command_tag == 8'h02);
                        all_accesses <= (command_tag == 8'h03);
                        command_ok <= 1'b1;
                    end
                    default: ;
                endcase
            end
        end
    end
endmodule

`default_nettype wire
