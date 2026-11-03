// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

// PROJECT CYAN - INT2 Interrupt Controller
// SGI IP20/IP22 Local I/O Interrupt Controller
//
// Base address: 0x1FBD9000 (PBUS channel 4)
//
// Register offsets (big-endian byte addressing with IP22BOFF |0x3):
//   0x00 LIO_0_ISR   - Local IO 0 interrupt status (ro)
//   0x04 LIO_0_MASK  - Local IO 0 interrupt mask (rw)
//   0x08 LIO_1_ISR   - Local IO 1 interrupt status (ro)
//   0x0C LIO_1_MASK  - Local IO 1 interrupt mask (rw)
//   0x10 LIO_2_3_ISR - Local IO 2/3 interrupt status (ro)
//   0x14 LIO_2_MASK  - Mapped LIO 0 (LIO2) mask (rw)
//   0x18 LIO_3_MASK  - Mapped LIO 1 (LIO3) mask (rw)
//   0x1C PORT_CONFIG - Port configuration (rw)
//   0x20 TIMER_ACK   - Timer acknowledge (wo)
//   0x30 PT_CLOCK    - 8254 Timer registers
//
// Since IP20 uses big-endian byte access, the PROM reads/writes
// byte 3 of each 32-bit word (offset |0x3).

`default_nettype none

module int2 (
    input  wire        clk,
    input  wire        rst_n,

    // CPU bus interface
    input  wire [7:0]  addr,       // Offset within INT2 (0x00-0xFF)
    input  wire [63:0] wdata,
    output reg  [63:0] rdata,
    input  wire [7:0]  wmask,
    input  wire        valid,
    input  wire        write,
    output reg         ready,

    // Interrupt inputs (directly from peripherals)
    input  wire        scsi0_irq,
    input  wire        scsi1_irq,
    input  wire        enet_irq,
    input  wire        hpc1_irq,

    // Timer interrupt inputs (from HPC1's 8254 timer)
    input  wire        timer_irq_0,  // Counter 0 (scheduling clock)
    input  wire        timer_irq_1,  // Counter 1 (profiling clock)

    // Interrupt output to CPU
    output wire [5:0]  cpu_irq
);

    // =========================================================================
    // Registers
    // =========================================================================

    // Mask registers (read/write)
    reg [7:0] lio_0_mask;    // LIO 0 interrupt mask
    reg [7:0] lio_1_mask;    // LIO 1 interrupt mask
    reg [7:0] lio_2_mask;    // LIO 2 interrupt mask (mapped)
    reg [7:0] lio_3_mask;    // LIO 3 interrupt mask (mapped)
    reg [7:0] port_config;   // Port configuration

    // Timer registers (the actual 8254 is in HPC1 at 0x1FB801F0)
    // INT2 only has LIO/mask registers at 0x1FBD9xxx
    // Timer IRQs come from HPC1 via timer_irq_0/timer_irq_1 inputs

    // =========================================================================
    // Interrupt Status (directly from inputs)
    // =========================================================================

    // LIO 0 status bits (IP20.h):
    //   [0] FIFO/GIO-0     LIO_FIFO   0x01
    //   [1] Centronics      LIO_CENTR  0x02
    //   [2] SCSI            LIO_SCSI   0x04
    //   [3] Ethernet        LIO_ENET   0x08
    //   [4] Graphics DMA    LIO_GDMA   0x10
    //   [5] DUART           LIO_DUART  0x20
    //   [6] GIO-1/2nd HPC   LIO_GIO_1  0x40
    //   [7] VME-0           LIO_VME0   0x80
    wire [7:0] lio_0_isr = {
        1'b0,           // [7] VME-0
        1'b0,           // [6] GIO-1/2nd HPC
        1'b0,           // [5] DUART
        1'b0,           // [4] Graphics DMA
        enet_irq,       // [3] Ethernet
        scsi0_irq,      // [2] SCSI
        1'b0,           // [1] Centronics
        1'b0            // [0] FIFO/GIO-0
    };

    // LIO 1 status bits (IP20.h):
    //   [0] unused
    //   [1] GR1 mode        LIO_GR1MODE 0x02
    //   [2] unused
    //   [3] VME-1           LIO_VME1    0x08
    //   [4] DSP             LIO_DSP     0x10
    //   [5] AC fail         LIO_AC      0x20
    //   [6] Video           LIO_VIDEO   0x40
    //   [7] Vert retrace    LIO_GIO_2   0x80
    wire [7:0] lio_1_isr = {
        1'b0,           // [7] GIO-2
        1'b0,           // [6] Video
        1'b0,           // [5] AC fail
        hpc1_irq,       // [4] DSP
        1'b0,           // [3] VME-1
        1'b0,           // [2] unused
        1'b0,           // [1] GR1 mode
        1'b0            // [0] unused
    };

    // LIO 2/3 status (mapped interrupts)
    wire [7:0] lio_2_3_isr = 8'h00;

    // =========================================================================
    // CPU interrupt output
    // =========================================================================

    // Masked interrupts
    wire [7:0] masked_lio0 = lio_0_isr & lio_0_mask;
    wire [7:0] masked_lio1 = lio_1_isr & lio_1_mask;

    // CPU IRQ lines (active high)
    // IP20 interrupt mapping (from IP20.c interrupt vector table):
    //   cpu_irq[0] → Cause IP[2] = SR_IBIT3 = LIO0 (SCSI, Ethernet, etc)
    //   cpu_irq[1] → Cause IP[3] = SR_IBIT4 = LIO1 (Power, HPC1, etc)
    //   cpu_irq[2] → Cause IP[4] = SR_IBIT5 = 8254 counter 0 (scheduling/RTCLOCK)
    //   cpu_irq[3] → Cause IP[5] = SR_IBIT6 = 8254 counter 1 (profiling/KGCLOCK)
    //   cpu_irq[4] → Cause IP[6] = SR_IBIT7 = bus error (MC)
    //   cpu_irq[5] → Cause IP[7] = SR_IBIT8 = R4000 Count/Compare (internal)
    assign cpu_irq = {
        1'b0,                   // [5] → IP[7] Count/Compare (internal to CPU)
        1'b0,                   // [4] → IP[6] Bus error (MC)
        timer_irq_1,            // [3] → IP[5] counter 1 (profiling/KGCLOCK)
        timer_irq_0,            // [2] → IP[4] counter 0 (scheduling/RTCLOCK)
        (|masked_lio1),         // [1] → IP[3] LIO1
        (|masked_lio0)          // [0] → IP[2] LIO0
    };

    // =========================================================================
    // Address decode
    // =========================================================================

    // Aligned word address (big-endian: byte 3 access means word access)
    wire [5:0] word_addr = addr[7:2];  // Word-aligned address

    // =========================================================================
    // Bus response logic
    // =========================================================================

    // Determine which byte is being accessed for writes
    wire [7:0] wdata_byte =
        wmask[7] ? wdata[63:56] :
        wmask[6] ? wdata[55:48] :
        wmask[5] ? wdata[47:40] :
        wmask[4] ? wdata[39:32] :
        wmask[3] ? wdata[31:24] :
        wmask[2] ? wdata[23:16] :
        wmask[1] ? wdata[15:8]  :
                   wdata[7:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lio_0_mask <= 8'h00;     // All masked initially
            lio_1_mask <= 8'h00;
            lio_2_mask <= 8'h00;
            lio_3_mask <= 8'h00;
            port_config <= 8'h00;
            rdata <= 64'h0;
            ready <= 1'b0;
        end else begin
            // Only clear ready when valid drops (prevents duplicate transactions)
            if (!valid) ready <= 1'b0;

            if (valid && !ready) begin
                ready <= 1'b1;
                // $display("INT2: access addr=%02x write=%d wmask=%02x wdata=%016x",
                //     addr, write, wmask, wdata);

                if (write) begin
                    case (word_addr)
                        6'h01: begin  // 0x04: LIO_0_MASK
                            lio_0_mask <= wdata_byte;
                            // $display("INT2: LIO_0_MASK write = %02x", wdata_byte);
                        end
                        6'h03: begin  // 0x0C: LIO_1_MASK
                            lio_1_mask <= wdata_byte;
                            // $display("INT2: LIO_1_MASK write = %02x", wdata_byte);
                        end
                        6'h05: begin  // 0x14: LIO_2_MASK
                            lio_2_mask <= wdata_byte;
                            // $display("INT2: LIO_2_MASK write = %02x", wdata_byte);
                        end
                        6'h06: begin  // 0x18: LIO_3_MASK
                            lio_3_mask <= wdata_byte;
                            // $display("INT2: LIO_3_MASK write = %02x", wdata_byte);
                        end
                        6'h07: begin  // 0x1C: PORT_CONFIG
                            port_config <= wdata_byte;
                            // $display("INT2: PORT_CONFIG write = %02x", wdata_byte);
                        end
                        default: ;
                    endcase
                end else begin
                    // Read operations
                    case (word_addr)
                        6'h00: begin  // 0x00: LIO_0_ISR
                            rdata <= {8{lio_0_isr}};
                            // $display("INT2: LIO_0_ISR read = %02x", lio_0_isr);
                        end
                        6'h01: begin  // 0x04: LIO_0_MASK
                            rdata <= {8{lio_0_mask}};
                            // $display("INT2: LIO_0_MASK read = %02x", lio_0_mask);
                        end
                        6'h02: begin  // 0x08: LIO_1_ISR
                            rdata <= {8{lio_1_isr}};
                            // $display("INT2: LIO_1_ISR read = %02x", lio_1_isr);
                        end
                        6'h03: begin  // 0x0C: LIO_1_MASK
                            rdata <= {8{lio_1_mask}};
                            // $display("INT2: LIO_1_MASK read = %02x", lio_1_mask);
                        end
                        6'h04: begin  // 0x10: LIO_2_3_ISR
                            rdata <= {8{lio_2_3_isr}};
                            // $display("INT2: LIO_2_3_ISR read = %02x", lio_2_3_isr);
                        end
                        6'h05: begin  // 0x14: LIO_2_MASK
                            rdata <= {8{lio_2_mask}};
                        end
                        6'h06: begin  // 0x18: LIO_3_MASK
                            rdata <= {8{lio_3_mask}};
                        end
                        6'h07: begin  // 0x1C: PORT_CONFIG
                            rdata <= {8{port_config}};
                        end
                        default: begin
                            rdata <= 64'h0;
                        end
                    endcase
                end
            end
        end
    end

endmodule

`default_nettype wire
