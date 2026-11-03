// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

//////////////////////////////////////////////////////////////////////////////
// DSP56001 Address Generation Unit (AGU)
//
// Implements all DSP56001 addressing modes with 16-bit R/N/M registers:
// - Linear:    M = 0xFFFF (default), simple signed arithmetic
// - Modulo:    0 < M < 0x7FFF, circular buffer addressing
// - Bit-reverse: M = 0x0000, FFT butterfly addressing
//
// R0-R7: 16-bit address registers
// N0-N7: 16-bit offset registers
// M0-M7: 16-bit modifier registers (default 0xFFFF = linear)
//
// AGU pipeline delay: 1-instruction delay before new R/N/M values
// are used as pointers. Replicate Hatari's agu_pipeline mechanism.
//////////////////////////////////////////////////////////////////////////////

module dsp56001_agu (
    // ========================================================================
    // AGU Register File (indexed access)
    // ========================================================================
    input  wire [15:0] r_in,              // Current R[rrr] value
    input  wire [15:0] n_in,              // Current N[rrr] value
    input  wire [15:0] m_in,              // Current M[rrr] value

    output wire [15:0] r_out,             // New R value to write
    output wire        r_wr_en,           // Write enable for R[rrr]
    output wire [2:0]  r_wr_idx,          // Which R register to write

    // ========================================================================
    // Control (from instruction decoder)
    // ========================================================================
    input  wire [2:0]  agu_rrr,           // Register select (0-7)
    input  wire [2:0]  agu_mmm,           // Addressing mode

    // ========================================================================
    // Effective Address Output (combinational)
    // ========================================================================
    output wire [15:0] eff_addr,          // Effective address for memory access
    output wire        eff_addr_valid     // Address is valid (mode is not AbsAddr)
);

    // ========================================================================
    // Addressing Mode Encoding
    // ========================================================================
    localparam [2:0] MMM_RnMinusNn     = 3'b000;  // (Rn)-Nn
    localparam [2:0] MMM_RnPlusNn      = 3'b001;  // (Rn)+Nn
    localparam [2:0] MMM_RnMinus       = 3'b010;  // (Rn)-
    localparam [2:0] MMM_RnPlus        = 3'b011;  // (Rn)+
    localparam [2:0] MMM_Rn            = 3'b100;  // (Rn)
    localparam [2:0] MMM_RnPlusNnNoUpd = 3'b101;  // (Rn+Nn)
    localparam [2:0] MMM_AbsAddr       = 3'b110;  // Absolute address
    localparam [2:0] MMM_MinusRn       = 3'b111;  // -(Rn)

    // ========================================================================
    // Mode Detection
    // ========================================================================
    wire linear_mode    = (m_in == 16'hFFFF);
    wire bitreverse_mode = (m_in == 16'h0000);
    wire modulo_mode    = !linear_mode && !bitreverse_mode;

    // ========================================================================
    // Modulo Mask Calculation
    // ========================================================================
    wire [15:0] m_step1 = m_in | (m_in >> 1);
    wire [15:0] m_step2 = m_step1 | (m_step1 >> 2);
    wire [15:0] m_step3 = m_step2 | (m_step2 >> 4);
    wire [15:0] m_step4 = m_step3 | (m_step3 >> 8);
    wire [15:0] modulo_mask = m_step4;

    // ========================================================================
    // Offset Value Selection
    // ========================================================================
    reg signed [16:0] offset;
    reg        use_n;

    always @(*) begin
        case (agu_mmm)
            MMM_RnMinusNn:     begin offset = -{1'b0, n_in}; use_n = 1'b1; end
            MMM_RnPlusNn:      begin offset =  {1'b0, n_in}; use_n = 1'b1; end
            MMM_RnMinus:       begin offset = -17'sd1;        use_n = 1'b0; end
            MMM_RnPlus:        begin offset =  17'sd1;        use_n = 1'b0; end
            MMM_Rn:            begin offset =  17'sd0;        use_n = 1'b0; end
            MMM_RnPlusNnNoUpd: begin offset =  {1'b0, n_in}; use_n = 1'b1; end
            MMM_MinusRn:       begin offset = -17'sd1;        use_n = 1'b0; end
            default:           begin offset =  17'sd0;        use_n = 1'b0; end
        endcase
    end

    wire update_r   = (agu_mmm != MMM_Rn) && (agu_mmm != MMM_RnPlusNnNoUpd) && (agu_mmm != MMM_AbsAddr);
    wire pre_update = (agu_mmm == MMM_MinusRn);

    // ========================================================================
    // Linear Address Calculation
    // ========================================================================
    wire signed [16:0] r_signed = {1'b0, r_in};
    wire signed [16:0] linear_result = r_signed + offset;

    // ========================================================================
    // Modulo Address Calculation
    // ========================================================================
    wire [15:0] lower_bound = r_in & ~modulo_mask;
    // Work relative to the aligned buffer base before truncating to 16 bits.
    // Comparing the truncated linear address loses a carry at $ffff and a
    // borrow at $0000, which can move a legal update into an unrelated block.
    wire signed [17:0] modulo_limit = $signed({2'b00, m_in});
    wire signed [17:0] modulo_size = modulo_limit + 18'sd1;
    wire signed [17:0] modulo_relative =
        $signed({2'b00, r_in}) - $signed({2'b00, lower_bound}) +
        $signed(offset);
    wire signed [17:0] modulo_relative_corrected =
        (modulo_relative < 0) ? (modulo_relative + modulo_size) :
        (modulo_relative > modulo_limit) ?
            (modulo_relative - modulo_size) : modulo_relative;
    wire [17:0] modulo_address =
        {2'b00, lower_bound} + modulo_relative_corrected;
    wire [15:0] modulo_corrected = modulo_address[15:0];

    // UM05 5.3.2.2 defines N=P*2^k as a linear jump to the same relative
    // position in another aligned buffer, even though Mn selects modulo mode.
    wire modulo_block_jump = use_n && (n_in != 16'h0000) &&
                             ((n_in & modulo_mask) == 16'h0000);

    // ========================================================================
    // Bit-Reverse Address Calculation
    // ========================================================================
    function [15:0] bitreverse16;
        input [15:0] x;
        reg [15:0] t1, t2, t3;
        begin
            t1 = ((x & 16'hAAAA) >> 1) | ((x & 16'h5555) << 1);
            t2 = ((t1 & 16'hCCCC) >> 2) | ((t1 & 16'h3333) << 2);
            t3 = ((t2 & 16'hF0F0) >> 4) | ((t2 & 16'h0F0F) << 4);
            bitreverse16 = ((t3 & 16'hFF00) >> 8) | ((t3 & 16'h00FF) << 8);
        end
    endfunction

    wire [15:0] r_rev = bitreverse16(r_in);
    wire [15:0] n_rev = bitreverse16(n_in);
    wire [15:0] bitrev_sum = offset[16] ? (r_rev - n_rev) : (r_rev + n_rev);
    wire [15:0] bitrev_result = bitreverse16(bitrev_sum);
    wire [15:0] one_rev = 16'h8000;  // bitreverse16(16'd1)
    wire [15:0] bitrev_inc = r_rev + one_rev;
    wire [15:0] bitrev_dec = r_rev - one_rev;
    wire [15:0] bitrev_pm1 = offset[16] ? bitreverse16(bitrev_dec) : bitreverse16(bitrev_inc);

    // ========================================================================
    // Final Address Selection
    // ========================================================================
    reg [15:0] new_r;

    always @(*) begin
        if (linear_mode)
            new_r = linear_result[15:0];
        else if (bitreverse_mode)
            new_r = use_n ? bitrev_result : bitrev_pm1;
        else
            new_r = modulo_block_jump ? linear_result[15:0] :
                                        modulo_corrected;
    end

    wire [15:0] addr_for_access = pre_update ? new_r : r_in;
    // ========================================================================
    // Combinational Outputs
    // ========================================================================
    // Indexed addressing uses the same Mn-selected arithmetic as an update;
    // only the Rn writeback is suppressed.
    assign eff_addr = (agu_mmm == MMM_RnPlusNnNoUpd) ? new_r : addr_for_access;
    assign eff_addr_valid = (agu_mmm != MMM_AbsAddr);
    assign r_out    = new_r;
    assign r_wr_idx = agu_rrr;
    assign r_wr_en  = update_r;

endmodule
