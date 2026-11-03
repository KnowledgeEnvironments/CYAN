// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

//////////////////////////////////////////////////////////////////////////////
// DSP56001 Data ALU
//
// 56-bit ALU with integrated 24x24 MAC unit.
// Implements all DSP56001 data ALU operations:
//   Arithmetic: ABS, ADC, ADD, ADDL, ADDR, CLR, CMP, CMPM, NEG, RND,
//               SBC, SUB, SUBL, TFR, TST
//   Multiply:   MPY, MPYR, MAC, MACR
//   Logic:      AND, OR, EOR, NOT (on A1/B1 only)
//   Shift:      ASL, ASR, LSL, LSR, ROL, ROR (single-bit)
//   Division:   DIV
//   Normalize:  NORM (iterative, shifts D one bit toward normalization)
//
// Accumulator format (56-bit):
//   [55:48] = A2/B2 (extension byte, sign extension)
//   [47:24] = A1/B1 (high word, MSP, 24-bit)
//   [23:0]  = A0/B0 (low word, LSP, 24-bit)
//
// CCR flags: [7:0] = {S, L, E, U, N, Z, V, C}
//////////////////////////////////////////////////////////////////////////////

module dsp56001_alu (
    // ========================================================================
    // Operand Inputs
    // ========================================================================
    input  wire [55:0] acc_a,           // Accumulator A
    input  wire [55:0] acc_b,           // Accumulator B
    input  wire [23:0] x0, x1,         // X register halves
    input  wire [23:0] y0, y1,         // Y register halves

    // ========================================================================
    // Operation Control
    // ========================================================================
    input  wire [5:0]  alu_opcode,      // Operation code
    input  wire        dest_sel,        // Destination: 0=A, 1=B
    input  wire [2:0]  src_jjj,         // JJJ source select
    input  wire        negate,          // Negate product (MPY/MAC sign bit)

    // ========================================================================
    // Multiply Source Selection
    // ========================================================================
    input  wire [2:0]  mpy_src_qq,      // SSS source pair select (3-bit)

    // ========================================================================
    // Status Register Input (for carry, scaling mode)
    // ========================================================================
    input  wire [23:0] sr_in,

    // ========================================================================
    // Results (combinational)
    // ========================================================================
    output wire [55:0] result,          // ALU result
    output wire        result_valid,    // Result should be written back

    // ========================================================================
    // CCR Output (combinational)
    // ========================================================================
    output wire [7:0]  ccr_out,         // CCR flags {0,L,E,U,N,Z,V,C}
    output wire        ccr_valid,       // CCR update valid

    // ========================================================================
    // NORM Direction Output (for AGU R register update)
    // ========================================================================
    output wire        norm_shift_dir,  // 1=ASR (right), 0=ASL (left)
    output wire        norm_did_shift   // 1 if NORM actually shifted
);

    // ========================================================================
    // ALU Opcode Definitions
    // ========================================================================
    localparam [5:0] ALU_NOP   = 6'd0;
    localparam [5:0] ALU_ABS   = 6'd1;
    localparam [5:0] ALU_ADD   = 6'd2;
    localparam [5:0] ALU_SUB   = 6'd3;
    localparam [5:0] ALU_AND   = 6'd4;
    localparam [5:0] ALU_OR    = 6'd5;
    localparam [5:0] ALU_EOR   = 6'd6;
    localparam [5:0] ALU_NOT   = 6'd7;
    localparam [5:0] ALU_NEG   = 6'd8;
    localparam [5:0] ALU_CLR   = 6'd9;
    localparam [5:0] ALU_TST   = 6'd10;
    localparam [5:0] ALU_CMP   = 6'd11;
    localparam [5:0] ALU_TFR   = 6'd12;
    localparam [5:0] ALU_ASL   = 6'd13;
    localparam [5:0] ALU_ASR   = 6'd14;
    localparam [5:0] ALU_LSL   = 6'd15;
    localparam [5:0] ALU_LSR   = 6'd16;
    localparam [5:0] ALU_RND   = 6'd17;
    localparam [5:0] ALU_MPY   = 6'd20;
    localparam [5:0] ALU_MAC   = 6'd21;
    localparam [5:0] ALU_CMPM  = 6'd22;
    localparam [5:0] ALU_ADDL  = 6'd23;
    localparam [5:0] ALU_ADDR  = 6'd24;
    localparam [5:0] ALU_SUBL  = 6'd25;
    localparam [5:0] ALU_ROL   = 6'd26;
    localparam [5:0] ALU_ROR   = 6'd27;
    localparam [5:0] ALU_DIV   = 6'd28;
    localparam [5:0] ALU_MPYR  = 6'd29;
    localparam [5:0] ALU_MACR  = 6'd30;
    localparam [5:0] ALU_ADC   = 6'd31;  // DSP56001-specific
    localparam [5:0] ALU_SBC   = 6'd32;  // DSP56001-specific
    localparam [5:0] ALU_NORM  = 6'd33;  // DSP56001-specific (iterative)
    localparam [5:0] ALU_SUBR  = 6'd34;  // SUBR: D = S - D/2

    // ========================================================================
    // CCR Bit Positions
    // ========================================================================
    localparam CCR_C = 0;   // Carry
    localparam CCR_V = 1;   // Overflow
    localparam CCR_Z = 2;   // Zero
    localparam CCR_N = 3;   // Negative
    localparam CCR_U = 4;   // Unnormalized
    localparam CCR_E = 5;   // Extension
    localparam CCR_L = 6;   // Limit (sticky)

    // SR scaling mode bits
    localparam SR_S0 = 10;
    localparam SR_S1 = 11;

    // ========================================================================
    // Destination / Source Selection
    // ========================================================================
    wire [55:0] dst_acc   = dest_sel ? acc_b : acc_a;
    wire [55:0] other_acc = dest_sel ? acc_a : acc_b;

    // JJJ source operand decoding (same as DSP56300)
    reg [55:0] src_operand;
    always @(*) begin
        case (src_jjj)
            3'b000: src_operand = other_acc;                                     // B/A (opposite)
            3'b001: src_operand = other_acc;                                     // same
            3'b010: src_operand = {{8{x1[23]}}, x1, x0};                         // X (sign-ext)
            3'b011: src_operand = {{8{y1[23]}}, y1, y0};                         // Y (sign-ext)
            3'b100: src_operand = {{8{x0[23]}}, x0, 24'h0};                      // X0 in A1 pos
            3'b101: src_operand = {{8{y0[23]}}, y0, 24'h0};                      // Y0 in A1 pos
            3'b110: src_operand = {{8{x1[23]}}, x1, 24'h0};                      // X1 in A1 pos
            3'b111: src_operand = {{8{y1[23]}}, y1, 24'h0};                      // Y1 in A1 pos
        endcase
    end

    // ========================================================================
    // 56-bit Arithmetic
    // ========================================================================
    wire [56:0] add_full = {1'b0, dst_acc} + {1'b0, src_operand};
    wire [55:0] add_result = add_full[55:0];
    wire        add_carry  = add_full[56];
    wire        add_overflow = (dst_acc[55] == src_operand[55]) &&
                               (add_result[55] != dst_acc[55]);

    wire [56:0] sub_full = {1'b0, dst_acc} - {1'b0, src_operand};
    wire [55:0] sub_result = sub_full[55:0];
    wire        sub_borrow = sub_full[56];
    wire        sub_overflow = (dst_acc[55] != src_operand[55]) &&
                               (sub_result[55] != dst_acc[55]);

    // ADC: add with carry (D + S + C)
    wire [56:0] adc_full = {1'b0, dst_acc} + {1'b0, src_operand} + {56'b0, sr_in[CCR_C]};
    wire [55:0] adc_result = adc_full[55:0];
    wire        adc_carry  = adc_full[56];
    wire        adc_overflow = (dst_acc[55] == src_operand[55]) &&
                               (adc_result[55] != dst_acc[55]);

    // SBC: subtract with carry (D - S - C)
    wire [56:0] sbc_full = {1'b0, dst_acc} - {1'b0, src_operand} - {56'b0, sr_in[CCR_C]};
    wire [55:0] sbc_result = sbc_full[55:0];
    wire        sbc_borrow = sbc_full[56];
    wire        sbc_overflow = (dst_acc[55] != src_operand[55]) &&
                               (sbc_result[55] != dst_acc[55]);

    wire [55:0] neg_result = (~dst_acc) + 56'h1;
    wire [55:0] abs_result = dst_acc[55] ? neg_result : dst_acc;

    // ADDL: (D << 1) + S.  Carry/borrow is from the final arithmetic, while
    // overflow also includes a sign change caused by the initial left shift.
    wire [55:0] left_shifted = {dst_acc[54:0], 1'b0};
    wire        left_shift_overflow = dst_acc[55] != dst_acc[54];
    wire [56:0] addl_full = {1'b0, left_shifted} + {1'b0, src_operand};
    wire [55:0] addl_result = addl_full[55:0];
    wire        addl_carry = addl_full[56];
    wire        addl_overflow = left_shift_overflow ||
                                ((left_shifted[55] == src_operand[55]) &&
                                 (addl_result[55] != left_shifted[55]));
    // ADDR: (D >> 1, sign-preserving) + S
    wire [55:0] addr_half = {dst_acc[55], dst_acc[55:1]};
    wire [56:0] addr_full = {1'b0, addr_half} + {1'b0, src_operand};
    wire [55:0] addr_result = addr_full[55:0];
    wire        addr_carry = addr_full[56];
    wire        addr_overflow = (addr_half[55] == src_operand[55]) &&
                                (addr_result[55] != addr_half[55]);
    // SUBL: (D << 1) - S
    wire [56:0] subl_full = {1'b0, left_shifted} - {1'b0, src_operand};
    wire [55:0] subl_result = subl_full[55:0];
    wire        subl_borrow = subl_full[56];
    wire        subl_overflow = left_shift_overflow ||
                                ((left_shifted[55] != src_operand[55]) &&
                                 (subl_result[55] != left_shifted[55]));
    // SUBR: (D >> 1, sign-preserving) - S
    wire [56:0] subr_full = {1'b0, addr_half} - {1'b0, src_operand};
    wire [55:0] subr_result = subr_full[55:0];
    wire        subr_borrow = subr_full[56];
    wire        subr_overflow = (addr_half[55] != src_operand[55]) &&
                                (subr_result[55] != addr_half[55]);

    // ========================================================================
    // Logical Operations (on bits [47:24] = A1/B1 only)
    // ========================================================================
    wire [55:0] and_result = {dst_acc[55:48], dst_acc[47:24] & src_operand[47:24], dst_acc[23:0]};
    wire [55:0] or_result  = {dst_acc[55:48], dst_acc[47:24] | src_operand[47:24], dst_acc[23:0]};
    wire [55:0] eor_result = {dst_acc[55:48], dst_acc[47:24] ^ src_operand[47:24], dst_acc[23:0]};
    wire [55:0] not_result = {dst_acc[55:48], ~dst_acc[47:24], dst_acc[23:0]};

    // ========================================================================
    // Single-Bit Shift Operations
    // ========================================================================
    // ASL: arithmetic shift left (full 56-bit)
    wire [55:0] asl_result = {dst_acc[54:0], 1'b0};
    wire        asl_carry  = dst_acc[55];
    wire        asl_overflow = (dst_acc[55] != dst_acc[54]);

    // ASR: arithmetic shift right (full 56-bit, sign-preserving)
    wire [55:0] asr_result = {dst_acc[55], dst_acc[55:1]};
    wire        asr_carry  = dst_acc[0];

    // LSL: logical shift left (A1/B1 = bits 47:24 only)
    wire [23:0] dst_a1 = dst_acc[47:24];
    wire [55:0] lsl_result = {dst_acc[55:48], dst_a1[22:0], 1'b0, dst_acc[23:0]};
    wire        lsl_carry  = dst_a1[23];

    // LSR: logical shift right (A1/B1 only)
    wire [55:0] lsr_result = {dst_acc[55:48], 1'b0, dst_a1[23:1], dst_acc[23:0]};
    wire        lsr_carry  = dst_a1[0];

    // ROL: rotate left through carry (A1/B1 only)
    wire [55:0] rol_result = {dst_acc[55:48], dst_a1[22:0], sr_in[CCR_C], dst_acc[23:0]};
    wire        rol_carry  = dst_a1[23];

    // ROR: rotate right through carry (A1/B1 only)
    wire [55:0] ror_result = {dst_acc[55:48], sr_in[CCR_C], dst_a1[23:1], dst_acc[23:0]};
    wire        ror_carry  = dst_a1[0];

    // ========================================================================
    // RND: Convergent Rounding
    // ========================================================================
    wire rnd_s0 = sr_in[SR_S0];
    wire rnd_s1 = sr_in[SR_S1];
    wire [24:0] rnd_rounder = rnd_s1 ? 25'h0400000 :
                              rnd_s0 ? 25'h1000000 :
                                       25'h0800000;
    wire [25:0] rnd_mask = (rnd_rounder << 1) - 26'd1;
    wire [56:0] rnd_added = {1'b0, dst_acc} + {32'b0, rnd_rounder};
    wire rnd_overflow = !dst_acc[55] && rnd_added[55];
    wire rnd_at_midpoint = (rnd_added[25:0] & rnd_mask[25:0]) == 26'h0;
    wire rnd_clear_extra = rnd_at_midpoint;  // Convergent: clear for even
    wire [55:0] rnd_masked = rnd_added[55:0] & ~{30'h0, rnd_mask[25:0]};
    wire [55:0] rnd_result = rnd_clear_extra ?
        (rnd_masked & ~{30'h0, rnd_rounder[24:0], 1'b0}) : rnd_masked;

    // ========================================================================
    // DIV: Non-restoring Division Iteration
    // ========================================================================
    wire [23:0] div_src_24;
    assign div_src_24 = (src_jjj[1:0] == 2'b00) ? x0 :
                        (src_jjj[1:0] == 2'b01) ? y0 :
                        (src_jjj[1:0] == 2'b10) ? x1 : y1;

    wire        div_c = (dst_acc[55] != div_src_24[23]);
    wire [55:0] div_shifted = {dst_acc[54:0], sr_in[CCR_C]};
    wire [31:0] div_src_ext = {{8{div_src_24[23]}}, div_src_24};
    wire [31:0] div_upper = div_shifted[55:24];
    wire [31:0] div_upper_result = div_c ? (div_upper + div_src_ext) :
                                           (div_upper - div_src_ext);
    wire [55:0] div_result = {div_upper_result, div_shifted[23:0]};
    wire        div_carry = ~div_result[55];
    wire        div_overflow = (div_shifted[55] != dst_acc[55]);

    // ========================================================================
    // NORM: Iterative Normalization
    // The instruction consumes the E/U/Z classification already present in
    // the CCR.  The manual requires callers to establish those flags for D
    // before NORM; recomputing the decision here changes architectural state
    // when the incoming classification selects a different action.
    //   !E && U && !Z: ASL D and decrement Rn
    //    E:            ASR D and increment Rn
    //   otherwise:     no shift
    // ========================================================================
    wire        norm_did_asl = !sr_in[CCR_E] && sr_in[CCR_U] &&
                               !sr_in[CCR_Z];
    wire        norm_did_asr = sr_in[CCR_E];
    wire [55:0] norm_result  = norm_did_asr ? asr_result :
                               norm_did_asl ? asl_result : dst_acc;
    wire        norm_overflow = norm_did_asl &&
                                (dst_acc[55] != dst_acc[54]);

    // ========================================================================
    // 24x24 Signed Multiply (Synthesizer-Friendly)
    // ========================================================================
    // Uses native signed 25x25 multiply to match Cyclone IV DSP block.
    // Quartus can infer two 18x18 DSP blocks for a 25x25 signed multiply.
    // Registered inputs/outputs help inference.
    //
    // SSS source pair decoding (from Hatari opcodes_alu, 3-bit):
    //   000: X0*X0, 001: Y0*Y0, 010: X1*X0, 011: Y1*Y0
    //   100: X0*Y1, 101: Y0*X0, 110: X1*Y0, 111: Y1*X1
    reg [23:0] mpy_s1, mpy_s2;
    always @(*) begin
        case (mpy_src_qq)
            3'b000: begin mpy_s1 = x0; mpy_s2 = x0; end
            3'b001: begin mpy_s1 = y0; mpy_s2 = y0; end
            3'b010: begin mpy_s1 = x1; mpy_s2 = x0; end
            3'b011: begin mpy_s1 = y1; mpy_s2 = y0; end
            3'b100: begin mpy_s1 = x0; mpy_s2 = y1; end
            3'b101: begin mpy_s1 = y0; mpy_s2 = x0; end
            3'b110: begin mpy_s1 = x1; mpy_s2 = y0; end
            3'b111: begin mpy_s1 = y1; mpy_s2 = x1; end
        endcase
    end

    // Sign-extend 24-bit to 25-bit for signed multiply (keeps in DSP block range)
    wire signed [24:0] mpy_a = {mpy_s1[23], mpy_s1};
    wire signed [24:0] mpy_b = {mpy_s2[23], mpy_s2};

    // 25x25 signed multiply → 50-bit product
    // Cyclone IV: inferred as 2x 18x18 DSP blocks
    wire signed [49:0] mpy_product_raw = mpy_a * mpy_b;

    // The true 24x24 signed product is in bits [47:0] of the 50-bit result
    wire signed [47:0] mpy_product_48 = mpy_product_raw[47:0];

    // Fractional mode (default): shift left by 1, sign-extend to 56-bit
    wire [55:0] mpy_product = {{8{mpy_product_48[47]}}, mpy_product_48[46:0], 1'b0};

    // Negate if k-bit set
    wire [55:0] mpy_negated = negate ? (~mpy_product + 56'h1) : mpy_product;

    // MPY result (no accumulate)
    wire [55:0] mpy_result = mpy_negated;

    // MAC result (accumulate into destination)
    wire signed [56:0] mac_acc_ext  = {dst_acc[55], dst_acc};
    wire signed [56:0] mac_prod_ext = {mpy_negated[55], mpy_negated};
    wire signed [56:0] mac_sum = mac_acc_ext + mac_prod_ext;
    wire [55:0] mac_result = mac_sum[55:0];
    wire        mac_overflow = (mac_sum[56] != mac_sum[55]);

    // Rounding for MPYR/MACR uses the same scaling-mode-selected position and
    // convergent midpoint rule as RND.
    wire [55:0] mpy_to_round;
    wire is_mac_type = (alu_opcode == ALU_MAC) || (alu_opcode == ALU_MACR);
    assign mpy_to_round = is_mac_type ? mac_result : mpy_result;
    wire [56:0] mpyr_added =
        {1'b0, mpy_to_round} + {32'b0, rnd_rounder};
    wire mpyr_round_overflow = !mpy_to_round[55] && mpyr_added[55];
    wire mpyr_at_midpoint =
        (mpyr_added[25:0] & rnd_mask[25:0]) == 26'h0;
    wire [55:0] mpyr_masked =
        mpyr_added[55:0] & ~{30'h0, rnd_mask[25:0]};
    wire [55:0] mpyr_result = mpyr_at_midpoint ?
        (mpyr_masked & ~{30'h0, rnd_rounder[24:0], 1'b0}) : mpyr_masked;

    // ========================================================================
    // CMPM: Compare Magnitudes
    // ========================================================================
    wire [55:0] dst_abs = dst_acc[55] ? (~dst_acc + 56'h1) : dst_acc;
    wire [55:0] src_abs = src_operand[55] ? (~src_operand + 56'h1) : src_operand;
    wire [56:0] cmpm_sub = {1'b0, dst_abs} - {1'b0, src_abs};
    // Magnitudes span 0 through 2^55.  Their difference fits the signed
    // 56-bit flag result except for the positive +2^55 endpoint.
    wire        cmpm_overflow = !cmpm_sub[56] && cmpm_sub[55];

    // ========================================================================
    // Result Selection
    // ========================================================================
    reg [55:0] alu_result;
    reg        carry_out;
    reg        overflow;
    reg        update_ccr;
    reg        is_cmp_type;  // CMP/CMPM/TST: don't write result

    always @(*) begin
        alu_result = dst_acc;
        carry_out = 1'b0;
        overflow = 1'b0;
        update_ccr = 1'b1;
        is_cmp_type = 1'b0;

        case (alu_opcode)
            ALU_NOP: begin
                update_ccr = 1'b0;
            end

            ALU_ABS: begin
                alu_result = abs_result;
                overflow = (dst_acc == 56'h80_000000_000000);
            end

            ALU_ADD: begin
                alu_result = add_result;
                carry_out = add_carry;
                overflow = add_overflow;
            end

            ALU_SUB: begin
                alu_result = sub_result;
                carry_out = sub_borrow;
                overflow = sub_overflow;
            end

            ALU_ADC: begin
                alu_result = adc_result;
                carry_out = adc_carry;
                overflow = adc_overflow;
            end

            ALU_SBC: begin
                alu_result = sbc_result;
                carry_out = sbc_borrow;
                overflow = sbc_overflow;
            end

            ALU_AND: begin
                alu_result = and_result;
            end

            ALU_OR: begin
                alu_result = or_result;
            end

            ALU_EOR: begin
                alu_result = eor_result;
            end

            ALU_NOT: begin
                alu_result = not_result;
            end

            ALU_NEG: begin
                alu_result = neg_result;
                overflow = (dst_acc == 56'h80_000000_000000);
                carry_out = (dst_acc != 56'h0);
            end

            ALU_CLR: begin
                alu_result = 56'h0;
            end

            ALU_TST: begin
                alu_result = dst_acc;
                is_cmp_type = 1'b1;
            end

            ALU_CMP: begin
                alu_result = dst_acc;  // Don't write back
                carry_out = sub_borrow;
                overflow = sub_overflow;
                is_cmp_type = 1'b1;
            end

            ALU_CMPM: begin
                alu_result = dst_acc;
                carry_out = cmpm_sub[56];
                overflow = cmpm_overflow;
                is_cmp_type = 1'b1;
            end

            ALU_TFR: begin
                alu_result = src_operand;
                update_ccr = 1'b0;  // TFR doesn't update CCR
            end

            ALU_ASL: begin
                alu_result = asl_result;
                carry_out = asl_carry;
                overflow = asl_overflow;
            end

            ALU_ASR: begin
                alu_result = asr_result;
                carry_out = asr_carry;
            end

            ALU_LSL: begin
                alu_result = lsl_result;
                carry_out = lsl_carry;
            end

            ALU_LSR: begin
                alu_result = lsr_result;
                carry_out = lsr_carry;
            end

            ALU_ROL: begin
                alu_result = rol_result;
                carry_out = rol_carry;
            end

            ALU_ROR: begin
                alu_result = ror_result;
                carry_out = ror_carry;
            end

            ALU_RND: begin
                alu_result = rnd_result;
                overflow = rnd_overflow;
            end

            ALU_ADDL: begin
                alu_result = addl_result;
                carry_out = addl_carry;
                overflow = addl_overflow;
            end

            ALU_ADDR: begin
                alu_result = addr_result;
                carry_out = addr_carry;
                overflow = addr_overflow;
            end

            ALU_SUBL: begin
                alu_result = subl_result;
                carry_out = subl_borrow;
                overflow = subl_overflow;
            end

            ALU_SUBR: begin
                alu_result = subr_result;
                carry_out = subr_borrow;
                overflow = subr_overflow;
            end

            ALU_DIV: begin
                alu_result = div_result;
                carry_out = div_carry;
                overflow = div_overflow;
            end

            ALU_MPY: begin
                alu_result = mpy_result;
            end

            ALU_MPYR: begin
                alu_result = mpyr_result;
            end

            ALU_MAC: begin
                alu_result = mac_result;
                overflow = mac_overflow;
            end

            ALU_MACR: begin
                alu_result = mpyr_result;
                overflow = mac_overflow || mpyr_round_overflow;
            end

            ALU_NORM: begin
                alu_result = norm_result;
                overflow = norm_overflow;
            end

            default: begin
                update_ccr = 1'b0;
            end
        endcase
    end

    // ========================================================================
    // CCR Flag Computation
    // ========================================================================
    // For CMP/TST/CMPM, flags are computed on the comparison result
    wire [55:0] flag_src = (alu_opcode == ALU_CMP) ? sub_result :
                           (alu_opcode == ALU_CMPM) ? cmpm_sub[55:0] :
                           alu_result;

    wire logic_op = (alu_opcode == ALU_AND) || (alu_opcode == ALU_OR) ||
                    (alu_opcode == ALU_EOR) || (alu_opcode == ALU_NOT);
    wire shift24_op = (alu_opcode == ALU_LSL) || (alu_opcode == ALU_LSR) ||
                      (alu_opcode == ALU_ROL) || (alu_opcode == ALU_ROR);
    wire round_multiply_op = (alu_opcode == ALU_RND) ||
                             (alu_opcode == ALU_MPY) ||
                             (alu_opcode == ALU_MPYR) ||
                             (alu_opcode == ALU_MAC) ||
                             (alu_opcode == ALU_MACR);
    wire a1_flag_op = logic_op || shift24_op;
    wire preserve_eunz_op = (alu_opcode == ALU_DIV);
    wire preserve_c_op = logic_op || (alu_opcode == ALU_ABS) ||
                         (alu_opcode == ALU_NEG) || (alu_opcode == ALU_CLR) ||
                         (alu_opcode == ALU_TST) ||
                         (alu_opcode == ALU_NORM) || round_multiply_op;
    // Logical and 24-bit shift/rotate instructions operate only on A1/B1.
    // Their N/Z classification must therefore ignore the untouched extension
    // and low accumulator pieces, and E/U are explicitly unaffected.  Logical
    // operations preserve C, as do the unary/test and round/multiply
    // operations named by preserve_c_op; shifts and rotates use their
    // shifted bit.
    wire flag_z = preserve_eunz_op ? sr_in[CCR_Z] :
                  a1_flag_op ? (flag_src[47:24] == 24'h0) :
                               (flag_src == 56'h0);
    wire flag_n = preserve_eunz_op ? sr_in[CCR_N] :
                  a1_flag_op ? flag_src[47] : flag_src[55];
    // Full-accumulator E/U classification follows the selected scaling mode.
    // Reserved mode 11 retains the no-scaling classification rather than
    // inventing behavior outside the documented 56001 modes.
    wire [1:0] scaling_mode = sr_in[SR_S1:SR_S0];
    wire full_flag_e = (scaling_mode == 2'b01) ?
                           ((flag_src[55:48] != 8'h00) &&
                            (flag_src[55:48] != 8'hff)) :
                       (scaling_mode == 2'b10) ?
                           ((flag_src[55:46] != 10'h000) &&
                            (flag_src[55:46] != 10'h3ff)) :
                           ((flag_src[55:47] != 9'h000) &&
                            (flag_src[55:47] != 9'h1ff));
    wire full_flag_u = (scaling_mode == 2'b01) ?
                           (flag_src[48] == flag_src[47]) :
                       (scaling_mode == 2'b10) ?
                           (flag_src[46] == flag_src[45]) :
                           (flag_src[47] == flag_src[46]);
    wire flag_e = (a1_flag_op || preserve_eunz_op) ?
                      sr_in[CCR_E] : full_flag_e;
    wire flag_u = (a1_flag_op || preserve_eunz_op) ?
                      sr_in[CCR_U] : full_flag_u;
    wire flag_v = overflow;
    wire flag_l = sr_in[CCR_L] | overflow;
    wire flag_c = preserve_c_op ? sr_in[CCR_C] : carry_out;

    // ========================================================================
    // Combinational Outputs
    // ========================================================================
    assign result       = alu_result & 56'h00FFFFFFFFFFFFFF;
    assign result_valid = !is_cmp_type;
    // DSP56000/DSP56001 CCR bit 7 is reserved and reads as zero.
    assign ccr_out      = {1'b0, flag_l, flag_e, flag_u, flag_n, flag_z, flag_v, flag_c};
    assign ccr_valid    = update_ccr;

    assign norm_shift_dir = norm_did_asr;
    assign norm_did_shift = norm_did_asr || norm_did_asl;

endmodule
