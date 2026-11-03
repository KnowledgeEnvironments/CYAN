// Project CYAN <src@lispm.net>
// Copyright (C) 2026 I. Finder
// SPDX-License-Identifier: MIT

//////////////////////////////////////////////////////////////////////////////
// DSP56001 Instruction Decoder
//
// Two-level dispatch matching DSP56001 instruction encoding:
//
// Non-parallel (bits 23:20 == 0x0):
//   Primary decode from {inst[16:11], inst[7:5]} = 9-bit index
//   Covers: NOP, JMP, JSR, Jcc, JScc,
//           DO, REP, ENDDO, STOP, WAIT, RESET, SWI, RTS, RTI,
//           MOVEC, MOVEM, MOVEP, ANDI, ORI, BCLR, BSET, BCHG, BTST,
//           JCLR, JSET, JSCLR, JSSET, LUA, NORM, DIV
//
// Parallel (bits 23:20 >= 1):
//   Move decoder:  bits 23:20 → 16 parallel move types
//   ALU decoder:   bits 7:0  → 256 entries
//
// ALU opcode byte (bits 7:0) for parallel instructions (from Hatari opcodes_alu):
//   Format: bit[7]=multiply, bits[6:4]=JJJ, bit[3]=d(dest), bits[2:0]=BBB(op)
//
//   Bit 7 = 0: Arithmetic/Logic/Shift
//     JJJ=1xx (100-111): Regular two-operand (JJJ selects source: X0/Y0/X1/Y1)
//       BBB=000: ADD, 001: TFR, 010: OR, 011: EOR
//       BBB=100: SUB, 101: CMP, 110: AND, 111: CMPM
//     JJJ=000: 000=NOP, 001=TFR, 010=ADDR, 011=TST, 101=CMP, 110=SUBR, 111=CMPM
//     JJJ=001: 000=ADD, 001=RND, 010=ADDL, 011=CLR, 100=SUB, 110=SUBL, 111=NOT
//     JJJ=010: 000=ADD, 001=ADC, 010=ASR, 011=LSR, 100=SUB, 101=SBC, 110=ABS, 111=ROR
//     JJJ=011: 000=ADD, 001=ADC, 010=ASL, 011=LSL, 100=SUB, 101=SBC, 110=NEG, 111=ROL
//
//   Bit 7 = 1: Multiply — 1SSSdknn
//     SSS (bits[6:4]): source pair (8 pairs)
//     d (bit[3]): dest (0=A, 1=B)
//     k (bit[2]): negate
//     nn (bits[1:0]): 00=MPY, 01=MPYR, 10=MAC, 11=MACR
//////////////////////////////////////////////////////////////////////////////

module dsp56001_decode (
    // ========================================================================
    // Instruction Word Input
    // ========================================================================
    input  wire [23:0] inst,              // 24-bit instruction word
    input  wire [23:0] ext_word,          // Extension word (for 2-word instructions)
    input  wire        decode_en,         // Decode enable

    // ========================================================================
    // Instruction Classification
    // ========================================================================
    output wire        is_two_word,       // Needs extension word
    output wire        is_parallel,       // Parallel move + ALU
    output wire        is_nop,
    output wire        is_alu_op,
    output wire        is_move_op,
    output wire        is_branch_op,
    output wire        is_loop_op,
    output wire        is_bit_op,
    output wire        is_control_op,

    // ========================================================================
    // ALU Control Signals
    // ========================================================================
    output reg  [5:0]  alu_opcode,
    output reg         alu_dest,          // 0=A, 1=B
    output reg  [2:0]  alu_src_jjj,       // JJJ source select
    output reg  [2:0]  alu_mpy_qq,        // SSS multiply source pair (3-bit)
    output reg         alu_negate,        // Negate (MPY/MAC k-bit)
    output reg         alu_update_ccr,
    output reg         alu_no_writeback,  // CMP/CMPM/TST: no acc write

    // ========================================================================
    // Move Control Signals
    // ========================================================================
    output reg  [3:0]  move_type,         // Parallel move type
    output reg  [5:0]  move_src_reg,
    output reg  [5:0]  move_dst_reg,
    output reg         move_src_mem,      // Source is memory
    output reg         move_dst_mem,      // Destination is memory
    output reg         move_mem_space,    // 0=X, 1=Y
    output reg  [23:0] move_imm_data,     // Immediate data
    output reg         move_is_imm,
    output wire        move_addr_update,  // MOVE (Rn){+/-N,+/-}: AGU only

    // ========================================================================
    // Address Mode
    // ========================================================================
    output reg  [2:0]  addr_mode_mmm,
    output reg  [2:0]  addr_reg_rrr,
    output reg  [5:0]  addr_absolute,     // 6-bit absolute address for short abs

    // ========================================================================
    // Branch Control
    // ========================================================================
    output reg  [3:0]  branch_cond,
    output reg         branch_uncond,
    output reg         branch_push_pc,    // JSR/BSR: push return address
    output reg         branch_use_ea,     // Use effective address (AGU)
    output reg  [23:0] branch_target,

    // ========================================================================
    // Loop Control
    // ========================================================================
    output reg         loop_is_do,
    output reg         loop_is_rep,
    output reg         loop_is_enddo,

    // ========================================================================
    // Bit Operation Control
    // ========================================================================
    output reg  [1:0]  bit_op_type,       // 00=BTST, 01=BCHG, 10=BCLR, 11=BSET
    output reg  [4:0]  bit_op_bit,        // Bit number (0-23)
    output reg         bit_op_mem,        // Operates on memory (vs register)
    output reg         bit_op_space,      // 0=X, 1=Y
    output reg  [5:0]  bit_op_addr,       // 6-bit address or EA code
    output reg  [1:0]  bit_op_amode,      // 00=aa(short abs), 01=ea, 10=pp(periph), 11=reg

    // ========================================================================
    // MOVEC / MOVEP / ANDI / ORI
    // ========================================================================
    output reg         is_movec,
    output reg         is_movem,
    output reg         is_movep,
    output reg         is_andi_ori,
    output reg         andi_ori_is_and,
    output reg  [7:0]  andi_ori_imm,
    output reg  [1:0]  andi_ori_target,   // 0=MR, 1=CCR, 2=OMR

    // ========================================================================
    // Control Instructions
    // ========================================================================
    output reg         is_rts,
    output reg         is_rti,
    output reg         is_stop,
    output reg         is_wait,
    output reg         is_swi,
    output reg         is_illegal,
    output reg         is_tcc,
    output reg         tcc_dual,
    output reg  [2:0]  tcc_alu_src,
    output reg         tcc_alu_dest,
    output reg  [2:0]  tcc_r_src,
    output reg  [2:0]  tcc_r_dest,
    output reg         is_reset_inst,
    output reg         is_norm,
    output reg  [2:0]  norm_rrr,          // R register for NORM
    output reg         is_jclr,           // JCLR (bit test + conditional jump)
    output reg         is_jset,           // JSET (bit test + conditional jump)

    // ========================================================================
    // LUA (Load Update Address)
    // ========================================================================
    output reg         is_lua,
    output reg  [2:0]  lua_mmm,           // Effective-address update mode
    output reg  [2:0]  lua_rrr,
    output reg  [4:0]  lua_ddddd,         // Destination register

    // ========================================================================
    // MOVEC Register-to-Register
    // ========================================================================
    output reg         is_movec_rr        // MOVEC register-to-register variant
);

    // ========================================================================
    // ALU Opcode Definitions (must match dsp56001_alu.v)
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
    localparam [5:0] ALU_ADC   = 6'd31;
    localparam [5:0] ALU_SBC   = 6'd32;
    localparam [5:0] ALU_NORM  = 6'd33;
    localparam [5:0] ALU_SUBR  = 6'd34;

    // ========================================================================
    // Move Type Definitions
    // ========================================================================
    localparam [3:0] MOVE_NONE     = 4'd0;   // No move (NOP or standalone ALU)
    localparam [3:0] MOVE_REG      = 4'd1;   // Register-to-register
    localparam [3:0] MOVE_IMM      = 4'd2;   // Immediate to register
    localparam [3:0] MOVE_X_MEM    = 4'd3;   // X memory move
    localparam [3:0] MOVE_Y_MEM    = 4'd4;   // Y memory move
    localparam [3:0] MOVE_XY_MEM   = 4'd5;   // X:Y dual memory move
    localparam [3:0] MOVE_L_MEM    = 4'd6;   // L (long) move
    localparam [3:0] MOVE_NO_MOVE  = 4'd7;   // No parallel move (standalone)
    localparam [3:0] MOVE_IMM_SHORT = 4'd8;  // Short immediate #xx,D
    localparam [3:0] MOVE_XR_RY_CLASS1 = 4'd9; // X:R or R:Y Class-I move
    localparam [3:0] MOVE_XR_RY_CLASS2 = 4'd10; // Accumulator store + X0/Y0 load

    // ========================================================================
    // Instruction Field Extraction
    // ========================================================================
    wire [3:0] high_nibble = inst[23:20];     // Parallel move type selector
    // Class-II X:R/R:Y occupies part of the otherwise non-parallel 0x08/0x09
    // region.  Bit 14 distinguishes it from MOVEP, while MMM=110 remains a
    // reserved Class-II addressing mode rather than becoming MOVEP.
    wire       is_class2_prefix = (inst[23:17] == 7'b0000100) && !inst[14];
    wire       is_class2_valid = is_class2_prefix &&
                                 (inst[13:11] != 3'b110);
    // Class-I X:R/R:Y admits all ordinary address-register modes.  Within
    // MMM=110, only RRR=000 (long absolute) and W=1/RRR=100 (immediate)
    // are defined by the DSP56001 encoding tables.
    wire       is_class1_valid = (high_nibble == 4'h1) &&
                                 ((inst[13:11] != 3'b110) ||
                                  (inst[10:8] == 3'b000) ||
                                  (inst[15] && inst[10:8] == 3'b100));
    wire       is_parallel_move_nop =
        ((inst & 24'hffff00) == 24'h200000);
    wire       is_register_move_format =
        ((inst & 24'hfc0000) == 24'h200000) &&
        !is_parallel_move_nop && !move_addr_update;
    wire       register_move_reserved = is_register_move_format &&
        ((inst[17:13] < 5'd4) || (inst[12:8] < 5'd4));
    wire       is_l_move_format = (high_nibble == 4'h4) && !inst[18];
    wire       is_xy_memory_move_format =
        (high_nibble >= 4'h4) && (high_nibble <= 4'h7) &&
        !is_l_move_format;
    wire       memory_move_reserved = inst[14] &&
        (inst[13:11] == 3'b110) &&
        ((is_l_move_format && (inst[10:8] != 3'b000)) ||
         (is_xy_memory_move_format &&
          (inst[10:8] != 3'b000) &&
          !(inst[15] && (inst[10:8] == 3'b100))));
    wire       par_move_reserved =
        (is_class2_prefix && !is_class2_valid) ||
        ((high_nibble == 4'h1) && !is_class1_valid) ||
        register_move_reserved || memory_move_reserved;
    wire       is_non_parallel = (high_nibble == 4'h0) &&
                                 !is_class2_prefix;

    // Non-parallel: 9-bit index for dispatch
    wire [5:0] np_hi = inst[16:11];
    wire [2:0] np_lo = inst[7:5];
    wire [8:0] np_index = {np_hi, np_lo};

    // Motorola's six-bit architectural register table leaves 00--03 and
    // 28--38 unassigned.  MOVEC's five-bit control-register subset contains
    // only M0--M7 (00--07) and SR--LC (19--1f).
    wire       high_register_code_valid =
        ((inst[13:8] >= 6'h04) && (inst[13:8] <= 6'h27)) ||
        (inst[13:8] >= 6'h39);
    wire       low_register_code_valid =
        ((inst[5:0] >= 6'h04) && (inst[5:0] <= 6'h27)) ||
        (inst[5:0] >= 6'h39);
    wire       movec_control_code_valid =
        (inst[4:0] <= 5'h07) || (inst[4:0] >= 5'h19);
    wire       high_movec_control_code_valid =
        ((inst[13:8] >= 6'h20) && (inst[13:8] <= 6'h27)) ||
        (inst[13:8] >= 6'h39);
    wire       low_movec_control_code_valid =
        ((inst[5:0] >= 6'h20) && (inst[5:0] <= 6'h27)) ||
        (inst[5:0] >= 6'h39);
    wire       movec_register_valid =
        high_register_code_valid && low_register_code_valid &&
        (high_movec_control_code_valid || low_movec_control_code_valid);
    wire       movec_memory_form = !inst[7] && inst[5];
    wire       movec_immediate_form = (inst[7:5] == 3'b101);
    wire       movec_memory_selector_valid =
        !inst[14] || (inst[13:11] != 3'b110) ||
        (inst[10:8] == 3'b000) ||
        (inst[15] && (inst[10:8] == 3'b100));
    wire       movec_family_valid = movec_control_code_valid &&
        (movec_immediate_form ||
         (movec_memory_form && movec_memory_selector_valid));
    wire       movem_format_valid =
        (!inst[14] && (inst[7:6] == 2'b00)) ||
        (inst[14] && (inst[7:6] == 2'b10));
    wire       movem_selector_valid =
        !inst[14] || (inst[13:11] != 3'b110) ||
        (inst[10:8] == 3'b000);
    wire       movem_family_valid = low_register_code_valid &&
        movem_format_valid && movem_selector_valid;
    wire       movep_register_form = (inst[7:6] == 2'b00);
    wire       movep_program_form = (inst[7:6] == 2'b01);
    wire       movep_xy_form = inst[7];
    wire       movep_program_selector_valid =
        (inst[13:11] != 3'b110) || (inst[10:8] == 3'b000);
    wire       movep_xy_selector_valid =
        (inst[13:11] != 3'b110) || (inst[10:8] == 3'b000) ||
        (inst[15] && (inst[10:8] == 3'b100));
    wire       movep_family_valid = inst[14] &&
        ((movep_register_form && high_register_code_valid) ||
         (movep_program_form && movep_program_selector_valid) ||
         (movep_xy_form && movep_xy_selector_valid));

    // Parallel ALU field
    wire [7:0] par_alu = inst[7:0];
    wire       par_alu_is_mpy = par_alu[7];
    // DSP56001UMA3 tables A-29/A-30 reserve MOVE with d=1, JJJ=000
    // with kkk=100, and JJJ=001 with kkk=101.  These are illegal
    // instructions even when paired with an otherwise-valid move field.
    wire       par_alu_reserved =
        (par_alu == 8'h08) ||
        ((par_alu[7:4] == 4'h0) && (par_alu[2:0] == 3'b100)) ||
        ((par_alu[7:4] == 4'h1) && (par_alu[2:0] == 3'b101));

    // Common field extraction
    wire       field_d = inst[3];              // Destination: 0=A, 1=B
    wire [2:0] field_JJJ = inst[6:4];         // Source select (non-multiply)
    wire [1:0] field_QQ = inst[6:5];          // Multiply source pair
    wire       field_k = inst[4];             // Multiply negate
    wire [2:0] field_MMM = inst[13:11];       // Address mode
    wire [2:0] field_RRR = inst[10:8];        // Register select
    wire [3:0] field_CCCC = inst[3:0];        // Condition code
    wire [5:0] field_DDDDDD = inst[13:8];     // 6-bit register encoding

    // ========================================================================
    // Parallel/Non-Parallel Classification
    // ========================================================================
    assign is_parallel = !is_non_parallel;
    assign move_addr_update = is_parallel &&
                              ((inst & 24'hFFE000) == 24'h204000);

    // ========================================================================
    // Two-Word Instruction Detection
    // ========================================================================
    // Non-parallel instructions that need extension word:
    // - JMP/JSR with absolute address
    // - DO with loop end address
    // - MOVEC/MOVEM with absolute address
    // - MOVEP with absolute
    // - JCLR/JSET/JSCLR/JSSET with target address
    // - Bit ops with absolute addressing
    reg need_ext;
    always @(*) begin
        need_ext = 1'b0;
        if (is_non_parallel) begin
            casez (inst[23:0])
                // JMP/JSR ea fix the complete low byte at $80.  The only
                // extension-word effective-address encoding is absolute
                // MMMRRR=110000; the other 110rrr values are undefined.
                24'b0000_1010_11??_????_1000_0000:
                    need_ext = (inst[13:8] == 6'b110000);
                24'b0000_1011_11??_????_1000_0000:
                    need_ext = (inst[13:8] == 6'b110000);
                // Jcc/JScc ea fix bits 7:4 at $a and carry cc in 3:0.
                24'b0000_1010_11??_????_1010_????:
                    need_ext = (inst[13:8] == 6'b110000);
                24'b0000_1011_11??_????_1010_????:
                    need_ext = (inst[13:8] == 6'b110000);
                // JCLR/JSET/JSCLR/JSSET register forms always carry the jump
                // target in an extension word.  bbbbb=11bbb is reserved and
                // therefore has no architectural extension word.
                24'b0000_1010_11??_????_00??_????:
                    need_ext = (inst[4:3] != 2'b11);
                24'b0000_1011_11??_????_00??_????:
                    need_ext = (inst[4:3] != 2'b11);
                // JCLR/JSET and JSCLR/JSSET memory forms use modes 00, 01,
                // and 10.  Mode 11 with bit 7 set is undefined (the distinct
                // register form fixes bit 7 at zero).
                24'b0000_1010_????_????_1???_????:
                    need_ext = (inst[15:14] != 2'b11) &&
                               (inst[4:3] != 2'b11) &&
                               !(inst[15:14] == 2'b01 &&
                                 inst[13:11] == 3'b110);
                24'b0000_1011_????_????_1???_????:
                    need_ext = (inst[15:14] != 2'b11) &&
                               (inst[4:3] != 2'b11) &&
                               !(inst[15:14] == 2'b01 &&
                                 inst[13:11] == 3'b110);
                // BCLR/BSET ea (0x0A, bit7=0, inst[15:14]=01): 2-word if MMM=110
                24'b0000_1010_01??_????_0???_????:
                    need_ext = (inst[4:3] != 2'b11) &&
                               (inst[13:11] == 3'b110);
                // BCHG/BTST ea (0x0B, bit7=0, inst[15:14]=01): same
                24'b0000_1011_01??_????_0???_????:
                    need_ext = (inst[4:3] != 2'b11) &&
                               (inst[13:11] == 3'b110);
                // DO X/Y:ea,expr. MMM=110 is reserved for DO.
                24'b0000_0110_01??_????_0?00_0000:
                    need_ext = (inst[13:11] != 3'b110);
                // DO X/Y:aa,expr
                24'b0000_0110_00??_????_0?00_0000:
                    need_ext = 1'b1;
                // DO #xxx,expr
                24'b0000_0110_????_????_1000_????:
                    need_ext = 1'b1;
                // DO S,expr.  SSH is illegal, as are the reserved register
                // source encodings.
                24'b0000_0110_11??_????_0000_0000:
                    need_ext = ((inst[13:8] >= 6'h04 &&
                                 inst[13:8] <= 6'h27) ||
                                (inst[13:8] >= 6'h39 &&
                                 inst[13:8] != 6'h3c));
                // MOVEM: only the EA form has MMM bits. Short absolute
                // address bits must not consume an instruction extension.
                24'b0000_0111_????_????_????_????:
                    need_ext = movem_family_valid && inst[14] &&
                               (inst[13:11] == 3'b110);
                // MOVEC short immediate is always one word:
                //   00000101 iiiiiiii 101ddddd
                // For the X/Y effective-address form, bit 14 selects EA over
                // six-bit absolute-short and MMM=110 consumes the extension.
                // Immediate bits must never be interpreted as MMM/form bits.
                24'b0000_0101_????_????_????_????:
                    need_ext = movec_family_valid && movec_memory_form &&
                               inst[14] &&
                               (inst[13:11] == 3'b110);
                // MOVEP
                24'b0000_100?_????_????_????_????:
                    need_ext = movep_family_valid &&
                               (inst[13:11] == 3'b110);
                default: need_ext = 1'b0;
            endcase
        end else if (is_class2_prefix) begin
            // All legal Class-II forms are one word.  MMM=110 is reserved and
            // must not consume the following program word as an extension.
            need_ext = 1'b0;
        end else begin
            // X/Y memory data moves use bit 14 to distinguish the six-bit
            // absolute-short form from the MMMRRR effective-address form.
            // Only the latter consumes an extension for MMM=110 (absolute
            // address or immediate data).  Other parallel move classes retain
            // their existing extension-word rule.
            if (high_nibble == 4'h1)
                need_ext = is_class1_valid &&
                           (inst[13:11] == 3'b110);
            else if (high_nibble == 4'h2 || high_nibble == 4'h3)
                // Register, short-immediate and address-update parallel
                // moves are all one word.  In these formats bits 13:11 are
                // register/immediate payload, not an effective-address mode.
                need_ext = 1'b0;
            else if (high_nibble >= 4'h4 && high_nibble <= 4'h7)
                need_ext = !memory_move_reserved && inst[14] &&
                           (inst[13:11] == 3'b110);
            else if (high_nibble >= 4'h8)
                // X:Y carries two restricted AGU fields entirely in the
                // instruction word.  Its {Y-rr,X-mm} payload may equal 110,
                // but that is not the ordinary MMM absolute-address form.
                need_ext = 1'b0;
            else if (inst[13:11] == 3'b110)
                need_ext = 1'b1;
        end
    end
    assign is_two_word = need_ext;

    // ========================================================================
    // Parallel ALU Decoder (bits 7:0)
    // ========================================================================
    reg [5:0]  par_alu_opcode;
    reg        par_alu_dest;
    reg [2:0]  par_alu_jjj;
    reg [2:0]  par_alu_qq;
    reg        par_alu_neg;
    reg        par_alu_is_alu;
    reg        par_alu_no_wb;

    always @(*) begin
        par_alu_opcode = ALU_NOP;
        par_alu_dest   = par_alu[3];  // bit 3 = d (0=A, 1=B)
        par_alu_jjj    = par_alu[6:4];
        par_alu_qq     = par_alu[6:4];
        par_alu_neg    = 1'b0;
        par_alu_is_alu = 1'b0;
        par_alu_no_wb  = 1'b0;

        if (par_alu[7]) begin
            // ================================================================
            // Multiply group: 1SSSdknn
            //   SSS=bits[6:4]: source pair, d=bit[3]: dest, k=bit[2]: negate
            //   nn=bits[1:0]: 00=MPY, 01=MPYR, 10=MAC, 11=MACR
            // ================================================================
            par_alu_is_alu = 1'b1;
            par_alu_dest = par_alu[3];
            par_alu_neg  = par_alu[2];
            par_alu_qq   = par_alu[6:4];
            case (par_alu[1:0])
                2'b00: par_alu_opcode = ALU_MPY;
                2'b01: par_alu_opcode = ALU_MPYR;
                2'b10: par_alu_opcode = ALU_MAC;
                2'b11: par_alu_opcode = ALU_MACR;
            endcase
        end else if (par_alu[6]) begin
            // ================================================================
            // JJJ = 1xx (100-111): Regular two-operand pattern
            //   JJJ selects source: 100=X0, 101=Y0, 110=X1, 111=Y1
            //   BBB selects operation
            // ================================================================
            par_alu_is_alu = 1'b1;
            case (par_alu[2:0])
                3'b000: par_alu_opcode = ALU_ADD;
                3'b001: par_alu_opcode = ALU_TFR;
                3'b010: par_alu_opcode = ALU_OR;
                3'b011: par_alu_opcode = ALU_EOR;
                3'b100: par_alu_opcode = ALU_SUB;
                3'b101: begin par_alu_opcode = ALU_CMP;  par_alu_no_wb = 1'b1; end
                3'b110: par_alu_opcode = ALU_AND;
                3'b111: begin par_alu_opcode = ALU_CMPM; par_alu_no_wb = 1'b1; end
            endcase
        end else begin
            // ================================================================
            // JJJ = 0xx (000-011): Special operations
            //   Full decode of {JJJ[1:0], BBB} determines operation
            //   Source for ADD/SUB/CMP/CMPM: opposite acc (JJJ=000/001),
            //     X={X1,X0} (JJJ=010), Y={Y1,Y0} (JJJ=011)
            // ================================================================
            case ({par_alu[5:4], par_alu[2:0]})
                // JJJ=000: single-operand and opposite-acc operations
                5'b00_000: begin /* MOVE/NOP — no ALU op */ end
                5'b00_001: begin par_alu_opcode = ALU_TFR;  par_alu_is_alu = 1'b1; par_alu_jjj = 3'b000; end
                5'b00_010: begin par_alu_opcode = ALU_ADDR; par_alu_is_alu = 1'b1; par_alu_jjj = 3'b000; end
                5'b00_011: begin par_alu_opcode = ALU_TST;  par_alu_is_alu = 1'b1; par_alu_no_wb = 1'b1; end
                5'b00_100: begin /* undefined */ end
                5'b00_101: begin par_alu_opcode = ALU_CMP;  par_alu_is_alu = 1'b1; par_alu_jjj = 3'b000; par_alu_no_wb = 1'b1; end
                5'b00_110: begin par_alu_opcode = ALU_SUBR; par_alu_is_alu = 1'b1; par_alu_jjj = 3'b000; end
                5'b00_111: begin par_alu_opcode = ALU_CMPM; par_alu_is_alu = 1'b1; par_alu_jjj = 3'b000; par_alu_no_wb = 1'b1; end

                // JJJ=001: opposite-acc ADD/SUB + single-operand
                5'b01_000: begin par_alu_opcode = ALU_ADD;  par_alu_is_alu = 1'b1; par_alu_jjj = 3'b001; end
                5'b01_001: begin par_alu_opcode = ALU_RND;  par_alu_is_alu = 1'b1; end
                5'b01_010: begin par_alu_opcode = ALU_ADDL; par_alu_is_alu = 1'b1; par_alu_jjj = 3'b001; end
                5'b01_011: begin par_alu_opcode = ALU_CLR;  par_alu_is_alu = 1'b1; end
                5'b01_100: begin par_alu_opcode = ALU_SUB;  par_alu_is_alu = 1'b1; par_alu_jjj = 3'b001; end
                5'b01_101: begin /* undefined */ end
                5'b01_110: begin par_alu_opcode = ALU_SUBL; par_alu_is_alu = 1'b1; par_alu_jjj = 3'b001; end
                5'b01_111: begin par_alu_opcode = ALU_NOT;  par_alu_is_alu = 1'b1; end

                // JJJ=010: X-source ADD/SUB + shifts + ABS/ROR
                5'b10_000: begin par_alu_opcode = ALU_ADD;  par_alu_is_alu = 1'b1; par_alu_jjj = 3'b010; end
                5'b10_001: begin par_alu_opcode = ALU_ADC;  par_alu_is_alu = 1'b1; par_alu_jjj = 3'b010; end
                5'b10_010: begin par_alu_opcode = ALU_ASR;  par_alu_is_alu = 1'b1; end
                5'b10_011: begin par_alu_opcode = ALU_LSR;  par_alu_is_alu = 1'b1; end
                5'b10_100: begin par_alu_opcode = ALU_SUB;  par_alu_is_alu = 1'b1; par_alu_jjj = 3'b010; end
                5'b10_101: begin par_alu_opcode = ALU_SBC;  par_alu_is_alu = 1'b1; par_alu_jjj = 3'b010; end
                5'b10_110: begin par_alu_opcode = ALU_ABS;  par_alu_is_alu = 1'b1; end
                5'b10_111: begin par_alu_opcode = ALU_ROR;  par_alu_is_alu = 1'b1; end

                // JJJ=011: Y-source ADD/SUB + shifts + NEG/ROL
                5'b11_000: begin par_alu_opcode = ALU_ADD;  par_alu_is_alu = 1'b1; par_alu_jjj = 3'b011; end
                5'b11_001: begin par_alu_opcode = ALU_ADC;  par_alu_is_alu = 1'b1; par_alu_jjj = 3'b011; end
                5'b11_010: begin par_alu_opcode = ALU_ASL;  par_alu_is_alu = 1'b1; end
                5'b11_011: begin par_alu_opcode = ALU_LSL;  par_alu_is_alu = 1'b1; end
                5'b11_100: begin par_alu_opcode = ALU_SUB;  par_alu_is_alu = 1'b1; par_alu_jjj = 3'b011; end
                5'b11_101: begin par_alu_opcode = ALU_SBC;  par_alu_is_alu = 1'b1; par_alu_jjj = 3'b011; end
                5'b11_110: begin par_alu_opcode = ALU_NEG;  par_alu_is_alu = 1'b1; end
                5'b11_111: begin par_alu_opcode = ALU_ROL;  par_alu_is_alu = 1'b1; end
            endcase
        end
    end

    // ========================================================================
    // Parallel Move Decoder (bits 23:20)
    // ========================================================================
    reg [3:0]  par_move_type;
    reg [5:0]  par_move_src_reg;
    reg [5:0]  par_move_dst_reg;
    reg        par_move_src_mem;
    reg        par_move_dst_mem;
    reg        par_move_space;

    always @(*) begin
        par_move_type    = MOVE_NONE;
        par_move_src_reg = 6'h0;
        par_move_dst_reg = 6'h0;
        par_move_src_mem = 1'b0;
        par_move_dst_mem = 1'b0;
        par_move_space   = 1'b0;

        case (high_nibble)
            4'h0: begin
                if (is_class2_valid) begin
                    // Class-II X:R/R:Y stores the selected accumulator through
                    // the data-bus limiter and independently loads X0/Y0 into
                    // that same accumulator.  The core performs the second
                    // transfer after sampling the memory-store source.
                    par_move_type = MOVE_XR_RY_CLASS2;
                    par_move_src_reg = inst[16] ? 6'h0f : 6'h0e;
                    par_move_dst_mem = 1'b1;
                    par_move_space = inst[15];
                end else begin
                    // Reserved Class-II MMM=110 is deliberately side-effect
                    // free at the move boundary.
                    par_move_type = MOVE_NONE;
                end
            end
            4'h1: begin
                // Class-I X:R (bit 14 clear) and R:Y (bit 14 set) admit all
                // seven Rn modes plus the two-word MMMRRR=110000 absolute-
                // memory and 110100 immediate-load forms.  W selects the
                // primary memory direction.  The independent register half
                // is A/B -> Y0/Y1 for X:R and A/B -> X0/X1 for R:Y.
                if (is_class1_valid) begin
                    par_move_type = MOVE_XR_RY_CLASS1;
                    par_move_space = inst[14];
                    if (!inst[14]) begin
                        case (inst[19:18])
                            2'b00: par_move_src_reg = 6'h04; // X0
                            2'b01: par_move_src_reg = 6'h05; // X1
                            2'b10: par_move_src_reg = 6'h0e; // A
                            2'b11: par_move_src_reg = 6'h0f; // B
                        endcase
                    end else begin
                        case (inst[17:16])
                            2'b00: par_move_src_reg = 6'h06; // Y0
                            2'b01: par_move_src_reg = 6'h07; // Y1
                            2'b10: par_move_src_reg = 6'h0e; // A
                            2'b11: par_move_src_reg = 6'h0f; // B
                        endcase
                    end
                    if (inst[15]) begin
                        par_move_dst_reg = par_move_src_reg;
                        if (inst[13:8] != 6'b110100)
                            par_move_src_mem = 1'b1;
                    end else if (!inst[15]) begin
                        par_move_dst_mem = 1'b1;
                    end
                end else begin
                    par_move_type = MOVE_NONE;
                end
            end
            4'h2, 4'h3: begin
                if (is_parallel_move_nop) begin
                    // 0010 0000 0000 0000: no parallel data move.  The low
                    // byte remains the independently decoded ALU operation.
                    par_move_type = MOVE_NONE;
                end else if (move_addr_update) begin
                    // 0010 0000 010m mrrr: address-register update with
                    // no data transfer.  The low byte still carries the
                    // parallel ALU operation (zero for plain MOVE (Rn)+).
                    par_move_type = MOVE_NONE;
                end else if ((inst & 24'hFC0000) == 24'h200000) begin
                    // 0010 00ee eeed dddd: register-to-register transfer.
                    // Five-bit codes 0--3 are reserved; the overlapping NOP
                    // and U formats were handled above.
                    if (!register_move_reserved) begin
                        par_move_type = MOVE_REG;
                        par_move_src_reg = {1'b0, inst[17:13]};
                        par_move_dst_reg = {1'b0, inst[12:8]};
                    end
                end else begin
                    // Short immediate: #xx,D
                    // inst[20:16] = 5-bit destination register
                    // inst[15:8] = 8-bit immediate data
                    par_move_type = MOVE_IMM_SHORT;
                    par_move_dst_reg = {1'b0, inst[20:16]};
                end
            end
            4'h4, 4'h5, 4'h6, 4'h7: begin
                if (is_l_move_format) begin
                    // L: long-memory move overlaps the otherwise-reserved
                    // X-move register encodings 000xx:
                    //   0100 L0LL W1MMMRRR / W0aaaaaa
                    // It transfers both X and Y memory at one effective
                    // address.  LLL is consumed by the core as a 48-bit
                    // register-pair selector.
                    if (!memory_move_reserved) begin
                        par_move_type = MOVE_L_MEM;
                        if (inst[15])
                            par_move_src_mem = 1'b1;
                        else
                            par_move_dst_mem = 1'b1;
                    end
                end else begin
                    // X or Y memory data move.  inst[19] selects space and
                    // the split five-bit data-register code retains program
                    // bit order {inst[21:20], inst[18:16]}.
                    if (!memory_move_reserved) begin
                        par_move_space = inst[19];
                        par_move_type = inst[19] ? MOVE_Y_MEM : MOVE_X_MEM;
                        if (inst[15]) begin
                            // W=1: read direction (memory/immediate -> register)
                            par_move_dst_reg = {1'b0, inst[21:20], inst[18:16]};
                            if (inst[14] && inst[13:8] == 6'b110100) begin
                                // EA=110100: immediate data from ext_word.  EA=110000
                                // is instead an absolute memory read through ext_word.
                                par_move_type = MOVE_IMM;
                            end else begin
                                par_move_src_mem = 1'b1;
                            end
                        end else begin
                            // W=0: write direction (register -> memory)
                            par_move_dst_mem = 1'b1;
                            par_move_src_reg = {1'b0, inst[21:20], inst[18:16]};
                        end
                    end
                end
            end
            4'h8, 4'h9, 4'hA, 4'hB,
            4'hC, 4'hD, 4'hE, 4'hF: begin
                // X:Y dual-memory move.  Bit 22 is the X-side direction,
                // so X-memory reads occupy 8--B and writes occupy C--F.
                // L: moves use the distinct 0100L0LL format above.
                par_move_type = MOVE_XY_MEM;
            end
        endcase
    end

    // ========================================================================
    // Non-Parallel Instruction Decoder
    // ========================================================================
    reg [5:0]  np_alu_opcode;
    reg        np_alu_dest;
    reg [2:0]  np_alu_jjj;
    reg        np_is_alu;
    reg        np_is_move;
    reg        np_is_branch;
    reg        np_is_loop;
    reg        np_is_bit;
    reg        np_is_control;

    always @(*) begin
        // Default all outputs
        np_alu_opcode = ALU_NOP;
        np_alu_dest   = 1'b0;
        np_alu_jjj    = 3'b000;
        np_is_alu     = 1'b0;
        np_is_move    = 1'b0;
        np_is_branch  = 1'b0;
        np_is_loop    = 1'b0;
        np_is_bit     = 1'b0;
        np_is_control = 1'b0;

        // Reset per-instruction fields
        is_rts       = 1'b0;
        is_rti       = 1'b0;
        is_stop      = 1'b0;
        is_wait      = 1'b0;
        is_swi       = 1'b0;
        is_illegal   = is_parallel &&
                       (par_alu_reserved || par_move_reserved);
        is_tcc       = 1'b0;
        tcc_dual     = 1'b0;
        tcc_alu_src  = 3'b0;
        tcc_alu_dest = 1'b0;
        tcc_r_src    = 3'b0;
        tcc_r_dest   = 3'b0;
        is_reset_inst = 1'b0;
        is_movec     = 1'b0;
        is_movec_rr  = 1'b0;
        is_movem     = 1'b0;
        is_movep     = 1'b0;
        is_andi_ori  = 1'b0;
        andi_ori_is_and = 1'b0;
        andi_ori_imm = 8'h0;
        andi_ori_target = 2'b0;
        is_norm      = 1'b0;
        norm_rrr     = 3'b0;
        is_jclr      = 1'b0;
        is_jset      = 1'b0;
        is_lua       = 1'b0;
        lua_mmm      = 3'b0;
        lua_rrr      = 3'b0;
        lua_ddddd    = 5'b0;
        loop_is_do   = 1'b0;
        loop_is_rep  = 1'b0;
        loop_is_enddo = 1'b0;
        branch_cond  = 4'h0;
        branch_uncond = 1'b0;
        branch_push_pc = 1'b0;
        branch_use_ea = 1'b0;
        branch_target = 24'h0;
        bit_op_type  = 2'b0;
        bit_op_bit   = 5'b0;
        bit_op_mem   = 1'b0;
        bit_op_space = 1'b0;
        bit_op_addr  = 6'h0;
        bit_op_amode = 2'b0;

        if (is_non_parallel) begin
            casez (inst[23:0])
                // ============================================================
                // Simple control instructions (special opcodes)
                // ============================================================
                24'h000000: begin
                    // NOP
                    np_is_control = 1'b1;
                end
                24'h00000C: begin
                    // RTS
                    np_is_control = 1'b1;
                    is_rts = 1'b1;
                end
                24'h000004: begin
                    // RTI
                    np_is_control = 1'b1;
                    is_rti = 1'b1;
                end
                24'h000087: begin
                    // STOP
                    np_is_control = 1'b1;
                    is_stop = 1'b1;
                end
                24'h000086: begin
                    // WAIT
                    np_is_control = 1'b1;
                    is_wait = 1'b1;
                end
                24'h000006: begin
                    // SWI
                    np_is_control = 1'b1;
                    is_swi = 1'b1;
                end
                24'h000005: begin
                    // ILLEGAL: architectural nonmaskable IPL-3 exception.
                    np_is_control = 1'b1;
                    is_illegal = 1'b1;
                end
                24'h000084: begin
                    // RESET
                    np_is_control = 1'b1;
                    is_reset_inst = 1'b1;
                end
                24'h00008C: begin
                    // ENDDO
                    np_is_loop = 1'b1;
                    loop_is_enddo = 1'b1;
                end

                // ============================================================
                // One-word 12-bit absolute flow forms (UMA3 A-244).
                // The low twelve bits are a zero-extended program address;
                // unlike the effective-address family these forms consume no
                // extension word and never invoke the AGU.
                // ============================================================
                24'b0000_1100_0000_????_????_????: begin
                    // JMP xxx
                    np_is_branch = 1'b1;
                    branch_uncond = 1'b1;
                    branch_target = {12'h000, inst[11:0]};
                end
                24'b0000_1101_0000_????_????_????: begin
                    // JSR xxx
                    np_is_branch = 1'b1;
                    branch_uncond = 1'b1;
                    branch_push_pc = 1'b1;
                    branch_target = {12'h000, inst[11:0]};
                end
                24'b0000_1110_????_????_????_????: begin
                    // Jcc xxx
                    np_is_branch = 1'b1;
                    branch_cond = inst[15:12];
                    branch_target = {12'h000, inst[11:0]};
                end
                24'b0000_1111_????_????_????_????: begin
                    // JScc xxx
                    np_is_branch = 1'b1;
                    branch_cond = inst[15:12];
                    branch_push_pc = 1'b1;
                    branch_target = {12'h000, inst[11:0]};
                end

                // ============================================================
                // Tcc conditional transfers
                //
                //   00000010 CCCC 00000 JJJD 000       S1 -> D1
                //   00000011 CCCC 0ttt 0JJJ DTTT       plus Rt -> RT
                //
                // Only JJJ=000 (opposite accumulator) and JJJ=100..111
                // (X0,Y0,X1,Y1) are defined by the DSP56001 register table.
                // ============================================================
                24'b0000_0010_????_0000_0???_?000: begin
                    np_is_move = 1'b1;
                    if (inst[6:4] == 3'b000 || inst[6]) begin
                        is_tcc = 1'b1;
                        branch_cond = inst[15:12];
                        tcc_alu_src = inst[6:4];
                        tcc_alu_dest = inst[3];
                    end else begin
                        is_illegal = 1'b1;
                    end
                end
                24'b0000_0011_????_0???_0???_????: begin
                    np_is_move = 1'b1;
                    if (inst[6:4] == 3'b000 || inst[6]) begin
                        is_tcc = 1'b1;
                        tcc_dual = 1'b1;
                        branch_cond = inst[15:12];
                        tcc_alu_src = inst[6:4];
                        tcc_alu_dest = inst[3];
                        tcc_r_src = inst[10:8];
                        tcc_r_dest = inst[2:0];
                    end else begin
                        is_illegal = 1'b1;
                    end
                end

                // ============================================================
                // ANDI/ORI #xx,D (immediate to control register)
                // ANDI: 00000000 iiiiiiii 101110EE
                // ORI:  00000000 iiiiiiii 111110EE
                // ============================================================
                24'b0000_0000_????_????_1111_10??: begin
                    np_is_control = 1'b1;
                    if (inst[1:0] != 2'b11) begin
                        is_andi_ori = 1'b1;
                        andi_ori_is_and = 1'b0;  // ORI
                        andi_ori_imm = inst[15:8];
                        // 00=MR, 01=CCR, 10=OMR; 11 is reserved.
                        andi_ori_target = inst[1:0];
                    end else begin
                        is_illegal = 1'b1;
                    end
                end
                24'b0000_0000_????_????_1011_10??: begin
                    np_is_control = 1'b1;
                    if (inst[1:0] != 2'b11) begin
                        is_andi_ori = 1'b1;
                        andi_ori_is_and = 1'b1;  // ANDI
                        andi_ori_imm = inst[15:8];
                        andi_ori_target = inst[1:0];
                    end else begin
                        is_illegal = 1'b1;
                    end
                end

                // ============================================================
                // 0x0A/0x0B group: JMP/JSR, Jcc/JScc, BCLR/BSET, JCLR/JSET
                //
                // Dispatch based on {inst[15:14], inst[7:5]}:
                //   inst[15:14]=11:
                //     inst[7:5]=100 → JMP/JSR ea
                //     inst[7:5]=101 → Jcc/JScc ea
                //     inst[7:5]=0xx → JCLR/JSET/BCLR/BSET reg
                //   inst[15:14]={00,01,10} (aa/ea/pp):
                //     inst[7]=1 → JCLR/JSET
                //     inst[7]=0 → BCLR/BSET
                //   inst[5] selects: 0=clear, 1=set
                //   inst[6] = memory space (0=X, 1=Y)
                //   inst[4:0] = bit number
                //   inst[13:8] = address (aa/pp) or EA code (ea)
                // ============================================================

                // JMP ea: inst[15:14]=11, inst[7:0]=10000000
                24'b0000_1010_11??_????_1000_0000: begin
                    if (inst[13:11] != 3'b110 ||
                        inst[13:8] == 6'b110000) begin
                        np_is_branch = 1'b1;
                        branch_uncond = 1'b1;
                        branch_target = ext_word;
                        if (inst[13:11] != 3'b110)
                            branch_use_ea = 1'b1;
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end
                // JSR ea: inst[15:14]=11, inst[7:0]=10000000
                24'b0000_1011_11??_????_1000_0000: begin
                    if (inst[13:11] != 3'b110 ||
                        inst[13:8] == 6'b110000) begin
                        np_is_branch = 1'b1;
                        branch_uncond = 1'b1;
                        branch_push_pc = 1'b1;
                        branch_target = ext_word;
                        if (inst[13:11] != 3'b110)
                            branch_use_ea = 1'b1;
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end

                // Jcc ea: inst[15:14]=11, inst[7:4]=1010, cc=inst[3:0]
                24'b0000_1010_11??_????_1010_????: begin
                    if (inst[13:11] != 3'b110 ||
                        inst[13:8] == 6'b110000) begin
                        np_is_branch = 1'b1;
                        branch_cond = inst[3:0];
                        branch_target = ext_word;
                        if (inst[13:11] != 3'b110)
                            branch_use_ea = 1'b1;
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end
                // JScc ea: inst[15:14]=11, inst[7:4]=1010
                24'b0000_1011_11??_????_1010_????: begin
                    if (inst[13:11] != 3'b110 ||
                        inst[13:8] == 6'b110000) begin
                        np_is_branch = 1'b1;
                        branch_cond = inst[3:0];
                        branch_push_pc = 1'b1;
                        branch_target = ext_word;
                        if (inst[13:11] != 3'b110)
                            branch_use_ea = 1'b1;
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end

                // Register-source JCLR/JSET.  Unlike the memory forms below,
                // bit 7 is zero and inst[13:8] names an architectural source
                // register.  Decode these before the generic BCLR/BSET case.
                24'b0000_1010_11??_????_00??_????: begin
                    if (inst[4:3] != 2'b11 &&
                        ((inst[13:8] >= 6'h04 && inst[13:8] <= 6'h27) ||
                         (inst[13:8] >= 6'h39 && inst[13:8] <= 6'h3f))) begin
                        np_is_bit = 1'b1;
                        if (inst[5]) begin
                            is_jset = 1'b1;
                        end else begin
                            is_jclr = 1'b1;
                        end
                        bit_op_bit = inst[4:0];
                        bit_op_mem = 1'b0;
                        bit_op_addr = inst[13:8];
                        branch_target = ext_word;
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end
                // Register-source JSCLR/JSSET.  The 0x0B family identifies
                // the conditional call; branch_push_pc requests a stack push
                // only when the selected source bit takes the branch.
                24'b0000_1011_11??_????_00??_????: begin
                    if (inst[4:3] != 2'b11 &&
                        ((inst[13:8] >= 6'h04 && inst[13:8] <= 6'h27) ||
                         (inst[13:8] >= 6'h39 && inst[13:8] <= 6'h3f))) begin
                        np_is_bit = 1'b1;
                        branch_push_pc = 1'b1;
                        if (inst[5]) begin
                            is_jset = 1'b1;
                        end else begin
                            is_jclr = 1'b1;
                        end
                        bit_op_bit = inst[4:0];
                        bit_op_mem = 1'b0;
                        bit_op_addr = inst[13:8];
                        branch_target = ext_word;
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end

                // Admitted register-destination bit operations: direct X/Y,
                // accumulator pieces/full accumulators, zero-extended
                // 16-bit R/N/M, and the assigned controller fields below.
                // Reserved controller bits continue to fail closed.
                24'b0000_1010_11??_????_01??_????: begin
                    if (inst[4:3] != 2'b11 &&
                        ((inst[13:8] >= 6'h04 && inst[13:8] <= 6'h0f) ||
                        (inst[13:8] >= 6'h10 && inst[13:8] <= 6'h27) ||
                        (inst[13:8] == 6'h29 &&
                         (inst[4:0] <= 5'd6 ||
                          (inst[4:0] >= 5'd8 && inst[4:0] <= 5'd11) ||
                          inst[4:0] == 5'd13 || inst[4:0] == 5'd15)) ||
                        inst[13:8] == 6'h2a ||
                        (inst[13:8] == 6'h2b && inst[4:0] <= 5'd6) ||
                        (inst[13:8] == 6'h2c && inst[4:0] <= 5'd23) ||
                         (inst[13:8] >= 6'h2d && inst[13:8] <= 6'h2f))) begin
                        np_is_bit = 1'b1;
                        bit_op_type = {1'b1, inst[5]}; // 10=BCLR, 11=BSET
                        bit_op_bit = inst[4:0];
                        bit_op_mem = 1'b0;
                        bit_op_addr = inst[13:8];
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end
                24'b0000_1011_11??_????_01??_????: begin
                    if (inst[4:3] != 2'b11 &&
                        ((inst[13:8] >= 6'h04 && inst[13:8] <= 6'h0f) ||
                        (inst[13:8] >= 6'h10 && inst[13:8] <= 6'h27) ||
                        (inst[13:8] == 6'h29 &&
                         (inst[4:0] <= 5'd6 ||
                          (inst[4:0] >= 5'd8 && inst[4:0] <= 5'd11) ||
                          inst[4:0] == 5'd13 || inst[4:0] == 5'd15)) ||
                        inst[13:8] == 6'h2a ||
                        (inst[13:8] == 6'h2b && inst[4:0] <= 5'd6) ||
                        (inst[13:8] == 6'h2c && inst[4:0] <= 5'd23) ||
                         (inst[13:8] >= 6'h2d && inst[13:8] <= 6'h2f))) begin
                        np_is_bit = 1'b1;
                        bit_op_type = {1'b0, ~inst[5]}; // 01=BCHG, 00=BTST
                        bit_op_bit = inst[4:0];
                        bit_op_mem = 1'b0;
                        bit_op_addr = inst[13:8];
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end

                // JCLR/JSET: 0x0A with bit 7=1 (aa/ea/pp forms)
                // inst[15:14] determines addressing: 00=aa, 01=ea, 10=pp;
                // the distinct bit-7-zero register forms are handled above.
                // Extension word = jump target
                24'b0000_1010_????_????_1???_????: begin
                    if (inst[15:14] != 2'b11 &&
                        inst[4:3] != 2'b11 &&
                        !(inst[15:14] == 2'b01 &&
                          inst[13:11] == 3'b110)) begin
                        np_is_bit = 1'b1;
                        if (inst[5]) begin
                            is_jset = 1'b1;
                        end else begin
                            is_jclr = 1'b1;
                        end
                        bit_op_bit   = inst[4:0];
                        bit_op_space = inst[6];
                        bit_op_mem   = 1'b1;
                        bit_op_addr  = inst[13:8];
                        bit_op_amode = inst[15:14];
                        branch_target = ext_word;
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end
                // JSCLR/JSSET: 0x0B with bit 7=1
                24'b0000_1011_????_????_1???_????: begin
                    if (inst[15:14] != 2'b11 &&
                        inst[4:3] != 2'b11 &&
                        !(inst[15:14] == 2'b01 &&
                          inst[13:11] == 3'b110)) begin
                        np_is_bit = 1'b1;
                        // This is a bit-conditioned subroutine, not a generic
                        // condition-code branch.  branch_push_pc identifies
                        // the call side effect; the tested source bit alone
                        // decides whether it is taken.
                        branch_push_pc = 1'b1;
                        if (inst[5]) begin
                            is_jset = 1'b1;
                        end else begin
                            is_jclr = 1'b1;
                        end
                        bit_op_bit   = inst[4:0];
                        bit_op_space = inst[6];
                        bit_op_mem   = 1'b1;
                        bit_op_addr  = inst[13:8];
                        bit_op_amode = inst[15:14];
                        branch_target = ext_word;
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end

                // BCLR/BSET: 0x0A with bit 7=0 (aa/ea/pp forms)
                // inst[5]=0: BCLR, inst[5]=1: BSET
                24'b0000_1010_????_????_0???_????: begin
                    if (inst[4:3] != 2'b11) begin
                        np_is_bit = 1'b1;
                        bit_op_type  = {1'b1, inst[5]};  // 10=BCLR, 11=BSET
                        bit_op_bit   = inst[4:0];
                        bit_op_space = inst[6];
                        bit_op_mem   = 1'b1;
                        bit_op_addr  = inst[13:8];
                        bit_op_amode = inst[15:14];
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end
                // BCHG/BTST: 0x0B with bit 7=0
                // inst[5]=0: BCHG, inst[5]=1: BTST
                24'b0000_1011_????_????_0???_????: begin
                    if (inst[4:3] != 2'b11) begin
                        np_is_bit = 1'b1;
                        bit_op_type  = {1'b0, ~inst[5]}; // 00=BTST, 01=BCHG
                        bit_op_bit   = inst[4:0];
                        bit_op_space = inst[6];
                        bit_op_mem   = 1'b1;
                        bit_op_addr  = inst[13:8];
                        bit_op_amode = inst[15:14];
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end

                // ============================================================
                // DO (hardware loop).  All forms consume an extension word
                // containing the assembler-adjusted last loop address.
                //
                //   S: 0000 0110 11dd dddd 0000 0000
                // #xxx:0000 0110 iiii iiii 1000 hhhh
                // X/Y:ea: 0000 0110 01MM MRRR 0s00 0000
                // X/Y:aa: 0000 0110 00aa aaaa 0s00 0000
                // ============================================================
                24'b0000_0110_11??_????_0000_0000: begin
                    if ((inst[13:8] >= 6'h04 && inst[13:8] <= 6'h27) ||
                        (inst[13:8] >= 6'h39 && inst[13:8] != 6'h3c)) begin
                        np_is_loop = 1'b1;
                        loop_is_do = 1'b1;
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end
                24'b0000_0110_????_????_1000_????: begin
                    np_is_loop = 1'b1;
                    loop_is_do = 1'b1;
                end
                24'b0000_0110_01??_????_0?00_0000: begin
                    if (inst[13:11] != 3'b110) begin
                        np_is_loop = 1'b1;
                        loop_is_do = 1'b1;
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end
                24'b0000_0110_00??_????_0?00_0000: begin
                    np_is_loop = 1'b1;
                    loop_is_do = 1'b1;
                end

                // ============================================================
                // REP (repeat single instruction)
                //
                // REP shares the 0x06xxxx opcode family with DO.  Motorola's
                // model-specific encodings distinguish the forms in the low
                // byte; matching the entire 0x06 family turns 06CF00
                // (DO B,expr) into REP and corrupts program flow.
                //
                //   #xxx: 0000 0110 iiii iiii 1010 hhhh
                //      S: 0000 0110 11dd dddd 0010 0000
                // X/Y:ea: 0000 0110 01MM MRRR 0s10 0000
                // X/Y:aa: 0000 0110 00aa aaaa 0s10 0000
                // ============================================================
                24'b0000_0110_????_????_1010_????: begin
                    np_is_loop = 1'b1;
                    loop_is_rep = 1'b1;
                end
                24'b0000_0110_11??_????_0010_0000: begin
                    // Only the documented register map is legal; 00-03 and
                    // 28-38 are reserved in this source field.
                    if ((inst[13:8] >= 6'h04 && inst[13:8] <= 6'h27) ||
                        inst[13:8] >= 6'h39) begin
                        np_is_loop = 1'b1;
                        loop_is_rep = 1'b1;
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end
                24'b0000_0110_01??_????_0?10_0000: begin
                    // MMM=110 is not a REP effective-address encoding.
                    if (inst[13:11] != 3'b110) begin
                        np_is_loop = 1'b1;
                        loop_is_rep = 1'b1;
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end
                24'b0000_0110_00??_????_0?10_0000: begin
                    np_is_loop = 1'b1;
                    loop_is_rep = 1'b1;
                end

                // ============================================================
                // MOVEC (move to/from control register)
                // 0000 0101 W0 aaaaaa 0s1ddddd = short memory
                // 0000 0101 W1 MMMRRR 0s1ddddd = effective-address memory
                // 0000 0101 iiiiiiii 101ddddd = short immediate
                // ============================================================
                24'b0000_0101_????_????_????_????: begin
                    if (movec_family_valid) begin
                        np_is_move = 1'b1;
                        is_movec = 1'b1;
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end

                // ============================================================
                // MOVEM (move to/from program memory)
                // 0000 0111 W0 aaaaaa 00dddddd = short absolute
                // 0000 0111 W1 MMMRRR 10dddddd = effective address
                // ============================================================
                24'b0000_0111_????_????_????_????: begin
                    if (movem_family_valid) begin
                        np_is_move = 1'b1;
                        is_movem = 1'b1;
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end

                // ============================================================
                // MOVEP (move to/from peripheral)
                // 0000 100S W1 dddddd 00pppppp = register reference
                // 0000 100S W1 MMMRRR 01pppppp = P-memory reference
                // 0000 100s W1 MMMRRR 1Spppppp = X/Y-memory reference
                // ============================================================
                24'b0000_100?_????_????_????_????: begin
                    if (movep_family_valid) begin
                        np_is_move = 1'b1;
                        is_movep = 1'b1;
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end

                // ============================================================
                // DIV S,D: 0000 0001 1000 0000 01JJ d000
                // ============================================================
                24'b0000_0001_1000_0000_01??_?000: begin
                    np_is_alu = 1'b1;
                    np_alu_opcode = ALU_DIV;
                    np_alu_dest = inst[3];
                    np_alu_jjj = {1'b1, inst[5:4]};  // Map JJ to JJJ
                end

                // ============================================================
                // NORM Rn,D: 0000 0001 1101 1RRR 0001 d101
                // ============================================================
                24'b0000_0001_1101_1???_0001_?101: begin
                    np_is_alu = 1'b1;
                    np_alu_opcode = ALU_NORM;
                    np_alu_dest = inst[3];
                    is_norm = 1'b1;
                    norm_rrr = inst[10:8];
                end

                // ============================================================
                // MOVEC register-to-register
                // 0000 0100 W1 eeeeee 10dddddd
                // W selects source direction.  Either field may be the
                // control-register subset operand; the other uses the full
                // architectural register map.
                // ============================================================
                24'b0000_0100_?1??_????_10??_????: begin
                    if (movec_register_valid) begin
                        np_is_move = 1'b1;
                        is_movec = 1'b1;
                        is_movec_rr = 1'b1;
                    end else begin
                        np_is_control = 1'b1;
                        is_illegal = 1'b1;
                    end
                end

                // ============================================================
                // LUA (Load Update Address)
                // 0000 0100 010M MRRR 0001 dddd
                // ============================================================
                24'b0000_0100_010?_????_0001_????: begin
                    np_is_control = 1'b1;
                    is_lua = 1'b1;
                    lua_mmm = inst[13:11];
                    lua_rrr = inst[10:8];
                    lua_ddddd = inst[4:0];
                end

                // ============================================================
                // Bit manipulation: BTST/BCHG/BCLR/BSET
                // 0000 1010 AAAAAA BB bbbbb
                // ============================================================
                24'b0000_1011_0???_????_????_????: begin
                    np_is_bit = 1'b1;
                    bit_op_type = inst[9:8];
                    bit_op_bit = inst[4:0];
                    bit_op_mem = 1'b1;
                    bit_op_space = inst[6];
                    bit_op_addr = inst[19:14];
                end

                // ============================================================
                // JCLR/JSET/JSCLR/JSSET
                // ============================================================
                // These will be added as needed during cosim

                default: begin
                    // Undefined operation codes enter the architectural
                    // illegal-instruction exception rather than retiring as
                    // silent NOPs.
                    np_is_control = 1'b1;
                    is_illegal = 1'b1;
                end
            endcase
        end
    end

    // ========================================================================
    // Output Multiplexing
    // ========================================================================
    assign is_nop = is_non_parallel && (inst == 24'h000000);

    assign is_alu_op     = is_parallel ? par_alu_is_alu : np_is_alu;
    assign is_move_op    = is_parallel ? (par_move_type != MOVE_NONE && par_move_type != MOVE_NO_MOVE) : np_is_move;
    assign is_branch_op  = np_is_branch;
    assign is_loop_op    = np_is_loop;
    assign is_bit_op     = np_is_bit;
    assign is_control_op = is_non_parallel && np_is_control;

    always @(*) begin
        if (is_parallel) begin
            alu_opcode      = par_alu_opcode;
            alu_dest        = par_alu_dest;
            alu_src_jjj     = par_alu_jjj;
            alu_mpy_qq      = par_alu_qq;
            alu_negate      = par_alu_neg;
            alu_update_ccr  = par_alu_is_alu;
            alu_no_writeback = par_alu_no_wb;

            move_type       = par_move_type;
            move_src_reg    = par_move_src_reg;
            move_dst_reg    = par_move_dst_reg;
            move_src_mem    = par_move_src_mem;
            move_dst_mem    = par_move_dst_mem;
            move_mem_space  = par_move_space;
            // A short immediate is an 8-bit signed fractional operand for
            // the four data registers and the full accumulators.  The same
            // encoding is an unsigned integer for accumulator components
            // and address registers (DSP56001 UMA, pp. A-120--A-121).
            move_imm_data   = (par_move_type == MOVE_IMM) ? ext_word :
                              ((par_move_type == MOVE_IMM_SHORT) &&
                               (((par_move_dst_reg >= 6'h04) &&
                                 (par_move_dst_reg <= 6'h07)) ||
                                (par_move_dst_reg == 6'h0e) ||
                                (par_move_dst_reg == 6'h0f))) ?
                              {inst[15:8], 16'h0000} :
                              {16'h0000, inst[15:8]};
            move_is_imm     = (par_move_type == MOVE_IMM_SHORT) || (par_move_type == MOVE_IMM);

            addr_mode_mmm   = inst[13:11];
            addr_reg_rrr    = inst[10:8];
            addr_absolute   = inst[13:8];
        end else begin
            alu_opcode      = np_alu_opcode;
            alu_dest        = np_alu_dest;
            alu_src_jjj     = np_alu_jjj;
            alu_mpy_qq      = 3'b0;
            alu_negate      = 1'b0;
            alu_update_ccr  = np_is_alu;
            alu_no_writeback = 1'b0;

            move_type       = MOVE_NO_MOVE;
            move_src_reg    = 6'h0;
            move_dst_reg    = 6'h0;
            move_src_mem    = 1'b0;
            move_dst_mem    = 1'b0;
            move_mem_space  = 1'b0;
            move_imm_data   = ext_word;
            move_is_imm     = 1'b0;

            addr_mode_mmm   = inst[13:11];
            addr_reg_rrr    = inst[10:8];
            addr_absolute   = inst[13:8];
        end
    end

endmodule
