# DSP and audio status

CYAN includes a synthesizable DSP56001-compatible execution core and the
workstation audio path around it. The public source gate tests the product RTL
directly and does not require proprietary firmware, diagnostic binaries, or
vendor manuals.

## Implemented hardware

The source tree contains:

- the DSP instruction decoder, ALU, address-generation unit, register state,
  program flow, interrupts, system stack, program/data memories, and factory
  data-ROM tables;
- the CPU-facing host interface and normal/folded DMA in both directions;
- DSP program, X, and Y memory access, including external wait states;
- Port C and system registers, SCI/SSI register and interrupt boundaries; and
- SSI transmit framing and S/PDIF output integration.

## Public regression coverage

`make dsp-test` exercises the synthesizable modules through their production
ports. The suite currently includes:

- ALU-family and differential arithmetic checks, multiply selectors, DIV,
  NORM, RND, TFR, condition-code behavior, and reserved status bits;
- Tcc condition, data-source, accumulator-destination, and complete R-to-R
  transfer matrices, plus all legal LUA update modes across linear, modulo,
  and bit-reversed address arithmetic;
- all 16 Jcc/JScc conditions in both polarities, short and direct-EA forms,
  including call-stack and timing behavior;
- differential and architectural AGU checks;
- DO/REP decoding and loop behavior;
- WAIT, SWI, trace, jump timing, illegal-instruction handling, interrupt
  priority/edge/nesting/combination behavior, and interrupt arrival during
  memory retirement;
- STOP oscillator-recovery timing, RESET pulse/timing and state preservation,
  and system-stack overflow, underflow, forced-UF, and explicit recovery;
- BSET/BCLR/BCHG/BTST timing and effects across memory, data-register,
  accumulator, address-register, and controller-register representatives;
- MOVEC, MOVEM, MOVEP, Class-I, Class-II, X:Y parallel moves, selector maps,
  address-register latency, and instruction timing;
- program fetch/data and X/Y writeback wait-state handling;
- system-register, Port C, SCI, exact 512-entry data-ROM, and SSI TX/RX
  data/status/exception/reset boundaries;
- Port-B audio-glue register strobes, interrupt lifecycle, RESET preservation,
  and exact transmit-block cadence;
- S/PDIF preamble, PCM, metadata, parity, request/underrun, telemetry, and
  three-subframe A0/B0/A1 stereo phase behavior;
- the 24-bit HPC GIO counter's exact 2:3 cadence, read-only/reset/wrap
  behavior, and 33 complete bidirectional host/DSP handshake rounds; and
- normal/folded DMA, shared-port CPU contention, host/DSP event collisions,
  and write-zero-to-clear completion collisions.

The tests are deterministic, architecture-derived, and MIT-licensed. Scratch
build products remain outside Git.

## Remaining release evidence

Passing the public source gate is not a claim of exhaustive DSP56001 ISA or
cycle coverage. It also does not by itself prove a complete workstation audio
session. The current KCU board integration ties the DSP SSI receive input
inactive, so physical audio input is not implemented by that target. Release
qualification still requires:

- clean implementation timing and design-rule signoff for an exact source
  commit;
- tests using legally supplied system firmware and software outside this
  repository; and
- physical playback acceptance, including volume/mute behavior and independent
  left/right channel phase.

Firmware images, operating-system software, proprietary diagnostics, generated
FPGA IP, and bitstreams are not distributed by this project.
