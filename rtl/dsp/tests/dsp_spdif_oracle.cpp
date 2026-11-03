// SPDX-License-Identifier: MIT
// Copyright (c) 2026 I. Finder <src@lispm.net>

// Production SSI-word to S/PDIF serializer boundary oracle.

#include <array>
#include <cstdint>
#include <cstdio>

#include "Vcyan_spdif_tx.h"
#include "Vcyan_spdif_tx___024root.h"
#include "verilated.h"

namespace {

constexpr uint32_t kSample = 0x00a5963c;
constexpr uint32_t kChannelStatus = 0x00000001;
constexpr uint32_t kUserData = 0x00000001;

class Harness {
public:
    Vcyan_spdif_tx dut;

    Harness() {
        dut.clk = 0;
        dut.rst_n = 0;
        dut.sample_data = 0;
        dut.sample_valid = 0;
        dut.channel_status_data = kChannelStatus;
        dut.channel_status_valid = 1;
        dut.user_data = kUserData;
        dut.user_valid = 1;
        dut.stream_active = 0;
        dut.sample_rate_select = 0;
        for (unsigned i = 0; i < 3; ++i)
            tick();
        dut.rst_n = 1;
        tick();
        dut.stream_active = 1;
    }

    void tick() {
        dut.clk = 0;
        dut.eval();
        dut.clk = 1;
        dut.eval();
        dut.clk = 0;
        dut.eval();
    }

    bool wait_ready(unsigned limit) {
        for (unsigned i = 0; i < limit; ++i) {
            tick();
            if (dut.sample_ready)
                return true;
        }
        return false;
    }
};

bool serializer_case() {
    Harness h;
    const bool initial_request = h.wait_ready(80);

    std::array<uint8_t, 64> cells{};
    h.dut.sample_data = kSample;
    for (unsigned i = 0; i < cells.size(); ++i) {
        h.dut.sample_valid = i == 0;
        h.tick();
        cells[i] = h.dut.spdif;
    }
    h.dut.sample_valid = 0;

    // Channel A, frame zero uses the B preamble. Polarity may invert to
    // preserve BMC continuity, but its relative eight-half-cell pattern may
    // not change.
    constexpr uint8_t preamble = 0xe8;
    const bool inversion = cells[0] != ((preamble >> 7) & 1);
    bool preamble_ok = true;
    for (unsigned i = 0; i < 8; ++i)
        preamble_ok &= cells[i] ==
            (uint8_t(((preamble >> (7 - i)) & 1) ^ inversion));

    std::array<uint8_t, 28> bits{};
    for (unsigned i = 0; i < bits.size(); ++i) {
        const unsigned first = 8 + i * 2;
        bits[i] = cells[first] != cells[first + 1];
    }

    bool pcm_ok = true;
    for (unsigned i = 0; i < 24; ++i)
        pcm_ok &= bits[i] == ((kSample >> i) & 1);
    const uint8_t parity = __builtin_parity(kSample) ^ 1 ^ 1;
    const bool metadata_ok = bits[24] == 0 && bits[25] == 1 &&
                             bits[26] == 1 && bits[27] == parity;
    const bool second_request = h.dut.sample_ready;

    // Leave the second request unanswered through half-cell seven. The
    // serializer must substitute silence and record exactly one underrun.
    for (unsigned i = 0; i < 8; ++i)
        h.tick();
    h.dut.stream_active = 0;
    h.tick();

    const auto *root = h.dut.rootp;
    const uint32_t requests =
        root->cyan_spdif_tx__DOT__telemetry_request_count;
    const uint32_t accepts =
        root->cyan_spdif_tx__DOT__telemetry_accept_count;
    const uint16_t underruns =
        root->cyan_spdif_tx__DOT__telemetry_underrun_count;
    const uint8_t nonzero =
        root->cyan_spdif_tx__DOT__telemetry_nonzero_count;
    const uint32_t signature =
        root->cyan_spdif_tx__DOT__telemetry_sample_signature;
    const bool completed =
        root->cyan_spdif_tx__DOT__telemetry_session_completed;
    const bool ledger_ok = requests == 2 && accepts == 1 && underruns == 1 &&
                           nonzero == 1 && signature == kSample && completed;
    const bool pass = initial_request && second_request && preamble_ok &&
                      pcm_ok && metadata_ok && ledger_ok;
    std::printf("%s SPDIF subframe request=%u/%u preamble=%u PCM=%u "
                "metadata=%u bits[VUCP]=%u%u%u%u ledger=%u "
                "req=%u acc=%u under=%u nonzero=%u sig=%06x done=%u\n",
                pass ? "PASS" : "FAIL", initial_request, second_request,
                preamble_ok, pcm_ok, metadata_ok, bits[24], bits[25],
                bits[26], bits[27], ledger_ok, requests, accepts, underruns,
                nonzero, signature, completed);
    return pass;
}

bool stereo_phase_case() {
    Harness h;
    constexpr std::array<uint32_t, 3> samples = {
        0x00123456, 0x00abcdef, 0x0055aa33
    };
    constexpr std::array<uint8_t, 3> preambles = {
        0xe8, // Channel A, block/frame zero: B preamble.
        0xe4, // Channel B: W preamble.
        0xe2, // Channel A, subsequent frame: M preamble.
    };

    bool requests = h.wait_ready(80);
    bool preamble_ok = true;
    bool pcm_ok = true;
    for (unsigned subframe = 0; subframe < samples.size(); ++subframe) {
        std::array<uint8_t, 64> cells{};
        h.dut.sample_data = samples[subframe];
        for (unsigned i = 0; i < cells.size(); ++i) {
            h.dut.sample_valid = i == 0;
            h.tick();
            cells[i] = h.dut.spdif;
        }
        h.dut.sample_valid = 0;
        requests &= h.dut.sample_ready;

        const bool inversion =
            cells[0] != ((preambles[subframe] >> 7) & 1);
        for (unsigned i = 0; i < 8; ++i)
            preamble_ok &= cells[i] ==
                uint8_t(((preambles[subframe] >> (7 - i)) & 1) ^ inversion);
        for (unsigned bit = 0; bit < 24; ++bit) {
            const unsigned first = 8 + bit * 2;
            const bool decoded = cells[first] != cells[first + 1];
            pcm_ok &= decoded == ((samples[subframe] >> bit) & 1);
        }
    }
    h.dut.stream_active = 0;
    h.tick();

    const bool pass = requests && preamble_ok && pcm_ok;
    std::printf("%s SPDIF stereo phase requests=%u preambles[BWM]=%u "
                "PCM[A0/B0/A1]=%u samples=%06x/%06x/%06x\n",
                pass ? "PASS" : "FAIL", requests, preamble_ok, pcm_ok,
                samples[0], samples[1], samples[2]);
    return pass;
}

} // namespace

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    const bool serializer_ok = serializer_case();
    const bool stereo_ok = stereo_phase_case();
    return serializer_ok && stereo_ok ? 0 : 1;
}
