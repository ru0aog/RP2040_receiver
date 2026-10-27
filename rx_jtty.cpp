// rx_jtty.cpp — JTTY decoder for RP2040_receiver.
// 4-FSK, 31.25 baud, 500 samples/symbol at 15625 Hz.
// Uses the shared rx_goertzel bank only; no FFT and no private Goertzel bank.

#include "rx_jtty.h"
#include "rx_goertzel.h"
#include "rx_decoders.h"

#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

// -----------------------------------------------------------------------------
// Protocol parameters
// -----------------------------------------------------------------------------
static constexpr float    JTTY_FS_HZ       = 15625.0f;
static constexpr float    JTTY_TONE_STEP   = 31.25f;
static constexpr int      JTTY_NTONES      = 4;
static constexpr int      JTTY_SPS         = 500;
static constexpr int      JTTY_NSYNC       = 13;
static constexpr int      JTTY_NDATA       = 46;
static constexpr int      JTTY_INFO_BITS   = 46;
static constexpr int      JTTY_PAYLOAD_BITS = 34;
static constexpr int      JTTY_NU          = 9;
static constexpr int      JTTY_NSTATES     = 1 << JTTY_NU;
static constexpr uint16_t JTTY_G0          = 01167;
static constexpr uint16_t JTTY_G1          = 01545;
static constexpr uint16_t JTTY_CRC_POLY    = 0x80F;

// The sync sequence is also used by RP2040_beacon/jtty_modem.cpp.
static const uint8_t JTTY_IS13[JTTY_NSYNC] = {
    0, 2, 2, 3, 0, 0, 3, 2, 1, 3, 1, 2, 0
};

// TEXT5 alphabet from the JTTY source encoding table (64 entries).
// Index 36 is a space; indices not assigned by the protocol should not be
// guessed or remapped here.
static const char JTTY_ALPHABET[65] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ +-./?!\"#$%,&*()_'=[]{}<>|:;";

// The shared Goertzel bank produces one result every 100 input samples.
// A symbol-aligned 500-sample window therefore occurs every five results.
static constexpr int JTTY_HOP_SAMPLES = 100;
static constexpr int JTTY_HOPS_PER_SYMBOL =
    JTTY_SPS / JTTY_HOP_SAMPLES;

// Keep enough overlapping-window decisions to test 13 symbols spaced five
// results apart, for every possible phase modulo 100 samples.
static constexpr int JTTY_SYNC_HISTORY =
    (JTTY_NSYNC - 1) * JTTY_HOPS_PER_SYMBOL + 1;

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------
static bool g_active = false;
static bool g_hunting = true;

static uint8_t g_sync_tones[JTTY_SYNC_HISTORY];
static int g_sync_write = 0;
static int g_sync_count = 0;

static float g_data_power[JTTY_NDATA][JTTY_NTONES];
static int g_data_count = 0;
static int g_hops_since_sync = 0;

// Output text is assembled until an EOM atom is received.
static char g_message[128];
static size_t g_message_len = 0;

// Viterbi working memory. A byte per state/step is compact and avoids
// heap allocation; bit 0 records which predecessor's high state bit won.
static float g_metric_a[JTTY_NSTATES];
static float g_metric_b[JTTY_NSTATES];
static uint8_t g_predecessor[JTTY_INFO_BITS][JTTY_NSTATES];

// Tone index from the two convolutionally encoded bits:
// b0,b1 = 00,01,10,11 -> tone 0,1,3,2.
static const uint8_t JTTY_GRAY_TO_TONE[4] = {0, 1, 3, 2};

// -----------------------------------------------------------------------------
// Small helpers
// -----------------------------------------------------------------------------
static inline int jtty_parity(uint32_t x)
{
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return (int)(x & 1u);
}

static void jtty_message_clear(void)
{
    g_message_len = 0;
    g_message[0] = '\0';
}

static void jtty_message_append(const char* text)
{
    if (text == nullptr || *text == '\0') {
        return;
    }

    const size_t n = strlen(text);
    if (n >= sizeof(g_message) - 1 ||
        g_message_len + n >= sizeof(g_message) - 1) {
        // Never emit a truncated/concatenated message after overflow.
        jtty_message_clear();
        if (n >= sizeof(g_message) - 1) {
            return;
        }
    }

    memcpy(g_message + g_message_len, text, n);
    g_message_len += n;
    g_message[g_message_len] = '\0';
}

static void jtty_message_flush(void)
{
    // The protocol filler is represented by '<'. Trim trailing filler and
    // spaces; do not invent replacements for filler inside the message.
    while (g_message_len > 0 &&
           (g_message[g_message_len - 1] == '<' ||
            g_message[g_message_len - 1] == ' ')) {
        --g_message_len;
    }
    g_message[g_message_len] = '\0';

    for (size_t i = 0; i < g_message_len; ++i) {
        const unsigned char c = (unsigned char)g_message[i];
        if (c >= 32 && c <= 126) {
            rx_decoders_emit((char)c);
        }
    }
    if (g_message_len != 0) {
        rx_decoders_emit('\n');
    }
    jtty_message_clear();
}

// CRC-12 over the 34 payload bits, MSB first.
static uint16_t jtty_crc12(const uint8_t* bits)
{
    uint16_t reg = 0;

    for (int i = 0; i < JTTY_PAYLOAD_BITS; ++i) {
        const uint16_t feedback =
            (uint16_t)(((reg >> 11) ^ bits[i]) & 1u);
        reg = (uint16_t)((reg << 1) & 0x0FFFu);
        if (feedback != 0) {
            reg ^= JTTY_CRC_POLY;
        }
    }
    return reg;
}

// -----------------------------------------------------------------------------
// Tail-biting Viterbi
//
// State convention:
//   next_state = ((previous_state << 1) | input_bit) & 0x1ff
//   input_bit  = next_state & 1
// For a given next state, the two possible predecessor states differ in bit 8.
// The selected predecessor's bit 8 is stored for traceback.
//
// Pass 1: all initial states have equal metric; select the best ending state.
// Pass 2: constrain the start to that state and traceback from the same state.
// This is a bounded-cost approximation to full tail-biting list decoding. The
// frame is accepted only if the resulting path passes CRC-12.
// -----------------------------------------------------------------------------
static bool jtty_viterbi_decode(uint8_t decoded[JTTY_INFO_BITS],
                                const float power[JTTY_NDATA][JTTY_NTONES])
{
    const float NEG_INF = -1.0e30f;

    for (int s = 0; s < JTTY_NSTATES; ++s) {
        g_metric_a[s] = 0.0f;
    }

    // Unconstrained pass: estimate the most likely circular state.
    for (int step = 0; step < JTTY_INFO_BITS; ++step) {
        float max_power = power[step][0];
        for (int t = 1; t < JTTY_NTONES; ++t) {
            if (power[step][t] > max_power) {
                max_power = power[step][t];
            }
        }

        for (int s = 0; s < JTTY_NSTATES; ++s) {
            g_metric_b[s] = NEG_INF;
        }

        for (int s = 0; s < JTTY_NSTATES; ++s) {
            const int input_bit = s & 1;
            const int pred_base = s >> 1;

            for (int pred_msb = 0; pred_msb <= 1; ++pred_msb) {
                const int prev = pred_base | (pred_msb << (JTTY_NU - 1));
                const float old_metric = g_metric_a[prev];
                if (old_metric <= NEG_INF / 2.0f) {
                    continue;
                }

                const uint16_t reg =
                    (uint16_t)(((prev << 1) | input_bit) & 0x03FFu);
                const int b0 = jtty_parity((uint32_t)(reg & JTTY_G0));
                const int b1 = jtty_parity((uint32_t)(reg & JTTY_G1));
                const int tone = JTTY_GRAY_TO_TONE[(b0 << 1) | b1];

                // Subtracting the per-symbol maximum keeps the metric bounded
                // without changing which path maximizes it.
                const float candidate =
                    old_metric + (power[step][tone] - max_power);

                if (candidate > g_metric_b[s]) {
                    g_metric_b[s] = candidate;
                    g_predecessor[step][s] = (uint8_t)pred_msb;
                }
            }
        }

        float* tmp = g_metric_a;
        memcpy(g_metric_a, g_metric_b, sizeof(g_metric_a));
        (void)tmp;
    }

    int best_state = 0;
    for (int s = 1; s < JTTY_NSTATES; ++s) {
        if (g_metric_a[s] > g_metric_a[best_state]) {
            best_state = s;
        }
    }

    // Constrained pass: start and end at best_state.
    for (int s = 0; s < JTTY_NSTATES; ++s) {
        g_metric_a[s] = NEG_INF;
    }
    g_metric_a[best_state] = 0.0f;

    for (int step = 0; step < JTTY_INFO_BITS; ++step) {
        float max_power = power[step][0];
        for (int t = 1; t < JTTY_NTONES; ++t) {
            if (power[step][t] > max_power) {
                max_power = power[step][t];
            }
        }

        for (int s = 0; s < JTTY_NSTATES; ++s) {
            g_metric_b[s] = NEG_INF;
        }

        for (int s = 0; s < JTTY_NSTATES; ++s) {
            const int input_bit = s & 1;
            const int pred_base = s >> 1;

            for (int pred_msb = 0; pred_msb <= 1; ++pred_msb) {
                const int prev = pred_base | (pred_msb << (JTTY_NU - 1));
                const float old_metric = g_metric_a[prev];
                if (old_metric <= NEG_INF / 2.0f) {
                    continue;
                }

                const uint16_t reg =
                    (uint16_t)(((prev << 1) | input_bit) & 0x03FFu);
                const int b0 = jtty_parity((uint32_t)(reg & JTTY_G0));
                const int b1 = jtty_parity((uint32_t)(reg & JTTY_G1));
                const int tone = JTTY_GRAY_TO_TONE[(b0 << 1) | b1];

                const float candidate =
                    old_metric + (power[step][tone] - max_power);

                if (candidate > g_metric_b[s]) {
                    g_metric_b[s] = candidate;
                    g_predecessor[step][s] = (uint8_t)pred_msb;
                }
            }
        }

        memcpy(g_metric_a, g_metric_b, sizeof(g_metric_a));
    }

    if (g_metric_a[best_state] <= NEG_INF / 2.0f) {
        return false;
    }

    int state = best_state;
    for (int step = JTTY_INFO_BITS - 1; step >= 0; --step) {
        decoded[step] = (uint8_t)(state & 1);
        const int pred_msb = g_predecessor[step][state] & 1u;
        state = (state >> 1) | (pred_msb << (JTTY_NU - 1));
    }

    // Enforce the tail-biting condition explicitly.
    return state == best_state;
}

// -----------------------------------------------------------------------------
// Payload handling
// -----------------------------------------------------------------------------
static void jtty_process_frame(void)
{
    uint8_t info[JTTY_INFO_BITS];

    if (!jtty_viterbi_decode(info, g_data_power)) {
#ifdef JTTY_DEBUG
        Serial.println(F("# JTTY: Viterbi/tail-biting failure"));
#endif
        return;
    }

    const uint16_t calculated_crc = jtty_crc12(info);
    uint16_t received_crc = 0;
    for (int i = JTTY_PAYLOAD_BITS; i < JTTY_INFO_BITS; ++i) {
        received_crc = (uint16_t)((received_crc << 1) | (info[i] & 1u));
    }

    if (calculated_crc != received_crc) {
#ifdef JTTY_DEBUG
        Serial.println(F("# JTTY: CRC failure"));
#endif
        return;
    }

    // Payload bits 0..31 are the atom value; bit 32 is reserved and bit 33 EOM.
    uint32_t atom = 0;
    for (int i = 0; i < 32; ++i) {
        atom = (atom << 1) | (info[i] & 1u);
    }

    const bool reserved = info[32] != 0;
    const bool eom = info[33] != 0;
    const uint8_t type = (uint8_t)(atom & 0x03u);

    if (!reserved) {
        if (type == 3u) {
            // TEXT5: five 6-bit alphabet indices occupy the upper 30 bits.
            uint32_t packed = atom >> 2;
            char text[6];

            for (int i = 4; i >= 0; --i) {
                const uint8_t index = (uint8_t)(packed & 0x3Fu);
                packed >>= 6;

                // The encoding table is expected to contain exactly 64
                // characters; guard the lookup even if the table is edited.
                text[i] = (index < 64u) ? JTTY_ALPHABET[index] : '?';
            }
            text[5] = '\0';
            jtty_message_append(text);
        } else {
            // CALL (type 0/1) is intentionally not decoded here. The exact
            // pack28/unpack28 bit layout must be verified against the
            // reference golden vectors before displaying a callsign.
            // Type 2 is a non-text atom and is ignored.
        }
    }

    // EOM is meaningful even when the final atom is not TEXT5.
    if (eom) {
        jtty_message_flush();
    }
}

// -----------------------------------------------------------------------------
// Synchronization
// -----------------------------------------------------------------------------
static void jtty_clear_sync_history(void)
{
    memset(g_sync_tones, 0, sizeof(g_sync_tones));
    g_sync_write = 0;
    g_sync_count = 0;
}

static bool jtty_sync_match(void)
{
    if (g_sync_count < JTTY_SYNC_HISTORY) {
        return false;
    }

    int matches = 0;
    for (int i = 0; i < JTTY_NSYNC; ++i) {
        // Current result is the end of symbol 12. Previous sync symbols are
        // exactly five Goertzel hops apart. This tests the five possible
        // phases implicitly as each new 100-sample hop arrives.
        const int back = (JTTY_NSYNC - 1 - i) * JTTY_HOPS_PER_SYMBOL;
        int idx = g_sync_write - 1 - back;
        while (idx < 0) {
            idx += JTTY_SYNC_HISTORY;
        }
        idx %= JTTY_SYNC_HISTORY;

        if (g_sync_tones[idx] == JTTY_IS13[i]) {
            ++matches;
        }
    }

    return matches >= 7;
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
void rx_jtty_init(float center_hz)
{
    g_active = false;
    g_hunting = true;
    g_data_count = 0;
    g_hops_since_sync = 0;
    jtty_clear_sync_history();
    jtty_message_clear();

    if (!isfinite(center_hz) || center_hz <= 0.0f ||
        center_hz >= JTTY_FS_HZ * 0.5f) {
        return;
    }

    // Four tones are symmetric around the RSID center:
    // tone 0 = center - 1.5 * spacing, tone 3 = center + 1.5 * spacing.
    const float base_hz = center_hz - 1.5f * JTTY_TONE_STEP;
    const float last_hz = base_hz + (JTTY_NTONES - 1) * JTTY_TONE_STEP;

    if (base_hz < 0.0f || last_hz >= JTTY_FS_HZ * 0.5f) {
        return;
    }

    const gb_cfg_t cfg = {
        JTTY_NTONES,
        base_hz,
        JTTY_TONE_STEP,
        JTTY_FS_HZ,
        JTTY_SPS
    };

    // Keep one shared-bank configuration throughout hunt and collection.
    // Search is performed on overlapping 500-sample windows, hop=100.
    if (!gb_init_ex(&cfg, JTTY_SPS, JTTY_HOP_SAMPLES, GB_POWER_LINEAR)) {
        return;
    }

    g_active = true;
}

void rx_jtty_stop(void)
{
    g_active = false;
    g_hunting = true;
    g_data_count = 0;
    g_hops_since_sync = 0;
    jtty_clear_sync_history();
    jtty_message_clear();

    // Do not call gb_reset() here: the Goertzel bank is shared globally and
    // may already have been reconfigured by the decoder dispatcher.
}

void rx_jtty_push(int16_t sample)
{
    if (!g_active) {
        return;
    }

    float power[JTTY_NTONES];
    if (!gb_push_ex(sample, power)) {
        return;
    }

    if (g_hunting) {
        int best_tone = 0;
        for (int t = 1; t < JTTY_NTONES; ++t) {
            if (power[t] > power[best_tone]) {
                best_tone = t;
            }
        }

        g_sync_tones[g_sync_write] = (uint8_t)best_tone;
        g_sync_write = (g_sync_write + 1) % JTTY_SYNC_HISTORY;
        if (g_sync_count < JTTY_SYNC_HISTORY) {
            ++g_sync_count;
        }

        if (jtty_sync_match()) {
#ifdef JTTY_DEBUG
            Serial.println(F("# JTTY: sync found"));
#endif
            g_hunting = false;
            g_data_count = 0;
            g_hops_since_sync = 0;
        }
        return;
    }

    // The sync detector fires at the end of the final sync-symbol window.
    // Wait for five 100-sample hops so the next 500-sample window is the first
    // data symbol, then collect one symbol every five hops.
    if (++g_hops_since_sync < JTTY_HOPS_PER_SYMBOL) {
        return;
    }
    g_hops_since_sync = 0;

    if (g_data_count < JTTY_NDATA) {
        for (int t = 0; t < JTTY_NTONES; ++t) {
            g_data_power[g_data_count][t] = power[t];
        }
        ++g_data_count;
    }

    if (g_data_count >= JTTY_NDATA) {
        jtty_process_frame();

        // Continue using the same global bank and hop cadence. Clear history
        // so the frame's data tones cannot be mistaken for a fresh preamble.
        g_hunting = true;
        g_data_count = 0;
        g_hops_since_sync = 0;
        jtty_clear_sync_history();
    }
}
