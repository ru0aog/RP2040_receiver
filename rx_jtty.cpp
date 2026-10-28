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

// Отладочный вывод JTTY: синхра, запуск декода, причина сброса кадра.  
// Убрать/закомментировать перед релизом — печать идёт на Core 0.  
#define JTTY_DEBUG

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
  
// Minimum matches out of 13 sync positions to accept a preamble.  
// jtty_mdecode channel 0 requires nsync > 6; 8 gives margin for our  
// overlapping-window counting, which scores more loosely than theirs.  
static constexpr int JTTY_SYNC_MIN = 8;

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------
static bool g_active = false;
static bool g_hunting = true;

static uint8_t g_sync_tones[JTTY_SYNC_HISTORY];
static int g_sync_write = 0;
static int g_sync_count = 0;

static float g_data_power[JTTY_NDATA][JTTY_NTONES];    
static float g_dec_power[JTTY_NDATA][JTTY_NTONES];  // snapshot for the decoder  
static int g_data_count = 0;    
static int g_hops_since_sync = 0;
  
// Resumable frame decode state machine. The Viterbi passes run in small  
// quanta so that decoding never blocks the audio polling loop on Core 0.  
typedef enum {  
    JTTY_DEC_IDLE = 0,  
    JTTY_DEC_PASS1,  
    JTTY_DEC_PASS2,  
    JTTY_DEC_TRACEBACK,  
    JTTY_DEC_FINISH  
} jtty_dec_state_t;  
  
static jtty_dec_state_t g_dec_state = JTTY_DEC_IDLE;  
static int  g_dec_step = 0;        // symbol step within a pass  
static int  g_dec_state_i = 0;     // destination-state iterator within a step  
static int  g_dec_pass2_state = 0; // tail-biting start/end state from pass 1  
static bool g_dec_ok = false;      // traceback passed the circular check  
static uint8_t g_dec_bits[JTTY_INFO_BITS]; // decoded info bits when g_dec_ok  
  
// Viterbi quanta per service call. JTTY_VIT_STEPS are state iterations  
// inside one symbol step; the number of serviced calls is small enough that  
// a frame finishes within a few Goertzel hops.  
static constexpr int JTTY_VIT_STATES_PER_CALL = 256; // ~half a state sweep

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

// One symbol step of a Viterbi pass, bounded to JTTY_VIT_STATES_PER_CALL  
// destination-state updates. Returns true when the step is finished.  
// g_dec_state_i tracks progress within the step; on entry to a new step it  
// resets g_metric_b.  
static bool jtty_vit_step_quanta(const float power[JTTY_NDATA][JTTY_NTONES],  
                                 int step)  
{  
    const float NEG_INF = -1.0e30f;  
  
    if (g_dec_state_i == 0) {  
        for (int s = 0; s < JTTY_NSTATES; ++s) {  
            g_metric_b[s] = NEG_INF;  
        }  
    }  
  
    float max_power = power[step][0];  
    for (int t = 1; t < JTTY_NTONES; ++t) {  
        if (power[step][t] > max_power) {  
            max_power = power[step][t];  
        }  
    }  
  
    const int s_end = g_dec_state_i + JTTY_VIT_STATES_PER_CALL;  
    const int limit = (s_end < JTTY_NSTATES) ? s_end : JTTY_NSTATES;  
  
    for (int s = g_dec_state_i; s < limit; ++s) {  
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
  
    g_dec_state_i = limit;  
    if (g_dec_state_i < JTTY_NSTATES) {  
        return false; // step not finished yet  
    }  
  
    // Step complete: commit metrics.  
    memcpy(g_metric_a, g_metric_b, sizeof(g_metric_a));  
    g_dec_state_i = 0;  
    return true;  
}  
  
// Advance the decode state machine once per service call. Returns true when  
// the whole frame decode is done (g_dec_ok holds the result, g_dec_bits the  
// info bits on success).  
static bool jtty_dec_service(void)  
{  
    const float NEG_INF = -1.0e30f;  
  
    switch (g_dec_state) {  
  
    case JTTY_DEC_PASS1:  
        // Pass 1 launch happens in jtty_dec_start(). Here we iterate steps.  
        if (jtty_vit_step_quanta(g_dec_power, g_dec_step)) {
            if (++g_dec_step >= JTTY_INFO_BITS) {  
                // Pick best ending state.  
                int best_state = 0;  
                for (int s = 1; s < JTTY_NSTATES; ++s) {  
                    if (g_metric_a[s] > g_metric_a[best_state]) {  
                        best_state = s;  
                    }  
                }  
                g_dec_pass2_state = best_state;  
  
                // Launch constrained pass 2.  
                for (int s = 0; s < JTTY_NSTATES; ++s) {  
                    g_metric_a[s] = NEG_INF;  
                }  
                g_metric_a[best_state] = 0.0f;  
                g_dec_step = 0;  
                g_dec_state = JTTY_DEC_PASS2;  
            }  
        }  
        return false;  
  
    case JTTY_DEC_PASS2:  
        if (jtty_vit_step_quanta(g_dec_power, g_dec_step)) {
            if (++g_dec_step >= JTTY_INFO_BITS) {  
                g_dec_ok = false;  
                if (g_metric_a[g_dec_pass2_state] > NEG_INF / 2.0f) {  
                    g_dec_step = JTTY_INFO_BITS - 1;  
                    g_dec_state_i = g_dec_pass2_state; // reuse as traceback ptr  
                    g_dec_state = JTTY_DEC_TRACEBACK;  
                } else {  
                    g_dec_state = JTTY_DEC_FINISH;  
                }  
            }  
        }  
        return false;  
  
    case JTTY_DEC_TRACEBACK: {  
        // Traceback is only 46 iterations — do it in one quantum.  
        int state = g_dec_state_i;  
        for (int step = JTTY_INFO_BITS - 1; step >= 0; --step) {  
            g_dec_bits[step] = (uint8_t)(state & 1);  
            const int pred_msb = g_predecessor[step][state] & 1u;  
            state = (state >> 1) | (pred_msb << (JTTY_NU - 1));  
        }  
        g_dec_ok = (state == g_dec_pass2_state);  
        g_dec_state = JTTY_DEC_FINISH;  
        return false;  
    }  
  
    case JTTY_DEC_FINISH:  
        g_dec_state = JTTY_DEC_IDLE;  
        return true;  
  
    case JTTY_DEC_IDLE:  
    default:  
        return true;  
    }  
}  
  
// Launch the resumable decode on the collected frame.  
static void jtty_dec_start(void)    
{    
    memcpy(g_dec_power, g_data_power, sizeof(g_dec_power));  // ~0.1 ms, once per frame  
    for (int s = 0; s < JTTY_NSTATES; ++s) {    
        g_metric_a[s] = 0.0f;    
    }    
    g_dec_step = 0;    
    g_dec_state_i = 0;    
    g_dec_ok = false;    
    g_dec_state = JTTY_DEC_PASS1;    
}

// -----------------------------------------------------------------------------
// Payload handling
// -----------------------------------------------------------------------------
// Called once when a full 46-symbol data frame is in g_data_power: start the  
// resumable decode. The audio path returns to hunting immediately; decode  
// progresses through jtty_dec_service() on subsequent calls.  
static void jtty_frame_launch(void)    
{    
    if (g_dec_state != JTTY_DEC_IDLE) {  
#ifdef JTTY_DEBUG  
        Serial.println(F("# JTTY: frame dropped (decoder busy)"));  
#endif  
        return;  
    }  
#ifdef JTTY_DEBUG  
    Serial.println(F("# JTTY: frame collected, decode start"));  
#endif  
    jtty_dec_start();    
}
  
// Called when jtty_dec_service() reports completion: run CRC and emit atoms.  
static void jtty_frame_finish(void)    
{    
    const uint8_t* info = g_dec_bits;    
    
    if (!g_dec_ok) {    
#ifdef JTTY_DEBUG    
        Serial.println(F("# JTTY: Viterbi/tail-biting failure"));    
#endif    
        return;    
    }    
  
#ifdef JTTY_DEBUG  
    // Грубая оценка качества: сумма мощностей лучших тонов к полной.  
    float p_sig = 0.0f, p_sum = 0.0f;  
    for (int i = 0; i < JTTY_NDATA; ++i) {    
        float best = g_dec_power[i][0];    
        float row = 0.0f;    
        for (int t = 0; t < JTTY_NTONES; ++t) {    
            row += g_dec_power[i][t];    
            if (g_dec_power[i][t] > best) best = g_dec_power[i][t];    
        }
        p_sig += best;  
        p_sum += row;  
    }  
    const float ratio = (p_sum > 0.0f) ? p_sig / p_sum : 0.0f;  
    Serial.printf("# JTTY: Viterbi ok, tone_ratio=%.2f\n", (double)ratio);  
#endif
  
    const uint16_t calculated_crc = jtty_crc12(info);  
    uint16_t received_crc = 0;  
    for (int i = JTTY_PAYLOAD_BITS; i < JTTY_INFO_BITS; ++i) {  
        received_crc = (uint16_t)((received_crc << 1) | (info[i] & 1u));  
    }  
  
if (calculated_crc != received_crc) {    
#ifdef JTTY_DEBUG    
        Serial.printf("# JTTY: CRC failure calc=%03X rx=%03X\n",  
                      calculated_crc, received_crc);  
#endif    
        return;    
    }    
  
#ifdef JTTY_DEBUG  
    Serial.println(F("# JTTY: CRC ok"));  
#endif

    // Payload bits 0..31 are the atom value; bit 32 is reserved and bit 33 EOM.    
    uint32_t atom = 0;    
    for (int i = 0; i < 32; ++i) {    
        atom = (atom << 1) | (info[i] & 1u);    
    }    
  
#ifdef JTTY_DEBUG    
    Serial.printf("# JTTY atom=%08lX reserved=%d eom=%d\n",  
                  (unsigned long)atom, (int)(info[32] != 0), (int)(info[33] != 0));    
#endif
  
    const bool reserved = info[32] != 0;  
    const bool eom = info[33] != 0;  
    const uint8_t type = (uint8_t)(atom & 0x03u);  
  
    if (!reserved) {  
        if (type == 3u) {  
            uint32_t packed = atom >> 2;  
            char text[6];  
  
            for (int i = 4; i >= 0; --i) {  
                const uint8_t index = (uint8_t)(packed & 0x3Fu);  
                packed >>= 6;  
                text[i] = (index < 64u) ? JTTY_ALPHABET[index] : '?';  
            }  
            text[5] = '\0';  
            jtty_message_append(text);  
        }  
        // CALL atoms intentionally not decoded (see previous revision notes).  
    }  
  
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

static int jtty_sync_matches(void)  
{  
    if (g_sync_count < JTTY_SYNC_HISTORY) {  
        return 0;  
    }  
  
    int matches = 0;  
    for (int i = 0; i < JTTY_NSYNC; ++i) {  
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
    return matches;  
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
    g_dec_state = JTTY_DEC_IDLE;  
    g_dec_step = 0;  
    g_dec_state_i = 0;

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
    g_dec_state = JTTY_DEC_IDLE;  
    g_dec_step = 0;  
    g_dec_state_i = 0;

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
  
    // Advance a pending frame decode by one bounded quantum. Runs on every  
    // completed Goertzel result (~156 Hz), so a frame finishes in well under  
    // a second without ever blocking the audio loop.  
    if (g_dec_state != JTTY_DEC_IDLE && jtty_dec_service()) {  
        jtty_frame_finish();  
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

    if (jtty_sync_matches() >= JTTY_SYNC_MIN) {  
#ifdef JTTY_DEBUG  
            Serial.printf("# JTTY: sync found %d/%d\n",  
                          jtty_sync_matches(), JTTY_NSYNC);  
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
        jtty_frame_launch();  
  
        // Return to hunting immediately; the decode state machine keeps  
        // running in the background on subsequent service calls.  
        g_hunting = true;  
        g_data_count = 0;  
        g_hops_since_sync = 0;  
        jtty_clear_sync_history();  
    }  
}
