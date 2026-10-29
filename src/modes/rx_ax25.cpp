
// rx_ax25.cpp — AX.25 / HDLC decoder for RP2040_receiver
//
// Input: mono audio, 15625 samples/s
// Baud: 300
//
// Signal chain:
//   tone correlators -> discriminator -> hysteresis slicer
//   -> DPLL -> NRZI -> HDLC flag detector
//   -> bit destuffing -> byte assembly -> FCS
//
// CRC-16/X.25:
//   reflected polynomial = 0x8408
//   initial CRC = 0xFFFF
//   good residue including FCS = 0xF0B8

#include "../modes/rx_ax25.h"
#include "../modes/rx_decoders.h"

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

static constexpr float AX25_FS       = 15625.0f;
static constexpr float AX25_BAUD     = 300.0f;
static constexpr float AX25_LP_ALPHA = 0.04f;

static constexpr float AX25_SHIFT_HZ = 200.0f;
static constexpr float AX25_HYST_FRACTION = 0.10f;

static constexpr int AX25_MAX_FRAME = 330;

static constexpr uint16_t AX25_FCS_MAGIC = 0xF0B8;

#define AX25_DEBUG 1
#define AX25_FIXED_MIDPOINT 1

static constexpr float PLL_SEARCHING = 0.25f;
static constexpr float PLL_LOCKED    = 0.10f;
static constexpr int PLL_GOOD_BITS   = 8;

// -----------------------------------------------------------------------------
// Diagnostics
// -----------------------------------------------------------------------------

#if AX25_DEBUG
static uint32_t dbg_samples = 0;
static uint32_t dbg_bits = 0;
static uint32_t dbg_flags = 0;
static uint32_t dbg_stuff = 0;
static uint32_t dbg_abort = 0;
static uint32_t dbg_fcs_bad = 0;
static uint32_t dbg_fcs_ok = 0;
static uint32_t dbg_trans = 0;

static uint32_t dbg_run_sum = 0;
static uint32_t dbg_run_n = 0;

static float dbg_dmin = 1.0e30f;
static float dbg_dmax = -1.0e30f;
#endif

// -----------------------------------------------------------------------------
// Tone correlators
// -----------------------------------------------------------------------------

struct ToneCorrelator {
    float osc_re;
    float osc_im;

    float rot_re;
    float rot_im;

    float lp_re;
    float lp_im;
};

static ToneCorrelator g_mark;
static ToneCorrelator g_space;

static void tone_init(ToneCorrelator* t, float frequency_hz)
{
    t->osc_re = 1.0f;
    t->osc_im = 0.0f;

    const float w =
        2.0f * (float)M_PI * frequency_hz / AX25_FS;

    t->rot_re = cosf(w);
    t->rot_im = sinf(w);

    t->lp_re = 0.0f;
    t->lp_im = 0.0f;
}

static float tone_push(ToneCorrelator* t, float x)
{
    const float yr = x * t->osc_re;
    const float yi = x * t->osc_im;

    const float nr =
        t->osc_re * t->rot_re -
        t->osc_im * t->rot_im;

    const float ni =
        t->osc_re * t->rot_im +
        t->osc_im * t->rot_re;

    t->osc_re = nr;
    t->osc_im = ni;

    t->lp_re += AX25_LP_ALPHA * (yr - t->lp_re);
    t->lp_im += AX25_LP_ALPHA * (yi - t->lp_im);

    return t->lp_re * t->lp_re +
           t->lp_im * t->lp_im;
}

// -----------------------------------------------------------------------------
// Receiver state
// -----------------------------------------------------------------------------

static bool g_active = false;

static float g_peak = 100.0f;
static float g_valley = -100.0f;

static uint8_t g_tone_state = 0;

static uint32_t g_dpll = 0;
static uint32_t g_dpll_step = 0;

static int g_pll_quality = 0;

static uint8_t g_prev_tone = 0;
static uint8_t g_prev_raw = 0;

// -----------------------------------------------------------------------------
// HDLC state
//
// The eight-bit sliding window delays delivery to the data decoder.
// A candidate flag is recognized before its bits can be delivered
// to the destuffer or byte assembler.
//
// g_in_abort:
//   true after seven consecutive data ones;
//   data is ignored until the next flag.
// -----------------------------------------------------------------------------

static uint8_t g_hdlc_window = 0;
static uint8_t g_window_bits = 0;

static uint8_t g_ones = 0;
static bool g_in_abort = false;

// Byte assembly, LSB-first.
static uint8_t g_oacc = 0;
static uint8_t g_olen = 0;

// Frame storage.
static uint8_t g_frame_buf[AX25_MAX_FRAME];
static int g_frame_len = 0;

// -----------------------------------------------------------------------------
// CRC-16/X.25
// -----------------------------------------------------------------------------

static uint16_t ax25_fcs_update(uint16_t crc, uint8_t b)
{
    crc ^= b;

    for (int i = 0; i < 8; ++i) {
        if (crc & 1u) {
            crc = (uint16_t)((crc >> 1) ^ 0x8408u);
        } else {
            crc = (uint16_t)(crc >> 1);
        }
    }

    return crc;
}

// -----------------------------------------------------------------------------
// Output callsign
// -----------------------------------------------------------------------------

static void ax25_emit_callsign(const uint8_t* p)
{
    for (int i = 0; i < 6; ++i) {
        const char c = (char)(p[i] >> 1);

        if (c != ' ') {
            rx_decoders_emit(c);
        }
    }

    const int ssid = (p[6] >> 1) & 0x0F;

    if (ssid != 0) {
        rx_decoders_emit('-');

        if (ssid >= 10) {
            rx_decoders_emit((char)('0' + ssid / 10));
        }

        rx_decoders_emit((char)('0' + ssid % 10));
    }
}

// -----------------------------------------------------------------------------
// Reset current frame assembly
// -----------------------------------------------------------------------------

static void ax25_frame_reset(void)
{
    g_frame_len = 0;

    g_oacc = 0;
    g_olen = 0;

    g_ones = 0;
}

// -----------------------------------------------------------------------------
// Process one bit known not to be part of a flag
//
// HDLC bit stuffing:
//   after five consecutive data ones, the transmitter inserts a zero;
//   that zero is discarded by the receiver.
//
// HDLC abort:
//   seven consecutive ones abort the frame.
//   Enter abort state once, then ignore data until the next flag.
// -----------------------------------------------------------------------------

static void ax25_process_data_bit(uint8_t dbit)
{
    // After an abort, do not process data until a flag is found.
    if (g_in_abort) {
        return;
    }

    if (dbit != 0) {
        if (g_ones < 7) {
            ++g_ones;
        }

        if (g_ones >= 7) {
#if AX25_DEBUG
            ++dbg_abort;
#endif
            ax25_frame_reset();

            // Prevent the same run of ones from repeatedly counting
            // as separate aborts.
            g_in_abort = true;
            return;
        }
    } else {
        if (g_ones == 5) {
            // Stuffed zero: discard it, not a data bit.
#if AX25_DEBUG
            ++dbg_stuff;
#endif
            g_ones = 0;
            return;
        }

        g_ones = 0;
    }

    // Assemble the de-stuffed data stream, LSB-first.
    g_oacc = (uint8_t)(g_oacc >> 1);

    if (dbit != 0) {
        g_oacc |= 0x80;
    }

    ++g_olen;

    if (g_olen == 8) {
        g_olen = 0;

        if (g_frame_len >= 0 &&
            g_frame_len < AX25_MAX_FRAME) {
            g_frame_buf[g_frame_len++] = g_oacc;
        } else {
            g_frame_len = -1;
        }

        g_oacc = 0;
    }
}

// -----------------------------------------------------------------------------
// Finish and validate an AX.25 frame
// -----------------------------------------------------------------------------

static void ax25_frame_finish(void)
{
    if (g_frame_len < 18) {
        return;
    }

    if (g_frame_len < 0) {
        return;
    }

    uint16_t crc = 0xFFFF;

    for (int i = 0; i < g_frame_len; ++i) {
        crc = ax25_fcs_update(crc, g_frame_buf[i]);
    }

    if (crc != AX25_FCS_MAGIC) {
#if AX25_DEBUG
        ++dbg_fcs_bad;

        Serial.printf(
            "# AX25 FCS bad len=%d res=%04X bytes: ",
            g_frame_len,
            crc
        );

        for (int i = 0; i < g_frame_len; ++i) {
            Serial.printf("%02X", g_frame_buf[i]);
        }

        Serial.println();
#endif
        return;
    }

#if AX25_DEBUG
    ++dbg_fcs_ok;
#endif

    if (g_pll_quality < PLL_GOOD_BITS * 2) {
        ++g_pll_quality;
    }

    // Exclude the two FCS bytes from the information field.
    const int data_len = g_frame_len - 2;

    if (data_len < 16) {
        return;
    }

    const uint8_t* destination = &g_frame_buf[0];
    const uint8_t* source = &g_frame_buf[7];

    // Address field: destination + source, then optional digipeaters.
    int addr_pos = 14;
    const uint8_t* last_addr = source;

    while ((last_addr[6] & 0x01u) == 0u) {
        if (addr_pos + 7 > data_len) {
            return;
        }

        last_addr = &g_frame_buf[addr_pos];
        addr_pos += 7;
    }

    // Need control and PID bytes after the address field.
    if (addr_pos + 2 > data_len) {
        return;
    }

    const uint8_t control = g_frame_buf[addr_pos];

    // Emit UI frames only.
    if (control != 0x03 && control != 0x13) {
        return;
    }

    ax25_emit_callsign(source);
    rx_decoders_emit('>');
    ax25_emit_callsign(destination);

    // Emit digipeater addresses, if present.
    int digi_pos = 14;

    while (digi_pos < addr_pos) {
        rx_decoders_emit(',');
        ax25_emit_callsign(&g_frame_buf[digi_pos]);
        digi_pos += 7;
    }

    rx_decoders_emit(':');

    // Skip control and PID.
    const int payload_pos = addr_pos + 2;

    for (int i = payload_pos; i < data_len; ++i) {
        const uint8_t c = g_frame_buf[i];

        rx_decoders_emit(
            (c >= 0x20 && c < 0x7F) ? (char)c : '.'
        );
    }

    rx_decoders_emit('\n');
}

// -----------------------------------------------------------------------------
// NRZI -> HDLC
//
// NRZI:
//   tone transition = 0
//   no tone transition = 1
//
// HDLC flag on the wire, LSB-first: 0 1 1 1 1 1 1 0
//
// The window is checked before its oldest bit is delivered. If it is
// a flag, all eight bits are consumed as a flag and none enter the frame.
// -----------------------------------------------------------------------------

static void ax25_hdlc_bit(uint8_t raw)
{
    const uint8_t dbit =
        (raw == g_prev_raw) ? 1u : 0u;

    g_prev_raw = raw;

    g_hdlc_window =
        (uint8_t)((g_hdlc_window << 1) | dbit);

    if (g_window_bits < 8) {
        ++g_window_bits;
    }

    if (g_window_bits < 8) {
        return;
    }

    // Flag recognition takes precedence over data/abort processing.
    if (g_hdlc_window == 0x7E) {
#if AX25_DEBUG
        ++dbg_flags;
#endif

        // Repeated flags are reliable timing landmarks even before a valid
        // FCS has been received. Use them to let the DPLL leave acquisition.
        if (g_pll_quality < PLL_GOOD_BITS * 2) {
            ++g_pll_quality;
        }

        // A flag closes the previous frame unless it was aborted.
        if (!g_in_abort) {
            ax25_frame_finish();
        }

        // This flag also establishes the start of the next frame.
        ax25_frame_reset();
        g_in_abort = false;

        // The flag bits must not enter the next frame.
        g_hdlc_window = 0;
        g_window_bits = 0;

        return;
    }

    // This eight-bit window is not a flag.
    // Deliver the oldest bit and keep the remaining seven bits.
    const uint8_t oldest =
        (uint8_t)((g_hdlc_window >> 7) & 1u);

    ax25_process_data_bit(oldest);

    g_hdlc_window &= 0x7F;
    g_window_bits = 7;
}

// -----------------------------------------------------------------------------
// Public initialization
// -----------------------------------------------------------------------------

void rx_ax25_init(float center_hz)
{
    g_active = false;

    const float mark_hz =
        center_hz - AX25_SHIFT_HZ * 0.5f;

    const float space_hz =
        center_hz + AX25_SHIFT_HZ * 0.5f;

    tone_init(&g_mark, mark_hz);
    tone_init(&g_space, space_hz);

    g_dpll_step =
        (uint32_t)(4294967296.0 * AX25_BAUD / AX25_FS);

    g_dpll = 0;
    g_pll_quality = 0;

    g_peak = 100.0f;
    g_valley = -100.0f;

    g_tone_state = 0;

    g_prev_tone = 0;
    g_prev_raw = 0;

    g_hdlc_window = 0;
    g_window_bits = 0;

    g_in_abort = false;

    ax25_frame_reset();

#if AX25_DEBUG
    dbg_samples = 0;
    dbg_bits = 0;
    dbg_flags = 0;
    dbg_stuff = 0;
    dbg_abort = 0;
    dbg_fcs_bad = 0;
    dbg_fcs_ok = 0;
    dbg_trans = 0;

    dbg_run_sum = 0;
    dbg_run_n = 0;

    dbg_dmin = 1.0e30f;
    dbg_dmax = -1.0e30f;

    Serial.printf(
        "# AX25 init center=%.1f mark=%.1f space=%.1f "
        "fs=%.0f baud=%.0f samples/bit=%.3f\n",
        (double)center_hz,
        (double)mark_hz,
        (double)space_hz,
        (double)AX25_FS,
        (double)AX25_BAUD,
        (double)(AX25_FS / AX25_BAUD)
    );
#endif

    g_active = true;
}

// -----------------------------------------------------------------------------
// Public stop
// -----------------------------------------------------------------------------

void rx_ax25_stop(void)
{
    g_active = false;

    g_dpll = 0;
    g_pll_quality = 0;

    g_peak = 100.0f;
    g_valley = -100.0f;

    g_tone_state = 0;

    g_prev_tone = 0;
    g_prev_raw = 0;

    g_hdlc_window = 0;
    g_window_bits = 0;

    g_in_abort = false;

    ax25_frame_reset();
}

// -----------------------------------------------------------------------------
// Public sample input
// -----------------------------------------------------------------------------

void rx_ax25_push(int16_t sample)
{
    if (!g_active) {
        return;
    }

    const float pm = tone_push(&g_mark, (float)sample);
    const float ps = tone_push(&g_space, (float)sample);

    const float d = pm - ps;

#if AX25_DEBUG
    if (d < dbg_dmin) dbg_dmin = d;
    if (d > dbg_dmax) dbg_dmax = d;
#endif

    // Peak and valley tracking.
    if (d > g_peak) {
        g_peak += 0.50f * (d - g_peak);
    } else {
        g_peak += 0.02f * (d - g_peak);
    }

    if (d < g_valley) {
        g_valley += 0.50f * (d - g_valley);
    } else {
        g_valley += 0.02f * (d - g_valley);
    }

#if AX25_FIXED_MIDPOINT
    const float mid = 0.0f;
#else
    const float mid = (g_peak + g_valley) * 0.5f;
#endif

    float range = g_peak - g_valley;

    if (range < 0.0f) {
        range = 0.0f;
    }

    const float hyst = range * AX25_HYST_FRACTION;

    // Hysteresis slicer.
    if (d > mid + hyst) {
        g_tone_state = 1;
    } else if (d < mid - hyst) {
        g_tone_state = 0;
    }

    const uint8_t tone = g_tone_state;

    // DPLL bit strobe.
    const uint32_t prev_phase = g_dpll;
    g_dpll += g_dpll_step;

    if (g_dpll < prev_phase) {
#if AX25_DEBUG
        ++dbg_bits;
#endif
        ax25_hdlc_bit(tone);
    }

    // Correct DPLL phase on slicer transitions.
    if (tone != g_prev_tone) {
#if AX25_DEBUG
        ++dbg_trans;
        dbg_run_sum += dbg_run_n;
        dbg_run_n = 0;
#endif

        // A tone transition marks the beginning of an NRZI bit.
        // The DPLL strobe should fall near the middle of the bit, so
        // immediately after a transition the phase accumulator should
        // be near 0.5 cycle (0x80000000), not zero.
        const int32_t phase_err =
            (int32_t)(g_dpll - 0x80000000u);

        const float gain =
            (g_pll_quality >= PLL_GOOD_BITS)
                ? PLL_LOCKED
                : PLL_SEARCHING;

        int64_t correction =
            (int64_t)((double)gain * (double)phase_err);

        // Limit phase correction to one quarter cycle per transition.
        const int64_t max_corr = (int64_t)1 << 30;

        if (correction > max_corr) {
            correction = max_corr;
        }

        if (correction < -max_corr) {
            correction = -max_corr;
        }

        // Unsigned subtraction intentionally wraps modulo 2^32.
        g_dpll -= (uint32_t)correction;

        g_prev_tone = tone;
    }

#if AX25_DEBUG
    ++dbg_run_n;

    if (++dbg_samples >= (uint32_t)AX25_FS) {
        const uint32_t sample_count = dbg_samples;

        const float samples_per_bit =
            dbg_bits
                ? (float)sample_count / (float)dbg_bits
                : 0.0f;

        const float samples_per_transition =
            dbg_trans
                ? (float)dbg_run_sum / (float)dbg_trans
                : 0.0f;

        Serial.printf(
            "# AX25 d[%.0f..%.0f] pk=%.0f vl=%.0f mid=%.1f "
            "bits=%lu smp/bit=%.2f smp/trans=%.2f "
            "flags=%lu stuff=%lu abort=%lu "
            "fcs_ok=%lu fcs_bad=%lu pll=%d\n",
            (double)dbg_dmin,
            (double)dbg_dmax,
            (double)g_peak,
            (double)g_valley,
            (double)mid,
            (unsigned long)dbg_bits,
            (double)samples_per_bit,
            (double)samples_per_transition,
            (unsigned long)dbg_flags,
            (unsigned long)dbg_stuff,
            (unsigned long)dbg_abort,
            (unsigned long)dbg_fcs_ok,
            (unsigned long)dbg_fcs_bad,
            g_pll_quality
        );

        dbg_samples = 0;
        dbg_bits = 0;
        dbg_trans = 0;
        dbg_run_sum = 0;

        dbg_dmin = 1.0e30f;
        dbg_dmax = -1.0e30f;
    }
#endif
}
