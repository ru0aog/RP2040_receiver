// rx_ax25.cpp — AX.25 300bd decoder for RP2040_receiver.  
// Bell 202 AFSK: mark 1200 Hz, space 2200 Hz.  
// Pipeline: two-tone matched filter -> slicer (AGC) -> DPLL bit clock  
//           -> NRZI -> HDLC (flag/destuff/abort) -> FCS CRC-16.  
// References: refs/demod_afsk.c (nudge_pll), refs/hdlc_rec.c (pat_det),  
//             refs/hdlc.c (FCS, 0xf0b8 magic).  
  
#include "rx_ax25.h"  
#include "src/modes/rx_decoders.h"  
  
#include <Arduino.h>  
#include <stdint.h>  
#include <string.h>  
#include <math.h>  
  
// #define AX25_DEBUG  
  
// -----------------------------------------------------------------------------  
// Protocol parameters  
// -----------------------------------------------------------------------------  
static constexpr float    AX25_FS        = 15625.0f;  
static constexpr float    AX25_BAUD      = 300.0f;  
static constexpr float    AX25_MARK_HZ   = 1200.0f;  
static constexpr float    AX25_SPACE_HZ  = 2200.0f;  
static constexpr float    AX25_LP_ALPHA  = 0.20f;    // matched-filter LP, ~50 Hz  
  
static constexpr int      AX25_MAX_FRAME = 330;  
static constexpr uint16_t AX25_FCS_MAGIC = 0xF0B8;   // residue of valid frame+FCS  
  
// DPLL constants (from refs/demod_afsk.c nudge_pll)  
static constexpr float    PLL_LOCKED     = 0.74f;  
static constexpr float    PLL_SEARCHING  = 0.50f;  
static constexpr int      PLL_GOOD_BITS  = 8;  
  
// -----------------------------------------------------------------------------  
// Matched filter: mixer to DC + phase rotator + one-pole LP (per tone)  
// -----------------------------------------------------------------------------  
typedef struct {  
    float osc_re, osc_im;   // running phasor at tone frequency  
    float rot_re, rot_im;   // per-sample rotation e^{j*2*pi*f/fs}  
    float lp_re, lp_im;     // low-passed complex envelope  
} tone_t;  
  
static tone_t g_tone_mark;  
static tone_t g_tone_space;  
  
static void ax25_tone_init(tone_t* t, float freq_hz) {  
    t->osc_re = 1.0f; t->osc_im = 0.0f;  
    const float w = 2.0f * (float)M_PI * freq_hz / AX25_FS;  
    t->rot_re = cosf(w); t->rot_im = sinf(w);  
    t->lp_re = 0.0f; t->lp_im = 0.0f;  
}  
  
static float ax25_tone_push(tone_t* t, float x) {  
    // y = x * osc  (real input, xq = 0)  
    const float yr = x * t->osc_re;  
    const float yi = x * t->osc_im;  
    const float nr = t->osc_re * t->rot_re - t->osc_im * t->rot_im;  
    const float ni = t->osc_re * t->rot_im + t->osc_im * t->rot_re;  
    t->osc_re = nr; t->osc_im = ni;  
    t->lp_re += AX25_LP_ALPHA * (yr - t->lp_re);  
    t->lp_im += AX25_LP_ALPHA * (yi - t->lp_im);  
    return t->lp_re * t->lp_re + t->lp_im * t->lp_im;  
}  
  
// -----------------------------------------------------------------------------  
// State  
// -----------------------------------------------------------------------------  
static bool      g_active = false;  
  
// AGC / slicer  
static float     g_peak   = 100.0f;  
static float     g_valley = -100.0f;  
  
// DPLL  
static uint32_t  g_dpll = 0;  
static uint32_t  g_dpll_step = 0;  
static int       g_pll_quality = 0;  
static uint8_t   g_prev_tone = 0;  
  
// HDLC  
static uint8_t   g_prev_raw = 0;  
static uint8_t   g_pat_det = 0;  
static int       g_olen = 0;  
static uint8_t   g_oacc = 0;  
static uint8_t   g_frame_buf[AX25_MAX_FRAME];  
static int       g_frame_len = 0;  
  
// -----------------------------------------------------------------------------  
// FCS CRC-16-CCITT, reflected poly 0x8408, init/xorout 0xFFFF.  
// Feeding frame incl. FCS bytes yields AX25_FCS_MAGIC on valid frames.  
// -----------------------------------------------------------------------------  
static uint16_t ax25_fcs_update(uint16_t crc, uint8_t b) {  
    crc ^= b;  
    for (int i = 0; i < 8; ++i) {  
        crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408) : (uint16_t)(crc >> 1);  
    }  
    return crc;  
}  
  
// -----------------------------------------------------------------------------  
// Frame output: unpack address field + control/PID + info, emit as text.  
// -----------------------------------------------------------------------------  
static void ax25_emit_callsign(const uint8_t* p) {  
    for (int i = 0; i < 6; ++i) {  
        char c = (char)(p[i] >> 1);  
        if (c != ' ') rx_decoders_emit(c);  
    }  
    const int ssid = (p[6] >> 1) & 0x0F;  
    if (ssid) {  
        rx_decoders_emit('-');  
        if (ssid >= 10) rx_decoders_emit((char)('0' + ssid / 10));  
        rx_decoders_emit((char)('0' + ssid % 10));  
    }  
}  
  
static void ax25_frame_finish(void) {  
    uint16_t crc = 0xFFFF;  
    for (int i = 0; i < g_frame_len; ++i) {  
        crc = ax25_fcs_update(crc, g_frame_buf[i]);  
    }  
    if (crc != AX25_FCS_MAGIC) {  
#ifdef AX25_DEBUG  
        Serial.printf("# AX25 FCS bad len=%d crc=%04X\n", g_frame_len, crc);  
#endif  
        return;  
    }  
    if (g_pll_quality < PLL_GOOD_BITS * 2) ++g_pll_quality;  
  
    const int payload = g_frame_len - 2;              // strip FCS  
    if (payload < 15) return;                          // min: 2 addr + ctrl + PID  
  
    ax25_emit_callsign(g_frame_buf + 7);               // source  
    rx_decoders_emit('>');  
    ax25_emit_callsign(g_frame_buf);                   // destination  
  
    int i = 14;  
    while (!(g_frame_buf[i - 1] & 0x01) && i + 7 <= payload) {  // digipeaters  
        rx_decoders_emit(',');  
        ax25_emit_callsign(g_frame_buf + i);  
        i += 7;  
    }  
    rx_decoders_emit(':');  
  
    // i -> control, i+1 -> PID (skip both for UI frames)  
    i += 2;  
    for (; i < payload; ++i) {  
        uint8_t c = g_frame_buf[i];  
        rx_decoders_emit((c >= 0x20 && c < 0x7F) ? (char)c : '.');  
    }  
    rx_decoders_emit('\n');  
}  
  
// -----------------------------------------------------------------------------  
// HDLC bit consumer — port of hdlc_rec_bit (refs/hdlc_rec.c)  
// -----------------------------------------------------------------------------  
static void ax25_hdlc_bit(uint8_t raw) {  
    // NRZI: transition = 0, no transition = 1  
    const int dbit = (raw == g_prev_raw);  
    g_prev_raw = raw;  
  
    g_pat_det = (uint8_t)((g_pat_det << 1) | (dbit & 1));  
  
    if (g_pat_det == 0x7E) {  
        // Flag: end of frame (if any bits accumulated)  
        if (g_frame_len >= 2 + 15) {  
            ax25_frame_finish();  
        }  
        g_frame_len = 0;  
        g_olen = 0;  
        g_oacc = 0;  
        return;  
    }  
    if (g_pat_det == 0xFE) {  
        // Seven '1' bits: abort / loss of signal — drop frame in progress  
        g_frame_len = 0;  
        g_olen = 0;  
        return;  
    }  
    if ((g_pat_det & 0xFC) == 0x7C) {  
        // Five '1' then '0': stuffed bit — discard, keep accumulating  
        return;  
    }  
  
    // Normal data bit  
    g_oacc = (uint8_t)(g_oacc >> 1);  
    if (dbit) g_oacc |= 0x80;  
    ++g_olen;  
  
    if (g_olen >= 8) {  
        g_olen = 0;  
        if (g_frame_len < AX25_MAX_FRAME) {  
            g_frame_buf[g_frame_len++] = g_oacc;  
        }  
        g_oacc = 0;  
    }  
}  
  
// -----------------------------------------------------------------------------  
// Public API  
// -----------------------------------------------------------------------------  
void rx_ax25_init(float center_hz) {  
    (void)center_hz;   // tones are fixed Bell 202 frequencies  
  
    ax25_tone_init(&g_tone_mark,  AX25_MARK_HZ);  
    ax25_tone_init(&g_tone_space, AX25_SPACE_HZ);  
  
    g_dpll_step   = (uint32_t)(4294967296.0 * AX25_BAUD / AX25_FS);  // ~82.5e6  
    g_dpll        = 0;  
    g_peak        = 100.0f;  
    g_valley      = -100.0f;  
    g_pll_quality = 0;  
    g_prev_tone   = 0;  
    g_prev_raw    = 0;  
    g_pat_det     = 0;  
    g_olen        = 0;  
    g_oacc        = 0;  
    g_frame_len   = 0;  
    g_active      = true;  
  
#ifdef AX25_DEBUG  
    Serial.printf("# AX25 init center=%.1f mark=%.0f space=%.0f\n",  
                  center_hz, AX25_MARK_HZ, AX25_SPACE_HZ);  
#endif  
}  
  
void rx_ax25_stop(void) {  
    g_active = false;  
    // Reset AGC/PLL so next session doesn't inherit skewed levels or stale lock.  
    g_peak        = 100.0f;  
    g_valley      = -100.0f;  
    g_pll_quality = 0;  
    g_prev_raw    = 0;  
    g_pat_det     = 0;  
    g_olen        = 0;  
    g_oacc        = 0;  
    g_frame_len   = 0;  
}  
  
void rx_ax25_push(int16_t m) {  
    if (!g_active) return;  
  
    const float pm = ax25_tone_push(&g_tone_mark,  (float)m);  
    const float ps = ax25_tone_push(&g_tone_space, (float)m);  
  
    // Slicer input: mark positive, space negative  
    const float d = pm - ps;  
  
    // AGC: attack fast / decay slow (refs/demod_afsk.c)  
    if (d > g_peak)   g_peak   += (d - g_peak)   * 0.5f;  
    else              g_peak   += (d - g_peak)   * 0.02f;  
    if (d < g_valley) g_valley += (d - g_valley) * 0.5f;  
    else              g_valley += (d - g_valley) * 0.02f;  
  
    const float mid = (g_peak + g_valley) * 0.5f;  
    const uint8_t tone = (d > mid) ? 1u : 0u;  
  
    // DPLL: strobe bit on wrap FIRST, then nudge on transition.  
    const uint32_t prev_dpll = g_dpll;  
    g_dpll += g_dpll_step;  
    if (g_dpll < prev_dpll) {                     // wrapped -> sample bit  
        ax25_hdlc_bit(tone);  
    }  
    if (tone != g_prev_tone) {                    // transition -> nudge phase  
        const int32_t err = (int32_t)g_dpll;  
        const float inertia = (g_pll_quality >= PLL_GOOD_BITS)  
                            ? PLL_LOCKED : PLL_SEARCHING;  
        g_dpll -= (uint32_t)(inertia * (float)err);  
    }  
    g_prev_tone = tone;  
}
