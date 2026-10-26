// rx_ifkp.cpp — декодер IFKP (incremental FSK, 33 тона, шаг 11.697 Гц).  
// Переиспользует FFT из rx_rsid (11025 Гц, 1024 бина, окно ~46 мс).  
// Символ ~500 мс; argmax тона на границе символа -> дельта IFK+ -> символ.  
// Таблица — инверсия кодера маяка (ifkp_modem.cpp). Вывод: rx_decoders_emit.  
#include <Arduino.h>  
#include <math.h>  
#include <string.h>  
#include "rx_ifkp.h"  
#include "rx_decoders.h"  
  
#define IFKP_NTONES        33  
#define IFKP_STEP_HZ       11.697f  
#define IFKP_OFFSET_HZ     (16.0f * IFKP_STEP_HZ)   // 187.15: центр на 16-м тоне  
#define IFKP_SYM_MS        500.0f  
#define IFKP_WIN_MS        46.4f                    // 512/11025 c  
#define IFKP_WINS_PER_SYM  (IFKP_SYM_MS / IFKP_WIN_MS)   // ~10.78  
  
static bool   active = false;  
static float  base_hz = 0.0f, cur_bin_hz = 5.383f;  
static int    tone_bin[IFKP_NTONES];  
  
static float  acc[IFKP_NTONES];  
static float  wins_acc = 0.0f;  
  
static int    prev_tone = -1;  static bool have_prev = false;  
static int    base_delta = -1; static bool have_base = false;  
  
static void compute_bins(float bin_hz) {  
    cur_bin_hz = bin_hz;  
    for (int i = 0; i < IFKP_NTONES; i++) {  
        float f = base_hz + i * IFKP_STEP_HZ;  
        int b = (int)(f / bin_hz + 0.5f);  
        tone_bin[i] = (b < 0) ? 0 : b;  
    }  
}  
  
void rx_ifkp_init(float center_hz) {  
    base_hz = center_hz - IFKP_OFFSET_HZ;  
    memset(acc, 0, sizeof(acc));  
    wins_acc = 0.0f;  
    prev_tone = -1; have_prev = false;  
    base_delta = -1; have_base = false;  
    active = true;  
    compute_bins(cur_bin_hz);  
}  
  
void rx_ifkp_stop(void) { active = false; }  
  
// Одноделтовые: 1..26->a..z, 27->'.', 28->' '.  
static char decode_single(int d) {  
    if (d >= 1 && d <= 26) return (char)('a' + d - 1);  
    if (d == 27) return '.';  
    if (d == 28) return ' ';  
    return 0;  
}  
// Двухделтовые: base + модификатор (29/30/31).  
static char decode_pair(int base, int mod) {  
    if (mod == 29) {                       // ЗАГЛАВНЫЕ + @ , ?  
        if (base >= 1 && base <= 26) return (char)('A' + base - 1);  
        if (base == 0)  return '@';  
        if (base == 27) return ',';  
        if (base == 28) return '?';  
    } else if (mod == 30) {                // цифры + символы  
        if (base >= 1 && base <= 9) return (char)('1' + base - 1);  
        if (base == 10) return '0';  
        if (base == 28) return '\n';  
        if (base == 0)  return '~';  
        if (base == 11) return '!'; if (base == 12) return '"';  
        if (base == 13) return '#'; if (base == 14) return '$';  
        if (base == 15) return '%'; if (base == 16) return '&';  
        if (base == 17) return '\''; if (base == 18) return '(';  
        if (base == 19) return ')'; if (base == 20) return '*';  
        if (base == 21) return '+'; if (base == 22) return '-';  
        if (base == 23) return '/'; if (base == 24) return ':';  
        if (base == 25) return ';'; if (base == 26) return '<';  
        if (base == 27) return '>';  
    } else if (mod == 31) {                // символы-2  
        if (base == 0) return '='; if (base == 1) return '[';  
        if (base == 2) return '\\'; if (base == 3) return ']';  
        if (base == 5) return '_'; if (base == 6) return '{';  
        if (base == 7) return '|'; if (base == 8) return '}';  
        if (base == 9) return '`';  
    }  
    return 0;  
}  
  
static void on_delta(int d) {  
    if (d == 29 || d == 30 || d == 31) {       // модификатор завершает пару  
        if (have_base) {  
            char c = decode_pair(base_delta, d);  
            if (c) rx_decoders_emit(c);  
            have_base = false;  
        }  
        return;  
    }  
    if (have_base) {                           // предыдущий base был одиночным  
        char c = decode_single(base_delta);  
        if (c) rx_decoders_emit(c);  
    }  
    base_delta = d; have_base = true;  
}  
  
static void on_symbol_tone(int tone) {  
    if (!have_prev) { prev_tone = tone; have_prev = true; return; }  
    int d = tone - prev_tone - 1;              // offset=1  
    while (d < 0) d += IFKP_NTONES;  
    d %= IFKP_NTONES;  
    prev_tone = tone;  
    on_delta(d);  
}  
  
void rx_ifkp_on_fft(const uint64_t* ampl, int nbins, float bin_hz) {  
    if (!active) return;  
    if (fabsf(bin_hz - cur_bin_hz) > 1e-3f) compute_bins(bin_hz);  
  
    for (int i = 0; i < IFKP_NTONES; i++) {    // мощность тона = бин ±1  
        int b = tone_bin[i];  
        if (b < 1 || b >= nbins - 1) continue;  
        acc[i] += (float)(ampl[b-1] + ampl[b] + ampl[b+1]);  
    }  
    wins_acc += 1.0f;  
    if (wins_acc >= IFKP_WINS_PER_SYM) {       // граница символа  
        wins_acc -= IFKP_WINS_PER_SYM;  
        int best = -1; float bp = -1.0f;  
        for (int i = 0; i < IFKP_NTONES; i++)  
            if (acc[i] > bp) { bp = acc[i]; best = i; }  
        memset(acc, 0, sizeof(acc));  
        if (best >= 0) on_symbol_tone(best);  
    }  
}
