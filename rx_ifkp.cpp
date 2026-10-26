// rx_ifkp.cpp — декодер IFKP (incremental FSK, 33 тона, шаг 11.697 Гц).  
// Переиспользует FFT из rx_rsid (11025 Гц, 1024 бина, окно ~46 мс).  
// Символ ~500 мс; argmax тона на границе символа -> дельта IFK+ -> символ.  
// Таблица — инверсия кодера маяка (ifkp_modem.cpp). Вывод: rx_decoders_emit.  
#include <Arduino.h>  
#include <math.h>  
#include <string.h>  
#include "rx_ifkp.h"  
#include "rx_decoders.h"
#include "rx_goertzel.h"
  
#define IFKP_NTONES        33  
#define IFKP_STEP_HZ       11.697f  
#define IFKP_OFFSET_HZ     (16.0f * IFKP_STEP_HZ)   // 187.15: центр на 16-м тоне  
// Детект символа «по кромке» как в FLDigi process_tones(): пик должен  
// продержаться IFKP_PEAK_HITS окон подряд и отличаться от прошлого
#define IFKP_PEAK_HITS     2  
// Сглаживание энергии тона между окнами (аналог binfilt Cmovavg в FLDigi).  
#define IFKP_SMOOTH_A      0.3f   // ~ movavg 4 окна (как binfilt FLDigi для IFKP 0.5)
  
static bool   active = false;  
static float  base_hz = 0.0f;  
  
static float  tpwr[IFKP_NTONES];     // сглаженная энергия каждого тона  
static int    peak = 0, prev_peak = -1, last_peak = -1;  
static int    peak_counter = 0; 
  
static int    prev_tone = -1;  static bool have_prev = false;  
static int    base_delta = -1; static bool have_base = false;  
  
  
void rx_ifkp_init(float center_hz) {  
    base_hz = center_hz - IFKP_OFFSET_HZ;  
    memset(tpwr, 0, sizeof(tpwr));  
    peak = 0; prev_peak = -1; last_peak = -1; peak_counter = 0;  
    prev_tone = -1; have_prev = false;  
    base_delta = -1; have_base = false;  
  // Конфиг банка Гёрцеля (block_n=2048 => полоса fs/N ≈ 7.6 Гц < шага 11.697; ~3.8 блока/символ):  
  gb_cfg_t cfg = { IFKP_NTONES, base_hz, IFKP_STEP_HZ, 15625.0f, 2048 };
    gb_init(&cfg);  
    active = true;  
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

void rx_ifkp_push(int16_t sample) {  
    if (!active) return;  
    static float pwr[IFKP_NTONES];  
    if (!gb_push(sample, pwr)) return;         // ждём границу блока  
  
    int best = 0; float bp = -1.0f;  
    for (int i = 0; i < IFKP_NTONES; i++) {  
        tpwr[i] += (pwr[i] - tpwr[i]) * IFKP_SMOOTH_A;   // сглаживание как было  
        if (tpwr[i] > bp) { bp = tpwr[i]; best = i; }  
    }  
    // детект символа по кромке (peak_counter/last_peak) — без изменений  
    peak = best;  
    if (peak == prev_peak) peak_counter++;  
    else                   peak_counter = 0;  
    if (peak_counter >= IFKP_PEAK_HITS && peak != last_peak) {  
        on_symbol_tone(peak);  
        peak_counter = 0;  
        last_peak = peak;  
    }  
    prev_peak = peak;  
}

