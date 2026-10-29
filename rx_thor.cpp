// rx_thor.cpp — THOR-11 decoder for RP2040_receiver.  
//  
// Сигнал (RP2040_beacon/thor_modem.cpp, fldigi thor.cxx):  
//   18 тонов, шаг 10.7666 Гц (11025/1024), символ 92879 мкс (~1451 сэмпл);  
//   IFK+:  tone = (prev_tone + sym + 2) % 18   ->  sym = (tone - prev - 2) mod 18;  
//   FEC:   бит -> свёрточный k=7 (poly1=0x6d, poly2=0x4f) -> 2 бита ->  
//          пачки по 4 бита -> перемежитель 4x10 -> символ (nibble);  
//   текст: varicode IZ8BLY (primary), граница символа — два нуля подряд.  
//  
// Конвейер приёма:  
//   банк Гёрцеля (gb_push, block=256) -> argmax тона -> IFK-де-дельта ->  
//   nibble -> обратный перемежитель -> 4 бита -> soft Viterbi (k=7) ->  
//   биты varicode -> символ -> rx_decoders_emit.  
//  
// Синхронизация символов — «по кромке» как в rx_ifkp: пик тона должен  
// продержаться THOR_PEAK_HITS окон подряд и смениться.  
  
#include "rx_thor.h"  
#include "src/modes/rx_decoders.h"
#include "src/dsp/rx_goertzel.h"  

  
#include <Arduino.h>  
#include <stdint.h>  
#include <string.h>  
#include <math.h>  
  
// #define THOR_DEBUG  
  
// ---------------------------------------------------------------------------  
// Параметры протокола  
// ---------------------------------------------------------------------------  
static constexpr float   THOR_FS          = 15625.0f;  
static constexpr int     THOR_NTONES      = 18;  
static constexpr float   THOR_STEP_HZ     = 10.7666f;          // 11025/1024  
static constexpr float   THOR_CENTER_OFFS = 8.5f * THOR_STEP_HZ; // метка — центр сетки  
static constexpr int     THOR_IFK_OFFSET  = 2;                  // IFK+  
static constexpr int     THOR_PEAK_HITS   = 2;                  // окон стабильного пика  
static constexpr float   THOR_SMOOTH_A    = 0.4f;  
  
// Символ ~1451 сэмпл; блок 256 сэмплов (~16.4 мс) -> ~5.7 блока на символ.  
static constexpr int     THOR_BLOCK_N     = 256;  
  
// FEC  
static constexpr int     THOR_K           = 7;  
static constexpr uint8_t THOR_POLY1       = 0x6d;  
static constexpr uint8_t THOR_POLY2       = 0x4f;  
static constexpr int     THOR_NSTATES     = 1 << (THOR_K - 1);  // 64  
  
// Перемежитель 4x10 (как у маяка: THOR_IL_SIZE x THOR_IL_DEPTH)  
static constexpr int     THOR_IL_SIZE     = 4;  
static constexpr int     THOR_IL_DEPTH    = 10;  
  
// Мягкий Витерби: метрика Хэмминга по 2 кодовым битам на входной бит.  
// Буфер декодирования — ограничиваем длину сообщения в битах.  
static constexpr int     THOR_MAX_BITS    = 2048;  
  
// ---------------------------------------------------------------------------  
// Varicode — инверсия таблицы маяка.  
// Собираем биты до разделителя «00» и ищем код в таблице.  
// ---------------------------------------------------------------------------  
static const char *thor_rx_varicode[] = {  
    "11101011100", "11101100000", "11101101000", "11101101100",  
    "11101110000", "11101110100", "11101111000", "11101111100",  
    "10101000",    "11110000000", "11110100000", "11110101000",  
    "11110101100", "10101100",    "11110110000", "11110110100",  
    "11110111000", "11110111100", "11111000000", "11111001000",  
    "11111010100", "11111011000", "11111100000", "11111101000",  
    "11111101100", "11111110100", "11111111000", "11111111100",  
    "100000000000","101000000000",  
    "100",         "111000000",   "111111100",   "1011011000",  
    "1010101000",  "1010100000",  "1000000000",  "110111100",  
    "111110100",   "111110000",   "1010110100",  "10100000",  
    "111011000",   "111010100",   "111101000",   "11100000",  
    "11110000",    "101000000",   "101010100",   "101110100",  
    "101100000",   "101101100",   "110100000",   "110000000",  
    "110101100",   "111101100",   "111111000",   "1011000000",  
    "111011100",   "1010111100",  "111010000",   "1010000000",  
    "10111100",    "100000000",   "11010100",    "11011100",  
    "10111000",    "11111000",    "101010000",   "101011000",  
    "11000000",    "110110100",   "101111100",   "11110100",  
    "11101000",    "11111100",    "11011000",    "10110100",  
    "10110000",    "110101000",   "101101000",   "10110000",  
    "101011100",   "110101000",   "101101000",   "101111000",  
    "110111000",   "1011101000",  "1011010000",  "1011101100",  
    "1011010100",  "1010110000",  "1011101100",  "1011100100",  
    // a..z  
    "10100",       "1100000",     "111000",      "110100",  
    "1000",        "1011000",     "11000",       "10000000",  
    "101100",      "100000",      "11100",       "10000",  
    "1010100",     "100000",      "101000",      "1100",  
    "1110100",     "1101100",     "1101000",     "1110100",  
    "1011100",     "1101000",     "11011100",  
    // 123..126 + DEL  
    "1011011100",  "1010111000",  "1011100000",  "1011110000",  
    "101010000000"  
};  
#define THOR_VARICODE_N 128  
  
// ---------------------------------------------------------------------------  
// Состояние  
// ---------------------------------------------------------------------------  
static bool   g_active = false;  
static float  g_base_hz = 0.0f;  
  
static float  tpwr[THOR_NTONES];  
static int    peak = 0, prev_peak = -1, last_peak = -1;  
static int    peak_counter = 0;  
  
static int    prev_tone = -1;  
static bool   have_prev = false;  
  
// Обратный перемежитель: та же структура, что у TX (il_tab[k][i][j]),  
// только выходная колонка берётся REV: psyms[i] = tab[k][i][i].  
static uint8_t il_table[THOR_IL_DEPTH * THOR_IL_SIZE * THOR_IL_SIZE];  
static inline uint8_t* il_tab(int k, int i, int j) {  
    return &il_table[(THOR_IL_SIZE * THOR_IL_SIZE * k) + (THOR_IL_SIZE * i) + j];  
}  
static void il_rev_bits(uint8_t *sym4 /*4 бита -> 4 бита*/) {  
    uint8_t out[THOR_IL_SIZE];  
    for (int k = 0; k < THOR_IL_DEPTH; k++) {  
        for (int i = 0; i < THOR_IL_SIZE; i++)  
            for (int j = 0; j < THOR_IL_SIZE - 1; j++)  
                *il_tab(k, i, j) = *il_tab(k, i, j + 1);  
        for (int i = 0; i < THOR_IL_SIZE; i++)  
            *il_tab(k, i, THOR_IL_SIZE - 1) = sym4[i];  
        for (int i = 0; i < THOR_IL_SIZE; i++)  
            out[i] = *il_tab(k, i, i);          // REV: колонка i  
    }  
    memcpy(sym4, out, THOR_IL_SIZE);  
}  
  
// Витерби (hard-decision, метрика Хэмминга), k=7 -> 64 состояния.  
// Приём битов идёт парой: (o0,o1) на один информационный бит.  
static int      g_vit_pairs   = 0;                   // принято кодовых пар  
static uint8_t  g_vit_sym[THOR_MAX_BITS];            // 2 бита на пару  
static int      g_dec_bits_n  = 0;  
  
// Переменные Витерби  
static uint16_t vit_metric[THOR_NSTATES];  
static uint8_t  vit_prev[THOR_MAX_BITS / 4][THOR_NSTATES]; // выбор предка на шаг  
// note: один шаг Витерби = один информационный бит = 2 кодовых бита = половина nibble.  
// Храним предков на каждый информационный бит.  
  
static inline uint8_t parity8(uint8_t x) { return (uint8_t)__builtin_parity(x); }  
  
// Восстановленный кодовый выход кодера для (state,input bit):  
// shreg = (state<<1)|bit, out0=parity(POLY1&shreg), out1=parity(POLY2&shreg)  
static inline uint8_t enc_out(int state, int bit) {  
    uint8_t sh = (uint8_t)((state << 1) | bit);  
    return (uint8_t)(parity8(sh & THOR_POLY1) | (parity8(sh & THOR_POLY2) << 1));  
}  
  
// ---------------------------------------------------------------------------  
// Varicode-накопитель: биты до «00», потом поиск в таблице.  
// Два подряд нуля — разделитель; регистр VC_MAX_BITS глубины хватает на коды ≤12.  
// ---------------------------------------------------------------------------  
static uint32_t vc_shreg = 0;   // накопленные биты текущего кода (LSB-first append)  
static int      vc_nbits = 0;  
static int      vc_zeros = 0;   // хвостовые нули  
  
static void vc_emit_code(void) {  
    // Сравнение: код — строка '0'/'1'; проверяем точное совпадение бит.  
    for (int c = 0; c < THOR_VARICODE_N; c++) {  
        const char *s = thor_rx_varicode[c];  
        int len = (int)strlen(s);  
        if (len != vc_nbits) continue;  
        bool match = true;  
        for (int b = 0; b < len; b++) {  
            int bit = (vc_shreg >> b) & 1;            // первый бит = LSB  
            if (bit != (s[b] - '0')) { match = false; break; }  
        }  
        if (match) {  
            if (c >= 32) rx_decoders_emit((char)c);   // печатные  
            vc_shreg = 0; vc_nbits = 0; vc_zeros = 0;  
            return;  
        }  
    }  
    vc_shreg = 0; vc_nbits = 0; vc_zeros = 0;         // код не найден — сброс  
}  
  
static void vc_push_bit(uint8_t bit) {  
    if (bit == 0) {  
        vc_zeros++;  
        if (vc_zeros >= 2 && vc_nbits > 0) vc_emit_code();  
        return;                                      // разделитель не копим  
    }  
    vc_zeros = 0;  
    if (vc_nbits < 24) {  
        vc_shreg |= (1u << vc_nbits);  
        vc_nbits++;  
    } else {  
        vc_shreg = 0; vc_nbits = 0;                  // переполнение — мусор  
    }  
}  
  
// ---------------------------------------------------------------------------  
// Витерби: добавить кодовую пару (2 бита) и, при накоплении хвоста, декодировать.  
// Для простоты — потоковый вариант: накапливаем пары и периодически прогоняем  
// полный traceback по буферу, эмитя биты старше глубины 4*K.  
// ---------------------------------------------------------------------------  
static void vit_process(void) {  
    // Полный проход по накопленным парам -> информационные биты.  
    // Буфер ограничен THOR_MAX_BITS/2 информационных бит.  
    int npairs = g_vit_pairs;  
    memset(vit_metric, 0x3F, sizeof(vit_metric));     // большая метрика  
    vit_metric[0] = 0;                                 // старт state=0 (как у TX)  
  
    for (int t = 0; t < npairs; t++) {  
        uint8_t rx = g_vit_sym[t];                     // 2 кодовых бита  
        uint16_t nm[THOR_NSTATES];  
        memset(nm, 0x3F, sizeof(nm));  
        for (int s = 0; s < THOR_NSTATES; s++) {  
            if (vit_metric[s] > 0x3E00) continue;  
            for (int b = 0; b < 2; b++) {  
                int ns = ((s << 1) | b) & (THOR_NSTATES - 1);  
                uint8_t exp = enc_out(s, b);  
                int dist = ((exp ^ rx) & 1) + (((exp ^ rx) >> 1) & 1);  
                uint16_t m = vit_metric[s] + (uint16_t)dist;  
                if (m < nm[ns]) { nm[ns] = m; vit_prev[t][ns] = (uint8_t)s; }  
            }  
        }  
        memcpy(vit_metric, nm, sizeof(nm));  
    }  
  
    // Traceback от лучшего конечного состояния  
    int best = 0;  
    for (int s = 1; s < THOR_NSTATES; s++)  
        if (vit_metric[s] < vit_metric[best]) best = s;  
  
    static uint8_t bits[THOR_MAX_BITS / 2];  
    int st = best;  
    for (int t = npairs - 1; t >= 0; t--) {  
        int ps = vit_prev[t][st];  
        bits[t] = (uint8_t)(((st >> (THOR_K - 2)) & 1)); // старший новый бит state  
        st = ps;  
    }  
  
    // Эмитим биты, оставляя хвост 4*K на неопределённость.  
    int emit_upto = npairs - 4 * THOR_K;  
    if (emit_upto > g_dec_bits_n) {  
        for (int i = g_dec_bits_n; i < emit_upto; i++)  
            vc_push_bit(bits[i]);  
        g_dec_bits_n = emit_upto;  
        // сдвигаем обратно «шлейф» нет — traceback уже отсчитан от конца.  
    }  
}  
  
// ---------------------------------------------------------------------------  
// Приём nibble-символа -> обратный перемежитель -> 4 бита = 2 кодовые пары.  
// ---------------------------------------------------------------------------  
static void thor_on_symbol(int tone) {  
    if (!have_prev) { prev_tone = tone; have_prev = true; return; }  
  
    int d = tone - prev_tone - THOR_IFK_OFFSET;  
    while (d < 0) d += THOR_NTONES;  
    d %= THOR_NTONES;  
    prev_tone = tone;  
  
    // nibble -> 4 бита (MSB-first, как у TX: syms[i] = bit (IL_SIZE-i-1))  
    uint8_t sym4[THOR_IL_SIZE];  
    for (int i = 0; i < THOR_IL_SIZE; i++)  
        sym4[i] = (uint8_t)((d >> (THOR_IL_SIZE - i - 1)) & 1);  
  
    il_rev_bits(sym4);  
  
    // 4 бита = 2 кодовые пары по 2 бита (порядок TX: o0 потом o1)  
    for (int i = 0; i < THOR_IL_SIZE; i += 2) {  
        if (g_vit_pairs < THOR_MAX_BITS / 2)  
            g_vit_sym[g_vit_pairs] = (uint8_t)(sym4[i] | (sym4[i + 1] << 1));  
        g_vit_pairs++;  
        if (g_vit_pairs >= THOR_MAX_BITS / 2) g_vit_pairs = THOR_MAX_BITS / 2;  
    }  
  
    vit_process();  
  
#ifdef THOR_DEBUG  
    Serial.printf("# THOR sym=%d pairs=%d\n", d, g_vit_pairs);  
#endif  
}  
  
// ---------------------------------------------------------------------------  
// API  
// ---------------------------------------------------------------------------  
void rx_thor_init(float center_hz) {  
    g_active = false;  
    g_base_hz = center_hz - THOR_CENTER_OFFS;  
  
    memset(tpwr, 0, sizeof(tpwr));  
    peak = 0; prev_peak = -1; last_peak = -1; peak_counter = 0;  
    prev_tone = -1; have_prev = false;  
  
    memset(il_table, 0, sizeof(il_table));  
    g_vit_pairs = 0; g_dec_bits_n = 0;  
    vc_shreg = 0; vc_nbits = 0; vc_zeros = 0;  
  
    gb_cfg_t cfg = { THOR_NTONES, g_base_hz, THOR_STEP_HZ, THOR_FS, THOR_BLOCK_N };  
    gb_init(&cfg);  
  
    g_active = true;  
  
#ifdef THOR_DEBUG  
    Serial.printf("# THOR init base=%.1f step=%.4f n=%d\n",  
                  (double)g_base_hz, (double)THOR_STEP_HZ, THOR_NTONES);  
#endif  
}  
  
void rx_thor_stop(void) { g_active = false; }  
  
void rx_thor_push(int16_t m) {  
    if (!g_active) return;  
    static float pwr[THOR_NTONES];  
    if (!gb_push(m, pwr)) return;  
  
    int best = 0; float bp = -1.0f;  
    for (int i = 0; i < THOR_NTONES; i++) {  
        tpwr[i] += (pwr[i] - tpwr[i]) * THOR_SMOOTH_A;  
        if (tpwr[i] > bp) { bp = tpwr[i]; best = i; }  
    }  
  
    peak = best;  
    if (peak == prev_peak) peak_counter++;  
    else                   peak_counter = 0;  
    if (peak_counter >= THOR_PEAK_HITS && peak != last_peak) {  
        thor_on_symbol(peak);  
        peak_counter = 0;  
        last_peak = peak;  
    }  
    prev_peak = peak;  
}