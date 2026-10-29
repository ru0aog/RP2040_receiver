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
//   nibble -> обратный перемежитель -> 4 бита -> Viterbi (k=7, hard) ->  
//   биты varicode -> символ -> rx_decoders_emit.  
//  
// Символьные часы фиксированной длительности; не сбрасываются от каждого изменения тона.  
  
#include "rx_thor.h"  
#include "src/modes/rx_decoders.h"  
#include "src/dsp/rx_goertzel.h"  
  
#include <Arduino.h>  
#include <stdint.h>  
#include <string.h>  
#include <math.h>  
  
#define THOR_DEBUG  
  
// ---------------------------------------------------------------------------  
// Параметры протокола  
// ---------------------------------------------------------------------------  
static constexpr float   THOR_FS          = 15625.0f;  
static constexpr int     THOR_NTONES      = 18;  
static constexpr float   THOR_STEP_HZ     = 10.7666f;            // 11025/1024  
static constexpr float   THOR_CENTER_OFFS = 8.5f * THOR_STEP_HZ; // метка — центр сетки  
static constexpr int     THOR_IFK_OFFSET  = 2;                   // IFK+  
static constexpr float   THOR_SMOOTH_A    = 0.4f;  
  
// Символ 92879 мкс ~ 1451 сэмпл; блок Гёрцеля 256 сэмплов (~16.4 мс)  
// -> ~5.7 блока на символ.  
static constexpr int     THOR_BLOCK_N     = 256;  
static constexpr int     THOR_SYM_SAMP    = 1451;                // отсчётов на символ  
static constexpr int     THOR_SYM_HALF    = THOR_SYM_SAMP / 2;   // строб — середина  
  
// FEC  
static constexpr int     THOR_K           = 7;  
static constexpr uint8_t THOR_POLY1       = 0x6d;  
static constexpr uint8_t THOR_POLY2       = 0x4f;  
static constexpr int     THOR_NSTATES     = 1 << (THOR_K - 1);   // 64  
  
// Перемежитель 4x10 (как у маяка: THOR_IL_SIZE x THOR_IL_DEPTH)  
static constexpr int     THOR_IL_SIZE     = 4;  
static constexpr int     THOR_IL_DEPTH    = 10;  
  
// Максимум информационных бит на сообщение. Один шаг Витерби = 1 инфо-бит  
// = 2 кодовых бита = половина nibble.  
static constexpr int     THOR_MAX_BITS    = 2048;                // информационных бит  
static constexpr int     THOR_MAX_PAIRS   = THOR_MAX_BITS;       // пар по 2 кодовых бита  
  
// ---------------------------------------------------------------------------  
// Varicode — инверсия таблицы маяка (varicode[]/thor_varicode[]).  
// Биты кода считываются LSB-first из vc_shreg: s[b] == бит b.  
// ВНИМАНИЕ: сверить с побитовой инверсией varicode[] маяка — записи  
// 'n'/'t' в исходной таблице были с дубликатами, здесь исправлены.  
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
    // a..z  (97..122)  
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
#define THOR_VARICODE_N (sizeof(thor_rx_varicode) / sizeof(thor_rx_varicode[0]))  
#define VC_MAX_BITS     24        // глубина накопителя varicode  
  
// ---------------------------------------------------------------------------  
// Состояние  
// ---------------------------------------------------------------------------  
static bool   g_active = false;  
static float  g_base_hz = 0.0f;  
  
static float  tpwr[THOR_NTONES];  
static int    last_tone   = -1;   // тон на прошлом блоке (для детекта перехода)  
static int    clk         = 0;    // сэмплы с последнего перехода тона  
static bool   clk_valid   = false;  
  
static int    prev_tone = -1;     // предыдущий принятый символьный тон (IFK)  
static bool   have_prev = false;  
  
// Обратный перемежитель: ОДНА таблица 4x4 (не DEPTH экземпляров).  
// TX-схема: столбцы сдвигаются влево, новый nibble в последний столбец,  
// читается диагональ [i][i]; глубина реализуется очередью по i.  
static uint8_t il_table[THOR_IL_SIZE][THOR_IL_DEPTH];  
// il_table[i][d]: d — задержка в символах, столбец диагонали  
static void il_rev_bits(uint8_t *sym4 /*4 бита -> 4 бита*/) {  
    // Сдвиг задержек: старшие глубины вправо, новый бит i попадает на глубину i.  
    for (int i = 0; i < THOR_IL_SIZE; i++) {  
        for (int d = THOR_IL_DEPTH - 1; d > i; d--)  
            il_table[i][d] = il_table[i][d - 1];  
        il_table[i][i] = sym4[i];  
        sym4[i] = il_table[i][THOR_IL_DEPTH - 1];   // выход — самый «старый»  
    }  
}  
  
// Потоковый Витерби с ограниченной памятью: traceback 96 x 64 = 6144 байта.
static constexpr int THOR_TRACE_DEPTH = 96;
static constexpr uint16_t VIT_INF = 0x3FFF;
static uint16_t vit_metric[THOR_NSTATES];
static uint16_t vit_next[THOR_NSTATES];
static uint8_t vit_trace[THOR_TRACE_DEPTH][THOR_NSTATES];
static uint32_t vit_steps = 0;
static int g_vit_pairs = 0;
static void vc_push_bit(uint8_t bit); // вызывается из Витерби

static inline uint8_t parity8(uint8_t x) { return (uint8_t)__builtin_parity(x); }  
  
// Кодовый выход кодера для (state, input bit):  
// shreg = (state<<1)|bit, out0=parity(POLY1&shreg), out1=parity(POLY2&shreg)  
static inline uint8_t enc_out(int state, int bit) {  
    uint8_t sh = (uint8_t)((state << 1) | bit);  
    return (uint8_t)(parity8(sh & THOR_POLY1) | (parity8(sh & THOR_POLY2) << 1));  
}  
  
// ---------------------------------------------------------------------------  
// Varicode-накопитель: биты до «00», потом поиск в таблице.  
// ---------------------------------------------------------------------------  
static uint32_t vc_shreg = 0;  
static int      vc_nbits = 0;  
static int      vc_zeros = 0;  
  
static void vc_emit_code(void) {  
    for (int c = 0; c < THOR_VARICODE_N; c++) {  
        const char *s = thor_rx_varicode[c];  
        int len = (int)strlen(s);  
        if (len != vc_nbits) continue;  
        bool match = true;  
        for (int b = 0; b < len; b++) {  
            int bit = (vc_shreg >> b) & 1;           // первый бит = LSB  
            if (bit != (s[b] - '0')) { match = false; break; }  
        }  
        if (match) {  
            if (c >= 32) rx_decoders_emit((char)c);  
            vc_shreg = 0; vc_nbits = 0; vc_zeros = 0;  
            return;  
        }  
    }  
    vc_shreg = 0; vc_nbits = 0; vc_zeros = 0;        // код не найден — сброс  
}  
  
static void vc_push_bit(uint8_t bit) {  
    if (bit == 0) {  
        vc_zeros++;  
        if (vc_zeros >= 2 && vc_nbits > 0) vc_emit_code();  
        return;                                     // разделитель не копим  
    }  
    vc_zeros = 0;  
    if (vc_nbits < VC_MAX_BITS) {                   // ИСПРАВЛЕНО: было «24»  
        vc_shreg |= (1u << vc_nbits);  
        vc_nbits++;  
    } else {  
        vc_shreg = 0; vc_nbits = 0;                 // переполнение — мусор  
    }  
}  
  
// ---------------------------------------------------------------------------
// Потоковый Витерби: один шаг на принятую пару кодовых бит.
// В каждом шаге хранится только выбор предшественника (0/1), а не вся история.
static void vit_reset() {
    for (int i = 0; i < THOR_NSTATES; ++i) vit_metric[i] = VIT_INF;
    vit_metric[0] = 0;
    vit_steps = 0;
    g_vit_pairs = 0;
}

static void vit_step(uint8_t rx) {
    const uint32_t step = vit_steps;
    uint8_t *row = vit_trace[step % THOR_TRACE_DEPTH];
    for (int ns = 0; ns < THOR_NSTATES; ++ns) {
        const int bit = ns & 1;
        const int p0 = ns >> 1;
        const int p1 = p0 | 32;
        const uint8_t e0 = enc_out(p0, bit);
        const uint8_t e1 = enc_out(p1, bit);
        const int d0 = ((e0 ^ rx) & 1) + (((e0 ^ rx) >> 1) & 1);
        const int d1 = ((e1 ^ rx) & 1) + (((e1 ^ rx) >> 1) & 1);
        const uint16_t m0 = (vit_metric[p0] >= VIT_INF) ? VIT_INF : (uint16_t)(vit_metric[p0] + d0);
        const uint16_t m1 = (vit_metric[p1] >= VIT_INF) ? VIT_INF : (uint16_t)(vit_metric[p1] + d1);
        if (m1 < m0) { vit_next[ns] = m1; row[ns] = 1; }
        else         { vit_next[ns] = m0; row[ns] = 0; }
    }
    uint16_t mn = VIT_INF;
    for (int i = 0; i < THOR_NSTATES; ++i) if (vit_next[i] < mn) mn = vit_next[i];
    for (int i = 0; i < THOR_NSTATES; ++i)
        vit_metric[i] = (vit_next[i] >= VIT_INF || mn >= VIT_INF) ? VIT_INF : (uint16_t)(vit_next[i] - mn);
    ++vit_steps;
    ++g_vit_pairs;
    if (vit_steps < THOR_TRACE_DEPTH) return;

    int st = 0;
    for (int i = 1; i < THOR_NSTATES; ++i) if (vit_metric[i] < vit_metric[st]) st = i;
    // Пройти назад до входного бита самого старого перехода в окне.
    for (int back = 0; back < THOR_TRACE_DEPTH - 1; ++back) {
        const uint32_t tr_step = vit_steps - 1 - (uint32_t)back;
        const uint8_t high = vit_trace[tr_step % THOR_TRACE_DEPTH][st] & 1;
        st = (st >> 1) | (high ? 32 : 0);
    }
    vc_push_bit((uint8_t)(st & 1));
}

// ---------------------------------------------------------------------------  
// Приём символьного тона -> IFK-де-дельта -> nibble -> deinterleave -> Витерби.  
// ---------------------------------------------------------------------------  
static void thor_on_symbol(int tone) {  
    if (!have_prev) { prev_tone = tone; have_prev = true; return; }  
  
    int d = tone - prev_tone - THOR_IFK_OFFSET;  
    while (d < 0) d += THOR_NTONES;  
    d %= THOR_NTONES;  
    prev_tone = tone;  
  
    // nibble -> 4 бита (порядок TX: sym4[i] = бит (IL_SIZE-1-i) от nibble)  
    uint8_t sym4[THOR_IL_SIZE];  
    for (int i = 0; i < THOR_IL_SIZE; i++)  
        sym4[i] = (uint8_t)((d >> (THOR_IL_SIZE - i - 1)) & 1);  
  
    il_rev_bits(sym4);  
  
    // 4 бита -> две кодовые пары; не храним всю историю в SRAM.
    for (int i = 0; i < THOR_IL_SIZE; i += 2)
        vit_step((uint8_t)(sym4[i] | (sym4[i + 1] << 1)));

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
    last_tone = -1; clk = 0; clk_valid = false;  
    prev_tone = -1; have_prev = false;  
  
    memset(il_table, 0, sizeof(il_table));
    vit_reset();
    memset(vit_trace, 0, sizeof(vit_trace));
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
  
    // Сглаживание + argmax тона на этом блоке.  
    int best = 0; float bp = -1.0f;  
    for (int i = 0; i < THOR_NTONES; i++) {  
        tpwr[i] += (pwr[i] - tpwr[i]) * THOR_SMOOTH_A;  
        if (tpwr[i] > bp) { bp = tpwr[i]; best = i; }  
    }  
  
    // Фиксированная символьная сетка: не сбрасываем часы на каждом изменении argmax.
    // Частота 15625 Гц, блок 256, символ около 1451 отсчёта.
    last_tone = best;
    if (!clk_valid) {
        clk_valid = true;
        clk = 0;
        return;
    }
    const int old_clk = clk;
    clk += THOR_BLOCK_N;
    if (old_clk < THOR_SYM_HALF && clk >= THOR_SYM_HALF)
        thor_on_symbol(best);
    if (clk >= THOR_SYM_SAMP) clk -= THOR_SYM_SAMP;
}