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
//   nibble -> обратный перемежитель -> 4 бита -> инкрементальный Viterbi  
//   (k=7, hard, кольцевой traceback) -> биты varicode -> rx_decoders_emit.  
//  
// Символьные часы — прямые: счётчик сэмплов, смена пика тона сбрасывает  
// часы на 0, строб на середине символа.  
  
#include "rx_thor.h"  
#include "src/modes/rx_decoders.h"  
#include "src/dsp/rx_goertzel.h"  
  
#include <Arduino.h>  
#include <stdint.h>  
#include <string.h>  
#include <math.h>  
  
// #define THOR_DEBUG   // посимвольный printf в hot path — вызывает ресет
  
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
static constexpr int     THOR_SYM_SAMP    = 1451;  
static constexpr int     THOR_SYM_HALF    = THOR_SYM_SAMP / 2;  
  
// FEC  
static constexpr int     THOR_K           = 7;  
static constexpr uint8_t THOR_POLY1       = 0x6d;  
static constexpr uint8_t THOR_POLY2       = 0x4f;  
static constexpr int     THOR_NSTATES     = 1 << (THOR_K - 1);   // 64  
  
// Перемежитель 4x10  
static constexpr int     THOR_IL_SIZE     = 4;  
static constexpr int     THOR_IL_DEPTH    = 10;  
  
// Кольцевое окно traceback Витерби: 96 шагов x 64 состояния = 6 КБ.  
// Глубина >> 5*K=35 — длины сходимости решётки хватает с запасом.  
static constexpr int     THOR_TR_DEPTH    = 96;  
  
// ---------------------------------------------------------------------------  
// Varicode — инверсия таблицы маяка (varicode[]/thor_varicode[]).  
// Биты кода считываются LSB-first из vc_shreg: s[b] == бит b.  
// ВНИМАНИЕ: сверить с побитовой инверсией varicode[] маяка —  
// в исходной таблице были дубликаты записей (n/t и др.).  
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
#define THOR_VARICODE_N 128  
#define VC_MAX_BITS     24  
  
// ---------------------------------------------------------------------------  
// Состояние  
// ---------------------------------------------------------------------------  
static bool   g_active = false;  
static float  g_base_hz = 0.0f;  
  
static float  tpwr[THOR_NTONES];  
static int    last_tone = -1;     // тон на прошлом блоке (детект перехода)  
static int    clk       = 0;      // сэмплы с последнего перехода тона  
static bool   clk_valid = false;  
  
static int    prev_tone = -1;     // предыдущий символьный тон (IFK)  
static bool   have_prev = false;  
  
// Обратный перемежитель: одна таблица 4x10.  
// il_table[i][d]: бит i-й позиции nibble с задержкой d символов.  
static uint8_t il_table[THOR_IL_SIZE][THOR_IL_DEPTH];  
  
static void il_rev_bits(uint8_t *sym4 /*4 бита -> 4 бита*/) {  
    for (int i = 0; i < THOR_IL_SIZE; i++) {  
        for (int d = THOR_IL_DEPTH - 1; d > i; d--)  
            il_table[i][d] = il_table[i][d - 1];  
        il_table[i][i] = sym4[i];                 // вход на глубину i  
        sym4[i] = il_table[i][THOR_IL_DEPTH - 1]; // выход — самый старый  
    }  
}  
  
// ---------------------------------------------------------------------------  
// Витерби (hard-decision, Хэмминг), k=7 -> 64 состояния.  
// Инкрементальный: метрики [64], кольцо traceback [96][64] (~6.5 КБ).  
// ---------------------------------------------------------------------------  
static uint16_t vit_metric[THOR_NSTATES];  
static uint8_t  vit_tr[THOR_TR_DEPTH][THOR_NSTATES];  // parent state на шаг  
static int      vit_wpos  = 0;   // куда пишется следующий шаг  
static int      vit_fill  = 0;   // сколько шагов хранится (<= TR_DEPTH)  
static int      vit_pairs = 0;   // всего принято кодовых пар  
  
static inline uint8_t parity8(uint8_t x) { return (uint8_t)__builtin_parity(x); }  
  
// Кодовый выход кодера для (state, input bit):  
// shreg = (state<<1)|bit, out0=parity(POLY1&shreg), out1=parity(POLY2&shreg)  
static inline uint8_t enc_out(int state, int bit) {  
    uint8_t sh = (uint8_t)((state << 1) | bit);  
    return (uint8_t)(parity8(sh & THOR_POLY1) | (parity8(sh & THOR_POLY2) << 1));  
}  
  
static void vc_push_bit(uint8_t bit);   // fwd  
  
// Эмит бита самого старого шага в кольце: traceback от лучшего  
// состояния на (vit_fill - 1) шаг назад, бит = LSB состояния.  
static void vit_emit_oldest(void) {  
    if (vit_fill < 2) return;  
    int best = 0;  
    for (int s = 1; s < THOR_NSTATES; s++)  
        if (vit_metric[s] < vit_metric[best]) best = s;  
  
    int st  = best;  
    int pos = (vit_wpos - 1 + THOR_TR_DEPTH) % THOR_TR_DEPTH;  // новейший  
    for (int k = vit_fill - 1; k > 0; k--) {  
        st  = vit_tr[pos][st];  
        pos = (pos - 1 + THOR_TR_DEPTH) % THOR_TR_DEPTH;  
    }  
    // st — состояние после самого старого шага; его LSB = входной бит.  
    vc_push_bit((uint8_t)(st & 1));  
    --vit_fill;  
}  
  
// Один шаг решётки (одна кодовая пара = 2 бита).  
static void vit_step(uint8_t rx) {  
    // Если кольцо полно — сначала выталкиваем самый старый бит  
    // (иначе перезапишем его parent-информацию).  
    if (vit_fill == THOR_TR_DEPTH) vit_emit_oldest();  
  
    uint16_t nm[THOR_NSTATES];  
    memset(nm, 0x3F, sizeof(nm));  
    for (int s = 0; s < THOR_NSTATES; s++) {  
        if (vit_metric[s] > 0x3E00) continue;  
        for (int b = 0; b < 2; b++) {  
            int ns = ((s << 1) | b) & (THOR_NSTATES - 1);  
            uint8_t exp = enc_out(s, b);  
            uint8_t dd  = (uint8_t)(exp ^ rx);  
            uint16_t m  = vit_metric[s] + (uint16_t)((dd & 1) + (dd >> 1));  
            if (m < nm[ns]) { nm[ns] = m; vit_tr[vit_wpos][ns] = (uint8_t)s; }  
        }  
    }  
  
    // Нормализация: вычесть минимум, uint16 не переполняется бесконечно.  
    uint16_t mn = nm[0];  
    for (int s = 1; s < THOR_NSTATES; s++) if (nm[s] < mn) mn = nm[s];  
    if (mn) for (int s = 0; s < THOR_NSTATES; s++) nm[s] -= mn;  
  
    memcpy(vit_metric, nm, sizeof(nm));  
  
    vit_wpos = (vit_wpos + 1) % THOR_TR_DEPTH;  
    if (vit_fill < THOR_TR_DEPTH) ++vit_fill;  
    ++vit_pairs;  
}  
  
// Принудительный слив хвоста (при остановке/конце передачи).  
static void vit_flush(void) {  
    while (vit_fill > 0) vit_emit_oldest();  
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
            break;  
        }  
    }  
    vc_shreg = 0; vc_nbits = 0; vc_zeros = 0;  
}  
  
static void vc_push_bit(uint8_t bit) {    
    if (bit == 0) {    
        ++vc_zeros;    
        if (vc_zeros >= 2) {                        // "00" — разделитель кодов    
            if (vc_nbits > 0) vc_emit_code();    
            else { vc_shreg = 0; vc_nbits = 0; }    
            vc_zeros = 0;    
        }    
        return;                    // одиночный 0 — отложенный: решим по след. биту    
    }    
    // bit == 1    
    if (vc_zeros == 1) {             // отложенный 0 оказался внутренним —    
        vc_zeros = 0;                // записываем его позицию (бит уже 0)    
        if (vc_nbits < VC_MAX_BITS) ++vc_nbits;    
        else { vc_shreg = 0; vc_nbits = 0; return; }    
    } else {    
        vc_zeros = 0;    
    }    
    if (vc_nbits < VC_MAX_BITS) {    
        vc_shreg |= (1u << vc_nbits);    
        ++vc_nbits;    
    } else {    
        vc_shreg = 0; vc_nbits = 0;                 // переполнение — сброс    
    }    
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
  
    if (d > 15) return;               // 16/17 — ошибка тона; пропускаем символ,    
                                    // Витерби переживёт пропуск пары    
  
    // nibble -> 4 бита (порядок TX: sym4[i] = бит (IL_SIZE-1-i) от nibble)  
    uint8_t sym4[THOR_IL_SIZE];  
    for (int i = 0; i < THOR_IL_SIZE; i++)  
        sym4[i] = (uint8_t)((d >> (THOR_IL_SIZE - i - 1)) & 1);  
  
    il_rev_bits(sym4);  
  
    // 4 бита = 2 кодовые пары по 2 бита (порядок TX: o0 потом o1)  
    for (int i = 0; i < THOR_IL_SIZE; i += 2)  
        vit_step((uint8_t)(sym4[i] | (sym4[i + 1] << 1)));  
  
#ifdef THOR_DEBUG  
    Serial.printf("# THOR sym=%d pairs=%d\n", d, vit_pairs);  
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
  
    memset(vit_metric, 0x3F, sizeof(vit_metric));  
    vit_metric[0] = 0;                              // старт state=0 (как у TX)  
    memset(vit_tr, 0, sizeof(vit_tr));  
    vit_wpos = 0; vit_fill = 0; vit_pairs = 0;  
  
    vc_shreg = 0; vc_nbits = 0; vc_zeros = 0;  
  
    gb_cfg_t cfg = { THOR_NTONES, g_base_hz, THOR_STEP_HZ, THOR_FS, THOR_BLOCK_N };  
    gb_init(&cfg);  
  
    g_active = true;  
  
#ifdef THOR_DEBUG  
    Serial.printf("# THOR init base=%.1f step=%.4f n=%d\n",  
                  (double)g_base_hz, (double)THOR_STEP_HZ, THOR_NTONES);  
#endif  
}  
  
void rx_thor_stop(void) {  
    if (g_active) vit_flush();                      // дожать хвост бит в varicode  
    g_active = false;  
}  
  
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
  
// Символьный такт с подтверждением перехода: clk=0 только если  
    // тон сменился два блока подряд — одиночный шумовой всплеск в другом  
    // бине Гёрцеля не собьёт синхронизацию.  
    static int    trans_cnt = 0;  
    last_tone = -1; clk = 0; clk_valid = false; trans_cnt = 0;;  
    clk += THOR_BLOCK_N;  
    if (best != last_tone) {  
        if (++trans_cnt >= 2) {  
            clk = 0;  
            clk_valid = true;  
            last_tone = best;  
            trans_cnt = 0;  
        }  
    } else {  
        trans_cnt = 0;  
    }  
    if (clk_valid && clk >= THOR_SYM_HALF && clk - THOR_BLOCK_N < THOR_SYM_HALF) {  
        thor_on_symbol(last_tone);  
    }  
}