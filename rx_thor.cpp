// rx_thor.cpp — THOR-11 decoder for RP2040_receiver.  
//  
// Сигнал (RP2040_beacon/thor_modem.cpp, fldigi thor.cxx):  
//   18 тонов, шаг 10.7666 Гц (11025/1024), символ 92879 мкс (~1451 сэмпл);  
//   IFK+:  tone = (prev_tone + sym + 2) % 18   ->  sym = (tone - prev - 2) mod 18;  
//   FEC:   бит -> свёрточный k=7 (poly1=0x6d, poly2=0x4f) -> 2 бита ->  
//          пачки по 4 бита -> перемежитель 4x10 -> символ (nibble);  
//   текст: varicode IZ8BLY (primary), граница символа — два нуля подряд,  
//          нули ВХОДЯТ в код (таблица varicode[] маяка: "10100", "11101011100"...)  
//  
// Конвейер приёма:  
//   банк Гёрцеля (gb_push, block=256) -> argmax тона -> IFK-де-дельта ->  
//   nibble -> обратный перемежитель -> 4 бита -> инкрементальный Viterbi  
//   (k=7, hard, кольцевой traceback) -> биты varicode -> текст.  
  
#include "rx_thor.h"  
#include "src/modes/rx_decoders.h"  
#include "src/dsp/rx_goertzel.h"  
  
#include <Arduino.h>  
#include <stdint.h>  
#include <string.h>  
#include <math.h>  
  
// -----------------------------------------------------------------------------  
// Конфигурация  
// -----------------------------------------------------------------------------  
  
#define THOR_DEBUG   // выключить после отладки: посимвольный printf грузит Core0  
  
static constexpr float  THOR_FS        = 15625.0f;  
static constexpr int    THOR_BLOCK_N   = 256;      // размер блока Гёрцеля  
static constexpr int    THOR_NTONES    = 18;  
static constexpr float  THOR_STEP_HZ   = 11025.0f / 1024.0f;   // 10.7666 Гц  
static constexpr float  THOR_SYM_S     = 0.092879f;            // 92879 мкс  
static constexpr int    THOR_SYM_SAMP  = (int)(THOR_SYM_S * THOR_FS + 0.5f); // ~1451  
static constexpr int    THOR_SYM_HALF  = THOR_SYM_SAMP / 2;  
  
// Сетка тонов: метка RSID находится в центре сетки (base + 8.5*step).  
static constexpr float  THOR_CENTER_OFFS = 8.5f * THOR_STEP_HZ;  // ~91.5 Гц  
  
static constexpr float  THOR_SMOOTH_A  = 0.20f;    // IIR-сглаживание мощности тонов  
static constexpr int    THOR_PEAK_HITS = 3;        // подтверждение перехода (блоков)  
  
// Viterbi k=7, 64 состояния, кольцевое traceback-окно.  
static constexpr int    THOR_NSTATES   = 64;  
static constexpr int    THOR_TR_DEPTH  = 96;       // глубина окна (символьных пар)  

// Перемежитель (как в маяке thor_modem.cpp): THOR_IL_SIZE x THOR_IL_DEPTH,  
// внутри каждого слоя таблица THOR_IL_SIZE x THOR_IL_SIZE  
static constexpr int   THOR_IL_SIZE  = 4;    // бит в nibble / размер таблицы  
static constexpr int   THOR_IL_DEPTH = 10;   // число слоёв

static constexpr int    VC_MAX_BITS    = 16;       // max длина varicode-кода  
  
// -----------------------------------------------------------------------------  
// Varicode IZ8BLY primary (как varicode[] в thor_modem.cpp маяка;  
// коды ВКЛЮЧАЮТ два завершающих нуля)  
// -----------------------------------------------------------------------------  
  
static const char* thor_rx_varicode[] = {  
    "1010101011", "1010111011", "1011101101", "1101110101",  
    "1010110101", "1110101111", "1101101111", "1101110111",  
    "1101111011", "1101111111", "1110110111", "1110111011",  
    "1110111101", "1110111111", "1111010111", "1111011011",  
    "1111011101", "1111011111", "1111101011", "1111101101",  
    "1111101111", "1111110101", "1111110111", "1111111011",  
    "1111111101", "1111111110", "1010111111", "1101010111",  
    "1101011011", "1101011101", "1101011111", "1101101011",  
    "1101101101", "1101101111", "1101110011", "1101110101",  
    "1101110111", "1101111011", "1110101011", "1110101101",  
    "1110101111", "1110110011", "1110110101", "1110110111",  
    "1110111011", "1110111101", "1110111111", "1111010011",  
    "1111010101", "1111010111", "1111011011", "1111011101",  
    "1111011111", "1111100111", "1111101011", "1111101101",  
    "1111101111", "1111110011", "1111110101", "1111110111",  
    "1111111011", "1111111101", "1111111110", "1",  
    "1010101011", "1010101101", "1010101111", "1010110011",  
    "1010110101", "1010110111", "1010111011", "1010111101",  
    "1010111111", "1011001011", "1011001101", "1011001111",  
    "1011010011", "1011010101", "1011010111", "1011011011",  
    "1011011101", "1011011111", "1011100111", "1011101011",  
    "1011101101", "1011101111", "1011110011", "1011110101",  
    "1011110111", "1011111011", "1011111101", "1100110111",  
    "1100111011", "1100111101", "1100111111", "1101001111",  
    "1101010011", "1101010101", "1101010111", "1101011011",  
    "1101011101", "1101011111", "1101100111", "1101101011",  
    "1101101101", "1101101111", "1101110011", "1101110101",  
    "1101110111", "1101111011", "1101111111", "1110010111",  
    "1110011011", "1110011101", "1110011111", "1110100111",  
    "1110101011", "1110101101", "1110101111", "1110110011",  
    "1110110101", "1110110111", "1110111011", "1110111101",  
    "1110111111", "1111010011"  
};  
  
static const int THOR_RX_VC_N = (int)(sizeof(thor_rx_varicode) / sizeof(thor_rx_varicode[0]));  
  
// -----------------------------------------------------------------------------  
// Состояние  
// -----------------------------------------------------------------------------  
  
static bool   g_active = false;  
static float  tpwr[THOR_NTONES];       // сглаженная мощность тонов  
static int    last_tone = -1;          // последний подтверждённый тон  
static int    prev_tone = -1;          // предыдущий тон для IFK-дельты  
static int    clk = 0;                 // сэмплы с последнего перехода  
static bool   clk_valid = false;  
  
// Кольцевой traceback Витерби.  
static uint16_t vit_metric[THOR_NSTATES];  
static uint8_t  vit_tr[THOR_TR_DEPTH][THOR_NSTATES];   // выбранный предок  
static int      vit_wpos = 0;                          // позиция записи в кольце  
static int      vit_fill = 0;                          // заполнение кольца  
static int      vit_pairs = 0;  
  
// Varicode-накопитель.  
static uint32_t vc_shreg = 0;  
static int      vc_nbits = 0;  
static int      vc_zeros = 0;  
  
#ifdef THOR_DEBUG  
static uint32_t dbg_blocks = 0, dbg_sym = 0;  
static uint32_t dbg_vit_bits = 0, dbg_vc_emit = 0, dbg_vc_miss = 0;  
static uint32_t dbg_trans = 0;  
static float    dbg_bp_max = 0.0f;  
static uint32_t dbg_tone_hist[THOR_NTONES];  
#endif  
  
// -----------------------------------------------------------------------------  
// Обратный перемежитель — зеркальная копия TX (il_tab[k][i][j]),  
// чтение по REV-диагонали [k][i][i].  
// -----------------------------------------------------------------------------  
  
static uint8_t il_tab_buf[THOR_IL_DEPTH * THOR_IL_SIZE * THOR_IL_SIZE];  
  
static inline uint8_t* il_tab(int k, int i, int j) {  
    return &il_tab_buf[(k * THOR_IL_SIZE + i) * THOR_IL_SIZE + j];  
}  
  
static void il_rev_bits(uint8_t* sym4) {  
    uint8_t out[THOR_IL_SIZE];  
    for (int k = 0; k < THOR_IL_DEPTH; k++) {  
        for (int i = 0; i < THOR_IL_SIZE; i++)  
            for (int j = 0; j < THOR_IL_SIZE - 1; j++)  
                *il_tab(k, i, j) = *il_tab(k, i, j + 1);  
        for (int i = 0; i < THOR_IL_SIZE; i++)  
            *il_tab(k, i, THOR_IL_SIZE - 1) = sym4[i];  
        for (int i = 0; i < THOR_IL_SIZE; i++)  
            out[i] = *il_tab(k, i, i);            // REV-диагональ  
    }  
    memcpy(sym4, out, THOR_IL_SIZE);  
}  
  
// -----------------------------------------------------------------------------  
// Varicode: накопление бит, эмиссия по "00" (нули — часть кода)  
// -----------------------------------------------------------------------------  
  
static void vc_emit_code(void) {  
    const int len = vc_nbits;  
    const char* s = NULL;  
    for (int i = 0; i < THOR_RX_VC_N; i++) {  
        const char* t = thor_rx_varicode[i];  
        if ((int)strlen(t) != len) continue;  
        bool ok = true;  
        for (int b = 0; b < len; b++) {  
            int bit = (int)((vc_shreg >> (len - 1 - b)) & 1u);  
            if ((t[b] == '1') != (bit != 0)) { ok = false; break; }  
        }  
        if (ok) { s = t; break; }  
    }  
    if (s) {  
        int idx = -1;  
        for (int i = 0; i < THOR_RX_VC_N; i++)  
            if (thor_rx_varicode[i] == s) { idx = i; break; }  
        if (idx >= 0) {  
            uint8_t ch = (uint8_t)idx;  
            rx_decoders_emit(ch);  
#ifdef THOR_DEBUG  
            ++dbg_vc_emit;  
#endif  
        }  
    } else {  
#ifdef THOR_DEBUG  
        ++dbg_vc_miss;  
        Serial.printf("# THOR vc? nbits=%d shreg=%05lX\n",  
                      vc_nbits, (unsigned long)vc_shreg);  
#endif  
    }  
    vc_shreg = 0;  
    vc_nbits = 0;  
}  
  
static void vc_push_bit(uint8_t bit) {  
    if (vc_nbits >= VC_MAX_BITS) {   // код не сошёлся — сброс до потери синхры  
        vc_shreg = 0; vc_nbits = 0; vc_zeros = 0;  
    }  
    vc_shreg = (vc_shreg << 1) | (bit & 1u);  
    ++vc_nbits;  
    if (bit) { vc_zeros = 0; return; }  
    if (++vc_zeros >= 2) {  
        if (vc_shreg != 0)          // код из одних нулей (промежуток) — пропуск  
            vc_emit_code();  
        else { vc_shreg = 0; vc_nbits = 0; }  
        vc_zeros = 0;  
    }  
}  
  
// -----------------------------------------------------------------------------  
// Витерби k=7 (0x6d, 0x4f), hard-decision, кольцевой traceback 96 шагов  
// -----------------------------------------------------------------------------  
  
static inline int parity8(uint8_t v) {  
    v ^= v >> 4; v ^= v >> 2; v ^= v >> 1;  
    return v & 1;  
}  
  
static void vit_emit_oldest(void) {  
    // лучшее текущее состояние  
    int st = 0;  
    uint16_t m = vit_metric[0];  
    for (int s = 1; s < THOR_NSTATES; s++)  
        if (vit_metric[s] < m) { m = vit_metric[s]; st = s; }  
    // назад по кольцу до самого старого шага  
    int pos = (vit_wpos - 1 + THOR_TR_DEPTH) % THOR_TR_DEPTH;  
    for (int k = vit_fill - 1; k > 0; k--) {  
        st  = vit_tr[pos][st];  
        pos = (pos - 1 + THOR_TR_DEPTH) % THOR_TR_DEPTH;  
    }  
    // st — состояние ПОСЛЕ самого старого шага; входной бит = LSB  
    uint8_t bit = (uint8_t)(st & 1);  
    vc_push_bit(bit);  
    --vit_fill;  
#ifdef THOR_DEBUG  
    ++dbg_vit_bits;  
#endif  
}  
  
static void vit_flush(void) {  
    while (vit_fill > 0) vit_emit_oldest();  
}  
  
static void vit_step(uint8_t pair /* o0 | o1<<1 */) {  
    if (vit_fill >= THOR_TR_DEPTH) vit_emit_oldest();  
  
    uint16_t nm[THOR_NSTATES];  
    for (int s = 0; s < THOR_NSTATES; s++) nm[s] = 0xFFFF;  
  
    for (int s = 0; s < THOR_NSTATES; s++) {  
        if (vit_metric[s] == 0xFFFF) continue;  
        for (int b = 0; b < 2; b++) {  
            uint8_t ns  = (uint8_t)(((s << 1) | b) & 0x3F);  
            uint8_t e0  = (uint8_t)parity8((uint8_t)(ns & 0x6D));  
            uint8_t e1  = (uint8_t)parity8((uint8_t)(ns & 0x4F));  
            uint8_t enc = (uint8_t)(e0 | (e1 << 1));  
            uint16_t cost = (uint16_t)__builtin_popcount((unsigned)(enc ^ pair));  
            uint16_t cand = (uint16_t)(vit_metric[s] + cost);  
            if (cand < nm[ns]) {  
                nm[ns] = cand;  
                vit_tr[vit_wpos][ns] = (uint8_t)s;  
            }  
        }  
    }  
  
    // нормализация метрик — защита от переполнения  
    uint16_t mn = 0xFFFF;  
    for (int s = 0; s < THOR_NSTATES; s++) if (nm[s] < mn) mn = nm[s];  
    if (mn > 0) for (int s = 0; s < THOR_NSTATES; s++) nm[s] = (uint16_t)(nm[s] - mn);  
  
    memcpy(vit_metric, nm, sizeof(nm));  
    vit_wpos = (vit_wpos + 1) % THOR_TR_DEPTH;  
    ++vit_fill;  
    ++vit_pairs;  
}  
  
// -----------------------------------------------------------------------------  
// Символ: IFK-дельта -> nibble -> deinterleave -> 2 пары в Витерби  
// -----------------------------------------------------------------------------  
  
static void thor_on_symbol(int tone) {  
    if (prev_tone < 0) { prev_tone = tone; return; }  
  
    int d = tone - prev_tone - 2;  
    prev_tone = tone;  
    d %= THOR_NTONES;  
    if (d < 0) d += THOR_NTONES;  
    if (d > 15) return;                 // 16/17 — ошибка тона, пропуск  
  
#ifdef THOR_DEBUG  
    ++dbg_sym;  
#endif  
  
    // nibble -> 4 бита, MSB-first (как TX: v = (v<<1)|syms[i])  
    uint8_t sym4[THOR_IL_SIZE];  
    for (int i = 0; i < THOR_IL_SIZE; i++)  
        sym4[i] = (uint8_t)((d >> (THOR_IL_SIZE - i - 1)) & 1);  
  
    il_rev_bits(sym4);  
  
    // 4 бита = 2 кодовые пары (o0 потом o1 — порядок TX)  
    for (int i = 0; i < THOR_IL_SIZE; i += 2)  
        vit_step((uint8_t)(sym4[i] | (sym4[i + 1] << 1)));  
  
#ifdef THOR_DEBUG  
    Serial.printf("# THOR sym=%d pairs=%d\n", d, vit_pairs);  
#endif  
}  
  
// -----------------------------------------------------------------------------  
// API  
// -----------------------------------------------------------------------------  
  
void rx_thor_init(float freq_hz) {  
    g_active  = true;  
    last_tone = prev_tone = -1;  
    clk = 0; clk_valid = false;  
    memset(tpwr, 0, sizeof(tpwr));  
    memset(vit_metric, 0xFF, sizeof(vit_metric));  
    vit_metric[0] = 0;                  // старт из state=0  
    memset(vit_tr, 0, sizeof(vit_tr));  
    vit_wpos = vit_fill = vit_pairs = 0;  
    vc_shreg = 0; vc_nbits = 0; vc_zeros = 0;  
    memset(il_tab_buf, 0, sizeof(il_tab_buf));  
  
#ifdef THOR_DEBUG  
    dbg_blocks = dbg_sym = 0;  
    dbg_vit_bits = dbg_vc_emit = dbg_vc_miss = 0;  
    dbg_trans = 0; dbg_bp_max = 0.0f;  
    memset(dbg_tone_hist, 0, sizeof(dbg_tone_hist));  
#endif  
  
    gb_cfg_t cfg;  
    cfg.base_hz = freq_hz - THOR_CENTER_OFFS;   // нижний тон сетки  
    cfg.step_hz = THOR_STEP_HZ;  
    cfg.ntones  = THOR_NTONES;  
    gb_init(&cfg);  
  
#ifdef THOR_DEBUG  
    Serial.printf("# THOR init base=%.1f step=%.4f n=%d\n",  
                  (double)(freq_hz - THOR_CENTER_OFFS),  
                  (double)THOR_STEP_HZ, THOR_NTONES);  
#endif  
}  
  
void rx_thor_stop(void) {  
    vit_flush();                        // дожать хвост сообщения в varicode  
    g_active = false;  
#ifdef THOR_DEBUG  
    Serial.printf("# THOR done sym=%lu vit=%lu vc=%lu miss=%lu\n",  
                  (unsigned long)dbg_sym, (unsigned long)dbg_vit_bits,  
                  (unsigned long)dbg_vc_emit, (unsigned long)dbg_vc_miss);  
#endif  
}  
  
void rx_thor_push(int16_t m) {  
    if (!g_active) return;  
    static float pwr[THOR_NTONES];  
    if (!gb_push(m, pwr)) return;  
  
#ifdef THOR_DEBUG  
    ++dbg_blocks;  
#endif  
  
    // Сглаживание + argmax тона на этом блоке.  
    int best = 0; float bp = -1.0f;  
    for (int i = 0; i < THOR_NTONES; i++) {  
        tpwr[i] += (pwr[i] - tpwr[i]) * THOR_SMOOTH_A;  
        if (tpwr[i] > bp) { bp = tpwr[i]; best = i; }  
    }  
  
#ifdef THOR_DEBUG  
    if (bp > dbg_bp_max) dbg_bp_max = bp;  
    ++dbg_tone_hist[best];  
    // периодическая статистика раз в ~5 сек (300 блоков по 256 сэмплов)  
    if (dbg_blocks % 300 == 0) {  
        Serial.printf("# THOR blk=%lu bp=%.0f trans=%lu sym=%lu vit=%lu vc=%lu miss=%lu hist:",  
                      (unsigned long)dbg_blocks, (double)dbg_bp_max,  
                      (unsigned long)dbg_trans, (unsigned long)dbg_sym,  
                      (unsigned long)dbg_vit_bits, (unsigned long)dbg_vc_emit,  
                      (unsigned long)dbg_vc_miss);  
        for (int i = 0; i < THOR_NTONES; i++)  
            Serial.printf(" %lu", (unsigned long)dbg_tone_hist[i]);  
        Serial.printf("\n");  
    }  
#endif  
  
    // 1. Подтверждение перехода тона: THOR_PEAK_HITS блоков подряд  
    //    на новом тоне — одиночный шумовой пик не собьёт такт.  
    static int candidate_tone = -1;  
    static int candidate_hits = 0;  
  
    if (best != last_tone) {  
        if (best == candidate_tone) ++candidate_hits;  
        else { candidate_tone = best; candidate_hits = 1; }  
  
        if (candidate_hits >= THOR_PEAK_HITS) {  
            last_tone = best;  
            candidate_tone = -1;  
            candidate_hits = 0;  
#ifdef THOR_DEBUG  
            ++dbg_trans;  
#endif  
            // Ресинхронизация: переход тона = граница символа (IFK  
            // гарантирует переход каждый символ). clk ставим так, чтобы  
            // строб попадал на середину: подтверждение отстаёт от фронта  
            // на ~PEAK_HITS блоков, поэтому clk стартует с отрицательным  
            // смещением — строб через SYM_HALF от реальной границы.  
            clk = -(candidate_hits * THOR_BLOCK_N);  
            clk_valid = true;  
        }  
    } else {  
        candidate_tone = -1;  
        candidate_hits = 0;  
    }  
  
    // 2. Модульные символьные часы: clk бежит по mod SYM_SAMP, строб  
    //    на середине. Переход лишь поправляет фазу (ресинх), а не  
    //    перезапускает цикл — строб сохраняет период даже при ложном  
    //    или пропущенном переходе.  
    if (clk_valid) {  
        clk += THOR_BLOCK_N;  
        if (clk >= THOR_SYM_HALF && clk - THOR_BLOCK_N < THOR_SYM_HALF)  
            thor_on_symbol(last_tone);      // подтверждённый тон, не сырой best  
        if (clk >= THOR_SYM_SAMP) clk -= THOR_SYM_SAMP;  
    }  
}