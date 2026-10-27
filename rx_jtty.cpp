// rx_jtty.cpp — декодер JTTY по спецификации WSJT-X 3.3.  
// Параметры (jtty_mod / jtty_codec / tbcc_code_profile):  
//   4-FSK, символ 32 мс (500 отсчётов при 15625 Гц), тоны шагом 31.25 Гц;  
//   кадр = 13 sync (is13) + 46 данных; tail-biting свёртка K=10  
//   (полиномы 0o1167/0o1545, 512 состояний), CRC-12(0x80F), payload 34 бита.  
// Gray-маппинг дибитов (tbcc_encode): 00->0, 01->1, 11->2, 10->3.  
#include "rx_jtty.h"  
#include <Arduino.h>  
#include <math.h>  
#include <string.h>  
  
#define JTTY_FS           15625.0f  
#define JTTY_SPS          500        // отсчётов на символ  
#define JTTY_NTONES       4  
#define JTTY_TONE_STEP    31.25f  
#define JTTY_NSYNC        13  
#define JTTY_NDATA        46  
#define JTTY_SYNC_MIN     7          // >6 из 13 — порог канала 0 jtty_mdecode  
#define JTTY_INFO_BITS    46  
#define JTTY_PAYLOAD_BITS 34  
#define JTTY_NU           9  
#define JTTY_NSTATES      512  
#define JTTY_G0           01167  
#define JTTY_G1           01545  
#define JTTY_CRC_POLY     0x80F  
  
// Синхропреамбула (из JTTY_beacon/jtty_codec.cpp — тот же источник, что jtty_mod).  
static const uint8_t IS13[JTTY_NSYNC] = {0,2,2,3,0,0,3,2,1,3,1,2,0};  
  
// Алфавит TEXT5 (jtty_source_encoding): индекс 36 = пробел.  
static const char JTTY_ALPHABET[65] =  
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ +-./?!\"#$%,&*()_'=[]{}<>|:;";  
  
// --- Гёрцель: 4 резонатора, power = |X|^2 на конец символа ---  
static float g_coeff[JTTY_NTONES];  
static float g_s1[JTTY_NTONES], g_s2[JTTY_NTONES];  
static uint16_t g_cnt = 0;          // позиция внутри символа  
static bool   jtty_active = false;  
  
// Кольцо последних символов для поиска IS13.  
#define SYM_RING 64  
static uint8_t  sym_ring[SYM_RING];  
static uint8_t  sym_w = 0;  
  
// Состояние сбора кадра после детекта синхры.  
static uint8_t  data_syms[JTTY_NDATA];  
static float    data_pwr[JTTY_NDATA][JTTY_NTONES];  
static int      data_have = 0;      // -1 = ждём синхру  
  
// Сообщение: накопление текста до EOM.  
static char msg_buf[96];  
static int  msg_len = 0;  
  
static inline float tone_power(int t) {  
    float y = g_s1[t];              // s2 уже «подвешена» — Goertzel power:  
    // стандартный power на N-ном шаге: s1^2 + s2^2 - coeff*s1*s2  
    return y * y + g_s2[t] * g_s2[t] - g_coeff[t] * y * g_s2[t];  
}  
  
static void msg_append(const char* s) {  
    int n = (int)strlen(s);  
    if (msg_len + n >= (int)sizeof(msg_buf) - 1) msg_len = 0;  // переполнение — сброс  
    memcpy(msg_buf + msg_len, s, n);  
    msg_len += n;  
    msg_buf[msg_len] = 0;  
}  
  
static void msg_flush(bool eom) {  
    // Филлер jtty_filler: хвостовые серии '<' заменяем пробелом, трим.  
    char* e = msg_buf + msg_len;  
    while (e > msg_buf && (e[-1] == ' ' || e[-1] == '<')) --e;  
    *e = 0;  
    if (msg_buf[0]) Serial.printf("# JTTY: \"%s\"%s\n", msg_buf, eom ? " [EOM]" : "");  
    msg_len = 0; msg_buf[0] = 0;  
}  
  
static inline int parity32(uint32_t x) {  
    x ^= x >> 16; x ^= x >> 8; x ^= x >> 4; x ^= x >> 2; x ^= x >> 1;  
    return (int)(x & 1);  
}  
  
static uint16_t crc12(const uint8_t* info /*34*/) {  
    uint16_t reg = 0;  
    for (int i = 0; i < JTTY_PAYLOAD_BITS; i++) {  
        uint16_t fb = (uint16_t)(((reg >> 11) ^ info[i]) & 1);  
        reg = (uint16_t)(((reg << 1) & 0xFFF) ^ (fb ? JTTY_CRC_POLY : 0));  
    }  
    return reg;  
}  
  
// --- Tail-biting Viterbi: 512 состояний, 46 шагов ---  
// Метрика: сумма power соответствующего тона. Решение двухпроходное:  
// проход 1 — все стартовые состояния равны, берём лучший финальный;  
// проход 2 — фиксируем старт=финал (tail-biting) и восстанавливаем биты.  
static float  m_curr[JTTY_NSTATES], m_next[JTTY_NSTATES];  
static uint8_t path[JTTY_INFO_BITS][JTTY_NSTATES / 8];  // бит выбора предка  
  
static const uint8_t GRAY2TONE[4] = {0, 1, 3, 2};  // (b0<<1)|b1 -> tone  
  
static bool vit_decode(uint8_t* info46 /*out*/) {  
    const float BIG = 1e30f;  
    for (int s = 0; s < JTTY_NSTATES; s++) m_curr[s] = 0.0f;  
  
    for (int step = 0; step < JTTY_INFO_BITS; step++) {  
        const float* p = data_pwr[step];  
        for (int s = 0; s < JTTY_NSTATES; s++) m_next[s] = -BIG;  
        for (int s = 0; s < JTTY_NSTATES; s++) {  
            // Два возможных входных бита 0/1: предок состояния s — (s>>1) и (s>>1)+256.  
            for (int bit = 0; bit < 2; bit++) {  
                int prev = (s >> 1) | (bit << (JTTY_NU - 1));  
                // reg предка после сдвига даёт текущее состояние s:  
                // reg10 = (prev<<1)|bit → эмиссия из reg10  
                uint16_t reg10 = (uint16_t)(((prev << 1) | bit) & 0x3FF);  
                int b0 = parity32(reg10 & JTTY_G0);  
                int b1 = parity32(reg10 & JTTY_G1);  
                int tone = GRAY2TONE[(b0 << 1) | b1];  
                float m = m_curr[prev] + p[tone];  
                if (m > m_next[s]) {  
                    m_next[s] = m;  
                    if (bit) path[step][s >> 3] |=  (uint8_t)(1u << (s & 7));  
                    else     path[step][s >> 3] &= (uint8_t)~(1u << (s & 7));  
                }  
            }  
        }  
        memcpy(m_curr, m_next, sizeof(m_curr));  
    }  
  
    // Лучший финальный state — и по tail-biting он же стартовый.  
    int best = 0;  
    for (int s = 1; s < JTTY_NSTATES; s++)  
        if (m_curr[s] > m_curr[best]) best = s;  
  
    // Проход 2: фиксированный старт.  
    for (int s = 0; s < JTTY_NSTATES; s++) m_curr[s] = -BIG;  
    m_curr[best] = 0.0f;  
    for (int step = 0; step < JTTY_INFO_BITS; step++) {  
        const float* p = data_pwr[step];  
        for (int s = 0; s < JTTY_NSTATES; s++) m_next[s] = -BIG;  
        for (int s = 0; s < JTTY_NSTATES; s++) {  
            for (int bit = 0; bit < 2; bit++) {  
                int prev = (s >> 1) | (bit << (JTTY_NU - 1));  
                uint16_t reg10 = (uint16_t)(((prev << 1) | bit) & 0x3FF);  
                int b0 = parity32(reg10 & JTTY_G0);  
                int b1 = parity32(reg10 & JTTY_G1);  
                int tone = GRAY2TONE[(b0 << 1) | b1];  
                float m = m_curr[prev] + p[tone];  
                if (m > m_next[s]) {  
                    m_next[s] = m;  
                    if (bit) path[step][s >> 3] |=  (uint8_t)(1u << (s & 7));  
                    else     path[step][s >> 3] &= (uint8_t)~(1u << (s & 7));  
                }  
            }  
        }  
        memcpy(m_curr, m_next, sizeof(m_curr));  
    }  
    if (m_curr[best] <= -BIG / 2) return false;  
  
    // Трейсбэк от состояния best.  
    int st = best;  
    for (int step = JTTY_INFO_BITS - 1; step >= 0; step--) {  
        int bit = (path[step][st >> 3] >> (st & 7)) & 1;  
        info46[step] = (uint8_t)bit;  
        st = (st >> 1) | (bit << (JTTY_NU - 1));  
    }  
    return true;  
}  
  
static void frame_process(void) {  
    uint8_t info[JTTY_INFO_BITS];  
    if (!vit_decode(info)) return;  
  
    // CRC-12: первые 34 бита — payload, последние 12 — CRC MSB-first.  
    uint16_t crc_calc = crc12(info);  
    uint16_t crc_recv = 0;  
    for (int i = 0; i < 12; i++) crc_recv = (uint16_t)((crc_recv << 1) | info[34 + i]);  
    if (crc_calc != crc_recv) {  
        Serial.printf("# JTTY frame CRC fail\n");  
        return;  
    }  
  
    // payload34: биты 0..31 = n32, бит 32 = резерв, бит 33 = EOM.  
    uint32_t n32 = 0;  
    for (int i = 0; i < 32; i++) n32 = (n32 << 1) | info[i];  
    bool eom = info[33] != 0;  
    uint8_t i2 = (uint8_t)(n32 & 3u);  
  
    if (i2 == 3) {  // TEXT5: top30 = n32 >> 2, пять 6-битных индексов  
        uint32_t top30 = n32 >> 2;  
        char t[6];  
        for (int k = 4; k >= 0; k--) { t[k] = JTTY_ALPHABET[top30 % 64u]; top30 /= 64u; }  
        t[5] = 0;  
        Serial.printf("# JTTY atom: \"%s\"\n", t);  
        msg_append(t);  
    } else {  
        Serial.printf("# JTTY atom i2=%u (не TEXT5, пропуск)\n", i2);  
    }  
    if (eom) msg_flush(true);  
}  
  
void rx_jtty_init(float center_hz) {  
    // Тон 0 на center - 1.5*step (как в jtty: f0 = center - JTTY_CENTER_OFFSET,  
    // 4 тона по 31.25 Гц симметрично вокруг center).  
    float f0 = center_hz - 1.5f * JTTY_TONE_STEP;  
    for (int t = 0; t < JTTY_NTONES; t++)  
        g_coeff[t] = 2.0f * cosf(2.0f * (float)M_PI * (f0 + t * JTTY_TONE_STEP) / JTTY_FS);  
    memset(g_s1, 0, sizeof(g_s1));  
    memset(g_s2, 0, sizeof(g_s2));  
    g_cnt = 0; sym_w = 0; data_have = -1;  
    msg_len = 0; msg_buf[0] = 0;  
    jtty_active = true;  
}  
  
void rx_jtty_stop(void) { jtty_active = false; data_have = -1; }  
  
void rx_jtty_push(int16_t m) {  
    if (!jtty_active) return;  
    float x = (float)m;  
    for (int t = 0; t < JTTY_NTONES; t++) {  
        float s0 = x + g_coeff[t] * g_s1[t] - g_s2[t];  
        g_s2[t] = g_s1[t];  
        g_s1[t] = s0;  
    }  
    if (++g_cnt < JTTY_SPS) return;  
    g_cnt = 0;  
  
    // Конец символа: power по 4 тонам, argmax.  
    int sym = 0; float best = 0;  
    for (int t = 0; t < JTTY_NTONES; t++) {  
        float pw = tone_power(t);  
        if (data_have >= 0) data_pwr[data_have][t] = pw;  
        if (pw > best) { best = pw; sym = t; }  
        g_s1[t] = 0; g_s2[t] = 0;  
    }  
  
    if (data_have >= 0) {                    // собираем 46 символов данных  
        data_syms[data_have++] = (uint8_t)sym;  
        if (data_have >= JTTY_NDATA) { data_have = -1; frame_process(); }  
        return;  
    }  
  
    // Поиск IS13 в кольце последних 13 символов.  
    sym_ring[sym_w] = (uint8_t)sym;  
    sym_w = (uint8_t)((sym_w + 1) % SYM_RING);  
    int match = 0;  
    for (int i = 0; i < JTTY_NSYNC; i++) {  
        int idx = (sym_w - JTTY_NSYNC + i + SYM_RING * 2) % SYM_RING;  
        if (sym_ring[idx] == IS13[i]) match++;  
    }  
    if (match >= JTTY_SYNC_MIN) {  
        data_have = 0;                       // следующие 46 символов — данные  
        Serial.printf("# JTTY sync %d/13\n", match);  
    }  
}
