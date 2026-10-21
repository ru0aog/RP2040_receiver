// ============================================================================  
// rx_rtty.cpp — декодер RTTY-45 (45.45 бод, shift ~170 Гц, Baudot)  
// Вход: кадры мощностей банка rx_tones (кадр 43 отсчёта = 22.02 мс ≈ 1 бит).  
// Логика: автопоиск пары каналов с разносом 140..200 Гц и SNR >= 10 дБ;  
// далее mark/space по мощности, сборка битов (стоп-промахи переоткрывают кадр).  
// ============================================================================  
  
#include <Arduino.h>  
#include <math.h>  
#include "rx_tones.h"  
  
extern const int RX_TONES_N;          // из rx_tones.cpp  
static const float STEP_HZ = 1953.125f / 500.0f;   // шаг сетки банка ~3.906 Гц  
  
// ---------- параметры ----------  
#define LOCK_SNR_DB   10.0f   // оба тона должны быть выше медианы  
#define SHIFT_MIN_HZ  140.0f  
#define SHIFT_MAX_HZ  200.0f  
  
// mark/space-каналы: -1 = не залочены  
static int ch_mark  = -1;  
static int ch_space = -1;  
  
// ---------- битовая сборка ----------  
// кадр = один бит RTTY: старт(0) + 5 бодо-битов + стоп(>=1)  
static uint8_t  bit_frame = 0;   // принятые биты (LSB-first)  
static uint8_t  bit_cnt   = 0;  
static uint8_t  shift_ltrs = 1;  // текущий регистр: 1=LTRS, 0=FIGS  
  
// Бодо LTRS (индекс 0..31). FIGS — сокращённо, при необходимости доукомплектовать.  
static const char bodo_ltrs[32] = {  
    '\0','E','\n','A',' ','S','I','U','\r','D','R','J','N','F','C','K',  
    'T','Z','L','W','H','Y','P','Q','O','B','G', 27,'M','X','V', 31  
    // 26=LTRS(принудителен), 27=FIGS, 31=LTRS  
};  
static const char bodo_figs[32] = {  
    '\0','3','\n','-',' ','\'','8','7','\r','$','4','\a',',','!',':','(',  
    '5','"',')','2','#','6','0','1','9','?','&', 27,'.','/',';', 31  
};  
  
static void rtty_reset_char(void) { bit_frame = 0; bit_cnt = 0; }  
  
static void rtty_emit(uint8_t code) {  
    code &= 31;  
    if (code == 27)      { shift_ltrs = 0; return; }  
    if (code == 31)      { shift_ltrs = 1; return; }  
    char c = shift_ltrs ? bodo_ltrs[code] : bodo_figs[code];  
    if (c && c != 27 && c != 31) Serial.write(c);  
}  
  
// bit: true = mark (лог.1), false = space (лог.0)  
static void rtty_bit(bool bit) {  
    if (bit_cnt == 0) {                      // ждём старт-бит  
        if (!bit) { bit_frame = 0; bit_cnt = 1; }  
        return;  
    }  
    if (bit_cnt <= 5) {                      // 5 информационных бит, LSB first  
        bit_frame |= (bit ? 1 : 0) << (bit_cnt - 1);  
        bit_cnt++;  
        return;  
    }  
    // стоп-бит: должен быть mark  
    if (bit) rtty_emit(bit_frame);  
    rtty_reset_char();                       // не-mark стоп = потеря синхры  
}  
  
// ---------- автопоиск пары mark/space ----------  
// Ищем два канала с P >= медиана * LOCK_SNR, разносом 140..200 Гц.  
// Сильнейший нижний = space, верхний = mark (стандарт: mark выше).  
static bool rtty_acquire(void) {  
    float med = rx_tones_median();  
    float need = med * powf(10.0f, LOCK_SNR_DB / 10.0f);  
  
    int   best_lo = -1, best_hi = -1;  
    float best_p  = 0.0f;  
  
    for (int lo = 25; lo < RX_TONES_N - 25; lo++) {  
        if (rx_tones_power(lo) < need) continue;  
        float f_lo_hz = rx_tones_freq_hz(lo);  
        for (int hi = lo + 20; hi < RX_TONES_N - 25; hi++) {  
            float df = rx_tones_freq_hz(hi) - f_lo_hz;  
            if (df < SHIFT_MIN_HZ) continue;  
            if (df > SHIFT_MAX_HZ) break;  
            if (rx_tones_power(hi) < need) continue;  
            float pp = rx_tones_power(lo) + rx_tones_power(hi);  
            if (pp > best_p) { best_p = pp; best_lo = lo; best_hi = hi; }  
        }  
    }  
    if (best_lo < 0) return false;  
    ch_space = best_lo;  
    ch_mark  = best_hi;  
    Serial.printf("#RTTY_LOCK,space=%.0f,mark=%.0f,delta=%.0f\n",  
                  (double)rx_tones_freq_hz(ch_space),  
                  (double)rx_tones_freq_hz(ch_mark),  
                  (double)(rx_tones_freq_hz(ch_mark) - rx_tones_freq_hz(ch_space)));  
    rtty_reset_char();  
    shift_ltrs = 1;  
    return true;  
}  
  
// ---------- кадр ----------  
void rx_rtty_frame(void) {  
    if (ch_mark < 0) {  
        rtty_acquire();          // ищем пару каждый кадр, пока не залочимся  
        return;  
    }  
    // потеря сигнала: обе мощности упали — разлочка  
    float med = rx_tones_median();  
    if (rx_tones_power(ch_mark) < med * 4.0f &&  
        rx_tones_power(ch_space) < med * 4.0f) {  
        ch_mark = ch_space = -1;  
        rtty_reset_char();  
        Serial.println("#RTTY_LOST");  
        return;  
    }  
    rtty_bit(rx_tones_power(ch_mark) > rx_tones_power(ch_space));  
}