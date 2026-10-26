#include <Arduino.h>  
#include <math.h>  
#include <string.h>  
#include "rx_goertzel.h"  
  
#define GB_COEFF_SHIFT 14          // Q14: coeff = 2*cos(w) ∈ [-2,2] -> ±32768  
  
static int      g_ntones = 0, g_block_n = 1024, g_cnt = 0;  
static int32_t  g_coeff[GB_MAX_TONES];                 // Q14  
static int32_t  g_q1[GB_MAX_TONES], g_q2[GB_MAX_TONES]; // состояние резонатора
  
void gb_init(const gb_cfg_t* c) {  
    g_ntones  = (c->ntones > GB_MAX_TONES) ? GB_MAX_TONES : c->ntones;  
    g_block_n = c->block_n;  
    for (int i = 0; i < g_ntones; i++) {  
        float f = c->base_hz + i * c->step_hz;  
        float w = 2.0f * (float)M_PI * f / c->fs_hz;  
        // Q14. cosf/lroundf — только при инициализации (холодный путь).  
        g_coeff[i] = (int32_t)lroundf(2.0f * cosf(w) * (float)(1 << GB_COEFF_SHIFT));  
    }
    gb_reset();  
}  
  
void gb_reset(void) {  
    memset(g_q1, 0, sizeof(g_q1));  
    memset(g_q2, 0, sizeof(g_q2));  
    g_cnt = 0;  
}
  
// Рекуррент Гёрцеля на целых: на отсчёт — 1 расширяющее умножение 32x32->64  
// + сдвиг + сложения на тон. Состояние q1/q2 в int32 (при block_n<=4096 и  
// полношкальном входе рост <~8e7, запас к 2^31 большой).  
bool gb_push(int16_t x, float* pwr) {  
    int32_t s = (int32_t)x;  
    for (int i = 0; i < g_ntones; i++) {  
        int32_t q1 = g_q1[i];  
        int32_t q2 = g_q2[i];  
        // coeff(Q14) * q1 -> 64 бита, затем >>14. coeff*q1 ~1.3e12 не влезает в int32.  
        int32_t q0 = (int32_t)(((int64_t)g_coeff[i] * q1) >> GB_COEFF_SHIFT) - q2 + s;  
        g_q2[i] = q1;  
        g_q1[i] = q0;  
    }  
    if (++g_cnt >= g_block_n) {               // граница блока (холодный путь ~раз в N отсчётов)  
        for (int i = 0; i < g_ntones; i++) {  
            int64_t q1 = g_q1[i];  
            int64_t q2 = g_q2[i];  
            int64_t cross = ((int64_t)g_coeff[i] * q1) >> GB_COEFF_SHIFT; // = coeff*q1 (Q0)  
            int64_t p = q1 * q1 + q2 * q2 - cross * q2;                   // |X|^2, до 1.5e15  
            if (p < 0) p = 0;  
            pwr[i] = (float)p;                // argmax/сглаживание в rx_ifkp — масштаб не важен  
            g_q1[i] = g_q2[i] = 0;  
        }  
        g_cnt = 0;  
        return true;  
    }  
    return false;  
}
