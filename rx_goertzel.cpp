#include <Arduino.h>  
#include <math.h>  
#include <string.h>  
#include "rx_goertzel.h"  
  
static int    g_ntones = 0, g_block_n = 1024, g_cnt = 0;  
static float  g_coeff[GB_MAX_TONES];  
static float  g_q1[GB_MAX_TONES], g_q2[GB_MAX_TONES];  
  
void gb_init(const gb_cfg_t* c) {  
    g_ntones  = (c->ntones > GB_MAX_TONES) ? GB_MAX_TONES : c->ntones;  
    g_block_n = c->block_n;  
    for (int i = 0; i < g_ntones; i++) {  
        float f = c->base_hz + i * c->step_hz;  
        float w = 2.0f * (float)M_PI * f / c->fs_hz;  
        g_coeff[i] = 2.0f * cosf(w);          // коэффициент резонатора  
    }  
    gb_reset();  
}  
  
void gb_reset(void) {  
    memset(g_q1, 0, sizeof(g_q1));  
    memset(g_q2, 0, sizeof(g_q2));  
    g_cnt = 0;  
}  
  
// Рекуррент Гёрцеля — 1 MAC на тон на отсчёт (легко для 11025 Гц).  
bool gb_push(int16_t x, float* pwr) {  
    float s = (float)x;  
    for (int i = 0; i < g_ntones; i++) {  
        float q0 = g_coeff[i] * g_q1[i] - g_q2[i] + s;  
        g_q2[i] = g_q1[i];  
        g_q1[i] = q0;  
    }  
    if (++g_cnt >= g_block_n) {               // граница блока: мощность + сброс  
        for (int i = 0; i < g_ntones; i++) {  
            pwr[i] = g_q1[i]*g_q1[i] + g_q2[i]*g_q2[i]  
                     - g_coeff[i]*g_q1[i]*g_q2[i];  
            g_q1[i] = g_q2[i] = 0.0f;  
        }  
        g_cnt = 0;  
        return true;  
    }  
    return false;  
}
