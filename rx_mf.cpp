// ============================================================================  
// rx_mf.cpp — согласованные фильтры mark/space для RTTY (подход FLDigi).  
// Каждый тон сносится в DC собственным гетеродином и сглаживается однополюсным  
// НЧ-фильтром с полосой ~ бодовой скорости. Мощность |z|^2 раз в кадр (~1/5  
// бита) подаётся в бодовый автомат rx_rtty.  
// ============================================================================  
#include <Arduino.h>  
#include <math.h>  
#include "rx_mf.h"  
#include "rx_rtty.h"  
  
#define RTTY_BAUD     45.45f  
#define SUBS_PER_BIT  5          // кадров на бит (как в rx_rtty: SUBS=5)  
// Полоса НЧ матч-фильтра ~ бодовой скорости. alpha = 1 - exp(-2*pi*fc/Fs).  
#define MF_LP_FC_HZ   50.0f  
  
static float fs_hz = 15625.0f;  
  
typedef struct {  
    float osc_re, osc_im;   // текущий фазор гетеродина  
    float rot_re, rot_im;   // поворот на -w за отсчёт (сносит +f в DC)  
    float lp_re, lp_im;     // состояние однополюсного НЧ  
} tone_t;  
  
static tone_t t_mark, t_space;  
static float  lp_alpha  = 0.02f;  
static int    frame_div = 69;   // отсчётов на кадр  
static int    frame_cnt = 0;  
  
static void tone_init(tone_t* t, float f_hz) {  
    float w = 2.0f * (float)M_PI * f_hz / fs_hz;  
    t->rot_re =  cosf(w);  
    t->rot_im = -sinf(w);           // знак «минус» => тон +f едет в ноль  
    t->osc_re = 1.0f; t->osc_im = 0.0f;  
    t->lp_re  = t->lp_im = 0.0f;  
}  
  
void rx_mf_init(float fs, float center_hz, float shift_hz) {  
    fs_hz = fs;  
    tone_init(&t_mark,  center_hz + shift_hz * 0.5f);   // mark выше по частоте  
    tone_init(&t_space, center_hz - shift_hz * 0.5f);  
    lp_alpha  = 1.0f - expf(-2.0f * (float)M_PI * MF_LP_FC_HZ / fs_hz);  
    frame_div = (int)(fs_hz / RTTY_BAUD / (float)SUBS_PER_BIT + 0.5f);  
    if (frame_div < 1) frame_div = 1;  
    frame_cnt = 0;  
}  
  
static inline float tone_push(tone_t* t, float xi, float xq) {  
    // снос в DC: y = x * osc  
    float yr = xi * t->osc_re - xq * t->osc_im;  
    float yi = xi * t->osc_im + xq * t->osc_re;  
    // продвижение фазора  
    float nr = t->osc_re * t->rot_re - t->osc_im * t->rot_im;  
    float ni = t->osc_re * t->rot_im + t->osc_im * t->rot_re;  
    t->osc_re = nr; t->osc_im = ni;  
    // однополюсный НЧ (матч-фильтр)  
    t->lp_re += lp_alpha * (yr - t->lp_re);  
    t->lp_im += lp_alpha * (yi - t->lp_im);  
    return t->lp_re * t->lp_re + t->lp_im * t->lp_im;  
}  
  
// быстрая ренормализация фазора к |osc|=1 (борьба с дрейфом float)  
static inline void tone_renorm(tone_t* t) {  
    float m = t->osc_re * t->osc_re + t->osc_im * t->osc_im;  
    float g = 1.5f - 0.5f * m;      // прибл. 1/sqrt вблизи 1  
    t->osc_re *= g; t->osc_im *= g;  
}  
  
void rx_mf_sample(int16_t xi, int16_t xq) {  
    float fi = (float)xi, fq = (float)xq;  
    float pm = tone_push(&t_mark,  fi, fq);  
    float ps = tone_push(&t_space, fi, fq);  
  
    if (++frame_cnt >= frame_div) {  
        frame_cnt = 0;  
        tone_renorm(&t_mark);  
        tone_renorm(&t_space);  
        rx_rtty_frame_levels(pm, ps);  
    }  
}