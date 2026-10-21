// ============================================================================  
// rx_tones.cpp — скользящий банк Гёрцеля по всей полосе Fs_out  
// Каналы от -0.9*Fs/2 до +0.9*Fs/2 (край ~10% от Найквиста не берём:  
// там завал АЧХ последнего HB-каскада).  
// Шаг канала ~3.9 Гц, окно накопления FRAME_LEN=9 отсчётов (~4.6 мс,  
// ~5 кадров на бит RTTY-45 — передискретизация для бодового PLL).  
// Нагрузка: ~N_CH * 4 MAC на отсчёт.  
// ============================================================================  
  
#include <Arduino.h>  
#include <math.h>  
#include <stdlib.h>  
#include <string.h>  
#include "rx_tones.h"  
#include "rx_rtty.h"  
  
#define FRAME_LEN   9       // 9/1953 = 4.6 мс ≈ 1/5 бита RTTY-45  
#define EDGE_SKIP   25      // крайних каналов от Найквиста не делаем (~48 Гц)  
#define THRESH_DB   9.0f    // порог над медианой (для детектора #TONE, сейчас выкл.)  
  
static float   fs_hz      = 1953.125f;  
static int     n_ch       = 0;         // фактическое число каналов  
const int RX_TONES_N = 500;            // верхняя граница (±976 Гц / 3.906)  
  
// Скользящий Гёрцель с экспоненциальным затуханием (однополюсный  
// «оконный» эквивалент): z = (z + x) * rot; P = Re²+Im²  
// rot = e^{j*2π*f/Fs} * damp, damp < 1 ограничивает память канала.  
#define DAMP  0.94f    // постоянная времени ~17 отсчётов (~9 мс)  
  
static float rot_re[RX_TONES_N];  
static float rot_im[RX_TONES_N];  
static float z_re[RX_TONES_N];  
static float z_im[RX_TONES_N];  
static float power[RX_TONES_N];  
static float buf_sort[RX_TONES_N];   // для медианы  
static uint16_t frame_cnt = 0;  
static float median_p = 1e-9f;  
  
void rx_tones_init(float fs) {  
    fs_hz = fs;  
    // симметричная сетка: ch 0..n_ch-1 -> f = f_lo + ch*step  
    float f_lo  = -0.5f * fs_hz * 0.9f;  
    float f_hi  =  0.5f * fs_hz * 0.9f;  
    float step  = fs_hz / 500.0f;        // ~3.906 Гц  
    n_ch = (int)((f_hi - f_lo) / step);  
    if (n_ch > RX_TONES_N) n_ch = RX_TONES_N;  
  
    for (int ch = 0; ch < n_ch; ch++) {  
        float f = f_lo + ch * step;  
        float w = 2.0f * (float)M_PI * f / fs_hz;  
        rot_re[ch] = DAMP * cosf(w);  
        rot_im[ch] = DAMP * sinf(w);  
        z_re[ch] = z_im[ch] = power[ch] = 0.0f;  
    }  
    frame_cnt = 0;  
}  
  
void rx_tones_sample(int16_t xi, int16_t xq) {  
    for (int ch = 0; ch < n_ch; ch++) {  
        // комплексное обновление: z = (z + x) * rot  
        float nr = z_re[ch] + (float)xi;  
        float ni = z_im[ch] + (float)xq;  
        z_re[ch] = nr * rot_re[ch] - ni * rot_im[ch];  
        z_im[ch] = nr * rot_im[ch] + ni * rot_re[ch];  
    }  
    if (++frame_cnt >= FRAME_LEN) {  
        frame_cnt = 0;  
        rx_tones_frame();  
    }  
}  
  
// медиана через qsort (быстрее сортировки вставками на ~450 элементах)  
static int cmp_f(const void* a, const void* b) {  
    float d = *(const float*)a - *(const float*)b;  
    return (d > 0) - (d < 0);  
}  
static float median_of(float* a, int n) {  
    memcpy(buf_sort, a, n * sizeof(float));  
    qsort(buf_sort, n, sizeof(float), cmp_f);  
    return buf_sort[n / 2];  
}  
  
void rx_tones_frame(void) {  
    for (int ch = 0; ch < n_ch; ch++)  
        power[ch] = z_re[ch]*z_re[ch] + z_im[ch]*z_im[ch];  
    median_p = median_of(power, n_ch) + 1e-9f;  
  
    // декодер RTTY (каждый кадр ~4.6 мс)  
    rx_rtty_frame();  
}  
  
float rx_tones_freq_hz(int ch) {  
    return -0.5f * fs_hz * 0.9f + ch * (fs_hz / 500.0f);  
}  
float rx_tones_power(int ch)  { return power[ch]; }  
float rx_tones_median(void)   { return median_p; }  
int   rx_tones_nch(void)      { return n_ch; }