// ============================================================================  
// rx_tones.cpp — скользящий банк Гёрцеля по всей полосе Fs_out  
// Каналы от -0.9*Fs/2 до +0.9*Fs/2 (край ~10% от Найквиста не берём:  
// там завал АЧХ последнего HB-каскада).  
// Шаг канала ~3.9 Гц, окно накопления FRAME_LEN=48 отсчётов (~24.6 мс).  
// Детектор: порог = медиана(P) + 9 дБ, вывод #TONE в Serial.  
// Нагрузка: ~N_CH * 4 MAC на отсчёт.  
// ============================================================================  
  
#include <Arduino.h>  
#include <math.h>  
#include "rx_tones.h"  
  
#define FRAME_LEN   43      // 43/1953 = 22.02 мс = бит RTTY-45
#define EDGE_SKIP   25      // крайних каналов от Найквиста не делаем (~48 Гц)  
#define THRESH_DB   9.0f    // порог над медианой  
  
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
  
// признак «тон присутствовал в прошлом кадре» для #TONE-сообщений  
static uint8_t tone_on[RX_TONES_N / 8 + 1];  
  
static inline void tone_set_on(int ch, bool on) {  
    if (on) tone_on[ch >> 3] |=  (1 << (ch & 7));  
    else    tone_on[ch >> 3] &= ~(1 << (ch & 7));  
}  
static inline bool tone_get_on(int ch) {  
    return tone_on[ch >> 3] & (1 << (ch & 7));  
}  
  
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
    memset(tone_on, 0, sizeof(tone_on));  
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
  
// простая медиана (сортировка вставками раз в кадр — ~500²/4 сравнений)  
static float median_of(float* a, int n) {  
    for (int i = 0; i < n; i++) buf_sort[i] = a[i];  
    for (int i = 1; i < n; i++) {  
        float v = buf_sort[i]; int j = i - 1;  
        while (j >= 0 && buf_sort[j] > v) { buf_sort[j+1] = buf_sort[j]; j--; }  
        buf_sort[j+1] = v;  
    }  
    return buf_sort[n / 2];  
}  
  
void rx_tones_frame(void) {  
    for (int ch = 0; ch < n_ch; ch++)  
        power[ch] = z_re[ch]*z_re[ch] + z_im[ch]*z_im[ch];
    median_p = median_of(power, n_ch) + 1e-9f;
  /*
    float thresh = median_p * powf(10.0f, THRESH_DB / 10.0f);  
    float f_lo = -0.5f * fs_hz * 0.9f;  
    float step = fs_hz / 500.0f;  
  
    for (int ch = EDGE_SKIP; ch < n_ch - EDGE_SKIP; ch++) {  
        // локальный максимум: выше обоих соседей и выше порога  
        bool peak = power[ch] > thresh  
                 && power[ch] >= power[ch-1]  
                 && power[ch] >  power[ch+1];  
        if (peak && !tone_get_on(ch)) {  
            tone_set_on(ch, true);  
            float f = f_lo + ch * step;  
            float snr = 10.0f * log10f(power[ch] / median_p);  
            Serial.printf("#TONE,ch=%d,freq=%.1f,snr=%.1f\n", ch, f, snr);  
        } else if (!peak && tone_get_on(ch) && power[ch] < thresh * 0.5f) {  
            tone_set_on(ch, false);  
        }  
    }
  */
    rx_rtty_frame();  
}


  
float rx_tones_freq_hz(int ch) {  
    return -0.5f * fs_hz * 0.9f + ch * (fs_hz / 500.0f);  
}  
float rx_tones_power(int ch)  { return power[ch]; }  
float rx_tones_median(void)   { return median_p; }

