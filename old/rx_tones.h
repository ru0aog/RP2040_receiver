#pragma once  
#include <stdint.h>  
  
// Инициализация банка. fs_hz — актуальная Fs потока (rx_dsp_fs_out()).  
void rx_tones_init(float fs_hz);  
  
// Подача пары I/Q в банк. Вызывается на каждый отсчёт.  
void rx_tones_sample(int16_t xi, int16_t xq);  
  
// Обновление мощности всех каналов (вызывается раз в кадр, ~9 отсчётов).  
void rx_tones_frame(void);  
  
// Размер банка (верхняя граница)  
extern const int RX_TONES_N;  
// Фактическое число инициализированных каналов  
int rx_tones_nch(void);  
// Частота канала  
float rx_tones_freq_hz(int ch);  
// Мощность канала последнего кадра  
float rx_tones_power(int ch);  
// Медиана мощности по кадру (шумовой пол)  
float rx_tones_median(void);
