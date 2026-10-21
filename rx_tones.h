#pragma once  
#include <stdint.h>  
  
// Инициализация банка. freq_hz — актуальная Fs потока (rx_dsp_fs_out()).  
void rx_tones_init(float fs_hz);  
  
// Подача пары I/Q в банк. Вызывается на каждый отсчёт.  
void rx_tones_sample(int16_t xi, int16_t xq);  
  
// Обновление мощности всех каналов (вызывать раз в кадр, ~48 отсчётов).  
void rx_tones_frame(void);  
  
// Размер банка  
extern const int RX_TONES_N;  
// Частота канала  
float rx_tones_freq_hz(int ch);  
// Мощность канала последнего кадра  
float rx_tones_power(int ch);  
// Медиана мощности по кадру (шумовой пол)  
float rx_tones_median(void);

extern void rtty_frame(void);   // или через rx_rtty.h  

// скормить кадр мощностей декодерам (вызывается на каждый кадр ~22 мс)  
extern void rx_rtty_frame(void); 