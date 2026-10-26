#pragma once  
#include <stdint.h>  
  
// Инициализация CW-декодера.  
//   fs_hz    — частота базовой полосы (rx_dsp_fs_out() = 15625 Гц)  
//   tone_hz  — звуковая частота CW-тона (из RSID: r->freq_hz)  
void rx_cw_init(float fs_hz, float tone_hz);  
  
// Подать ОДИН отсчёт базовой полосы (вызывается из get_demod_input).  
void rx_cw_sample(int16_t x);
