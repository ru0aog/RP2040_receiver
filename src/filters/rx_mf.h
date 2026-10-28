#pragma once  
#include <stdint.h>  
  
// Инициализация двух согласованных фильтров RTTY.  
//   fs_hz    — частота базовой полосы (rx_dsp_fs_out() = 15625 Гц)  
//   center_hz — звуковая отстройка центра пары тонов (напр. 2450)  
//   shift_hz  — разнос mark/space (RTTY-45: 170)  
void rx_mf_init(float fs_hz, float center_hz, float shift_hz);  
  
// Подать одну пару I/Q базовой полосы (на каждый отсчёт Fs).  
void rx_mf_sample(int16_t xi, int16_t xq);