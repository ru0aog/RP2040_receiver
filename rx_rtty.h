#pragma once  
  
// Инициализация бодового автомата под частоту вызова decoder'а.  
//   fs — частота, с которой приходят уровни (вызов на КАЖДЫЙ отсчёт базовой  
//   полосы = rx_dsp_fs_out() = 15625 Гц).  
void rx_rtty_init(float fs);  
  
// Подать уровни mark/space ОДНОГО отсчёта (pm = |mark|^2, ps = |space|^2).  
void rx_rtty_frame_levels(float pm, float ps);