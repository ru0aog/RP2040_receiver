#pragma once  
#include <stdint.h>  
  
// Инициализация IFKP-декодера. center_hz — центр сигнала из RSID (r->freq_hz).  
void rx_ifkp_init(float center_hz);  
// Остановить (при переключении на другую моду).  
void rx_ifkp_stop(void);  
// Сток спектра из rx_rsid (каждое FFT-окно ~46 мс), сигнатура rsid_fft_sink_t.  
void rx_ifkp_on_fft(const uint64_t* ampl, int nbins, float bin_hz);
