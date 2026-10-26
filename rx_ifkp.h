#pragma once  
#include <stdint.h>  
  
// Инициализация IFKP-декодера. center_hz — центр сигнала из RSID (r->freq_hz).  
void rx_ifkp_init(float center_hz);  
// Остановить (при переключении на другую моду).  
void rx_ifkp_stop(void);  
// Подать один отсчёт потока 15625 Гц. Внутри — банк Гёрцеля.  
void rx_ifkp_push(int16_t sample);
