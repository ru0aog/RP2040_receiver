// rx_jtty.h — декодер JTTY (WSJT-X 3.3): 4-FSK, 31.25 Бод, кадр 59 символов.  
// Работает на Core 0 через rx_decoders_sample() — как прочие Goertzel-декодеры.  
#ifndef RX_JTTY_H  
#define RX_JTTY_H  
  
#include <stdint.h>  
  
void rx_jtty_init(float center_hz);   // активировать декодер на центре RSID  
void rx_jtty_stop(void);              // сброс состояния  
void rx_jtty_push(int16_t m);         // пер-сэмпловая точка входа, fs=15625 Гц  
  
#endif // RX_JTTY_H
