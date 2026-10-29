// rx_ax25.h — декодер AX.25 300bd: Bell 202 AFSK (mark 1200 / space 2200 Гц),  
// NRZI + HDLC, FCS CRC-16-CCITT. Работает на Core 0 через rx_decoders_sample().  
#ifndef RX_AX25_H  
#define RX_AX25_H  
  
#include <stdint.h>  
  
void rx_ax25_init(float center_hz);   // активировать декодер на центре RSID  
void rx_ax25_stop(void);              // сброс состояния (flush битового буфера)  
void rx_ax25_push(int16_t m);         // пер-сэмпловая точка входа, fs=15625 Гц  
  
#endif // RX_AX25_H