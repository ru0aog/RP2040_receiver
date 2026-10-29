// rx_thor.h — декодер THOR-11 для RP2040_receiver.  
// 18 тонов, шаг 10.7666 Гц, символ ~92.9 мс, IFK+ (+2),  
// FEC k=7 (0x6d/0x4f), перемежитель 4x10, varicode IZ8BLY.  
#ifndef RX_THOR_H  
#define RX_THOR_H  
  
#include <stdint.h>  
  
void rx_thor_init(float center_hz);  // активировать на частоте метки RSID  
void rx_thor_stop(void);             // останов и сброс состояния  
void rx_thor_push(int16_t m);        // пер-сэмпловая точка входа, fs=15625 Гц  
  
#endif // RX_THOR_H