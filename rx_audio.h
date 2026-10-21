#ifndef RX_AUDIO_H  
#define RX_AUDIO_H  
#include <Arduino.h>  
  
void rx_audio_init(void);   // UAC-микрофон 16 кГц mono — вызвать до startReceiver()  
void rx_audio_poll(void);   // в loop() на core0  
  
#endif