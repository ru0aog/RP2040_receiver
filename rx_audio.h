#ifndef RX_AUDIO_H  
#define RX_AUDIO_H  
#include <Arduino.h>  
  
void rx_audio_init(void);   // UAC-микрофон 16 кГц mono — вызвать до startReceiver()  
void rx_audio_poll(void);   // в loop() на core0  
  
// Runtime-режимы вывода USB-аудио.  
enum RxAudioMode {  
    RX_MODE_RAW_IQ = 0,   // сырые I/Q в стерео (I=левый, Q=правый)  
    RX_MODE_SSB    = 1,   // демодулированный SSB, моно в оба канала  
    RX_MODE_TEST   = 2,   // тестовый синус 1000 Гц  
};  
  
// Переключение режима на лету (вызывается с core0, напр. из CAT).  
void    rx_audio_set_mode(uint8_t mode);  
uint8_t rx_audio_get_mode(void);  
  
// Выбор боковой/демодулятора для RX_MODE_SSB (DEMOD_USB=0, DEMOD_LSB=1, ...).  
void    rx_audio_set_demod(uint8_t demod);  
uint8_t rx_audio_get_demod(void);
  
#endif