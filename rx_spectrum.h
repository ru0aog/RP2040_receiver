#pragma once  
#include <stdint.h>  
  
// Опциональный FFT-модуль: накопление кадра, расчёт, неблокирующий вывод в Serial.  
void rx_spectrum_init(void);          // вызвать из setup() до startReceiver()  
void rx_spectrum_set_started(bool);   // вызвать из startReceiver() вместо rxStarted=true  
void FFT_to_COM(void);                // вызывать из loop()  
const float* rx_spectrum_db(void);    // spectrumDb[] — для будущего RSID-детектора  
float rx_spectrum_bin_hz(void);       // шаг бина, Гц