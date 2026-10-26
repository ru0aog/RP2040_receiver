#pragma once  
#include <stdint.h>  
#include "rx_rsid.h"   // rsid_result_t  
  
// Единая точка вывода символов декодера (по умолчанию — Serial).  
typedef void (*rx_output_cb_t)(char c);  
  
// Инициализация маршрутизатора: регистрирует колбэк в rx_rsid.  
void rx_decoders_init(float fs_hz);  
  
// Подменить приёмник готовых символов (необязательно).  
void rx_decoders_set_output(rx_output_cb_t cb);  
  
// Выдать один декодированный символ (зовут rx_rtty/rx_cw).  
void rx_decoders_emit(char c);  
  
// Колбэк результата RSID: переключение моды + перенастройка фильтров.  
void rx_decoders_on_rsid(const rsid_result_t* r);  
  
// Подать один отсчёт базовой полосы в активный декодер (из rx_audio.cpp).  
void rx_decoders_sample(int16_t m);
