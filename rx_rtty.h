#pragma once  
  
// Скормить кадр мощностей RTTY-декодеру  
// (вызывается из rx_tones на каждый кадр ~4.6 мс).  
void rx_rtty_frame(void);