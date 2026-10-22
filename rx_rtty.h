#pragma once  
  
// Кадр уровней mark/space от согласованных фильтров (rx_mf).  
// pm, ps — мощности тонов (|z|^2). Вызывается ~5 раз на бит.  
void rx_rtty_frame_levels(float pm, float ps);