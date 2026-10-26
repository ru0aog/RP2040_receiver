#pragma once  
#include <stdint.h>  
  
// Универсальный блочный банк Гёрцеля для MFSK-мод (IFKP, позже Olivia/Contestia/  
// Thor/MFSK16/DominoEX/FSQ). Резонатор на КАЖДУЮ тон-частоту сетки — без  
// квантизации по бинам FFT. Вход — поток 15625 Гц
#define GB_MAX_TONES   64  
  
typedef struct {  
    int   ntones;     // число тонов сетки  
    float base_hz;    // частота нижнего тона (тон 0)  
    float step_hz;    // шаг сетки  
    float fs_hz;      // частота дискретизации входа (15625)  
    int   block_n;    // отсчётов на аналитический блок (разрешение = fs/block_n)  
} gb_cfg_t;  
  
void gb_init(const gb_cfg_t* cfg);  
void gb_reset(void);  
// Подать один отсчёт. Когда накоплен блок — возвращает true и заполняет pwr[ntones].  
bool gb_push(int16_t x, float* pwr);
