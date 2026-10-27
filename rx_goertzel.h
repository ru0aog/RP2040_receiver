#pragma once

#include <stdint.h>
#include <stdbool.h>

// Универсальный блочный банк Гёрцеля.
// Входной поток: 15625 Гц.
// Один экземпляр банка обслуживает один активный декодер.
// Внимание: состояние банка глобальное, gb_init() перенастраивает его целиком.

#define GB_MAX_TONES 64

typedef struct {
    int ntones;       // число тонов
    float base_hz;    // частота нижнего тона
    float step_hz;    // шаг частотной сетки
    float fs_hz;      // частота дискретизации
    int block_n;      // размер аналитического блока
} gb_cfg_t;

void gb_init(const gb_cfg_t* cfg);
void gb_reset(void);

// Возвращает true после накопления block_n сэмплов.
// pwr должен указывать на массив размером не менее ntones элементов.
bool gb_push(int16_t x, float* pwr);