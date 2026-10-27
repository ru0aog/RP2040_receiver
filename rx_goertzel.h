// rx_goertzel.h
#pragma once

#include <stdint.h>
#include <stdbool.h>

// Общий банк Гёрцеля. Один активный декодер одновременно.
// Входная частота проекта: 15625 Гц.
#define GB_MAX_TONES 64
#define GB_MAX_BLOCK 4096

typedef struct {
    int   ntones;       // количество тонов
    float base_hz;      // частота нижнего тона
    float step_hz;      // шаг сетки частот
    float fs_hz;        // частота дискретизации
    int   block_n;      // длина окна анализа
} gb_cfg_t;

// Обычный неперекрывающийся режим.
// Совместим с существующими декодерами.
void gb_init(const gb_cfg_t* cfg);
bool gb_push(int16_t x, float* pwr);

// Перекрывающийся режим.
// block_n — длина окна, hop_n — шаг между результатами.
// Например: окно 500, шаг 125.
void gb_init_overlap(const gb_cfg_t* cfg, int hop_n);
bool gb_push_overlap(int16_t x, float* pwr);

// Сбрасывает состояние текущего режима, сохраняя конфигурацию.
void gb_reset(void);
