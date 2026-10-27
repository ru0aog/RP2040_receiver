#pragma once

#include <stdint.h>
#include <stdbool.h>

#define GB_MAX_TONES 64
#define GB_MAX_BLOCK 4096

typedef struct {
    int   ntones;
    float base_hz;
    float step_hz;
    float fs_hz;
    int   block_n;
} gb_cfg_t;

// Режимы выходной мощности.
typedef enum {
    GB_POWER_LINEAR = 0, // Совместимый режим: |X|^2 в линейном масштабе.
    GB_POWER_DB     = 1  // 10*log10(|X|^2); нулевая мощность ограничивается -120 dB.
} gb_power_mode_t;

// Старый API. Поведение сохраняется для существующих декодеров.
void gb_init(const gb_cfg_t* cfg);
void gb_reset(void);
bool gb_push(int16_t x, float* pwr);

// Расширенный API (один общий банк, одновременно один активный режим).
//
// window_n:
//   0  -> использовать cfg->block_n
//   >0 -> переопределить длину окна
//
// hop_n:
//   0  -> hop равен window_n (обычные неперекрывающиеся окна)
//   >0 -> выдавать результат через hop_n новых отсчётов
//
// power_mode:
//   GB_POWER_LINEAR -> исходный линейный |X|^2
//   GB_POWER_DB     -> 10*log10(|X|^2)
//
// Возвращает false, если конфигурация недопустима.
bool gb_init_ex(const gb_cfg_t* cfg,
                int window_n,
                int hop_n,
                gb_power_mode_t power_mode);

// Псевдоним с явным названием для вызывающего кода.
bool gb_push_ex(int16_t x, float* pwr);
