// rx_olivia.cpp — основа декодера OLIVIA 8/250.
//
// Вход: монофонический сигнал базовой полосы, Fs = 15625 Гц.
// Банк Гёрцеля: 8 тонов с шагом 31.25 Гц.
//
// ВАЖНО:
// 1. Один символ OLIVIA 8/250 длится 32 мс, а не 250 мс.
// 2. Этот модуль пока собирает статистику мощности тонов.
// 3. Декодирование символов, синхронизация и FEC здесь не реализованы.
// 4. Банк Гёрцеля общий с другими декодерами, поэтому gb_init()
//    должен вызываться только после остановки предыдущего пользователя.

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "rx_olivia.h"
#include "rx_goertzel.h"

// ---- OLIVIA 8/250 ----------------------------------------------------------

#define OLV_NTONES          8
#define OLV_BAND_HZ         250.0f
#define OLV_STEP_HZ         31.25f
#define OLV_BAUD            31.25f

#define OLV_FS_HZ           15625.0f
#define OLV_SYM_SAMPLES     500

// Размер блока Гёрцеля должен поддерживаться rx_goertzel.cpp.
#define OLV_GOERTZEL_N      500

// Центральная частота находится посередине сетки восьми тонов.
#define OLV_TONE0_OFF_HZ    (3.5f * OLV_STEP_HZ)

// Сглаживание результатов анализа.
#define OLV_PWR_ALPHA       0.35f

static bool active = false;

static float tpwr[OLV_NTONES];
static float sym_acc[OLV_NTONES];

static uint32_t sym_phase = 0;
static uint32_t sym_cnt = 0;

// Число полностью обработанных блоков.
static uint32_t blocks_processed = 0;

// Последние мощности восьми тонов доступны для отладки.
static float last_pwr[OLV_NTONES];

// ---------------------------------------------------------------------------
// Инициализация
// ---------------------------------------------------------------------------

void rx_olivia_init(float center_hz)
{
    active = false;

    memset(tpwr, 0, sizeof(tpwr));
    memset(sym_acc, 0, sizeof(sym_acc));
    memset(last_pwr, 0, sizeof(last_pwr));

    sym_phase = 0;
    sym_cnt = 0;
    blocks_processed = 0;

    if (!isfinite(center_hz) ||
        center_hz <= 0.0f ||
        center_hz >= OLV_FS_HZ * 0.5f) {
        return;
    }

    const float base_hz = center_hz - OLV_TONE0_OFF_HZ;

    // Проверяем весь диапазон тонов.
    const float last_hz =
        base_hz + (OLV_NTONES - 1) * OLV_STEP_HZ;

    if (base_hz < 0.0f ||
        last_hz >= OLV_FS_HZ * 0.5f) {
        return;
    }

    const gb_cfg_t cfg = {
        OLV_NTONES,
        base_hz,
        OLV_STEP_HZ,
        OLV_FS_HZ,
        OLV_GOERTZEL_N
    };

    gb_init(&cfg);

    active = true;
}

// ---------------------------------------------------------------------------
// Остановка
// ---------------------------------------------------------------------------

void rx_olivia_stop(void)
{
    active = false;

    sym_phase = 0;
    sym_cnt = 0;

    memset(tpwr, 0, sizeof(tpwr));
    memset(sym_acc, 0, sizeof(sym_acc));
    memset(last_pwr, 0, sizeof(last_pwr));

    // Не вызываем gb_reset(): банк Гёрцеля общий,
    // и его состояние может принадлежать другому декодеру.
}

bool rx_olivia_active(void)
{
    return active;
}

// ---------------------------------------------------------------------------
// Обработка одного входного сэмпла
// ---------------------------------------------------------------------------

void rx_olivia_push(int16_t sample)
{
    if (!active) {
        return;
    }

    static float pwr[OLV_NTONES];

    // Считаем каждый входной сэмпл, а не только готовые блоки.
    ++sym_phase;

    if (!gb_push(sample, pwr)) {
        return;
    }

    ++blocks_processed;

    float total = 0.0f;

    for (int i = 0; i < OLV_NTONES; ++i) {
        // Защита от отрицательных и нечисловых результатов.
        float v = pwr[i];

        if (!isfinite(v) || v < 0.0f) {
            v = 0.0f;
        }

        // Сглаживание мощности.
        tpwr[i] += (v - tpwr[i]) * OLV_PWR_ALPHA;

        // Сохраняем последнее значение для диагностики.
        last_pwr[i] = tpwr[i];

        // Интегрируем мощность по текущему интервалу.
        sym_acc[i] += tpwr[i];

        total += tpwr[i];
    }

    ++sym_cnt;

    // Один символ длится 500 входных сэмплов.
    //
    // ВАЖНО: gb_push() возвращает данные только по завершении блока.
    // Если размер блока не делит 500 без остатка, здесь нельзя
    // просто сбрасывать фазу: граница символа окажется смещена.
    //
    // В текущей конфигурации OLV_GOERTZEL_N == OLV_SYM_SAMPLES,
    // поэтому блок и символ имеют одинаковую длительность.
    if (sym_phase >= OLV_SYM_SAMPLES) {
        sym_phase = 0;

        // Здесь пока нет декодирования символа.
        // sym_acc содержит накопленные мощности тонов,
        // но их нельзя напрямую преобразовывать в ASCII.

        memset(sym_acc, 0, sizeof(sym_acc));
        sym_cnt = 0;
    }
}
