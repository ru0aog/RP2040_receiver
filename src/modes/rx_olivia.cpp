/**
 * ============================================================================
 * rx_olivia.cpp — OLIVIA 8/250 RX для RP2040_receiver.
 *
 * Совместимость: кодировщик из RP2040_beacon/olivia_modem.cpp.
 *
 * Вход:
 *   Fs       = 15625 Гц
 *   Формат   = int16_t, монофонический сигнал с тональной сеткой OLIVIA
 *
 * OLIVIA 8/250:
 *   8 тонов
 *   Шаг между тонами: 31.25 Гц
 *   Длительность символа: 32 мс
 *   Отсчётов на символ: 500
 *   Размер блока кодирования: 64 тона
 *   В одном блоке восстанавливаются 3 символа
 *
 * Кодирование передатчика:
 *   Walsh-Hadamard, SPB=64
 *   Scramble = 0xE257E6D0291574EC
 *   nShift   = 13
 *   7-битный ASCII
 *   Gray-кодирование номера тона
 *
 * ВАЖНО:
 *   Банк Гёрцеля общий с другими декодерами.
 *   gb_init() вызывается только после остановки предыдущего декодера.
 * ============================================================================
 */

#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "../modes/rx_olivia.h"
#include "../modes/rx_decoders.h"
#include "../dsp/rx_goertzel.h"

// ---------------------------------------------------------------------------
// Параметры режима
// ---------------------------------------------------------------------------

#define OLV_NTONES          8
#define OLV_FS_HZ           15625.0f
#define OLV_STEP_HZ         31.25f
#define OLV_SYMBOL_SAMPLES  500
#define OLV_BLOCK_SYMBOLS   64
#define OLV_BLOCK_CHARS     3

#define OLV_CENTER_OFFSET  (3.5f * OLV_STEP_HZ)

static const uint64_t OLV_SCRAMBLE =
    UINT64_C(0xE257E6D0291574EC);

static const uint8_t OLV_NSHIFT = 13;

// Для поиска преамбулы допускаем ошибки классификации тонов.
// 40 совпадений из 64 — начальный порог, который нужно проверить
// на реальном сигнале и при наличии помех.
#define OLV_SYNC_THRESHOLD 40

// ---------------------------------------------------------------------------
// Состояние
// ---------------------------------------------------------------------------

static bool g_active = false;
static bool g_synced = false;

static uint8_t g_tone_history[OLV_BLOCK_SYMBOLS];
static uint8_t g_history_pos = 0;
static uint8_t g_history_count = 0;

static uint8_t g_block_tones[OLV_BLOCK_SYMBOLS];
static uint8_t g_block_count = 0;

static uint32_t g_symbols_seen = 0;
static uint32_t g_blocks_decoded = 0;
static uint32_t g_sync_count = 0;

static float g_power[OLV_NTONES];

// ---------------------------------------------------------------------------
// Вспомогательные функции
// ---------------------------------------------------------------------------

static inline uint8_t olv_to_gray(uint8_t v)
{
    return (uint8_t)(v ^ (v >> 1));
}

static inline uint8_t olv_from_gray(uint8_t g)
{
    uint8_t v = g;

    v ^= v >> 1;
    v ^= v >> 2;
    v ^= v >> 4;

    return (uint8_t)(v & 7u);
}

static inline uint8_t olv_parity(uint32_t x)
{
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    x &= 0x0Fu;

    return (uint8_t)((0x6996u >> x) & 1u);
}

// ---------------------------------------------------------------------------
// Ожидаемый тон для преамбулы.
//
// Передатчик передаёт шесть блоков по три NUL-символа.
// Для каждого NUL кодовое слово Уолша имеет индекс 0 и не инвертируется.
// Поэтому последовательность тонов преамбулы детерминирована.
//
// Возвращается номер тона ПОСЛЕ Gray-кодирования, то есть значение,
// которое должно соответствовать индексу максимальной мощности Гёрцеля.
// ---------------------------------------------------------------------------

static uint8_t olv_expected_preamble_tone(uint8_t t)
{
    uint8_t tone_bits = 0;

    for (uint8_t freq_bit = 0; freq_bit < 3; ++freq_bit) {
        uint8_t code_bit =
            (uint8_t)((freq_bit * OLV_NSHIFT + t) & 63u);

        uint8_t scramble_bit =
            (uint8_t)((OLV_SCRAMBLE >> code_bit) & 1u);

        if (scramble_bit) {
            uint8_t bit_position =
                (uint8_t)((freq_bit + (t % 3u)) % 3u);

            tone_bits |= (uint8_t)(1u << bit_position);
        }
    }

    return olv_to_gray(tone_bits);
}

// ---------------------------------------------------------------------------
// Поиск преамбулы.
//
// Проверяем последние 64 определённых тона. Если последовательность
// достаточно близка к преамбуле передатчика, считаем синхронизацию найденной.
//
// Это первоначальный синхронизатор по известной последовательности.
// Он не выполняет непрерывную подстройку тактовой фазы.
// ---------------------------------------------------------------------------

static bool olv_try_sync(void)
{
    if (g_history_count < OLV_BLOCK_SYMBOLS) {
        return false;
    }

    unsigned matches = 0;

    for (uint8_t i = 0; i < OLV_BLOCK_SYMBOLS; ++i) {
        uint8_t idx =
            (uint8_t)((g_history_pos + i) & 63u);

        if (g_tone_history[idx] == olv_expected_preamble_tone(i)) {
            ++matches;
        }
    }

    return matches >= OLV_SYNC_THRESHOLD;
}

// ---------------------------------------------------------------------------
// Обратное преобразование одного 64-тонального блока.
//
// Входные значения — Gray-кодированные номера тонов, как в передатчике.
//
// Выход:
//   chars[0..2] — три восстановленных 7-битных символа.
//
// Для каждого из трёх битовых потоков восстанавливаем 64-битное слово,
// снимаем скремблирование и ищем ближайшее кодовое слово Уолша.
//
// Полярность кодового слова определяет старший бит ASCII.
// ---------------------------------------------------------------------------

static bool olv_decode_block(
    const uint8_t tones[OLV_BLOCK_SYMBOLS],
    uint8_t chars[OLV_BLOCK_CHARS])
{
    uint8_t row_bits[3][64];

    // 1. Gray -> исходное трёхбитное значение тона.
    // 2. Обратное распределение битов по трём плоскостям.
    // 3. Снятие скремблирования.
    for (uint8_t t = 0; t < 64; ++t) {
        uint8_t tone_value = olv_from_gray(tones[t]);

        for (uint8_t freq_bit = 0; freq_bit < 3; ++freq_bit) {
            uint8_t bit_position =
                (uint8_t)((freq_bit + (t % 3u)) % 3u);

            uint8_t bit =
                (uint8_t)((tone_value >> bit_position) & 1u);

            uint8_t code_bit =
                (uint8_t)((freq_bit * OLV_NSHIFT + t) & 63u);

            bit ^= (uint8_t)((OLV_SCRAMBLE >> code_bit) & 1u);

            row_bits[freq_bit][t] = bit;
        }
    }

    // Восстанавливаем каждый символ из 64-битной последовательности.
    for (uint8_t freq_bit = 0; freq_bit < 3; ++freq_bit) {
        int best_corr = -65;
        uint8_t best_index = 0;
        bool best_inverted = false;

        // Перебираем 64 кодовых слова Уолша.
        for (uint8_t candidate = 0; candidate < 64; ++candidate) {
            int corr = 0;

            const uint8_t parity_candidate =
                olv_parity(candidate);

            for (uint8_t t = 0; t < 64; ++t) {
                // Соответствует build_walsh() в передатчике:
                //
                // bit = parity(candidate & t) XOR parity(candidate)
                //
                // Бит 1 означает отрицательный знак.
                uint8_t expected =
                    (uint8_t)(
                        olv_parity((uint32_t)candidate & t)
                        ^ parity_candidate
                    );

                // Совпадение знаков даёт +1, несовпадение -1.
                corr += (row_bits[freq_bit][t] == expected)
                    ? 1
                    : -1;
            }

            // Проверяем обе полярности слова.
            if (corr > best_corr) {
                best_corr = corr;
                best_index = candidate;
                best_inverted = false;
            }

            if (-corr > best_corr) {
                best_corr = -corr;
                best_index = candidate;
                best_inverted = true;
            }
        }

        // Слабое корреляционное совпадение означает, что блок,
        // вероятно, повреждён или неверно синхронизирован.
        //
        // Не выдаём случайный символ, если нет уверенного решения.
        if (best_corr < 24) {
            return false;
        }

        uint8_t value = best_index;

        if (best_inverted) {
            value |= 0x40u;
        }

        chars[freq_bit] = value & 0x7Fu;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Выдача восстановленных символов.
//
// NUL-символы передатчика используются в преамбуле/постамбуле.
// Управляющие символы, кроме CR/LF/TAB, не выводим.
// ---------------------------------------------------------------------------

static void olv_emit_chars(const uint8_t chars[OLV_BLOCK_CHARS])
{
    for (uint8_t i = 0; i < OLV_BLOCK_CHARS; ++i) {
        const uint8_t c = chars[i];

        if (c == 0) {
            continue;
        }

        if (c == '\r' || c == '\n' || c == '\t' ||
            (c >= 32 && c <= 126)) {
            rx_decoders_emit((char)c);
        }
    }
}

// ---------------------------------------------------------------------------
// Обработка полного 64-тонального блока
// ---------------------------------------------------------------------------

static void olv_process_block(void)
{
    uint8_t chars[OLV_BLOCK_CHARS] = {0, 0, 0};

    ++g_blocks_decoded;

    if (!olv_decode_block(g_block_tones, chars)) {
        // Повреждённый блок пропускаем, не выдавая мусор в текст.
        return;
    }

    olv_emit_chars(chars);
}

// ---------------------------------------------------------------------------
// Инициализация
// ---------------------------------------------------------------------------

void rx_olivia_init(float center_hz)
{
    g_active = false;
    g_synced = false;

    g_history_pos = 0;
    g_history_count = 0;

    g_block_count = 0;

    g_symbols_seen = 0;
    g_blocks_decoded = 0;
    g_sync_count = 0;

    memset(g_tone_history, 0, sizeof(g_tone_history));
    memset(g_block_tones, 0, sizeof(g_block_tones));
    memset(g_power, 0, sizeof(g_power));

    if (!isfinite(center_hz) ||
        center_hz <= 0.0f ||
        center_hz >= OLV_FS_HZ * 0.5f) {
        return;
    }

    // RSID передатчика указывает центр полосы.
    // Нижний тон находится на 3.5 шага ниже центра.
    const float base_hz = center_hz - OLV_CENTER_OFFSET;

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
        OLV_SYMBOL_SAMPLES
    };

    // Используем исходный API, не меняя поведение IFKP.
    gb_init(&cfg);

    g_active = true;
}

// ---------------------------------------------------------------------------
// Остановка
// ---------------------------------------------------------------------------

void rx_olivia_stop(void)
{
    g_active = false;
    g_synced = false;

    g_history_pos = 0;
    g_history_count = 0;
    g_block_count = 0;

    memset(g_tone_history, 0, sizeof(g_tone_history));
    memset(g_block_tones, 0, sizeof(g_block_tones));
    memset(g_power, 0, sizeof(g_power));

    // gb_reset() намеренно не вызываем:
    // общий банк Гёрцеля принадлежит активному декодеру.
}

// ---------------------------------------------------------------------------
// Статус
// ---------------------------------------------------------------------------

bool rx_olivia_active(void)
{
    return g_active;
}

// ---------------------------------------------------------------------------
// Обработка одного входного отсчёта
// ---------------------------------------------------------------------------

void rx_olivia_push(int16_t sample)
{
    if (!g_active) {
        return;
    }

    // Банк возвращает мощности после каждого окна из 500 отсчётов.
    if (!gb_push(sample, g_power)) {
        return;
    }

    ++g_symbols_seen;

    // Выбираем тон с максимальной мощностью.
    uint8_t best_tone = 0;
    float best_power = g_power[0];

    for (uint8_t i = 1; i < OLV_NTONES; ++i) {
        if (g_power[i] > best_power) {
            best_power = g_power[i];
            best_tone = i;
        }
    }

    // Сохраняем историю для поиска преамбулы.
    g_tone_history[g_history_pos] = best_tone;
    g_history_pos =
        (uint8_t)((g_history_pos + 1u) & 63u);

    if (g_history_count < OLV_BLOCK_SYMBOLS) {
        ++g_history_count;
    }

    // Пока не найдена преамбула — ищем известную последовательность.
    if (!g_synced) {
        if (olv_try_sync()) {
            g_synced = true;
            ++g_sync_count;

            // Текущий тон завершил обнаруженную последовательность.
            // Следующий тон начинает следующий 64-тональный блок.
            g_block_count = 0;
        }

        return;
    }

    // После синхронизации собираем блок из 64 тонов.
    g_block_tones[g_block_count++] = best_tone;

    if (g_block_count >= OLV_BLOCK_SYMBOLS) {
        g_block_count = 0;
        olv_process_block();
    }
}
