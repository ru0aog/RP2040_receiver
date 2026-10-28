/**
 * ============================================================================
 * rx_contestia.cpp — CONTESTIA 8/250 для RP2040_receiver
 *
 * Совместимость с RP2040_beacon/olivia_modem.cpp:
 *   Fs             = 15625 Hz
 *   Tones          = 8
 *   Tone spacing   = 31.25 Hz
 *   Symbol samples = 500
 *   Symbols/block  = 32
 *   Characters     = 3 per block
 *   Scrambler      = 0xEDB88320
 *   nShift         = 5
 *   Character code = 6-bit Contestia mapping
 *
 * Общий банк Гёрцеля с другими декодерами не изменяется.
 * ============================================================================
 */

#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "../modes/rx_contestia.h"
#include "../dsp/rx_goertzel.h"
#include "../modes/rx_decoders.h"

// ---------------------------------------------------------------------------
// Параметры
// ---------------------------------------------------------------------------

#define CT_NTONES          8
#define CT_FS_HZ           15625.0f
#define CT_STEP_HZ         31.25f
#define CT_SYMBOL_SAMPLES  500

#define CT_BLOCK_SYMBOLS   32
#define CT_BLOCK_CHARS     3

#define CT_CENTER_OFFSET_HZ (3.5f * CT_STEP_HZ)

// Scrambler и shift из RP2040_beacon/olivia_modem.cpp.
static const uint32_t CT_SCRAMBLE = UINT32_C(0xEDB88320);
static const uint8_t CT_NSHIFT = 5;

// Минимальная корреляция для принятия кодового слова.
// Максимум для 32 позиций — 32.
#define CT_MIN_CORRELATION 14

// Порог обнаружения преамбулы.
#define CT_SYNC_THRESHOLD 24

// ---------------------------------------------------------------------------
// Состояние декодера
// ---------------------------------------------------------------------------

static bool g_active = false;
static bool g_synced = false;

static float g_power[CT_NTONES];

static uint8_t g_history[CT_BLOCK_SYMBOLS];
static uint8_t g_history_pos = 0;
static uint8_t g_history_count = 0;

static uint8_t g_block[CT_BLOCK_SYMBOLS];
static uint8_t g_block_count = 0;

static uint32_t g_symbols_seen = 0;
static uint32_t g_blocks_decoded = 0;

// ---------------------------------------------------------------------------
// Gray-код
// ---------------------------------------------------------------------------

static inline uint8_t ct_from_gray(uint8_t g)
{
    uint8_t v = g;
    v ^= v >> 1;
    v ^= v >> 2;
    v ^= v >> 4;
    return (uint8_t)(v & 7u);
}

// ---------------------------------------------------------------------------
// Чётность числа единичных битов
// ---------------------------------------------------------------------------

static inline uint8_t ct_parity(uint32_t x)
{
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    x &= 0x0Fu;

    return (uint8_t)((0x6996u >> x) & 1u);
}

// ---------------------------------------------------------------------------
// Ожидаемый тон преамбулы.
//
// В передатчике каждый символ NUL кодируется значением 0.
// Для Contestia строка Уолша имеет индекс 0, поэтому до скремблирования
// все её знаки равны нулю.
//
// Получаем ожидаемые 32 тона, включая scrambler и Gray-кодирование.
// ---------------------------------------------------------------------------

static uint8_t ct_expected_preamble_tone(uint8_t t)
{
    uint8_t tone_value = 0;

    for (uint8_t freq_bit = 0; freq_bit < 3; ++freq_bit) {
        const uint8_t code_bit =
            (uint8_t)((freq_bit * CT_NSHIFT + t) & 31u);

        const uint8_t scrambled =
            (uint8_t)((CT_SCRAMBLE >> code_bit) & 1u);

        if (scrambled) {
            const uint8_t bit_position =
                (uint8_t)((freq_bit + (t % 3u)) % 3u);

            tone_value |= (uint8_t)(1u << bit_position);
        }
    }

    return (uint8_t)(tone_value ^ (tone_value >> 1));
}

// ---------------------------------------------------------------------------
// Поиск повторяющейся преамбулы.
//
// В передатчике Contestia использует шесть блоков NUL-символов.
// Каждый блок содержит 32 тона.
//
// После нахождения последовательности начинаем сбор следующего блока.
// ---------------------------------------------------------------------------

static bool ct_try_sync(void)
{
    if (g_history_count < CT_BLOCK_SYMBOLS) {
        return false;
    }

    unsigned matches = 0;

    for (uint8_t i = 0; i < CT_BLOCK_SYMBOLS; ++i) {
        const uint8_t idx =
            (uint8_t)((g_history_pos + i) & 31u);

        if (g_history[idx] == ct_expected_preamble_tone(i)) {
            ++matches;
        }
    }

    return matches >= CT_SYNC_THRESHOLD;
}

// ---------------------------------------------------------------------------
// Обратное кодирование Contestia.
//
// Каждый из трёх битовых потоков кодирует один шестибитный символ.
// Для каждого потока:
//   1. Собираем 32 бита после обратного Gray-кодирования.
//   2. Снимаем скремблирование.
//   3. Сопоставляем с 32 кодовыми словами Уолша.
//   4. Восстанавливаем индекс и старший бит символа.
// ---------------------------------------------------------------------------

static bool ct_decode_block(
    const uint8_t tones[CT_BLOCK_SYMBOLS],
    uint8_t chars[CT_BLOCK_CHARS])
{
    uint8_t plane[3][CT_BLOCK_SYMBOLS];

    // Восстанавливаем три битовых плоскости.
    for (uint8_t t = 0; t < CT_BLOCK_SYMBOLS; ++t) {
        const uint8_t tone_value = ct_from_gray(tones[t]);

        for (uint8_t freq_bit = 0; freq_bit < 3; ++freq_bit) {
            const uint8_t bit_position =
                (uint8_t)((freq_bit + (t % 3u)) % 3u);

            uint8_t bit =
                (uint8_t)((tone_value >> bit_position) & 1u);

            const uint8_t code_bit =
                (uint8_t)((freq_bit * CT_NSHIFT + t) & 31u);

            bit ^= (uint8_t)((CT_SCRAMBLE >> code_bit) & 1u);

            plane[freq_bit][t] = bit;
        }
    }

    // Декодируем каждую плоскость по 32 кодовым словам Уолша.
    for (uint8_t freq_bit = 0; freq_bit < 3; ++freq_bit) {
        int best_corr = -33;
        uint8_t best_index = 0;
        bool best_inverted = false;

        for (uint8_t candidate = 0; candidate < 32; ++candidate) {
            const uint8_t parity_candidate = ct_parity(candidate);

            int corr = 0;

            for (uint8_t t = 0; t < CT_BLOCK_SYMBOLS; ++t) {
                // Соответствует build_walsh() передатчика:
                // row bit = parity(candidate & t) XOR parity(candidate).
                const uint8_t expected =
                    (uint8_t)(
                        ct_parity((uint32_t)candidate & t)
                        ^ parity_candidate
                    );

                corr += (plane[freq_bit][t] == expected) ? 1 : -1;
            }

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

        if (best_corr < CT_MIN_CORRELATION) {
            return false;
        }

        // В Contestia старший бит значения определяется инверсией
        // кодового слова; младшие пять бит — индекс строки Уолша.
        chars[freq_bit] =
            (uint8_t)(best_index | (best_inverted ? 32u : 0u));
    }

    return true;
}

// ---------------------------------------------------------------------------
// Преобразование 6-битного кода в символ.
//
// Таблица соответствует contestia_encode() в передатчике:
//   0      -> NUL/LF
//   1..58  -> ASCII 33..90
//   59     -> пробел
//   60     -> CR
//   61     -> backspace
//   31     -> '?'
// ---------------------------------------------------------------------------

static char ct_decode_character(uint8_t value)
{
    value &= 0x3Fu;

    if (value == 0) {
        return '\0';
    }

    if (value >= 1 && value <= 58) {
        return (char)(value + 32);
    }

    if (value == 59) {
        return ' ';
    }

    if (value == 60) {
        return '\r';
    }

    if (value == 61) {
        return '\b';
    }

    if (value == 31) {
        return '?';
    }

    return '?';
}

// ---------------------------------------------------------------------------
// Вывод трёх символов.
//
// Блок из трёх NUL-символов соответствует idle-преамбуле/постамбуле.
// Его не выводим.
//
// В остальных блоках нулевой символ также пропускаем, чтобы не выводить
// управляющие NUL в Serial.
// ---------------------------------------------------------------------------

static void ct_emit_chars(const uint8_t chars[CT_BLOCK_CHARS])
{
    if (chars[0] == 0 && chars[1] == 0 && chars[2] == 0) {
        return;
    }

    for (uint8_t i = 0; i < CT_BLOCK_CHARS; ++i) {
        const char c = ct_decode_character(chars[i]);

        if (c == '\0' || c == '\b') {
            continue;
        }

        if (c == '\r' || c == '\n' || c == '\t' ||
            (c >= 32 && c <= 126)) {
            rx_decoders_emit(c);
        }
    }
}

// ---------------------------------------------------------------------------
// Обработка полного блока
// ---------------------------------------------------------------------------

static void ct_process_block(void)
{
    uint8_t chars[CT_BLOCK_CHARS] = {0, 0, 0};

    ++g_blocks_decoded;

    if (!ct_decode_block(g_block, chars)) {
        return;
    }

    ct_emit_chars(chars);
}

// ---------------------------------------------------------------------------
// Инициализация
// ---------------------------------------------------------------------------

void rx_contestia_init(float center_hz)
{
    g_active = false;
    g_synced = false;

    g_history_pos = 0;
    g_history_count = 0;
    g_block_count = 0;

    g_symbols_seen = 0;
    g_blocks_decoded = 0;

    memset(g_history, 0, sizeof(g_history));
    memset(g_block, 0, sizeof(g_block));
    memset(g_power, 0, sizeof(g_power));

    if (!isfinite(center_hz) ||
        center_hz <= 0.0f ||
        center_hz >= CT_FS_HZ * 0.5f) {
        return;
    }

    const float base_hz = center_hz - CT_CENTER_OFFSET_HZ;

    const float last_hz =
        base_hz + (CT_NTONES - 1) * CT_STEP_HZ;

    if (base_hz < 0.0f ||
        last_hz >= CT_FS_HZ * 0.5f) {
        return;
    }

    const gb_cfg_t cfg = {
        CT_NTONES,
        base_hz,
        CT_STEP_HZ,
        CT_FS_HZ,
        CT_SYMBOL_SAMPLES
    };

    // Используем тот же API, что и работающая OLIVIA.
    gb_init(&cfg);

    g_active = true;
}

// ---------------------------------------------------------------------------
// Остановка
// ---------------------------------------------------------------------------

void rx_contestia_stop(void)
{
    g_active = false;
    g_synced = false;

    g_history_pos = 0;
    g_history_count = 0;
    g_block_count = 0;

    memset(g_history, 0, sizeof(g_history));
    memset(g_block, 0, sizeof(g_block));
    memset(g_power, 0, sizeof(g_power));

    // Не сбрасываем общий банк Гёрцеля: он может быть передан другому
    // декодеру диспетчером.
}

bool rx_contestia_active(void)
{
    return g_active;
}

// ---------------------------------------------------------------------------
// Обработка одного входного отсчёта
// ---------------------------------------------------------------------------

void rx_contestia_push(int16_t sample)
{
    if (!g_active) {
        return;
    }

    // gb_push() возвращает true после накопления 500 отсчётов.
    if (!gb_push(sample, g_power)) {
        return;
    }

    ++g_symbols_seen;

    // Жёсткое решение по максимальной мощности одного из восьми тонов.
    uint8_t best_tone = 0;
    float best_power = g_power[0];

    for (uint8_t i = 1; i < CT_NTONES; ++i) {
        if (g_power[i] > best_power) {
            best_power = g_power[i];
            best_tone = i;
        }
    }

    // Кольцевой буфер для поиска преамбулы.
    g_history[g_history_pos] = best_tone;
    g_history_pos =
        (uint8_t)((g_history_pos + 1u) & 31u);

    if (g_history_count < CT_BLOCK_SYMBOLS) {
        ++g_history_count;
    }

    if (!g_synced) {
        if (ct_try_sync()) {
            g_synced = true;
            g_block_count = 0;
        }

        return;
    }

    g_block[g_block_count++] = best_tone;

    if (g_block_count >= CT_BLOCK_SYMBOLS) {
        g_block_count = 0;
        ct_process_block();
    }
}
