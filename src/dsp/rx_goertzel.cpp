#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "../dsp/rx_goertzel.h"

#define GB_COEFF_SHIFT 14
#define GB_DB_FLOOR    (-120.0f)

static int      g_ntones = 0;
static int      g_block_n = 1024;
static int      g_cnt = 0;
static int32_t  g_coeff[GB_MAX_TONES];
static int32_t  g_q1[GB_MAX_TONES], g_q2[GB_MAX_TONES];

// Расширенный режим. В обычном режиме эти поля не участвуют в обработке.
static bool            g_extended = false;
static int             g_hop_n = 0;
static int             g_hop_count = 0;
static int16_t         g_ring[GB_MAX_BLOCK];
static int             g_ring_pos = 0;
static int             g_ring_fill = 0;
static gb_power_mode_t g_power_mode = GB_POWER_LINEAR;

// -----------------------------------------------------------------------------
// Сброс рабочих состояний. Не изменяет коэффициенты/конфигурацию.
// -----------------------------------------------------------------------------
void gb_reset(void)
{
    memset(g_q1, 0, sizeof(g_q1));
    memset(g_q2, 0, sizeof(g_q2));
    g_cnt = 0;

    g_hop_count = 0;
    g_ring_pos = 0;
    g_ring_fill = 0;
    memset(g_ring, 0, sizeof(g_ring));
}

// -----------------------------------------------------------------------------
// СТАРЫЙ API. Алгоритм расчёта и расписание IFKP сохранены.
// -----------------------------------------------------------------------------
void gb_init(const gb_cfg_t* c)
{
    // Любой вызов старого API явно выбирает исходный режим.
    g_extended = false;
    g_hop_n = 0;
    g_hop_count = 0;
    g_ring_pos = 0;
    g_ring_fill = 0;
    g_power_mode = GB_POWER_LINEAR;

    if (c == nullptr ||
        c->ntones <= 0 ||
        c->ntones > GB_MAX_TONES ||
        c->block_n <= 0 ||
        c->fs_hz <= 0.0f ||
        c->step_hz < 0.0f) {

        g_ntones = 0;
        g_block_n = 0;
        g_cnt = 0;

        memset(g_coeff, 0, sizeof(g_coeff));
        memset(g_q1, 0, sizeof(g_q1));
        memset(g_q2, 0, sizeof(g_q2));
        return;
    }

    g_ntones = c->ntones;
    g_block_n = c->block_n;

    for (int i = 0; i < g_ntones; i++) {
        const float f = c->base_hz + i * c->step_hz;

        if (f < 0.0f || f >= c->fs_hz * 0.5f) {
            g_ntones = 0;
            g_block_n = 0;
            gb_reset();
            return;
        }

        const float w = 2.0f * (float)M_PI * f / c->fs_hz;

        g_coeff[i] = (int32_t)lroundf(
            2.0f * cosf(w) * (float)(1 << GB_COEFF_SHIFT)
        );
    }

    gb_reset();
}

// -----------------------------------------------------------------------------
// Преобразование линейной мощности в выбранный выходной формат.
// Вызывается только расширенным режимом, чтобы старый результат не менялся.
// -----------------------------------------------------------------------------
static inline float gb_format_power(int64_t p)
{
    if (p < 0) {
        p = 0;
    }

    if (g_power_mode == GB_POWER_DB) {
        if (p == 0) {
            return GB_DB_FLOOR;
        }

        const float db = 10.0f * log10f((float)p);
        return (db < GB_DB_FLOOR) ? GB_DB_FLOOR : db;
    }

    return (float)p;
}

// -----------------------------------------------------------------------------
// Расчёт мощности одного тона по окну в кольцевом буфере.
// g_ring_pos указывает на самый старый отсчёт, когда окно заполнено.
// -----------------------------------------------------------------------------
static void gb_analyze_ring(float* pwr)
{
    for (int i = 0; i < g_ntones; ++i) {
        int32_t q1 = 0;
        int32_t q2 = 0;
        const int32_t coeff = g_coeff[i];

        int idx = g_ring_pos;

        for (int n = 0; n < g_block_n; ++n) {
            const int32_t s = (int32_t)g_ring[idx];

            // Та же целочисленная рекурсия Q14, что в старом режиме.
            const int32_t q0 =
                (int32_t)(((int64_t)coeff * q1) >> GB_COEFF_SHIFT)
                - q2 + s;

            q2 = q1;
            q1 = q0;

            if (++idx >= g_block_n) {
                idx = 0;
            }
        }

        const int64_t a = q1;
        const int64_t b = q2;
        const int64_t cross =
            ((int64_t)coeff * a) >> GB_COEFF_SHIFT;

        int64_t p = a * a + b * b - cross * b;
        if (p < 0) {
            p = 0;
        }

        pwr[i] = gb_format_power(p);
    }
}

// -----------------------------------------------------------------------------
// Расширенная инициализация. Окно/шаг/формат мощности задаются явно.
// -----------------------------------------------------------------------------
bool gb_init_ex(const gb_cfg_t* cfg,
                int window_n,
                int hop_n,
                gb_power_mode_t power_mode)
{
    if (cfg == nullptr) {
        gb_init(nullptr);
        return false;
    }

    const int n = (window_n == 0) ? cfg->block_n : window_n;
    const int hop = (hop_n == 0) ? n : hop_n;

    if (n <= 0 || n > GB_MAX_BLOCK ||
        hop <= 0 || hop > n ||
        (power_mode != GB_POWER_LINEAR && power_mode != GB_POWER_DB)) {
        gb_init(nullptr);
        return false;
    }

    gb_cfg_t local = *cfg;
    local.block_n = n;

    // Используем проверку частот и вычисление коэффициентов существующего API.
    gb_init(&local);

    if (g_ntones <= 0 || g_block_n != n) {
        g_extended = false;
        return false;
    }

    g_extended = true;
    g_hop_n = hop;
    g_hop_count = 0;
    g_ring_pos = 0;
    g_ring_fill = 0;
    g_power_mode = power_mode;

    memset(g_ring, 0, sizeof(g_ring));
    gb_reset();
    return true;
}

// -----------------------------------------------------------------------------
// Старый API gb_push: код сохранён, чтобы IFKP получал те же результаты.
// Если был вызван gb_init_ex(), обычный gb_push() не смешивает режимы.
// -----------------------------------------------------------------------------
bool gb_push(int16_t x, float* pwr)
{
    if (g_extended ||
        g_ntones <= 0 ||
        g_block_n <= 0 ||
        pwr == nullptr) {
        return false;
    }

    int32_t s = (int32_t)x;
    for (int i = 0; i < g_ntones; i++) {
        int32_t q1 = g_q1[i];
        int32_t q2 = g_q2[i];

        int32_t q0 =
            (int32_t)(((int64_t)g_coeff[i] * q1) >> GB_COEFF_SHIFT)
            - q2 + s;

        g_q2[i] = q1;
        g_q1[i] = q0;
    }

    if (++g_cnt >= g_block_n) {
        for (int i = 0; i < g_ntones; i++) {
            int64_t q1 = g_q1[i];
            int64_t q2 = g_q2[i];
            int64_t cross =
                ((int64_t)g_coeff[i] * q1) >> GB_COEFF_SHIFT;

            int64_t p = q1 * q1 + q2 * q2 - cross * q2;
            if (p < 0) {
                p = 0;
            }

            // Историческое поведение: линейная мощность |X|^2.
            pwr[i] = (float)p;

            g_q1[i] = g_q2[i] = 0;
        }

        g_cnt = 0;
        return true;
    }

    return false;
}

// -----------------------------------------------------------------------------
// Расширенный push: окно заполняется один раз, затем анализ выполняется
// каждые g_hop_n новых отсчётов. Для hop_n == window_n окна не перекрываются.
// -----------------------------------------------------------------------------
bool gb_push_ex(int16_t x, float* pwr)
{
    if (!g_extended ||
        g_ntones <= 0 ||
        g_block_n <= 0 ||
        g_hop_n <= 0 ||
        pwr == nullptr) {
        return false;
    }

    g_ring[g_ring_pos] = x;
    if (++g_ring_pos >= g_block_n) {
        g_ring_pos = 0;
    }

    if (g_ring_fill < g_block_n) {
        ++g_ring_fill;

        if (g_ring_fill < g_block_n) {
            return false;
        }

        // Первое окно: ровно block_n отсчётов.
        g_hop_count = 0;
        gb_analyze_ring(pwr);
        return true;
    }

    if (++g_hop_count < g_hop_n) {
        return false;
    }

    g_hop_count = 0;
    gb_analyze_ring(pwr);
    return true;
}
