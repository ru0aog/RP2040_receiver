#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "vfo_hardware.h"
#include "rx_audio.h"
#include "AudioTools.h"
#include "AudioTools/Communication/USB/USBAudioStream.h"

// Входная частота аудиоветви DSP: 15625 Гц.
// Выход USB-аудио: 44100 Гц, stereo, signed 16-bit PCM.

// ---- LPF Butterworth-подобный biquad, Fs=15625 Гц ----
// Коэффициенты сохранены из исходного файла.
static constexpr float LP_B0 = 0.1933f;
static constexpr float LP_B1 = 0.3866f;
static constexpr float LP_B2 = 0.1933f;
static constexpr float LP_A1 = -0.4317f;
static constexpr float LP_A2 = 0.2050f;

static float lp_x1 = 0.0f;
static float lp_x2 = 0.0f;
static float lp_y1 = 0.0f;
static float lp_y2 = 0.0f;

static inline float lp_filter(float x) {
    const float y = LP_B0 * x
                  + LP_B1 * lp_x1
                  + LP_B2 * lp_x2
                  - LP_A1 * lp_y1
                  - LP_A2 * lp_y2;

    lp_x2 = lp_x1;
    lp_x1 = x;
    lp_y2 = lp_y1;
    lp_y1 = y;
    return y;
}

// ---- FM discriminator state ----
static int32_t prev_i = 0;
static int32_t prev_q = 0;

// ---- Демодулятор ----
#define DEMOD_USB  0
#define DEMOD_LSB  1
#define DEMOD_AM   2
#define DEMOD_FM   3
#define DEMOD_CW   4

#define DEMOD_MODE DEMOD_USB

// ---- Hilbert FIR: 31 taps, Fs=15625 Гц ----
#define HILB_N 31
#define HILB_M ((HILB_N - 1) / 2)  // 15 отсчётов групповой задержки

static float hilb_h[HILB_N] = {};
static float hilb_buf[HILB_N] = {};
static int hilb_idx = 0;

// Для задержки ровно на 15 входных отсчётов нужен кольцевой буфер
// длиной 16: читаем элемент, который был записан 15 вызовов назад.
static float i_del[HILB_M + 1] = {};
static int i_del_idx = 0;

static float am_dc = 0.0f;

static void hilb_init(void) {
    for (int k = 0; k < HILB_N; ++k) {
        const int n = k - HILB_M;
        float h = 0.0f;

        // Идеальный импульсный отклик преобразователя Гильберта.
        if ((n & 1) != 0) {
            h = 2.0f / (3.14159265358979323846f * (float)n);
        }

        // Окно Hamming.
        const float w = 0.54f - 0.46f *
            cosf(6.2831853071795864769f * (float)k / (float)(HILB_N - 1));
        hilb_h[k] = h * w;
    }
}

// Подаёт одну пару I/Q (15625 Гц), возвращает один демодулированный отсчёт.
static inline float demod_sample(int32_t i, int32_t q) {
    // Задержка I на 15 отсчётов для согласования с Hilbert FIR.
    i_del[i_del_idx] = (float)i;
    const float i_d = i_del[(i_del_idx + 1) % (HILB_M + 1)];
    i_del_idx = (i_del_idx + 1) % (HILB_M + 1);

    // Q через Hilbert FIR.
    hilb_buf[hilb_idx] = (float)q;
    hilb_idx = (hilb_idx + 1) % HILB_N;

    float hq = 0.0f;
    for (int k = 0; k < HILB_N; ++k) {
        const int idx = (hilb_idx + HILB_N - 1 - k) % HILB_N;
        hq += hilb_h[k] * hilb_buf[idx];
    }

    float dem = 0.0f;

    switch (DEMOD_MODE) {
case DEMOD_USB:
case DEMOD_LSB:
case DEMOD_CW:
    dem = (float)i;
    break;

        case DEMOD_AM: {
            const float fi = (float)i;
            const float fq = (float)q;
            const float env = sqrtf(fi * fi + fq * fq);
            am_dc += (env - am_dc) * 0.001f;
            dem = env - am_dc;
            break;
        }

        case DEMOD_FM:
        default: {
            const int64_t d = (int64_t)i * (int64_t)prev_q
                            - (int64_t)q * (int64_t)prev_i;
            prev_i = i;
            prev_q = q;
            dem = (float)(d >> 8);
            break;
        }
    }

    return dem;
}

// Необязательная АРУ. В текущем режиме вызов оставлен выключенным,
// чтобы сохранить исходный уровень сигнала.
static float agc_peak = 4096.0f;

static inline int16_t agc_limit(float x) {
    const float ax = fabsf(x);

    if (ax > agc_peak) {
        agc_peak += (ax - agc_peak) * 0.0625f;
    } else {
        agc_peak += (ax - agc_peak) * 0.0002f;
    }

    if (agc_peak < 256.0f) {
        agc_peak = 256.0f;
    }

    float g = x * 24576.0f / agc_peak;
    if (g > 32767.0f)  g = 32767.0f;
    if (g < -32768.0f) g = -32768.0f;
    return (int16_t)g;
}

// ---- Источник аудиоданных от rx_dsp ----
static const int16_t* bi = nullptr;
static const int16_t* bq = nullptr;
static uint8_t cur_n = 0;
static uint8_t pos = 0;

// Забирает следующий входной I/Q-отсчёт, демодулирует и фильтрует его.
// Блок освобождается ровно один раз — когда все его отсчёты прочитаны.
static bool get_demod_input(float* out) {
    if (out == nullptr) {
        return false;
    }

    while (cur_n == 0 || pos >= cur_n) {
        if (cur_n != 0) {
            rx_dsp_release_audio();
            cur_n = 0;
            pos = 0;
            bi = nullptr;
            bq = nullptr;
        }

        if (!rx_dsp_get_audio(&bi, &bq, &cur_n)) {
            cur_n = 0;
            pos = 0;
            bi = nullptr;
            bq = nullptr;
            return false;
        }

        pos = 0;

        // Защита от некорректного пустого блока.
        if (cur_n == 0 || bi == nullptr || bq == nullptr) {
            rx_dsp_release_audio();
            cur_n = 0;
            pos = 0;
            bi = nullptr;
            bq = nullptr;
            return false;
        }
    }

    const int32_t i = bi[pos];
    const int32_t q = bq[pos];
    ++pos;

    *out = lp_filter(demod_sample(i, q));
    return true;
}

// ---- Ресемплер 15625 -> 44100 Гц ----
// rs_phase — дробная позиция между rs_x0 и rs_x1, в единицах 1/44100.
// На каждом выходном отсчёте фаза увеличивается на 15625.
// Если фаза пересекает 44100, на следующем вызове нужно сдвинуть входную
// пару на один отсчёт. Отложенный сдвиг сохраняется при временно пустой очереди.
static uint32_t rs_phase = 0;
static float rs_x0 = 0.0f;
static float rs_x1 = 0.0f;
static bool rs_started = false;
static uint8_t rs_init_count = 0;
static bool rs_advance_pending = false;

#define TEST_SINE 0

static bool next_sample(int16_t* out) {
    if (out == nullptr) {
        return false;
    }

#if TEST_SINE
    // Тестовый синус 1000 Гц при выходной частоте 44100 Гц.
    static uint32_t ph = 0;
    ph += (uint32_t)(((uint64_t)1000 << 32) / 44100);
    const float s = 10000.0f *
        sinf(6.2831853071795864769f * (float)(ph >> 24) / 256.0f);
    *out = (int16_t)s;
    return true;
#else
    // Первоначально нужны два входных отсчёта для интерполяции.
    if (!rs_started) {
        if (rs_init_count == 0) {
            if (!get_demod_input(&rs_x0)) {
                return false;
            }
            rs_init_count = 1;
        }

        if (rs_init_count == 1) {
            if (!get_demod_input(&rs_x1)) {
                return false;
            }
            rs_init_count = 2;
        }

        rs_phase = 0;
        rs_advance_pending = false;
        rs_started = true;
    }

    // Если предыдущий выходной отсчёт пересёк границу входного периода,
    // сначала получаем следующий входной отсчёт. При нехватке данных состояние
    // не теряется, и следующий вызов повторит эту операцию.
    if (rs_advance_pending) {
        float next_x;
        if (!get_demod_input(&next_x)) {
            return false;
        }

        rs_x0 = rs_x1;
        rs_x1 = next_x;
        rs_advance_pending = false;
    }

    const float frac = (float)rs_phase / 44100.0f;
    const float y = rs_x0 + (rs_x1 - rs_x0) * frac;

    // Продвигаем фазу только после успешной подготовки и выдачи отсчёта.
    rs_phase += 15625;
    if (rs_phase >= 44100) {
        rs_phase -= 44100;
        rs_advance_pending = true;
    }

    // Ограничение уровня. Для включения АРУ замените эту строку на:
    // *out = agc_limit(y);
    float clipped = y;
    if (clipped > 32767.0f)  clipped = 32767.0f;
    if (clipped < -32768.0f) clipped = -32768.0f;
    *out = (int16_t)clipped;

    return true;
#endif
}

static USBAudioStream usb_out;
static int16_t pcm_buffer[192 * 2];  // до 192 stereo frames

void rx_audio_init(void) {
    // Сброс состояний фильтров и демодулятора.
    lp_x1 = lp_x2 = lp_y1 = lp_y2 = 0.0f;

    memset(hilb_h, 0, sizeof(hilb_h));
    memset(hilb_buf, 0, sizeof(hilb_buf));
    memset(i_del, 0, sizeof(i_del));
    hilb_idx = 0;
    i_del_idx = 0;

    prev_i = 0;
    prev_q = 0;
    am_dc = 0.0f;
    agc_peak = 4096.0f;

    // Сброс состояния источника и ресемплера.
    bi = nullptr;
    bq = nullptr;
    cur_n = 0;
    pos = 0;

    rs_phase = 0;
    rs_x0 = 0.0f;
    rs_x1 = 0.0f;
    rs_started = false;
    rs_init_count = 0;
    rs_advance_pending = false;

    memset(pcm_buffer, 0, sizeof(pcm_buffer));
    hilb_init();

    if (!TinyUSBDevice.isInitialized()) {
        TinyUSBDevice.begin(0);
    }

    auto cfg = usb_out.defaultConfig(TX_MODE);
    cfg.sample_rate = 44100;
    cfg.channels = 2;
    cfg.bits_per_sample = 16;

    usb_out.begin(cfg);
}

// Вызывать регулярно из loop(). Не вызывать параллельно из таймера:
// у USB-потока и ресемплера должен быть один потребитель.
void rx_audio_poll(void) {
    for (;;) {
        int avail_frames = usb_out.availableForWrite() / 4;
        if (avail_frames <= 0) {
            break;
        }

        if (avail_frames > 192) {
            avail_frames = 192;
        }

        int frames = 0;
        int16_t s = 0;

        while (frames < avail_frames && next_sample(&s)) {
            pcm_buffer[frames * 2] = s;
            pcm_buffer[frames * 2 + 1] = s;
            ++frames;
        }

        if (frames > 0) {
            usb_out.write((uint8_t*)pcm_buffer, frames * 4);
        }

        // Источник временно пуст. Не читаем дальше и сохраняем состояние
        // ресемплера до следующего вызова rx_audio_poll().
        if (frames < avail_frames) {
            break;
        }
    }
}
