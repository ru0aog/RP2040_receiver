#include <Arduino.h>  
#include "vfo_hardware.h"  
#include "rx_audio.h"  
#include "AudioTools.h"
#include "pico/util/datetime.h"
#include "hardware/timer.h"
#include "AudioTools/Communication/USB/USBAudioStream.h"  
  
// ---- LPF Butterworth 2 порядка, Fs=15625, fc=3000 ----  
static const float LP_B0 = 0.1933f, LP_B1 = 0.3866f, LP_B2 = 0.1933f;  
static const float LP_A1 = -0.4317f, LP_A2 = 0.2050f;  
static float lp_x1 = 0, lp_x2 = 0, lp_y1 = 0, lp_y2 = 0;  
  
static inline float lp_filter(float x) {  
    float y = LP_B0*x + LP_B1*lp_x1 + LP_B2*lp_x2 - LP_A1*lp_y1 - LP_A2*lp_y2;  
    lp_x2 = lp_x1; lp_x1 = x;  
    lp_y2 = lp_y1; lp_y1 = y;  
    return y;  
}  
  
// ---- NBFM demod + AGC ----  
static int32_t prev_i = 0, prev_q = 0;  
static float agc_peak = 4096.0f;  
  
static inline int16_t agc_limit(float x) {  
    float ax = fabsf(x);  
    if (ax > agc_peak) agc_peak += (ax - agc_peak) * 0.0625f;  
    else               agc_peak += (ax - agc_peak) * 0.0002f;  
    if (agc_peak < 256.0f) agc_peak = 256.0f;  
    float g = x * 24576.0f / agc_peak;  
    if (g >  32767.0f) g =  32767.0f;  
    if (g < -32768.0f) g = -32768.0f;  
    return (int16_t)g;  
}  
  
// ---- состояние источника ----  
static const int16_t* bi = nullptr;  
static const int16_t* bq = nullptr;  
static uint8_t cur_n = 0, pos = 0;  
static uint32_t acc = 0;  
static float rs_prev = 0, cur = 0;  
  
#define TEST_SINE   1   // 1 = тестовый ТОН (работает внутри SDR), 0 = ДЕМОДУЛЯТОР ЭФИРА  
  
// выдать один PCM-отсчёт int16; false = данных нет  
static bool next_sample(int16_t* out) {  
#if TEST_SINE  
    static uint32_t ph = 0;  
    // Генерируем синус 440 Гц, но пересчитанный под частоту шага 44100 Гц!
    ph += (uint32_t)(((uint64_t)440 << 32) / 44100);  
    *out = (int16_t)(10000.0f * sinf(6.2831853f * (ph >> 24) / 256.0f));  
    return true;  
#else  
    // Код демодулятора с шагом ресемплинга из 15625 Гц в 44100 Гц
    for (;;) {  
        if (pos >= cur_n) {  
            if (cur_n) rx_dsp_release_audio();  
            if (!rx_dsp_get_audio(&bi, &bq, &cur_n)) return false;  
            pos = 0;  
        }  
        int32_t i = bi[pos], q = bq[pos];  
        pos++;  
  
        int32_t d = i * prev_q - q * prev_i;  
        prev_i = i; prev_q = q;  
        cur = lp_filter((float)(d >> 8));  
  
        // Ресемплинг: шаг 44100 Гц
        acc += 44100;  
        if (acc >= 15625) {  
            acc -= 15625;  
            float frac = (float)acc / 15625.0f;  
            float y = rs_prev + (cur - rs_prev) * frac;  
            rs_prev = cur;  
            *out = agc_limit(y);  
            return true;  
        }  
        rs_prev = cur;  
    }  
#endif  
}  
  
static USBAudioStream usb_out;
static struct repeating_timer audio_timer;

// Буфер для отправки одного ровного стерео-пакета
static int16_t pcm_buffer[128];  

// Аппаратный колбек таймера — вызывается строго каждые 2 миллисекунды
bool __not_in_flash_func(audio_timer_callback)(struct repeating_timer *t) {
    int avail_bytes = usb_out.availableForWrite();  
    int avail_samples = avail_bytes / 4; // Стерео отсчет = 4 байта

    // Каждые 2 мс для частоты 44100 Гц нам нужно выдать ~88 Стерео-отсчетов.
    // Выдаем фиксированную пачку по 64 отсчета (128 значений int16), если в USB есть место
    if (avail_samples >= 64) {
        int idx = 0;
        while (idx < 64) {
            int16_t mono_sample = 0;
            if (!next_sample(&mono_sample)) {
                break;
            }
            // Превращаем моно в стерео для ПК
            pcm_buffer[idx++] = mono_sample; // Левый канал
            pcm_buffer[idx++] = mono_sample; // Правый канал
        }
        
        if (idx > 0) {
            usb_out.write((uint8_t*)pcm_buffer, idx * 2);
        }
    } else {
        // Если USB буфер полон, всё равно продвигаем генерацию "вхолостую" для стабильности
        int16_t dummy;
        for (int i = 0; i < 32; i++) {
            next_sample(&dummy);
        }
    }
    return true; // Продолжать работу таймера
}

void rx_audio_init(void) {  
    if (!TinyUSBDevice.isInitialized()) {  
        TinyUSBDevice.begin(0);  
    }  
    
    auto cfg = usb_out.defaultConfig(TX_MODE);  
    cfg.sample_rate     = 44100;  
    cfg.channels        = 2;  
    cfg.bits_per_sample = 16;  
    usb_out.begin(cfg);  

    // Запускаем аппаратный таймер на интервал 2000 микросекунд (2 мс)
    // Он будет вызывать audio_timer_callback в фоне независимо от loop()
    add_repeating_timer_us(-2000, audio_timer_callback, NULL, &audio_timer);
}  
  
void rx_audio_poll(void) {  
    // Метод пустой! Вся отправка теперь работает по строгим прерываниям таймера,
    // что полностью убирает цифровой хрип и рассинхронизацию.
}
