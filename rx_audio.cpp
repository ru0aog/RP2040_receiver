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
  
#define TEST_SINE   0   // 1 = тестовый ТОН (работает внутри SDR), 0 = ДЕМОДУЛЯТОР ЭФИРА  
  
// выдать один PCM-отсчёт int16; false = данных нет  
static bool next_sample(int16_t* out) {  
#if TEST_SINE  
    static uint32_t ph = 0;  
    // Генерируем синус 440 Гц, но пересчитанный под частоту шага 44100 Гц!
    ph += (uint32_t)(((uint64_t)1000 << 32) / 44100);  
    *out = (int16_t)(10000.0f * sinf(6.2831853f * (ph >> 24) / 256.0f));  
    return true;  
#else  
    // Ресемплер 15625 -> 44100: на каждый выходной отсчёт  
    // продвигаем входную фазу на 15625/44100  
    for (;;) {  
        // пока не накоплена позиция выхода — тянем входные I/Q  
        while (acc >= 44100) {  
            acc -= 44100;  
            if (pos >= cur_n) {  
                if (cur_n) rx_dsp_release_audio();  
                if (!rx_dsp_get_audio(&bi, &bq, &cur_n)) return false;  
                pos = 0;  
            }  
            int32_t i = bi[pos], q = bq[pos];  
            pos++;  
            // int32_t d = i * prev_q - q * prev_i;  
            // prev_i = i; prev_q = q;  
            rs_prev = cur;  
            // cur = lp_filter((float)(d >> 8));
            cur = lp_filter((float)i * 4.0f);
        }  
        // выдать один выходной отсчёт, линейная интерполяция  
        float frac = (float)acc / 44100.0f;          // позиция между rs_prev и cur  
        float y = rs_prev + (cur - rs_prev) * frac;  
        acc += 15625;  
        // *out = agc_limit(y);  // АРУ
        *out = (int16_t)constrain(y, -32768, 32767); // ограничение

        return true;  
    }  
#endif
}  
  
static USBAudioStream usb_out;
static struct repeating_timer audio_timer;

// Буфер для отправки одного ровного стерео-пакета
static int16_t pcm_buffer[192 * 2];   // до 192 стерео-фреймов

// Аппаратный колбек таймера — вызывается строго каждые 2 миллисекунды
bool __not_in_flash_func(audio_timer_callback)(struct repeating_timer *t) {  
    int avail_frames = usb_out.availableForWrite() / 4;  
    if (avail_frames <= 0) return true;  
  
    if (avail_frames > 192) avail_frames = 192;  
  
    int frames = 0;  
    int16_t s;  
    while (frames < avail_frames && next_sample(&s)) {  
        pcm_buffer[frames * 2]     = s;  
        pcm_buffer[frames * 2 + 1] = s;  
        frames++;  
    }  
    if (frames) usb_out.write((uint8_t*)pcm_buffer, frames * 4);  
    return true;  
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
    // add_repeating_timer_us(-2000, audio_timer_callback, NULL, &audio_timer);
}  
  
void rx_audio_poll(void) {  
    static uint32_t t = 0;  
    if (millis() - t > 1000) {  
        t = millis();  
        const int16_t *i, *q; uint8_t n;  
        bool has = rx_dsp_get_audio(&i, &q, &n);   // peek без release  
        Serial.printf("[AUD] q=%d n=%d i0=%d\n", (int)has, (int)n, has ? i[0] : 0);  
    }  
    for (;;) {  
        int avail_frames = usb_out.availableForWrite() / 4;  
        if (avail_frames <= 0) break;  
        if (avail_frames > 192) avail_frames = 192;  
  
        int frames = 0; int16_t s;  
        while (frames < avail_frames && next_sample(&s)) {  
            pcm_buffer[frames*2] = s; pcm_buffer[frames*2+1] = s; frames++;  
        }  
        if (frames) usb_out.write((uint8_t*)pcm_buffer, frames * 4);  
        if (frames < avail_frames) break;   // источник пуст — выходим  
    }  
}
