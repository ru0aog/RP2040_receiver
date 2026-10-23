#include <Arduino.h>  
#include <math.h>  
#include <string.h>  
  
#include "vfo_hardware.h"  
#include "rx_audio.h"  
#include "AudioTools.h"  
#include "AudioTools/Communication/USB/USBAudioStream.h"  

  
// ---- Демодулятор ----  
#define DEMOD_USB  0  
#define DEMOD_LSB  1  
  
// Фиксированное усиление сырых I/Q перед упаковкой в PCM.  
#define RAW_IQ_GAIN 16  
  
// сырой I/Q → rx_dsp_set_demod(RXD_DEMOD_RAW);
// SSB верхняя → rx_dsp_set_demod(RXD_DEMOD_USB);
// SSB нижняя → rx_dsp_set_demod(RXD_DEMOD_LSB).



// ---- Runtime-состояние режима вывода ----  
// Стартовый режим — сырые I/Q (как раньше при RAW_IQ_OUT=1).  
static volatile uint8_t rx_mode     = RX_MODE_SSB;   // старт сразу в SSB-моно  // rx_mode     = RX_MODE_RAW_IQ;  
static volatile uint8_t demod_mode  = DEMOD_USB;   // боковая для RX_MODE_SSB  
static volatile bool    mode_changed = false;      // запрос на сброс ресемплера

// хелпер: перевести (rx_mode, demod_mode) в режим Core 1  
static void push_demod_to_core1(void) {  
    if (rx_mode == RX_MODE_SSB) {  
        rx_dsp_set_demod(demod_mode == DEMOD_LSB ? RXD_DEMOD_LSB : RXD_DEMOD_USB);  
    } else {  
        rx_dsp_set_demod(RXD_DEMOD_RAW);   // сырой I/Q и тест  
    }  
}  
  
void rx_audio_set_mode(uint8_t mode) {  
    if (mode > RX_MODE_TEST) return;  
    if (mode != rx_mode) { rx_mode = mode; mode_changed = true; }  
    push_demod_to_core1();  
}  

void rx_audio_set_demod(uint8_t demod) { demod_mode = demod; push_demod_to_core1(); }
 
// Необязательная АРУ. В текущем режиме вызов оставлен выключенным,  
// чтобы сохранить исходный уровень сигнала.  
static float agc_peak = 4096.0f;  
  
static inline int16_t agc_limit(float x) {  
    const float ax = fabsf(x);  
  
    // Детектор пиков (атака и спад)  
    if (ax > agc_peak) {  
        // agc_peak += (ax - agc_peak) * 0.0625f;  
        agc_peak += (ax - agc_peak) * 0.01f; 
    } else {  
        agc_peak += (ax - agc_peak) * 0.0002f;  
    }  
    // Защита от разгона усиления в паузах (минимальный порог)  
    // Поднимите выше 2048.0f, если тихие звуки всё ещё слишком сильно усиливаются  
    if (agc_peak < 2048.0f) {  // 256.0f  максимальное усиление 96 крат  
        agc_peak = 2048.0f;    // 2048.0f максимальное усиление 12 крат  
    }  
    float g = x * 24576.0f / agc_peak;  // потолок усиления 24576.0f = -2,5 дБ  
    // Жесткий лимитер на крайний случай  
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
  
    const float m = (float)bi[pos];  // уже демодулировано на Core 1 (моно в aud_i)  
    ++pos;  
  
    *out = m;  
    return true;
}  
  
// Забирает следующую пару I/Q без демодуляции и фильтрации.  
// Та же логика освобождения блока, что и в get_demod_input.  
static bool get_iq_input(float* i_out, float* q_out) {  
    if (i_out == nullptr || q_out == nullptr) {  
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
  
        if (cur_n == 0 || bi == nullptr || bq == nullptr) {  
            rx_dsp_release_audio();  
            cur_n = 0;  
            pos = 0;  
            bi = nullptr;  
            bq = nullptr;  
            return false;  
        }  
    }  
  
    *i_out = (float)bi[pos];  
    *q_out = (float)bq[pos];  
    ++pos;  
    return true;  
}  

  
// ---- Ресемплер 15625 -> 44100 Гц ----  
// rs_phase — дробная позиция между rs_x0 и rs_x1, в единицах 1/44100.  
// На каждом выходном отсчёте фаза увеличивается на 15625.  
// Если фаза пересекает 44100, на следующем вызове нужно сдвинуть входную  
// пару на один отсчёт. Отложенный сдвиг сохраняется при временно пустой очереди.  
static float rs_x0 = 0.0f;  
static float rs_x1 = 0.0f;  
static bool rs_started = false;  
static uint8_t rs_init_count = 0;  
static bool rs_advance_pending = false;  

// rx_audio.cpp  
static float rs_phase = 0.0f;  
// Адаптивный шаг ресемплера. Номинал 15625 (отношение 15625/44100).  
// Подстраивается по заполнению aud-очереди в rx_audio_poll().  
static float rs_step = 15625.0f;  
  
// 1/Fout: умножение дешевле программного float-деления на RP2040 (нет FPU).  
static const float RS_INV_OUT = 1.0f / 44100.0f;

// Второй канал (Q) ресемплера. Фаза rs_phase общая для I и Q,  
// чтобы каналы не разъехались во времени.  
static float rs_q0 = 0.0f;  
static float rs_q1 = 0.0f;  
  
static inline int16_t raw_iq_clip(float x) {  
    float g = x * (float)RAW_IQ_GAIN;  
    if (g > 32767.0f)  g = 32767.0f;  
    if (g < -32768.0f) g = -32768.0f;  
    return (int16_t)g;  
}  


static bool next_frame(int16_t* l, int16_t* r) {  
    if (l == nullptr || r == nullptr) {  
        return false;  
    }  
  
    // Запрос смены режима: сбрасываем ресемплер и отпускаем удерживаемый блок,  
    // чтобы стерео(2 канала) и моно(1 канал) не перемешались в интерполяторе.  
    if (mode_changed) {  
        mode_changed      = false;  
        rs_started        = false;  
        rs_init_count     = 0;  
        rs_advance_pending = false;  
        rs_phase          = 0;  
        if (cur_n != 0) {  
            rx_dsp_release_audio();  
            cur_n = 0;  
            pos   = 0;  
            bi    = nullptr;  
            bq    = nullptr;  
        }  
        return false;   // один пустой кадр на переключении — несущественно  
    }  
  
    if (rx_mode == RX_MODE_TEST) {  
        // Тестовый синус 1000 Гц при выходной частоте 44100 Гц.  
        static uint32_t ph = 0;  
        ph += (uint32_t)(((uint64_t)1000 << 32) / 44100);  
        const float s = 10000.0f *  
            sinf(6.2831853071795864769f * (float)(ph >> 24) / 256.0f);  
        *l = (int16_t)s;  
        *r = (int16_t)s;  
        return true;  
    }  
  
    if (rx_mode == RX_MODE_RAW_IQ) {  
        // Сырые I/Q, общая фаза, линейная интерполяция 15625 -> 44100 на оба канала.  
        if (!rs_started) {  
            if (rs_init_count == 0) {  
                if (!get_iq_input(&rs_x0, &rs_q0)) return false;  
                rs_init_count = 1;  
            }  
            if (rs_init_count == 1) {  
                if (!get_iq_input(&rs_x1, &rs_q1)) return false;  
                rs_init_count = 2;  
            }  
            rs_phase = 0;  
            rs_advance_pending = false;  
            rs_started = true;  
        }  
  
        if (rs_advance_pending) {  
            float next_i, next_q;  
            if (!get_iq_input(&next_i, &next_q)) return false;  
            rs_x0 = rs_x1;  
            rs_x1 = next_i;  
            rs_q0 = rs_q1;  
            rs_q1 = next_q;  
            rs_advance_pending = false;  
        }  
  
        // RX_MODE_RAW_IQ  
        const float frac = rs_phase * RS_INV_OUT;  
        const float yi = rs_x0 + (rs_x1 - rs_x0) * frac;  
        const float yq = rs_q0 + (rs_q1 - rs_q0) * frac; 
    
        rs_phase += rs_step;  
        if (rs_phase >= 44100.0f) {  
            rs_phase -= 44100.0f;  
            rs_advance_pending = true;  
        }
  
        *l = raw_iq_clip(yi);  // I -> левый  
        *r = raw_iq_clip(yq);  // Q -> правый  
        return true;  
    }  
  
    // RX_MODE_SSB: демодулированный моно.  
    if (!rs_started) {  
        if (rs_init_count == 0) {  
            if (!get_demod_input(&rs_x0)) return false;  
            rs_init_count = 1;  
        }  
        if (rs_init_count == 1) {  
            if (!get_demod_input(&rs_x1)) return false;  
            rs_init_count = 2;  
        }  
        rs_phase = 0;  
        rs_advance_pending = false;  
        rs_started = true;  
    }  
  
    if (rs_advance_pending) {  
        float next_x;  
        if (!get_demod_input(&next_x)) return false;  
        rs_x0 = rs_x1;  
        rs_x1 = next_x;  
        rs_advance_pending = false;  
    }  
  
    // RX_MODE_SSB  
    const float frac = rs_phase * RS_INV_OUT;  
    const float y = rs_x0 + (rs_x1 - rs_x0) * frac; 
  
    rs_phase += rs_step;  
    if (rs_phase >= 44100.0f) {  
        rs_phase -= 44100.0f;  
        rs_advance_pending = true;  
    }
  
    const int16_t s = agc_limit(y);  
    *l = s;  
    *r = s;  
    return true;  
}  

  
static USBAudioStream usb_out;  
static int16_t pcm_buffer[192 * 2];  // до 192 stereo frames  
  
void rx_audio_init(void) {  
    agc_peak = 4096.0f;  
    memset(pcm_buffer, 0, sizeof(pcm_buffer));  
    // Синхронизируем демодулятор Core 1 со стартовым режимом (rx_mode/demod_mode),    
    // иначе Core 1 остаётся в RXD_DEMOD_RAW и боковая не давится.    
    push_demod_to_core1();

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
    // --- Адаптивная подстройка темпа выдачи под часы USB-хоста ---  
    // Держим backlog очереди aud около середины (TARGET).  
    // Середина aud-очереди (глубина RX_AUD_BUFS=8 в rx_dsp.cpp).  
    // Литерал, т.к. сам макрос RX_AUD_BUFS в rx_audio.cpp не виден.  
    const float   TARGET = 4.0f; // TARGET = (float)(RX_AUD_BUFS / 2);  // ≈4 при RX_AUD_BUFS=8
    const float   K      = 6.0f;      // крутизна подстройки (Гц на слот)  
    const float   LIMIT  = 60.0f;     // макс. отклонение шага, Гц (~0.4%)  
    const uint8_t fill   = rx_dsp_aud_fill();  
  
    float adj = K * ((float)fill - TARGET);  
    if (adj >  LIMIT) adj =  LIMIT;  
    if (adj < -LIMIT) adj = -LIMIT;  
    rs_step = 15625.0f + adj;   // backlog велик → шаг больше → входы читаются чаще  
  
    for (;;) {  
        int avail_frames = usb_out.availableForWrite() / 4;  
        if (avail_frames <= 0) {  
            break;  
        }  
  
        if (avail_frames > 192) {  
            avail_frames = 192;  
        }  
  
        int frames = 0;  
        int16_t l = 0;  
        int16_t r = 0;  
  
        while (frames < avail_frames && next_frame(&l, &r)) {  
            pcm_buffer[frames * 2]     = l;  
            pcm_buffer[frames * 2 + 1] = r;  
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


uint8_t rx_audio_get_mode(void) { return rx_mode; }  
 
uint8_t rx_audio_get_demod(void) { return demod_mode; }
