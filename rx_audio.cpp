/**  
 * rx_audio.cpp — демодуляция I/Q 15.625 ksps → PCM 16 kHz mono → UAC-микрофон.  
 * Источник: очередь rx_dsp_get_audio() (выход CIC в rx_dsp, core1).  
 * Вывод: USBAudioStream (arduino-audio-tools), TX_MODE.  
 */  
#include <Arduino.h>  
#include "vfo_hardware.h"  
#include "rx_audio.h"  
#include "AudioTools.h"  
#include "AudioTools/Communication/USB/USBAudioStream.h"  
  
// Коэффициенты biquad LPF Butterworth 2 порядка, Fs=15625 Гц, fc=3000 Гц:  
// b = [0.1933, 0.3866, 0.1933], a = [1, -0.4317, 0.2050]  
static const float LP_B0 = 0.1933f, LP_B1 = 0.3866f, LP_B2 = 0.1933f;  
static const float LP_A1 = -0.4317f, LP_A2 = 0.2050f;  
static float lp_x1 = 0, lp_x2 = 0, lp_y1 = 0, lp_y2 = 0;  
  
static inline float lp_filter(float x) {  
    float y = LP_B0 * x + LP_B1 * lp_x1 + LP_B2 * lp_x2  
            - LP_A1 * lp_y1 - LP_A2 * lp_y2;  
    lp_x2 = lp_x1; lp_x1 = x;  
    lp_y2 = lp_y1; lp_y1 = y;  
    return y;  
}  
  
// ---- NBFM-демодулятор: d = I*Q' - Q*I' ----  
static int32_t prev_i = 0, prev_q = 0;  
  
// АРУ: EMA пика  
static float agc_peak = 4096.0f;  
  
static inline int16_t agc_limit(float x) {  
    float ax = fabsf(x);  
    if (ax > agc_peak) agc_peak += (ax - agc_peak) * 0.0625f;   // атака  
    else               agc_peak += (ax - agc_peak) * 0.0002f;   // спад  
    if (agc_peak < 256.0f) agc_peak = 256.0f;  
    float g = x * 24576.0f / agc_peak;  
    if (g > 32767.0f)  g = 32767.0f;  
    if (g < -32768.0f) g = -32768.0f;  
    return (int16_t)g;  
}  
  
// ---- Источник PCM для StreamCopy: читает очередь, демодулирует, ресемплит ----  
class RxAudioSource : public Stream {  
public:  
    // Stream::readBytes() (невиртуальная) тянет байты через этот read()  
    int read() override {  
        uint8_t b;  
        if (!pullByte(&b)) return -1;  
        return b;  
    }  
  
    int peek() override { return -1; }  
  
    int available() override {  
        // байт доступен, если есть недочитанный PCM или очередь базбенда не пуста  
        if (pcm_pos < pcm_len) return pcm_len - pcm_pos;  
        const int16_t *i, *q; uint8_t n;  
        return rx_dsp_get_audio(&i, &q, &n) ? 512 : 0;  
    }  
  
    size_t write(uint8_t) override { return 1; }  
  
    // собственная эффективная версия — без override,  
    // вызывается напрямую если статический тип известен  
    size_t readBytes(uint8_t* data, size_t len) {  
        size_t got = 0;  
        while (got < len) {  
            int r = read();  
            if (r < 0) break;  
            data[got++] = (uint8_t)r;  
        }  
        return got;  
    }  
  
private:  
    // --- заполнение PCM-буфера одним отсчётом int16 ---  
    bool pullByte(uint8_t* b) {  
        if (pcm_pos >= pcm_len) {  
            if (!fillSample()) return false;  
            pcm_pos = 0;  
        }  
        *b = ((const uint8_t*)&pcm_sample)[pcm_pos++];  
        return true;  
    }  
  
    bool fillSample() {  
        for (;;) {  
            if (pos >= cur_n) {  
                if (!rx_dsp_get_audio(&bi, &bq, &cur_n)) return false;  
                pos = 0;  
            }  
            int32_t i = bi[pos], q = bq[pos];  
            pos++;  
  
            int32_t d = i * prev_q - q * prev_i;  
            prev_i = i; prev_q = q;  
            float a = lp_filter((float)(d >> 8));  
            cur = a;  
  
            acc += 16000;  
            if (acc >= 15625) {  
                acc -= 15625;  
                float frac = (float)acc / 15625.0f;  
                float y = rs_prev + (cur - rs_prev) * frac;  
                pcm_sample = agc_limit(y);  
                pcm_len = 2;  
                rs_prev = cur;  
                return true;  
            }  
            rs_prev = cur;  
        }  
    }  
  
    const int16_t* bi = nullptr;  
    const int16_t* bq = nullptr;  
    uint8_t cur_n = 0, pos = 0;  
    uint32_t acc = 0;  
    float rs_prev = 0, cur = 0;  
    int16_t pcm_sample = 0;  
    uint8_t pcm_pos = 0, pcm_len = 0;  
};
  
static RxAudioSource rx_source;  
static USBAudioStream usb_out;  
static StreamCopy copier(usb_out, rx_source);   // source -> USB  
  
void rx_audio_init(void) {  
    if (!TinyUSBDevice.isInitialized()) {  
        TinyUSBDevice.begin(0);  
    }  
    auto cfg = usb_out.defaultConfig(TX_MODE);  
    cfg.sample_rate      = 16000;  
    cfg.channels         = 1;  
    cfg.bits_per_sample  = 16;  
    usb_out.begin(cfg);  
}  
  
void rx_audio_poll(void) {  
    copier.copy();  
}