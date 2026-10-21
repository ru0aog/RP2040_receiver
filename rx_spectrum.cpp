/**    
 * ============================================================================    
 *  rx_spectrum.cpp — FFT-анализ
 *  Работает на core0 после rx_dsp
 *  Fs_in = rx_dsp_get_output ≈ 1.953 кс/с
 * ============================================================================    
 */  

#include "rx_spectrum.h"
#include "rx_capture.cpp"

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>




// ------------------------------------------------------------
// Radix-2 FFT
// ------------------------------------------------------------
static void fftRadix(float* re, float* im)
{
    uint16_t j = 0;

    for (uint16_t i = 1; i < FFT_N; ++i) {
        uint16_t bit = FFT_N >> 1;
        while (j & bit) {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;

        if (i < j) {
            float t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }

    for (uint16_t len = 2; len <= FFT_N; len <<= 1) {
        const float angle = -2.0f * PI / (float)len;
        const float wrStep = cosf(angle);
        const float wiStep = sinf(angle);

        for (uint16_t base = 0; base < FFT_N; base += len) {
            float wr = 1.0f;
            float wi = 0.0f;

            for (uint16_t k = 0; k < len / 2; ++k) {
                const uint16_t a = base + k;
                const uint16_t b = a + len / 2;

                const float vr = re[b] * wr - im[b] * wi;
                const float vi = re[b] * wi + im[b] * wr;

                re[b] = re[a] - vr;
                im[b] = im[a] - vi;
                re[a] += vr;
                im[a] += vi;

                const float nextWr = wr * wrStep - wi * wiStep;
                wi = wr * wiStep + wi * wrStep;
                wr = nextWr;
            }
        }
    }
}

// ------------------------------------------------------------
// Рассчитать спектр и сохранить уровни в порядке от -Fs/2 до +Fs/2.
// ------------------------------------------------------------
static void calculateSpectrum()
{
    fftRadix(fftRe, fftIm);

    for (uint16_t k = 0; k < FFT_N; ++k) {
        const uint16_t b = (uint16_t)((k + FFT_N / 2) % FFT_N);
        const float mag = sqrtf(fftRe[b] * fftRe[b] + fftIm[b] * fftIm[b]);
        const float normalized = mag / ((float)FFT_N * FFT_WINDOW_GAIN);
        spectrumDb[k] = 20.0f * log10f(normalized + 1.0e-12f);
    }

    if (!spectrumTxPending) {
        spectrumTxPending = true;
        spectrumTxBin = -2;
    }
}

// ------------------------------------------------------------
// Неблокирующий вывод CSV. Формат кадра:
// #BEGIN,Fs_Hz,bin_Hz
// frequency_Hz,level_dB
// ... 256 строк ...
// #END
// ------------------------------------------------------------
static void serviceSpectrumSerial()
{
    if (!spectrumTxPending) return;

    char line[64];
    const float fs = rx_dsp_fs_out();
    const float binHz = fs / (float)FFT_N;
    int len = 0;

    if (spectrumTxBin == -2) {
        len = snprintf(line, sizeof(line), "#BEGIN,%.2f,%.4f\n",
                       (double)fs, (double)binHz);
    } else if (spectrumTxBin >= 0 && spectrumTxBin < (int16_t)FFT_N) {
        const int16_t shiftedBin = (int16_t)spectrumTxBin - (int16_t)(FFT_N / 2);
        const float freqHz = (float)shiftedBin * binHz;
        len = snprintf(line, sizeof(line), "%.2f,%.2f\n",
                       (double)freqHz, (double)spectrumDb[spectrumTxBin]);
    } else {
        len = snprintf(line, sizeof(line), "#END\n");
    }

    if (len <= 0 || len >= (int)sizeof(line)) {
        spectrumTxPending = false;
        return;
    }
    if (Serial.availableForWrite() < len) return;

    Serial.write((const uint8_t*)line, (size_t)len);

    if (spectrumTxBin == -2) {
        spectrumTxBin = 0;
    } else if (spectrumTxBin >= 0 && spectrumTxBin < (int16_t)FFT_N - 1) {  
        ++spectrumTxBin;  
    } else if (spectrumTxBin == (int16_t)FFT_N - 1) {  
        spectrumTxBin = -1;   // следующая итерация напечатает #END  
    } else {  
        spectrumTxPending = false;  
        spectrumTxBin = -2;  
    }
}


void FFT_to_COM() 
{
    if (!rxStarted) return;

    // Обрабатываем максимум один DSP-блок за итерацию, затем обслуживаем Serial.
    const int16_t* iBuf = nullptr;
    const int16_t* qBuf = nullptr;
    uint8_t n = 0;

    if (rx_dsp_get_output(&iBuf, &qBuf, &n)) {
        for (uint8_t i = 0; i < n; ++i) {
            fftRe[fftPos] = ((float)iBuf[i] / 32768.0f) * hannWindow[fftPos];
            fftIm[fftPos] = ((float)qBuf[i] / 32768.0f) * hannWindow[fftPos];

            ++fftPos;
            if (fftPos >= FFT_N) {
                fftPos = 0;
                // Пока предыдущий кадр отправляется, новый кадр пропускаем.
                if (!spectrumTxPending) {
                    calculateSpectrum();
                }
            }
        }
        rx_dsp_release_output();
    }

    serviceSpectrumSerial();

    // Статистика печатается только между спектральными кадрами, чтобы не
    // вклинивать строку #STATUS внутрь блока #BEGIN...#END.
    const uint32_t now = millis();
    if (!spectrumTxPending && now - lastStatusMs >= RX_STATUS_INTERVAL_MS) {
        lastStatusMs = now;
        Serial.printf("#STATUS,capture_dropped=%lu\n",
                      (unsigned long)rx_capture_dropped());
    }
}