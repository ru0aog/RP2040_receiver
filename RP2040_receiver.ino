#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "vfo_hardware.h"
#include <Adafruit_TinyUSB.h>

// ------------------------------------------------------------
// Настройки приёмника
// ------------------------------------------------------------
static constexpr uint32_t RX_FREQ_HZ = 3600000UL;
static constexpr uint32_t SERIAL_BAUD = 115200UL;
static constexpr uint16_t FFT_N = 1024;
static constexpr uint32_t RX_STATUS_INTERVAL_MS = 2000UL;
static constexpr float FFT_WINDOW_GAIN = 0.5f;

static float fftRe[FFT_N];
static float fftIm[FFT_N];
static float hannWindow[FFT_N];
static float spectrumDb[FFT_N];

static uint16_t fftPos = 0;
static bool rxStarted = false;
static bool spectrumTxPending = false;
static int16_t spectrumTxBin = -2; // -2 = заголовок, -1 = #END, 0..255 = bins
static uint32_t lastStatusMs = 0;

// API DSP (объявлены также в vfo_hardware.h).
extern void rx_dsp_start(float f_target_hz);
extern bool rx_dsp_get_output(const int16_t** i, const int16_t** q, uint8_t* n);
extern void rx_dsp_release_output(void);
extern float rx_dsp_fs_out(void);

// ------------------------------------------------------------
// Radix-2 FFT, N=256
// ------------------------------------------------------------
static void fft256(float* re, float* im)
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
    fft256(fftRe, fftIm);

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
    } else {
        spectrumTxPending = false;
        spectrumTxBin = -2;
    }
}

// ------------------------------------------------------------
// Запуск RX
// ------------------------------------------------------------
static void startReceiver()
{
    Serial.println("# Initializing receiver");

    vfo_subband_init();
    vfo_subband_apply(RX_FREQ_HZ);

    // LO будет настроен на RX_FREQ_HZ - RX_IF_SHIFT_HZ.
    // Функция сбрасывает Core 1, поэтому вызывается до запуска DSP.
    vfo_quadrature_start(RX_FREQ_HZ);

    // Запускаем ADC/DMA перед DSP.
    rx_capture_start();

    // Последним запускаем DSP на Core 1.
    // В текущем rx_dsp_entry() DC/NCO-коррекция закомментирована:
    // FFT показывает спектр после фильтрации/децимации без цифрового переноса.
    rx_dsp_start((float)RX_FREQ_HZ);

    Serial.printf("# RX_STARTED,target_hz=%lu,lo_actual_hz=%lu,fs_out_hz=%.2f\n",
                  (unsigned long)RX_FREQ_HZ,
                  (unsigned long)vfo_quadrature_lo_hz(),
                  (double)rx_dsp_fs_out());
    rxStarted = true;
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
  // ожидание коннекта - до 5 сек
  uint32_t timeout = millis();
  while (!Serial && (millis() - timeout < 5000)) {
      #if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
      TinyUSBDevice.task();
      #endif
  }

    for (uint16_t i = 0; i < FFT_N; ++i) {
        hannWindow[i] = 0.5f - 0.5f *
            cosf(2.0f * PI * (float)i / (float)(FFT_N - 1));
    }

    startReceiver();
}

void loop() {
    ReceiverLoop();
}


void ReceiverLoop() 
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
