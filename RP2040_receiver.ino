
#include <Arduino.h>
#include <math.h>

// ============================================================
// RP2040_receiver.ino
// Назначение:
//   1. Запустить приём при включении.
//   2. Получать блоки комплексных IQ.
//   3. Считать FFT.
//   4. Выдавать спектр в Serial.
// ============================================================

static constexpr uint32_t SERIAL_BAUD = 921600;
static constexpr uint16_t FFT_N = 256;

// Частота децимированного IQ из существующего rx_dsp.
static constexpr float FS_HZ = 15625.0f;

// Выдавать один спектр каждые 100 мс.
static constexpr uint32_t SPECTRUM_PERIOD_MS = 100;

// Буферы FFT.
static float re[FFT_N];
static float im[FFT_N];
static float windowFn[FFT_N];

static uint16_t fillCount = 0;
static uint32_t lastSpectrumMs = 0;

// ------------------------------------------------------------
// Подключение к rx_dsp.
//
// Заменить эти две функции вызовами реального API проекта.
// Не запускать второй ADC, если rx_dsp уже управляет им.
// ------------------------------------------------------------

bool receiverBegin()
{
  // TODO:
  // Запустить существующий rx_dsp и приём ADC/DMA.
  //
  // Например, здесь должны быть вызовы инициализации
  // из rx_dsp.h / rx_dsp.cpp.
  //
  // Вернуть true, когда приём запущен.
  return false;
}

bool receiverReadIQ(int16_t &iSample, int16_t &qSample)
{
  // TODO:
  // Получить следующую пару IQ из rx_dsp.
  //
  // Если API выдаёт блоки:
  //   rx_dsp_get_output(&i, &q, &n);
  // нужно сохранять текущие указатели и индекс блока,
  // а после обработки вызывать rx_dsp_release_output().
  //
  // Вернуть true, если получен новый отсчёт.
  return false;
}

// ------------------------------------------------------------
// FFT: bit reversal + radix-2 Cooley–Tukey.
// ------------------------------------------------------------

void fft256(float *r, float *x)
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
      float t = r[i];
      r[i] = r[j];
      r[j] = t;

      t = x[i];
      x[i] = x[j];
      x[j] = t;
    }
  }

  for (uint16_t len = 2; len <= FFT_N; len <<= 1) {
    const float angle = -2.0f * PI / len;
    const float wlenR = cosf(angle);
    const float wlenI = sinf(angle);

    for (uint16_t base = 0; base < FFT_N; base += len) {
      float wr = 1.0f;
      float wi = 0.0f;

      for (uint16_t k = 0; k < len / 2; ++k) {
        const uint16_t a = base + k;
        const uint16_t b = a + len / 2;

        const float vr = r[b] * wr - x[b] * wi;
        const float vi = r[b] * wi + x[b] * wr;

        const float ur = r[a];
        const float ui = x[a];

        r[a] = ur + vr;
        x[a] = ui + vi;
        r[b] = ur - vr;
        x[b] = ui - vi;

        const float nextWr = wr * wlenR - wi * wlenI;
        wi = wr * wlenI + wi * wlenR;
        wr = nextWr;
      }
    }
  }
}

// ------------------------------------------------------------
// Подготовка и печать спектра.
// Вход: комплексные IQ.
// ------------------------------------------------------------

void processIQ(int16_t iSample, int16_t qSample)
{
  // Периодическое окно Hann.
  const float scale = 1.0f / 32768.0f;

  re[fillCount] = (float)iSample * scale * windowFn[fillCount];
  im[fillCount] = (float)qSample * scale * windowFn[fillCount];

  ++fillCount;

  if (fillCount < FFT_N) {
    return;
  }

  fft256(re, im);

  // Комплексный IQ: спектр отображается от -Fs/2 до +Fs/2.
  // Перестановка половин нужна для удобного вывода.
  Serial.println("BEGIN");

  for (uint16_t outBin = 0; outBin < FFT_N; ++outBin) {
    const uint16_t bin = (outBin + FFT_N / 2) % FFT_N;

    const float mag = sqrtf(
      re[bin] * re[bin] + im[bin] * im[bin]
    );

    const float db = 20.0f * log10f(mag + 1.0e-12f);
    const float freq =
      ((float)outBin - FFT_N / 2) * FS_HZ / FFT_N;

    Serial.print(freq, 1);
    Serial.print(',');
    Serial.println(db, 2);
  }

  Serial.println("END");

  fillCount = 0;
}

// ------------------------------------------------------------

void setup()
{
  Serial.begin(SERIAL_BAUD);

  // Не ждать подключения Serial: приём должен стартовать сразу.
  for (uint16_t n = 0; n < FFT_N; ++n) {
    windowFn[n] =
      0.5f - 0.5f * cosf(2.0f * PI * n / (FFT_N - 1));
  }

  delay(50);

  Serial.println("# RP2040 receiver starting");

  if (!receiverBegin()) {
    Serial.println("# ERROR: connect receiverBegin() to rx_dsp");
    return;
  }

  Serial.println("# RX STARTED");
}

void loop()
{
  static bool rxStarted = false;

  // После ошибки инициализации не пытаемся читать несуществующий поток.
  if (!rxStarted) {
    // В текущем каркасе receiverBegin() вызывается в setup().
    // Успешный запуск отмечается отдельным флагом в рабочей версии.
    return;
  }

  int16_t iSample;
  int16_t qSample;

  while (receiverReadIQ(iSample, qSample)) {
    processIQ(iSample, qSample);
  }
}
