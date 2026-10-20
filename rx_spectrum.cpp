#include "rx_spectrum.h"

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// 4096-point complex FFT, 50% overlap, Hann window.
// The FFT uses a 1/N-scaled Q15 radix-2 implementation to avoid floating-point
// work in the hot path on RP2040 (Cortex-M0+ has no hardware FPU).
namespace {
constexpr uint16_t FFT_N = 4096;
constexpr uint16_t FFT_MASK = FFT_N - 1;
constexpr uint16_t FFT_HALF = FFT_N / 2;
constexpr uint16_t HOP_N = FFT_N / 2;
constexpr uint8_t PEAKS_OUT = 24;
constexpr uint8_t PEAK_SEPARATION_BINS = 3;
constexpr size_t TX_CAPACITY = 1400;
constexpr size_t TX_CHUNK_MAX = 32;

static int16_t ring_i[FFT_N];
static int16_t ring_q[FFT_N];

// Данные FFT храним с дополнительными 8 битами точности.
// Это уменьшает потерю слабых сигналов при делении на 2
// на каждом этапе FFT.
static int32_t fft_i[FFT_N];
static int32_t fft_q[FFT_N];

static int16_t window_q15[FFT_N];
static int16_t tw_re[FFT_HALF];
static int16_t tw_im[FFT_HALF];

static uint16_t ring_write = 0;
static uint16_t samples_seen = 0;
static uint16_t samples_since_frame = 0;
static volatile bool frame_due = false;
static uint32_t frame_seq = 0;
static uint32_t skipped_frames = 0;
static float fs_hz = 15625.0f;

static char txbuf[TX_CAPACITY];
static size_t tx_len = 0;
static size_t tx_pos = 0;
static bool running = false;

struct Peak {
  uint16_t bin;
  uint64_t power;
};
static Peak peaks[PEAKS_OUT];

static inline int16_t sat16(int32_t x) {
  if (x > 32767) return 32767;
  if (x < -32768) return -32768;
  return (int16_t)x;
}

static inline uint64_t bin_power(uint16_t k)
{
  const int64_t re = fft_i[k];
  const int64_t im = fft_q[k];

  return (uint64_t)(re * re + im * im);
}

static void init_tables() {
  for (uint16_t n = 0; n < FFT_N; ++n) {
    const float w = 0.5f - 0.5f * cosf(6.2831853071795864769f *
                                      (float)n / (float)(FFT_N - 1));
    window_q15[n] = sat16((int32_t)(w * 32767.0f + 0.5f));
  }
  for (uint16_t k = 0; k < FFT_HALF; ++k) {
    const float a = -6.2831853071795864769f * (float)k / (float)FFT_N;
    tw_re[k] = sat16((int32_t)(cosf(a) * 32767.0f));
    tw_im[k] = sat16((int32_t)(sinf(a) * 32767.0f));
  }
}


static void fft_q15()
{
  static_assert((FFT_N & (FFT_N - 1)) == 0,
                "FFT_N must be a power of two");
  static_assert(FFT_N == 4096,
                "Twiddle table indexing assumes FFT_N = 4096");

  // 1. Bit-reversal permutation.
  uint16_t j = 0;

  for (uint16_t i = 1; i < FFT_N; ++i) {
    uint16_t bit = FFT_N >> 1;

    while (j & bit) {
      j ^= bit;
      bit >>= 1;
    }

    j ^= bit;

    if (i < j) {
      int32_t tmp;

      tmp = fft_i[i];
      fft_i[i] = fft_i[j];
      fft_i[j] = tmp;

      tmp = fft_q[i];
      fft_q[i] = fft_q[j];
      fft_q[j] = tmp;
    }
  }

  // 2. Iterative radix-2 Cooley-Tukey FFT.
  for (uint16_t len = 2; len <= FFT_N; len <<= 1) {
    const uint16_t half = len >> 1;

    for (uint16_t base = 0; base < FFT_N; base += len) {
      for (uint16_t k = 0; k < half; ++k) {
        const uint16_t a = base + k;
        const uint16_t b = a + half;

        // W = exp(-j * 2*pi*k/len).
        // Twiddle tables contain exp(-j * 2*pi*index/FFT_N).
        const uint16_t tw_idx =
            (uint16_t)(((uint32_t)k * FFT_N) / len);

        const int32_t wr = tw_re[tw_idx];
        const int32_t wi = tw_im[tw_idx];

        const int32_t ar = fft_i[a];
        const int32_t ai = fft_q[a];
        const int32_t br = fft_i[b];
        const int32_t bi = fft_q[b];

        // t = (br + j*bi) * (wr + j*wi), Q15.
        const int32_t tr = (int32_t)(
            ((int64_t)br * wr - (int64_t)bi * wi) >> 15
        );

        const int32_t ti = (int32_t)(
            ((int64_t)br * wi + (int64_t)bi * wr) >> 15
        );

        // Divide each stage by two to limit intermediate growth.
        fft_i[a] = (ar + tr) >> 1;
        fft_q[a] = (ai + ti) >> 1;
        fft_i[b] = (ar - tr) >> 1;
        fft_q[b] = (ai - ti) >> 1;
      }
    }
  }
}


static void take_frame_snapshot_and_fft() {
    // ring_write указывает на самый старый отсчёт,
    // поскольку кольцевой буфер уже заполнен.

    // 1. Оцениваем постоянную составляющую комплексного сигнала.
    int64_t sum_i = 0;
    int64_t sum_q = 0;

    for (uint16_t n = 0; n < FFT_N; ++n) {
        const uint16_t src = (ring_write + n) & FFT_MASK;
        sum_i += ring_i[src];
        sum_q += ring_q[src];
    }

    const int32_t mean_i = (int32_t)(sum_i / FFT_N);
    const int32_t mean_q = (int32_t)(sum_q / FFT_N);

    // 2. Вычитаем среднее, применяем Hann и оставляем
    //    8 дополнительных бит для точности FFT.
    uint16_t src = ring_write;

    for (uint16_t n = 0; n < FFT_N; ++n) {
        const int32_t xi = (int32_t)ring_i[src] - mean_i;
        const int32_t xq = (int32_t)ring_q[src] - mean_q;
        const int32_t w = window_q15[n];

        // Q15 * вход >> 7 => входной сигнал в масштабе x256.
        // Максимальная величина остаётся в пределах int32_t.
        fft_i[n] = (int32_t)(((int64_t)xi * w) >> 7);
        fft_q[n] = (int32_t)(((int64_t)xq * w) >> 7);

        src = (src + 1) & FFT_MASK;
    }

    // 3. Комплексное преобразование.
    fft_q15();


}




static uint8_t find_peaks(Peak* out, uint64_t* max_power_out)
{
  if (max_power_out != nullptr) {
    *max_power_out = 0;
  }

  if (out == nullptr) {
    return 0;
  }

  // Максимальная мощность спектра.
  uint64_t max_power = 0;

  for (uint16_t i = 0; i < FFT_N; ++i) {
    const uint64_t p = bin_power(i);
    if (p > max_power) {
      max_power = p;
    }
  }

  if (max_power_out != nullptr) {
    *max_power_out = max_power;
  }

  if (max_power == 0) {
    return 0;
  }

  const uint64_t min_power = max_power / 1000000ULL;
  uint8_t count = 0;

  // Мощности трёх соседних бинов.
  uint64_t p_prev = bin_power(0);
  uint64_t p = bin_power(1);

  for (uint16_t i = 1; i < FFT_N - 1; ++i) {
    const uint64_t p_next = bin_power(i + 1);

    if (p >= min_power &&
        p > p_prev &&
        p >= p_next) {

      bool too_close = false;

      for (uint8_t k = 0; k < count; ++k) {
        int32_t distance =
            (int32_t)i - (int32_t)out[k].bin;

        if (distance < 0) {
          distance = -distance;
        }

        if (distance > FFT_N / 2) {
          distance = FFT_N - distance;
        }

        if (distance < PEAK_SEPARATION_BINS) {
          too_close = true;

          if (p > out[k].power) {
            out[k].bin = i;
            out[k].power = p;
          }

          break;
        }
      }

      if (!too_close) {
        if (count < PEAKS_OUT) {
          out[count].bin = i;
          out[count].power = p;
          ++count;
        } else {
          uint8_t weakest = 0;

          for (uint8_t k = 1; k < count; ++k) {
            if (out[k].power < out[weakest].power) {
              weakest = k;
            }
          }

          if (p > out[weakest].power) {
            out[weakest].bin = i;
            out[weakest].power = p;
          }
        }
      }
    }

    p_prev = p;
    p = p_next;
  }

  // Сортируем выбранные пики по убыванию мощности.
  for (uint8_t i = 0; i < count; ++i) {
    for (uint8_t j = i + 1; j < count; ++j) {
      if (out[j].power > out[i].power) {
        const Peak tmp = out[i];
        out[i] = out[j];
        out[j] = tmp;
      }
    }
  }

  return count;
}




static int32_t bin_to_signed(uint16_t k) {
  return (k >= FFT_HALF) ? ((int32_t)k - FFT_N) : (int32_t)k;
}


static void build_packet()
{
  uint64_t maxp = 0;
  const uint8_t peak_count = find_peaks(peaks, &maxp);

  tx_len = 0;
  tx_pos = 0;

  int n = snprintf(
      txbuf, sizeof(txbuf),
      "SPEC,seq=%lu,fs=%.3f,n=%u,df=%.6f,span=%.3f\n",
      (unsigned long)frame_seq,
      (double)fs_hz,
      (unsigned)FFT_N,
      (double)(fs_hz / FFT_N),
      (double)fs_hz
  );

  if (n < 0 || (size_t)n >= sizeof(txbuf)) {
    tx_len = tx_pos = 0;
    return;
  }

  tx_len = (size_t)n;

  for (uint8_t p = 0; p < peak_count; ++p) {
    const float rel_db =
        (maxp > 0 && peaks[p].power > 0)
            ? 10.0f * log10f(
                  (float)((double)peaks[p].power / (double)maxp))
            : 0.0f;

    const float freq =
        (float)bin_to_signed(peaks[p].bin) * fs_hz / FFT_N;

    n = snprintf(
        txbuf + tx_len,
        sizeof(txbuf) - tx_len,
        "P,%.2f,%.2f\n",
        (double)freq,
        (double)rel_db
    );

    if (n < 0 || (size_t)n >= sizeof(txbuf) - tx_len) {
      break;
    }

    tx_len += (size_t)n;
  }

  n = snprintf(
      txbuf + tx_len,
      sizeof(txbuf) - tx_len,
      "END,skipped=%lu\n",
      (unsigned long)skipped_frames
  );

  if (n > 0 && (size_t)n < sizeof(txbuf) - tx_len) {
    tx_len += (size_t)n;
  }
}


} // namespace

void rx_spectrum_start(float sample_rate_hz) {
  running = false;
  fs_hz = (sample_rate_hz > 0.0f) ? sample_rate_hz : 15625.0f;
  memset(ring_i, 0, sizeof(ring_i));
  memset(ring_q, 0, sizeof(ring_q));
  memset(fft_i, 0, sizeof(fft_i));
  memset(fft_q, 0, sizeof(fft_q));
  ring_write = 0;
  samples_seen = 0;
  samples_since_frame = 0;
  frame_due = false;
  frame_seq = 0;
  skipped_frames = 0;
  tx_len = tx_pos = 0;
  init_tables();
  running = true;
}

void rx_spectrum_stop(void) {
  running = false;
  frame_due = false;
  tx_len = tx_pos = 0;
}

void rx_spectrum_process(const int16_t* xi, const int16_t* xq, uint8_t n) {
  if (!running || !xi || !xq) return;
  for (uint8_t k = 0; k < n; ++k) {
    ring_i[ring_write] = xi[k];
    ring_q[ring_write] = xq[k];
    ring_write = (ring_write + 1) & FFT_MASK;
    if (samples_seen < FFT_N) {
      ++samples_seen;
      if (samples_seen == FFT_N) {
        // Первый кадр готов сразу после первых 4096 отсчётов.
        frame_due = true;
        samples_since_frame = 0;
      }
    } else {
      ++samples_since_frame;
      if (samples_since_frame >= HOP_N) {
        samples_since_frame = 0;
        if (frame_due) {
          ++skipped_frames;
        } else {
          frame_due = true;
        }
      }
    }
  }
}

void rx_spectrum_poll(void) {
  if (!running) return;

  // Nonblocking serial drain: never wait for the host to read the USB CDC port.
  if (tx_pos < tx_len) {
    int avail = Serial.availableForWrite();
    if (avail > 0) {
      size_t count = (size_t)avail;
      if (count > TX_CHUNK_MAX) count = TX_CHUNK_MAX;
      if (count > tx_len - tx_pos) count = tx_len - tx_pos;
      const size_t sent = Serial.write(
          reinterpret_cast<const uint8_t*>(txbuf + tx_pos), count);
      tx_pos += sent;
      if (tx_pos >= tx_len) tx_len = tx_pos = 0;
    }
    return;
  }

  if (frame_due && samples_seen >= FFT_N) {
    frame_due = false;
    take_frame_snapshot_and_fft();
    ++frame_seq;
    build_packet();
  }
}
