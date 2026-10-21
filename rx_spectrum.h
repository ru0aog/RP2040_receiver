#include <stdint.h>





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



void rx_spectrum_start(float sample_rate_hz);
void rx_spectrum_stop(void);
void rx_spectrum_process(const int16_t* xi, const int16_t* xq, uint8_t n);
void rx_spectrum_poll(void);
void FFT_to_COM();
