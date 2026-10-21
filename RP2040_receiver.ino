#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "rx_tones.h"
#include "vfo_hardware.h"
#include "rx_spectrum.h"
#include "rx_audio.h"
#include <Adafruit_TinyUSB.h>

static constexpr uint32_t RX_FREQ_HZ = 3600000UL;
static constexpr uint32_t SERIAL_BAUD = 115200UL;

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
    // NCO в rx_dsp компенсирует ПЧ-сдвиг ~+7.4 кГц  
    rx_dsp_start((float)RX_FREQ_HZ);  
  
    rx_tones_init(rx_dsp_fs_out());   // банк Гёрцеля: Fs=1953 -> ~500 каналов, шаг ~3.9 Гц  

    Serial.printf("# RX_STARTED,target_hz=%lu,lo_actual_hz=%lu,fs_out_hz=%.2f\n",
                  (unsigned long)RX_FREQ_HZ,
                  (unsigned long)vfo_quadrature_lo_hz(),
                  (double)rx_dsp_fs_out());
    rx_spectrum_set_started(true); 
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

    // clk_sys = 129.333 МГц (VCO 1164 МГц / 9) — при этой частоте делитель  
    // квадратурного LO для 3.6 МГц выходит целым (D=9), джиттера нет.  
    // pll_usb (48 МГц) не трогаем: ADC и USB работают как раньше.  
    set_sys_clock_pll(1164UL * 1000 * 1000, 3, 3);
    rx_spectrum_init();
    rx_audio_init();
    startReceiver();
}

void loop() {
    rx_audio_poll();  
    //FFT_to_COM();
}



