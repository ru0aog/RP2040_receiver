#include <Arduino.h>  
#include <math.h>  
#include <stdio.h>  
#include <string.h>  
  
#include "vfo_hardware.h"    
#include "rx_audio.h"    
#include "rx_mf.h"    
#include "rx_rtty.h"    
#include "fldigi_cat.h"    
#include <Adafruit_TinyUSB.h> 
  
static constexpr uint32_t RX_FREQ_HZ = 3600000UL;  
  
// Включение бортового RTTY-декодера (matched-filter mark/space).  
// Сырой I/Q в USB-аудио идёт параллельно по независимой очереди aud.  
#define RTTY_DECODE 1
  
// Текущая частота приёма — единая точка правды для CAT.  
static uint32_t current_rx_hz = RX_FREQ_HZ;  
  
uint32_t rx_current_hz(void) { return current_rx_hz; }  
  
// Полная переустановка приёмного тракта на freq_hz.  
// ВАЖНО: vfo_quadrature_start() и rx_dsp_start() сбрасывают Core 1 —  
// порядок вызовов менять нельзя.  
static void rx_full_tune(uint32_t freq_hz)  
{  
    vfo_subband_apply(freq_hz);  
  
    // LO будет настроен на freq_hz - RX_IF_SHIFT_HZ.  
    vfo_quadrature_start(freq_hz);  
  
    // Запускаем ADC/DMA перед DSP.  
    rx_capture_start();  
  
    // Последним запускаем DSP на Core 1.  
    rx_dsp_start((float)freq_hz);  
  
#if RTTY_DECODE  
    // matched-filter детектор: центр +2450 Гц, разнос 170 Гц (RTTY-45)  
    rx_mf_init(rx_dsp_fs_out(), 2450.0f, 170.0f);  
#endif  
  
    current_rx_hz = freq_hz;  
  
    Serial.printf("# RX_RETUNE,target_hz=%lu,lo_actual_hz=%lu,fs_out_hz=%.2f\n",  
                  (unsigned long)freq_hz,  
                  (unsigned long)vfo_quadrature_lo_hz(),  
                  (double)rx_dsp_fs_out());  
}  
  
// Перестройка на лету. Малая отстройка (в пределах полосы DSP от текущего LO) —  
// сдвиг NCO без перезапуска ядер; большая — полная переустановка LO.  
void rx_retune(uint32_t freq_hz)  
{  
    uint32_t lo_hz     = vfo_quadrature_lo_hz();  
    uint32_t cur_centre = lo_hz + RX_IF_SHIFT_HZ;  
    long     delta     = (long)freq_hz - (long)cur_centre;  
  
    if (delta > -7000 && delta < 7000) {  
        // Остаёмся на том же LO, двигаем только цифровой гетеродин.  
        rx_dsp_set_nco_hz((float)freq_hz - (float)lo_hz);  
        current_rx_hz = freq_hz;
        
        if (delta > -7000 && delta < 7000) {  
            rx_dsp_set_nco_hz((float)freq_hz - (float)lo_hz);  
            current_rx_hz = freq_hz;  
            Serial.printf("# RX_NCO,target_hz=%lu,lo_hz=%lu,nco_hz=%ld\n",  
                        (unsigned long)freq_hz, (unsigned long)lo_hz,  
                        (long)freq_hz - (long)lo_hz);  
        } else {  
            rx_full_tune(freq_hz);  
        }
    } else {  
        rx_full_tune(freq_hz);

        if (delta > -7000 && delta < 7000) {  
            rx_dsp_set_nco_hz((float)freq_hz - (float)lo_hz);  
            current_rx_hz = freq_hz;  
            Serial.printf("# RX_NCO,target_hz=%lu,lo_hz=%lu,nco_hz=%ld\n",  
                        (unsigned long)freq_hz, (unsigned long)lo_hz,  
                        (long)freq_hz - (long)lo_hz);  
        } else {  
            rx_full_tune(freq_hz);  
        }
    }  
}  
  
static void startReceiver()  
{  
    Serial.println("# Initializing receiver");  
  
    vfo_subband_init();  
    rx_full_tune(RX_FREQ_HZ);  
  
    Serial.printf("# RX_STARTED,target_hz=%lu,lo_actual_hz=%lu,fs_out_hz=%.2f\n",  
                  (unsigned long)RX_FREQ_HZ,  
                  (unsigned long)vfo_quadrature_lo_hz(),  
                  (double)rx_dsp_fs_out());  
}
  
void setup()  
{  
    rx_audio_init();  
    Serial.begin(115200);  
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
  
    // Принудительная перенастройка периферийного тактирования на pll_usb (48 МГц),  
    // чтобы TinyUSB и АЦП работали штатно независимо от разгона ядра.  
    clock_configure(  
        clk_peri,  
        0,  
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,  
        48000000UL,  
        48000000UL  
    );  
  
    startReceiver();
  
    // Второй USB-CDC для CAT (FLDigi, Kenwood TS-2000).    
    fldigi_cat_init();  
}  
  
void loop() {  
    rx_audio_poll();
    fldigi_cat_poll();   // неблокирующий разбор CAT-команд    
  
#if RTTY_DECODE  
    // Потребитель базовой полосы 15.625 ksps для matched-filter RTTY.  
    const int16_t* bi;  
    const int16_t* bq;  
    uint8_t n;  
    while (rx_dsp_get_output(&bi, &bq, &n)) {  
        for (uint8_t k = 0; k < n; k++) {  
            rx_mf_sample(bi[k], bq[k]);  
        }  
        rx_dsp_release_output();  
    }  
#endif  
}