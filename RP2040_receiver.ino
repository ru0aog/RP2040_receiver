#include <Arduino.h>  
#include <math.h>  
#include <stdio.h>  
#include <string.h>  
#include "pico/stdlib.h"   // sleep_ms
#include "hardware/structs/ssi.h"   // ssi_hw  
#include "hardware/vreg.h"          // vreg_set_voltage, VREG_VOLTAGE_1_30  
#include "hardware/clocks.h"        // set_sys_clock_pll, clock_configure
#include "hardware/pll.h"  
#include "hardware/sync.h"      // save_and_disable_interrupts  

#include "vfo_hardware.h"    
#include "rx_audio.h"    
#include "rx_mf.h"    
#include "rx_rtty.h"    
#include "fldigi_cat.h"    
#include <Adafruit_TinyUSB.h> 
  
static constexpr uint32_t RX_FREQ_HZ = 3600000UL;  
  
// Включение бортового RTTY-декодера: определено в vfo_hardware.h (единая точка)
  
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
    uint32_t lo_hz      = vfo_quadrature_lo_hz();  
    uint32_t cur_centre = lo_hz + RX_IF_SHIFT_HZ;  
    long     delta      = (long)freq_hz - (long)cur_centre;  
  
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

    startReceiver();
  
    // Второй USB-CDC для CAT (FLDigi, Kenwood TS-2000).    
    fldigi_cat_init();  
}  
  
// RP2040_receiver.ino  
void loop() {    
    // --- Учёт загрузки Core 0 (по образцу Core 1) ---    
    // total: полное время итерации; busy: время внутри рабочих функций.    
    // Прим.: у Core 0 нет явного idle-маркера, метрика = доля wall-clock,    
    // проведённая в rx_audio_poll()/RTTY, а не «истинная» занятость CPU.    
    static uint32_t t_prev       = 0;    
    static uint32_t busy_us_acc  = 0;    
    static uint32_t total_us_acc = 0;    
    static uint32_t core0_load_permille = 0;    
    static bool     t_prev_init  = false;    
  
    const uint32_t t_now = time_us_32();    
    if (!t_prev_init) { t_prev = t_now; t_prev_init = true; }    
    total_us_acc += (t_now - t_prev);    
    t_prev = t_now;    
  
    const uint32_t t_busy_start = time_us_32();    
  
    rx_audio_poll();
    
    // CAT опрашиваем не каждую итерацию, а раз в ~15 мс: обращения к  
    // TinyUSB-CDC (dtr()/available()) в fldigi_cat_poll() дорогие,  
    // а для разбора CAT-команд такой частоты более чем достаточно.  
    static uint32_t t_cat = 0;  
    if (millis() - t_cat >= 15) {  
        t_cat = millis();  
        fldigi_cat_poll();   // неблокирующий разбор CAT-команд  
    }     
    
#if RTTY_DECODE    
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
  
    busy_us_acc += (time_us_32() - t_busy_start);    
  
    if (total_us_acc >= 1000000u) {    
        core0_load_permille = (uint32_t)(((uint64_t)busy_us_acc * 1000u) / total_us_acc);    
        busy_us_acc  = 0;    
        total_us_acc = 0;    
    }    
  
    // Диагностика: загрузка Core 0/Core 1 и дропы, раз в секунду.    
    static uint32_t t_dbg = 0;    
    if (millis() - t_dbg >= 10000) {    
        t_dbg = millis();    
        Serial.printf("# CORE0_LOAD=%.1f%% CORE1_LOAD=%.1f%% aud_dropped=%lu out_dropped=%lu clip=%lu\n",  
                      core0_load_permille / 10.0,  
                      rx_dsp_load_permille() / 10.0,  
                      (unsigned long)rx_dsp_aud_dropped(),  
                      (unsigned long)rx_dsp_out_dropped(),  
                      (unsigned long)rx_dsp_clip_count());   
    }

}
