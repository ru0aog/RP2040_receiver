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
#include "rx_rsid.h"
#include "rx_decoders.h" 
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
    uint32_t lo_hz = vfo_quadrature_lo_hz();  
    // Куда попадёт дырка (f_nco), если сделать быстрый сдвиг NCO без ретюна LO.  
    long     f_nco = (long)freq_hz - (long)lo_hz;  
  
    // Быстрый путь РАЗРЕШЁН только пока дырка строго в сладкой зоне  
    // [RX_NCO_SWEET_LO..RX_NCO_SWEET_HI] = [9600..14500]. Это гарантирует,  
    // что при подстройке вниз мелкими шагами f_nco НЕ сползёт к 5 кГц и ниже  
    // (дырка не войдёт в звуковую полосу). Как только f_nco вышел за зону —  
    // полная переустановка LO вернёт f_nco к центру RX_IF_SHIFT_HZ (11000).  
    if (f_nco >= (long)RX_NCO_SWEET_LO && f_nco <= (long)RX_NCO_SWEET_HI) {  
        rx_dsp_set_nco_hz((float)f_nco);  
        current_rx_hz = freq_hz;  
        Serial.printf("# RX_NCO,target_hz=%lu,lo_hz=%lu,nco_hz=%ld\n",  
                      (unsigned long)freq_hz, (unsigned long)lo_hz,  
                      (long)f_nco);  
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

    // Ожидание подключения Serial — оставить существующий код.

    // Сначала полностью инициализируем RSID.
    rsid_rx_init(RSID_RX_FS_HZ);

    // Затем запускаем ADC и DSP на ядре 1.
    startReceiver();

    // Диспетчер декодеров инициализируем после запуска DSP,
    // поскольку ему нужна фактическая частота выходного потока.
    rx_decoders_init(rx_dsp_fs_out());

    fldigi_cat_init();
}
  
// RP2040_receiver.ino  
void loop() {

// ---- Учёт загрузки прикладных задач Core 0 ----

static uint32_t core0_busy_us = 0;
static uint32_t core0_window_start_us = 0;
static uint32_t core0_load_permille = 0;

static bool core0_timer_initialized = false;

    uint32_t now = time_us_32();

    if (!core0_timer_initialized) {
        core0_window_start_us = now;
        core0_timer_initialized = true;
    }

    // Начало измеряемой работы.
    const uint32_t busy_start = now;

    rx_audio_poll();

    rsid_rx_dispatch();

    // CAT опрашивается примерно раз в 15 мс.
    static uint32_t t_cat = 0;

    if (millis() - t_cat >= 15) {
        t_cat = millis();
        fldigi_cat_poll();
    }

    // Завершение измеряемой работы.
    now = time_us_32();

    core0_busy_us += now - busy_start;

    // Расчёт за окно реального времени.
    const uint32_t window_us = now - core0_window_start_us;

    if (window_us >= 1000000u) {
        uint32_t busy = core0_busy_us;

        if (busy > window_us) {
            busy = window_us;
        }

        core0_load_permille =
            (uint32_t)(((uint64_t)busy * 1000u) / window_us);

        core0_busy_us = 0;
        core0_window_start_us = now;
    }

 
    // Диагностика: загрузка Core 0/Core 1 и дропы, раз в секунду.    
static uint32_t t_dbg = 0;


if (millis() - t_dbg >= 1000000) {
    t_dbg = millis();

    static uint32_t rsid_prev_calls = 0;
    static uint32_t rsid_prev_callback_count = 0;
    static uint32_t rsid_prev_report_us = 0;

    const uint32_t now_us = micros();

    // Длительность прошедшего интервала измерения.
    uint32_t elapsed_us = 0;

    if (rsid_prev_report_us != 0) {
        elapsed_us = now_us - rsid_prev_report_us;
    }

    // RSID service: общее количество вызовов.
    const uint32_t calls_total = rsid_rx_service_calls();

    // RSID callback: общее количество вызовов.
    const uint32_t callback_count = rsid_callback_count;
    const uint32_t callback_max_us = rsid_callback_max_us;

    uint32_t rsid_calls_s = 0;
    uint32_t callback_calls_s = 0;

    if (elapsed_us > 0) {
        const uint32_t calls_delta =
            calls_total - rsid_prev_calls;

        const uint32_t callback_delta =
            callback_count - rsid_prev_callback_count;

        rsid_calls_s = (uint32_t)(
            ((uint64_t)calls_delta * 1000000ULL) / elapsed_us
        );

        callback_calls_s = (uint32_t)(
            ((uint64_t)callback_delta * 1000000ULL) / elapsed_us
        );
    }

    // Обновляем предыдущие значения только после расчёта обеих частот.
    rsid_prev_calls = calls_total;
    rsid_prev_callback_count = callback_count;
    rsid_prev_report_us = now_us;

    const uint32_t rsid_max_us = rsid_rx_service_max_us();

    // Снимаем значения загрузки один раз.
    const uint32_t core1_load = rx_dsp_load_permille();
    const uint32_t core1_dsp  = rx_dsp_compute_load_permille();
    const uint32_t core1_rsid = rx_dsp_rsid_load_permille();

    Serial.printf(
        "# CORE0_TASK_LOAD=%lu.%lu%% "
        "CORE1_LOAD=%lu.%lu%% "
        "CORE1_DSP=%lu.%lu%% "
        "CORE1_RSID=%lu.%lu%% "
        "aud_dropped=%lu "
        "out_dropped=%lu "
        "clip=%lu "
        "rsid_drop=%lu "
        "rsid_sample_drop=%lu "
        "rsid_window_drop=%lu "
        "rsid_q=%lu/1024 "
        "rsid_sm=%lu "
        "rsid_max_us=%lu "
        "rsid_calls_s=%lu "
        "rsid_callback_count=%lu "
        "rsid_callback_calls_s=%lu "
        "rsid_callback_max_us=%lu\n",

        // Core 0
        (unsigned long)(core0_load_permille / 10u),
        (unsigned long)(core0_load_permille % 10u),

        // Core 1: общая загрузка
        (unsigned long)(core1_load / 10u),
        (unsigned long)(core1_load % 10u),

        // Core 1: DSP
        (unsigned long)(core1_dsp / 10u),
        (unsigned long)(core1_dsp % 10u),

        // Core 1: RSID
        (unsigned long)(core1_rsid / 10u),
        (unsigned long)(core1_rsid % 10u),

        // Счётчики аудио и DSP
        (unsigned long)rx_dsp_aud_dropped(),
        (unsigned long)rx_dsp_out_dropped(),
        (unsigned long)rx_dsp_clip_count(),

        // Потери RSID
        (unsigned long)rsid_rx_dropped(),
        (unsigned long)rsid_rx_sample_dropped(),
        (unsigned long)rsid_rx_window_dropped(),

        // Очередь и состояние автомата RSID
        (unsigned long)rsid_rx_queue_fill(),
        (unsigned long)rsid_rx_sm_state(),

        // Диагностика RSID service
        (unsigned long)rsid_max_us,
        (unsigned long)rsid_calls_s,

        // Диагностика callback
        (unsigned long)callback_count,
        (unsigned long)callback_calls_s,
        (unsigned long)callback_max_us
    );
}





}
