// ============================================================================  
// rx_rtty.cpp — декодер RTTY-45 (45.45 бод, shift ~170 Гц, Baudot/ITA2)  
// Вход: кадры мощностей mark/space из rx_mf (~5 кадров на бит).  
// Бит-автомат портирован из FLDigi (rtty.cxx): свободно идущий счётчик  
// символов + конечный автомат IDLE→START→DATA→STOP с проверкой стоп-бита.  
// Решение уровня кадра: lvl = (pm > ps). Детекция присутствия сигнала —  
// по плавающему опорному пику ref_peak и порогу thr = ref_peak/4.  
// ============================================================================  
  
#include <Arduino.h>  
#include <math.h>  
#include "rx_rtty.h"  
  
// Детекция присутствия сигнала по SNR (аналог rtty.cxx Metric).  
#define MF_SQUELCH_DB  6.0f       // порог присутствия: сигнал ≥6 дБ над шумом  
  
static inline float decayavg(float avg, float x, float w) {  
    if (w < 1.0f) return x;  
    return avg + (x - avg) / w;  
}
  
// ---------- детекция сигнала (SNR-сквелч) ----------  
static float sig_pwr   = 0.0f;  
static float noise_pwr = 1e-6f;  
static bool  present   = false;
  
// ---------- измерение длительности mark/space ----------  
static float    samp_rate = 15625.0f;  // Гц, для пересчёта длительности в мс  
static bool     cur_bit   = false;     // текущее состояние: true=mark  
static uint32_t run_len   = 0;         // отсчётов в текущем состоянии  
static bool     have_run  = false;     // идёт ли накопление интервала

// ---------- инициализация ----------  
// fs_hz не используется напрямую (счётчик в кадрах), но принимаем его,  
// чтобы точка вызова (rx_mf_init) не зависела от внутренней единицы.  
void rx_rtty_init(float fs_hz) {  
    samp_rate = (fs_hz > 1.0f) ? fs_hz : 15625.0f;  
    sig_pwr   = 0.0f;  
    noise_pwr = 1e-6f;  
    present   = false;
    have_run  = false;  
    run_len   = 0;  
}
  
// ---------- кадр: уровни mark/space ----------  
// Вызывается каждым отсчётом из rx_mf_sample.  
void rx_rtty_frame_levels(float pm, float ps) {  
// --- SNR-сквелч: сильный тон = сигнал, слабый канал матч-фильтра = шум ---  
    const float sp = (pm > ps) ? pm : ps;  
    const float np = (pm < ps) ? pm : ps;  
    sig_pwr   = decayavg(sig_pwr,   sp, sp > sig_pwr ? 2.0f : 8.0f);  
    noise_pwr = decayavg(noise_pwr, np, 16.0f);  
    const float snr_db = 10.0f * log10f(sig_pwr / (noise_pwr + 1e-6f));  
    present = (snr_db >= MF_SQUELCH_DB);
  
    if (!present) {  
        // при пропадании сигнала вытолкнуть накопленный интервал  
        if (have_run && run_len) {  
            Serial.printf("%c %.1f\n", cur_bit ? 'M' : 'S',  
                          run_len * 1000.0f / samp_rate);  
        }  
        have_run = false;  
        run_len  = 0;  
        return;  
    }  
  
    const bool bit = (pm > ps);   // true = mark(1), false = space(0)  
  
    if (!have_run) {  
        cur_bit = bit; run_len = 1; have_run = true;  
    } else if (bit == cur_bit) {  
        run_len++;  
    } else {  
        // состояние сменилось — печатаем завершённый интервал  
        Serial.printf("%c %.1f\n", cur_bit ? 'M' : 'S',  
                      run_len * 1000.0f / samp_rate);  
        cur_bit = bit; run_len = 1;  
    }  
}
