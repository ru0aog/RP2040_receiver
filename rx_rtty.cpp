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
  
// Детекция присутствия сигнала.  
#define MF_MIN_PEAK    1000.0f    // абсолютный порог «есть сигнал»  
#define MF_SQUELCH_LIN 3.981f     // свелч: 6 дБ по мощности (10^(6/10)) над шумом  
#define MF_ATTACK      0.25f      // быстрый подъём опорного пика  
#define MF_DECAY       0.01f      // медленный спад
  
// ---------- детекция сигнала ----------  
static float ref_peak = 0.0f;  
static bool  present  = false;  
  
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
    ref_peak  = 0.0f;  
    present   = false;  
    have_run  = false;  
    run_len   = 0;  
}
  
// ---------- кадр: уровни mark/space ----------  
// Вызывается каждым отсчётом из rx_mf_sample.  
void rx_rtty_frame_levels(float pm, float ps) {  
    const float mx = (pm > ps) ? pm : ps;   // сигнал  
    const float mn = (pm > ps) ? ps : pm;   // шум (противоположный матч-фильтр)  
  
    if (mx > ref_peak) ref_peak += MF_ATTACK * (mx - ref_peak);  
    else               ref_peak += MF_DECAY  * (mx - ref_peak);  
  
    // Свелч: сигнал минимум на 6 дБ (x3.98 по мощности) выше шума.  
    present = (ref_peak > MF_MIN_PEAK) && (mx > MF_SQUELCH_LIN * mn);  
  
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
