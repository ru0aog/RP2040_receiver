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
  
// ---------- бодовый автомат RTTY-45 ----------    
#define RTTY_BAUD   45.45f    
enum { RX_IDLE = 0, RX_START, RX_DATA, RX_STOP };    
  
static float    samp_rate = 15625.0f;  // Гц    
static float    bit_len   = 343.78f;   // отсчётов на бит = fs/baud    
static int      state     = RX_IDLE;    
static float    counter   = 0.0f;      // отсчёт до середины текущего бита    
static int      bit_idx   = 0;    
static uint8_t  data_bits = 0;    
static bool     shift_ltrs = true;     // true=LTRS, false=FIGS    
  
// Baudot/ITA2. Индекс 27 = FIGS-сдвиг, 31 = LTRS-сдвиг.    
static const char bodo_ltrs[32] = {    
    '\0','E','\n','A',' ','S','I','U','\r','D','R','J','N','F','C','K',    
    'T','Z','L','W','H','Y','P','Q','O','B','G', 27,'M','X','V', 31    
};    
static const char bodo_figs[32] = {    
    '\0','3','\n','-',' ','\'','8','7','\r','$','4','\a',',','!',':','(',    
    '5','"',')','2','#','6','0','1','9','?','&', 27,'.','/',';', 31    
};    
  
static void rtty_emit(uint8_t code) {    
    code &= 31;    
    if (code == 27) { shift_ltrs = false; return; }   // FIGS    
    if (code == 31) { shift_ltrs = true;  return; }   // LTRS    
    char c = shift_ltrs ? bodo_ltrs[code] : bodo_figs[code];  
    if (c && c != 27 && c != 31) { Serial.write(c); extern uint32_t rtty_dbg_emit_hook(void); }
}    
  
// ---------- инициализация ----------    
void rx_rtty_init(float fs_hz) {    
    samp_rate  = (fs_hz > 1.0f) ? fs_hz : 15625.0f;    
    bit_len    = samp_rate / RTTY_BAUD;    
    sig_pwr    = 0.0f;    
    noise_pwr  = 1e-6f;    
    present    = false;    
    state      = RX_IDLE;    
    counter    = 0.0f;    
    bit_idx    = 0;    
    data_bits  = 0;    
    shift_ltrs = true;    
}    
  
// ---------- кадр: уровни mark/space ----------    
void rx_rtty_frame_levels(float pm, float ps) {    
    const float sp = (pm > ps) ? pm : ps;    
    const float np = (pm < ps) ? pm : ps;    
    sig_pwr   = decayavg(sig_pwr,   sp, sp > sig_pwr ? 2.0f : 8.0f);    
    noise_pwr = decayavg(noise_pwr, np, 16.0f);    
    const float snr_db = 10.0f * log10f(sig_pwr / (noise_pwr + 1e-6f));    
    present = (snr_db >= MF_SQUELCH_DB);    

    // --- ДИАГНОСТИКА: раз в ~1 с печатаем, что видит автомат ---  
    static uint32_t dbg_calls = 0;   // сколько отсчётов пришло за секунду  
    static uint32_t dbg_t     = 0;  
    static uint32_t dbg_emit  = 0;   // сколько символов декодировано  
    ++dbg_calls;  
    if (millis() - dbg_t >= 1000) {  
        dbg_t = millis();  
        Serial.printf("# RTTYDBG calls/s=%lu pm=%.0f ps=%.0f snr=%.1f present=%d state=%d emits/s=%lu\n",  
                      (unsigned long)dbg_calls, pm, ps, snr_db,  
                      (int)present, state, (unsigned long)dbg_emit);  
        dbg_calls = 0;  
        dbg_emit  = 0;  
    }  

    if (!present) { state = RX_IDLE; return; }   // нет сигнала — ждём старт-бит    
  
    const int bit = (pm > ps) ? 1 : 0;   // 1=mark(idle/stop), 0=space(start)    
  
    switch (state) {    
    case RX_IDLE:    
        if (bit == 0) {                  // фронт в space = возможный старт-бит    
            state   = RX_START;    
            counter = bit_len * 0.5f;    // сэмплируем середину старт-бита    
        }    
        break;    
  
    case RX_START:    
        if ((counter -= 1.0f) <= 0.0f) {    
            if (bit == 0) {              // старт-бит подтверждён    
                state     = RX_DATA;    
                bit_idx   = 0;    
                data_bits = 0;    
                counter  += bit_len;    
            } else {    
                state = RX_IDLE;         // ложный старт    
            }    
        }    
        break;    
  
    case RX_DATA:    
        if ((counter -= 1.0f) <= 0.0f) {    
            data_bits |= (uint8_t)(bit << bit_idx);   // LSB первый    
            if (++bit_idx >= 5) state = RX_STOP;    
            counter += bit_len;    
        }    
        break;    
  
    case RX_STOP:    
        if ((counter -= 1.0f) <= 0.0f) {    
            if (bit == 1) rtty_emit(data_bits);       // валиден только со стоп-битом mark    
            state = RX_IDLE;    
        }    
        break;    
    }    
}
