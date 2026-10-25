// ============================================================================  
// rx_rtty.cpp — декодер RTTY-45 (45.45 бод, shift ~170 Гц, Baudot/ITA2)  
// Вход: уровни мощности mark/space (pm=|mark|^2, ps=|space|^2) на КАЖДЫЙ  
// отсчёт 15625 Гц из rx_mf_sample. Бит-автомат портирован из FLDigi (rtty.cxx):  
// свободно идущий счётчик по symbollen + КА IDLE→START→DATA→STOP с проверкой  
// стоп-бита. Решение бита: bit = (pm > ps) (mark=1). Присутствие сигнала —  
// по SNR-сквелчу (аналог rtty.cxx Metric).  
// ============================================================================  
  
#include <Arduino.h>  
#include <math.h>  
#include "rx_rtty.h"  
  
// ---------------- параметры RTTY ----------------  
#define RTTY_BAUD       45.45f  
#define RTTY_NBITS      5          // Baudot/ITA2  
#define MF_SQUELCH_DB   6.0f       // порог присутствия: сигнал ≥6 дБ над шумом  
  
// ---------------- Baudot/ITA2 (из FLDigi rtty.cxx) ----------------  
static const char letters[32] = {  
    '\0','E','\n','A',' ','S','I','U',  
    '\r','D','R','J','N','F','C','K',  
    'T','Z','L','W','H','Y','P','Q',  
    'O','B','G',' ','M','X','V',' '  
};  
static const char figures[32] = {  
    '\0','3','\n','-',' ','\a','8','7',  
    '\r','$','4','\'',',','!',':','(',  
    '5','"',')','2','#','6','0','1',  
    '9','?','&',' ','.','/',';',' '  
};  
  
#define RTTY_LTRS  0x1F   // 11111 — сдвиг в регистр букв  
#define RTTY_FIGS  0x1B   // 11011 — сдвиг в регистр цифр  
  
// ---------------- состояние автомата ----------------  
enum { RX_IDLE = 0, RX_START = 1, RX_DATA = 2, RX_STOP = 3 };  
  
static float    fs_hz     = 15625.0f;  
static float    symbollen = 15625.0f / RTTY_BAUD;  // отсчётов на бит (~344)  
  
static int      rxstate   = RX_IDLE;  
static float    counter   = 0.0f;      // отсчётов до следующего строба (float)  
static uint8_t  bitcntr   = 0;  
static uint8_t  rxdata    = 0;  
static bool     figs_mode = false;     // false=LTRS, true=FIGS  
  
// ---------------- SNR-сквелч ----------------  
static float sig_pwr   = 0.0f;  
static float noise_pwr = 1e-6f;  
static bool  present   = false;  
  
static inline float decayavg(float avg, float x, float w) {  
    if (w < 1.0f) return x;  
    return avg + (x - avg) / w;  
}  
  
// ---------------- диагностика (можно убрать) ----------------  
static uint32_t dbg_calls = 0, dbg_emits = 0;  
static uint32_t dbg_t0 = 0;  
  
// ---------------- вывод декодированного символа ----------------  
static void rtty_emit(uint8_t c) {  
    c &= 0x1F;  
    if (c == RTTY_LTRS) { figs_mode = false; return; }  
    if (c == RTTY_FIGS) { figs_mode = true;  return; }  
    char out = figs_mode ? figures[c] : letters[c];  
    if (out) {  
        Serial.print(out);  
        dbg_emits++;  
    }  
}  
  
// ---------------- инициализация ----------------  
void rx_rtty_init(float fs) {  
    fs_hz     = (fs > 1.0f) ? fs : 15625.0f;  
    symbollen = fs_hz / RTTY_BAUD;  
  
    rxstate   = RX_IDLE;  
    counter   = 0.0f;  
    bitcntr   = 0;  
    rxdata    = 0;  
    figs_mode = false;  
  
    sig_pwr   = 0.0f;  
    noise_pwr = 1e-6f;  
    present   = false;  
  
    dbg_calls = dbg_emits = 0;  
    dbg_t0    = millis();  
}  
  
// ---------------- кадр: уровни mark/space (на каждый отсчёт) ----------------  
void rx_rtty_frame_levels(float pm, float ps) {  
    dbg_calls++;  
  
    // --- SNR-сквелч: сильный тон = сигнал, слабый канал = шум ---  
    const float sp = (pm > ps) ? pm : ps;  
    const float np = (pm < ps) ? pm : ps;  
    sig_pwr   = decayavg(sig_pwr,   sp, sp > sig_pwr ? 2.0f : 8.0f);  
    noise_pwr = decayavg(noise_pwr, np, 16.0f);  
    const float snr_db = 10.0f * log10f(sig_pwr / (noise_pwr + 1e-6f));  
    present = (snr_db >= MF_SQUELCH_DB);  
  
    const bool bit = (pm > ps);   // true = mark(1), false = space(0)  
  
    if (!present) {  
        // нет сигнала — держим автомат в IDLE  
        rxstate = RX_IDLE;  
    } else {  
        switch (rxstate) {  
        case RX_IDLE:  
            // старт-бит = space (bit==0), ждём его фронт  
            if (!bit) {  
                rxstate = RX_START;  
                counter = symbollen * 0.5f;   // строб в середину старт-бита  
            }  
            break;  
  
        case RX_START:  
            if ((counter -= 1.0f) <= 0.0f) {  
                if (!bit) {                    // старт-бит подтверждён  
                    rxstate = RX_DATA;  
                    counter = symbollen;  
                    bitcntr = 0;  
                    rxdata  = 0;  
                } else {  
                    rxstate = RX_IDLE;         // ложный старт  
                }  
            }  
            break;  
  
        case RX_DATA:  
            if ((counter -= 1.0f) <= 0.0f) {  
                rxdata |= (bit ? 1 : 0) << bitcntr;  // LSB первым  
                counter = symbollen;  
                if (++bitcntr >= RTTY_NBITS) rxstate = RX_STOP;  
            }  
            break;  
  
        case RX_STOP:  
            if ((counter -= 1.0f) <= 0.0f) {  
                if (bit) {                     // стоп-бит = mark -> кадр валиден  
                    rtty_emit(rxdata & 0x1F);  
                }  
                rxstate = RX_IDLE;  
            }  
            break;  
        }  
    }  
  
    // --- диагностика раз в секунду (временно) ---  
    uint32_t now = millis();  
    if (now - dbg_t0 >= 1000) {  
        Serial.printf("# RTTYDBG calls/s=%lu pm=%.0f ps=%.0f snr=%.1f "  
                      "present=%d state=%d emits/s=%lu\n",  
                      (unsigned long)dbg_calls, (double)pm, (double)ps,  
                      (double)snr_db, present ? 1 : 0, rxstate,  
                      (unsigned long)dbg_emits);  
        dbg_calls = dbg_emits = 0;  
        dbg_t0 = now;  
    }  
}