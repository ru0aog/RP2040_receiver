// ============================================================================  
// rx_rtty.cpp — декодер RTTY-45 (45.45 бод, shift ~170 Гц, Baudot)  
// Вход: кадры мощностей банка rx_tones (кадр 9 отсчётов = 4.6 мс ≈ 1/5 бита).  
// Бодовая синхронизация (PLL): бит = SUBS кадров, решение — majority vote  
// по кадрам в центре бита; фронт mark/space сдвигает фазу.  
// Автопоиск пары тонов: разнос 140..200 Гц, оба >= медиана + 10 дБ.  
// ============================================================================  
  
#include <Arduino.h>  
#include <math.h>  
#include "rx_tones.h"  
#include "rx_rtty.h"  
  
// ---------- параметры ----------  
#define LOCK_SNR_DB   10.0f   // оба тона должны быть выше медианы  
#define SHIFT_MIN_HZ  140.0f  
#define SHIFT_MAX_HZ  200.0f  
  
// Бодовая сетка: бит 45.45 бод = 22.02 мс; кадр = 9 отсчётов @1953.125 Гц  
// 5 кадров = 45 отсчётов = 23.04 мс (ошибка ~4.6%/бит гасится PLL по фронтам).  
#define SUBS        5     // кадров на бит  
#define CENTER_LO   2     // majority vote по кадрам 2..3 (центр бита)  
#define CENTER_HI   3  
  
// watchdog потери сигнала: оба тона < med*4 непрерывно ~1 с -> разлочка  
#define LOST_FRAMES 220  
  
// mark/space-каналы: -1 = не залочены  
static int ch_mark  = -1;  
static int ch_space = -1;  
static int lost_cnt = 0;  
  
// ---------- бодовый автомат ----------  
static int  bit_phase  = 0;    // кадр внутри текущего бита (0..SUBS-1)  
static int  mid_votes  = 0;    // mark-голоса в центре бита  
static int  mid_count  = 0;  
static bool cur_bit    = true; // оценка текущего бита (по фронту)  
static bool prev_level = true; // уровень предыдущего кадра (для поиска фронтов)  
  
// ---------- сборка символа ----------  
static uint8_t bit_frame  = 0;  // принятые биты (LSB-first)  
static uint8_t bit_cnt    = 0;  // 0 = ждём старт; 1..5 = данные; >=6 = стоп  
static uint8_t shift_ltrs = 1;  // 1 = LTRS, 0 = FIGS  
  
// Бодо по ITA2. Индекс 27 = FIGS-сдвиг, 31 = LTRS-сдвиг (обрабатываются отдельно).  
static const char bodo_ltrs[32] = {  
    '\0','E','\n','A',' ','S','I','U','\r','D','R','J','N','F','C','K',  
    'T','Z','L','W','H','Y','P','Q','O','B','G', 27,'M','X','V', 31  
};  
static const char bodo_figs[32] = {  
    '\0','3','\n','-',' ','\'','8','7','\r','$','4','\a',',','!',':','(',  
    '5','"',')','2','#','6','0','1','9','?','&', 27,'.','/',';', 31  
};  
  
static void rtty_reset_char(void) { bit_frame = 0; bit_cnt = 0; }  
static void rtty_reset_pll(void)  { bit_phase = 0; mid_votes = 0; mid_count = 0; }  
  
static void rtty_emit(uint8_t code) {  
    code &= 31;  
    if (code == 27) { shift_ltrs = 0; return; }   // FIGS  
    if (code == 31) { shift_ltrs = 1; return; }   // LTRS  
    char c = shift_ltrs ? bodo_ltrs[code] : bodo_figs[code];  
    if (c && c != 27 && c != 31) Serial.write(c);  
}  
  
// вызывается один раз на закрытый бит (bit: true = mark, false = space)  
static void rtty_bit(bool bit) {  
    if (bit_cnt == 0) {                      // ждём старт-бит (space)  
        if (!bit) { bit_frame = 0; bit_cnt = 1; }  
        return;  
    }  
    if (bit_cnt <= 5) {                      // 5 информационных бит, LSB first  
        bit_frame |= (bit ? 1 : 0) << (bit_cnt - 1);  
        bit_cnt++;  
        return;  
    }  
    // стоп-бит: mark закрывает символ; длина не проверяется  
    // (поддерживаются стопы 1 и 1.5 бита; дидл в mark просто ждёт space-фронта)  
    if (bit) {  
        rtty_emit(bit_frame);  
        rtty_reset_char();  
    } else {  
        // сломанный стоп — ресинхронизация, ждём новый space-фронт  
        rtty_reset_char();  
        rtty_reset_pll();  
    }  
}  
  
// ---------- автопоиск пары mark/space ----------  
// Два канала с P >= медиана*LOCK_SNR, разнос 140..200 Гц.  
// Нижний = space, верхний = mark (стандарт: mark выше по частоте).  
static bool rtty_acquire(void) {  
    float med  = rx_tones_median();  
    float need = med * powf(10.0f, LOCK_SNR_DB / 10.0f);  
  
    int n = rx_tones_nch();  
    // разнос в каналах — по фактическому шагу сетки (не привязан к Fs=1953)  
    float step_hz = rx_tones_freq_hz(1) - rx_tones_freq_hz(0);  
    int d_lo = (int)(SHIFT_MIN_HZ / step_hz);  
    int d_hi = (int)(SHIFT_MAX_HZ / step_hz) + 1;  
    if (d_lo < 1) d_lo = 1;  
  
    int   best_lo = -1, best_hi = -1;  
    float best_p  = 0.0f;  
  
    for (int lo = 25; lo < n - 25 - d_lo; lo++) {  
        if (rx_tones_power(lo) < need) continue;  
        int hi_max = lo + d_hi;  
        if (hi_max > n - 25) hi_max = n - 25;  
        for (int hi = lo + d_lo; hi < hi_max; hi++) {  
            if (rx_tones_power(hi) < need) continue;  
            float pp = rx_tones_power(lo) + rx_tones_power(hi);  
            if (pp > best_p) { best_p = pp; best_lo = lo; best_hi = hi; }  
        }  
    }  
    if (best_lo < 0) return false;  
  
    ch_space = best_lo;  
    ch_mark  = best_hi;  
    Serial.printf("#RTTY_LOCK,space=%.0f,mark=%.0f,delta=%.0f\n",  
                  (double)rx_tones_freq_hz(ch_space),  
                  (double)rx_tones_freq_hz(ch_mark),  
                  (double)(rx_tones_freq_hz(ch_mark) - rx_tones_freq_hz(ch_space)));  
    rtty_reset_char();  
    rtty_reset_pll();  
    shift_ltrs = 1;  
    lost_cnt = 0;  
    prev_level = true;  
    return true;  
}  
  
// ---------- кадр: уровень + бодовый PLL ----------  
void rx_rtty_frame(void) {  
    if (ch_mark < 0) { rtty_acquire(); return; }  
  
    float pm  = rx_tones_power(ch_mark);  
    float ps  = rx_tones_power(ch_space);  
    float med = rx_tones_median();  
  
    // watchdog: оба тона пропали ~1 с -> разлочка  
    if (pm < med * 4.0f && ps < med * 4.0f) {  
        if (++lost_cnt >= LOST_FRAMES) {  
            ch_mark = ch_space = -1;  
            lost_cnt = 0;  
            rtty_reset_char();  
            rtty_reset_pll();  
            Serial.println("#RTTY_LOST");  
        }  
        return;  
    }  
    lost_cnt = 0;  
  
    bool lvl = (pm > ps);                    // уровень этого кадра  
  
    // фронт -> сдвигаем фазу к фронту (PLL)  
    if (lvl != prev_level) {  
        // если бит был почти закрыт (уже есть голоса в центре) —  
        // сначала закрываем его, иначе бит потеряется  
        if (mid_count > 0) {  
            rtty_bit(mid_votes * 2 > mid_count);  
        }  
        bit_phase = 0; mid_votes = 0; mid_count = 0;  
        cur_bit = lvl;  
    }  
    prev_level = lvl; 
  
    // центр бита — majority vote  
    if (bit_phase >= CENTER_LO && bit_phase <= CENTER_HI) {  
        mid_votes += lvl ? 1 : 0;  
        mid_count++;  
    }  
  
    if (++bit_phase >= SUBS) {  
        // бит закрыт: решаем по голосам в центре, иначе по значению на фронте  
        bool bit = (mid_count > 0) ? (mid_votes * 2 > mid_count) : cur_bit;  
        rtty_bit(bit);  
        bit_phase = 0; mid_votes = 0; mid_count = 0;  
        cur_bit = lvl;  
    }  
}
