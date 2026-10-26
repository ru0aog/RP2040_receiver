// ============================================================================  
// rx_cw.cpp — декодер CW (Морзе) по одному тону.  
// Вход: отсчёты базовой полосы 15625 Гц из rx_cw_sample. Тон сносится в DC  
// собственным гетеродином + однополюсный НЧ (как канал rx_mf). Огибающая  
// копится за окно (~5 мс), затем AGC + гистерезис + дебаунс (порт из beacon).  
// Границы ключа дают точки/тире -> символы Морзе -> rx_decoders_emit().  
// ============================================================================  
#include <Arduino.h>  
#include <math.h>  
#include "rx_cw.h"  
#include "rx_decoders.h"   // единая точка вывода  
  
// ---- параметры ----    
#define CW_LP_FC_HZ    80.0f     // полоса НЧ детектора тона    
#define CW_WIN_MS      5.0f      // окно огибающей    
#define CW_EMA_SHIFT   3         // EMA огибающей ~8 окон    
  
// Стартовая длина точки (в окнах): 5 мс/окно, ~60 мс точка ≈ 20 WPM.    
// Предзасев, чтобы первый элемент не задавал dot_win сам по себе.    
#define CW_DEFAULT_DOT_WIN  12.0f    
// Число окон прогрева AGC: быстрое схождение sig/noise в начале посылки.    
#define CW_WARMUP_WINDOWS   16
  
static float fs_hz = 15625.0f;  
  
// гетеродин + НЧ (один тон)  
static float osc_re, osc_im, rot_re, rot_im, lp_re, lp_im, lp_alpha;  
  
// накопление окна  
static int   win_div = 78, win_cnt = 0;  
static float win_acc = 0.0f;  
  
// огибающая/AGC (float-версия beacon)  
static float p_ema = 0.0f, sig_avg = 0.0f, noise_avg = 0.0f;  
static bool  tone_on = false;  
static int   low_cnt = 0;
static int   warmup  = 0;   // счётчик окон прогрева AGC
  
// тайминг Морзе (в окнах)  
static uint32_t win_idx = 0, on_start = 0, off_start = 0;  
static float    dot_win = 0.0f;   // адаптивная длина точки (в окнах)  
static uint8_t  code = 1;         // сборка: 1 = старт-бит, dot=0, dash=1  
static bool     in_char = false;  
  
// ---- таблица Морзе: индекс = собранный код (старт-бит слева) ----  
static char morse_decode(uint8_t c) {  
    switch (c) {  
    case 0b101: return 'A'; case 0b11000: return 'B'; case 0b11010: return 'C';  
    case 0b1100: return 'D'; case 0b10: return 'E'; case 0b10010: return 'F';  
    case 0b1110: return 'G'; case 0b10000: return 'H'; case 0b100: return 'I';  
    case 0b10111: return 'J'; case 0b1101: return 'K'; case 0b10100: return 'L';  
    case 0b111: return 'M'; case 0b110: return 'N'; case 0b1111: return 'O';  
    case 0b10110: return 'P'; case 0b11101: return 'Q'; case 0b1010: return 'R';  
    case 0b1000: return 'S'; case 0b11: return 'T'; case 0b1001: return 'U';  
    case 0b10001: return 'V'; case 0b1011: return 'W'; case 0b11001: return 'X';  
    case 0b11011: return 'Y'; case 0b11100: return 'Z';  
    case 0b111111: return '0'; case 0b101111: return '1'; case 0b100111: return '2';  
    case 0b100011: return '3'; case 0b100001: return '4'; case 0b100000: return '5';  
    case 0b110000: return '6'; case 0b111000: return '7'; case 0b111100: return '8';  
    case 0b111110: return '9';  
    default: return 0;  
    }  
}  
  
static void emit_char(void) {  
    if (code > 1) {  
        char ch = morse_decode(code);  
        if (ch) rx_decoders_emit(ch);  
    }  
    code = 1;  
    in_char = false;  
}  
  
void rx_cw_init(float fs, float tone_hz) {  
    fs_hz = (fs > 1.0f) ? fs : 15625.0f;  
    float w = 2.0f * (float)M_PI * tone_hz / fs_hz;  
    rot_re =  cosf(w);  
    rot_im = -sinf(w);           // сносит +tone_hz в DC  
    osc_re = 1.0f; osc_im = 0.0f;  
    lp_re = lp_im = 0.0f;  
    lp_alpha = 1.0f - expf(-2.0f * (float)M_PI * CW_LP_FC_HZ / fs_hz);  
  
    win_div = (int)(fs_hz * CW_WIN_MS / 1000.0f + 0.5f);  
    if (win_div < 1) win_div = 1;  
    win_cnt = 0; win_acc = 0.0f;  
  
    p_ema = sig_avg = noise_avg = 0.0f;    
    tone_on = false; low_cnt = 0;    
    win_idx = on_start = off_start = 0;    
    dot_win = CW_DEFAULT_DOT_WIN;   // стартовая длина точки, адаптируется дальше    
    code = 1; in_char = false;    
    warmup = CW_WARMUP_WINDOWS;     // окна быстрого схода AGC
}  
  
static void process_window(float pe) {    
    // Прогрев: пока AGC не набрала разброс, быстро тянем sig вверх к пикам,    
    // а noise — вниз к провалам, чтобы пороги стали валидны за первые окна.    
    if (warmup > 0) {    
        warmup--;    
        if (pe > sig_avg)   sig_avg   = pe;              // мгновенно к пику    
        else                sig_avg  -= sig_avg * (1.0f/64.0f);    
        if (pe < noise_avg || noise_avg == 0.0f) noise_avg = pe;  // мгновенно к провалу    
        else                noise_avg += (pe - noise_avg) * (1.0f/16.0f);    
    } else {    
        // двухуровневая AGC + гистерезис (порт beacon)    
        if (pe > noise_avg) noise_avg += (pe - noise_avg) * (1.0f/256.0f);    
        else                noise_avg += (pe - noise_avg) * (1.0f/8.0f);    
        if (tone_on && pe > sig_avg) sig_avg += (pe - sig_avg) * 0.5f;    
        else                         sig_avg -= sig_avg * (1.0f/256.0f);    
        if (!tone_on)                noise_avg += (pe - noise_avg) * (1.0f/16.0f);    
    }    
  
    float range = sig_avg - noise_avg; if (range < 1.0f) range = 1.0f;    
    float thr_hi = noise_avg + range * 0.6f;    
    float thr_lo = noise_avg + range * 0.4f; 
  
    bool key;  
    if      (pe > thr_hi) key = true;  
    else if (pe < thr_lo) key = false;  
    else                  key = tone_on;  
  
    bool key_deb;  
    if (key) { low_cnt = 0; key_deb = true; }  
    else { if (low_cnt < 3) { low_cnt++; key_deb = tone_on; } else key_deb = false; }  
  
    win_idx++;  
  
    if (key_deb != tone_on) {  
        if (key_deb) {  
            // старт тона: оценить длину предыдущей паузы  
            uint32_t off = win_idx - off_start;  
            if (in_char && dot_win > 0.0f) {  
                if (off >= dot_win * 2.0f) emit_char();        // пауза >= 3 эл. -> конец буквы  
                if (off >= dot_win * 5.0f) rx_decoders_emit(' ');  // пауза слова  
            }  
            on_start = win_idx;  
        } else {  
            // конец тона: длительность -> точка/тире  
            uint32_t on = win_idx - on_start;  
            if (dot_win <= 0.0f) dot_win = (float)on;           // инициализация  
            bool dash = ((float)on > dot_win * 2.0f);  
            // адаптация длины точки по короткому элементу  
            if (!dash) dot_win += ((float)on - dot_win) * 0.2f;  
            code = (uint8_t)((code << 1) | (dash ? 1 : 0));  
            in_char = true;  
            off_start = win_idx;  
        }  
        tone_on = key_deb;  
    }  
}  
  
void rx_cw_sample(int16_t x) {  
    float xi = (float)x;  
    // снос в DC + НЧ (один тон, как канал rx_mf)  
    float yr = xi * osc_re;  
    float yi = xi * osc_im;  
    float nr = osc_re * rot_re - osc_im * rot_im;  
    float ni = osc_re * rot_im + osc_im * rot_re;  
    osc_re = nr; osc_im = ni;  
    lp_re += lp_alpha * (yr - lp_re);  
    lp_im += lp_alpha * (yi - lp_im);  
    win_acc += lp_re * lp_re + lp_im * lp_im;  
  
    if (++win_cnt >= win_div) {  
        float p = win_acc / (float)win_div;  
        win_cnt = 0; win_acc = 0.0f;  
        // EMA огибающей ~8 окон  
        p_ema += (p - p_ema) * (1.0f / (1 << CW_EMA_SHIFT));  
        process_window(p_ema);  
        // ренорм фазора  
        float m = osc_re*osc_re + osc_im*osc_im;  
        float g = 1.5f - 0.5f * m;  
        osc_re *= g; osc_im *= g;  
    }  
}
