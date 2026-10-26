// path: rx_cw.cpp  (ru0aog/RP2040_receiver)  
// ============================================================================  
// rx_cw.cpp — декодер CW (Морзе) по одному тону.  
// Вход: отсчёты базовой полосы 15625 Гц из rx_cw_sample. Тон сносится в DC  
// собственным гетеродином + однополюсный НЧ (как канал rx_mf). Огибающая  
// копится за окно (~5 мс). Детекция ключа — порт FLDigi cw.cxx:  
//   sig_avg / noise_floor / agc_peak (быстрый attack, медленный decay),  
//   нормализация value /= agc_peak, асимметричные пороги гистерезиса,  
//   отсев шумовых всплесков, порог точки/тире two_dots от WPM + трекинг.  
// Границы ключа дают точки/тире -> символы Морзе -> rx_decoders_emit().  
// ============================================================================  
#include <Arduino.h>  
#include <math.h>  
#include "rx_cw.h"  
#include "rx_decoders.h"   // единая точка вывода  
  
// ---- параметры фронта ----  
#define CW_LP_FC_HZ    80.0f     // полоса НЧ детектора тона  
#define CW_WIN_MS      5.0f      // окно огибающей (мс)  
  
// ---- параметры тайминга (в окнах по CW_WIN_MS) ----  
#define CW_DEFAULT_WPM     20.0f     // стартовая скорость; dot = 1200/WPM мс  
// attack/decay AGC в окнах: быстрый подъём, медленный спад  
#define CW_AGC_ATTACK      4.0f      // ~20 мс — агрессивно к пику/провалу  
#define CW_AGC_DECAY       64.0f     // ~320 мс — медленный возврат  
  
static float fs_hz = 15625.0f;  
  
// гетеродин + НЧ (один тон)  
static float osc_re, osc_im, rot_re, rot_im, lp_re, lp_im, lp_alpha;  
  
// накопление окна  
static int   win_div = 78, win_cnt = 0;  
static float win_acc = 0.0f;  
  
// AGC (порт FLDigi decode_stream)  
static float sig_avg, noise_floor, agc_peak;  
  
// тайминг Морзе (в окнах)  
static uint32_t win_idx;          // счётчик окон  
static uint32_t tone_start;       // окно начала тона  
static uint32_t key_up_at;        // окно последнего спада ключа  
static float    dot_win;          // длина точки (окон), адаптивная  
static float    two_dots;         // порог точка/тире (окон)  
static float    spike_win;        // мин. длительность элемента (окон)  
static int      last_elem;        // длительность предыдущего элемента (окон)  
static uint8_t  code;             // сборка: 1 = старт-бит, dot=0, dash=1  
static bool     space_sent;  
  
// состояние приёма (как cw_receive_state в FLDigi)  
enum { RS_IDLE = 0, RS_IN_TONE, RS_AFTER_TONE };  
static int rx_state;  
  
static inline float decayavg(float a, float in, float w) {  
    if (w < 1.0f) w = 1.0f;  
    return a + (in - a) / w;  
}  
  
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
    // знаки препинания (как в FLDigi morse.cxx)  
    case 0b110010:  return '/';  case 0b1110011: return ','; case 0b1001100: return '?';
    case 0b110001:  return '=';  case 0b101010:  return '+'; case 0b1010101: return '.';
    case 0b1100001: return '-';  case 0b1011010: return '@'; case 0b110110:  return '(';
    case 0b1101101: return ')';  case 0b1111000: return ':'; case 0b1101010: return ';';
    case 0b1001101: return '_';  case 0b1011110: return '\''; case 0b1010010: return '\\';
    case 0b1101011: return '!';  case 0b10001001: return '$';
    default: return 0;  
    }  
}  
  
static void emit_char(void) {  
    if (code > 1) {  
        char ch = morse_decode(code);  
        if (ch) rx_decoders_emit(ch);  
    }  
    code = 1;  
}  
  
// Подстройка two_dots по парам точка-тире / тире-точка (FLDigi update_tracking)  
static void update_tracking(int d1, int d2) {  
    if (d1 <= 0 || d2 <= 0) return;  
    if (d1 > d2 && d1 > 4 * d2) return;  
    if (d2 > d1 && d2 > 4 * d1) return;  
    float avg = (float)(d1 + d2) * 0.5f;      // ≈ (dot+dash)/2 ≈ 2*dot  
    two_dots = decayavg(two_dots, avg, 16.0f);  
    dot_win  = two_dots * 0.5f;  
    if (dot_win < 1.0f) dot_win = 1.0f;  
    spike_win = dot_win * 0.5f;               // отсев всплесков < полуточки  
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
  
    // AGC: agc_peak стартует с 1, чтобы деление было корректным сразу  
    sig_avg = 0.0f; noise_floor = 0.0f; agc_peak = 1.0f;  
  
    // тайминг от стартовой WPM: dot(мс)=1200/WPM; в окнах = dot/CW_WIN_MS  
    dot_win   = (1200.0f / CW_DEFAULT_WPM) / CW_WIN_MS;   // ≈ 12 окон @20WPM  
    if (dot_win < 1.0f) dot_win = 1.0f;  
    two_dots  = dot_win * 2.0f;  
    spike_win = dot_win * 0.5f;  
  
    win_idx = 0; tone_start = 0; key_up_at = 0;  
    last_elem = 0; code = 1; space_sent = true;  
    rx_state = RS_IDLE;  
}  
  
static void process_window(float value) {  
    // --- AGC (FLDigi decode_stream) ---  
    sig_avg = decayavg(sig_avg, value, CW_AGC_DECAY);  
  
    if (value < sig_avg) {  
        noise_floor = decayavg(noise_floor, value,  
                     (value < noise_floor) ? CW_AGC_ATTACK : CW_AGC_DECAY);  
    }  
    if (value > sig_avg) {  
        agc_peak = decayavg(agc_peak, value,  
                   (value > agc_peak) ? CW_AGC_ATTACK : CW_AGC_DECAY);  
    }  
    if (agc_peak < 1e-6f) agc_peak = 1e-6f;  
  
    float norm_noise = noise_floor / agc_peak;  
    float norm_sig   = sig_avg     / agc_peak;  
    float v          = value       / agc_peak;  
  
    float diff  = norm_sig - norm_noise;  
    float upper = norm_sig   - 0.2f * diff;   // порог keydown  
    float lower = norm_noise + 0.7f * diff;   // порог keyup (ближе к шуму)  
  
    win_idx++;  
  
    // --- гистерезисный детектор ключа ---  
    if (v > upper && rx_state != RS_IN_TONE) {  
        // keydown  
        if (rx_state == RS_IDLE) { code = 1; }  
        tone_start = win_idx;  
        rx_state   = RS_IN_TONE;  
    } else if (v < lower && rx_state == RS_IN_TONE) {  
        // keyup — измеряем длительность элемента  
        int elem = (int)(win_idx - tone_start);  
        key_up_at = win_idx;  
  
        if ((float)elem < spike_win) {        // шумовой всплеск — игнор  
            rx_state = RS_IDLE;  
            return;  
        }  
        // трекинг скорости по паре элементов  
        if (last_elem > 0) {  
            if (elem > 2 * last_elem && elem < 4 * last_elem)  
                update_tracking(last_elem, elem);  
            if (last_elem > 2 * elem && last_elem < 4 * elem)  
                update_tracking(elem, last_elem);  
        }  
        last_elem = elem;  
  
        bool dash = ((float)elem > two_dots);  
        code = (uint8_t)((code << 1) | (dash ? 1 : 0));  
        space_sent = false;  
        rx_state = RS_AFTER_TONE;  
    }  
  
    // --- тайминг пауз (char gap / word space) ---  
    if (rx_state != RS_IN_TONE) {  
        float gap = (float)(win_idx - key_up_at);  
        if (rx_state == RS_AFTER_TONE && gap >= two_dots) {  
            emit_char();                       // пауза >= 2*dot -> конец буквы  
            rx_state  = RS_IDLE;  
            last_elem = 0;  
        }  
        if (!space_sent && gap > two_dots * 2.0f) {  
            rx_decoders_emit(' ');             // пауза > 4*dot -> слово  
            space_sent = true;  
        }  
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
        // огибающая = модуль (как abs(zp) в FLDigi), sqrtf раз в окно (~200/с)  
        float mag = sqrtf(win_acc / (float)win_div);  
        win_cnt = 0; win_acc = 0.0f;  
        process_window(mag);  
        // ренорм фазора  
        float m = osc_re*osc_re + osc_im*osc_im;  
        float g = 1.5f - 0.5f * m;  
        osc_re *= g; osc_im *= g;  
    }  
}
