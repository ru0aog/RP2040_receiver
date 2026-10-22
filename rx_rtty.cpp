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
  
// ---------- параметры ----------  
#define RTTY_BITS      5          // информационных бит ITA2  
#define SUBS_PER_BIT   5.0f       // кадров на бит (как frame_div в rx_mf)  
  
// Детекция присутствия сигнала по опорному пику.  
#define MF_MIN_PEAK    1000.0f    // абсолютный порог «есть сигнал»  
#define MF_THR_FRAC    0.25f      // порог = ref_peak * этой доли  
#define MF_ATTACK      0.25f      // быстрый подъём опорного пика  
#define MF_DECAY       0.01f      // медленный спад  
  
// Диагностика в Serial (0 = выкл). На Core 1 Serial блокирующий —  
// держите делитель большим и выключайте после отладки.  
#define MF_DIAG        1  
#define MF_DIAG_DIV    256  
  
// ---------- состояние автомата ----------  
enum rx_state_t { RX_IDLE, RX_START, RX_DATA, RX_STOP };  
static rx_state_t rxstate   = RX_IDLE;  
static float      symbollen = SUBS_PER_BIT;   // длина бита в кадрах  
static float      counter   = 0.0f;           // счётчик до следующей точки решения  
static int        bitcntr   = 0;              // индекс принятого бита (0..RTTY_BITS-1)  
static uint8_t    rxdata    = 0;              // накопленные биты (LSB-first)  
  
// ---------- детекция сигнала ----------  
static float ref_peak = 0.0f;  
static bool  present  = false;  
  
// ---------- сборка символа / сдвиг ----------  
static uint8_t shift_ltrs = 1;   // 1 = LTRS, 0 = FIGS  
  
// Бодо по ITA2. Индекс 27 = FIGS-сдвиг, 31 = LTRS-сдвиг (обрабатываются отдельно).  
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
    if (code == 27) { shift_ltrs = 0; return; }   // FIGS  
    if (code == 31) { shift_ltrs = 1; return; }   // LTRS  
    char c = shift_ltrs ? bodo_ltrs[code] : bodo_figs[code];  
    if (c && c != 27 && c != 31) Serial.write(c);  
}  
  
// ---------- инициализация ----------  
// fs_hz не используется напрямую (счётчик в кадрах), но принимаем его,  
// чтобы точка вызова (rx_mf_init) не зависела от внутренней единицы.  
void rx_rtty_init(float fs_hz) {  
    (void)fs_hz;  
    rxstate   = RX_IDLE;  
    symbollen = SUBS_PER_BIT;  
    counter   = 0.0f;  
    bitcntr   = 0;  
    rxdata    = 0;  
    ref_peak  = 0.0f;  
    present   = false;  
    shift_ltrs = 1;  
}  
  
// ---------- кадр: уровни mark/space ----------  
// Вызывается один раз на кадр из rx_mf_sample (frame_div отсчётов на кадр).  
void rx_rtty_frame_levels(float pm, float ps) {  
    // --- опорный пик и порог присутствия ---  
    const float mx = (pm > ps) ? pm : ps;  
    if (mx > ref_peak) ref_peak += MF_ATTACK * (mx - ref_peak);  
    else               ref_peak += MF_DECAY  * (mx - ref_peak);  
  
    const float thr = ref_peak * MF_THR_FRAC;  
    present = (ref_peak > MF_MIN_PEAK) && (mx > thr);  
  
    const bool bit = (pm > ps);   // true = mark(1), false = space(0)  
  
#if MF_DIAG  
    static uint32_t dbg = 0;  
    if (++dbg >= MF_DIAG_DIV) {  
        dbg = 0;  
        Serial.printf("#MF pm=%ld ps=%ld peak=%ld thr=%ld pres=%d lvl=%d\n",  
                      (long)pm, (long)ps, (long)ref_peak, (long)thr,  
                      present, bit);  
    }  
#endif  
  
    // Нет сигнала -> держим автомат в покое.  
    if (!present) {  
        rxstate = RX_IDLE;  
        bitcntr = 0;  
        rxdata  = 0;  
        return;  
    }  
  
    // --- бит-автомат (портирован из FLDigi rtty.cxx) ---  
    switch (rxstate) {  
  
    case RX_IDLE:  
        // ждём фронт mark->space (начало старт-бита)  
        if (!bit) {  
            rxstate = RX_START;  
            counter = symbollen * 0.5f;   // идём к СЕРЕДИНЕ старт-бита  
        }  
        break;  
  
    case RX_START:  
        if ((counter -= 1.0f) <= 0.0f) {  
            if (!bit) {                   // в центре всё ещё space -> валидный старт  
                rxstate = RX_DATA;  
                counter = symbollen;  
                bitcntr = 0;  
                rxdata  = 0;  
            } else {  
                rxstate = RX_IDLE;        // ложный старт (шумовой фронт)  
            }  
        }  
        break;  
  
    case RX_DATA:  
        if ((counter -= 1.0f) <= 0.0f) {  
            rxdata |= (bit ? 1 : 0) << bitcntr;   // LSB-first  
            bitcntr++;  
            counter = symbollen;  
            if (bitcntr >= RTTY_BITS) rxstate = RX_STOP;  
        }  
        break;  
  
    case RX_STOP:  
        if ((counter -= 1.0f) <= 0.0f) {  
            if (bit) rtty_emit(rxdata);   // стоп = mark -> символ валиден  
            rxstate = RX_IDLE;  
        }  
        break;  
    }  
}
