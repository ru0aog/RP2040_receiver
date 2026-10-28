#include <Arduino.h>  
#include <math.h>  
#include <string.h>  
  
#include "../vfo/vfo_rp2040.h"  
#include "../audio/rx_audio.h"
#include "../modes/rx_mf.h"
#include "../rsid/rx_rsid.h"
#include "../modes/rx_decoders.h"
#include "AudioTools.h"  
#include "AudioTools/Communication/USB/USBAudioStream.h"  
 
 
// ---- Демодулятор ----  
#define DEMOD_USB  0  
#define DEMOD_LSB  1
#define DEMOD_AM   2    
#define DEMOD_FM   3
  
// Фиксированное усиление сырых I/Q перед упаковкой в PCM.  
#define RAW_IQ_GAIN 16  
  
// Прямое объявление: вызов в get_demod_input() идёт раньше определения ниже.  
// Спецификатор inline обязан совпадать с определением.  
static inline void rsid_rs_push(int32_t x);

// ---- Runtime-состояние режима вывода ----  
// Стартовый режим — сырые I/Q (как раньше при RAW_IQ_OUT=1).  
static volatile uint8_t rx_mode     = RX_MODE_SSB;   // старт сразу в SSB-моно  // rx_mode     = RX_MODE_RAW_IQ;  
static volatile uint8_t demod_mode  = DEMOD_USB;   // боковая для RX_MODE_SSB  
static volatile bool    mode_changed = false;      // запрос на сброс ресемплера

// хелпер: перевести (rx_mode, demod_mode) в режим Core 1  
static void push_demod_to_core1(void) {  
    if (rx_mode == RX_MODE_SSB) {  
        switch (demod_mode) {    
            case DEMOD_LSB: rx_dsp_set_demod(RXD_DEMOD_LSB); break;    
            case DEMOD_AM:  rx_dsp_set_demod(RXD_DEMOD_AM);  break;    
            case DEMOD_FM:  rx_dsp_set_demod(RXD_DEMOD_FM);  break;    
            default:        rx_dsp_set_demod(RXD_DEMOD_USB); break;    
        }    
    } else {  
        rx_dsp_set_demod(RXD_DEMOD_RAW);   // сырой I/Q и тест  
    }  
}  
  
void rx_audio_set_mode(uint8_t mode) {  
    if (mode > RX_MODE_TEST) return;  
    if (mode != rx_mode) { rx_mode = mode; mode_changed = true; }  
    push_demod_to_core1();  
}  

void rx_audio_set_demod(uint8_t demod) { demod_mode = demod; push_demod_to_core1(); }
 
// АРУ — два независимых контура:  
//  A) медленный левелинг по СРЕДНЕМУ |y| (τ≈10 с, симметрично) → объём/динамика  
//     сохраняются + долговременная подстройка под шум эфира (слабая реакция на средний уровень);  
//  B) быстрый лимитер поверх → мгновенная защита от «удара по ушам», не трогает обычную динамику.  
// Состояние в Q16 (значение << 16), коэффициенты EMA в Q24. τ ≈ 2^24/(Q24·44100).  
  
// --- Контур A: медленное среднее |y| ---  
static int64_t agc_avg_q16 = (int64_t)4096 << 16;   // среднее уровня, Q16  
#define AGC_AVG_ATT_Q24  38                 // рост среднего, τ≈10 с  
#define AGC_AVG_DEC_Q24  38                 // спад среднего,  τ≈10 с (симметрично → слабая реакция)  
#define AGC_REF        24576                // целевой уровень: base_gain = AGC_REF/avg  
#define AGC_AVG_MIN  ((int64_t)2048 << 16)  // мин. среднее → потолок усиления 12 крат  
  
// --- Контур B: быстрый лимитер ---  
static int64_t lim_peak_q16 = (int64_t)2048 << 16;  // пик лимитера, Q16  
#define LIMIT_CEIL      28000               // безопасный потолок по модулю (защита ушей)  
#define LIMIT_ATT_Q24  (1 << 24)            // атака мгновенная (α≈1.0)  
#define LIMIT_DEC_Q24   1902                // спад лимитера, τ≈0.2 с

// Два контура: A — медленный левелинг по среднему, B — быстрый лимитер.  
static inline int16_t agc_limit(int32_t x) {  
    // --- Контур A: медленное среднее |x| (τ≈10 с, симметрично) ---  
    int64_t ax  = (x < 0) ? -x : x;  
    int64_t tgt = ax << 16;                              // |отсчёт| в Q16  
    int64_t da  = tgt - agc_avg_q16;  
    agc_avg_q16 += (da * (da > 0 ? AGC_AVG_ATT_Q24  
                                 : AGC_AVG_DEC_Q24)) >> 24;  
    if (agc_avg_q16 < AGC_AVG_MIN) agc_avg_q16 = AGC_AVG_MIN;  
  
    int32_t avg = (int32_t)(agc_avg_q16 >> 16);  
    if (avg < 1) avg = 1;  
    // Базовое усиление ведём по СРЕДНЕМУ → громкие сигналы остаются громче шума.  
    // y1 может превышать int16 — держим в int32, клип делает лимитер ниже.  
    int32_t y1 = (int32_t)(((int64_t)x * AGC_REF) / avg);  
  
    // --- Контур B: быстрый лимитер (мгновенная атака, спад τ≈0.2 с) ---  
    int64_t ay  = (y1 < 0) ? -y1 : y1;  
    int64_t tp  = ay << 16;                              // |y1| в Q16  
    int64_t dp  = tp - lim_peak_q16;  
    lim_peak_q16 += (dp * (dp > 0 ? LIMIT_ATT_Q24  
                                  : LIMIT_DEC_Q24)) >> 24;  
    if (lim_peak_q16 < ((int64_t)1 << 16)) lim_peak_q16 = (int64_t)1 << 16;  
  
    int32_t lpk = (int32_t)(lim_peak_q16 >> 16);  
    if (lpk < 1) lpk = 1;  
    if (lpk > LIMIT_CEIL) {  
        // Прижимаем ТОЛЬКО когда пик выше безопасного потолка — иначе не вмешиваемся.  
        y1 = (int32_t)(((int64_t)y1 * LIMIT_CEIL) / lpk);  
    }  
  
    if (y1 >  32767) y1 =  32767;                        // финальный клип-страховка  
    if (y1 < -32768) y1 = -32768;  
    return (int16_t)y1;  
}
  
// ---- Источник аудиоданных от rx_dsp ----  
static const int16_t* bi = nullptr;  
static const int16_t* bq = nullptr;  
static uint8_t cur_n = 0;  
static uint8_t pos = 0;  
  
// Забирает следующий входной I/Q-отсчёт, демодулирует и фильтрует его.  
// Блок освобождается ровно один раз — когда все его отсчёты прочитаны.  
static bool get_demod_input(int32_t* out) {  
    if (out == nullptr) return false;  
  
    while (cur_n == 0 || pos >= cur_n) {  
        if (cur_n != 0) {  
            rx_dsp_release_audio();  
            cur_n = 0;  
            pos = 0;  
            bi = nullptr;  
            bq = nullptr;  
        }  
  
        if (!rx_dsp_get_audio(&bi, &bq, &cur_n)) {  
            cur_n = 0;  
            pos = 0;  
            bi = nullptr;  
            bq = nullptr;  
            return false;  
        }  
  
        pos = 0;  
  
        // Защита от некорректного пустого блока.  
        if (cur_n == 0 || bi == nullptr || bq == nullptr) {  
            rx_dsp_release_audio();  
            cur_n = 0;  
            pos = 0;  
            bi = nullptr;  
            bq = nullptr;  
            return false;  
        }  
    }  
  
    const int32_t m = (int32_t)bi[pos];
    ++pos;

    // Тап в активный декодер.
    rx_decoders_sample((int16_t)m);

    *out = m;
    return true;

}  
  
// Забирает следующую пару I/Q без демодуляции и фильтрации.  
// Та же логика освобождения блока, что и в get_demod_input.  
static bool get_iq_input(int32_t* i_out, int32_t* q_out) {  
    if (i_out == nullptr || q_out == nullptr) return false;  
  
    while (cur_n == 0 || pos >= cur_n) {  
        if (cur_n != 0) {  
            rx_dsp_release_audio();  
            cur_n = 0;  
            pos = 0;  
            bi = nullptr;  
            bq = nullptr;  
        }  
  
        if (!rx_dsp_get_audio(&bi, &bq, &cur_n)) {  
            cur_n = 0;  
            pos = 0;  
            bi = nullptr;  
            bq = nullptr;  
            return false;  
        }  
  
        pos = 0;  
  
        if (cur_n == 0 || bi == nullptr || bq == nullptr) {  
            rx_dsp_release_audio();  
            cur_n = 0;  
            pos = 0;  
            bi = nullptr;  
            bq = nullptr;  
            return false;  
        }  
    }  
  
    *i_out = (int32_t)bi[pos];  
    *q_out = (int32_t)bq[pos]; 
    ++pos;  
    return true;  
}  

  
// ---- Ресемплер 15625 -> 44100 Гц ----  
// rs_phase — дробная позиция между rs_x0 и rs_x1, в единицах 1/44100.  
// На каждом выходном отсчёте фаза увеличивается на 15625.  
// Если фаза пересекает 44100, на следующем вызове нужно сдвинуть входную  
// пару на один отсчёт. Отложенный сдвиг сохраняется при временно пустой очереди.  
static int32_t rs_xm1 = 0, rs_x0 = 0, rs_x1 = 0, rs_x2 = 0;   // окно I/SSB  
static int32_t rs_qm1 = 0, rs_q0 = 0, rs_q1 = 0, rs_q2 = 0;   // окно Q (RAW_IQ) 
static bool rs_started = false;  
static uint8_t rs_init_count = 0;  
static bool rs_advance_pending = false;  

// Фаза: 32-битный аккумулятор, один входной отсчёт = 2^32.  
// mu(Q15) = старшие 15 бит фазы. rs_step_q задаётся в rx_audio_poll().  
static uint32_t rs_phase_q = 0;  
static uint32_t rs_step_q  = 0;  
static float    rs_step    = 15625.0f;   // адаптивный номинал (Гц), из него считаем rs_step_q

static inline int16_t raw_iq_clip(float x) {  
    float g = x * (float)RAW_IQ_GAIN;  
    if (g > 32767.0f)  g = 32767.0f;  
    if (g < -32768.0f) g = -32768.0f;  
    return (int16_t)g;  
}  

// ---- Антиалиасинг-ФНЧ на выходе 44100 Гц (подавление образов ресемплера >12 кГц) ----  
// Каскад 2 секций → Баттерворт 4-го порядка, fc ≈ 5300 Гц (полоса 0…5 кГц ровная ≤1 дБ).
// Коэффициенты Q13 (1.0 = 8192), Direct Form I, int64-аккумулятор.  
// ВНИМАНИЕ: значения стартовые — проверить/подстроить на железе по реальной АЧХ.  
#define BQ_Q 13  
  
typedef struct {  
    int32_t b0, b1, b2, a1, a2;   // a1,a2 с «родным» знаком RBJ (вычитаются в DF-I)  
    int64_t z1, z2;               // состояние (int64 — запас от переполнения)  
} biquad_t;  
  
// Две секции 4-го порядка Баттерворта, fc=5300 Гц @ 44100 (Q=0.5412 и Q=1.3066):
static biquad_t bq_ssb[2] = {  
    { 682, 1363, 682, -7305, 1840, 0, 0 },  
    { 882, 1764, 882, -9452, 4788, 0, 0 },  
};
static biquad_t bq_i[2] = {   // RAW_IQ: отдельный экземпляр на I  
    { 682, 1363, 682, -7305, 1840, 0, 0 },  
    { 882, 1764, 882, -9452, 4788, 0, 0 },
};  
static biquad_t bq_q[2] = {   // RAW_IQ: отдельный экземпляр на Q  
    { 682, 1363, 682, -7305, 1840, 0, 0 },  
    { 882, 1764, 882, -9452, 4788, 0, 0 },
};  

  
static inline int32_t __not_in_flash_func(biquad_run)(biquad_t* s, int32_t x) {  
    int64_t acc = (int64_t)s->b0 * x + s->z1;  
    int32_t y   = (int32_t)(acc >> BQ_Q);  
    s->z1 = (int64_t)s->b1 * x - (int64_t)s->a1 * y + s->z2;  
    s->z2 = (int64_t)s->b2 * x - (int64_t)s->a2 * y;  
    return y;  
}  
  
static inline int32_t biquad_cascade(biquad_t* sec, int32_t x) {  
    x = biquad_run(&sec[0], x);  
    x = biquad_run(&sec[1], x);  
    return x;  
}  
  
static inline void biquad_reset(biquad_t* sec) {  
    sec[0].z1 = sec[0].z2 = 0;  
    sec[1].z1 = sec[1].z2 = 0;  
}

// Коэффициенты Catmull-Rom, умноженные на 2 (чтобы убрать 0.5). int32.  
static int32_t rs_c0 = 0, rs_c1 = 0, rs_c2 = 0, rs_c3 = 0;      // SSB / I  
static int32_t rs_cq0 = 0, rs_cq1 = 0, rs_cq2 = 0, rs_cq3 = 0;  // Q (RAW_IQ)

// 2*c0=2y0; 2*c1=y1-ym1; 2*c2=2ym1-5y0+4y1-y2; 2*c3=3(y0-y1)+y2-ym1  
static inline void farrow_calc_i(int32_t ym1, int32_t y0, int32_t y1, int32_t y2,  
                                 int32_t* c0, int32_t* c1, int32_t* c2, int32_t* c3) {  
    *c0 = 2 * y0;  
    *c1 = y1 - ym1;  
    *c2 = 2 * ym1 - 5 * y0 + 4 * y1 - y2;  
    *c3 = 3 * (y0 - y1) + y2 - ym1;  
}  
  
// Оценка полинома по схеме Горнера — единственное, что считается на 44100 Гц.  
// Горнер в Q15. Возвращает 2*y (деление на 2 с округлением — у вызывающего).  
static inline int32_t farrow_eval_i(int32_t c0, int32_t c1, int32_t c2, int32_t c3,  
                                    uint32_t mu_q15) {  
    int32_t t = c3;  
    t = (int32_t)(((int64_t)t * mu_q15) >> 15) + c2;  
    t = (int32_t)(((int64_t)t * mu_q15) >> 15) + c1;  
    t = (int32_t)(((int64_t)t * mu_q15) >> 15) + c0;  
    return t;   // = 2*y  
}


static bool next_frame(int16_t* l, int16_t* r) {  
    if (l == nullptr || r == nullptr) {  
        return false;  
    }  
  
    // Запрос смены режима: сбрасываем ресемплер и отпускаем удерживаемый блок,  
    // чтобы стерео(2 канала) и моно(1 канал) не перемешались в интерполяторе.  
    if (mode_changed) {    
        mode_changed      = false;    
        rs_started        = false;    
        rs_init_count     = 0;    
        rs_advance_pending = false;    
        rs_phase_q         = 0;    
        biquad_reset(bq_ssb);     // сброс фильтров, чтобы не щёлкало на SSB↔RAW_IQ  
        biquad_reset(bq_i);    
        biquad_reset(bq_q);    
        if (cur_n != 0) {
            rx_dsp_release_audio();  
            cur_n = 0;  
            pos   = 0;  
            bi    = nullptr;  
            bq    = nullptr;  
        }  
        return false;   // один пустой кадр на переключении — несущественно  
    }  
  
    if (rx_mode == RX_MODE_TEST) {  
        // Тестовый синус 1000 Гц при выходной частоте 44100 Гц.  
        static uint32_t ph = 0;  
        ph += (uint32_t)(((uint64_t)1000 << 32) / 44100);  
        const float s = 10000.0f *  
            sinf(6.2831853071795864769f * (float)(ph >> 24) / 256.0f);  
        *l = (int16_t)s;  
        *r = (int16_t)s;  
        return true;  
    }  
  
    if (rx_mode == RX_MODE_RAW_IQ) {  
        // Сырые I/Q, общая фаза, линейная интерполяция 15625 -> 44100 на оба канала.  
        if (!rs_started) {  
            if (rs_init_count == 0) {  
                if (!get_iq_input(&rs_xm1, &rs_qm1)) return false;  
                rs_init_count = 1;  
            }  
            if (rs_init_count == 1) {  
                if (!get_iq_input(&rs_x0, &rs_q0)) return false;  
                rs_init_count = 2;  
            }  
            if (rs_init_count == 2) {  
                if (!get_iq_input(&rs_x1, &rs_q1)) return false;  
                rs_init_count = 3;  
            }  
            if (rs_init_count == 3) {  
                if (!get_iq_input(&rs_x2, &rs_q2)) return false;  
                rs_init_count = 4;  
            }  
            farrow_calc_i(rs_xm1, rs_x0, rs_x1, rs_x2, &rs_c0, &rs_c1, &rs_c2, &rs_c3);  
            farrow_calc_i(rs_qm1, rs_q0, rs_q1, rs_q2, &rs_cq0, &rs_cq1, &rs_cq2, &rs_cq3);
            rs_phase_q  = 0;  
            rs_advance_pending = false;  
            rs_started = true;
        }
  
        if (rs_advance_pending) {  
            int32_t next_i, next_q; 
            if (!get_iq_input(&next_i, &next_q)) return false;  
            rs_xm1 = rs_x0;  rs_x0 = rs_x1;  rs_x1 = rs_x2;  rs_x2 = next_i;  
            rs_qm1 = rs_q0;  rs_q0 = rs_q1;  rs_q1 = rs_q2;  rs_q2 = next_q;  
            farrow_calc_i(rs_xm1, rs_x0, rs_x1, rs_x2, &rs_c0, &rs_c1, &rs_c2, &rs_c3);  
            farrow_calc_i(rs_qm1, rs_q0, rs_q1, rs_q2, &rs_cq0, &rs_cq1, &rs_cq2, &rs_cq3);
            rs_advance_pending = false;  
        }
  
        // RX_MODE_RAW_IQ  
        const uint32_t mu_q15 = rs_phase_q >> 17;    
        const int32_t  yi2 = farrow_eval_i(rs_c0,  rs_c1,  rs_c2,  rs_c3,  mu_q15);    
        const int32_t  yq2 = farrow_eval_i(rs_cq0, rs_cq1, rs_cq2, rs_cq3, mu_q15);    
        int32_t        yi  = (yi2 + 1) >> 1;    
        int32_t        yq  = (yq2 + 1) >> 1;    
        yi = biquad_cascade(bq_i, yi);     // антиалиасинг-ФНЧ на I  
        yq = biquad_cascade(bq_q, yq);     // антиалиасинг-ФНЧ на Q  
          
        const uint32_t prev = rs_phase_q;    
        rs_phase_q += rs_step_q;    
        if (rs_phase_q < prev) rs_advance_pending = true;  
    
        *l = raw_iq_clip((float)yi);  // I -> левый    
        *r = raw_iq_clip((float)yq);  // Q -> правый
        return true;  
    }  
  
    // RX_MODE_SSB: демодулированный моно.  
    if (!rs_started) {  
        if (rs_init_count == 0) {  
            if (!get_demod_input(&rs_xm1)) return false;  
            rs_init_count = 1;  
        }  
        if (rs_init_count == 1) {  
            if (!get_demod_input(&rs_x0)) return false;  
            rs_init_count = 2;  
        }  
        if (rs_init_count == 2) {  
            if (!get_demod_input(&rs_x1)) return false;  
            rs_init_count = 3;  
        }  
        if (rs_init_count == 3) {  
            if (!get_demod_input(&rs_x2)) return false;  
            rs_init_count = 4;  
        }  
        farrow_calc_i(rs_xm1, rs_x0, rs_x1, rs_x2, &rs_c0, &rs_c1, &rs_c2, &rs_c3);  
        rs_phase_q  = 0;  
        rs_advance_pending = false;  
        rs_started = true;
    }
  
    if (rs_advance_pending) {  
        int32_t next_x; 
        if (!get_demod_input(&next_x)) return false;  
        rs_xm1 = rs_x0;  
        rs_x0  = rs_x1;  
        rs_x1  = rs_x2;  
        rs_x2  = next_x;  
        farrow_calc_i(rs_xm1, rs_x0, rs_x1, rs_x2, &rs_c0, &rs_c1, &rs_c2, &rs_c3); 
        rs_advance_pending = false;  
    }
  
    // RX_MODE_SSB  
    const uint32_t mu_q15 = rs_phase_q >> 17;          // старшие 15 бит фазы    
    const int32_t  y2x    = farrow_eval_i(rs_c0, rs_c1, rs_c2, rs_c3, mu_q15);    
    int32_t        y      = (y2x + 1) >> 1;            // 2*y -> y с округлением    
    y = biquad_cascade(bq_ssb, y);                     // антиалиасинг-ФНЧ 44.1 кГц  
      
    const uint32_t prev = rs_phase_q;    
    rs_phase_q += rs_step_q;    
    if (rs_phase_q < prev) rs_advance_pending = true;  // перенос 2^32 = сдвиг окна  
    
    const int16_t s = agc_limit(y);                    // agc_limit(int32_t) — ок
    *l = s;  
    *r = s;  
    return true;  
}  

  
static USBAudioStream usb_out;  
static int16_t pcm_buffer[192 * 2];  // до 192 stereo frames  
  
void rx_audio_init(void) {    
    agc_avg_q16  = (int64_t)4096 << 16;  
    lim_peak_q16 = (int64_t)2048 << 16;
    memset(pcm_buffer, 0, sizeof(pcm_buffer));  
    // Синхронизируем демодулятор Core 1 со стартовым режимом (rx_mode/demod_mode),    
    // иначе Core 1 остаётся в RXD_DEMOD_RAW и боковая не давится.    
    push_demod_to_core1();

    if (!TinyUSBDevice.isInitialized()) {  
        TinyUSBDevice.begin(0);  
    }  
  
    auto cfg = usb_out.defaultConfig(TX_MODE);  
    cfg.sample_rate = 44100;  
    cfg.channels = 2;  
    cfg.bits_per_sample = 16;  
  
    usb_out.begin(cfg);  
}  
  
// Вызывать регулярно из loop(). Не вызывать параллельно из таймера:  
// у USB-потока и ресемплера должен быть один потребитель.  
void rx_audio_poll(void) {
    // --- Адаптивная подстройка темпа выдачи под часы USB-хоста ---  
    // Держим backlog очереди aud около середины (TARGET).  
    // Середина aud-очереди (глубина RX_AUD_BUFS=8 в rx_dsp.cpp).  
    // Литерал, т.к. сам макрос RX_AUD_BUFS в rx_audio.cpp не виден.  
    const float   TARGET = 4.0f; // TARGET = (float)(RX_AUD_BUFS / 2);  // ≈4 при RX_AUD_BUFS=8
    const float   K      = 6.0f;      // крутизна подстройки (Гц на слот)  
    const float   LIMIT  = 60.0f;     // макс. отклонение шага, Гц (~0.4%)  
    const uint8_t fill   = rx_dsp_aud_fill();  
  
    float adj = K * ((float)fill - TARGET);  
    if (adj >  LIMIT) adj =  LIMIT;  
    if (adj < -LIMIT) adj = -LIMIT;  
    rs_step = 15625.0f + adj;   // backlog велик → шаг больше → входы читаются чаще  
    // Целочисленный шаг фазы Фарроу: (rs_step / 44100) * 2^32.  
    rs_step_q = (uint32_t)((double)rs_step * (4294967296.0 / 44100.0));

    for (;;) {  
        int avail_frames = usb_out.availableForWrite() / 4;  
        if (avail_frames <= 0) {  
            break;  
        }  
  
        if (avail_frames > 192) {  
            avail_frames = 192;  
        }  
  
        int frames = 0;  
        int16_t l = 0;  
        int16_t r = 0;  
  
        while (frames < avail_frames && next_frame(&l, &r)) {  
            pcm_buffer[frames * 2]     = l;  
            pcm_buffer[frames * 2 + 1] = r;  
            ++frames;  
        }  
  
        if (frames > 0) {  
            usb_out.write((uint8_t*)pcm_buffer, frames * 4);  
        }  
  
        // Источник временно пуст. Не читаем дальше и сохраняем состояние  
        // ресемплера до следующего вызова rx_audio_poll().  
        if (frames < avail_frames) {  
            break;  
        }  
    }  
}


uint8_t rx_audio_get_mode(void) { return rx_mode; }  
 
uint8_t rx_audio_get_demod(void) { return demod_mode; }
