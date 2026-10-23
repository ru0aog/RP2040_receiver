/**  
 * ============================================================================  
 *  rx_dsp.cpp — модуль децимирующего DSP-тракта (core1)  
 *  Версия 2.26 от 2026-10-20, автор RU0AOG  
 *  
 *  Тракт: 32-бит слова [I|Q] из rx_capture → центрирование → NCO-сдвиг →  
 *  3 каскада half-band FIR (/8) → SPSC-очередь базовой полосы  
 *  int16 I/Q по 64 отсчёта на блок.  
 *  
 *  Fs на канал = RX_ADC_SAMPLE_RATE_HZ/2 (~125 ksps при 250k слов/с),  
 *  Fs_out = Fs/8 ≈ 15.6 ksps на канал (Nyquist ±7.8 кГц — покрывает  
 *  полосу всех узкополосных модов и ПЧ-сдвиг).  
 * ============================================================================  
 */  
  
#include <Arduino.h>  
#include "vfo_hardware.h"  
#include "pico/multicore.h"  
  
// ---- Параметры тракта ----  
#define RX_FS_IN_HZ           (RX_ADC_SAMPLE_RATE_HZ / 2.0f)  
#define CIC_R                 2            // децимация CIC: 125k → 62.5k    
#define CIC_N                 3            // каскадов интеграторов/гребёнок    
#define RX_DECIM_STAGES       2            // HB-каскады: 62.5k → 31.25k → 15.625k    
#define RX_FS_OUT_HZ          (RX_FS_IN_HZ / 8.0f)    // 15625 Гц

// NCO: 32-битный фазовый аккумулятор  
static volatile uint32_t nco_phase = 0;  
static volatile uint32_t nco_step  = 0;     // 2^32 * f_nco / Fs_in  

static volatile uint32_t clip_count = 0;   // диагностика: число срабатываний лимитера

// sin/cos LUT: 1024 точки, Q15, квадратурная пара  
// табличная генерация опорного гармонического сигнала (LUT — Look-Up Table)
static int16_t nco_lut[1024];       // таблица LUT 1024*16бит = 2 кБ
// Функция заполняет таблицу одним полным периодом идеальной синусоиды
// на целочисленном CPU функция медленная!
static void nco_lut_init(void) {
    for (int i = 0; i < 1024; i++)  
        nco_lut[i] = (int16_t)(32767.0f * sinf(2.0f * PI * i / 1024.0f));  
}
// быстрое извлечение квадратурной пары (синуса и косинуса) из заранее подготовленной таблицы
static inline void nco_sincos(uint32_t ph, int16_t* s, int16_t* c) {  
    uint16_t idx = ph >> 22;                // Битовый сдвиг фазы. Старшие 10 бит
    *s = nco_lut[idx];                      // Извлечение синуса
    *c = nco_lut[(idx + 256) & 1023];       // Извлечение косинуса. cos = sin сдвинутый на 90°  
}  


// Каскад 0 (15.6 ksps): проверенный 15-отводный, ~80 дБ.  
// Сумма = 16384 + 2·(10030 − 2250 + 532 − 120) = 32768 → DC-gain = 1.0.  
#define HB0_NTAPS 15  
static const int16_t hb0_taps[HB0_NTAPS] = {  
    -120,     0,   532,     0, -2250,  
       0, 10030, 16384, 10030,     0,  
    -2250,     0,   532,     0,  -120  
};

// Каскады 1–2 (7.8 / 3.9 ksps): 31 отвод, окно Блэкмана-Харриса,  
// ~90 дБ заграждения — доедаем то, что заворачивается у краёв.  
#define HBX_NTAPS 31  
static const int16_t hbx_taps[HBX_NTAPS] = {  
       0,     0,     3,     0,   -25,     0,   119,     0,  
    -399,     0,  1086,     0, -2760,     0, 10168, 16384,  
   10168,     0, -2760,     0,  1086,     0,  -399,     0,  
     119,     0,   -25,     0,     3,     0,     0  
};


typedef struct {  
    int32_t d[HBX_NTAPS];          // линия задержки под максимальную длину  
    uint8_t w;                     // позиция записи  
    uint8_t phase;                 // триггер чётности децимации  
    uint8_t ntaps;                 // длина этого каскада  
    const int16_t* taps;           // таблица коэффициентов  
} hb_state_t; 

static hb_state_t hb_i[RX_DECIM_STAGES];  
static hb_state_t hb_q[RX_DECIM_STAGES];
// CIC R=8, N=3. Интеграторы — на входной частоте, гребёнки — на 15.6 ksps.  
// Переполнение int32 в интеграторах — штатное (работает по модулю 2^32).  
static int32_t cic_int_i[CIC_N],  cic_int_q[CIC_N];  
static int32_t cic_comb_i[CIC_N], cic_comb_q[CIC_N];  
static uint8_t cic_cnt = 0;  
  
// Возвращает true раз в CIC_R входных отсчётов с децимированной парой.  
static inline bool cic_push_iq(int32_t xi, int32_t xq, int32_t* yi, int32_t* yq) {  
    // интеграторы на полной частоте  
    cic_int_i[0] += xi; cic_int_i[1] += cic_int_i[0]; cic_int_i[2] += cic_int_i[1];  
    cic_int_q[0] += xq; cic_int_q[1] += cic_int_q[0]; cic_int_q[2] += cic_int_q[1];  
  
    if (++cic_cnt < CIC_R) return false;  
    cic_cnt = 0;  
  
    // гребёнки уже на прореженной частоте  
    int32_t ti = cic_int_i[2], tq = cic_int_q[2];  
    for (int k = 0; k < CIC_N; ++k) {  
        int32_t di = ti - cic_comb_i[k]; cic_comb_i[k] = ti; ti = di;  
        int32_t dq = tq - cic_comb_q[k]; cic_comb_q[k] = tq; tq = dq;  
    }  
  
// DC-gain CIC = R^N = 2^3 = 8 → возвращаем масштаб (с округлением, +0.5 LSB)  
    *yi = (ti + 4) >> 3;      
    *yq = (tq + 4) >> 3;
    return true;  
}

// реализует один шаг фильтрации и децимации (прореживания) данных через Half-Band FIR (КИХ) фильтр
static inline bool hb_push(hb_state_t* st, int32_t x, int32_t* out) {
    // Новый входящий отсчёт x записывается в историю фильтра (линию задержки st->d) по текущему указателю записи st->w
    st->d[st->w] = x;
    // Указатель st->w сдвигается на следующий элемент
    // Операция % HB_NTAPS (в данном случае % 15) зацикливает массив,
    // обеспечивает циклический переход индекса в программном коде, который компилятор может оптимизировать.
    if (++st->w >= st->ntaps) st->w = 0;
    // Прореживание частоты на 2
    st->phase ^= 1;     // Переменная st->phase это однобитный триггер чётности 
    // (флаг, который при каждом вызове функции переключается между 0 и 1 с помощью операции XOR ^= 1)
    // Если шаг нечётный (st->phase == 1), функция просто сохраняет семпл в историю и мгновенно выходит, возвращая false
    if (st->phase) return false;      // нечётный вход — выхода нет
    // Математическое вычисление свертки КИХ-фильтра
    int32_t acc = 0;    // аккумулятор сбрасывается в ноль
    // Запускается цикл свертки на 15 отводов (HB_NTAPS)
    for (uint8_t k = 0; k < st->ntaps; k++) {
        // быстрое вычисление правильного физического индекса внутри кольцевого массива, 
        // начиная от самой старой сохраненной точки к самой новой
        int idx = (int)st->w + k;  
        if (idx >= (int)st->ntaps) idx -= st->ntaps;
        // цифровое умножение отсчёта сигнала на соответствующий коэффициент фильтра (hb_taps)
        acc += st->d[idx] * st->taps[k];  
    }
    // Масштабирование и выдача готового отсчёта (с округлением, +0.5 LSB)  
    *out = (acc + (1 << 14)) >> 15;
    return true;
}


// 16-битный ограничитель
static inline int16_t sat16(int32_t v) {
// Если значение превысило верхний порог 32767, оно жестко ограничивается до максимума 32767
// Если значение ушло ниже минимального порога -32768, оно ограничивается снизу до -32768
// Если значение находится в безопасных пределах, оно возвращается без изменений
    if ((v >> 15) ^ (v >> 31)) {
        clip_count++;                       // сигнал упёрся в потолок int16  
        return (v >> 31) ^ 0x7FFF;
    }
    return (int16_t)v;
}

#define RX_OUT_SAMPLES   (RX_BLOCK_SAMPLES / 8)      // 64 пары I/Q на блок
#define RX_OUT_BUFS      4

static int16_t out_i[RX_OUT_BUFS][RX_OUT_SAMPLES];  // выходные буферы обработанных данных I/Q
static int16_t out_q[RX_OUT_BUFS][RX_OUT_SAMPLES];
static volatile uint8_t  out_head = 0;              // запись: только core1  
static volatile uint8_t  out_tail = 0;              // запись: только core0  
static volatile uint32_t out_dropped = 0;           // диагностика переполнений  
static int32_t dc_i = 2048 << 8;
static int32_t dc_q = 2048 << 8;

// ---- Аудио-базбенд: I/Q с выхода CIC (15.625 ksps), до HB-каскадов ----  
#define RX_AUD_SAMPLES   64        // пар I/Q в слоте  
#define RX_AUD_BUFS      8  
  
static int16_t aud_i[RX_AUD_BUFS][RX_AUD_SAMPLES];  
static int16_t aud_q[RX_AUD_BUFS][RX_AUD_SAMPLES];  
static volatile uint8_t  aud_head = 0;   // пишет только core1  
static volatile uint8_t  aud_tail = 0;   // читает только core0  
static volatile uint32_t aud_dropped = 0;  
static uint8_t aud_wr_n = 0;             // заполнение текущего слота

// ---- Демодулятор, перенесён с Core 0 (rx_audio.cpp) ----  
#define RXD_DEMOD_RAW 0   // сырые I/Q в aud (стерео)  
#define RXD_DEMOD_USB 1   // верхняя боковая, моно  
#define RXD_DEMOD_LSB 2   // нижняя боковая, моно  
static volatile uint8_t rxd_demod_mode = RXD_DEMOD_RAW;  // ставит Core 0  

// ---- Hilbert FIR: 31 taps, Q15 fixed-point ----  
#define HILB_N 31  
#define HILB_M ((HILB_N - 1) / 2)   // 15 отсчётов групповой задержки  

// Только 16 ненулевых отводов (чётные позиции). Q15 = round(coeff * 32768).  
// Пара {смещение в буфере, коэффициент}. Смещение k = позиция в исходном hilb_h.  
static const int16_t hilb_hq15[16] = {  
    -111, -193, -439, -924, -1753, -3211, -6341, -20654,  
   20654,  6341,  3211,  1753,   924,   439,   193,    111  
};  
// k-индексы ненулевых отводов в исходном 31-отводном фильтре:  
static const uint8_t hilb_k[16] = {  
    0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30  
};  
  
static int16_t hilb_buf[HILB_N] = {};  
static int     hilb_idx = 0;  
  
static int16_t i_del[HILB_M + 1] = {};  
static int     i_del_idx = 0;

  
// Один демод-отсчёт из пары I/Q (15.625 ksps). Должен вызываться на КАЖДОМ  
// emit, даже если очередь полна — иначе рвётся фаза Гильберта.  
static inline int32_t __not_in_flash_func(rxd_demod_sample)(int32_t i, int32_t q) {  
    i_del[i_del_idx] = (int16_t)i;  
    int del = i_del_idx - HILB_M;  
    if (del < 0) del += (HILB_M + 1);  
    const int32_t i_d = (int32_t)i_del[del];  
    if (++i_del_idx >= (HILB_M + 1)) i_del_idx = 0;
  
    hilb_buf[hilb_idx] = (int16_t)q;  
    if (++hilb_idx >= HILB_N) hilb_idx = 0;  
  
    // Свёртка по ненулевым отводам. Фильтр антисимметричен:  
    // hilb_hq15[15-n] == -hilb_hq15[n], а hilb_k[15-n] == HILB_N-1 - hilb_k[n].  
    // => 16 умножений сводятся к 8: c[n]*(buf[idx_lo] - buf[idx_hi]).  
    // Умножение 32-битное (|c|<=20654, |diff|<=65534, |произв.|~1.35e9 < INT32_MAX),  
    // аккумулятор остаётся 64-битным (сумма 8 произв. в худшем случае ~2.2e9 > INT32_MAX).  
    int64_t acc = 0;  
    for (int n = 0; n < 8; ++n) {  
        const int k = hilb_k[n];               // k = 0,2,...,14  
  
        int idx_lo = hilb_idx + HILB_N - 1 - k; // соответствует отводу n  
        if (idx_lo >= HILB_N) idx_lo -= HILB_N;  
  
        int idx_hi = hilb_idx + k;              // соответствует отводу 15-n  
        if (idx_hi >= HILB_N) idx_hi -= HILB_N;  
  
        const int32_t diff = (int32_t)hilb_buf[idx_lo] - (int32_t)hilb_buf[idx_hi];  
        acc += (int32_t)((int32_t)hilb_hq15[n] * diff);  
    }  
    const int32_t hq = (int32_t)(acc >> 15);   // обратно в масштаб отсчёта
  
    return (rxd_demod_mode == RXD_DEMOD_LSB) ? (i_d + hq) : (i_d - hq);  
}


// ---- Аудио-НЧ-фильтр 3 кГц для SSB (работает на 15.625 ksps, Core 1) ----  
// Ограничивает полосу демодулированного моно-сигнала до ~3 кГц по -1 дБ.  
// Оконный sinc (окно Блэкмана), Q15, DC-gain = 1.0. Линейная фаза.  
#define AF_LP_N     63          // число отводов (нечётное → линейная фаза)  
#define AF_LP_FC_HZ 3100.0f     // -6 дБ чуть выше 3 кГц → ~ -1 дБ на 3 кГц  
  
static int16_t af_lp_taps[AF_LP_N];  
static int32_t af_lp_buf[AF_LP_N] = {};  
static int     af_lp_idx = 0;  
  
static void af_lp_init(void) {  
    const int   M  = (AF_LP_N - 1) / 2;  
    const float fc = AF_LP_FC_HZ / RX_FS_OUT_HZ;   // нормир. частота среза (0..0.5)  
    float h[AF_LP_N];  
    float sum = 0.0f;  
    for (int n = 0; n < AF_LP_N; n++) {  
        const int m = n - M;  
        float s = (m == 0) ? (2.0f * fc)  
                           : sinf(2.0f * PI * fc * m) / (PI * m);   // идеальный sinc  
        float w = 0.42f - 0.5f  * cosf(2.0f * PI * n / (AF_LP_N - 1))  
                        + 0.08f * cosf(4.0f * PI * n / (AF_LP_N - 1)); // окно Блэкмана  
        h[n] = s * w;  
        sum += h[n];  
    }  
    for (int n = 0; n < AF_LP_N; n++)                              // нормировка DC=1, Q15  
        af_lp_taps[n] = (int16_t)lroundf(h[n] / sum * 32768.0f);  
}  
  
// Один отсчёт НЧ-фильтра. Вызывать на КАЖДОМ emit в SSB — иначе рвётся  
// линия задержки (как у Гильберта).  
static inline int32_t __not_in_flash_func(af_lp_push)(int32_t x) {    
    af_lp_buf[af_lp_idx] = x;    
    if (++af_lp_idx >= AF_LP_N) af_lp_idx = 0;    // af_lp_idx — самый старый отсчёт    
    
    // Линейно-фазный FIR симметричен: taps[k] == taps[N-1-k].    
    // Сворачиваем пары → 32 умножения вместо 63.    
    // Умножение 32-битное (единичный MULS на M0+), а не int64 (__aeabi_lmul):    
    //   |tap| <= ~13000, |buf_lo+buf_hi| <= ~131072 → |произв.| < 1.7e9 < INT32_MAX.    
    // Аккумулятор держим 64-битным (сумма 32 произв. может превысить INT32_MAX).    
    int64_t acc = 0;    
    int lo = af_lp_idx;                  // самый старый (k = 0)    
    int hi = af_lp_idx + AF_LP_N - 1;    // самый новый (k = N-1)    
    if (hi >= AF_LP_N) hi -= AF_LP_N;    
    for (int k = 0; k < AF_LP_N / 2; k++) {              // 31 пара    
        acc += (int32_t)af_lp_taps[k] * (af_lp_buf[lo] + af_lp_buf[hi]);    
        if (++lo >= AF_LP_N) lo = 0;    
        if (--hi < 0)        hi = AF_LP_N - 1;    
    }    
    acc += (int32_t)af_lp_taps[AF_LP_N / 2] * af_lp_buf[lo];  // центральный отвод    
    return (int32_t)((acc + (1 << 14)) >> 15);    // Q15 → масштаб отсчёта, +0.5 LSB    
}


// ---- Инструментирование загрузки Core 1 ----  
static uint32_t busy_us_acc  = 0;   // время в обработке блоков за окно  
static uint32_t total_us_acc = 0;   // полное время за окно  
static volatile uint32_t dsp_load_permille = 0;  // загрузка, промилле (0..1000)

// бесконечный изолированный рабочий цикл ядра Core 1
// цифровой преобразователь частоты вниз (DDC — Digital Down Converter)
static void __not_in_flash_func(rx_dsp_entry)(void) {
    uint32_t t_prev = time_us_32();   // метка времени предыдущей итерации  
    // бесконечный цикл ядра Core 1
    while (true) {
        // кормим собачку
        watchdog_update();
        // Учёт полного времени итерации (включая холостое ожидание).  
        const uint32_t t_now = time_us_32();  
        total_us_acc += (t_now - t_prev);  
        t_prev = t_now;  
        // проверяем очередь АЦП (функция в rx_capture.cpp)
        const volatile uint32_t* blk = rx_capture_get_block();
        // если данных ещё нет - на следующую итерацию
        if (!blk) {
            tight_loop_contents();  // подсказка компилятору для цикла ожидания; в этой итерации новых данных нет
            continue;
        }
        // Начало полезной работы: засекаем время обработки блока.  
        const uint32_t t_busy_start = time_us_32();
        // данные из DMA пришли
    #if RTTY_DECODE 
        uint8_t wr        = out_head;                           // индекс текущего свободного буфера выходной очереди
        uint8_t next_head = (uint8_t)((wr + 1) % RX_OUT_BUFS);  // следующий шаг кольцевого буфера
        // Если очередь полна (queue_full == true), код не останавливает работу. 
        // Блок всё равно будет обработан, чтобы не разорвать внутренние состояния 
        // цифрового гетеродина и фильтров, но результат в конце будет просто сброшен в мусорку.
        bool    queue_full = (next_head == out_tail);
        uint8_t n_out = 0;
    #endif
        // устанавливаем указатель
        const volatile int16_t* s = (const volatile int16_t*)blk;
        // DMA_SIZE_16: каждое 32-битное слово FIFO = пара отсчётов  
        // [15:0] = ADC0 (I), [31:16] = ADC1 (Q). Пар в блоке = RX_BLOCK_SAMPLES. 
        // внутренний цикл обработки блока данных. Обрабатываем пары I/Q
        for (int n = 0; n < RX_BLOCK_SAMPLES; n++) {
            // извлекаем текущие отсчёты из массива
            int32_t i_meas = s[2 * n];
            int32_t q_meas = s[2 * n + 1];

            // Центрирование
                // EMA с коэффициентом 1/256
                dc_i += ((i_meas << 8) - dc_i) >> 8;
                dc_q += ((q_meas << 8) - dc_q) >> 8;
                // текущее вычисленное среднее (сдвинутое обратно на 8 бит вправо)
                // вычитается из измерений.
                // Получаем сигнал в i_raw и q_raw, компенсированный от постоянного смещения
                int32_t i_raw = i_meas - (dc_i >> 8);
                int32_t q_raw = q_meas - (dc_q >> 8);
  
            // Комплексный перенос частоты (Смеситель NCO)
                int16_t sn, cs;
                // Из таблицы синусов (NCO LUT) берутся значения опорного сигнала нужной частоты подстройки
                nco_sincos(nco_phase, &sn, &cs);
                // Фазовый аккумулятор продвигается вперед на величину nco_step
                nco_phase += nco_step;
                // Производится комплексное умножение вектора входного сигнала на вектор гетеродинирования
                // Сдвиг >> 15 возвращает результат из формата Q15 обратно к обычным целым числам
                // После этой операции целевой радиосигнал математически смещается в нулевую промежуточную частоту
                int32_t i_rot = (i_raw * cs + q_raw * sn) >> 15;  
                int32_t q_rot = (q_raw * cs - i_raw * sn) >> 15;

 
            // CIC /2 → 2 каскада HB /2 (итого /8) → 15.625 ksps  
            int32_t ci, cq;    
            bool emit = cic_push_iq(i_rot, q_rot, &ci, &cq);  
            int32_t vi = ci, vq = cq; 
  
            if (emit) {
                for (int k = 0; k < RX_DECIM_STAGES; ++k) {
                    int32_t oi, oq;

                    // Обязательно вызываем оба фильтра на каждом входном отсчёте.
                    bool emit_i = hb_push(&hb_i[k], vi, &oi);
                    bool emit_q = hb_push(&hb_q[k], vq, &oq);

                    // I и Q обязаны выдавать отсчёт одновременно.
                    if (emit_i != emit_q) {
                        emit = false;
                        break;
                    }

                    if (!emit_i) {
                        emit = false;
                        break;
                    }

                    vi = oi;
                    vq = oq;
                }
            }

            // Насыщение и запись в буфер базовой полосы
            // Если фильтры выдали отсчет (emit == true), выходная очередь не полна, и мы не вылезли за размер блока (64 отсчета)
            if (emit) {  
                // Гильберт считаем ВСЕГДА (даже при полной очереди), иначе рвётся фаза.  
                int16_t si, sq;  
                if (rxd_demod_mode == RXD_DEMOD_RAW) {  
                    si = sat16(vi);  
                    sq = sat16(vq);  
                } else {
                    int32_t d  = rxd_demod_sample(vi, vq);  // моно SSB, до фильтра  
                    int32_t df = af_lp_push(d);             // НЧ-фильтр 3 кГц  
                    int16_t m  = sat16(df);                 // моно SSB  
                    si = m;  
                    sq = m;  
                }  
            
                uint8_t anext = (uint8_t)((aud_head + 1) % RX_AUD_BUFS);  
                if (anext != aud_tail) {  
                    aud_i[aud_head][aud_wr_n] = si;  
                    aud_q[aud_head][aud_wr_n] = sq;  
                    if (++aud_wr_n >= RX_AUD_SAMPLES) {  
                        __dmb();  
                        aud_head = anext;  
                        aud_wr_n = 0;  
                    }  
                } else {  
                    aud_dropped++;  
                }  
                // очередь out убрана: с RTTY_DECODE=0 потребителя нет (снимаем мёртвую работу с Core 1)  
            }






        }
        // После окончания внутреннего цикла 
        // функция rx_capture_release() сообщает DMA-движку на Core 0,
        // что входной буфер blk полностью обработан и его можно использовать повторно
        rx_capture_release();
    #if RTTY_DECODE
        // Если выходная очередь была переполнена, инкрементируется диагностический счетчик потерь out_dropped
        if (queue_full) {  
            out_dropped++;            // блок выхода потерян, вход обработан  
        } else {
            // Если всё в порядке, инструкция барьера памяти __dmb() 
            // принудительно обеспечивает порядок операций с памятью между ядрами.
            // Только после этого индекс out_head сдвигается на next_head, делая новый блок видимым для декодеров на Core 0
            __dmb();                  // данные блока видимы до публикации head  
            out_head = next_head;
        }
    #endif
        // Конец полезной работы: накапливаем время обработки блока.  
        busy_us_acc += (time_us_32() - t_busy_start);  
  
        // Раз в ~1 с пересчитываем загрузку. Деление — вне горячего пути,  
        // только раз в секунду (программное деление на M0+ дорогое).  
        if (total_us_acc >= 1000000u) {  
            dsp_load_permille = (uint32_t)(((uint64_t)busy_us_acc * 1000u) / total_us_acc);  
            busy_us_acc  = 0;  
            total_us_acc = 0;  
        }
    }
}


// ---- Публичный API ----  

//#define RX_NCO_DISABLE

// запуск цифрового сигнального процессора (DSP) на втором ядре (Core 1)
void rx_dsp_start(float f_target_hz) {  
    // Остановка ядра
    // Принудительно гасит Core 1, исключая состояние гонки данных
    multicore_reset_core1();
    // Очистка памяти
    // Полностью обнуляет буферы фильтров промежуточной частоты 
    // и кольцевую межъядерную очередь
    memset(hb_i, 0, sizeof(hb_i));
    memset(hb_q, 0, sizeof(hb_q));
    // Раскладка: каскад 0 — короткий дешёвый фильтр (горячий вход),  
    // каскады 1+ — длинный BH на спокойных частотах.  
    for (int k = 0; k < RX_DECIM_STAGES; ++k) {  
        const int16_t* t = (k == 0) ? hb0_taps : hbx_taps;  
        uint8_t        n = (k == 0) ? HB0_NTAPS : HBX_NTAPS;  
        hb_i[k].taps = t;  hb_i[k].ntaps = n;  
        hb_q[k].taps = t;  hb_q[k].ntaps = n;  
    }
    memset(out_i, 0, sizeof(out_i));
    memset(out_q, 0, sizeof(out_q));
    out_head = out_tail = 0;

    aud_head = aud_tail = aud_wr_n = 0;    
    aud_dropped = 0;
    clip_count = 0;
  
    // сброс демодулятора, перенесённого на Core 1  
    memset(hilb_buf, 0, sizeof(hilb_buf));  
    memset(i_del,   0, sizeof(i_del));  
    hilb_idx = 0;  
    i_del_idx = 0;

    memset(cic_int_i, 0, sizeof(cic_int_i));  
    memset(cic_int_q, 0, sizeof(cic_int_q));  
    memset(cic_comb_i, 0, sizeof(cic_comb_i));  
    memset(cic_comb_q, 0, sizeof(cic_comb_q));  
    cic_cnt = 0;
    out_dropped = 0;
    dc_i = 2048 << 8;
    dc_q = 2048 << 8;
    // Инициализация гетеродина
    // Генерирует в ОЗУ таблицу синусов для быстрого смешивания частот
    nco_lut_init();
    // НЧ-фильтр
    af_lp_init();                       // коэффициенты НЧ-фильтра 3 кГц  
    memset(af_lp_buf, 0, sizeof(af_lp_buf));  
    af_lp_idx = 0;
    // Расчет шага сетки NCO
    // Вычисляет точную математическую разницу между желаемой частотой приема
    // и фактической частотой аппаратного генератора для цифровой подстройки
    float f_nco = f_target_hz - (float)vfo_quadrature_lo_hz();
    double step = 4294967296.0 * (double)f_nco / (double)RX_FS_IN_HZ;
    
    int64_t signed_step = (int64_t)step;  
    // === DIAG: отключить NCO — сигнал останется на ПЧ +3 кГц ===  
    #ifdef RX_NCO_DISABLE  
    signed_step = 0;  
    #endif  
    // ==========================================================  
    nco_step = (uint32_t)signed_step;
    nco_phase = 0;
    // вывод диагностики
    Serial.printf("[RXDBG] target=%.1f  lo_actual=%lu  f_nco=%.3f  nco_step=0x%08lX\n",  
                  (double)f_target_hz,  
                  (unsigned long)vfo_quadrature_lo_hz(),  
                  (double)f_nco,  
                  (unsigned long)nco_step);  
    
    // Старт конвейера
    // Запускает изолированный DSP-процесс rx_dsp_entry на Core 1
    multicore_launch_core1(rx_dsp_entry);  
}

// динамическое изменение частоты цифрового гетеродина (NCO) «на лету», прямо во время работы приёмника
// используется алгоритмами автоматической подстройки частоты (AFC)
void rx_dsp_set_nco_hz(float f_hz) {
    double step = 4294967296.0 * (double)f_hz / (double)RX_FS_IN_HZ;
    int64_t signed_step = (int64_t)step;
    nco_step = (uint32_t)signed_step;
}

// Функция реализует безопасное и неблокирующее извлечение отфильтрованных блоков данных из межъядерной SPSC-очереди
// Её вызывает потребитель (consumer) — управляющее ядро Core 0,
// чтобы забрать очищенный сигнал базовой полосы и передать его на вход декодерам цифровых мод
bool rx_dsp_get_output(const int16_t** i, const int16_t** q, uint8_t* n) {
    // текущий индекс чтения out_tail
    uint8_t tail = out_tail;
    // сравнение с индексом записи out_head
    // Если индексы равны, это означает, что математический движок еще не успел подготовить новый блок данных
    if (tail == out_head) return false;    // очередь пуста
      // После наблюдения нового head данные опубликованного блока
      // должны быть видимы до того, как consumer начнёт их читать
    __dmb();
    // Передача указателей на буферы данных
    *i = out_i[tail];
    *q = out_q[tail];
    *n = RX_OUT_SAMPLES;
    return true;
}

// завершает цикл межъядерного обмена данными базовой полосы
// Ее вызывает потребитель (consumer) на Core 0 строго после того,
// как текущий блок из 64 пар I/Q отсчетов был полностью прочитан и обработан декодером цифровой моды
void rx_dsp_release_output(void) {
    // Защитный барьер памяти
    // Все чтения текущего блока должны завершиться до освобождения слота
    __dmb();
    // Продвижение хвоста очереди вперед
    // Текущий буфер помечается как освобожденный. Индекс чтения out_tail сдвигается на единицу вперед
    // Операция остатка от деления % RX_OUT_BUFS (в данном случае % 4) обеспечивает
    // кольцевое зацикливание индекса: после слота 3 он автоматически сбросится в 0
    out_tail = (uint8_t)((out_tail + 1) % RX_OUT_BUFS);
}

// Счётчик потерянных выходных блоков — для диагностики захвата
// возвращает текущее значение счётчика потерянных блоков
// Для чистого спектрального измерения приращение за захват должно быть 0
// Если out_dropped увеличивается, выходной блок был отброшен, поскольку на момент проверки очередь была заполнена.
// Это может указывать на то, что Core 0 не успевает забирать блоки.
// Сам счётчик не определяет первопричину переполнения и не подтверждает отсутствие потерь на входе.
uint32_t rx_dsp_out_dropped(void) {
    return out_dropped;
    }

// Загрузка Core 1 в промилле (0..1000), обновляется раз в ~1 с.  
uint32_t rx_dsp_load_permille(void) {  
    return dsp_load_permille;  
}  
  
// Счётчик дропов аудио-очереди (переполнение aud на Core 1).    
uint32_t rx_dsp_aud_dropped(void) {    
    return aud_dropped;    
}  
  
// Счётчик срабатываний лимитера sat16 — диагностика клиппинга сверху.  
uint32_t rx_dsp_clip_count(void) {  
    return clip_count;  
}

// возвращает фактическую частоту дискретизации сигнала базовой полосы на выходе DSP-тракта
// нужна декодерам для таймингов
// RX_FS_OUT_HZ 15625 Гц при входной скорости АЦП в 250 кГц
float rx_dsp_fs_out(void) {
    return RX_FS_OUT_HZ;
    }


// Останавливает DSP-тракт: снимает core1 и гасит выходную очередь.
// Вызывается из rx off до rx_capture_stop — иначе core1 продолжит
// крутиться на освобождённых DMA-буферах.
void rx_dsp_stop(void) {
    multicore_reset_core1();   // гасим DSP-ядро
    out_head = out_tail = 0;
    out_dropped = 0;
}

// Вызывается с Core 0 при смене режима вывода USB-аудио.  
void rx_dsp_set_demod(uint8_t mode) { rxd_demod_mode = mode; }

// ---- Аудио-базбенд: читатель core0 ----  
bool rx_dsp_get_audio(const int16_t** i, const int16_t** q, uint8_t* n) {  
    uint8_t tail = aud_tail;  
    if (tail == aud_head) return false;  
    __dmb();  
    *i = aud_i[tail];  
    *q = aud_q[tail];  
    *n = RX_AUD_SAMPLES;  
    return true;  
}  
  
void rx_dsp_release_audio(void) {  
    __dmb();  
    aud_tail = (uint8_t)((aud_tail + 1) % RX_AUD_BUFS);  
}

// Число заполненных слотов очереди aud (0..RX_AUD_BUFS-1).  
// Читатель — core0; значение приблизительное (head пишет core1), но  
// для медленной подстройки ресемплера этого достаточно.  
uint8_t rx_dsp_aud_fill(void) {  
    uint8_t head = aud_head;  
    uint8_t tail = aud_tail;  
    return (uint8_t)((head - tail) % RX_AUD_BUFS);  
}
  
float rx_dsp_audio_fs(void) {  
    return RX_FS_OUT_HZ;                 // 15625.0 Гц — теперь с выхода HB-цепочки
}





