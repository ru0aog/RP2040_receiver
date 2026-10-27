// rx_rsid.cpp  
#include "rx_rsid.h"  
#include <Arduino.h>  
#include <math.h>  
#include <string.h>  
#include <complex>  
  
// Целочисленный Q15 FFT (порт RP2040_beacon/rx_spectrum.cpp) — без FPU и без g_fft.  
#define RSID_FFT_N    RSID_ARRAY_SIZE          // 2048  
#define RSID_FFT_HALF (RSID_FFT_N / 2)

// --- Локальная очередь сэмплов RSID на Core 1 ---
// Ресэмплер и обработчик RSID выполняются на одном ядре.
// Очередь отделяет приём отсчётов от пошаговой обработки FFT.
#define RSID_SAMPLE_Q_SIZE 1024u

static int16_t rsid_sample_queue[RSID_SAMPLE_Q_SIZE];
static volatile uint32_t rsid_sq_head = 0;
static volatile uint32_t rsid_sq_tail = 0;
  
static inline int16_t sat16(int32_t x) {  
    if (x > 32767) return 32767;  
    if (x < -32768) return -32768;  
    return (int16_t)x;  
}  
  
// Раздельная диагностика потерь RSID.
// sample_dropped: очередь входных отсчётов Core 0 -> Core 1.
// window_dropped: кольцо готовых окон FFT на Core 1.
static volatile uint32_t rsid_sample_dropped = 0;
static volatile uint32_t rsid_window_dropped = 0;

// Диагностика времени rsid_rx_service_sliced().
// Функция выполняется на Core 1; значения читаются диагностикой на Core 0.
static volatile uint32_t rsid_service_max_us = 0;
static volatile uint32_t rsid_service_calls_total = 0;

uint32_t rsid_rx_service_max_us(void) {return rsid_service_max_us;}
uint32_t rsid_rx_service_calls(void) {return rsid_service_calls_total;}

// Вызывать в конце каждого входа в rsid_rx_service_sliced().
static inline void rsid_service_record_time(uint32_t start_us) {
    const uint32_t elapsed = micros() - start_us;
    rsid_service_calls_total++;
    if (elapsed > rsid_service_max_us) {
        rsid_service_max_us = elapsed;
    }
}

// Старый API сохраняем: он возвращает сумму обоих счётчиков.
uint32_t rsid_rx_dropped(void) {return rsid_sample_dropped + rsid_window_dropped;}
uint32_t rsid_rx_sample_dropped(void) {return rsid_sample_dropped;}
uint32_t rsid_rx_window_dropped(void) {return rsid_window_dropped;}  
  
// Заполнение SPSC-очереди сэмплов (0..RSID_SAMPLE_Q_SIZE-1). Диагностика:    
// постоянно ~полная очередь = Core 1 не успевает забирать сэмплы.    
uint32_t rsid_rx_queue_fill(void) {    
    return (rsid_sq_head - rsid_sq_tail) % RSID_SAMPLE_Q_SIZE;    
}    
  
// ============================================================================  
//  Константы RSID (из refs/rsid.h). Вход уже 11025 Гц.  
// ============================================================================  
#define RSID_SAMPLE_RATE   11025.0  
#define RSID_FFT_SAMPLES   512  
#define RSID_FFT_SIZE      1024  
#define RSID_ARRAY_SIZE    (RSID_FFT_SIZE * 2)     // 2048  
#define RSID_BUFFER_SIZE   (RSID_ARRAY_SIZE * 2)   // 4096  
#define RSID_NSYMBOLS      15  
#define RSID_NTIMES        (RSID_NSYMBOLS * 2)      // 30  
#define RSID_ESCAPE        6  
#define RSID_NONE2         0   // код-терминатор набора 2 (как RSID_NONE2 в fldigi)  
  
// Сколько окон поиска держать активным набор 2 после приёма ESCAPE.  
// fldigi: 27 * RSID_SYMLEN отсчётов (10 gap + 15 символов + 2 на ошибки тайминга);  
// при шаге окна RSID_FFT_SAMPLES (512) это ~27 окон. Берём с запасом.  
#define RSID_SECONDARY_WINDOWS   60
  
// ============================================================================  
//  Таблицы мод RSID (code -> имя).  
//   rsid_ids_1 — основной (primary) набор из refs/rsid_defs.cxx.  
//   rsid_ids_2 — расширенный (extended) набор, доступен после ESCAPE (код 6).  
//     Здесь — только IFKP (1171); при необходимости дополняется по RSID_LIST2.  
// ============================================================================  
struct RSIDmap { unsigned short rs; const char* name; };  
  
#define RSID_LIST \
    ELEM_(263, "EOT")            ELEM_(6,   "ESCAPE")        \
    ELEM_(1,   "BPSK31")         ELEM_(110, "QPSK31")        \
    ELEM_(2,   "BPSK63")         ELEM_(3,   "QPSK63")        \
    ELEM_(4,   "BPSK125")        ELEM_(5,   "QPSK125")       \
    ELEM_(126, "BPSK250")        ELEM_(127, "QPSK250")       \
    ELEM_(173, "BPSK500")                                    \
    ELEM_(183, "PSK125R")        ELEM_(186, "PSK250R")       \
    ELEM_(187, "PSK500R")                                    \
    ELEM_(9,  "MT63-500L")       ELEM_(10, "MT63-500S")      \
    ELEM_(12, "MT63-1000L")      ELEM_(13, "MT63-1000S")     \
    ELEM_(15, "MT63-2000L")      ELEM_(17, "MT63-2000S")     \
    ELEM_(22, "PSK63F")          ELEM_(26, "CW")             \
    ELEM_(37, "RTTY-ASCII7")     ELEM_(38, "RTTY-ASCII8")    \
    ELEM_(39, "RTTY-45")         ELEM_(40, "RTTY-50")        \
    ELEM_(41, "RTTY-75")                                     \
    ELEM_(43, "THROB1")          ELEM_(44, "THROB2")         \
    ELEM_(45, "THROB4")                                      \
    ELEM_(204,"CONT-4/125")      ELEM_(55, "CONT-4/250")     \
    ELEM_(54, "CONT-4/500")      ELEM_(169,"CONT-8/125")     \
    ELEM_(49, "CONT-8/250")      ELEM_(52, "CONT-8/500")     \
    ELEM_(50, "CONT-16/500")     ELEM_(53, "CONT-16/1000")   \
    ELEM_(51, "CONT-32/1000")                                \
    ELEM_(60, "MFSK8")           ELEM_(57, "MFSK16")         \
    ELEM_(147,"MFSK32")          ELEM_(148,"MFSK11")         \
    ELEM_(152,"MFSK22")                                      \
    ELEM_(203,"OLIVIA-4/125")    ELEM_(75, "OLIVIA-4/250")   \
    ELEM_(74, "OLIVIA-4/500")    ELEM_(163,"OLIVIA-8/125")   \
    ELEM_(69, "OLIVIA-8/250")    ELEM_(72, "OLIVIA-8/500")   \
    ELEM_(70, "OLIVIA-16/500")   ELEM_(73, "OLIVIA-16/1000") \
    ELEM_(84, "DOMEX4")          ELEM_(85, "DOMEX5")         \
    ELEM_(86, "DOMEX8")          ELEM_(87, "DOMEX11")        \
    ELEM_(88, "DOMEX16")         ELEM_(90, "DOMEX22")        \
    ELEM_(97, "DOMEX8-FEC")                                  \
    ELEM_(104,"FELD-HELL")       ELEM_(106,"HELL80")         \
    ELEM_(136,"THOR4")           ELEM_(137,"THOR8")          \
    ELEM_(138,"THOR16")          ELEM_(139,"THOR5")          \
    ELEM_(143,"THOR11")          ELEM_(145,"THOR22")         \
    ELEM_(0,   "NONE")  
  
#define ELEM_(code_, name_) { code_, name_ },  
static const RSIDmap rsid_ids_1[] = { RSID_LIST };  
#undef ELEM_  
static const int rsid_ids_size1 =  
    (int)(sizeof(rsid_ids_1) / sizeof(rsid_ids_1[0])) - 1;  // без NONE  
  
// Расширенный набор. Терминатор должен иметь код RSID_NONE2 (0).  
#define RSID_LIST2 \
    ELEM2_(1171, "IFKP")  \
    ELEM2_(0,    "NONE2")  
  
#define ELEM2_(code_, name_) { code_, name_ },  
static const RSIDmap rsid_ids_2[] = { RSID_LIST2 };  
#undef ELEM2_  
static const int rsid_ids_size2 =  
    (int)(sizeof(rsid_ids_2) / sizeof(rsid_ids_2[0])) - 1;  // без NONE2  
  
// ============================================================================  
//  Reed-Solomon кодер (refs/rsid.cxx: Squares/indices/Encode).  
// ============================================================================  
static const int Squares[256] = {  
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,  
    0, 2, 4, 6, 8,10,12,14, 9,11,13,15, 1, 3, 5, 7,  
    0, 3, 6, 5,12,15,10, 9, 1, 2, 7, 4,13,14,11, 8,  
    0, 4, 8,12, 9,13, 1, 5,11,15, 3, 7, 2, 6,10,14,  
    0, 5,10,15,13, 8, 7, 2, 3, 6, 9,12,14,11, 4, 1,  
    0, 6,12,10, 1, 7,13,11, 2, 4,14, 8, 3, 5,15, 9,  
    0, 7,14, 9, 5, 2,11,12,10,13, 4, 3,15, 8, 1, 6,  
    0, 8, 9, 1,11, 3, 2,10,15, 7, 6,14, 4,12,13, 5,  
    0, 9,11, 2,15, 6, 4,13, 7,14,12, 5, 8, 1, 3,10,  
    0,10,13, 7, 3, 9,14, 4, 6,12,11, 1, 5,15, 8, 2,  
    0,11,15, 4, 7,12, 8, 3,14, 5, 1,10, 9, 2, 6,13,  
    0,12, 1,13, 2,14, 3,15, 4, 8, 5, 9, 6,10, 7,11,  
    0,13, 3,14, 6,11, 5, 8,12, 1,15, 2,10, 7, 9, 4,  
    0,14, 5,11,10, 4,15, 1,13, 3, 8, 6, 7, 9, 2,12,  
    0,15, 7, 8,14, 1, 9, 6, 5,10, 2,13,11, 4,12, 3  
};  
static const int indices[12] = { 2, 4, 8, 9, 11, 15, 7, 14, 5, 10, 13, 3 };  
  
static void rsid_encode(int code, unsigned char* rsid /*RSID_NSYMBOLS*/) {  
    rsid[0] = code >> 8;  
    rsid[1] = (code >> 4) & 0x0f;  
    rsid[2] = code & 0x0f;  
    for (int i = 3; i < RSID_NSYMBOLS; i++) rsid[i] = 0;  
    for (int i = 0; i < 12; i++) {  
        for (int j = RSID_NSYMBOLS - 1; j > 0; j--)  
            rsid[j] = rsid[j - 1] ^ Squares[(rsid[j] << 4) + indices[i]];  
        rsid[0] = Squares[(rsid[0] << 4) + indices[i]];  
    }  
}  


// --- Переменные асинхронной стейт-машины RSID ---
enum rsid_state_t {
    RSID_STATE_IDLE = 0,
    RSID_STATE_WINDOWING,
    RSID_STATE_FFT_BITREVERSE,
    RSID_STATE_FFT_STAGE,
    RSID_STATE_CALC_AMPL,
    RSID_STATE_SINK_AND_BUCKETS,
    RSID_STATE_SEARCH
};

static rsid_state_t rsid_sm_state = RSID_STATE_IDLE;

// Текущее состояние стейт-машины (0=IDLE..6=SEARCH). Диагностика:    
// если застряло не в IDLE — конвейер не доходит до поиска.    
uint32_t rsid_rx_sm_state(void) { return (uint32_t)rsid_sm_state; } 

// Переменные итераторов для сохранения контекста между вызовами  
static uint16_t fft_len = 2;  
static uint16_t fft_base = 0;  
  
// Бюджет времени на один заход rsid_rx_service_sliced() (мкс).  
// Несколько коротких квантов, но суммарно < интервала ADC-блока (4.096 мс).  
#ifndef RSID_SLICE_BUDGET_US  
#define RSID_SLICE_BUDGET_US 120u  
#endif  
  
// --- Итераторы нарезанного поиска (один код таблицы за квант) ---  
static bool srch_active    = false;  
static int  srch_i         = 0;  
static int  srch_best_dist = 1000;  
static int  srch_best_bin  = -1;  
static int  srch_best_sym  = -1;

// ============================================================================  
//  Состояние детектора  
// ============================================================================  
static bool           rsid_inited = false;  
  
static unsigned char  pCodes1[sizeof(rsid_ids_1) / sizeof(rsid_ids_1[0]) * RSID_NSYMBOLS];  
static unsigned char  pCodes2[sizeof(rsid_ids_2) / sizeof(rsid_ids_2[0]) * RSID_NSYMBOLS];  
  
static int16_t        aInputSamples[RSID_BUFFER_SIZE];
static int16_t        window_q15[RSID_FFT_N];  
static int16_t        tw_re[RSID_FFT_HALF];  
static int16_t        tw_im[RSID_FFT_HALF];  
static int16_t        fft_i[RSID_FFT_N];  
static int16_t        fft_q[RSID_FFT_N];  
static uint64_t       aFFTAmpl[RSID_FFT_SIZE];  
static int16_t        fft_buckets[RSID_NTIMES][RSID_FFT_SIZE];  
  
// Кольцо снимков окон: приём (ФАЗА 0) пишет head, стейт-машина читает tail.  
// Оба на Core 1 и работают последовательно, поэтому volatile/__dmb не нужны.  
#define RSID_WIN_SLOTS 4  
static int16_t rsid_proc[RSID_WIN_SLOTS][RSID_ARRAY_SIZE];  
static uint8_t rsid_win_head = 0;   // куда класть следующий снимок  
static uint8_t rsid_win_tail = 0;   // какой снимок обрабатывать  
static uint8_t rsid_cur_slot = 0;   // слот, обрабатываемый прямо сейчас
  
static int   inptr;  
static int   nBinLow, nBinHigh;  
static int   hamming_resolution = RSID_HAMMING_RESOLUTION;  
  
// Счётчик окон, в течение которых после ESCAPE ищем во втором наборе.  
static int   rsid_secondary_windows = 0;  
  
static rsid_detect_cb_t s_cb = nullptr;  

// --- Почтовый ящик детекта: пишется на Core 1, читается на Core 0.  
static volatile bool rsid_det_pending = false;  
static rsid_result_t rsid_det_result;

// ============================================================================  
//  Окно Хэмминга (refs/rsid.cxx -> filters.h hamming()).  
// ============================================================================  
static inline double rsid_hamming(double x) {  
    return 0.54 - 0.46 * cos(2.0 * M_PI * x);  
}  
  
// Сброс ТОЛЬКО детекторного состояния (владелец — Core 1). НЕ трогает  
// накопитель aInputSamples/inptr, которым владеет Core 0.  
static void rsid_reset_detect(void) {  
    memset(aFFTAmpl,    0, sizeof(aFFTAmpl));  
    memset(fft_buckets, 0, sizeof(fft_buckets));  
    hamming_resolution = RSID_HAMMING_RESOLUTION;  
    rsid_secondary_windows = 0;  
    srch_i = 0;  
    srch_active = false;                 // чистый старт поиска  
    rsid_sm_state = RSID_STATE_IDLE;  
    // НЕ трогаем rsid_win_head/tail/cur_slot здесь:  
    // эта функция зовётся из SEARCH до освобождения слота,  
    // сброс индексов разорвёт учёт кольца.
}
  
// Полный сброс (только с Core 0: init / retune) — сбрасывает и накопитель.  
static void rsid_reset_state(void) {  
    inptr = RSID_FFT_SIZE;  
    memset(aInputSamples, 0, sizeof(aInputSamples));
    rsid_reset_detect();
}
  
// ----------------------------------------------------------------------------  
//  CalculateBuckets (refs/rsid.cxx, без изменений логики).  
// ----------------------------------------------------------------------------  
static void CalculateBuckets(const uint64_t* pSpectrum, int iBegin, int iEnd) {  
    uint64_t Amp = 0, AmpMax = 0;  
    int iBucketMax = iBegin - 2;  
    int j;  
    for (int i = iBegin; i < iEnd; i += 2) {  
        if (iBucketMax == i - 2) {  
            AmpMax = pSpectrum[i];  
            iBucketMax = i;  
            for (j = i + 2; j < i + RSID_NTIMES + 2; j += 2) {  
                Amp = pSpectrum[j];  
                if (Amp > AmpMax) { AmpMax = Amp; iBucketMax = j; }  
            }  
        } else {  
            j = i + RSID_NTIMES;  
            Amp = pSpectrum[j];  
            if (Amp > AmpMax) { AmpMax = Amp; iBucketMax = j; }  
        }  
        fft_buckets[RSID_NTIMES - 1][i] = (int16_t)((iBucketMax - i) >> 1);  
    }  
}  
  
// ----------------------------------------------------------------------------  
//  HammingDistance / search_amp (refs/rsid.cxx).  
// ----------------------------------------------------------------------------  
static inline int HammingDistance(int iBucket, const unsigned char* p2) {  
    int dist = 0;  
    for (int i = 0, j = 1; i < RSID_NSYMBOLS; i++, j += 2) {  
        if (fft_buckets[j][iBucket] != p2[i]) {  
            if (++dist > hamming_resolution) break;  
        }  
    }  
    return dist;  
}  

// Пошаговый поиск: обрабатывает ОДИН код таблицы за вызов (один квант).  
// Возвращает true, когда перебор всей таблицы завершён; результат — в srch_best_*.  
static bool search_amp_step(const unsigned char* pCodes,  
                            const RSIDmap* ids, int ids_size) {  
    if (!srch_active) {               // старт нового перебора  
        srch_i = 0;  
        srch_best_dist = 1000;  
        srch_best_bin  = -1;  
        srch_best_sym  = -1;  
        srch_active = true;  
    }  
  
    const unsigned char* pc = pCodes + srch_i * RSID_NSYMBOLS;  
    for (int j = nBinLow; j < nBinHigh - RSID_NTIMES; j++) {  
        int d = HammingDistance(j, pc);  
        if (d < srch_best_dist) {  
            srch_best_dist = d;  
            srch_best_sym  = ids[srch_i].rs;  
            srch_best_bin  = j;  
            if (srch_best_dist == 0) break;  
        }  
    }  
  
    srch_i++;  
    if (srch_i >= ids_size || srch_best_dist == 0) {  
        srch_active = false;          // перебор завершён  
        return true;  
    }  
    return false;                     // остались ещё коды  
}

// ----------------------------------------------------------------------------  
//  apply: выдать (имя моды, частоту) из заданной таблицы.  
// ----------------------------------------------------------------------------  
static void rsid_apply(int iBin, int iSymbol,  
                       const RSIDmap* ids, int ids_size) {  
    double rsidfreq = (iBin + RSID_NSYMBOLS - 0.5) * RSID_SAMPLE_RATE / 2048.0;  
  
    const char* name = nullptr;  
    for (int n = 0; n < ids_size; n++) {  
        if (ids[n].rs == iSymbol) { name = ids[n].name; break; }  
    }  
    if (!name) return;   // неизвестный код  
  
    rsid_result_t r;  
    r.mode_code = iSymbol;  
    r.mode_name = name;  
    r.freq_hz   = (float)rsidfreq;  
  
    // Диспетчеризацию (Serial + смена моды) делает Core 0 в rsid_rx_dispatch().  
    rsid_det_result = r;  
    __dmb();  
    rsid_det_pending = true;
}  
  
// ----------------------------------------------------------------------------  
//  Центр RTTY берётся из частоты RSID в момент декодирования.  
// ----------------------------------------------------------------------------  
static float         rtty_center_hz = 0.0f;  
static volatile bool rtty_center_new = false;  
  
// ============================================================================  
//  Публичный интерфейс (rx_rsid.h).  
// ============================================================================  
void rsid_rx_init(float fs_hz) {  
    (void)fs_hz;  
  
    for (int i = 0; i < RSID_FFT_N; i++)  
        window_q15[i] = sat16((int32_t)(rsid_hamming(1.0 * i / RSID_FFT_N) * 32767.0 + 0.5));  
  
    for (int k = 0; k < RSID_FFT_HALF; k++) {  
        const double a = -6.2831853071795864769 * (double)k / (double)RSID_FFT_N;  
        tw_re[k] = sat16((int32_t)(cos(a) * 32767.0));  
        tw_im[k] = sat16((int32_t)(sin(a) * 32767.0));  
    }  
  
    memset(pCodes1, 0, sizeof(pCodes1));  
    for (int i = 0; i < rsid_ids_size1; i++)  
        rsid_encode(rsid_ids_1[i].rs, pCodes1 + i * RSID_NSYMBOLS);  
  
    memset(pCodes2, 0, sizeof(pCodes2));
    for (int i = 0; i < rsid_ids_size2; i++)
        rsid_encode(rsid_ids_2[i].rs, pCodes2 + i * RSID_NSYMBOLS);

    // Сброс очереди входных отсчётов.
    rsid_sq_head = 0;
    rsid_sq_tail = 0;

    // Сброс кольца готовых FFT-окон.
    rsid_win_head = 0;
    rsid_win_tail = 0;
    rsid_cur_slot = 0;

    // Сброс диагностических счётчиков потерь.
    rsid_sample_dropped = 0;
    rsid_window_dropped = 0;

    rsid_service_max_us = 0;
    rsid_service_calls_total = 0;

    rsid_reset_state();
    rsid_inited = true;
}  
  
void rsid_rx_set_callback(rsid_detect_cb_t cb) { s_cb = cb; }  
  
void rsid_rx_reset(void) { rsid_reset_state(); }  

// Принимает отсчёт 11025 Гц от ресэмплера на Core 1.
// При заполнении локальной очереди увеличивает счётчик потерь.
void rsid_rx_push(int16_t sample) {
    if (!rsid_inited) return;

    uint32_t head = rsid_sq_head;
    uint32_t next = (head + 1u) % RSID_SAMPLE_Q_SIZE;

    if (next != rsid_sq_tail) {
        rsid_sample_queue[head] = sample;
        __dmb(); // Гарантируем запись сэмпла до сдвига head
        rsid_sq_head = next;
    } else {
        // Очередь входных отсчётов заполнена:
        // Core 1 не успевает забирать данные от Core 0.
        rsid_sample_dropped++;
    }
}

// Вызывается из loop() (Core 0): забирает результат детекта, сделанного на Core 1,  
// и выполняет Serial-печать + смену моды в безопасном (не real-time) контексте.  
void rsid_rx_dispatch(void) {  
    if (!rsid_det_pending) return;  
    rsid_det_pending = false;  
    __dmb();  
    if (s_cb) s_cb(&rsid_det_result);  
    else Serial.printf("# RSID mode=%s code=%d freq=%.1f\n",  
                       rsid_det_result.mode_name,  
                       rsid_det_result.mode_code,  
                       rsid_det_result.freq_hz);  
}

  
bool rsid_get_rtty_center(float* f_hz) {  
    if (!rtty_center_new) return false;  
    rtty_center_new = false;  
    if (f_hz) *f_hz = rtty_center_hz;  
    return true;  
}


void rsid_rx_service_sliced(void) {
    const uint32_t t_service_start = micros();

    if (!rsid_inited) {
        rsid_service_record_time(t_service_start);
        return;
    }

    // --- ФАЗА 0: Попутный сбор сэмплов из Lock-Free шлюза (всегда на полной скорости) ---
    while (rsid_sq_tail != rsid_sq_head) {
        uint32_t tail = rsid_sq_tail;
        int16_t sample = rsid_sample_queue[tail];
        
        rsid_sq_tail = (tail + 1u) % RSID_SAMPLE_Q_SIZE;
        __dmb();

        aInputSamples[inptr++] = sample;
        if (inptr >= RSID_ARRAY_SIZE) {  
            uint8_t nh = (uint8_t)((rsid_win_head + 1) % RSID_WIN_SLOTS);  
            if (nh != rsid_win_tail) {                 // в кольце есть место  
                memcpy(rsid_proc[rsid_win_head], aInputSamples,  
                    RSID_ARRAY_SIZE * sizeof(int16_t));  
                rsid_win_head = nh;  
            } else {
                // Все слоты кольца FFT-окон заняты.
                // Обработчик RSID не успевает освобождать готовые окна.
                rsid_window_dropped++;
            }
            memmove(&aInputSamples[0], &aInputSamples[RSID_FFT_SAMPLES],  
                    (RSID_BUFFER_SIZE - RSID_FFT_SAMPLES) * sizeof(int16_t));  
            inptr -= RSID_FFT_SAMPLES;  
        }
    }

    // --- ФАЗА 1: Выполняем кванты стейт-машины в пределах таймбюджета ---  
    // Несколько лёгких квантов за один заход «!blk», но суммарно ограничиваем  
    // время, чтобы не задержать Core 1 и не уронить аудио. Каждый квант сам по  
    // себе короткий (FFT_STAGE — одна группа бабочек, SEARCH — один код).  
    const uint32_t t_slice_start = micros();  
    do {  
        // Делать нечего — выходим немедленно.  
        if (rsid_sm_state == RSID_STATE_IDLE && rsid_win_tail == rsid_win_head) break;
  
    switch (rsid_sm_state) {  
          
        case RSID_STATE_IDLE:  
        if (rsid_win_tail != rsid_win_head) {       // есть готовое окно  
            rsid_cur_slot = rsid_win_tail;          // фиксируем слот на весь конвейер  
            rsid_sm_state = RSID_STATE_WINDOWING;  
        }  
        break;

        case RSID_STATE_WINDOWING:  
        nBinLow  = 3;  
        nBinHigh = RSID_FFT_SIZE - 32;  
        for (int i = 0; i < RSID_FFT_N; i++) {  
            int32_t xi = (int32_t)rsid_proc[rsid_cur_slot][i];  
            fft_i[i] = (int16_t)(((int32_t)xi * window_q15[i]) >> 15);  
            fft_q[i] = 0;  
        }  
        rsid_sm_state = RSID_STATE_FFT_BITREVERSE;  
        break;

        case RSID_STATE_FFT_BITREVERSE: {
            uint16_t j = 0;  
            for (uint16_t i = 1; i < RSID_FFT_N; ++i) {  
                uint16_t bit = RSID_FFT_HALF;  
                while (j & bit) { j ^= bit; bit >>= 1; }  
                j ^= bit;  
                if (i < j) {  
                    int16_t t;  
                    t = fft_i[i]; fft_i[i] = fft_i[j]; fft_i[j] = t;  
                    t = fft_q[i]; fft_q[i] = fft_q[j]; fft_q[j] = t;  
                }  
            }
            fft_len = 2; // Инициализируем итераторы для стадий БПФ
            fft_base = 0;
            rsid_sm_state = RSID_STATE_FFT_STAGE;
            break;
        }

        case RSID_STATE_FFT_STAGE: {
            // Считаем только ОДНУ базовую группу бабочек за один вызов функции
            const uint16_t half = fft_len >> 1;  
            const uint16_t tw_step = RSID_FFT_N / fft_len;  
            
            for (uint16_t j2 = 0; j2 < half; ++j2) {  
                const uint16_t tw = j2 * tw_step;  
                const int32_t wr = tw_re[tw];  
                const int32_t wi = tw_im[tw];  
                const uint16_t a = fft_base + j2;  
                const uint16_t b = a + half;  
                const int32_t br = fft_i[b], bi = fft_q[b];  
                const int32_t tr = (br * wr - bi * wi) >> 15;  
                const int32_t ti = (br * wi + bi * wr) >> 15;  
                const int32_t ar = fft_i[a], ai = fft_q[a];  
                fft_i[a] = sat16((ar + tr) >> 1);  
                fft_q[a] = sat16((ai + ti) >> 1);  
                fft_i[b] = sat16((ar - tr) >> 1);  
                fft_q[b] = sat16((ai - ti) >> 1);  
            }  

            fft_base += fft_len;
            if (fft_base >= RSID_FFT_N) {
                fft_base = 0;
                fft_len <<= 1;
                if (fft_len > RSID_FFT_N) {
                    rsid_sm_state = RSID_STATE_CALC_AMPL;
                }
            }
            break;
        }

        case RSID_STATE_CALC_AMPL:
            for (int i = 0; i < RSID_FFT_SIZE; i++) {  
                const int32_t re = fft_i[i];  
                const int32_t im = fft_q[i];  
                aFFTAmpl[i] = (uint64_t)((int64_t)re * re + (int64_t)im * im);  
            }
            rsid_sm_state = RSID_STATE_SINK_AND_BUCKETS;
            break;

        case RSID_STATE_SINK_AND_BUCKETS: {
            //if (s_fft_sink) {
            //    s_fft_sink(aFFTAmpl, RSID_FFT_SIZE, (float)(RSID_SAMPLE_RATE / 2048.0));
            //}

            int bucket_low  = 3;  
            int bucket_high = RSID_FFT_SIZE - 32;  
          
            memmove(fft_buckets, &(fft_buckets[1][0]),  
                    (RSID_NTIMES - 1) * RSID_FFT_SIZE * sizeof(fft_buckets[0][0]));  
            memset(&(fft_buckets[RSID_NTIMES - 1][0]), 0, RSID_FFT_SIZE * sizeof(fft_buckets[0][0]));  
          
            CalculateBuckets(aFFTAmpl, bucket_low,     bucket_high - RSID_NTIMES);  
            CalculateBuckets(aFFTAmpl, bucket_low + 1, bucket_high - RSID_NTIMES);  
            
            rsid_sm_state = RSID_STATE_SEARCH;
            break;
        }

        case RSID_STATE_SEARCH: {  
            // Выбор таблицы фиксирован на всё время перебора окна  
            // (rsid_secondary_windows меняется только после завершения поиска).  
            const bool            primary  = (rsid_secondary_windows <= 0);  
            const unsigned char*  pCodes   = primary ? pCodes1       : pCodes2;  
            const RSIDmap*        ids      = primary ? rsid_ids_1     : rsid_ids_2;  
            const int             ids_size = primary ? rsid_ids_size1 : rsid_ids_size2;  
  
            // ОДИН код таблицы за квант. Пока не завершено — остаёмся в SEARCH.  
            if (!search_amp_step(pCodes, ids, ids_size)) {  
                break;  
            }  
  
            // Перебор завершён — оцениваем лучший результат.  
            const bool found = (srch_best_dist <= hamming_resolution);  
            const int  bin   = srch_best_bin;  
            const int  sym   = srch_best_sym;  
  
            if (primary) {  
                if (found) {  
                    if (sym != RSID_ESCAPE) {  
                        rsid_apply(bin, sym, rsid_ids_1, rsid_ids_size1);  
                        rtty_center_hz = (float)((bin + RSID_NSYMBOLS - 0.5) * RSID_SAMPLE_RATE / 2048.0);  
                        __dmb();  
                        rtty_center_new = true;  
                        rsid_reset_detect();  
                    } else {  
                        rsid_secondary_windows = RSID_SECONDARY_WINDOWS;  
                    }  
                }  
            } else {  
                rsid_secondary_windows--;  
                if (found) {  
                    if (sym != RSID_NONE2) {  
                        rsid_apply(bin, sym, rsid_ids_2, rsid_ids_size2);  
                        rsid_reset_detect();  
                    }  
                }  
                if (rsid_secondary_windows <= 0) {  
                    rsid_secondary_windows = 0;  
                }  
            }  
  
            // Слот освобождаем ЗДЕСЬ, до любого reset внутри обработчиков выше —  
            // reset сбрасывает rsid_sm_state в IDLE, но не трогает индексы кольца.  
            rsid_win_tail = (uint8_t)((rsid_win_tail + 1) % RSID_WIN_SLOTS);  
            rsid_sm_state = RSID_STATE_IDLE;  
            break;
        }
    }  
    } while ((uint32_t)(micros() - t_slice_start) < RSID_SLICE_BUDGET_US);

    // Измеряем полное время вызова, включая сбор входных сэмплов.
    rsid_service_record_time(t_service_start);
}

