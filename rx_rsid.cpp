// rx_rsid.cpp  
#include "rx_rsid.h"  
#include <Arduino.h>  
#include <math.h>  
#include <string.h>  
#include <complex>  
  
// Целочисленный Q15 FFT (порт RP2040_beacon/rx_spectrum.cpp) — без FPU и без g_fft.  
#define RSID_FFT_N    RSID_ARRAY_SIZE          // 2048  
#define RSID_FFT_HALF (RSID_FFT_N / 2)  
  
static inline int16_t sat16(int32_t x) {  
    if (x > 32767) return 32767;  
    if (x < -32768) return -32768;  
    return (int16_t)x;  
}  
  
static volatile uint32_t rsid_dropped_windows = 0;  
uint32_t rsid_rx_dropped(void) { return rsid_dropped_windows; }  
  
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
  
// float вместо double — экономия ОЗУ на RP2040.  
typedef float                       rs_fft_type;  
typedef std::complex<rs_fft_type>   rs_cpx_type;  
  
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

// --- Межъядерный шлюз для сэмплов 11025 Гц ---
#define RSID_SAMPLE_Q_SIZE 1024u
static int16_t rsid_sample_queue[RSID_SAMPLE_Q_SIZE];
static volatile uint32_t rsid_sq_head = 0; // Пишет Core 0
static volatile uint32_t rsid_sq_tail = 0; // Читает Core 1

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

// Переменные итераторов для сохранения контекста между вызовами
static uint16_t fft_len = 2;
static uint16_t fft_base = 0;


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
  
static int16_t        rsid_proc[RSID_ARRAY_SIZE];  
static volatile bool  rsid_window_ready = false;  
  
static int   inptr;  
static int   nBinLow, nBinHigh;  
static int   hamming_resolution = RSID_HAMMING_RESOLUTION;  
  
// Счётчик окон, в течение которых после ESCAPE ищем во втором наборе.  
static int   rsid_secondary_windows = 0;  
  
static rsid_detect_cb_t s_cb = nullptr;  
static rsid_fft_sink_t  s_fft_sink = nullptr;  
void rsid_rx_set_fft_sink(rsid_fft_sink_t sink) { s_fft_sink = sink; }

// --- Почтовый ящик детекта: пишется на Core 1 (rsid_search), читается на Core 0.  
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
}  
  
// Полный сброс (только с Core 0: init / retune) — сбрасывает и накопитель.  
static void rsid_reset_state(void) {  
    inptr = RSID_FFT_SIZE;  
    memset(aInputSamples, 0, sizeof(aInputSamples));  
    rsid_reset_detect();  
}
  
// In-place radix-2 DIT, деление на 2 на каждой стадии (значения в Q15).  
static void fft_q15(void) {  
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
    for (uint16_t len = 2; len <= RSID_FFT_N; len <<= 1) {  
        const uint16_t half = len >> 1;  
        const uint16_t tw_step = RSID_FFT_N / len;  
        for (uint16_t base = 0; base < RSID_FFT_N; base += len) {  
            for (uint16_t j2 = 0; j2 < half; ++j2) {  
                const uint16_t tw = j2 * tw_step;  
                const int32_t wr = tw_re[tw];  
                const int32_t wi = tw_im[tw];  
                const uint16_t a = base + j2;  
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
        }  
    }  
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
  
// Параметризованный поиск: по заданной таблице кодов pCodes и её ids/size.  
static bool search_amp(int& bin_out, int& symbol_out,  
                       const unsigned char* pCodes,  
                       const RSIDmap* ids, int ids_size) {  
    int iDistanceMin = 1000;  
    int iBin = -1, iSymbol = -1;  
    for (int i = 0; i < ids_size; i++) {  
        const unsigned char* pc = pCodes + i * RSID_NSYMBOLS;  
        for (int j = nBinLow; j < nBinHigh - RSID_NTIMES; j++) {  
            int d = HammingDistance(j, pc);  
            if (d < iDistanceMin) {  
                iDistanceMin = d;  
                iSymbol = ids[i].rs;  
                iBin = j;  
                if (iDistanceMin == 0) break;  
            }  
        }  
    }  
    if (iDistanceMin <= hamming_resolution) {  
        symbol_out = iSymbol;  
        bin_out    = iBin;  
        return true;  
    }  
    return false;  
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
  
// ----------------------------------------------------------------------------  
//  search (refs/rsid.cxx): FFT окна 2048 -> bucket'ы -> поиск кода.  
//  bReverse = false (USB, тон в положительной звуковой полосе).  
// ----------------------------------------------------------------------------  
static void rsid_search(void) {  
    nBinLow  = 3;  
    nBinHigh = RSID_FFT_SIZE - 32;  
  
    // Оконирование Q15: вход (int16) * окно (Q15) >> 15. Мнимая часть = 0.  
    for (int i = 0; i < RSID_FFT_N; i++) {  
        int32_t xi = (int32_t)rsid_proc[i];  
        fft_i[i] = (int16_t)(((int32_t)xi * window_q15[i]) >> 15);  
        fft_q[i] = 0;  
    }  
  
    fft_q15();  
  
    for (int i = 0; i < RSID_FFT_SIZE; i++) {  
        const int32_t re = fft_i[i];  
        const int32_t im = fft_q[i];  
        aFFTAmpl[i] = (uint64_t)((int64_t)re * re + (int64_t)im * im);  
    }  

    if (s_fft_sink)  
        s_fft_sink(aFFTAmpl, RSID_FFT_SIZE, (float)(RSID_SAMPLE_RATE / 2048.0));

    int bucket_low  = 3;  
    int bucket_high = RSID_FFT_SIZE - 32;  
  
    memmove(fft_buckets, &(fft_buckets[1][0]),  
            (RSID_NTIMES - 1) * RSID_FFT_SIZE * sizeof(fft_buckets[0][0]));  
    memset(&(fft_buckets[RSID_NTIMES - 1][0]), 0, RSID_FFT_SIZE * sizeof(fft_buckets[0][0]));  
  
    CalculateBuckets(aFFTAmpl, bucket_low,     bucket_high - RSID_NTIMES);  
    CalculateBuckets(aFFTAmpl, bucket_low + 1, bucket_high - RSID_NTIMES);  
  
    int sym = -1, bin = -1;  
  
    // ---- Набор 1 (primary), пока не активен приём набора 2 ----  
    if (rsid_secondary_windows <= 0) {  
        if (search_amp(bin, sym, pCodes1, rsid_ids_1, rsid_ids_size1)) {  
            if (sym != RSID_ESCAPE) {  
                rsid_apply(bin, sym, rsid_ids_1, rsid_ids_size1);  
  
                // Центр матч-фильтра RTTY — прямо из частоты RSID.  
                rtty_center_hz  = (float)((bin + RSID_NSYMBOLS - 0.5)  
                                          * RSID_SAMPLE_RATE / 2048.0);  
                __dmb();  
                rtty_center_new = true;
  
                rsid_reset_detect(); 
            } else {  
                // ESCAPE: следующие окна ищем в расширенном наборе.  
                rsid_secondary_windows = RSID_SECONDARY_WINDOWS;  
            }  
        }  
        return;  
    }  
  
    // ---- Набор 2 (extended), активен RSID_SECONDARY_WINDOWS окон ----  
    rsid_secondary_windows--;  
    if (search_amp(bin, sym, pCodes2, rsid_ids_2, rsid_ids_size2)) {  
        if (sym != RSID_NONE2) {  
            rsid_apply(bin, sym, rsid_ids_2, rsid_ids_size2);  
  
            // Частота известна и для расширенных мод; RTTY-центр  
            // для не-RTTY мод не задаём (routing по моде в rx_decoders).  
            rsid_reset_detect();   // сбросит и rsid_secondary_windows  
        }  
    }  
    if (rsid_secondary_windows <= 0) {  
        // Таймаут истёк — возвращаемся к поиску набора 1.  
        rsid_secondary_windows = 0;  
    }  
}  
  
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
  
    rsid_reset_state();  
    rsid_inited = true;  
}  
  
void rsid_rx_set_callback(rsid_detect_cb_t cb) { s_cb = cb; }  
  
void rsid_rx_reset(void) { rsid_reset_state(); }  
  
void rsid_rx_push(int16_t sample) {
    if (!rsid_inited) return;

    uint32_t head = rsid_sq_head;
    uint32_t next = (head + 1u) % RSID_SAMPLE_Q_SIZE;

    if (next != rsid_sq_tail) {
        rsid_sample_queue[head] = sample;
        __dmb(); // Гарантируем запись сэмпла до сдвига head
        rsid_sq_head = next;
    } else {
        // Очередь переполнена (Core 1 слишком долго считал БПФ)
        rsid_dropped_windows++; 
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

void rsid_rx_service(void) {
    if (!rsid_inited) return;

    // 1. Выгребаем сэмплы из межъядерной очереди во внутренний буфер БПФ
    while (rsid_sq_tail != rsid_sq_head) {
        uint32_t tail = rsid_sq_tail;
        int16_t sample = rsid_sample_queue[tail];
        
        rsid_sq_tail = (tail + 1u) % RSID_SAMPLE_Q_SIZE;
        __dmb();

        // Старая логика накопления окна, но теперь она безопасно выполняется на Core 1
        aInputSamples[inptr++] = sample;
        if (inptr >= RSID_ARRAY_SIZE) {
            if (!rsid_window_ready) {
                memcpy(rsid_proc, aInputSamples, RSID_ARRAY_SIZE * sizeof(int16_t));
                rsid_window_ready = true;
            } else { 
                rsid_dropped_windows++; 
            }
            memmove(&aInputSamples[0], &aInputSamples[RSID_FFT_SAMPLES],  
                    (RSID_BUFFER_SIZE - RSID_FFT_SAMPLES) * sizeof(int16_t));  
            inptr -= RSID_FFT_SAMPLES;  
        }
    }

    // 2. Если окно накопилось — запускаем тяжёлый БПФ
    if (rsid_window_ready) {  
        rsid_search();  
        rsid_window_ready = false;  
    }  
}

  
bool rsid_get_rtty_center(float* f_hz) {  
    if (!rtty_center_new) return false;  
    rtty_center_new = false;  
    if (f_hz) *f_hz = rtty_center_hz;  
    return true;  
}


void rsid_rx_service_sliced(void) {
    if (!rsid_inited) return;

    // --- ФАЗА 0: Попутный сбор сэмплов из Lock-Free шлюза (всегда на полной скорости) ---
    while (rsid_sq_tail != rsid_sq_head) {
        uint32_t tail = rsid_sq_tail;
        int16_t sample = rsid_sample_queue[tail];
        
        rsid_sq_tail = (tail + 1u) % RSID_SAMPLE_Q_SIZE;
        __dmb();

        aInputSamples[inptr++] = sample;
        if (inptr >= RSID_ARRAY_SIZE) {
            if (rsid_sm_state == RSID_STATE_IDLE && !rsid_window_ready) {
                memcpy(rsid_proc, aInputSamples, RSID_ARRAY_SIZE * sizeof(int16_t));
                rsid_window_ready = true;
            } else {
                rsid_dropped_windows++; // Процессор не успел обсчитать прошлое окно
            }
            memmove(&aInputSamples[0], &aInputSamples[RSID_FFT_SAMPLES],  
                    (RSID_BUFFER_SIZE - RSID_FFT_SAMPLES) * sizeof(int16_t));  
            inptr -= RSID_FFT_SAMPLES;  
        }
    }

    // --- ФАЗА 1: Выполнение ровно одного вычислительного кванта стейт-машины ---
    switch (rsid_sm_state) {
        
        case RSID_STATE_IDLE:
            if (rsid_window_ready) {
                rsid_sm_state = RSID_STATE_WINDOWING;
            }
            break;

        case RSID_STATE_WINDOWING:
            nBinLow  = 3;  
            nBinHigh = RSID_FFT_SIZE - 32;  
            for (int i = 0; i < RSID_FFT_N; i++) {  
                int32_t xi = (int32_t)rsid_proc[i];  
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
            if (s_fft_sink) {
                s_fft_sink(aFFTAmpl, RSID_FFT_SIZE, (float)(RSID_SAMPLE_RATE / 2048.0));
            }

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
            int sym = -1, bin = -1;  
          
            if (rsid_secondary_windows <= 0) {  
                if (search_amp(bin, sym, pCodes1, rsid_ids_1, rsid_ids_size1)) {  
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
                if (search_amp(bin, sym, pCodes2, rsid_ids_2, rsid_ids_size2)) {  
                    if (sym != RSID_NONE2) {  
                        rsid_apply(bin, sym, rsid_ids_2, rsid_ids_size2);  
                        rsid_reset_detect();   
                    }  
                }  
                if (rsid_secondary_windows <= 0) {  
                    rsid_secondary_windows = 0;  
                }  
            }

            rsid_window_ready = false;  
            rsid_sm_state = RSID_STATE_IDLE; // Цикл обработки кадра завершен
            break;
        }
    }
}

