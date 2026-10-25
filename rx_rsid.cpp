#include "rx_rsid.h"  
#include <Arduino.h>  
#include <math.h>  
#include <string.h>  
#include <complex>  
  
// FFT-движок из FLDigi (положи refs/gfft.h рядом или поправь путь include).  
#include "gfft.h"  
  
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
  
// float вместо double — экономия ОЗУ на RP2040.  
typedef float                       rs_fft_type;  
typedef std::complex<rs_fft_type>   rs_cpx_type;  
  
// ============================================================================  
//  Таблица мод RSID (code -> имя). Полный primary-набор из refs/rsid_defs.cxx,  
//  поле fldigi-mode отброшено — для МК достаточно кода и человекочитаемого тега.  
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
  
// ============================================================================  
//  Состояние детектора (аналог полей cRsId).  
// ============================================================================  
static g_fft<rs_fft_type>* rsfft = nullptr;  
  
static unsigned char  pCodes1[sizeof(rsid_ids_1) / sizeof(rsid_ids_1[0]) * RSID_NSYMBOLS];  
  
static float          aInputSamples[RSID_BUFFER_SIZE];   // кольцо входа 11025 Гц  
static rs_fft_type    fftwindow[RSID_ARRAY_SIZE];  
static rs_cpx_type    aFFTcmplx[RSID_ARRAY_SIZE];  
static rs_fft_type    aFFTAmpl[RSID_FFT_SIZE];  
static int16_t        fft_buckets[RSID_NTIMES][RSID_FFT_SIZE];  // ~60 КБ (int16)  
  
// Снимок готового окна 2048 отсчётов: наполняется в rsid_rx_push() (звуковой  
// тракт), обрабатывается в rsid_rx_service() (loop). Оба на Core 0.  
static float          rsid_proc[RSID_ARRAY_SIZE];     // +8 КБ  
static volatile bool  rsid_window_ready = false;  
  
static int   inptr;
static int   nBinLow, nBinHigh;  
static int   hamming_resolution = RSID_HAMMING_RESOLUTION;  
  
static rsid_detect_cb_t s_cb = nullptr;  
  
// ============================================================================  
//  Окно Хэмминга (refs/rsid.cxx -> filters.h hamming()).  
// ============================================================================  
static inline double rsid_hamming(double x) {  
    return 0.54 - 0.46 * cos(2.0 * M_PI * x);  
}  
  
static void rsid_reset_state(void) {  
    inptr = RSID_FFT_SIZE;  
    memset(aInputSamples, 0, sizeof(aInputSamples));  
    memset(aFFTAmpl,      0, sizeof(aFFTAmpl));  
    memset(fft_buckets,   0, sizeof(fft_buckets));  
    for (int n = 0; n < RSID_ARRAY_SIZE; n++) aFFTcmplx[n] = rs_cpx_type(0, 0);  
    hamming_resolution = RSID_HAMMING_RESOLUTION;  
}  
  
// ----------------------------------------------------------------------------  
//  CalculateBuckets (refs/rsid.cxx, без изменений логики).  
// ----------------------------------------------------------------------------  
static void CalculateBuckets(const rs_fft_type* pSpectrum, int iBegin, int iEnd) {  
    rs_fft_type Amp = 0.0f, AmpMax = 0.0f;  
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
  
static bool search_amp(int& bin_out, int& symbol_out) {  
    int iDistanceMin = 1000;  
    int iBin = -1, iSymbol = -1;  
    for (int i = 0; i < rsid_ids_size1; i++) {  
        const unsigned char* pc = pCodes1 + i * RSID_NSYMBOLS;  
        for (int j = nBinLow; j < nBinHigh - RSID_NTIMES; j++) {  
            int d = HammingDistance(j, pc);  
            if (d < iDistanceMin) {  
                iDistanceMin = d;  
                iSymbol = rsid_ids_1[i].rs;  
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
//  apply: выдать (имя моды, частоту). Вместо GUI/init_modem — колбэк/Serial.  
// ----------------------------------------------------------------------------  
static void rsid_apply(int iBin, int iSymbol) {  
    double rsidfreq = (iBin + RSID_NSYMBOLS - 0.5) * RSID_SAMPLE_RATE / 2048.0;  
  
    const char* name = nullptr;  
    for (int n = 0; n < rsid_ids_size1; n++) {  
        if (rsid_ids_1[n].rs == iSymbol) { name = rsid_ids_1[n].name; break; }  
    }  
    if (!name) return;   // неизвестный код  
  
    rsid_result_t r;  
    r.mode_code = iSymbol;  
    r.mode_name = name;  
    r.freq_hz   = (float)rsidfreq;  
  
    if (s_cb) s_cb(&r);  
    else Serial.printf("# RSID mode=%s code=%d freq=%.1f\n",  
                       r.mode_name, r.mode_code, r.freq_hz);  
}  
  
// ----------------------------------------------------------------------------  
//  search (refs/rsid.cxx): FFT окна 2048 -> bucket'ы -> поиск кода.  
//  bReverse = false (USB, тон в положительной звуковой полосе).  
// ----------------------------------------------------------------------------  
static void rsid_search(void) {  
    nBinLow  = 3;  
    nBinHigh = RSID_FFT_SIZE - 32;  
  
    for (int i = 0; i < RSID_ARRAY_SIZE; i++)  
        aFFTcmplx[i] = rs_cpx_type(rsid_proc[i] * fftwindow[i], 0);
  
    rsfft->ComplexFFT(aFFTcmplx);  
  
    memset(aFFTAmpl, 0, sizeof(aFFTAmpl));  
    const rs_fft_type pscale = (rs_fft_type)(4.0 / ((double)RSID_FFT_SIZE * RSID_FFT_SIZE));  
    for (int i = 0; i < RSID_FFT_SIZE; i++)  
        aFFTAmpl[i] = std::norm(aFFTcmplx[i]) * pscale;  
  
    int bucket_low  = 3;  
    int bucket_high = RSID_FFT_SIZE - 32;  
  
    memmove(fft_buckets, &(fft_buckets[1][0]),  
            (RSID_NTIMES - 1) * RSID_FFT_SIZE * sizeof(fft_buckets[0][0]));  
    memset(&(fft_buckets[RSID_NTIMES - 1][0]), 0, RSID_FFT_SIZE * sizeof(fft_buckets[0][0]));  
  
    CalculateBuckets(aFFTAmpl, bucket_low,     bucket_high - RSID_NTIMES);  
    CalculateBuckets(aFFTAmpl, bucket_low + 1, bucket_high - RSID_NTIMES);  
  
    int sym = -1, bin = -1;  
    if (search_amp(bin, sym)) {  
        if (sym != RSID_ESCAPE) {     // escape-набор 2 не реализован  
            rsid_apply(bin, sym);  
            rsid_reset_state();  
        }  
    }  
}  
  
// ============================================================================  
//  Публичный интерфейс (rx_rsid.h).  
// ============================================================================  
void rsid_rx_init(float fs_hz) {  
    (void)fs_hz;   // вход фиксирован RSID_SAMPLE_RATE (ресемплинг в rx_audio.cpp)  
  
    if (!rsfft) rsfft = new g_fft<rs_fft_type>(RSID_ARRAY_SIZE);  
  
    for (int i = 0; i < RSID_ARRAY_SIZE; i++)  
        fftwindow[i] = (rs_fft_type)rsid_hamming(1.0 * i / RSID_ARRAY_SIZE);  
  
    memset(pCodes1, 0, sizeof(pCodes1));  
    for (int i = 0; i < rsid_ids_size1; i++)  
        rsid_encode(rsid_ids_1[i].rs, pCodes1 + i * RSID_NSYMBOLS);  
  
    rsid_reset_state();  
}  
  
void rsid_rx_set_callback(rsid_detect_cb_t cb) { s_cb = cb; }  
  
void rsid_rx_reset(void) { rsid_reset_state(); }  
  
void rsid_rx_push(int16_t sample) {  
    if (!rsfft) return;                      // не инициализирован  
  
    aInputSamples[inptr++] = (float)sample;  
  
    if (inptr >= RSID_ARRAY_SIZE) {  
        // Тяжёлый rsid_search() НЕ вызываем здесь (блокирует наполнение  
        // USB-аудио → разрывы). Копируем готовое окно в снимок и отдаём  
        // обработку в rsid_rx_service() из loop(). Если предыдущее окно ещё  
        // не обработано — пропускаем (лучше потерять окно, чем рвать звук).  
        if (!rsid_window_ready) {  
            memcpy(rsid_proc, aInputSamples, RSID_ARRAY_SIZE * sizeof(float));  
            rsid_window_ready = true;  
        }  
        memmove(&aInputSamples[0], &aInputSamples[RSID_FFT_SAMPLES],  
                (RSID_BUFFER_SIZE - RSID_FFT_SAMPLES) * sizeof(float));  
        inptr -= RSID_FFT_SAMPLES;  
    }  
}  
  
// Выполняется в loop() ПОСЛЕ rx_audio_poll(): тяжёлый БПФ/поиск вне горячего  
// пути заполнения USB-буфера. Обрабатывает максимум одно окно за вызов.  
void rsid_rx_service(void) {  
    if (!rsfft) return;  
    if (!rsid_window_ready) return;  
    rsid_search();  
    rsid_window_ready = false;  
}
