/**    
 * ============================================================================    
 *  rx_decoders.cpp — декодеры цифровых мод на децимированной базовой полосе    
 *  Работает на core1 после rx_dsp (Goertzel-банк + per-mode обработчики).    
 *  Fs_in = rx_dsp_fs_out() ≈ 15.6 кс/с при RX_ADC_SAMPLE_RATE_HZ = 250 кГц.    
 * ============================================================================    
 */  

#include <Arduino.h>
#include "vfo_hardware.h"


// флаги отладки
    bool    dbg_iq       = true;    // [IQ] стрим сырого I/Q в Serial: заголовок + пакеты 64×(i,q)
    bool    dbg_rx       = false;    // флаг печати лога отладки
static bool dbg_spectrum = false;    // спектроскан полосы: [SPEC] печать мощностей бинов
    bool    dbg_bits     = false;    // [RTTY-BITS] печать каждого решённого бита: S=старт, 0-4=данные, P=стоп
static bool dbg_trace    = false;    // [TRC] огибающие+v3+state: фронты bit и каждые 32 отсчёта  
static bool rtty_afc_en  = false;    // AFC ВЫКЛЮЧЕН по умолчанию — включаем только после стабильного декода

// ---- API из rx_dsp.cpp ----    
extern bool     rx_dsp_get_output(const int16_t** i, const int16_t** q, uint8_t* n);    
extern void     rx_dsp_release_output(void);    
extern void     rx_dsp_set_nco_hz(float f_hz);    
extern float    rx_dsp_fs_out(void);    
  
extern uint32_t rx_capture_dropped(void);  
static void rtty_emit(uint8_t c);
static void spec_reset(void);
static void spec_sample(int16_t xr, int16_t xq);
static bool rtty_rx(bool bit);        // fwd — используется в rtty_rx_sample ниже

typedef enum {    
    RX_MODE_NONE=0, RX_MODE_CW, RX_MODE_RTTY,    
    RX_MODE_OLIVIA, RX_MODE_CONTESTIA,    
    RX_MODE_THOR, RX_MODE_MFSK16, RX_MODE_DOMEX, RX_MODE_FSQ, RX_MODE_IFKP,    
    RX_MODE_JTTY    
} rx_mode_t;    
  
// Единственная точка сопоставления имя↔режим. .ino вызывает её, не зная enum.    
uint8_t rx_decoders_mode_by_name(const char* name) {    
    if (!strcmp(name, "cw"))       return RX_MODE_CW;    
    if (!strcmp(name, "rtty"))     return RX_MODE_RTTY;    
    if (!strcmp(name, "olivia"))   return RX_MODE_OLIVIA;    
    if (!strcmp(name, "contestia"))return RX_MODE_CONTESTIA;    
    if (!strcmp(name, "thor"))     return RX_MODE_THOR;    
    if (!strcmp(name, "mfsk16"))   return RX_MODE_MFSK16;    
    if (!strcmp(name, "dominoex")) return RX_MODE_DOMEX;    
    if (!strcmp(name, "dom"))      return RX_MODE_DOMEX;    
    if (!strcmp(name, "fsq"))      return RX_MODE_FSQ;    
    if (!strcmp(name, "ifkp"))     return RX_MODE_IFKP;    
    if (!strcmp(name, "jtty"))     return RX_MODE_JTTY;    
    return RX_MODE_NONE;    
}  

  
static rx_mode_t  dec_mode = RX_MODE_NONE;    
static uint32_t   sym_samples;        // отсчётов на символ при Fs_out    
static uint32_t   sym_acc = 0;        // накоплено в окне    
static uint32_t   sample_idx = 0;  
static float      fs_out = 15625.0f;    
static float      f_center_hz = 0;    // для AFC    
static bool       afc_on = false;  
static bool rtty_scan = false;





  
void rx_decoders_stop(void) { dec_mode = RX_MODE_NONE; }    
    
// Вызывается из цикла core1 (rx_dsp_entry) после заполнения out_i/out_q    
void rx_decoders_process(const int16_t* xi, const int16_t* xq, uint8_t n) {  
    static uint32_t calls = 0;
    if (++calls == 1 || calls % 1000 == 0)    
        if (dbg_rx) {Serial.printf("[DEC] process: calls=%lu mode=%u n=%u\n",    
                      (unsigned long)calls, (unsigned)dec_mode, (unsigned)n);}  
  
    if (dec_mode == RX_MODE_NONE || n == 0) return;

    // ---- IQ-дамп в COM: пакет 'IQ'+len + n пар int16 (I,Q) LE ----  
    if (dbg_iq) {  
        Serial.write('I'); Serial.write('Q'); Serial.write(n);  
        for (uint8_t i = 0; i < n; i++) {  
            Serial.write((uint8_t*)&xi[i], 2);  
            Serial.write((uint8_t*)&xq[i], 2);  
        }  
    } 
  

  
}                                                       // конец rx_decoders_process


// ---------------- Спектроскан: банк NCO+интеграторов по полосе ----------------  
// Каждый бин — как канал rtty_mixer+boxcar: миксер на f_k, суммирование за окно.  
// Частота бина i: SPEC_F0 + i*SPEC_STEP. Печать раз в SPEC_WIN отсчётов.  
#define SPEC_NBINS 50  
#define SPEC_F0    (-1900.0f)        // нижний край скана, Гц  
#define SPEC_STEP  (40.0f)          // шаг, Гц  (-2000..+1840)  
#define SPEC_WIN   344               // окно интегрирования, отсчётов (~33 мс)  
  
static float    spec_zr[SPEC_NBINS], spec_zi[SPEC_NBINS];  // текущий фазор e^{jφ}  
static float    spec_sr[SPEC_NBINS], spec_si[SPEC_NBINS];  // шаг e^{j2πf/fs}  
static double   spec_ar[SPEC_NBINS];    
static double   spec_ai[SPEC_NBINS];    
static uint16_t spec_n = 0;  
  
static void spec_reset(void) {    
    for (uint8_t k = 0; k < SPEC_NBINS; k++) {    
        float w = 2.0f; //* (float)M_PI * (SPEC_F0 + k * SPEC_STEP) / rrx.fs;    
        spec_sr[k] = cosf(w); spec_si[k] = sinf(w);   // шаг — один раз    
        spec_zr[k] = 1.0f;  spec_zi[k] = 0.0f;    
        spec_ar[k] = 0;     spec_ai[k] = 0;    
    }    
    spec_n = 0;    
}    
  
static float spec_peak[SPEC_NBINS];  
  
static void spec_sample(int16_t xr, int16_t xq) {  
    for (uint8_t k = 0; k < SPEC_NBINS; k++) {  
        spec_ar[k] += xr * spec_zr[k] + xq * spec_zi[k];  
        spec_ai[k] += xq * spec_zr[k] - xr * spec_zi[k];  
        float nr = spec_zr[k] * spec_sr[k] - spec_zi[k] * spec_si[k];  
        float ni = spec_zr[k] * spec_si[k] + spec_zi[k] * spec_sr[k];  
        spec_zr[k] = nr; spec_zi[k] = ni;  
    }  
    if (++spec_n >= SPEC_WIN) {  
        for (uint8_t k = 0; k < SPEC_NBINS; k++) {  
            float m = sqrtf(spec_ar[k]*spec_ar[k] + spec_ai[k]*spec_ai[k]);  
            if (m > spec_peak[k]) spec_peak[k] = m;   // peak-hold  
        }  
        memset(spec_ar, 0, sizeof(spec_ar));  
        memset(spec_ai, 0, sizeof(spec_ai));  
        spec_n = 0;  
  
        static uint8_t rep = 0;  
        if (++rep >= 30) {                            // ~1 с усреднения  
            rep = 0;  
            Serial.printf("[SPEC-PK]");  
            for (uint8_t k = 0; k < SPEC_NBINS; k++) {  
                Serial.printf(" %.0f", (double)spec_peak[k]);  
                spec_peak[k] = 0;  
            }  
            Serial.println();  
        }  
    }  
}

