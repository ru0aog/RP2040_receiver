/**    
 * ============================================================================    
 *  rx_decoders.cpp — декодеры цифровых мод на децимированной базовой полосе    
 *  Fs_in = rx_dsp_fs_out() ≈ 15.6 кс/с при RX_ADC_SAMPLE_RATE_HZ = 250 кГц.    
 * ============================================================================    
 */  

#include <Arduino.h>  
#include "vfo_hardware.h"  
#include "rx_decoders.h"  
#include "rx_rsid.h"  
#include "rx_mf.h"
#include "rx_cw.h"
#include "rx_rtty.h"
#include "rx_ifkp.h"
#include "rx_olivia.h" 
  
// Коды мод из таблицы RSID (см. логи: RTTY-45=39, CW=26).  
#define RSID_CODE_RTTY45   39  
#define RSID_CODE_CW       26  
#define RTTY_SHIFT_HZ      170.0f
#define RSID_CODE_IFKP     1171
#define RSID_CODE_OLIVIA_8_250  69

enum { DEC_NONE = 0, DEC_RTTY, DEC_CW, DEC_IFKP, DEC_OLIV }; 
  
static int   active_dec = DEC_NONE;  
static float dec_fs_hz  = 15625.0f;

static void stop_all_goertzel_decoders();
  
static void default_output(char c) { Serial.print(c); }  
static rx_output_cb_t s_out = default_output;  
  
void rx_decoders_set_output(rx_output_cb_t cb) { s_out = cb ? cb : default_output; }  
void rx_decoders_emit(char c) { if (s_out) s_out(c); }  

// Один отсчёт базовой полосы (15625 Гц) в активный декодер
// Мода выбирается рантайм по результату RSID
void rx_decoders_sample(int16_t m) {  
    switch (active_dec) {  
    case DEC_RTTY: rx_mf_sample(m, 0); break;  
    case DEC_CW:   rx_cw_sample(m); break;
    case DEC_IFKP: rx_ifkp_push(m); break;
    case DEC_OLIV: rx_olivia_push(m); break;
    default:       break;   // DEC_NONE — пока метки нет, не декодируем  
    }  
}

// Вызывается из rsid_rx_service() (Core 0, loop) — Serial безопасен.  
void rx_decoders_on_rsid(const rsid_result_t* r) {  
    if (!r) return;  
  
    // ЕДИНСТВЕННАЯ точка печати RSID-метки.  
    Serial.printf("# RSID mode=%s code=%d freq=%.1f\n",  
                  r->mode_name, r->mode_code, r->freq_hz);  
  
    switch (r->mode_code) {  
    case RSID_CODE_RTTY45:  
        stop_all_goertzel_decoders(); 
        rx_mf_init(dec_fs_hz, r->freq_hz, RTTY_SHIFT_HZ);  
        rx_rtty_init(dec_fs_hz);  
        active_dec = DEC_RTTY;
        Serial.printf("# DECODER=RTTY center=%.1f Hz\n", (double)r->freq_hz);  
        break;  
    case RSID_CODE_CW:  
        stop_all_goertzel_decoders(); 
        rx_cw_init(dec_fs_hz, r->freq_hz);  
        active_dec = DEC_CW;
        Serial.printf("# DECODER=CW center=%.1f Hz\n", (double)r->freq_hz);  
        break;  
    case RSID_CODE_IFKP:
        stop_all_goertzel_decoders(); 
        rx_ifkp_init(r->freq_hz);           // первично — центр из RSID  
        active_dec = DEC_IFKP;
        Serial.printf("# DECODER=IFKP center=%.1f Hz\n", (double)r->freq_hz);  
        break;
    case RSID_CODE_OLIVIA_8_250:  
        stop_all_goertzel_decoders();  
        rx_olivia_init(r->freq_hz);         // центр из RSID, банк Гёрцеля 8 тонов  
        active_dec = DEC_OLIV;
        Serial.printf("# DECODER=OLIVIA center=%.1f Hz\n", (double)r->freq_hz);    
        break;
    default:  
        Serial.printf("# DECODER unsupported code=%d\n", r->mode_code);  
        break;  
    }  
}

// Остановить декодеры, использующие общий банк Гёрцеля.
static void stop_all_goertzel_decoders() {
    if      (active_dec == DEC_RTTY)    { /*rx_rtty_stop();*/   } 
    else if (active_dec == DEC_CW)      { /*rx_cw_stop();  */   }
    else if (active_dec == DEC_IFKP)    { rx_ifkp_stop();   }
    else if (active_dec == DEC_OLIV)    { rx_olivia_stop(); }
}

void rx_decoders_init(float fs_hz) {  
    dec_fs_hz  = (fs_hz > 1.0f) ? fs_hz : 15625.0f;  
    active_dec = DEC_NONE;  
    rsid_rx_set_callback(rx_decoders_on_rsid);   // весь вывод/маршрутизация через нас
}


