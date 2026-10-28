// ============================================================================  
// fldigi_cat.cpp — CAT-интерфейс приёмника (подмножество Kenwood TS-2000).  
// Отдельный USB-CDC порт ("RP2040 RX CAT"). Команды оканчиваются на ';'.  
// Поддержано: FA (get/set частоты), IF (статус), ID (идентификация).  
// По set-FA вызывается rx_retune() — перестройка приёмного тракта.  
// ============================================================================  
#include "../cat/fldigi_cat.h"  
#include "../vfo/vfo_rp2040.h"  
#include "../audio/rx_audio.h"   
#include <Adafruit_TinyUSB.h>  
#include <stdio.h>  
#include <string.h>  
#include <stdlib.h>  
  
static bool cat_enabled = false;  
  
// Буфер накопления одной CAT-команды (до ';').  
#define CAT_BUF_MAX 64  
static char    cat_buf[CAT_BUF_MAX];  
static uint8_t cat_len = 0;  
  
// Для сброса мусорного байта при открытии порта терминалом.  
static bool cat_prev_dtr = false;  
  
// Текущий режим Kenwood: 1=LSB 2=USB 3=CW 4=FM 5=AM. Старт — USB.  
static char cat_mode = '2';  
  
// Индексы боковой в rx_audio.cpp: DEMOD_USB=0, DEMOD_LSB=1.  
#define CAT_DEMOD_USB 0  
#define CAT_DEMOD_LSB 1
#define CAT_DEMOD_AM  2  
#define CAT_DEMOD_FM  3  
  
// ---- Ответы ------------------------------------------------------------------  
  
static void cat_send(const char* s) {
    Serial.write((const uint8_t*)s, strlen(s));
    Serial.flush();
}
  
// FA + 11 цифр частоты + ';'  
static void cat_reply_fa(void) {  
    char out[16];  
    snprintf(out, sizeof(out), "FA%011lu;", (unsigned long)rx_current_hz());  
    cat_send(out);  
}  
  
static void cat_reply_if(void) {
    char out_buf[48];
    snprintf(out_buf, sizeof(out_buf), "IF00000000000       -00300   20000000;");
    // Накладываем частоту поверх нулей шаблона (индексы 2..12)
    uint32_t temp_freq = rx_current_hz(); 
    for (int i = 12; i >= 2; i--) {
        out_buf[i] = '0' + (temp_freq % 10);
        temp_freq /= 10;
    }
    cat_send(out_buf);
}


static void cat_reply_id(void) {  
    cat_send("ID019;");   // 019 = Kenwood TS-2000  
}  

static void cat_reply_md(void) {    
    char out[8];    
    snprintf(out, sizeof(out), "MD%c;", cat_mode);    
    cat_send(out);    
}

static void cat_reply_err(void) {  
    cat_send("?;");  
}

// Таблицы TS-2000 для 2-значных индексов (режим SSB).  
// SL — нижний срез (low cut), SH — верхний срез (high cut), Гц.  
static const uint16_t SL_TABLE[12] = {  
      0,   50,  100,  200,  300,  400,  500,  600,  700,  800,  900, 1000  
};  
static const uint16_t SH_TABLE[12] = {  
   1400, 1600, 1800, 2000, 2200, 2400, 2600, 2800, 3000, 3400, 4000, 5000  
};  
  
// Ответы по текущим значениям. Отвечаем в 4-значном Гц-формате.  
static void cat_reply_sh(void) {  
    char out[12];  
    snprintf(out, sizeof(out), "SH%04lu;", (unsigned long)rx_dsp_af_hi_hz());  
    cat_send(out);  
}  
static void cat_reply_sl(void) {  
    char out[12];  
    snprintf(out, sizeof(out), "SL%04lu;", (unsigned long)rx_dsp_af_lo_hz());  
    cat_send(out);  
}




  
// ---- Разбор одной завершённой команды ---------------------------------------  
  
static void cat_execute(const char* cmd) {  
    size_t n = strlen(cmd);  
    if (n < 2) { cat_reply_err(); return; }  
  
    // FA — управление частотой
    if (cmd[0] == 'F' && cmd[1] == 'A') {  
        if (n == 2) { cat_reply_fa(); } 
        else if (n == 13) {  
            uint32_t hz = (uint32_t)strtoul(cmd + 2, nullptr, 10);  
            rx_retune(hz);  
        } else { cat_reply_err(); }  
        return;  
    }  
  
    // IF — запрос статуса
    if (cmd[0] == 'I' && cmd[1] == 'F' && n == 2) { cat_reply_if(); return; }  
  
    // ID — идентификация  
    if (cmd[0] == 'I' && cmd[1] == 'D' && n == 2) { cat_reply_id(); return; }    
  
    // MD — режим демодуляции (Kenwood): 1=LSB 2=USB 3=CW 4=FM 5=AM  
    if (cmd[0] == 'M' && cmd[1] == 'D') {    
        if (n == 2) { cat_reply_md(); return; }        // запрос текущего режима    
        if (n == 3) {    
            switch (cmd[2]) {    
                case '1':  // LSB    
                    rx_audio_set_mode(RX_MODE_SSB);    
                    rx_audio_set_demod(CAT_DEMOD_LSB);    
                    cat_mode = '1';    
                    return;    
                case '2':  // USB    
                    rx_audio_set_mode(RX_MODE_SSB);    
                    rx_audio_set_demod(CAT_DEMOD_USB);    
                    cat_mode = '2';    
                    return;    
                case '3':  // CW — реализуется как USB-боковая + узкая полоса (SH/SL)    
                    rx_audio_set_mode(RX_MODE_SSB);    
                    rx_audio_set_demod(CAT_DEMOD_USB);    
                    cat_mode = '3';    
                    // при желании здесь же выставить узкую полосу CW через SH/SL    
                    return;    
                case '4':  // FM      
                    rx_audio_set_mode(RX_MODE_SSB);      
                    rx_audio_set_demod(CAT_DEMOD_FM);      
                    cat_mode = '4';      
                    return;      
                case '5':  // AM      
                    rx_audio_set_mode(RX_MODE_SSB);      
                    rx_audio_set_demod(CAT_DEMOD_AM);      
                    cat_mode = '5';      
                    return;    
                default:    
                    cat_reply_err();    
                    return;    
            }    
        }    
        cat_reply_err();    
        return;    
    }    

    // ЗАГЛУШКА ДЛЯ TS-480 PTT: 
    // FLDigi при старте шлет "TX1;" для переключения на передачу. 
    // Нам нужно просто проигнорировать этот запрос (ничего не отвечать), чтобы порт не падал.
    if (cmd[0] == 'T' && cmd[1] == 'X' && cmd[2] == '1' && n == 3) { return; }
    if (cmd[0] == 'R' && cmd[1] == 'X' && n == 2) { return; }

    // NT — автоматический notch. NT0 выкл, NT1 вкл.  
    if (cmd[0] == 'N' && cmd[1] == 'T') {  
        if (n == 3) {  
            rx_dsp_set_notch(cmd[2] != '0');  
            return;  
        }  
        cat_reply_err();  
        return;  
    }

    // SH — верхний срез полосы (high cut). SH; запрос, SHnn индекс, SHnnnn Гц.  
    if (cmd[0] == 'S' && cmd[1] == 'H') {  
        if (n == 2) { cat_reply_sh(); return; }  
        uint32_t f_hi;  
        if (n == 4) {                              // 2 цифры → индекс TS-2000  
            int idx = (int)strtoul(cmd + 2, nullptr, 10);  
            if (idx < 0 || idx > 11) { cat_reply_err(); return; }  
            f_hi = SH_TABLE[idx];  
        } else if (n == 6) {                       // 4 цифры → Гц напрямую  
            f_hi = (uint32_t)strtoul(cmd + 2, nullptr, 10);  
        } else { cat_reply_err(); return; }  
        rx_dsp_set_af_band(rx_dsp_af_lo_hz(), f_hi);  
        return;  
    }  
  
    // SL — нижний срез полосы (low cut). SL; запрос, SLnn индекс, SLnnnn Гц.  
    if (cmd[0] == 'S' && cmd[1] == 'L') {  
        if (n == 2) { cat_reply_sl(); return; }  
        uint32_t f_lo;  
        if (n == 4) {                              // 2 цифры → индекс TS-2000  
            int idx = (int)strtoul(cmd + 2, nullptr, 10);  
            if (idx < 0 || idx > 11) { cat_reply_err(); return; }  
            f_lo = SL_TABLE[idx];  
        } else if (n == 6) {                       // 4 цифры → Гц напрямую  
            f_lo = (uint32_t)strtoul(cmd + 2, nullptr, 10);  
        } else { cat_reply_err(); return; }  
        rx_dsp_set_af_band(f_lo, rx_dsp_af_hi_hz());    
        return;    
    }  
  
    // NR — шумоподавление (ALE). NR; запрос, NR0 выкл, NR1 вкл.  
    if (cmd[0] == 'N' && cmd[1] == 'R') {  
        if (n == 2) {                              // запрос состояния  
            char buf[8];  
            snprintf(buf, sizeof(buf), "NR%d;", rx_dsp_get_nr() ? 1 : 0);  
            Serial.print(buf);                     // тот же вывод, что у cat_reply_*  
            return;  
        }  
        if (n == 3) {  
            if      (cmd[2] == '1') { rx_dsp_set_nr(true);  return; }  
            else if (cmd[2] == '0') { rx_dsp_set_nr(false); return; }  
        }  
        cat_reply_err();  
        return;  
    }  
  
    // На все остальные неизвестные команды шлем ошибку  
    cat_reply_err();    
}

  
// ---- Публичный API -----------------------------------------------------------  
  
void fldigi_cat_init(void) {  
    cat_enabled = true;   // CAT идёт по основному Serial, отдельный CDC не поднимаем  
    cat_len = 0;  
}  
  
void fldigi_cat_poll(void) {  
    if (!cat_enabled) return;  
      // Терминал только что открыл порт: чистим входной буфер и недособранную  
    // команду, чтобы спурьёзный байт от detach()/attach() не испортил разбор.  
    bool dtr = Serial.dtr();  
    if (!cat_prev_dtr && dtr) {  
        while (Serial.available() > 0) Serial.read();  
        cat_len = 0;  
    }  
    cat_prev_dtr = dtr;  
  
    while (Serial.available() > 0) {  
        char c = (char)Serial.read();  
        if (c == ';') {  
            cat_buf[cat_len] = '\0';
            cat_execute(cat_buf);  
            cat_len = 0;  
        } else if (c != '\r' && c != '\n') {  
            if (cat_len < CAT_BUF_MAX - 1) {  
                cat_buf[cat_len++] = c;  
            } else {  
                cat_len = 0;        // переполнение — сброс  
            }  
        }  
    }  
}
