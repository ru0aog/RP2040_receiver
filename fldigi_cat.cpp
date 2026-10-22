// ============================================================================  
// fldigi_cat.cpp — CAT-интерфейс приёмника (подмножество Kenwood TS-2000).  
// Отдельный USB-CDC порт ("RP2040 RX CAT"). Команды оканчиваются на ';'.  
// Поддержано: FA (get/set частоты), IF (статус), ID (идентификация).  
// По set-FA вызывается rx_retune() — перестройка приёмного тракта.  
// ============================================================================  
#include "fldigi_cat.h"  
#include "vfo_hardware.h"  
#include <Adafruit_TinyUSB.h>  
#include <stdio.h>  
#include <string.h>  
#include <stdlib.h>  
  
static Adafruit_USBD_CDC CatSerial;  
static bool cat_enabled = false;  
  
// Буфер накопления одной CAT-команды (до ';').  
#define CAT_BUF_MAX 64  
static char    cat_buf[CAT_BUF_MAX];  
static uint8_t cat_len = 0;  
  
// Для сброса мусорного байта при открытии порта терминалом.  
static bool cat_prev_dtr = false;  
  
// Режим Kenwood: 6 = FSK (RTTY). Используется только в ответе IF.  
static const char CAT_MODE = '6';  
  
// ---- Ответы ------------------------------------------------------------------  
  
static void cat_send(const char* s) {  
    Serial.printf("# CAT_TX <%s>\n", s);   // DEBUG: что мы ответили, убрать после отладки  
    CatSerial.write((const uint8_t*)s, strlen(s));
    CatSerial.flush();   // не держать ответ в USB-буфере (таймаут FLDigi ~50 мс)  
}  
  
// FA + 11 цифр частоты + ';'  
static void cat_reply_fa(void) {  
    char out[16];  
    snprintf(out, sizeof(out), "FA%011lu;", (unsigned long)rx_current_hz());  
    cat_send(out);  
}  
  
// Статус TS-2000. Поля-заглушки фиксированной длины; значащая часть — частота.  
// IF P1[11 freq] P2[5 spaces] P3[+/-5 RIT] P4[1 RIT] P5[1 XIT] P6[1] P7[2 mem]  
//    P8[1 RX/TX] P9[1 mode] P10[1 VFO] P11[1 scan] P12[1 split] P13[1 tone]  
//    P14[2 tone#] P15[1] ;

// Ответ IF строго по мануалу TS-2000 (38 байт, ';' на позиции 38):  
//   поз 1-2   "IF"  
//   поз 3-13  P1  частота, 11 цифр (незначащие = 0)  
//   поз 14-17 P2  шаг частоты      "0000"  
//   поз 18-23 P3  RIT/XIT ±99999   "+00000"  
//   поз 24    P4  RIT off          "0"  
//   поз 25    P5  XIT off          "0"  
//   поз 26    P6  банк             "0"  
//   поз 27-28 P7  канал            "00"  
//   поз 29    P8  RX               "0"  
//   поз 30    P9  режим (BYTE)     CAT_MODE  <- FILL 27 от начала  
//   поз 31    P10 FR/FT            "0"  
//   поз 32    P11 scan             "0"  
//   поз 33    P12 split            "0"  
//   поз 34    P13 tone             "0"  
//   поз 35-36 P14 tone №           "00"  
//   поз 37    P15 shift            "0"  
//   поз 38    ";"  
static void cat_reply_if(void) {  
    char out[40];  
    snprintf(out, sizeof(out),  
             "IF%011lu0000+00000000000%c0000000;",  
             (unsigned long)rx_current_hz(),  
             CAT_MODE);  
    cat_send(out);   // тело строго 38 байт, без пробелов  
}
  
static void cat_reply_id(void) {  
    cat_send("ID019;");   // 019 = Kenwood TS-2000  
}  
  
static void cat_reply_err(void) {  
    cat_send("?;");  
}  
  
// ---- Разбор одной завершённой команды ---------------------------------------  
  
static void cat_execute(const char* cmd) {  
    size_t n = strlen(cmd);  
    if (n < 2) { cat_reply_err(); return; }  
  
    // FA — частота VFO A  
    if (cmd[0] == 'F' && cmd[1] == 'A') {  
        if (n == 2) {               // "FA" — чтение  
            cat_reply_fa();  
        } else if (n == 13) {       // "FA" + 11 цифр — установка  
            bool digits = true;  
            for (int i = 2; i < 13; i++)  
                if (cmd[i] < '0' || cmd[i] > '9') { digits = false; break; }  
            if (!digits) { cat_reply_err(); return; }  
            uint32_t hz = (uint32_t)strtoul(cmd + 2, nullptr, 10);  
            rx_retune(hz);          // set-FA ответа не требует  
        } else {  
            cat_reply_err();  
        }  
        return;  
    }  
  
    // IF — статусная строка (FLDigi опрашивает ею частоту)  
    if (cmd[0] == 'I' && cmd[1] == 'F' && n == 2) { cat_reply_if(); return; }  
  
    // ID — идентификация  
    if (cmd[0] == 'I' && cmd[1] == 'D' && n == 2) { cat_reply_id(); return; }  
  
    // Неизвестная команда  
    cat_reply_err();  
}  
  
// ---- Публичный API -----------------------------------------------------------  
  
void fldigi_cat_init(void) {  
    CatSerial.setStringDescriptor("RP2040 RX CAT");  
    CatSerial.begin(115200);  
    cat_enabled = (Adafruit_USBD_CDC::getInstanceCount() >= 2);  
  
    // Интерфейс, добавленный после энумерации, появится только после  
    // переподключения — как в hayes_modem_init() маяка.  
    if (cat_enabled && TinyUSBDevice.mounted()) {  
        TinyUSBDevice.detach();  
        delay(10);  
        TinyUSBDevice.attach();  
    }  
    cat_len = 0;  
}  
  
void fldigi_cat_poll(void) {  
    if (!cat_enabled) return;  
  
    // Терминал только что открыл порт: чистим входной буфер и недособранную  
    // команду, чтобы спурьёзный байт от detach()/attach() не испортил разбор.  
    bool dtr = CatSerial.dtr();  
    if (!cat_prev_dtr && dtr) {  
        while (CatSerial.available() > 0) CatSerial.read();  
        cat_len = 0;  
    }  
    cat_prev_dtr = dtr;  
  
    while (CatSerial.available() > 0) {  
        char c = (char)CatSerial.read();  
        if (c == ';') {  
            cat_buf[cat_len] = '\0';
            Serial.printf("# CAT_RX <%s;>\n", cat_buf);   // DEBUG: убрать после отладки  
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
