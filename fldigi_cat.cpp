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
    CatSerial.write((const uint8_t*)s, strlen(s));  
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
static void cat_reply_if(void) {  
    char out[48];  
    snprintf(out, sizeof(out),  
             "IF%011lu     +00000000%c0000000;",  
             (unsigned long)rx_current_hz(),  
             CAT_MODE);  
    cat_send(out);  
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
