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
  
static bool cat_enabled = false;  
  
// Буфер накопления одной CAT-команды (до ';').  
#define CAT_BUF_MAX 64  
static char    cat_buf[CAT_BUF_MAX];  
static uint8_t cat_len = 0;  
  
// Для сброса мусорного байта при открытии порта терминалом.  
static bool cat_prev_dtr = false;  
  
// Изменяем режим Kenwood: '2' = USB (стандарт для цифровых видов во FLDigi)
static const char CAT_MODE = '2'; 
  
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
    char out_buf[48];
    
    // Добавлен ровно ОДИН ноль в конец перед точкой с запятой.
    // Теперь строка содержит строго 38 символов.
    // Позиция моды '2' осталась на правильном 30-м месте.
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
  
static void cat_reply_err(void) {  
    cat_send("?;");  
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

    // ЗАГЛУШКА ДЛЯ TS-480 PTT: 
    // FLDigi при старте шлет "TX1;" для переключения на передачу. 
    // Нам нужно просто проигнорировать этот запрос (ничего не отвечать), чтобы порт не падал.
    if (cmd[0] == 'T' && cmd[1] == 'X' && cmd[2] == '1' && n == 3) { return; }
    if (cmd[0] == 'R' && cmd[1] == 'X' && n == 2) { return; }
  
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
