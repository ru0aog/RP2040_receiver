#ifndef FLDIGI_CAT_H  
#define FLDIGI_CAT_H  
  
#include <Arduino.h>  
  
// Модуль CAT-управления приёмником по образцу Kenwood TS-2000.  
// Работает на ОТДЕЛЬНОМ (втором) USB-CDC порту, не мешая основному Serial.  
// FLDigi: RigCAT/Hamlib, модель Kenwood TS-2000, 115200 8N1.  
  
void fldigi_cat_init(void);   // поднять второй USB-CDC, вызывать из setup()  
void fldigi_cat_poll(void);   // неблокирующий разбор команд, вызывать из loop()  
  
#endif // FLDIGI_CAT_H