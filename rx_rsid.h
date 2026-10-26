#pragma once  
#include <stdint.h>  
  
// ============================================================================  
//  Бортовой RSID-декодер (порт refs/rsid.cxx -> cRsId, только RX).  
//  
//  Назначение: по короткой RSID-посылке (15 символов MFSK, шаг тона  
//  11025/1024 ≈ 10.77 Гц) определить МОДУ сигнала и его ЦЕНТРАЛЬНУЮ ЧАСТОТУ,  
//  чтобы автоматически подстроить матч-фильтр/демодулятор под реальный сигнал.  
//  
//  Поток данных:  
//    Core 1 (DSP) -> демод-моно 15625 Гц -> get_demod_input() (rx_audio.cpp)  
//      -> ресемплер Фарроу 15625->11025 -> rsid_rx_push() (каждый отсчёт 11025 Гц)  
//      -> накопление RSID_FFT_SAMPLES (512) -> search() -> (mode, freq).  
//  
//  Все вызовы идут из Core 0 (из rx_audio_poll()->next_frame()->get_demod_input()),  
//  поэтому Serial здесь безопасен (на Core 0 не real-time).  
// ============================================================================  
  
// Частота дискретизации, на которой работает RSID-детектор (после ресемплинга).  
// Совпадает с RSID_SAMPLE_RATE из refs/rsid.h.  
#ifndef RSID_RX_FS_HZ  
#define RSID_RX_FS_HZ   11025.0f  
#endif  
  
// Порог допустимого расстояния Хэмминга при сверке принятой последовательности  
// с таблицей мод (в FLDigi берётся из progdefaults.RsID_label_type; на МК —  
// константа). 0 = точное совпадение, больше = мягче (но выше ложные срабатывания).  
#ifndef RSID_HAMMING_RESOLUTION  
#define RSID_HAMMING_RESOLUTION   1  
#endif  
  
#ifdef __cplusplus  
extern "C" {  
#endif  
  
// Результат успешного распознавания RSID.  
typedef struct {  
    int         mode_code;   // код moды (trx_mode из refs/globals.h, напр. MODE_RTTY)  
    const char* mode_name;   // человекочитаемое имя моды (из mode_info[].sname)  
    float       freq_hz;     // центральная частота сигнала в звуковой полосе, Гц  
} rsid_result_t;  
  
// Тип колбэка, вызываемого при обнаружении RSID.  
typedef void (*rsid_detect_cb_t)(const rsid_result_t* r);  
  
// Инициализация декодера.  
//   fs_hz — частота входного потока (должна быть RSID_RX_FS_HZ = 11025).  
// Строит таблицы ожидаемых тон-последовательностей для всех известных мод  
// (Encode) и готовит FFT-движок.  
void rsid_rx_init(float fs_hz);  
  
// Подать ОДИН отсчёт потока 11025 Гц (из ресемплера в rx_audio.cpp).  
// Накапливает окно; при наборе 2048 отсчётов сохраняет снимок для обработки.  
void rsid_rx_push(int16_t sample);  
  
// Отложенная ДИСПЕТЧЕРИЗАЦИЯ результата RSID (печать метки + смена моды).  
// Вызывать из loop() на Core 0. Сам тяжёлый поиск идёт на Core 1 (rsid_rx_service).  
void rsid_rx_dispatch(void);
  
// Зарегистрировать колбэк обнаружения (необязательно). Если не задан —  
// результат печатается в Serial строкой "# RSID mode=<имя> freq=<Гц>".  
void rsid_rx_set_callback(rsid_detect_cb_t cb);  
  
// Сбросить внутреннее состояние детектора (например, при ретюне/смене режима).  
void rsid_rx_reset(void);  
  
// Вернуть найденный по FFT центр пары mark/space RTTY (Гц). true — если  
// с прошлого вызова появился новый центр. Подаётся в rx_mf_init().  
bool rsid_get_rtty_center(float* f_hz);

// Сток спектра IFKP: вызывается каждое FFT-окно (нижняя половина спектра).  
//   ampl[nbins] — мощности бинов, bin_hz = RSID_SAMPLE_RATE/2048 ≈ 5.383 Гц.  
typedef void (*rsid_fft_sink_t)(const uint64_t* ampl, int nbins, float bin_hz);  
void rsid_rx_set_fft_sink(rsid_fft_sink_t sink);




#pragma once  
#include <stdint.h>  
  
#ifndef RSID_RX_FS_HZ  
#define RSID_RX_FS_HZ   11025.0f  
#endif  
  
#ifndef RSID_HAMMING_RESOLUTION  
#define RSID_HAMMING_RESOLUTION   1  
#endif  
  
#ifdef __cplusplus  
extern "C" {  
#endif  
  
typedef struct {  
    int         mode_code;   
    const char* mode_name;   
    float       freq_hz;     
} rsid_result_t;  
  
typedef void (*rsid_detect_cb_t)(const rsid_result_t* r);  
  
void rsid_rx_init(float fs_hz);  
void rsid_rx_push(int16_t sample);  
void rsid_rx_service(void);

// Высокопроизводительный пошаговый (квантованный) обработчик для Core 1
// Выполнить ОДИН вычислительный квант асинхронной стейт-машины RSID  
// (попутный сбор сэмплов из межъядерной очереди + один шаг БПФ/поиска).  
// Вызывать из холостой ветки цикла Core 1. 
void rsid_rx_service_sliced(void);

void rsid_rx_dispatch(void);
void rsid_rx_set_callback(rsid_detect_cb_t cb);  
void rsid_rx_reset(void);  
bool rsid_get_rtty_center(float* f_hz);

typedef void (*rsid_fft_sink_t)(const uint64_t* ampl, int nbins, float bin_hz);  
void rsid_rx_set_fft_sink(rsid_fft_sink_t sink);  
  


#ifdef __cplusplus  
}  
#endif