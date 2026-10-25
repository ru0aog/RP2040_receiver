#ifndef VFO_HARDWARE_H
#define VFO_HARDWARE_H

#include <Arduino.h>

// Лимиты частотного плана
// vfo_hardware.h  
#define VFO_CLK_SYS_NOMINAL_HZ  133000000ULL  // электрический номинал без вольтмода  
#define VFO_CLK_SYS_MIN_HZ       48000000ULL  // нижняя граница поиска оптимума  
#define VFO_CLK_SYS_MAX_HZ      420000000ULL  // 420 - дефайн остаётся как АБСОЛЮТНЫЙ аппаратный барьер
#define PLL_MAX_HZ             4300000000ULL  // 4.3 - дефайн остаётся как АБСОЛЮТНЫЙ аппаратный барьер
#define VFO_FLASH_SCK_MAX_HZ     50000000ULL
#define VFO_THROTTLE_HI_C   80.0f   
#define VFO_THROTTLE_LO_C   60.0f
#define VFO_SIMPLE_FRAC_INT_MAX 50   // выше этого int вето простых дробей не применяется (НЧ: 1/2 на железе чисто)

// --- Приёмная часть (RX) ---  
// Сдвиг ПЧ (= целевой f_nco). Ставим в середину стопбанда последнего HB-каскада  
// (RX_FS_OUT_HZ=15625, стопбанд HB1 ~9.6..15 кГц), чтобы DC-дырку (утечка LO +  
// 1/f ADC) давило ~90 дБ ещё в децимации, до аудио. ВЫШЕ AF-среза 5 кГц.  
#define RX_IF_SHIFT_HZ        11000UL   // Гц, целевая ПЧ = целевой f_nco  
  
// --- Гард DC-дырки при подборе делителя RX-гетеродина ---  
#define RX_NCO_FORBID_LO         0UL    // запретное окно дырки (= звуковая полоса):  
#define RX_NCO_FORBID_HI      5000UL    //   f_nco НИКОГДА не должен сюда попадать (AF-срез 5 кГц)  
#define RX_NCO_SWEET_LO       9600UL    // «сладкая» зона = стопбанд HB1 (~90 дБ режекции)  
#define RX_NCO_SWEET_HI      14500UL    //   ниже фолда у 15625 с запасом  
#define RX_NCO_MAX_HZ        15000UL    // потолок f_nco (ниже фолда 15625 и << 62500 окна NCO)
  
// Нижняя граница системной частоты для RX-гетеродина (квадратурный LO).  
// Ниже этой шины спур-уровень и загрузка Core1 растут — держим шину высокой.  
#define RX_CLK_SYS_MIN_HZ   240000000UL // Гц, нижний предел clk_sys в RX-режиме

#define RX_ADC_SAMPLE_RATE_HZ 250000UL  // суммарная частота дискретизации (125 ksps на канал)
// ADC работает на 48 МГц. adc_set_clkdiv(48000000/250000) даст 192.0 → 250 ksps суммарно = 125 ksps/канал. 
// Младшие 12 бит каждого слова — отсчёт, формат FIFO [I0,Q0,I1,Q1,...].

// Уже есть: extern int subband_pins[4];  (если нет — добавить рядом с pin_freq_out)  
// Таблица переключаемых ДПФ: верхняя граница диапазона + 4-битный код  
// на пинах subband_pins[0..3] (бит0 → subband_pins[0] и т.д.).  
extern void vfo_subband_apply(uint32_t freq_hz);
extern uint8_t subband_current;
void vfo_subband_init();

// ---- Параметры захвата ----  
// FIFO 32-битное слово: [31:16] — канал Q (ADC1), [15:0] — I (ADC0),  
// т.к. round-robin выдаёт отсчёты по возрастанию канала и FIFO пакует по 2.  
#define RX_BLOCK_SAMPLES      512         // отсчётов I/Q на блок (пар в 32-бит словах) 
#define RX_ADC_PIN_I          26          // GPIO26 = ADC0, канал I  
#define RX_ADC_PIN_Q          27          // GPIO27 = ADC1, канал Q  

// --- RX-тракт ---  
void     rx_capture_start(void);  
void     rx_capture_stop(void);  
const volatile uint32_t* rx_capture_get_block(void);  
void     rx_capture_release(void);  
uint32_t rx_capture_dropped(void);  
  
void  rx_dsp_start(float f_target_hz);      // целевая частота приёма, Гц  
void  rx_dsp_set_nco_hz(float f_hz);        // AFC  
bool  rx_dsp_get_output(const int16_t** i, const int16_t** q, uint8_t* n);  
void  rx_dsp_release_output(void);  
float rx_dsp_fs_out(void);

uint32_t rx_dsp_out_dropped(void);  
uint32_t rx_dsp_aud_dropped(void);  
uint32_t rx_dsp_load_permille(void);
uint32_t rx_dsp_clip_count(void);   // диагностика клиппинга (sat16)
uint8_t  rx_dsp_aud_fill(void);   // заполнение очереди aud для адаптивного ресемплинга

// Демодулятор на Core 1 (USB-аудио):  
#define RXD_DEMOD_RAW 0  
#define RXD_DEMOD_USB 1  
#define RXD_DEMOD_LSB 2
#define RXD_DEMOD_AM  3   // амплитудный детектор (огибающая)    
#define RXD_DEMOD_FM  4   // частотный дискриминатор  
void rx_dsp_set_demod(uint8_t mode);
// Перестраиваемая полоса AF-фильтра (SSB), Гц. f_lo=0 → поведение прежнего ФНЧ.  
void rx_dsp_set_af_band(uint32_t f_lo_hz, uint32_t f_hi_hz);
void rx_dsp_set_notch(bool on);   // автоматический notch (ON/OFF)
void rx_dsp_set_nr(bool on);  
bool rx_dsp_get_nr(void);
uint32_t rx_dsp_af_lo_hz(void);  
uint32_t rx_dsp_af_hi_hz(void);

// Бортовой RTTY-декодер: единая точка правды, видна и в .ino, и в rx_dsp.cpp.    
#ifndef RTTY_DECODE    
#define RTTY_DECODE 1    
#endif

// Перестройка приёмного тракта на лету (CAT/FLDigi):  
// малая отстройка — сдвиг NCO, большая — полная переустановка LO.  
void     rx_retune(uint32_t freq_hz);  
uint32_t rx_current_hz(void);               // текущая частота приёма, Гц

bool  rx_dsp_get_audio(const int16_t** i, const int16_t** q, uint8_t* n);  
void  rx_dsp_release_audio(void);  
float rx_dsp_audio_fs(void);

void     vfo_quadrature_start(uint32_t freq_hz);  
void     vfo_quadrature_stop(uint32_t freq_hz, double step_hz);  
uint32_t vfo_quadrature_lo_hz(void);   // фактическая квантованная частота LO (для NCO)

void quad_test(uint32_t freq_hz);
void vfo_quadrature_debug(void);


// рабочий лимит как переменные — min(дефайн, flash-значение)
extern uint32_t vfo_max_clk_sys_hz;   // из Flash, паспорт=133 МГц  
extern uint32_t vfo_max_pll_vco_hz;   // из Flash, паспорт=1600 МГц 

// Эффективный лимит поиска: flash-доказанный, но не выше аппаратного барьера  
static inline uint64_t vfo_effective_clk_max(void) {  
    uint64_t lim = (uint64_t)vfo_max_clk_sys_hz;  
    return (lim < VFO_CLK_SYS_MAX_HZ) ? lim : VFO_CLK_SYS_MAX_HZ;  
}  
static inline uint64_t vfo_effective_pll_max(void) {  
    uint64_t lim = (uint64_t)vfo_max_pll_vco_hz;  
    return (lim < PLL_MAX_HZ) ? lim : PLL_MAX_HZ;  
}

struct PllConfig {
    uint32_t fbdiv;
    uint32_t p1;
    uint32_t p2;
    uint64_t clk_sys_hz;
    uint32_t vsel;         
    bool is_oc;    
    uint32_t refdiv;        
};

// --- Архитектурное разделение профилей тактирования ---
extern PllConfig pll_nominal;     // Базовый гражданский режим (133 МГц)
extern PllConfig pll_overclock;   // Активный boost-профиль ТЕКУЩЕГО сеанса передачи
extern PllConfig pll_ceiling;     // Абсолютный жесткий потолок, доказанный OCTEST
extern volatile bool clk_boosted; 

// Авто-BOOST: true, если flash-паспорт выше стандартного номинала (133 МГц / 1600 МГц VCO)  
extern volatile bool vfo_auto_boost;  
  
// true, если во flash записаны частоты выше стандартных (clk_sys > 133 МГц либо VCO > 1600 МГц)  
static inline bool vfo_flash_allows_boost(void) {  
    return (vfo_effective_clk_max() > VFO_CLK_SYS_NOMINAL_HZ) ||  
           (vfo_effective_pll_max() > 1600000000ULL);  
}


struct VfoParameters {
    uint32_t pio_int;
    uint32_t pio_frac;
    uint32_t dds_step;
    uint32_t target_freq_chz; 
};



#define VFO_TONES_COUNT     64 // количество тонов
#define VFO_TONE_NONE      255           

// Паспортный номинал кварца — константа; коррекция живёт во Flash  
// (пишется ключом [XOSC_PPB] в SET.TXT, хранится в SlotMeta).  
#define VFO_XOSC_NOMINAL_HZ     12000000ULL   // кварц 12.000000 МГц    
#define VFO_XOSC_DEFAULT_PPB    (+25000LL)    // дефолт при невалидном flash  
  
// Коррекция опоры в ppb, загружается из Flash (0 после старта до init_file_manager).  
extern int32_t vfo_xosc_corr_ppb;  
  
// Скорректированная частота кварца — считается в рантайме от flash-значения.  
static inline uint64_t vfo_calibrated_xosc_hz(void) {  
    // Всё в int64: знак коррекции должен сохраниться.  
    // uint64-промоушен превращал отрицательный ppb в ~1.8e19.  
    int64_t hz = (int64_t)VFO_XOSC_NOMINAL_HZ +  
                 (int64_t)VFO_XOSC_NOMINAL_HZ * (int64_t)vfo_xosc_corr_ppb / 1000000000LL;  
    return (hz > 0) ? (uint64_t)hz : (uint64_t)VFO_XOSC_NOMINAL_HZ;  
}



extern VfoParameters __attribute__((section(".time_critical.vfo_tones"))) vfo_tones[VFO_TONES_COUNT];
extern int pin_freq_out; 

#define VFO_OUTPUT_PIN       (pin_freq_out)

// Прототипы с аргументами по умолчанию
PllConfig vfo_find_optimal_pll(unsigned int target_frequency_hz, uint64_t max_clk_limit = 0); // 0 = auto

void vfo_hardware_init(unsigned int base_freq_hz, double step_hz);  
void vfo_set_tone_instant(uint8_t tone_index);  
void vfo_operation_set(bool key_down);  
void vfo_set_narrowband(bool on);              // вкл/выкл режим узкой полосы (Olivia): глушит FRAC-дизер  
extern volatile uint8_t vfo_narrowband_mode;   // 1 = дизер принудительно выключен
void vfo_clk_boost_enter(unsigned int target_freq_hz);

void vfo_clk_boost_exit(void);  
void vfo_clk_thermal_guard(void);       
void vfo_find_max_stable_clock(void);  
void vfo_test_pll_extreme_shurm(void);  
  
// Узкополосный режим: при взведённом флаге FRAC-дизер принудительно  
// отключается (чистые дискретные тоны для Olivia MFSK вместо шумовой полки).  
extern volatile uint8_t vfo_narrowband_mode;  
void vfo_set_narrowband(bool on);



#endif // VFO_HARDWARE_H
