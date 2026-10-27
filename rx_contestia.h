#pragma once

#include <stdint.h>
#include <stdbool.h>

// CONTESTIA 8/250
void rx_contestia_init(float center_hz);
void rx_contestia_push(int16_t sample);
void rx_contestia_stop(void);
bool rx_contestia_active(void);
