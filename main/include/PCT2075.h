#ifndef PCT2075_H
#define PCT2075_H

#include "esp_err.h"
#define PCT2075_ADDR 0x37
#define PCT2075_REG_CONF 0x01
#define PCT2075_REG_TEMP 0x00
#define PCT2075_REG_TOS 0x03
#define PCT2075_REG_THYST 0x02
#define PCT2075_REG_TIDLE 0x04

esp_err_t pct2075_read_temp(float *temperature);

#endif