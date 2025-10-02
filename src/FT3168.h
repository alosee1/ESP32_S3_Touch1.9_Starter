#ifndef FT3168_H
#define FT3168_H
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif 
void i2c_master_init(void);
void touch_init(void);

uint8_t getTouch(uint16_t *x,uint16_t *y);
#ifdef __cplusplus
}
#endif
#endif