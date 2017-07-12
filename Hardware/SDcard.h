#ifndef _SDCARD_H
#define _SDCARD_H

#include "stm32f4xx_hal.h"
#include "sdio.h"


void SDInit();
void testSD();

extern volatile uint8_t read_flag,write_flag;





#endif