#ifndef _I2C_GPIO_H
#define _I2C_GPIO_H

#include "userinc.h"

void I2C_Init(void);
unsigned char I2C_Write(unsigned char addr, unsigned char reg, unsigned char *data, unsigned char len);
unsigned char I2C_Read(unsigned char addr, unsigned char reg, unsigned char *buf, unsigned char len);

#endif
