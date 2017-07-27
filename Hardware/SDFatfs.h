#ifndef _SDFATFS_H
#define _SDFATFS_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "sdio.h"

extern FATFS fs;                 // Work area (file system object) for logical drive
extern FIL fil;                  // file objects

int SDFatFSInit();
int SDFatFSOpen(char *);
int SDFatFsClose();
int ParaWrite();
int SDFatFSRead(char *);

#endif