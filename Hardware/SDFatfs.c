#include "SDFatfs.h"
#include "userinc.h"

#define READ_PRINT      1
int32_t aa = -12234500;

FATFS fs;                 // Work area (file system object) for logical drive
FIL fil;                  // file objects

uint32_t byteswritten;                /* File write counts */
uint32_t bytesread;                   /* File read counts */
uint8_t rtext[100];                     /* File read buffers */

void SDFatFSInit()
{
  retSD = f_mount(&fs, "", 0);
  if(retSD)
  {
    printf("mount error : %d \r\n",retSD);
    Error_Handler();
  }
  else
    printf("mount sucess!!! \r\n");
}

void SDFatFSOpen(char *filename)
{
  retSD = f_open(&fil, filename, FA_OPEN_ALWAYS | FA_WRITE);
  if(retSD)
  {
    printf(" open file error : %d \r\n",retSD);
    Error_Handler();
  }
  else
  {
    printf(" open file success!\r\n");
    retSD = f_lseek(&fil, f_size(&fil));
  }
}

void SDFatFClose()
{
  retSD = f_close(&fil);
  if(retSD)
  {
    printf(" close file error : %d \r\n",retSD);
    Error_Handler();
  }
  else
  {
    printf(" close file success!\r\n");
  }
}

void SDFatFSRead(char *filename)
{
  DWORD p_file;
  retSD = f_open(&fil, filename, FA_READ);
  if(retSD)
  {
    printf(" open file error : %d \r\n",retSD);
    Error_Handler();
  }
  else
  {
    printf(" open file success!\r\n");
    p_file = 0;
    f_lseek(&fil, p_file);
  }
  
  __disable_irq();
  while (f_gets((char*)rtext, sizeof rtext, &fil)) 
  {
    printf((char const *)rtext);
  }
  __enable_irq();
  
  retSD = f_close(&fil);
}