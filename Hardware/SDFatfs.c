#include "SDFatfs.h"
#include "userinc.h"

#define READ_PRINT      1
#define ERROR_HANDLER        0
int32_t aa = -12234500;

FATFS fs;                 // Work area (file system object) for logical drive
FIL fil;                  // file objects

uint32_t byteswritten;                /* File write counts */
uint32_t bytesread;                   /* File read counts */
uint8_t rtext[100];                     /* File read buffers */

int SDFatFSInit()
{
  retSD = f_mount(&fs, "", 0);
  if(retSD)
  {
    printf("mount error : %d \r\n",retSD);
#if ERROR_HANDLER
    Error_Handler();
#endif
  }
  else
    printf("mount sucess!!! \r\n");
  return retSD;
}

int SDFatFSOpen(char *filename)
{
  for(int i=0;i<5;i++)
  {
    retSD = f_open(&fil, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if(retSD && i == 4)
    {
      printf(" open file error : %d \r\n",retSD);
#if ERROR_HANDLER
      Error_Handler();
#endif
    }
    else if (!retSD)
    {
      printf(" open file success!\r\n");
      retSD = f_lseek(&fil, f_size(&fil));
      break;
    }
  }
  return retSD;
}

int SDFatFsClose()
{
  for(int i=0;i<5;i++)
  {
    retSD = f_close(&fil);
    if(retSD && i == 4)
    {
      printf(" close file error : %d \r\n",retSD);
#if ERROR_HANDLER
      Error_Handler();
#endif
    }
    else if (!retSD)
    {
//      printf(" close file success!\r\n");
      break;
    }
  }
  return retSD;
}

int SDFatFSRead(char *filename)
{
  DWORD p_file;
  retSD = f_open(&fil, filename, FA_READ);
  if(retSD)
  {
    printf(" open file error : %d \r\n",retSD);
#if ERROR_HANDLER
    Error_Handler();
#endif
  }
  else
  {
//    printf(" open file success!\r\n");
    p_file = 0;
    f_lseek(&fil, p_file);
  }

#if READ_PRINT
  __disable_irq();
  while (f_gets((char*)rtext, sizeof rtext, &fil)) 
  {
    printf((char const *)rtext);
  }
  printf("\r\nNo.%s\r\n", filename);
  __enable_irq();
#endif
  
  retSD = f_close(&fil);
  return retSD;
}