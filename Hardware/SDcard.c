#include "SDcard.h"
#include "userinc.h"

#define BLOCK_SIZE 512
#define DATA_SIZE 128
int32_t SD_Data_Tx[DATA_SIZE] = {-6661,6662,6663,6664,-6665};
int32_t SD_Data_Rx[DATA_SIZE];
uint8_t SD_Buffer_Tx[BLOCK_SIZE];
uint8_t SD_Buffer_Rx[BLOCK_SIZE];
HAL_SD_CardStateTypeDef card_status;
static uint32_t start_block=0;
static uint32_t sd_block,sd_num=0;
volatile uint8_t read_flag,write_flag = 0;


void SDInit()
{
  int err;
  err = HAL_SD_InitCard(&hsd);
  if(!err)
  {
    HAL_SD_CardInfoTypeDef SDCardInfo;
    HAL_SD_GetCardInfo(&hsd,&SDCardInfo);
    printf(" CardCapacity  : %u MB \r\n",SDCardInfo.BlockNbr*SDCardInfo.BlockSize/1024/1024);
    printf(" CardBlockSize : %d \r\n",SDCardInfo.BlockSize);
    printf(" RCA           : %d \r\n",SDCardInfo.RelCardAdd);
    printf(" CardType      : %d \r\n",SDCardInfo.CardType);
  }
  else 
    printf("\n\rSD failed:%d\n\r",err);
}



void WriteSDBlock(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd)
{
  if(HAL_SD_WriteBlocks_DMA(hsd,pData,BlockAdd,1) == HAL_OK)
  {
    card_status = HAL_SD_GetCardState(hsd);
    write_flag = 0;
  }
}

void ReadSDBlock(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd)
{
  if(HAL_SD_ReadBlocks_DMA(hsd,pData,BlockAdd,1) == HAL_OK)
  {
    card_status = HAL_SD_GetCardState(hsd);
    read_flag = 0;
  }
}

void WriteSDData32(SD_HandleTypeDef *hsd, int32_t *pData, uint32_t BlockAdd)
{
  for(int aa = 0;aa<BLOCK_SIZE; aa++)
  {
    SD_Buffer_Tx[aa] = 0xff & pData[aa/4];
    pData[aa/4] = pData[aa/4]>>8;
  }
  WriteSDBlock(hsd,SD_Buffer_Tx,BlockAdd);
}

void ReadSDData32(SD_HandleTypeDef *hsd, int32_t *pData, uint32_t BlockAdd)
{
  int32_t buff = 0;
  ReadSDBlock(hsd,SD_Buffer_Rx,BlockAdd);
  while(!read_flag);
  for(int aa = 0;aa<BLOCK_SIZE; aa++)
  {
    buff |= SD_Buffer_Rx[aa] << 8*(aa%4);
    if(aa%4 == 3)
    {
      pData[aa/4] = buff;
      buff = 0;
    }
  }
}

void WriteSD(int32_t *pData, uint32_t StartAdd, uint16_t lenth)
{
  static uint32_t addr;
  
  if(addr != StartAdd)
  {
    if(sd_num != DATA_SIZE-1)
      WriteSDData32(&hsd, SD_Data_Tx, sd_block+addr);
    sd_block = sd_num = 0;
  }
  
  for(int i=sd_block;i<(sd_num+lenth-1)/DATA_SIZE+1;i++)
  {
    if(i<(sd_num+lenth-1)/DATA_SIZE)
    {
      for(int j=sd_num;j<DATA_SIZE;j++)
      {
        SD_Data_Tx[j] = pData[j];
        WriteSDData32(&hsd, SD_Data_Tx, i+StartAdd);
      }
    }
    else
    {
      for(int j=0;j<(sd_num+lenth)%DATA_SIZE;j++)
      {
        SD_Data_Tx[j] = pData[j];
        if(j == lenth%DATA_SIZE - 1)
          WriteSDData32(&hsd, SD_Data_Tx, i+StartAdd);
      }
    }
  }
  
  sd_block = lenth/DATA_SIZE;
  sd_num = lenth%DATA_SIZE;
  addr = StartAdd;
  
}

void testSD()
{
  for(int i =0;i<400;i++)
    WriteSDData32(&hsd,SD_Data_Tx,start_block);
  
}