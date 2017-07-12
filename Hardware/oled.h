#ifndef _OLED_H_
#define _OLED_H_
#include "userinc.h"

typedef uint8_t uint8;
typedef int8_t int8;
typedef int32_t int32;
typedef uint32_t uint32;

#define FALSE 0
#define TRUE 0

/********”√ªß≈‰÷√∂Àø⁄***********/

#define D0_OUT  PDout(8)
#define D1_OUT  PDout(9)
#define RST_OUT PDout(10)
#define DC_OUT  PDout(11)

void OLED_Init(void);
void LCD_CLS(void);
void LCD_ClearLine(u8 y);
void LCD_P6x8Str(u8 x,u8 y,u8 ch[]);
void LCD_P6x8StrW(u8 x,u8 y,u8 ch[]);
void LCD_P8x16Str(u8 x,u8 y,u8 ch[]);
void LCD_P14x16Str(u8 x,u8 y,u8 ch[]);
void LCD_Print(u8 x, u8 y, u8 ch[]);
void LCD_PutPixel(u8 x,u8 y);
void LCD_Rectangle(u8 x1,u8 y1,u8 x2,u8 y2,u8 gif);
void Draw_BMP(u8 x0,u8 y0,u8 x1,u8 y1,u8 bmp[]); 
void LCD_Fill(u8 dat);
void LCD_P16x16Ch(u8 x,u8 y,u8 ch[]);
void LCD_P16x16Str(u8 x,u8 y,u8 ch[]);

void full_bmp(u8 bmp[]);
void full_bmpv(u8 bmp[]);void Draw_BMP(u8 x0,u8 y0,u8 x1,u8 y1,u8 bmp[]); 
void Draw_BMPV(u8 x0,u8 bmp[]); 

void LCD_PrintValueFP(u8 x, u8 y, u16 data, u8 num);
void LCD_PrintValueF(u8 x, u8 y, float data, u8 num);

void LCD_P8x16Str8(u8 x,u8 y,u8 num);
void LCD_P6x8Str8(u8 x,u8 y,u8 num);
void LCD_P8x16Str16(u8 x,u8 y,u16 num);
void LCD_P6x8Str16(u8 x,u8 y,u16 num);

void oledprintf(uint8 x,uint8 y,const char *pFormat, ...);
void oledprintfw(uint8 x,uint8 y,const char *pFormat, ...);
#endif

