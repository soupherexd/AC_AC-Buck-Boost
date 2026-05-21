#ifndef __OLED_H
#define __OLED_H

#include "DSP28x_Project.h"
#include "iic.h"

#define Uchar unsigned char
#define Uint  unsigned int
#define Ulong unsigned long
#define uchar unsigned char
#define uint  unsigned int

void OLED_Init(void);
void OLED_WriteData(Uchar Data);
void OLED_WriteCommand(Uchar Data);
void OLED_SetPos(Uchar x,Uchar page);
void OLED_Clean(void);
void OLED_Fill(void);
void OLED_ShowChar(Uchar x,Uchar y,char chr);
void OLED_ShowString(Uchar x,Uchar y,char *s);
void OLED_ShowNum(Uchar x,Uchar y,int n,Uchar len);
void OLED_ShowFloat(Uint8 x,Uint8 y,float num,Uint8 n);
void num2char(unsigned char *str, double number, Uint8 g, Uint8 l);
void OLED_OFF(void);
void OLED_ON(void);
void OLED_ShowCHinese(Uchar x,Uchar y,Uchar no);
void OLED_DrawPoint(Uchar x,Uchar y);
void OLED_ErasePoint(Uchar x,Uchar y);
void OLED_DrawSin(float32 Freq, Uchar Phi);

#endif
