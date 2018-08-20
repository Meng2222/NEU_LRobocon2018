#ifndef __ADC_H
#define __ADC_H	
#include "stm32f4xx_adc.h"
 							   
void Adc_Init(void); 				//ADC通道初始化
u16  Get_Adc(u8 ch); 				//获得某个通道值 
u16 Get_Adc_Average(u8 ch,u8 times);//得到某个通道给定次数采样的平均值  
void GetDirection(uint8_t *getFlag);

#endif 







