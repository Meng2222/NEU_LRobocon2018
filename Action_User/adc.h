#ifndef __ADC_H
#define __ADC_H	
#include "stm32f4xx_adc.h"

/////////////////////////////////////////////////////////////////////////////////	 
//STM32F4????-?????
//????:http://mcudev.taobao.com								  
////////////////////////////////////////////////////////////////////////////////// 	 
 							   
void Adc_Init(void); 				//ADC?????
u16  Get_Adc(u8 ch); 				//??????? 
u16 Get_Adc_Average(u8 ch,u8 times);//????????????????  
#endif
