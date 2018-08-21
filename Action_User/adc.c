#include "adc.h"
#include "includes.h"
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_adc.h"
#include "moveBase.h"
//???ADC															   
void  Adc_Init(void)
{    
  GPIO_InitTypeDef  GPIO_InitStructure={0};
	ADC_CommonInitTypeDef ADC_CommonInitStructure={0};
	ADC_InitTypeDef       ADC_InitStructure={0};
//	ADC_StructInit(&ADC_InitStructure);
//  ADC_CommonStructInit(&ADC_CommonInitStructure);

	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//??GPIOC??
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //??ADC1??

  //????ADC1??5 IO?
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4| GPIO_Pin_5;//PC14,PC15 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//????
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//?????
  GPIO_Init(GPIOC, &GPIO_InitStructure);//???  
 
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1??
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//????	 
 
	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//????
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//???????????5???
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA??
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//???4???ADCCLK=PCLK2/4=84/4=21Mhz,ADC????????36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//???
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12???
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//?????	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//??????
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//??????,??????
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//???	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1????????? ??????????1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC???
	
 
	ADC_Cmd(ADC1, ENABLE);//??AD???	

}				  
//??ADC?
//ch: @ref ADC_channels 
//??? 0~16?????:ADC_Channel_0~ADC_Channel_16
//???:????
u16 Get_Adc(u8 ch)   
{
	//????ADC??????,????,????
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC??,480???,?????????????			    
  
	ADC_SoftwareStartConv(ADC1);		//?????ADC1?????????	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//??????

	return ADC_GetConversionValue(ADC1);	//??????ADC1????????
}
//????ch????,?times?,???? 
//ch:????
//times:????
//???:??ch?times????????
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
	}
	return temp_val/times;
} 
