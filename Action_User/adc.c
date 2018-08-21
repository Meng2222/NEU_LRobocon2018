#include "adc.h"


//初始化ADC															   
void  Adc_Init(void)
{    
    GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
    
    ADC_StructInit(&ADC_InitStructure);
    ADC_CommonInit(&ADC_CommonInitStructure);
    
	//使能GPIOC时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    //使能ADC1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 

    //先初始化ADC1通道10 11 IO口
    //PC0 通道10 11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    //模拟输入
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    //不带上下拉
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    //初始化
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);
    //复位结束	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE); 
 
	//独立模式
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    //两个采样阶段之间的延迟5个时钟
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    //DMA失能
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; 
    //预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; 
    //初始化
    ADC_CommonInit(&ADC_CommonInitStructure);
	//12位模式
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    //非扫描模式	
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    //关闭连续转换
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    //禁止触发检测，使用软件触发
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    //右对齐	
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    //1个转换在规则序列中 也就是只转换规则序列1 
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    //ADC初始化
    ADC_Init(ADC1, &ADC_InitStructure);
	
    //开启AD转换器
	ADC_Cmd(ADC1, ENABLE);	

}				  
//获得ADC值
//ch: @ref ADC_channels 
//通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//返回值:转换结果
u16 Get_Adc(u8 ch)   
{
	//设置指定ADC的规则组通道，一个序列，采样时间
    //ADC1,ADC通道,480个周期,提高采样时间可以提高精确度	
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );		    
    //使能指定的ADC1的软件转换启动功能	
	ADC_SoftwareStartConv(ADC1);
	//等待转换结束
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ) != SET);
	//返回最近一次ADC1规则组的转换结果
	return ADC_GetConversionValue(ADC1);
}


//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val = 0;
	for(uint8_t t=0;t<times;t++)
	{
		temp_val += Get_Adc(ch);
	}
	return temp_val/times;
} 
	 









