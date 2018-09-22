#include "key.h"

//行程开关1，切换模式
void KeyOne(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	//KEY引脚初始化设置
	//引脚PE0
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	//输入模式
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
	//GPIO速度
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	//上拉
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
	//初始化
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);
	
	EXTI_InitStructure.EXTI_Line=EXTI_Line0;

	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;

	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;

	EXTI_InitStructure.EXTI_LineCmd = ENABLE;

	EXTI_Init(&EXTI_InitStructure);             //初始化外设EXTI寄存器
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//外ShooterVelCtrl//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =4;		//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}

//行程开关2，倒球
void KeyTwo(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	//KEY引脚初始化设置
	//引脚PE1
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
	//输入模式
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
	//GPIO速度
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	//上拉
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
	//初始化
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);
	
	EXTI_InitStructure.EXTI_Line=EXTI_Line1;

	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;

	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;

	EXTI_InitStructure.EXTI_LineCmd = ENABLE;

	EXTI_Init(&EXTI_InitStructure);             //初始化外设EXTI寄存器
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;//外ShooterVelCtrl//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}



