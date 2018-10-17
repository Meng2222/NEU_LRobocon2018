#include "key.h"

//行程开关1，切换模式
void KeyOne(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
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
	
}

//行程开关2，倒球
void KeyTwo(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
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
}



