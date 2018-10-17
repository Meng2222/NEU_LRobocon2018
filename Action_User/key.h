#ifndef __KEY_H
#define __KEY_H
#include "stm32f4xx_it.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"

#define KEY_STATUS_ONE GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0)
#define KEY_STATUS_TWO GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_1)
void KeyOne(void);
void KeyTwo(void);

#endif


