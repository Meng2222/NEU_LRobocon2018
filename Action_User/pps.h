#ifndef __PPS_H
#define __PPS_H
#include "stdint.h"

/*接受几个来自定位系统的float数据*/
#define GET_PPS_VALUE_NUM      6
/*接受几个来自定位系统的uint8_t数据*/ /* 6 * 4byte = 24*/
#define GET_PPS_DATA_NUM       24

/**
 * [ppsInit定位系统初始化]
 */
void ppsInit(void);

void USART3_IRQHandler(void);

void TalkOpsToGetReady(void);
/*初始化并且让程序等待定位系统发数*/
void WaitOpsPrepare(void);

void SetOpsReady(uint8_t flag);

/*定位系统准备完毕*/
uint8_t GetOpsReady(void);


#endif 

