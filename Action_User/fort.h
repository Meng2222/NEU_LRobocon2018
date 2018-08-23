#ifndef FORT__H
#define FORT__H

#include "stdint.h"

/**
* @brief 炮台航向控制
* @param  ang:转台航向角度，范围为0~360度
* @retval none
* @attention none
*/
void YawPosCtrl(float ang);

/**
* @brief 发射电机转速控制
* @param  rps:发射电机速度，单位转每秒
* @retval none
* @attention none
*/
void ShooterVelCtrl(float rps);

/**
* @brief 接收炮台返回的数据
* @param data：串口每次中断接收到的一字节数据
* @retval none
* @attention 该函数请插入到对应的串口中断中
							注意清除标志位
*/
void GetValueFromFort(uint8_t data);

/**
 * [获得发射系统的各种值QAQ]
 */
float ReadShooterVel(void);
float ReadYawPos(void);
float ReadLaserAValue(void);
float ReadLaserBValue(void);

#endif
