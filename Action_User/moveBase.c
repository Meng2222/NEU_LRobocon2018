/**
  ******************************************************************************
  * @file	  moveBase.c
  * @author	  Action
  * @version   V1.0.0
  * @date	  2018/08/09
  * @brief	 2018省赛底盘运动控制部分
  ******************************************************************************
  * @attention
  *			None
  ******************************************************************************
  */
/* Includes -------------------------------------------------------------------------------------------*/

  #include "moveBase.h"
	#include "elmo.h"
	#include "math.h"
	#include "usart.h"
	#include "elmo.h"
	#include "pps.h"
/* Private typedef ------------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------------*/
//正方向为等边三角形（金字塔）（顶点为逆时针1 2 3） 与底边平行水平向右
/**
  * @brief  开环直线
  * @note
  * @param  
  * @retval None
  */
void OpenLoopLine(double V,float Ang)	//units : ° 
{
	double V1 = V * cos(Ang * PI / 180);
	double V3 = V * cos((120 - Ang) * PI / 180);
	double V2 = V * cos((120 + Ang) * PI / 180);
	
	VelCrl(CAN2,1,V1);
	VelCrl(CAN2,2,V2);
	VelCrl(CAN2,3,V3);
}
/**
  * @brief  开环圆
  * @note
  * @param  
  * @retval None
  */
int OpenLoopCircle(float velRotate,float cenX,float cenY)
{
	double R = fabs(sqrt(pow(cenX,2)+pow(cenY,2)));
	double W = velRotate / R ; //units : ° 
	double d1 = -fabs(cenY - R_BASE);
	double d2 = (-sqrt(3.0) * cenX - cenY - 2 *  R_BASE)/(sqrt(pow(sqrt(3.0),2) + 1));
	double d3 = ( sqrt(3.0) * cenX - cenY - 2 *  R_BASE)/(sqrt(pow(sqrt(3.0),2) + 1));
		
	double V1 = W * d1;
	double V2 = W * d2;
	double V3 = W * d3;
	
	USART_OUT(USART1,(uint8_t*)"X=\t%d\tY=\t%d\tAng=\t%d\r\n",(int)V1,(int)V2,(int)V3);

	VelCrl(CAN2,1,V1);
	VelCrl(CAN2,2,V2);
	VelCrl(CAN2,3,V3);
}

/**
* @brief  点到直线距离计算
* @param  ward:直线方向
* @param  pointPosX：直线上任意一点x坐标
* @param  pointPosY：直线上任意一点y坐标
* @param  curPosX：当前实际的横坐标
* @param  curPosY：当前实际的纵坐标
* @retval disPointToLine： 距离
* @author ACTION
*/
float CalcDisPointToLine(float ward, float pointPosX, float pointPosY,
	                     float curPosX, float curPosY)
{
	float errPos = 0.0f;
	float dirX = 0.0f, dirY = 0.0f;//方向向量
	float actX = 0.0f, actY = 0.0f;
	int checkResult = 0;
	float verticalWard = 0.0f;
	//计算目标直线垂线所在方向
	verticalWard = ward + 90;
	
	if (verticalWard > 180)
		verticalWard = verticalWard - 360;
	if (verticalWard < -180)
		verticalWard = verticalWard + 360;

	verticalWard = (float)ANGTORAD(verticalWard);

	/* 点到直线距离计算(向量法)*/

	dirX = (float)cos(verticalWard);  
	dirY = (float)sin(verticalWard);
	actX = pointPosX - curPosX;
	actY = pointPosY - curPosY;

	errPos = dirX*actX + dirY*actY;
		
	return errPos; 
}
//float CalcDisPointToLine(float ward, float pointPosX, float pointPosY,
//	                     float curPosX, float curPosY)
//{
//	float errX = 0.0,errY = 0.0;
//	float errPos = 0.0;
//	float verticalWard = 0.0f;
//	
//	//计算目标直线垂线所在方向
//	verticalWard = ward + 90;
//	
//	if (verticalWard > 180)
//			verticalWard = verticalWard - 360;
//	if (verticalWard < -180)
//			verticalWard = verticalWard + 360;
//		
//	verticalWard = verticalWard * PI / 180;
//	
//	errX = pointPosX - curPosX;
//	errY = pointPosY - curPosY;
//	
//	errPos = cos(verticalWard)*errX + sin(verticalWard)*errY;
//	
//	if(fabs(ward)<90)return errPos;                
//	if(fabs(ward)>=90) return -errPos;
//}


/**
* @brief  直线闭环
* @param  vel:车的速度（正数 Vel>0）
* @param  ward：前进方向
          取值范围：-180到+180
* @param  exPos ： 机器人姿态角度参考值
* @param  actPos：机器人姿态角度实际值
* @param  curPosX:底盘实际坐标
* @param  curPosY:底盘实际坐标
* @retval OUTOFRANGE：输入参数超出范围
  @retval INVALID_PARAMETER：输入参数无效
  @retval RETURN_OK：函数正常运行
* @author ACTION
*/
#define KP_POS 0.005f
#define KI_POS 0.0f
#define KD_POS 0.0f

//#define KP_POS 0.0f
//#define KI_POS 0.0f
//#define KD_POS 0.0f

//#define KP_ANG 0.0f
//#define KI_ANG 0.0f
//#define KD_ANG 0.0f

#define KP_ANG 2.0f
#define KI_ANG 0.0f
#define KD_ANG 0.0f

//限制角度闭环的PID输出最大值
#define MAXANGPIDOUT   180.0f  
//限制位置闭环的PID输出最大值
#define MAXPOSPIDOUT   1.0f 

static float posErrSum = 0.0f, angErrSum = 0.0f;
static float posErrLast = 0.0f, angErrLast = 0.0f; 

//位置闭环

void CloseLoopLine(float vel, float ward, float exPos, 
							float actPos, float curPosX, float curPosY, float pointOnLineX, float pointOnLineY)
{
	
	float errPos = 0.0f, errPosDif = 0.0f;
	float errAng = 0.0f, errAngDif = 0.0f;
	
	float angPidOut = 0.0f;
	float posPidout = 0.0f;

	float curVel1 = 0.0f , curVel2 = 0.0f , curVel3 = 0.0f;
	
	int checkResult = 0 , signForPosition = 0 , errPosSign = 0;
	
	static wheelSpeed_t speedOut = {0.0,0.0,0.0,0.0}, 
	                    speedBack =  {0.0,0.0,0.0,0.0};
	motorAcc_t wheelAcc;
	
	//计算位置的误差量（距离）										
	errPos = CalcDisPointToLine(ward, pointOnLineX, pointOnLineY, curPosX, curPosY);	//计算位置误差的积分（累积量）			
	
	if(errPos > 0)
	{
		errPosSign = 90;
	}	
	else if(errPos < 0)
	{
		errPosSign = -90;
		errPos = -errPos;
	}
	
	posErrSum += errPos;
	//计算位置误差的微分										
	errPosDif = errPos - posErrLast;
	posErrLast = errPos;
	
	//计算PID调节量
	posPidout = KP_POS*errPos + KI_POS*posErrSum + KD_POS*errPosDif;
	
			/* 最大输出限制 */
	if (posPidout > MAXPOSPIDOUT)
	{
		posPidout = MAXPOSPIDOUT;
	}
	else if (posPidout < -MAXPOSPIDOUT)
	{
		posPidout = -MAXPOSPIDOUT;
	}
	
	/* 计算角度PID三个环节 */
	//计算角度的误差量											
	errAng = exPos - actPos;
	//计算角度误差的积分（累积量）											
	angErrSum += errAng;
	//计算角度误差的微分											
	errAngDif = actPos - angErrLast;
	angErrLast = 	actPos;
	//计算PID调节量
	angPidOut = KP_ANG*errAng + KI_ANG*angErrSum + KD_ANG*errAngDif;
	
	if (angPidOut > MAXANGPIDOUT)
	{
		angPidOut = MAXANGPIDOUT;
	}
	else if (angPidOut < -MAXANGPIDOUT)
	{
		angPidOut = -MAXANGPIDOUT;
	}
	//计算直线平移与旋转速度
	speedOut = CalcVel(vel,ward,angPidOut,actPos);
	
	//计算垂直直线方向速度

	 if((float)(ward + errPosSign) > 180.0f )
	 {
		 ward -= 360.0f;
	 }
	 if((float)(ward + errPosSign) < -180.0f )
	 {
		 ward += 360.0f;
	 }
		speedBack = CalcVel(posPidout,(float)(ward + errPosSign),0,actPos);

	USART_OUT(USART1,(uint8_t*)"%d\t%d\t",(int)(posPidout * 100),(int)(angPidOut));

	speedOut.v1 += speedBack.v1;
	speedOut.v2 += speedBack.v2;
	speedOut.v3 += speedBack.v3;
	
	VelCrl(CAN2,1,speedOut.v1);        
	VelCrl(CAN2,2,speedOut.v2);     
	VelCrl(CAN2,3,speedOut.v3);
}

/**
* @brief  计算三轮速度
* @param  vel:车的和速度（正数 Vel>0）
* @param  ward:车的行进方向
          取值范围：-180到+180
* @param  velRotate:车自身的旋转速度（正数时右旋 俯视图）
* @param  selfPos:车自身的姿态角度
* @retval wheelSpeed:底盘应该达到的速度
* @note   根据车速计算轮速的具体分解方法参见文档
* @author ACTION
*/

wheelSpeed_t CalcVel(float vel, float ward, float velRotate,float selfPos)
{
	wheelSpeed_t speedOut;
	
	/* 计算当底盘旋转后底盘应该的前进方向 */
	//ward = ward - (selfPos + 90); 
	if (ward > 180)
		ward = ward - 360;
	if (ward < -180)
		ward = ward + 360;
	/* 分解计算出三个轮的速度 */
	speedOut.v3 = VELTOPULSE(vel) * cosf(2 * PI / 3 - ANGTORAD(ward)) - VELTOPULSE(MOVEBASERADIUS * ANGTORAD(velRotate));
	speedOut.v2 = VELTOPULSE(vel) * cosf(2 * PI / 3 + ANGTORAD(ward)) - VELTOPULSE(MOVEBASERADIUS * ANGTORAD(velRotate));
	speedOut.v1 = VELTOPULSE(vel) * cosf(ANGTORAD(ward)) - VELTOPULSE(MOVEBASERADIUS * ANGTORAD(velRotate)); 

	return speedOut;
}

/**
* @brief  设定圆心旋转(闭环)
* @param  velRotate：旋转线速度（顺时针为正，逆时针为负）
* @param  cenX ： 圆心横坐标
* @param  cenY ： 圆心纵坐标
* @param  beginWard ：初始位置圆的切线方向
* @param  endWard： 结束位置圆的切线方向
* @param  exPos ： 期望的机器人姿态角度
* @param  actPos ： 机器人自身的实际角度
* @param  rRotate ：期望的圆半径
* @retval OUTOFRANGE：输入参数超出范围
  @retval INVALID_PARAMETER：输入参数无效
  @retval RETURN_OK：函数正常运行
* @author ACTION
*/
void CloseLoopCircle(float velRotate, float cenX, float cenY,
					float beginWard, float endWard, float exPos,
				    float actPos, float rRotate)
{
	int checkResult = 0;
	int rotationDirection = 0 ;
	float actRadius = 0.0f, angle = 0.0f, wardAdd = 0.0f, disX = 0.0f, 
		  disY = 0.0f, pointOnTargetLineX = 0.0f, pointOnTargetLineY = 0.0f, temAngle = 0.0f;       //fix me   
	static int signForStop = 0;
	
	//参数合法性检验
	if(velRotate >= 0)
	{
		if(beginWard >= -180 && beginWard <= 180 && exPos >= -180 
		   && exPos <= 180 && endWard >= -180 && endWard <= 180)
		{
			checkResult = 1;
		}
	}
	
  rotationDirection = velRotate > 0 ? 1 : -1;

	//计算到圆心X方向距离和Y方向距离
	disX = GetX() - cenX;
	disY = GetY() - cenY;
		
	//计算与X轴正方向角度
	temAngle = RADTOANG(atan2(disY,disX));
	//计算所在角度对应弧上点坐标
	pointOnTargetLineX = cenX + rRotate * cos(ANGTORAD(temAngle));
	pointOnTargetLineY = cenY + rRotate * sin(ANGTORAD(temAngle));
    //计算转动的切线方向
	angle = temAngle - 90;
	if(rotationDirection < 0) angle += 180;
	if(angle > 180) angle -= 360;
	if(angle < -180) angle += 360;
	//设置弧上点坐标
	if(checkResult == 1)
	{
		if(signForStop==1)
		{
			VelCrl(CAN2,1,0);
			VelCrl(CAN2,2,0);
			VelCrl(CAN2,3,0);
		}
		else
		{
			CloseLoopLine((float)velRotate * (float)rotationDirection, angle, exPos, GetAngle(), GetX(), GetY(), pointOnTargetLineX, pointOnTargetLineY);	
		}
	}		
}
/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
