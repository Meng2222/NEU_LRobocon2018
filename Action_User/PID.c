#include "PID.h"
#include "fort.h"
u8 GetBallColor(void);
extern FortType fort;
extern GunneryData Gundata;
extern ScanData Scan;
extern u8 ballcolor;
Line_Value Line_N[54];
Arc_Value Arc_N[12];
Coordinate_Value Coordinate_N[12];
float ABS(float thing)                                                       //浮点绝对值函数
{
	if(thing > 0) return thing;
	else return (0-thing);
}

float Compare(float a1,float b1)                                             //比较函数，输出1或-1
{
	if(a1>b1) return 1.0f;
	else return -1.0f;
}

u8 Max(u8 a1,u8 a2,u8 a3)                                                    //比较函数
{
	u8 max = 0;
	u8 flag = 0;
	if(a1 >= a2) {max = a1; flag = 0;}
	else {max = a2; flag = 1;}
	if(max < a3) {max = a3; flag = 2;}
	return flag;
}

float constrain(float amt, float high, float low)                            //浮点数限幅函数
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

void Init_PID(PID_Value *pid_init)                                           //PID参数初始化（重开pid时需要用到）
{
	pid_init->Angle_Last = pid_init->Angle;
	pid_init->ITerm = pid_init->vel;
}

void PID_Line_Init(void)                                                     //直线参数初始化
{
	Line_N[0].x1 = 600;
	Line_N[0].y1 = 1800;
	Line_N[0].x2 = 600;
	Line_N[0].y2 = 3000;
	Line_N[0].line_kp = 20;
	Line_N[0].line_A = 0;
	Line_N[0].line_B = 0;
	Line_N[0].line_C = 0;
	Line_N[0].line_Angle = 0;
	Line_N[0].line_Error = 0;
	Line_N[0].line_Priority = 0;
	
	Line_N[1].x1 = 600;
	Line_N[1].y1 = 3000;
	Line_N[1].x2 = -600;
	Line_N[1].y2 = 3000;
	Line_N[1].line_kp = 20;
	Line_N[1].line_A = 0;
	Line_N[1].line_B = 0;
	Line_N[1].line_C = 0;
	Line_N[1].line_Angle = 0;
	Line_N[1].line_Error = 0;
	Line_N[1].line_Priority = 0;
	
	Line_N[2].x1 = -600;
	Line_N[2].y1 = 3000;
	Line_N[2].x2 = -600;
	Line_N[2].y2 = 1800;
	Line_N[2].line_kp = 20;
	Line_N[2].line_A = 0;
	Line_N[2].line_B = 0;
	Line_N[2].line_C = 0;
	Line_N[2].line_Angle = 0;
	Line_N[2].line_Error = 0;
	Line_N[2].line_Priority = 0;
	
	Line_N[3].x1 = -600;
	Line_N[3].y1 = 1800;
	Line_N[3].x2 = 600;
	Line_N[3].y2 = 1800;
	Line_N[3].line_kp = 20;
	Line_N[3].line_A = 0;
	Line_N[3].line_B = 0;
	Line_N[3].line_C = 0;
	Line_N[3].line_Angle = 0;
	Line_N[3].line_Error = 0;
	Line_N[3].line_Priority = 0;
	
	Line_N[4].x1 = 1000;
	Line_N[4].y1 = 1400;
	Line_N[4].x2 = 1000;
	Line_N[4].y2 = 3400;
	Line_N[4].line_kp = 20;
	Line_N[4].line_A = 0;
	Line_N[4].line_B = 0;
	Line_N[4].line_C = 0;
	Line_N[4].line_Angle = 0;
	Line_N[4].line_Error = 0;
	Line_N[4].line_Priority = 1;
	
	Line_N[5].x1 = 1000;
	Line_N[5].y1 = 3400;
	Line_N[5].x2 = -1000;
	Line_N[5].y2 = 3400;
	Line_N[5].line_kp = 20;
	Line_N[5].line_A = 0;
	Line_N[5].line_B = 0;
	Line_N[5].line_C = 0;
	Line_N[5].line_Angle = 0;
	Line_N[5].line_Error = 0;
	Line_N[5].line_Priority = 1;
	
	Line_N[6].x1 = -1000;
	Line_N[6].y1 = 3400;
	Line_N[6].x2 = -1000;
	Line_N[6].y2 = 1400;
	Line_N[6].line_kp = 20;
	Line_N[6].line_A = 0;
	Line_N[6].line_B = 0;
	Line_N[6].line_C = 0;
	Line_N[6].line_Angle = 0;
	Line_N[6].line_Error = 0;
	Line_N[6].line_Priority = 1;
	
	Line_N[7].x1 = -1000;
	Line_N[7].y1 = 1400;
	Line_N[7].x2 = 1000;
	Line_N[7].y2 = 1400;
	Line_N[7].line_kp = 20;
	Line_N[7].line_A = 0;
	Line_N[7].line_B = 0;
	Line_N[7].line_C = 0;
	Line_N[7].line_Angle = 0;
	Line_N[7].line_Error = 0;
	Line_N[7].line_Priority = 1;
	
	Line_N[8].x1 = 1400;
	Line_N[8].y1 = 1000;
	Line_N[8].x2 = 1400;
	Line_N[8].y2 = 3800;
	Line_N[8].line_kp = 20;
	Line_N[8].line_A = 0;
	Line_N[8].line_B = 0;
	Line_N[8].line_C = 0;
	Line_N[8].line_Angle = 0;
	Line_N[8].line_Error = 0;
	Line_N[8].line_Priority = 1000;
	
	Line_N[9].x1 = 1400;
	Line_N[9].y1 = 3800;
	Line_N[9].x2 = -1400;
	Line_N[9].y2 = 3800;
	Line_N[9].line_kp = 20;
	Line_N[9].line_A = 0;
	Line_N[9].line_B = 0;
	Line_N[9].line_C = 0;
	Line_N[9].line_Angle = 0;
	Line_N[9].line_Error = 0;
	Line_N[9].line_Priority = 1000;
	
	Line_N[10].x1 = -1400;
	Line_N[10].y1 = 3800;
	Line_N[10].x2 = -1400;
	Line_N[10].y2 = 1000;
	Line_N[10].line_kp = 20;
	Line_N[10].line_A = 0;
	Line_N[10].line_B = 0;
	Line_N[10].line_C = 0;
	Line_N[10].line_Angle = 0;
	Line_N[10].line_Error = 0;
	Line_N[10].line_Priority = 1000;
	
	Line_N[11].x1 = -1400;
	Line_N[11].y1 = 1000;
	Line_N[11].x2 = 1400;
	Line_N[11].y2 = 1000;
	Line_N[11].line_kp = 20;
	Line_N[11].line_A = 0;
	Line_N[11].line_B = 0;
	Line_N[11].line_C = 0;
	Line_N[11].line_Angle = 0;
	Line_N[11].line_Error = 0;
	Line_N[11].line_Priority = 1000;
	
	Line_N[12].x1 = 1800;
	Line_N[12].y1 = 600;
	Line_N[12].x2 = 1800;
	Line_N[12].y2 = 4200;
	Line_N[12].line_kp = 20;
	Line_N[12].line_A = 0;
	Line_N[12].line_B = 0;
	Line_N[12].line_C = 0;
	Line_N[12].line_Angle = 0;
	Line_N[12].line_Error = 0;
	Line_N[12].line_Priority = 1000;
		
	Line_N[13].x1 = 1800;
	Line_N[13].y1 = 4200;
	Line_N[13].x2 = -1800;
	Line_N[13].y2 = 4200;
	Line_N[13].line_kp = 20;
	Line_N[13].line_A = 0;
	Line_N[13].line_B = 0;
	Line_N[13].line_C = 0;
	Line_N[13].line_Angle = 0;
	Line_N[13].line_Error = 0;
	Line_N[13].line_Priority = 1000;
	
	Line_N[14].x1 = -1800;
	Line_N[14].y1 = 4200;
	Line_N[14].x2 = -1800;
	Line_N[14].y2 = 600;
	Line_N[14].line_kp = 20;
	Line_N[14].line_A = 0;
	Line_N[14].line_B = 0;
	Line_N[14].line_C = 0;
	Line_N[14].line_Angle = 0;
	Line_N[14].line_Error = 0;
	Line_N[14].line_Priority = 3;
	
	Line_N[15].x1 = -1800;
	Line_N[15].y1 = 600;
	Line_N[15].x2 = 1800;
	Line_N[15].y2 = 600;
	Line_N[15].line_kp = 20;
	Line_N[15].line_A = 0;
	Line_N[15].line_B = 0;
	Line_N[15].line_C = 0;
	Line_N[15].line_Angle = 0;
	Line_N[15].line_Error = 0;
	Line_N[15].line_Priority = 3;
	
	Line_N[16].x1 = 1800;
	Line_N[16].y1 = 600;
	Line_N[16].x2 = 1800;
	Line_N[16].y2 = 4200;
	Line_N[16].line_kp = 20;
	Line_N[16].line_A = 0;
	Line_N[16].line_B = 0;
	Line_N[16].line_C = 0;
	Line_N[16].line_Angle = 0;
	Line_N[16].line_Error = 0;
	Line_N[16].line_Priority = 1000;
	
	
	
	
	
	Line_N[19].x1 = -600;
	Line_N[19].y1 = 1800;
	Line_N[19].x2 = -600;
	Line_N[19].y2 = 3000;
	Line_N[19].line_kp = 20;
	Line_N[19].line_A = 0;
	Line_N[19].line_B = 0;
	Line_N[19].line_C = 0;
	Line_N[19].line_Angle = 0;
	Line_N[19].line_Error = 0;
	Line_N[19].line_Priority = 0;
	
	Line_N[18].x1 = -600;
	Line_N[18].y1 = 3000;
	Line_N[18].x2 = 600;
	Line_N[18].y2 = 3000;
	Line_N[18].line_kp = 20;
	Line_N[18].line_A = 0;
	Line_N[18].line_B = 0;
	Line_N[18].line_C = 0;
	Line_N[18].line_Angle = 0;
	Line_N[18].line_Error = 0;
	Line_N[18].line_Priority = 0;
	
	Line_N[17].x1 = 600;
	Line_N[17].y1 = 3000;
	Line_N[17].x2 = 600;
	Line_N[17].y2 = 1800;
	Line_N[17].line_kp = 20;
	Line_N[17].line_A = 0;
	Line_N[17].line_B = 0;
	Line_N[17].line_C = 0;
	Line_N[17].line_Angle = 0;
	Line_N[17].line_Error = 0;
	Line_N[17].line_Priority = 0;
	
	Line_N[20].x1 = 600;
	Line_N[20].y1 = 1800;
	Line_N[20].x2 = -600;
	Line_N[20].y2 = 1800;
	Line_N[20].line_kp = 20;
	Line_N[20].line_A = 0;
	Line_N[20].line_B = 0;
	Line_N[20].line_C = 0;
	Line_N[20].line_Angle = 0;
	Line_N[20].line_Error = 0;
	Line_N[20].line_Priority = 0;
	
	Line_N[23].x1 = -1000;
	Line_N[23].y1 = 1400;
	Line_N[23].x2 = -1000;
	Line_N[23].y2 = 3400;
	Line_N[23].line_kp = 20;
	Line_N[23].line_A = 0;
	Line_N[23].line_B = 0;
	Line_N[23].line_C = 0;
	Line_N[23].line_Angle = 0;
	Line_N[23].line_Error = 0;
	Line_N[23].line_Priority = 1;
	
	Line_N[22].x1 = -1000;
	Line_N[22].y1 = 3400;
	Line_N[22].x2 = 1000;
	Line_N[22].y2 = 3400;
	Line_N[22].line_kp = 20;
	Line_N[22].line_A = 0;
	Line_N[22].line_B = 0;
	Line_N[22].line_C = 0;
	Line_N[22].line_Angle = 0;
	Line_N[22].line_Error = 0;
	Line_N[22].line_Priority = 1;
	
	Line_N[21].x1 = 1000;
	Line_N[21].y1 = 3400;
	Line_N[21].x2 = 1000;
	Line_N[21].y2 = 1400;
	Line_N[21].line_kp = 20;
	Line_N[21].line_A = 0;
	Line_N[21].line_B = 0;
	Line_N[21].line_C = 0;
	Line_N[21].line_Angle = 0;
	Line_N[21].line_Error = 0;
	Line_N[21].line_Priority = 1;
	
	Line_N[24].x1 = 1000;
	Line_N[24].y1 = 1400;
	Line_N[24].x2 = -1000;
	Line_N[24].y2 = 1400;
	Line_N[24].line_kp = 20;
	Line_N[24].line_A = 0;
	Line_N[24].line_B = 0;
	Line_N[24].line_C = 0;
	Line_N[24].line_Angle = 0;
	Line_N[24].line_Error = 0;
	Line_N[24].line_Priority = 1;
	
	Line_N[27].x1 = -1400;
	Line_N[27].y1 = 1000;
	Line_N[27].x2 = -1400;
	Line_N[27].y2 = 3800;
	Line_N[27].line_kp = 20;
	Line_N[27].line_A = 0;
	Line_N[27].line_B = 0;
	Line_N[27].line_C = 0;
	Line_N[27].line_Angle = 0;
	Line_N[27].line_Error = 0;
	Line_N[27].line_Priority = 1000;
	
	Line_N[26].x1 = -1400;
	Line_N[26].y1 = 3800;
	Line_N[26].x2 = 1400;
	Line_N[26].y2 = 3800;
	Line_N[26].line_kp = 20;
	Line_N[26].line_A = 0;
	Line_N[26].line_B = 0;
	Line_N[26].line_C = 0;
	Line_N[26].line_Angle = 0;
	Line_N[26].line_Error = 0;
	Line_N[26].line_Priority = 1000;
	
	Line_N[25].x1 = 1400;
	Line_N[25].y1 = 3800;
	Line_N[25].x2 = 1400;
	Line_N[25].y2 = 1000;
	Line_N[25].line_kp = 20;
	Line_N[25].line_A = 0;
	Line_N[25].line_B = 0;
	Line_N[25].line_C = 0;
	Line_N[25].line_Angle = 0;
	Line_N[25].line_Error = 0;
	Line_N[25].line_Priority = 1000;
	
	Line_N[28].x1 = 1400;
	Line_N[28].y1 = 1000;
	Line_N[28].x2 = -1400;
	Line_N[28].y2 = 1000;
	Line_N[28].line_kp = 20;
	Line_N[28].line_A = 0;
	Line_N[28].line_B = 0;
	Line_N[28].line_C = 0;
	Line_N[28].line_Angle = 0;
	Line_N[28].line_Error = 0;
	Line_N[28].line_Priority = 1000;
	
	Line_N[31].x1 = -1800;
	Line_N[31].y1 = 600;
	Line_N[31].x2 = -1800;
	Line_N[31].y2 = 4200;
	Line_N[31].line_kp = 20;
	Line_N[31].line_A = 0;
	Line_N[31].line_B = 0;
	Line_N[31].line_C = 0;
	Line_N[31].line_Angle = 0;
	Line_N[31].line_Error = 0;
	Line_N[31].line_Priority = 1000;
	
	Line_N[30].x1 = -1800;
	Line_N[30].y1 = 4200;
	Line_N[30].x2 = 1800;
	Line_N[30].y2 = 4200;
	Line_N[30].line_kp = 20;
	Line_N[30].line_A = 0;
	Line_N[30].line_B = 0;
	Line_N[30].line_C = 0;
	Line_N[30].line_Angle = 0;
	Line_N[30].line_Error = 0;
	Line_N[30].line_Priority = 1000;
	
	Line_N[29].x1 = 1800;
	Line_N[29].y1 = 4200;
	Line_N[29].x2 = 1800;
	Line_N[29].y2 = 600;
	Line_N[29].line_kp = 20;
	Line_N[29].line_A = 0;
	Line_N[29].line_B = 0;
	Line_N[29].line_C = 0;
	Line_N[29].line_Angle = 0;
	Line_N[29].line_Error = 0;
	Line_N[29].line_Priority = 1000;
	
	Line_N[32].x1 = 1800;
	Line_N[32].y1 = 600;
	Line_N[32].x2 = -1800;
	Line_N[32].y2 = 600;
	Line_N[32].line_kp = 20;
	Line_N[32].line_A = 0;
	Line_N[32].line_B = 0;
	Line_N[32].line_C = 0;
	Line_N[32].line_Angle = 0;
	Line_N[32].line_Error = 0;
	Line_N[32].line_Priority = 1000;
	
	Line_N[33].x1 = -1800;
	Line_N[33].y1 = 600;
	Line_N[33].x2 = -1800;
	Line_N[33].y2 = 4200;
	Line_N[33].line_kp = 20;
	Line_N[33].line_A = 0;
	Line_N[33].line_B = 0;
	Line_N[33].line_C = 0;
	Line_N[33].line_Angle = 0;
	Line_N[33].line_Error = 0;
	Line_N[33].line_Priority = 1000;
	
	
	
	
	
	Line_N[40].x1 = -1800;
	Line_N[40].y1 = 600;
	Line_N[40].x2 = -1800;
	Line_N[40].y2 = 4200;
	Line_N[40].line_kp = 20;
	Line_N[40].line_A = 0;
	Line_N[40].line_B = 0;
	Line_N[40].line_C = 0;
	Line_N[40].line_Angle = 0;
	Line_N[40].line_Error = 0;
	Line_N[40].line_Priority = 1000;
	
	Line_N[41].x1 = -1800;
	Line_N[41].y1 = 600;
	Line_N[41].x2 = -1800;
	Line_N[41].y2 = 4200;
	Line_N[41].line_kp = 20;
	Line_N[41].line_A = 0;
	Line_N[41].line_B = 0;
	Line_N[41].line_C = 0;
	Line_N[41].line_Angle = 0;
	Line_N[41].line_Error = 0;
	Line_N[41].line_Priority = 1000;
	
	Line_N[42].x1 = -1800;
	Line_N[42].y1 = 600;
	Line_N[42].x2 = -1800;
	Line_N[42].y2 = 4200;
	Line_N[42].line_kp = 20;
	Line_N[42].line_A = 0;
	Line_N[42].line_B = 0;
	Line_N[42].line_C = 0;
	Line_N[42].line_Angle = 0;
	Line_N[42].line_Error = 0;
	Line_N[42].line_Priority = 1000;
	
	Line_N[43].x1 = -1800;
	Line_N[43].y1 = 600;
	Line_N[43].x2 = -1800;
	Line_N[43].y2 = 4200;
	Line_N[43].line_kp = 20;
	Line_N[43].line_A = 0;
	Line_N[43].line_B = 0;
	Line_N[43].line_C = 0;
	Line_N[43].line_Angle = 0;
	Line_N[43].line_Error = 0;
	Line_N[43].line_Priority = 1000;
	
	Line_N[50].x1 = -1800;
	Line_N[50].y1 = 600;
	Line_N[50].x2 = -1800;
	Line_N[50].y2 = 4200;
	Line_N[50].line_kp = 20;
	Line_N[50].line_A = 0;
	Line_N[50].line_B = 0;
	Line_N[50].line_C = 0;
	Line_N[50].line_Angle = 0;
	Line_N[50].line_Error = 0;
	Line_N[50].line_Priority = 1000;
	
	Line_N[51].x1 = -1800;
	Line_N[51].y1 = 600;
	Line_N[51].x2 = -1800;
	Line_N[51].y2 = 4200;
	Line_N[51].line_kp = 20;
	Line_N[51].line_A = 0;
	Line_N[51].line_B = 0;
	Line_N[51].line_C = 0;
	Line_N[51].line_Angle = 0;
	Line_N[51].line_Error = 0;
	Line_N[51].line_Priority = 1000;
	
	Line_N[52].x1 = -1800;
	Line_N[52].y1 = 600;
	Line_N[52].x2 = -1800;
	Line_N[52].y2 = 4200;
	Line_N[52].line_kp = 20;
	Line_N[52].line_A = 0;
	Line_N[52].line_B = 0;
	Line_N[52].line_C = 0;
	Line_N[52].line_Angle = 0;
	Line_N[52].line_Error = 0;
	Line_N[52].line_Priority = 1000;
	
	Line_N[53].x1 = -1800;
	Line_N[53].y1 = 600;
	Line_N[53].x2 = -1800;
	Line_N[53].y2 = 4200;
	Line_N[53].line_kp = 20;
	Line_N[53].line_A = 0;
	Line_N[53].line_B = 0;
	Line_N[53].line_C = 0;
	Line_N[53].line_Angle = 0;
	Line_N[53].line_Error = 0;
	Line_N[53].line_Priority = 1000;
}

void PID_Arc_Init(void)                                                      //圆形参数初始化
{
	Arc_N[0].x0 = 0;
	Arc_N[0].y0 = 2200;
	Arc_N[0].r0 = 600;
	Arc_N[0].arc_Angle = 0;
	Arc_N[0].arc_Error = 0;
	Arc_N[0].arc_Direction = CW;
	Arc_N[0].arc_kp = 20;
	
	Arc_N[1].x0 = 0;
	Arc_N[1].y0 = 2200;
	Arc_N[1].r0 = 1100;
	Arc_N[1].arc_Angle = 0;
	Arc_N[1].arc_Error = 0;
	Arc_N[1].arc_Direction = CW;
	Arc_N[1].arc_kp = 20;
	
	Arc_N[2].x0 = 0;
	Arc_N[2].y0 = 2200;
	Arc_N[2].r0 = 1600;
	Arc_N[2].arc_Angle = 0;
	Arc_N[2].arc_Error = 0;
	Arc_N[2].arc_Direction = CW;
	Arc_N[2].arc_kp = 20;
}

void PID_Coordinate_Init(void)                                               //坐标参数初始化
{
	Coordinate_N[0].x3 = 2400;
	Coordinate_N[0].y3 = 2400;
	Coordinate_N[0].coordinate_Angle = 0;
}
void PID_Init(PID_Value *PID_a)                                              //PID总参数初始化
{
	PID_a->direction = 0;
	PID_a->Angle_Last = 0;
	PID_a->Angle_Set = 0;
	PID_a->DTerm = 0;
	PID_a->ITerm = 0;
	PID_a->Error = 0;
	PID_a->kp = 10;
	PID_a->ki = 0;
	PID_a->kd = 100;
	PID_a->Mode = Line;
	PID_a->Mode_Last = Line;
	PID_a->V = 1200;
	PID_a->Coordinate_Num = 0;
	PID_a->Line_Num = 0;
	PID_a->Line_Num_Next = 0;
	PID_a->Line_Num_Last = 0;
	PID_a->corner = 0;
	PID_a->Arc_Num = 0;
	PID_a->V_Set = 1200;
	PID_a->vel = 0;
	PID_a->err_line_num = 0;
	PID_a->push_pos_up = 16384;
	PID_a->push_pos_down = -16384;
	PID_a->fire_request = 0;
	PID_a->fire_command = 0;
	PID_a->fire_flag = 0;
	PID_a->food = 1500;
	PID_a->dogHungry = 0;

	PID_Line_Init();
	PID_a->l = &Line_N[PID_a->Line_Num];
	
	PID_Arc_Init();
	PID_a->r = &Arc_N[PID_a->Arc_Num];
	
	PID_Coordinate_Init();
	PID_a->c = &Coordinate_N[PID_a->Coordinate_Num];
}

void PID_Control(PID_Value *p)                                               //PID控制函数，输出电机转速
{
	if(p->Mode == manual)
	{
		p->Mode_Last = manual;
		return;
	}
	if(p->Mode_Last == manual) Init_PID(p);	
	p->l = &Line_N[p->Line_Num];
	p->r = &Arc_N[p->Arc_Num];
	p->c = &Coordinate_N[p->Coordinate_Num];
	switch(p->Mode)
	{
		case Arc:
			p->r->arc_Error = sqrt((((p->X)-(p->r->x0))*((p->X)-(p->r->x0))+((p->Y)-(p->r->y0))*((p->Y)-(p->r->y0)))) - (p->r->r0);
			p->r->arc_Error = constrain(p->r->arc_Error,1800,-1800);
			if((p->X) == (p->r->x0))  p->r->arc_Angle = Compare((p->Y),(p->r->y0))*90;
			else if((p->X)>(p->r->x0)) p->r->arc_Angle = ((float)atan((double)(((p->Y)-(p->r->y0))/((p->X)-(p->r->x0)))))*(180/Pi);
			else if((p->X)<(p->r->x0)) p->r->arc_Angle = 180+((float)atan((double)(((p->Y)-(p->r->y0))/((p->X)-(p->r->x0)))))*(180/Pi);
			if((p->r->arc_Angle)>180) (p->r->arc_Angle) -= 360;
			else if((p->r->arc_Angle)<-180) (p->r->arc_Angle) += 360;
			if(p->r->arc_Direction == CW) (p->r->arc_Angle) -= 180;
			if(p->r->arc_Direction == CW) (p->r->arc_Angle) = (p->r->arc_Angle)-(p->r->arc_Error)/(p->r->arc_kp);
			else if(p->r->arc_Direction == ACW) (p->r->arc_Angle) = (p->r->arc_Angle)+(p->r->arc_Error)/(p->r->arc_kp);
			if((p->r->arc_Angle)>180) (p->r->arc_Angle) -= 360;
			else if((p->r->arc_Angle)<-180) (p->r->arc_Angle) += 360;
			p->Angle_Set = p->r->arc_Angle;
			break;
		case Line:
			p->l->line_A = (p->l->y1)-(p->l->y2);
			p->l->line_B = (p->l->x2)-(p->l->x1);
			p->l->line_C = (p->l->x1)*(p->l->y2)-(p->l->x2)*(p->l->y1);
			p->l->line_Error = ((p->l->line_A)*(p->X)+(p->l->line_B)*(p->Y)+(p->l->line_C))/(sqrt((p->l->line_A)*(p->l->line_A)+(p->l->line_B)*(p->l->line_B)));
			if((p->l->line_B != 0) && (p->l->line_A != 0)) p->l->line_Angle = (((0-(p->l->line_B))/(ABS(p->l->line_B)))*90-(((p->l->line_A)*(p->l->line_B))/(ABS((p->l->line_A)*(p->l->line_B))))*(((float)(atan((double)(ABS((p->l->line_A)/(p->l->line_B))))))*(180/Pi)));
			else if((p->l->line_A == 0) && (p->l->line_B > 0)) p->l->line_Angle = -90;
			else if((p->l->line_A == 0) && (p->l->line_B < 0)) p->l->line_Angle = 90;
			else if((p->l->line_A < 0) && (p->l->line_B == 0)) p->l->line_Angle = 0;
			else if((p->l->line_A > 0) && (p->l->line_B == 0)) p->l->line_Angle = 180;
			p->l->line_Error = constrain(p->l->line_Error,1800,-1800);
			p->l->line_Angle -= ((p->l->line_Error)/(p->l->line_kp));
			if((p->l->line_Angle)>180) (p->l->line_Angle) -= 360;
			else if((p->l->line_Angle)<-180) (p->l->line_Angle) += 360;
			p->Angle_Set = p->l->line_Angle;
			break;
		case Coordinate:
			if((p->X) != (p->c->x3)) p->c->coordinate_Angle = (Compare((p->X),(p->c->x3)))*90+((float)atan((double)(((p->Y)-(p->c->y3))/((p->X)-(p->c->x3)))))*(180/Pi);
			else if((p->Y) > (p->c->y3)) p->c->coordinate_Angle = 180;
			else if((p->Y) < (p->c->y3)) p->c->coordinate_Angle = 0;
			p->Angle_Set = p->c->coordinate_Angle;
			break;
		default:
			break;
	}
	p->Error = p->Angle_Set - p->Angle;
	if((p->Error) > 180) p->Error -= 360;
	else if((p->Error) < -180) p->Error += 360;
//	if ((p->Error) > -10 && (p->Error) < 10)
//	{
//		p->kp = 20;
//		p->kd = 100;
//		p->ki = 0.05;
//	}
//	else
//	{
//		p->kp = 20;
//		p->kd = 50;
//		p->ki = 0;
//	}
//	if(p->V_Set < 100 && p->V_Set > -100) p->kp = 5;
	p->ITerm += p->ki * p->Error;
	p->ITerm = constrain(p->ITerm,2000.0f,-2000.0f);
	p->DTerm = p->Angle_Last - p->Angle;
	p->vel = p->kp * p->Error + p->ki * p->ITerm + p->kd * p->DTerm;
	p->vel = constrain(p->vel,2000.0f,-2000.0f);
	p->Angle_Last = p->Angle;
	p->Mode_Last = p->Mode;
}

void PID_Pre(PID_Value *p)                                                   //去掉PID部分的PID函数^_^
{
	p->l = &Line_N[p->Line_Num];
	p->r = &Arc_N[p->Arc_Num];
	p->c = &Coordinate_N[p->Coordinate_Num];
	switch(p->Mode)
	{
		case Arc:
			p->r->arc_Error = sqrt((((p->X)-(p->r->x0))*((p->X)-(p->r->x0))+((p->Y)-(p->r->y0))*((p->Y)-(p->r->y0)))) - (p->r->r0);
			p->r->arc_Error = constrain(p->r->arc_Error,1800,-1800);
			if((p->X) == (p->r->x0))  p->r->arc_Angle = Compare((p->Y),(p->r->y0))*90;
			else if((p->X)>(p->r->x0)) p->r->arc_Angle = ((float)atan((double)(((p->Y)-(p->r->y0))/((p->X)-(p->r->x0)))))*(180/Pi);
			else if((p->X)<(p->r->x0)) p->r->arc_Angle = 180+((float)atan((double)(((p->Y)-(p->r->y0))/((p->X)-(p->r->x0)))))*(180/Pi);
			if((p->r->arc_Angle)>180) (p->r->arc_Angle) -= 360;
			else if((p->r->arc_Angle)<-180) (p->r->arc_Angle) += 360;
			if(p->r->arc_Direction == CW) (p->r->arc_Angle) -= 180;
			if(p->r->arc_Direction == CW) (p->r->arc_Angle) = (p->r->arc_Angle)-(p->r->arc_Error)/(p->r->arc_kp);
			else if(p->r->arc_Direction == ACW) (p->r->arc_Angle) = (p->r->arc_Angle)+(p->r->arc_Error)/(p->r->arc_kp);
			if((p->r->arc_Angle)>180) (p->r->arc_Angle) -= 360;
			else if((p->r->arc_Angle)<-180) (p->r->arc_Angle) += 360;
			break;
		case Line:
			p->l->line_A = (p->l->y1)-(p->l->y2);
			p->l->line_B = (p->l->x2)-(p->l->x1);
			p->l->line_C = (p->l->x1)*(p->l->y2)-(p->l->x2)*(p->l->y1);
			p->l->line_Error = ((p->l->line_A)*(p->X)+(p->l->line_B)*(p->Y)+(p->l->line_C))/(sqrt((p->l->line_A)*(p->l->line_A)+(p->l->line_B)*(p->l->line_B)));
			if((p->l->line_B != 0) && (p->l->line_A != 0)) p->l->line_Angle = (((0-(p->l->line_B))/(ABS(p->l->line_B)))*90-(((p->l->line_A)*(p->l->line_B))/(ABS((p->l->line_A)*(p->l->line_B))))*(((float)(atan((double)(ABS((p->l->line_A)/(p->l->line_B))))))*(180/Pi)));
			else if((p->l->line_A == 0) && (p->l->line_B > 0)) p->l->line_Angle = -90;
			else if((p->l->line_A == 0) && (p->l->line_B < 0)) p->l->line_Angle = 90;
			else if((p->l->line_A < 0) && (p->l->line_B == 0)) p->l->line_Angle = 0;
			else if((p->l->line_A > 0) && (p->l->line_B == 0)) p->l->line_Angle = 180;
			p->l->line_Error = constrain(p->l->line_Error,1800,-1800);
			p->l->line_Angle -= ((p->l->line_Error)/(p->l->line_kp));
			if((p->l->line_Angle)>180) (p->l->line_Angle) -= 360;
			else if((p->l->line_Angle)<-180) (p->l->line_Angle) += 360;
			break;
		case Coordinate:
			if((p->X) != (p->c->x3)) p->c->coordinate_Angle = (Compare((p->X),(p->c->x3)))*90+((float)atan((double)(((p->Y)-(p->c->y3))/((p->X)-(p->c->x3)))))*(180/Pi);
			else if((p->Y) > (p->c->y3)) p->c->coordinate_Angle = 180;
			else if((p->Y) < (p->c->y3)) p->c->coordinate_Angle = 0;
			break;
		default:
			break;
	}
}

void GO(PID_Value *p_GO)                                                     //电机控制函数
{
	VelCrl(CAN1,2,(int)((0+(32768/(120*Pi))*(p_GO->vel))+(32768/(120*Pi))*(p_GO->V)));
	VelCrl(CAN1,1,(int)((0+(32768/(120*Pi))*(p_GO->vel))-(32768/(120*Pi))*(p_GO->V)));
}

extern int ballcommand;
void shoot(PID_Value *p_gun, int targets[], int Debug)                                                 //射球函数
{

	static int pos = 0, posLast = 0, posGap = 0, timeCnt = 0, timeDelay = 0, whiteCnt = 0, blackCnt = 0, noneCnt = 0, flag = 0;
	if (p_gun->fire_request)
	{
		p_gun->food = 1300;
		if(!p_gun->fire_command) return;
		posLast = pos;
		pos += p_gun->push_pos_up;
		PosCrl(CAN2,7,ABSOLUTE_MODE,pos);

		p_gun->fire_request = 0;
		p_gun->fire_command = 0;
		Scan.FirePermitFlag = 0;
		
		if(Scan.SetTimeFlag == 1)
		{
			Scan.SetTimeFlag = 0;
			Scan.CntDelayTime = 550;	
		}
		
		if(Gundata.MovingShootFlag && !Scan.ScanShootFlag)	{targets[p_gun->target_Num] += 1;}
		else	{targets[p_gun->target_Num] += 2;}
		
		Scan.CntFireTimes++;
		if(Scan.CntFireTimes > 3)
		{
			Scan.CntFireTimes = 0;
			Scan.Bubble_Mode = !Scan.Bubble_Mode;
		}
		if(Debug == 1) USART_OUT(UART4,(uint8_t*)"fire	%d	%d	%d	%d	\r\n", (int)targets[0],(int)targets[1],(int)targets[2],(int)targets[3]);
	}
	else
	{
		posGap = GetMotor7Pos() - pos;
		timeCnt++;
		if(posGap > (0.f-700.f) && posGap < 100)
		{
			if(flag == 1) return;
			timeDelay ++;
			if(timeDelay < 60) return;
			if(ballcolor == 0) noneCnt += 1;
			else if(ballcolor == 1) whiteCnt += 1;
			else if(ballcolor == 2) blackCnt += 1;
			if(ballcommand == WHITE_BALL)
			{
				if(noneCnt > 20)
				{
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", (int)pos);
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", (int)GetMotor7Pos());
					posLast = pos;
					pos += p_gun->push_pos_down;
					PosCrl(CAN2,7,ABSOLUTE_MODE,pos);
					noneCnt = 0, whiteCnt = 0, blackCnt = 0, timeCnt = 0, timeDelay = 0;
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", 0);
//					if(Debug == 1) USART_SendData(UART4,'\r');
//					if(Debug == 1) USART_SendData(UART4,'\n');
					flag = 1;
				}
				else if(whiteCnt > 20)
				{
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", (int)pos);
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", (int)GetMotor7Pos());
					p_gun->fire_request = 1;
					noneCnt = 0, whiteCnt = 0, blackCnt = 0, timeCnt = 0, timeDelay = 0;
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", 1);
//					if(Debug == 1) USART_SendData(UART4,'\r');
//					if(Debug == 1) USART_SendData(UART4,'\n');
					flag = 1;
				}
				else if(blackCnt > 20)
				{
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", (int)pos);
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", (int)GetMotor7Pos());
					posLast = pos;
					pos += p_gun->push_pos_down;
					PosCrl(CAN2,7,ABSOLUTE_MODE,pos);
					noneCnt = 0, whiteCnt = 0, blackCnt = 0, timeCnt = 0, timeDelay = 0;
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", 2);
//					if(Debug == 1) USART_SendData(UART4,'\r');
//					if(Debug == 1) USART_SendData(UART4,'\n');
					flag = 1;
				}
			}
			else
			{
				if(noneCnt > 20)
				{
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", (int)pos);
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", (int)GetMotor7Pos());
					posLast = pos;
					pos += p_gun->push_pos_down;
					PosCrl(CAN2,7,ABSOLUTE_MODE,pos);
					noneCnt = 0, whiteCnt = 0, blackCnt = 0, timeCnt = 0, timeDelay = 0;
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", 0);
//					if(Debug == 1) USART_SendData(UART4,'\r');
//					if(Debug == 1) USART_SendData(UART4,'\n');
					flag = 1;
				}
				else if(blackCnt> 20)
				{
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", (int)pos);
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", (int)GetMotor7Pos());
					p_gun->fire_request = 1;
					noneCnt = 0, whiteCnt = 0, blackCnt = 0, timeCnt = 0, timeDelay = 0;
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", 1);
//					if(Debug == 1) USART_SendData(UART4,'\r');
//					if(Debug == 1) USART_SendData(UART4,'\n');
					flag = 1;
				}
				else if(whiteCnt > 20)
				{
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", (int)pos);
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", (int)GetMotor7Pos());
					posLast = pos;
					pos += p_gun->push_pos_down;
					PosCrl(CAN2,7,ABSOLUTE_MODE,pos);
					noneCnt = 0, whiteCnt = 0, blackCnt = 0, timeCnt = 0, timeDelay = 0;
//					if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", 2);
//					if(Debug == 1) USART_SendData(UART4,'\r');
//					if(Debug == 1) USART_SendData(UART4,'\n');
					flag = 1;
				}
			}
		}
		if((0 < posGap && posGap < 8000) || (0 > posGap && posGap > -8000))
		{
			flag = 0;
		}
		if(timeCnt > 600)
		{
			if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", (int)pos);
			if(Debug == 1) USART_OUT(UART4,(uint8_t*)"%d	", (int)GetMotor7Pos());
			PosCrl(CAN2,7,ABSOLUTE_MODE,posLast);
			pos = posLast;
			noneCnt = 0, whiteCnt = 0, blackCnt = 0, timeCnt = 0, timeDelay = 0;
			if(Debug == 1) USART_OUT(UART4,(uint8_t*)"stuck");
			if(Debug == 1) USART_SendData(UART4,'\r');
			if(Debug == 1) USART_SendData(UART4,'\n');
			flag = 0;
		}
	}
}

void UART4_OUT(PID_Value *pid_out , Err*error1)                                           //串口输出函数
{
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->Angle);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->X);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->Y);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->X_Speed);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->Y_Speed);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->Angle_Set);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)GetWZ());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->corner);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->V);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->vel);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)error1->flag);
	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->food);
	USART_OUT(UART4,(uint8_t*)"%d	", (int)error1->errCnt);
	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->stop);
	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->timeCnt);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->fire_flag);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->fire_turn);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)pid_out->Line_Num);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)(sqrt(GetSpeedX()*GetSpeedX()+GetSpeedY()*GetSpeedY())));
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)fort.shooterVelReceive);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)GetShooterVelCommand());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)fort.yawPosReceive);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)GetYawPosCommand());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)ReadLaserAValue());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)ReadLaserBValue());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)Get_Adc_Average(15,10));
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)Get_Adc_Average(14,10));
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)GetShooterVelCommand());
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)gundata.YawPos);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)gundata.YawPosTarActAngle);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)Gundata.ShooterVelRec);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)Gundata.ShooterVelSet);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)gundata.CarVel_X);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)gundata.CarVel_Y);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)gundata.ShooterTime);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)gundata.ShooterVel);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)gundata.ShooterVelSet);		
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)gundata.Angle_Deviation);
//	USART_OUT(UART4,(uint8_t*)"%d	", (int)gundata.cntIteration);
//	USART_SendData(UART4,'\r');
//	USART_SendData(UART4,'\n');
}

void PID_Priority(PID_Value *pid, u8 dir, Err *error, int targetp[])                     //新版走线
{
	if(pid->stop == 1 && error->flag == 0)
	{
		if(pid->Y > 1400 && pid->Y < 3400 && pid->X > 0-1000 && pid->X < 1000)
		{
			pid->vel = 0;
			pid->V = 0;
			error->stop = 1;
			error->timeCnt = 0;
		}
		else
		{
			if(pid->Line_Num < 17 && pid->Line_Num > 3)
			{
				pid->Line_Num = pid->Line_Num % 4 + 1;
				if(pid->Line_Num == 4) pid->Line_Num = 0;
			}
			else if(pid->Line_Num > 20)
			{
				pid->Line_Num = pid->Line_Num % 4 - 1 + 17;
				if(pid->Line_Num == 16) pid->Line_Num = 20;
			}
			PID_Control(pid);
			pid->V = 1000;
			error->stop = 0;
		}
		return;
	}
	error->stop = 0;
//	else if(pid->stop == 2 && error->flag == 0)
//	{
//		pid->vel = 0;
//		pid->V = 0;
//		error->stop = 1;
//		error->timeCnt = 0;
//		return;
//	}
	static u8 flag = 0;
	int i = 0;
	if(flag == 0 && dir == Right)
	{
		pid->Line_Num = 0;
		flag = 1;
		for(i=0;i<4;i++)
		{
			Line_N[4*i].line_Priority--;
			if(Line_N[4*i].line_Priority == -1) Line_N[4*i].line_Priority = 3;
		}
	}
	if(flag == 0 && dir == Left)
	{
		pid->Line_Num = 19;
		flag = 1;
		for(i=4;i<8;i++)
		{
			Line_N[4*i+3].line_Priority--;
			if(Line_N[4*i+3].line_Priority == -1) Line_N[4*i+3].line_Priority = 3;
		}
	}
	if(pid->Line_Num < 17)
	{
		if(pid->corner == 0)
		{
			if(error->flag == 0)
			{
				pid->kp = 15;
				pid->kd = 100;
				pid->direction = ACW;
				pid->Mode = Line;
				PID_Control(pid);
				pid->target_Num = pid->Line_Num % 4 + 1;
				if(pid->target_Num == 4) pid->target_Num = 0;
				pid->Line_Num_Last = pid->Line_Num;
				priority priority1 = {0};
				for(i=0;i<4;i++)
				{
					pid->Line_Num_Next = pid->Line_Num = 4*i+pid->target_Num;
					pid->l = &Line_N[pid->Line_Num];
					if(i == 0)
					{
						priority1.priority = pid->l->line_Priority;
						priority1.line_num = pid->Line_Num;
					}
					if(priority1.priority > pid->l->line_Priority)
					{
						priority1.priority = pid->l->line_Priority;
						priority1.line_num = pid->Line_Num;
					}
				}
				pid->Line_Num_Next = pid->Line_Num = priority1.line_num;
				pid->target_Next = pid->Line_Num_Next % 4 + 1;
				if(pid->target_Next == 4) pid->target_Next = 0;
				PID_Pre(pid);
				if(pid->l->line_Error>1300)
				{
					pid->fire_turn = 0;
					pid->Line_Num = pid->Line_Num_Last;
					if(pid->Error < 3) pid->V += 15;
					pid->V = constrain(pid->V,1800,1200);
				}
				else if(pid->l->line_Error<=1300)
				{
					pid->corner = 1;
					pid->Line_Num_Next = pid->Line_Num;
					for(i=0;i<4;i++)
					{
						Line_N[4*i+pid->target_Num].line_Priority--;
						if(Line_N[4*i+pid->target_Num].line_Priority == -1) Line_N[4*i+pid->target_Num].line_Priority = 3;
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(4*i+pid->target_Num));
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(Line_N[4*i+pid->target_Num].line_Priority));
//						USART_SendData(UART4,'\r');
//						USART_SendData(UART4,'\n');
					}
//					USART_OUT(UART4,(uint8_t*)"%d	", (int)(pid->Line_Num));
//					USART_SendData(UART4,'\r');
//					USART_SendData(UART4,'\n');
//					USART_SendData(UART4,'\r');
//					USART_SendData(UART4,'\n');
				}
			}
			else
			{
				static int timeCnt1 = 0;
				timeCnt1++;
				if(timeCnt1 < 150)
				{
					pid->Angle += 180;
					pid->kp = 2.5;
					PID_Control(pid);
					pid->V = -800;
				}
				else
				{
					timeCnt1 = 0;
					error->flag = 0;
					pid->V = 1200;
					pid->kp = 15;
					pid->Line_Num += 17;
					pid->l = &Line_N[pid->Line_Num];
					for(i=0;i<17;i++)
					{
						Line_N[i+17].line_Priority = Line_N[i].line_Priority;
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(i));
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(Line_N[i].line_Priority));
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(i+17));
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(Line_N[i+17].line_Priority));
//						USART_SendData(UART4,'\r');
//						USART_SendData(UART4,'\n');
//						USART_SendData(UART4,'\r');
//						USART_SendData(UART4,'\n');
					}
				}
			}
		}
		else
		{
			if(error->flag == 0)
			{
				pid->kp = 15;
				pid->Line_Num = pid->Line_Num_Next;
				pid->target_Num = pid->Line_Num % 4 + 1;
				if(pid->target_Num == 4) pid->target_Num = 0;
				PID_Pre(pid);
				if(pid->l->line_Error > 800)
				{
					pid->V = 1200;
					pid->Line_Num = pid->Line_Num_Last;
					PID_Control(pid);
					pid->fire_turn = 0;
				}
				else
				{
					pid->Line_Num = pid->Line_Num_Next;
					PID_Control(pid);
					pid->fire_turn = 1;
					if(ABS(pid->Error) < 20)
					{
						pid->corner = 0;
						float angle = pid->Angle + 1.f;
						CorrectAngle(angle);
					}
					return;
				}
				pid->Line_Num = pid->Line_Num_Last;
			}
			else
			{
				static int timeCnt2 = 0;
				timeCnt2++;
				if(timeCnt2 <150)
				{
					pid->Angle += 180;
					pid->kp = 2.5;
					PID_Control(pid);
					pid->V = -800;
				}
				else
				{
					pid->corner = 0;
					timeCnt2 = 0;
					error->flag = 0;
					pid->V = 1200;
					pid->kp = 20;
					pid->Line_Num += 17;
					pid->l = &Line_N[pid->Line_Num];
					for(i=0;i<17;i++)
					{
						Line_N[i+17].line_Priority = Line_N[i].line_Priority;
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(i));
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(Line_N[i].line_Priority));
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(i+17));
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(Line_N[i+17].line_Priority));
//						USART_SendData(UART4,'\r');
//						USART_SendData(UART4,'\n');
//						USART_SendData(UART4,'\r');
//						USART_SendData(UART4,'\n');
					}
				}
			}
		}
		for(i=0;i<17;i++)
		{
			Line_N[i+17].line_Priority = Line_N[i].line_Priority;
		}
	}
	else if(pid->Line_Num > 16)
	{
		if(pid->corner == 0)
		{
			if(error->flag == 0)
			{
				pid->kp = 15;
				pid->kd = 100;
				pid->direction = CW;
				pid->Mode = Line;
				PID_Control(pid);
				pid->target_Num = pid->Line_Num % 4 - 1;
				if(pid->target_Num == -1) pid->target_Num = 3;
				pid->Line_Num_Last = pid->Line_Num;
				priority priority2 = {0};
				if(pid->target_Num != 0)
				{
					for(i=4;i<8;i++)
					{
						pid->Line_Num_Next = pid->Line_Num = 4*i+pid->target_Num;
						pid->l = &Line_N[pid->Line_Num];
						if(i == 4)
						{
							priority2.priority = pid->l->line_Priority;
							priority2.line_num = pid->Line_Num;
						}
						if(priority2.priority > pid->l->line_Priority)
						{
							priority2.priority = pid->l->line_Priority;
							priority2.line_num = pid->Line_Num;
						}
					}
				}
				else if(pid->target_Num == 0)
				{
					for(i=4;i<8;i++)
					{
						pid->Line_Num_Next = pid->Line_Num = 4*i+4;
						pid->l = &Line_N[pid->Line_Num];
						if(i == 4)
						{
							priority2.priority = pid->l->line_Priority;
							priority2.line_num = pid->Line_Num;
						}
						if(priority2.priority > pid->l->line_Priority)
						{
							priority2.priority = pid->l->line_Priority;
							priority2.line_num = pid->Line_Num;
						}
					}
				}
				pid->Line_Num_Next = pid->Line_Num = priority2.line_num;
				pid->target_Next = pid->Line_Num_Next % 4 - 1;
				if(pid->target_Next == -1) pid->target_Next = 3;
				PID_Pre(pid);
				if(pid->l->line_Error <-1300)
				{
					pid->fire_turn = 0;
					pid->Line_Num = pid->Line_Num_Last;
					if(pid->Error < 3) pid->V += 15;
					pid->V = constrain(pid->V,1800,1200);
				}
				else if(pid->l->line_Error>=-1300)
				{
					pid->corner = 1;
					pid->Line_Num_Next = pid->Line_Num;
					if(pid->target_Num != 0)
					{
						for(i=4;i<8;i++)
						{
							Line_N[4*i+pid->target_Num].line_Priority--;
							if(Line_N[4*i+pid->target_Num].line_Priority == -1) Line_N[4*i+pid->target_Num].line_Priority = 3;
//							USART_OUT(UART4,(uint8_t*)"%d	", (int)(4*i+pid->target_Num));
//							USART_OUT(UART4,(uint8_t*)"%d	", (int)(Line_N[4*i+pid->target_Num].line_Priority));
//							USART_SendData(UART4,'\r');
//							USART_SendData(UART4,'\n');
						}
					}
					else if(pid->target_Num == 0)
					{
						for(i=4;i<8;i++)
						{
							Line_N[4*i+4].line_Priority--;
							if(Line_N[4*i+4].line_Priority == -1) Line_N[4*i+4].line_Priority = 3;
//							USART_OUT(UART4,(uint8_t*)"%d	", (int)(4*i+4));
//							USART_OUT(UART4,(uint8_t*)"%d	", (int)(Line_N[4*i+4].line_Priority));
//							USART_SendData(UART4,'\r');
//							USART_SendData(UART4,'\n');
						}
					}
//					USART_OUT(UART4,(uint8_t*)"%d	", (int)(pid->Line_Num));
//					USART_SendData(UART4,'\r');
//					USART_SendData(UART4,'\n');
//					USART_SendData(UART4,'\r');
//					USART_SendData(UART4,'\n');
				}
			}
			else
			{
				static int timeCnt3 = 0;
				timeCnt3++;
				if(timeCnt3 < 150)
				{
					pid->Angle += 180;
					pid->kp = 2.5;
					PID_Control(pid);
					pid->V = -800;
				}
				else
				{
					timeCnt3 = 0;
					error->flag = 0;
					pid->V = 1200;
					pid->kp = 15;
					pid->Line_Num -= 17;
					pid->l = &Line_N[pid->Line_Num];
					for(i=0;i<17;i++)
					{
						Line_N[i].line_Priority = Line_N[i+17].line_Priority;
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(i+17));
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(Line_N[i+17].line_Priority));
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(i));
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(Line_N[i].line_Priority));
//						USART_SendData(UART4,'\r');
//						USART_SendData(UART4,'\n');
//						USART_SendData(UART4,'\r');
//						USART_SendData(UART4,'\n');
					}
				}
			}
		}
		else
		{
			if(error->flag == 0)
			{
				pid->Line_Num = pid->Line_Num_Next;
				pid->target_Num = pid->Line_Num % 4 - 1;
				if(pid->target_Num == -1) pid->target_Num = 3;
				PID_Pre(pid);
				pid->kp = 15;
				if(pid->l->line_Error < -800)
				{
					pid->fire_turn = 0;
					pid->V = 1200;
					pid->Line_Num = pid->Line_Num_Last;
					PID_Control(pid);
				}
				else
				{
					pid->Line_Num = pid->Line_Num_Next;
					PID_Control(pid);
					pid->fire_turn = 1;
					if(ABS(pid->Error) < 20)
					{
						pid->corner = 0;
						float angle = pid->Angle - 1.f;
						CorrectAngle(angle);
					}
					return;
				}
				pid->Line_Num = pid->Line_Num_Last;
			}
			else
			{
				static int timeCnt4 = 0;
				timeCnt4++;
				if(timeCnt4 < 150)
				{
					pid->Angle += 180;
					pid->kp = 2.5;
					PID_Control(pid);
					pid->V = -1200;
				}
				else
				{
					pid->corner = 0;
					timeCnt4 = 0;
					error->flag = 0;
					pid->V = 1200;
					pid->kp = 15;
					pid->Line_Num -= 17;
					pid->l = &Line_N[pid->Line_Num];
					for(i=0;i<17;i++)
					{
						Line_N[i].line_Priority = Line_N[i+17].line_Priority;
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(i));
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(Line_N[i].line_Priority));
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(i+17));
//						USART_OUT(UART4,(uint8_t*)"%d	", (int)(Line_N[i+17].line_Priority));
//						USART_SendData(UART4,'\r');
//						USART_SendData(UART4,'\n');
//						USART_SendData(UART4,'\r');
//						USART_SendData(UART4,'\n');
					}
				}
			}
		}
		for(i=0;i<17;i++)
		{
			Line_N[i].line_Priority = Line_N[i+17].line_Priority;
		}
	}
}

void GetData(PID_Value *p)                                                   //读数
{
	if(GetAngle() > 180 || GetAngle() < -180) return;
	else if(GetX() > 4000 || GetX() < -4000) return;
	else if(GetY() > 4000 || GetY() < -4000) return;
	else if(GetSpeedY() > 3000 || GetSpeedY() < -3000) return;
	else if(GetSpeedX() > 3000 || GetSpeedX() < -3000) return;
	p->Angle = GetAngle();
	p->X = GetX() + 170.08f * sin((p->Angle)*(Pi/180));
	p->Y = GetY() + 75.03f + 170.68f - 170.68f * cos((p->Angle)*(Pi/180));
	p->X_Speed = GetSpeedX();
	p->Y_Speed = GetSpeedY();
}

void ErrorDisposal(PID_Value *pid,Err *error)                                //错误检测
{
	if(error->flag == 1 || error->stop == 1) return;
	if(error->timeCnt == 0)
	{
		error->Err_X = pid->X;
		error->Err_Y = pid->Y;
	}
	error->timeCnt++;
	if(error->timeCnt > 200)
	{
		error->timeCnt = 0;
		error->distance = sqrt((error->Err_X - pid->X)*(error->Err_X - pid->X)+(error->Err_Y - pid->Y)*(error->Err_Y - pid->Y));
		if(error->distance < error->err_distance)
		{
			error->flag = 1;
			error->errCnt += 1;
			pid->err_line_num = pid->Line_Num;
			if(pid->Line_Num < 8) pid->Line_Num += 8;
			else if(pid->Line_Num > 7 && pid->Line_Num < 17) pid->Line_Num -= 8;
			else if(pid->Line_Num > 16 && pid->Line_Num < 25) pid->Line_Num += 8;
			else if(pid->Line_Num > 24 && pid->Line_Num < 34) pid->Line_Num -= 8;
		}
//		USART_OUT(UART4,(uint8_t*)"%d	", (int)pid->Line_Num);
//		USART_OUT(UART4,(uint8_t*)"%d	", (int)error->flag);
//		USART_SendData(UART4,'\r');
//		USART_SendData(UART4,'\n');
	}
}

void WatchDog(PID_Value *Dog)
{
	Dog->food -- ;
	if(Dog->food < 0) Dog->dogHungry = 1;
	else Dog->dogHungry = 0;
}

void PriorityControl(PID_Value *PID,Err *err,int targetn[])
{
	int i = 0;
	int prioritySum = 0;
	PID->timeCnt ++;
	if(err->flag == 1) return;
	for( i = 0 ; i < 29 ; i ++ )/*锁住内三圈优先级*/
	{
		if(i < 12 || (i > 16 && i <29))
		{
			if(Line_N[i].line_Priority >= 3) Line_N[i].line_Priority = 1000;
			prioritySum += Line_N[i].line_Priority;/*内三圈优先级求和*/
		}
	}
	if(targetn[0] + targetn[1] + targetn[2] + targetn[3] > 2)
	{
		err->errCnt += 1;
		if(PID->fire_request == 1)
		{
			PID->stop = 1;
			for( i = 0 ; i < 29 ; i ++ )/*锁住内三圈优先级*/
			{
				if(i < 12 || (i > 16 && i <29))
				{
					Line_N[i].line_Priority = 1000;
					prioritySum += Line_N[i].line_Priority;/*内三圈优先级求和*/
				}
			}
		}
	}
	if(PID->Line_Num < 12 || (PID->Line_Num > 16 && PID->Line_Num <29)) Line_N[PID->Line_Num].line_Priority = 1000;
	if((prioritySum >= 23500 && ((PID->Line_Num > 11 && PID->Line_Num < 17) || PID->Line_Num > 28)) || PID->stop == 1)/*最外圈走形状态*/
	{
		if(err->errCnt == 0 && PID->timeCnt < 3000)/*未避障状态*/
		{
			if(PID->dogHungry == 0)/*有球*/
			{
				if(targetn[0] + targetn[1] + targetn[2] + targetn[3] >= 2) err->errCnt += 1;
				return;
			}
			else/*无球*/
			{
				if(PID->timeCnt > 2400) err->errCnt += 1;
				return;
			}
		}
		else/*扫描走形*/
		{
			if(PID->dogHungry == 0)/*有球*/
			{
				PID->stop = 1;/*去场中央投球*/
			}
			else/*无球*/
			{
				PID->stop = 0;
				if(PID->Line_Num < 17)
				{
					for(i = 0 ; i < 4 ; i ++ )
					{
						Line_N[i].line_Priority = 0;
						Line_N[i + 4].line_Priority = 1;
						Line_N[i + 8].line_Priority = 2;
						Line_N[i + 12].line_Priority = 3;
					}
				}
				else
				{
					for(i = 0 ; i < 4 ; i ++ )  
					{
						Line_N[i + 17].line_Priority = 0;
						Line_N[i + 21].line_Priority = 1;
						Line_N[i + 25].line_Priority = 2;
						Line_N[i + 29].line_Priority = 3;
					}
				}
			}
		}
	}
}
