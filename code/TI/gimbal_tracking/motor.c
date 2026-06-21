#include "motor.h"
float Position_KP=6.0,Position_KD=3; //相关速度PID参数
/*************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM增量
**************************************************************************/
float Position_PID_1(float Position,float Target)
{ 	                                          //增量输出
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Position;                                  //计算偏差	                                
	 Pwm=Position_KP*Bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;  
}
/*************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM增量
**************************************************************************/
float Position_PID_2(float Position,float Target)
{ 	                                         //增量输出
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Position;                                  //计算偏差                                
	 Pwm=Position_KP*Bias/100+Position_KD*(Bias-Last_Bias)/100;       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 return Pwm;  
}
void Set_PWM(int pwma,int pwmb)
{
    Position1 += pwma;
	Position2 += pwmb;
	Position1 = PWM_limit(Position1,2250,750);
	Position2 = PWM_limit(Position2,2250,750);
	DL_Timer_setCaptureCompareValue(PWM_0_INST,Position1,GPIO_PWM_0_C0_IDX);
	DL_Timer_setCaptureCompareValue(PWM_1_INST,Position2,GPIO_PWM_1_C1_IDX);
	
}

uint16_t PWM_limit(uint16_t in,uint16_t max,uint16_t min)
{
	if(in>max) in=max;
	if(in<min) in=min;
	return in;
}

