#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <math.h>
#include <ti/devices/msp/msp.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "main.h"

#define ENCODER_PPR      13    
#define WHEEL_DIAMETER   4.8    
#define TARGET_DISTANCE_CM   80.0
#define TARGET_PULSES 1500
#define BACK_PULSES 7
#define LOST_THRESHOLD 200 // 连续50ms丢线才确认

#define L2	!DL_GPIO_readPins(GPIO_GRP_0_L2_PORT, GPIO_GRP_0_L2_PIN)
#define L1	!DL_GPIO_readPins(GPIO_GRP_0_L1_PORT, GPIO_GRP_0_L1_PIN)
#define R1	!DL_GPIO_readPins(GPIO_GRP_0_R1_PORT, GPIO_GRP_0_R1_PIN)
#define R2	!DL_GPIO_readPins(GPIO_GRP_0_R2_PORT, GPIO_GRP_0_R2_PIN)

void Motor_Control(int16_t left_pwm, int16_t right_pwm);
void PWM_Init(void);
void track_control(void);
void motorleft(int16_t angle);
void motorright(uint16_t distance);
void motorforward(uint16_t angle);
void motorback(uint16_t angle);   
void motorsWrite(int speedL,int speedR);
void stopmotor(void);
void startmotor(void);
void rotateToAngle(float target_angle);
void KeepAngle(float target_angle);
void Readcircle(void);

uint8_t oled_buffer[32];

static uint16_t lost_counter = 0;
static uint16_t pulse_cnt = 0,cnt = 0;       
static uint16_t overflow = 0;     
float Kp = 600.0f, Ki=0.1f, Kd =5.0f;
float P = 0, I = 0, D = 0, PID_value = 0;
float new_error = 0, previous_error = 0;
float angle_new_error = 0, angle_previous_error = 0;
static int initial_motor_speed = 2000;
static uint16_t task = 0,angle_task = 0,stop_task = 0,turn_task = 0 ,circle = 0;
static uint16_t num = 0;
int16_t rxbuf = 0,cx1 = 160,cx2 = 160,cx = 160;
float nowangle = 0;

// ????
void PWM_Init(void);

// void GROUP1_IRQHandler(void)
// {
//     switch( DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1) )
//     {
// 			case ENCODE_INT_IIDX:
// 				if(task == 1)
// 				{
// 					pulse_cnt = pulse_cnt + 1;
// 				}
// 				if(pulse_cnt > TARGET_PULSES)
// 				{
// 					pulse_cnt = 0;
// 					stopmotor();
// 				}
// 			break;
//     }
// }

void TIMER_0_INST_IRQHandler(void)
{
	return;
}

void TIMER_1_INST_IRQHandler(void)//差量定时器
{
	DL_TimerA_stopCounter(TIMA1);
	if(task == 1)
		task = 2;
	else if(task == 2)
		task = 1;
}

void PID_INST_IRQHandler(void)
{
	if( (!L2 && !R1) || (!L1 && !R2))//1 1 1 1
	{
		stopmotor();
		return;
	}
	if(task == 1)
	{
		track_control();
		if(!L2 && !L1 && R1 && R2)//1 1 0 0
		{
			turn_task = 1;
		}
		if( ((L2 && R1) || (L1 && R2)) && (turn_task != 0))//0 0 0 0
		{
			cnt += 1;
			if(cnt >= 10)//0 0 0 0
			DL_TimerA_startCounter(TIMA1);//前进差量后转向
		}
		else {
			cnt = 0;
		}

	}

	//角度判断
	if(angle_task == 0 && task == 2)
	{
		if(turn_task == 1)
		{
			DL_TimerA_stopCounter(PID_INST);
			nowangle += 90;
			rotateToAngle(nowangle);
		}
		else if(turn_task == 2)
		{
			nowangle -= 90;
			rotateToAngle(nowangle);
		}
	}
}

void UART_K230_INST_IRQHandler(void)
{
	uint8_t gData;
	switch(DL_UART_Main_getPendingInterrupt(UART_K230_INST))
	{
		case DL_UART_MAIN_IIDX_RX:
			gData = DL_UART_Main_receiveData(UART_K230_INST);
			// if(gData == 'S')
			// {
			// 	stop_task = 1;
			// 	stopmotor();
			// }
			// else if(gData == 'L')
			// 	turn_task = 1;
			// else if(gData == 'R')
			// 	turn_task = 2;
			// else if(gData == 'T' && (task == 0))
			// {
			//  	task = 1;
			// }
		default:
			break;
	}
}

int main(void) {
	delay_cycles(320000000);
	SYSCFG_DL_init();
	while(DL_GPIO_readPins(GPIO_OLED_PORT,GPIO_OLED_BUTTON_PIN));
	// DL_UART_Main_transmitData(UART_K230_INST,'S');
	NVIC_EnableIRQ(UART_K230_INST_INT_IRQN);
	NVIC_ClearPendingIRQ(UART_K230_INST_INT_IRQN); 
	NVIC_EnableIRQ(PID_INST_INT_IRQN);
	SysTick_Init();
	PWM_Init();
    WIT_Init();
	// SYSCFG_DL_TIMER_0_init();
	SYSCFG_DL_TIMER_1_init();
	// NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_1_INST_INT_IRQN);
	SYSCFG_DL_PID_init();

	Readcircle();
	task = 1;
	stopmotor();
	while (1) 
	{
		
	}
}


void PWM_Init(void) {
    SYSCFG_DL_PWM_AB_init();
    DL_TimerG_startCounter(TIMG6);
}

void  motorsWrite(int speedL,int speedR)
{
	if(speedL > 0) 
	{
		//????
		DL_GPIO_setPins(GPIO_GRP_0_MOTOR_BIN1_PORT,GPIO_GRP_0_MOTOR_BIN1_PIN);
		DL_GPIO_clearPins(GPIO_GRP_0_MOTOR_BIN2_PORT,GPIO_GRP_0_MOTOR_BIN2_PIN);
		DL_TimerG_setCaptureCompareValue(PWM_AB_INST,abs(speedL),GPIO_PWM_AB_C0_IDX);
	}
	else	
	{
		//????
		DL_GPIO_setPins(GPIO_GRP_0_MOTOR_BIN2_PORT,GPIO_GRP_0_MOTOR_BIN2_PIN);
		DL_GPIO_clearPins(GPIO_GRP_0_MOTOR_BIN1_PORT,GPIO_GRP_0_MOTOR_BIN1_PIN);
		DL_TimerG_setCaptureCompareValue(PWM_AB_INST,abs(speedL),GPIO_PWM_AB_C0_IDX);
	}
	
	if(speedR > 0)
	{
		//????
		DL_GPIO_setPins(GPIO_GRP_0_MOTOR_AIN1_PORT,GPIO_GRP_0_MOTOR_AIN1_PIN);
		DL_GPIO_clearPins(GPIO_GRP_0_MOTOR_AIN2_PORT,GPIO_GRP_0_MOTOR_AIN2_PIN);
		DL_TimerG_setCaptureCompareValue(PWM_AB_INST,abs(speedR),GPIO_PWM_AB_C1_IDX);
	}
	else
	{
		//????
		DL_GPIO_setPins(GPIO_GRP_0_MOTOR_AIN2_PORT,GPIO_GRP_0_MOTOR_AIN2_PIN);
		DL_GPIO_clearPins(GPIO_GRP_0_MOTOR_AIN1_PORT,GPIO_GRP_0_MOTOR_AIN1_PIN);
		DL_TimerG_setCaptureCompareValue(PWM_AB_INST,abs(speedR),GPIO_PWM_AB_C1_IDX);
	}
}

void track_Set_PWM(int pwm_value)
{
    const float leftAdjust = 1;   
    const float rightAdjust = 1.07;  
    
    int left_motor_speed = initial_motor_speed + pwm_value;
    int right_motor_speed = initial_motor_speed - pwm_value;
    
    left_motor_speed = (int)(left_motor_speed * leftAdjust);
    right_motor_speed = (int)(right_motor_speed * rightAdjust);
    
    if(left_motor_speed > 9999) left_motor_speed = 9999;
    else if(left_motor_speed < -4999) left_motor_speed = -4999;
    
    if(right_motor_speed > 9999) right_motor_speed = 9999;
    else if(right_motor_speed < -4999) right_motor_speed = -4999;
    
    motorsWrite(left_motor_speed, right_motor_speed);
}

int track_scan(void){
	if( L2 && !L1 && !R1 && !R2 )//0 1 1 1
	{
		new_error = -3;
	}
	if( L2 && L1 && !R1 && !R2 )//0 0 1 1
	{
		new_error = -2;
	}
	if( !L2 && L1 && !R1 && !R2 )//1 0 1 1
	{
		new_error = -1;
	}
	if( !L2 && L1 && R1 && !R2 )//1 0 0 1
	{                                                                                        
		new_error = 0;
	}
	if( !L2 && !L1 && R1 && !R2 )//1 1 0 1
	{
		new_error = 1;
	}
	if( !L2 && !L1 && R1 && R2 )//1 1 0 0
	{
		new_error = 2;
	}
	if( !L2 && !L1 && !R1 && R2 )//1 1 1 0
	{
		new_error = 3;
	}
	if( !L2 && !L1 && !R1 && !R2 )//1 1 1 1
	{
		new_error = 0;
	}
	return 0;
}

//速度环PID
int track_pid(void)
{  
	P=new_error;	 
    if (fabs(new_error) < 2.0f) {
        I=I+new_error;
    } else {
        I = 0;
    }		        
	D=new_error-previous_error;	
	
	PID_value=(Kp*P)+(Ki*I)+(Kd*D);
	
	previous_error=new_error;
	
	return PID_value; 
}

// 角度环PID控制器
float anglePID(float target_angle, float current_angle, float dt) {
  // 计算归一化角度误差（考虑最短路径）
  float error = target_angle - current_angle;
  	if(error > 180)
		error = error - 360;
	if(error < -180)
		error = error + 360;
	float angle_Kp = 200.0f, angle_Ki=0.1f, angle_Kd =50.0f;

	//P项
  	P=error;

  	// 积分项（带抗饱和）
	if (fabs(error) < 2.0f) {
		I=I+error;
	} else {
		I = 0;
	}	
  
	// 微分项
	float D = (error - angle_previous_error) / dt;
	angle_previous_error = error;
  
	// PID输出
	float output = angle_Kp * P + angle_Ki * I + angle_Kd * D;
    	if(output > 2000)
		output = 2000;
	if(output < -2000)
		output = -2000;
  return output;
}

// 旋转到目标角度
void rotateToAngle(float target_angle) {
	// stopmotor();
  	const float tolerance = 5.0;  // 目标角度容差
      
      // 获取当前角度
	float current_angle = wit_data.yaw;
	float angle_diff = target_angle - current_angle;
  	if(angle_diff > 180)
		angle_diff = angle_diff - 360;
	if(angle_diff < -180)
		angle_diff = angle_diff + 360;
      
      // 计算PID输出
      float pid_output = anglePID(target_angle, current_angle, 1);
      
      // 应用PID输出到电机（差速转向）
      motorsWrite(-pid_output, pid_output);  // 左轮负，右轮正 = 左转
	  startmotor();
      
	// 检查是否达到目标角度
	if (fabs(angle_diff) <= tolerance) {
		lost_counter = lost_counter + 1;
		if(lost_counter > LOST_THRESHOLD) {
			// stopmotor();
			angle_task = 1;
			turn_task = 0;
			task = 1;//直线
			DL_TimerA_startCounter(PID_INST);
		}
	}
	else {
		lost_counter = 0;
	}
    // }
//   }
}

//保持角度，现在用循迹代替
// void KeepAngle(float target_angle) {
//   	const float tolerance = 1.0;  // 目标角度容差
      
//       // 获取当前角度
// 	float current_angle = wit_data.yaw;
// 	float angle_diff = target_angle - current_angle;
//   	if(angle_diff > 180)
// 		angle_diff = angle_diff - 360;
// 	if(angle_diff < -180)
// 		angle_diff = angle_diff + 360;
      
//       // 计算PID输出
//       float pid_output = anglePID(target_angle, current_angle, 1);
      
//       // 应用PID输出到电机（差速转向）
// 	  track_Set_PWM(-pid_output);
// 	  startmotor();
      
// 	// 检查是否达到目标角度
// 	if (fabs(angle_diff) <= tolerance) {
// 		lost_counter = lost_counter + 1;
// 		if(lost_counter > LOST_THRESHOLD) {
// 			// angle_task = 1;
// 			// stopmotor();
// 			// task = 3;
// 			// break;
// 			// task = 1;
// 			motorforward(nowangle);
// 		}
// 	}
// 	else {
// 		lost_counter = 0;
// 	}
//     // }
// //   }
// }

void track_control(void)
{
	track_scan();
	track_Set_PWM(track_pid());
}

void stopmotor(void)
{
	new_error = 0;
	previous_error = 0;
	DL_GPIO_clearPins(GPIO_GRP_0_MOTOR_STBY_PORT, GPIO_GRP_0_MOTOR_STBY_PIN); 
	DL_GPIO_setPins(GPIO_OLED_PORT, GPIO_OLED_LEDR_PIN); 
	stop_task = 0;
}

void startmotor(void)
{
	DL_GPIO_setPins(GPIO_GRP_0_MOTOR_STBY_PORT, GPIO_GRP_0_MOTOR_STBY_PIN); 
	DL_GPIO_clearPins(GPIO_OLED_PORT, GPIO_OLED_LEDR_PIN); 
}

void Readcircle(void)
{
	if(DL_GPIO_readPins(GPIO_CIRCLE_NUM_1_PORT,GPIO_CIRCLE_NUM_1_PIN))
	{
		circle += 1;
	}
	if(DL_GPIO_readPins(GPIO_CIRCLE_NUM_2_PORT,GPIO_CIRCLE_NUM_2_PIN))
	{
		circle += 2;
	}
	if(DL_GPIO_readPins(GPIO_CIRCLE_NUM_4_PORT,GPIO_CIRCLE_NUM_4_PIN))
	{
		circle += 4;
	}
}
