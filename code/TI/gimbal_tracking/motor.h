#ifndef _MOTOR_H
#define _MOTOR_H
#include "ti_msp_dl_config.h"
#include "board.h"

float Position_PID_1(float Position,float Target);
float Position_PID_2(float Position,float Target);
void Set_PWM(int pwma,int pwmb);
uint16_t PWM_limit(uint16_t in,uint16_t max,uint16_t min);
#endif