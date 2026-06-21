
#include "board.h"
#include "stdio.h"
#include "motor.h"
#include <math.h>
#include <stdlib.h>

// 全局变量定义
int32_t PWMA, PWMB;
float Position1 = 600, Position2 = 500;  // 当前位置（PWM值）
float Target1 = 600, Target2 = 500;     // 目标位置（PWM值）
uint16_t current_cx = 160, current_cy = 160, target_cx = 160, target_cy = 160;  // 坐标参数
uint8_t RxState = 0;        // 接收状态 (0:等待'#'，1:接收数字)
uint8_t coordType = 0;      // 坐标类型 (0:未定义, 1:矩形x, 2:矩形y, 3:红点x, 4:红点y)
uint8_t coordPhase = 0;     // 坐标阶段 (0:等待矩形, 1:等待红点)
uint16_t tempValue = 0;     // 临时存储数值
static uint16_t task = 0,angle_task = 0,stop_task = 0,turn_task = 0 , motor_task = 0,target_task = 0;
int cnt = 0;
// 用户自定义变量
uint8_t move_mode = 0;  // 移动模式：0=初始化 1=第三题 2=正向遍历 3=反向遍历
uint8_t move_sign = 0;  // 移动标志：0=单步移动, 1=连续移动
volatile uint8_t gData = 0;
float step_size = 2.0f;  // PWM步长大小
// const float diagonal_ratio = 1.0f; // 斜向移动比例

// 函数声明
void move_step(uint8_t mode);
void update_motor_positions(void);

// 按键2检测函数（请根据实际硬件修改引脚）
uint8_t key2_pressed(void)
{
    static uint8_t last_state = 0;
    uint8_t current_state = DL_GPIO_readPins(GPIO_LED_PORT, GPIO_LED_BUTTON_PIN) > 0;
    uint8_t pressed = 0;
    
    // 检测下降沿（按键按下）
    if (last_state && !current_state) {
        delay_ms(10); // 消抖
        if (DL_GPIO_readPins(GPIO_LED_PORT, GPIO_LED_BUTTON_PIN) == 0) {
            pressed = 1;
        }
    }
    
    last_state = current_state;
    return pressed;
}

void UART_K230_INST_IRQHandler(void)
{
    
    uint8_t gData = DL_UART_Main_receiveData(UART_K230_INST);
    
            if(gData == '#') {
                tempValue = 0;    // 重置临时变量
            }
            if(gData == '$') {
                // 矩形坐标阶段
                if(coordType == 0) {
                    // 第一个值：矩形x
                    current_cx = tempValue;
                    target_task = 1;
                    // coordType = 1;
                } else if(coordType == 1) {
                    // 第二个值：矩形y
                    current_cy = tempValue;
                    coordType = 0;       // 重置类型
                }
                tempValue = 0; // 重置临时值
            } 
            // 处理数字字符
            else if(gData >= '0' && gData <= '9') {
                tempValue = tempValue * 10 + (gData - '0');
            }
            // 遇到非法字符重置状态
            else {
                // coordType = 0;
                tempValue = 0;
            }
}

int main(void)
{
    SYSCFG_DL_init();

    if(!DL_GPIO_readPins(GPIO_CIRCLE_PORT,GPIO_CIRCLE_NUM_0_PIN))
    {
        move_mode+=1;
    }
    if(!DL_GPIO_readPins(GPIO_CIRCLE_PORT,GPIO_CIRCLE_NUM_1_PIN))
    {
        move_mode+=1;
    }
    NVIC_ClearPendingIRQ(UART_K230_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_K230_INST_INT_IRQN);

    while(!key2_pressed());
    if(target_task == 0)
    {
        if(move_mode == 0)
        {
            Target1 = 1500, Target2 = 500;
            Position1 = 1500,Position2 = 500;
            PWMA = Position_PID_1(1500, 1500);
            PWMB = Position_PID_2(500, 500);
            Set_PWM(PWMA, PWMB);
        }
        else if(move_mode == 1)
        {
            Target1 = 800, Target2 = 1500;
            Position1 = 800,Position2 = 1500;
            PWMA = Position_PID_1(800, 800);
            PWMB = Position_PID_2(500, 500);
            Set_PWM(PWMA, PWMB);
        }
        else if(move_mode == 2)
        {
            Target1 = 2200, Target2 = 1500;
            Position1 = 2200,Position2 = 1500;
            PWMA = Position_PID_1(2200, 2200);
            PWMB = Position_PID_2(500, 500);
            Set_PWM(PWMA, PWMB);
        }
    }

    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_0_INST);

    while (1) 
    {
        if(target_task == 1)
        {
            if(abs(current_cx - target_cx) >= 100)
            {
                step_size = 8.0;
                move_sign = 1;
            }
            else if(abs(current_cx - target_cx) >= 50)
            {
                step_size = 5.0;
                move_sign = 1;
            }
            else if(abs(current_cx - target_cx) >= 10)
            {
                step_size = 1.0;
                move_sign = 1;
            }
            else if(current_cx != target_cx){
                cnt++;
                if(cnt > 2)
                {
                    DL_GPIO_clearPins(GPIO_LED_PORT,GPIO_LED_LED_PIN);
                    DL_TimerA_stopCounter(TIMER_0_INST);
                    DL_TimerA_startCounter(TIMER_1_INST);
                    NVIC_DisableIRQ(UART_K230_INST_INT_IRQN);
                    // delay_cycles(64000000);
                    // DL_GPIO_setPins(GPIO_LED_PORT,GPIO_LED_LED_PIN);
                }
            }
            if(move_sign == 1) {
                move_step(move_mode);
            }
        }
        else if(target_task == 0)
        {
            if(move_mode == 1)
            {
                Target1+=5;
            }
            else if(move_mode == 2)
            {
                Target1-=5;
            }
            step_size = 5.0;
            // Target2+=10;
        }
            // Target1+=1;
            // // Target2+=1;
            // PWMA = Position_PID_1(Position1,Target1);
            // PWMB = Position_PID_2(Position2,Target2);
            delay_cycles(640000);
    }
}

// 执行移动步骤（支持6种模式）
void move_step(uint8_t mode)
{
    if(abs(current_cx - target_cx) >= 10)
    {
        if(current_cx < target_cx)//逆时针
            Target1 = PWM_limit(Target1 + step_size, 2400, 600);
        else if(current_cx > target_cx)//顺时针
            Target1 = PWM_limit(Target1 - step_size, 2400, 600);
    }
    else {
        move_sign = 0;
    }
}

// 定时器中断处理 - 更新舵机位置
void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        if (DL_TIMER_IIDX_ZERO)
        {
            if(key2_pressed())
            {
                DL_Timer_startCounter(PWM_0_INST);
                DL_Timer_startCounter(PWM_1_INST);
            }
            update_motor_positions();
        }
    }
}

void TIMER_1_INST_IRQHandler(void)
{
    DL_GPIO_setPins(GPIO_LED_PORT,GPIO_LED_LED_PIN);
}

// 更新舵机位置
void update_motor_positions(void)
{
    // 更新PID控制
    PWMA = Position_PID_1(Position1, Target1);
    PWMB = Position_PID_2(Position2, Target2);
    Set_PWM(PWMA, PWMB);
}