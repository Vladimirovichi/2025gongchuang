#ifndef __CONTROL_H__
#define __CONTROL_H__
 
#include "main.h"
#include "usart.h"
#include "sys.h"

//主动轮轮距，单位：m
extern float Wheel_spacing; 

//小车前后轴的轴距，单位：m
extern float Axle_spacing; 



//小车三轴目标运动速度，单位：m/s
extern float Move_X, Move_Y, Move_Z; 

//遥控小车的默认速度，单位：mm/s
extern float RC_Velocity; 

//编码器精度
extern float Encoder_precision; 

//轮子周长，单位：m
extern float Wheel_perimeter; 

//编码器数据读取频率
#define   CONTROL_FREQUENCY 100

//Motor speed control related parameters of the structure
//电机速度控制相关参数结构体
typedef struct  
{
	float Encoder;     //编码器数值，读取电机实时速度
	float Motor_Pwm;   //电机PWM数值，控制电机实时速度
	float Target;      //电机目标速度值，控制电机目标速度
	float Velocity_KP; //速度控制PID参数
	float	Velocity_KI; //速度控制PID参数
}Motor_parameter;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void Servo(uint8_t Servodata);
void AutoRunning(void);
void Empty(u8 mode);
void GetBall(void);
void PutBall(void);
void Get_Motor_Speed(void);
void Drive_Motor(void);
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d);
int Incremental_PI_A (float Encoder,float Target);
int Incremental_PI_B (float Encoder,float Target);
int Incremental_PI_C (float Encoder,float Target);
int Incremental_PI_D (float Encoder,float Target);
void Limit_Pwm(int amplitude);

#endif
