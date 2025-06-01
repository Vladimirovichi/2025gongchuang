#ifndef __CONTROL_H__
#define __CONTROL_H__
 
#include "main.h"
#include "usart.h"
#include "sys.h"

//�������־࣬��λ��m
extern float Wheel_spacing; 

//С��ǰ�������࣬��λ��m
extern float Axle_spacing; 



//С������Ŀ���˶��ٶȣ���λ��m/s
extern float Move_X, Move_Y, Move_Z; 

//ң��С����Ĭ���ٶȣ���λ��mm/s
extern float RC_Velocity; 

//����������
extern float Encoder_precision; 

//�����ܳ�����λ��m
extern float Wheel_perimeter; 

//���������ݶ�ȡƵ��
#define   CONTROL_FREQUENCY 100

//Motor speed control related parameters of the structure
//����ٶȿ�����ز����ṹ��
typedef struct  
{
	float Encoder;     //��������ֵ����ȡ���ʵʱ�ٶ�
	float Motor_Pwm;   //���PWM��ֵ�����Ƶ��ʵʱ�ٶ�
	float Target;      //���Ŀ���ٶ�ֵ�����Ƶ��Ŀ���ٶ�
	float Velocity_KP; //�ٶȿ���PID����
	float	Velocity_KI; //�ٶȿ���PID����
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
