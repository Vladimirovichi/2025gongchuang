#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

//�����ܳ�����λ��m
float Wheel_perimeter = 0.065f*PI;

//�������־࣬��λ��m
float Wheel_spacing = 0.187f;

//ң��С����Ĭ���ٶȣ���λ��mm/s
float RC_Velocity=50;

//С������Ŀ���˶��ٶȣ���λ��m/s
float Move_X, Move_Y, Move_Z; 

//����������
float Encoder_precision = 1560; 

//С��ǰ�������࣬��λ��m
float Axle_spacing = 0.173f; 

//PS2�ֱ�������ر���
extern float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 
extern u8 PS2_ON_Flag;
Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
//�ٶȿ���PID����
float Velocity_KP=0.8,Velocity_KI=0.2;

uint16_t count = 0;

// ״̬��״̬
typedef enum {
    WAITING_FOR_HEADER,  // �ȴ�֡ͷ
    RECEIVING_DATA,      // ��������
    WAITING_FOR_TAIL     // �ȴ�֡β
} UART_ReceiveState;

UART_ReceiveState uart_state = WAITING_FOR_HEADER;  // ��ʼ״̬
uint8_t rx_buffer;          // ���ֽڻ�����
uint8_t rx_data[4];         // ���ݻ����������ֽں͵��ֽڷֱ�洢��
uint8_t data_index = 0;     // ����
int16_t location_data[2];

/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)

�������ܣ�����ʽPI������
��ڲ���������������ֵ(ʵ���ٶ�)��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>500)Pwm=500;
	 if(Pwm<-500)Pwm=-500;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>500)Pwm=500;
	 if(Pwm<-500)Pwm=-500;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>500)Pwm=500;
	 if(Pwm<-500)Pwm=-500;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>500)Pwm=500;
	 if(Pwm<-500)Pwm=-500;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm; 
}

/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
�������ܣ��޷�����
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}


void Get_Motor_Speed(void)
{
		int countdata[4];
		countdata[0] = Get_Encoder_Data(2);
		countdata[1] = Get_Encoder_Data(3);
		countdata[2] = Get_Encoder_Data(4);
		countdata[3] = Get_Encoder_Data(5);		
		
		//������ԭʼ����ת��Ϊ�����ٶȣ���λm/s
		MOTOR_A.Encoder= countdata[0] *CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= countdata[1] *CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= countdata[2] *CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= countdata[3] *CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
		
		if(MOTOR_A.Encoder != 0)
		{
				if(MOTOR_A.Encoder <0)
				{
						MOTOR_A.Encoder = 5128*MOTOR_A.Encoder-30;
				}
				else
				{
						MOTOR_A.Encoder = 5128*MOTOR_A.Encoder+30;
				}
		}
		
		if(MOTOR_B.Encoder != 0)
		{
				if(MOTOR_B.Encoder <0)
				{
						MOTOR_B.Encoder = 5128*MOTOR_B.Encoder-30;
				}
				else
				{
						MOTOR_B.Encoder = 5128*MOTOR_B.Encoder+30;
				}
		}
		
		if(MOTOR_C.Encoder != 0)
		{
				if(MOTOR_C.Encoder <0)
				{
						MOTOR_C.Encoder = 5128*MOTOR_C.Encoder-30;
				}
				else
				{
						MOTOR_C.Encoder = 5128*MOTOR_C.Encoder+30;
				}
		}
		
		if(MOTOR_D.Encoder != 0)
		{
				if(MOTOR_D.Encoder <0)
				{
						MOTOR_D.Encoder = 5128*MOTOR_D.Encoder-30;
				}
				else
				{
						MOTOR_D.Encoder = 5128*MOTOR_D.Encoder+30;
				}
		}
		
		//printf("%f %f %f %f\r\n",MOTOR_A.Encoder,MOTOR_B.Encoder,MOTOR_C.Encoder,MOTOR_D.Encoder);
}

//void Drive_Motor(void)
//{
//		float amplitude=300; //Wheel target speed limit //����Ŀ���ٶ�PWM�޷�

//		if(location_data[0] == 0)
//		{
//				MOTOR_A.Target  = location_data[1]; //��������ֵ�Ŀ���ٶ�
//				MOTOR_B.Target  = location_data[1]; //��������ֵ�Ŀ���ٶ�
//				MOTOR_C.Target  = location_data[1]; //��������ֵ�Ŀ���ٶ�
//				MOTOR_D.Target  = location_data[1]; //��������ֵ�Ŀ���ٶ�
//		}
//		if(location_data[0] > 0)//��ת
//		{
//				if(location_data[1] < 50)
//				{
//						MOTOR_A.Target  = location_data[1] * 2 - location_data[0] / 2; //��������ֵ�Ŀ���ٶ�
//						MOTOR_B.Target  = location_data[1] * 2 - location_data[0] / 2; //��������ֵ�Ŀ���ٶ�
//						MOTOR_C.Target  = location_data[1] * 2 + location_data[0] / 2; //��������ֵ�Ŀ���ٶ�
//						MOTOR_D.Target  = location_data[1] * 2 + location_data[0] / 2; //��������ֵ�Ŀ���ٶ�
//				}
//				else
//				{
//						MOTOR_A.Target  = location_data[1] * 2 - location_data[0]; //��������ֵ�Ŀ���ٶ�
//						MOTOR_B.Target  = location_data[1] * 2 - location_data[0]; //��������ֵ�Ŀ���ٶ�
//						MOTOR_C.Target  = location_data[1] * 2 + location_data[0]; //��������ֵ�Ŀ���ٶ�
//						MOTOR_D.Target  = location_data[1] * 2 + location_data[0]; //��������ֵ�Ŀ���ٶ�
//				}
//				
//		}
//		if(location_data[0] < 0)//��ת
//		{
//				if(location_data[1] < 50)
//				{
//						MOTOR_A.Target  = location_data[1] * 2 - location_data[0] / 2; //��������ֵ�Ŀ���ٶ�
//						MOTOR_B.Target  = location_data[1] * 2 - location_data[0] / 2; //��������ֵ�Ŀ���ٶ�
//						MOTOR_C.Target  = location_data[1] * 2 + location_data[0] / 2; //��������ֵ�Ŀ���ٶ�
//						MOTOR_D.Target  = location_data[1] * 2 + location_data[0] / 2; //��������ֵ�Ŀ���ٶ�
//				}
//				else
//				{
//						MOTOR_A.Target  = location_data[1] * 2 - location_data[0]; //��������ֵ�Ŀ���ٶ�
//						MOTOR_B.Target  = location_data[1] * 2 - location_data[0]; //��������ֵ�Ŀ���ٶ�
//						MOTOR_C.Target  = location_data[1] * 2 + location_data[0]; //��������ֵ�Ŀ���ٶ�
//						MOTOR_D.Target  = location_data[1] * 2 + location_data[0]; //��������ֵ�Ŀ���ٶ�
//				}
//				
//		}
//		
//		//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
//		MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
//		MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
//		MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); 
//		MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); 	
//}

void Drive_Motor(void)
{
		float amplitude=300; //Wheel target speed limit //����Ŀ���ٶ�PWM�޷�

		if(location_data[0] == 0)
		{
				MOTOR_A.Target  = location_data[1]; //��������ֵ�Ŀ���ٶ�
				MOTOR_B.Target  = location_data[1]; //��������ֵ�Ŀ���ٶ�
				MOTOR_C.Target  = location_data[1]; //��������ֵ�Ŀ���ٶ�
				MOTOR_D.Target  = location_data[1]; //��������ֵ�Ŀ���ٶ�
		}
		if(location_data[0] > 0)//��ת
		{
				MOTOR_A.Target  = location_data[1] * 2 - location_data[0]; //��������ֵ�Ŀ���ٶ�
				MOTOR_B.Target  = location_data[1] * 2 - location_data[0]; //��������ֵ�Ŀ���ٶ�
				MOTOR_C.Target  = location_data[1] * 2 + location_data[0]; //��������ֵ�Ŀ���ٶ�
				MOTOR_D.Target  = location_data[1] * 2 + location_data[0]; //��������ֵ�Ŀ���ٶ�
		}
		if(location_data[0] < 0)//��ת
		{
				MOTOR_A.Target  = location_data[1] * 2 - location_data[0]; //��������ֵ�Ŀ���ٶ�
				MOTOR_B.Target  = location_data[1] * 2 - location_data[0]; //��������ֵ�Ŀ���ٶ�
				MOTOR_C.Target  = location_data[1] * 2 + location_data[0]; //��������ֵ�Ŀ���ٶ�
				MOTOR_D.Target  = location_data[1] * 2 + location_data[0]; //��������ֵ�Ŀ���ٶ�
		}
		
		//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
		MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
		MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); 
		MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); 	
}




void PS2_control(void)
{
//   	int LX,LY,RY;
//		int Threshold=20; //Threshold to ignore small movements of the joystick //��ֵ������ҡ��С���ȶ���
//			
//	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
//	  //128Ϊ��ֵ��PS2����ϵ��ROS����ϵ��X��Y�Ķ��岻һ��
//		LY=-(PS2_LX-128); 
//		LX=-(PS2_LY-128); 
//		RY=-(PS2_RX-128);
//	
//	  //Ignore small movements of the joystick //����ҡ��С���ȶ���
//		if(LX>-Threshold&&LX<Threshold)LX=0; 
//		if(LY>-Threshold&&LY<Threshold)LY=0; 
//		if(RY>-Threshold&&RY<Threshold)RY=0; 
//	
//	  if (PS2_KEY==11)		RC_Velocity+=5;  //�ĳɼ���
//	
//	  //Handle PS2 controller control commands
//	  //��PS2�ֱ�����������д���
//		Move_X=LX*RC_Velocity/128; 
//		Move_Y=LY*RC_Velocity/128; 
//	  Move_Z=RY*(PI/2)/128;      
//	
//	  //Z-axis data conversion //Z������ת��

//		if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
//		Move_Z=Move_Z*RC_Velocity/500;

//	  //Unit conversion, mm/s -> m/s
//    //��λת����mm/s -> m/s	
//		Move_X=Move_X/1000;        
//		Move_Y=Move_Y/1000;    
//		Move_Z=Move_Z;
//		
//		//Control target value is obtained and kinematics analysis is performed
//	  //�õ�����Ŀ��ֵ�������˶�ѧ����
//		Drive_Motor(Move_X,Move_Y,Move_Z);		 			
} 

void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d)
{
	//Forward and reverse control of motor


	//�������ת����
	if(motor_a<0)			
	{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,-motor_a);
	}
	else 	            
	{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,motor_a);
	}
	if(motor_b<0)			
	{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);     //
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,-motor_b);
	}
	else 	            
	{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);          //
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,motor_b);
	}
	if(motor_c<0)			
	{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);           //
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,-motor_c);
	}
	else 	            
	{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);                 //
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,motor_c);
	}
	if(motor_d<0)			
	{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,-motor_d);
	}
	else 	            
	{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,motor_d);
	}
	
}

void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	

void Servo(uint8_t Servodata)
{
		if(Servodata==1)
		{
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 196);//��ʼ
		}		
		else if(Servodata==0)
		{
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 245);//̧��
		}
		else if(Servodata==2)
		{
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 235);//����
		}
		else if(Servodata==3)
		{
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 160);//����
		}
		else if(Servodata==4)
		{
				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 202);//����׶
		}
}

void GetBall(void)
{
//			Set_Pwm(0, 0, 0, 0);
//			Servo(0);
//			Set_Pwm(0, 0, 0, 0);
//			delay_ms(500);
//			while(location_data[0] != 800)
//			{
//					if(location_data[0] == 700)
//					{
//							Set_Pwm(-200, -200, -200, -200 );
//							delay_ms(500);
//							break;
//					}
//					Set_Pwm(125, 125, 125, 125 );
//			}
//			Set_Pwm(0, 0, 0, 0);
//			Servo(1);
//			delay_ms(500);
//			
//			Set_Pwm(-200,-200, -200, -200);
//			delay_ms(300);
//			Set_Pwm(200,200, 200, 200);
//			delay_ms(300);
//			
//			Set_Pwm(0, 0, 0, 0);
//			delay_ms(500); 
}

void PutBall(void)
{
//			Set_Pwm(0, 0, 0, 0);
//			delay_ms(100);
//			Set_Pwm(-100,-100, -100, -100);
//			delay_ms(600);
//			Set_Pwm(0, 0, 0, 0);
//			delay_ms(500);
//			Set_Pwm(350,350, 350, 350);
//			delay_ms(400);
//			Set_Pwm(0, 0, 0, 0);
//			delay_ms(100);
//			Servo(2);
//			Set_Pwm(0, 0, 0, 0);
//			delay_ms(500);
//			Set_Pwm(-300, -300, -300, -300);
//			delay_ms(400);
//			Servo(1);
//			Set_Pwm(400, 400, -400, -400);  
//			delay_ms(1000);
//			Set_Pwm(0, 0, 0, 0);
//		
//			uint8_t data[] = {0xAA ,0xCC ,0x01 ,0xBB};
//			HAL_UART_Transmit(&huart4, data, sizeof(data), HAL_MAX_DELAY);
//			delay_ms(500);
//			Set_Pwm(0, 0, 0, 0);
}

void Empty(u8 mode)
{
		if(mode == 0)
		{
				count++;
				if(count == 3000)
				{
								
						Set_Pwm(200, 200, 200, 200);
						delay_ms(500);
				}
					if(count == 5000)
				{
												count = 0;
						Set_Pwm(-200, -200, -200,- 200);
						delay_ms(500);
				}			

				MOTOR_A.Target = 200;
				MOTOR_B.Target = 200;
				MOTOR_C.Target = -200;
				MOTOR_D.Target = -200;
				
				MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
				MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
				MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
				MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
							 
				Limit_Pwm(350); 
				
				Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm); 
		}
		else if(mode == 1)
		{
				MOTOR_A.Target = 200;
				MOTOR_B.Target = 200;
				MOTOR_C.Target = -200;
				MOTOR_D.Target = -200;
				
				MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
				MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
				MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
				MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
							 
				Limit_Pwm(350); 
				
				Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm); 
		} 
}		

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if (huart->Instance == UART4)
		{
				switch (uart_state)
				{
				case WAITING_FOR_HEADER:  // �ȴ�֡ͷ
						if (rx_buffer == 0xAA)  // ֡ͷƥ��
						{
								uart_state = RECEIVING_DATA;  // �л�����������״̬
								data_index = 0;
						}
						break;

				case RECEIVING_DATA:  // ��������
						rx_data[data_index++] = rx_buffer;

						if (data_index == 4)  // �յ� 4 �ֽ�����
						{
								uart_state = WAITING_FOR_TAIL;
						}
						break;

				case WAITING_FOR_TAIL:  // �ȴ�֡β
						if (rx_buffer == 0xBB)  // ֡βƥ��
						{
								// ���ݽ�����ɣ���������
								location_data[0] = (rx_data[0] << 8) | rx_data[1];  // ��ԭˮƽ����
								location_data[1] = (rx_data[2] << 8) | rx_data[3];  // ��ԭ��ֱ����

								printf("%d   %d\r\n", location_data[0],location_data[1]);
						}
						else
						{
								printf("Frame Error: Missing Tail\r\n");
						}

						// ����״̬
						uart_state = WAITING_FOR_HEADER;
						break;

				default:
						uart_state = WAITING_FOR_HEADER;
						break;
				}

				// �ٴν������ģʽ
				HAL_UART_Receive_IT(&huart4, &rx_buffer, 1);
		}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
		if (huart->Instance == UART4)
		{
				
		}
}

//void AutoRunning(void)
//{

//			Get_Motor_Speed();
//			
//			if(location_data[1] == 600)
//			{
//					GetBall();
//					location_data[0] = location_data[1] = 0;
//			}
//			
//			if(location_data[1] == 0)
//			{
//					Set_Pwm(0, 0,  0, 0);
//			}
//			
//			else
//			{
//					Drive_Motor();
//				
////					MOTOR_A.Target = 200;
////					MOTOR_B.Target = 200;
////					MOTOR_C.Target = 150;
////					MOTOR_D.Target = 150;
//					//�ٶȱջ����Ƽ�������PWMֵ��PWM������ʵ��ת�ٶ�ӦPWMֵ
//					MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
//					MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
//					MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
//					MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
//								 
//					Limit_Pwm(300); 
//					
//					Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm); 
//					//Set_Pwm(100, 100,  100, 100); 
//			}
//			
//}
