/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 
extern u8 PS2_ON_Flag;
extern int16_t location_data[2];
extern Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
uint8_t data[] = {0xAA ,0xCC ,0x03 ,0xBB};     // 0x00		走完一轮，进入新一轮找球
																							 // 0x01		确认抓到球且符合条件，开始找框
																							 // 0x02		
uint8_t first_flag = 1;
uint8_t state_flag = 0;
uint16_t cnt = 0;
uint16_t cat = 0;
// 状态机状态
typedef enum {
    FIND_TARGET,
    FIND_BALL
} Running_State;

Running_State run_state = FIND_BALL;  // 初始状态

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

u8 mode = 0;    //mode  1    Find_Ball_Red
								//mode  2		 Find_Ball_Blue
								//mode  3    Catch_Gall
								//mode  4    Find_Target_Location
								//mode  5    Push_Ball
								
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Mode */
osThreadId_t ModeHandle;
const osThreadAttr_t Mode_attributes = {
  .name = "Mode",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PS2_Task */
osThreadId_t PS2_TaskHandle;
const osThreadAttr_t PS2_Task_attributes = {
  .name = "PS2_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Find_Ball */
osThreadId_t Find_BallHandle;
const osThreadAttr_t Find_Ball_attributes = {
  .name = "Find_Ball",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Push_ball */
osThreadId_t Push_ballHandle;
const osThreadAttr_t Push_ball_attributes = {
  .name = "Push_ball",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Catch_ball */
osThreadId_t Catch_ballHandle;
const osThreadAttr_t Catch_ball_attributes = {
  .name = "Catch_ball",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Find_Target_Loc */
osThreadId_t Find_Target_LocHandle;
const osThreadAttr_t Find_Target_Loc_attributes = {
  .name = "Find_Target_Loc",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for test */
osThreadId_t testHandle;
const osThreadAttr_t test_attributes = {
  .name = "test",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for get_ball */
osSemaphoreId_t get_ballHandle;
const osSemaphoreAttr_t get_ball_attributes = {
  .name = "get_ball"
};
/* Definitions for put_ball */
osSemaphoreId_t put_ballHandle;
const osSemaphoreAttr_t put_ball_attributes = {
  .name = "put_ball"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_Mode(void *argument);
void StartPS2_Task(void *argument);
void Start_Find_Ball(void *argument);
void StartPush_ball(void *argument);
void StartCatch_ball(void *argument);
void Start_Find_Target_Location(void *argument);
void Start_test(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of get_ball */
  get_ballHandle = osSemaphoreNew(1, 0, &get_ball_attributes);

  /* creation of put_ball */
  put_ballHandle = osSemaphoreNew(1, 0, &put_ball_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Mode */
  ModeHandle = osThreadNew(Start_Mode, NULL, &Mode_attributes);

  /* creation of PS2_Task */
  PS2_TaskHandle = osThreadNew(StartPS2_Task, NULL, &PS2_Task_attributes);

  /* creation of Find_Ball */
  Find_BallHandle = osThreadNew(Start_Find_Ball, NULL, &Find_Ball_attributes);

  /* creation of Push_ball */
  Push_ballHandle = osThreadNew(StartPush_ball, NULL, &Push_ball_attributes);

  /* creation of Catch_ball */
  Catch_ballHandle = osThreadNew(StartCatch_ball, NULL, &Catch_ball_attributes);

  /* creation of Find_Target_Loc */
  Find_Target_LocHandle = osThreadNew(Start_Find_Target_Location, NULL, &Find_Target_Loc_attributes);

  /* creation of test */
  testHandle = osThreadNew(Start_test, NULL, &test_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_Mode */
/**
  * @brief  Function implementing the Mode thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Mode */
void Start_Mode(void *argument)
{
  /* USER CODE BEGIN Start_Mode */
	u8 value = 0;
  /* Infinite loop */
  for(;;)
  {
			if(PS2_TaskHandle)	vTaskSuspend(PS2_TaskHandle);
			if(Find_BallHandle)	vTaskSuspend(Find_BallHandle);
			if(Push_ballHandle)	vTaskSuspend(Push_ballHandle);
			if(Catch_ballHandle)	vTaskSuspend(Catch_ballHandle);
			if(Find_Target_LocHandle)	vTaskSuspend(Find_Target_LocHandle);
			if(testHandle)	vTaskSuspend(testHandle);
			value = KEY_Scan(0);
			if(value)
			{			
					if(value == KEY0_PRES)
							mode = 1;//Find_Ball_Red
					else if(value == KEY1_PRES)
							mode = 2;//Find_Ball_Blue
					if(PS2_TaskHandle)	vTaskResume(PS2_TaskHandle);
					if(Find_BallHandle)	vTaskResume(Find_BallHandle);
					if(Push_ballHandle)	vTaskResume(Push_ballHandle);
					if(Catch_ballHandle)	vTaskResume(Catch_ballHandle);
					if(testHandle)	vTaskResume(testHandle);
					
					delay_ms(500);
					Set_Pwm(450,450, 450, 450);
					delay_ms(700);
					Set_Pwm(200,200, 200, 200);
					delay_ms(200);
					
					vTaskDelete(ModeHandle);
					ModeHandle = NULL;
			}
			osDelay(1);
  }
  /* USER CODE END Start_Mode */
}

/* USER CODE BEGIN Header_StartPS2_Task */
/**
* @brief Function implementing the PS2_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPS2_Task */
void StartPS2_Task(void *argument)
{
  /* USER CODE BEGIN StartPS2_Task */
  /* Infinite loop */
  for(;;)
  {
		PS2_Read();
		
		if (PS2_ON_Flag)
		{
				if(Find_BallHandle)	vTaskSuspend(Find_BallHandle);
				if(Push_ballHandle)	vTaskSuspend(Push_ballHandle);
				if(Catch_ballHandle)	vTaskSuspend(Catch_ballHandle);
				if(Find_Target_LocHandle)	vTaskSuspend(Find_Target_LocHandle);
				if(testHandle)	vTaskSuspend(testHandle);
				printf("PS2_LX%d   PS2_LY%d   PS2_RX%d   PS2_RY%d   PS2_KEY%d\r\n", PS2_LX, PS2_LY, PS2_RX, PS2_RY, PS2_KEY);
				if(PS2_KEY == 7)
				{
						Servo(1);
				}
				if(PS2_KEY == 5)
				{
						Servo(0);
				}
				if(PS2_KEY == 6)
				{
						Servo(2);
				}
				if(PS2_KEY == 8)
				{
						Servo(3);
				}
				if(PS2_LY == 128)
				{
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);	
				}
				if(PS2_RY == 128)
				{
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,0);
				}
				if(PS2_LY < 128)
				{
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,350);
						
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);      //
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,350);
				}
				if(PS2_LY > 128)
				{
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,350);
						
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);    //
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,350);
				}

				if(PS2_RY < 128)
				{
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);      //
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,350);
						
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,350);
				}
				if(PS2_RY > 128)
				{
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);     //
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,350);
						
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,350);
				}
		}
    osDelay(1);
  }
  /* USER CODE END StartPS2_Task */
}

/* USER CODE BEGIN Header_Start_Find_Ball */
/**
* @brief Function implementing the Find_Ball thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Find_Ball */
void Start_Find_Ball(void *argument)
{
  /* USER CODE BEGIN Start_Find_Ball */
	int16_t Last_location_data = 0;
  /* Infinite loop */
  for(;;)
  {
			Get_Motor_Speed();
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
			data[2] = 0x00;
			Servo(1);
			
			if(eTaskGetState(Find_Target_LocHandle) != eSuspended)
			{
					vTaskSuspend(Find_Target_LocHandle);
			}
			
			if((location_data[1] == 600) && (location_data[1] != Last_location_data))
			{
					Set_Pwm(0, 0, 0, 0);
					delay_ms(100);
					osSemaphoreRelease(get_ballHandle);
					location_data[0] = location_data[1] = 0;
			}
			Last_location_data = location_data[1];
			if(location_data[1] == 0)
			{
					location_data[0] = location_data[1] = 0;
					Set_Pwm(0, 0,  0, 0);
			}
			else if(location_data[1] == 900)
			{
					Empty(0);
			}
			
			if(location_data[1] < 400)
			{	
					Drive_Motor();
					
					//速度闭环控制计算各电机PWM值，PWM代表车轮实际转速对应PWM值
					MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
					MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
					MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
					MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
								 
					Limit_Pwm(300); 
					
					Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm); 
			}
			osDelay(1);
  }
  /* USER CODE END Start_Find_Ball */
}

/* USER CODE BEGIN Header_StartPush_ball */
/**
* @brief Function implementing the Push_ball thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPush_ball */
void StartPush_ball(void *argument)
{
  /* USER CODE BEGIN StartPush_ball */
  /* Infinite loop */
  for(;;)
  {
			osSemaphoreAcquire(put_ballHandle,portMAX_DELAY);
			if(Find_Target_LocHandle)	vTaskSuspend(Find_Target_LocHandle);
			
			Set_Pwm(0, 0, 0, 0);
			delay_ms(1000);
			Servo(0);
			delay_ms(100);
			Set_Pwm(-300,-300, -300, -300);
			delay_ms(400);
			Set_Pwm(0, 0, 0, 0);
			delay_ms(500);
			Set_Pwm(600,600, 600, 600);
			delay_ms(600);
			Set_Pwm(0, 0, 0, 0);
			delay_ms(100);
			Servo(2);
			Set_Pwm(0, 0, 0, 0);
			delay_ms(500);
			Set_Pwm(-300, -300, -300, -300);
			delay_ms(400);
			
			Set_Pwm(400, 400, -400, -400);  
			delay_ms(1000);
			Set_Pwm(0, 0, 0, 0);
			Servo(1);
			
			data[2] = 0x00;

			Set_Pwm(0, 0, 0, 0);
			
			
			if(Find_BallHandle)	vTaskResume(Find_BallHandle);
			
			osDelay(1);
  }
  /* USER CODE END StartPush_ball */
}

/* USER CODE BEGIN Header_StartCatch_ball */
/**
* @brief Function implementing the Catch_ball thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCatch_ball */
void StartCatch_ball(void *argument)
{
  /* USER CODE BEGIN StartCatch_ball */
	u8 defeat = 0;
	u8 i = 0;
  /* Infinite loop */
  for(;;)
  {
			osSemaphoreAcquire(get_ballHandle,portMAX_DELAY);
			if(Find_BallHandle)	vTaskSuspend(Find_BallHandle);
			
			Set_Pwm(0, 0, 0, 0);
			Servo(0);
			Set_Pwm(0, 0, 0, 0);
			delay_ms(500);
			
			while(location_data[0] != 800)
			{
					if(location_data[0] == 750)
					{
							defeat = 1;
							break;
					}
					if(location_data[0] == 700)
					{
							Set_Pwm(-300, -300, -300, -300 );
							delay_ms(700);
							defeat = 1;
							break;
					}
					Set_Pwm(125, 125, 125, 125 );
			}
			Set_Pwm(0, 0, 0, 0);
			Servo(1);
			delay_ms(200);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
			while(1)
			{
					//delay_ms(20);
					
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
					
					if(location_data[0] == 850)
					{
							//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
							//delay_ms(100);
							if(location_data[0] == 850)
							{
									break;
							}
							else if(location_data[0] == 860)
							{
									defeat = 1;
									Servo(0);
									delay_ms(500);
									Set_Pwm(300,300, 300, 300);
									delay_ms(400);
									Set_Pwm(-300, -300, -300, -300 );
									delay_ms(1000);
									Set_Pwm(300, 300, -300, -300 );
									delay_ms(500);
									Set_Pwm(0, 0, 0, 0 );
									Servo(1);
									break;
							}
					}
					else if(location_data[0] == 860)
					{
							//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
							if(location_data[0] == 850)
							{
									break;
							}
							else if(location_data[0] == 860)
							{
									defeat = 1;
									Servo(0);
									delay_ms(500);
									Set_Pwm(300,300, 300, 300);
									delay_ms(400);
									Set_Pwm(-300, -300, -300, -300 );
									delay_ms(1000);
									Set_Pwm(300, 300, -300, -300 );
									delay_ms(500);
									Set_Pwm(0, 0, 0, 0 );
									Servo(1);
									break;
							}							
					}
					if(location_data[0] == 750)
					{
							defeat = 1;
							break;
					}
//					if(location_data[0] == 900)
//					{
//							cat++;
//							if(cat == 30000)
//							{
//									cat = 0;
//									break;
//							}
//					}
			}
			cat = 0;
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
			Set_Pwm(-200,-200, -200, -200);
			delay_ms(300);
			Set_Pwm(200,200, 200, 200);
			delay_ms(300);
			
			Set_Pwm(0, 0, 0, 0);
			delay_ms(100);
			
			for(i = 0;i<10;i++)
			{
					data[2] = 0x04;
					HAL_UART_Transmit_IT(&huart4, data, sizeof(data));
			}
			
			if(defeat)
			{
					defeat = 0;
					if(Find_BallHandle)	vTaskResume(Find_BallHandle);
			}
			else
			{
					data[2] = 0x01;
					//HAL_UART_Transmit_IT(&huart4, data, sizeof(data));
					
					if(Find_Target_LocHandle)	vTaskResume(Find_Target_LocHandle);
			}
			osDelay(1);
  }
  /* USER CODE END StartCatch_ball */
}

/* USER CODE BEGIN Header_Start_Find_Target_Location */
/**
* @brief Function implementing the Find_Target_Loc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Find_Target_Location */
void Start_Find_Target_Location(void *argument)
{
  /* USER CODE BEGIN Start_Find_Target_Location */
  /* Infinite loop */
  for(;;)
  {
			Get_Motor_Speed();
			
			data[2] = 0x01;
			Servo(1);
			state_flag = 0;
			
			if(eTaskGetState(Find_BallHandle) != eSuspended)
			{
					vTaskSuspend(Find_BallHandle);
			}
			
			if(location_data[1] == 650)
			{
					Set_Pwm(0, 0, 0, 0);
					state_flag = 1;
					osSemaphoreRelease(put_ballHandle);
					location_data[0] = location_data[1] = 0;
			}
			else if(location_data[1] == 0)
			{
					location_data[0] = location_data[1] = 0;
					Set_Pwm(0, 0,  0, 0);
			}
			else if(location_data[1] == 900)
			{
					cnt++;
					if(cnt == 60)
					{
							cnt = 0;
							Empty(1);
					}		
			}
			
			if(location_data[1] < 400)
			{
					cnt = 0;
					if(location_data[1] > 80)
					{
							location_data[1] = 100;
					}
					else
					{
							location_data[1] = 80;
					}
					
					Drive_Motor();
					
					//速度闭环控制计算各电机PWM值，PWM代表车轮实际转速对应PWM值
					MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
					MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
					MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
					MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
								 
					Limit_Pwm(300); 
					
					Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm); 
			}
			osDelay(1);
  }
  /* USER CODE END Start_Find_Target_Location */
}

/* USER CODE BEGIN Header_Start_test */
/**
* @brief Function implementing the test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_test */
void Start_test(void *argument)
{
  /* USER CODE BEGIN Start_test */
	//int16_t Last_location_data = 0;
	
  /* Infinite loop */
  while(1)
  {
			if(eTaskGetState(Find_BallHandle) != eSuspended)
			{
					HAL_UART_Transmit_IT(&huart4, data, sizeof(data));
					delay_ms(10);
			}
			if(eTaskGetState(Find_Target_LocHandle) != eSuspended)
			{
					HAL_UART_Transmit_IT(&huart4, data, sizeof(data));
					delay_ms(10);
			}
			
//			switch(run_state)
//			{
//				case FIND_BALL:
//						if(location_data[1] == 500)
//						{
//								run_state = FIND_BALL;
//								//HAL_UART_Transmit_IT(&huart4, data, sizeof(data));
//						}
//						else if(location_data[1] == 550)
//						{
//								run_state = FIND_TARGET;
//								break;
//						}
//						
//						if((location_data[1] != 500) && (location_data[1] != 550))
//						{
//								Get_Motor_Speed();
//						
//								if((location_data[1] == 600) && (location_data[1] != Last_location_data))
//								{
//										Set_Pwm(0, 0, 0, 0);
//										delay_ms(100);

//										
//										Set_Pwm(0, 0, 0, 0);
//										Servo(0);
//										Set_Pwm(0, 0, 0, 0);
//										delay_ms(500);
//										while(location_data[0] != 800)
//										{
//												if(location_data[0] == 700)
//												{
//														Set_Pwm(-200, -200, -200, -200 );
//														delay_ms(500);
//														break;
//												}
//												Set_Pwm(125, 125, 125, 125 );
//										}
//										Set_Pwm(0, 0, 0, 0);
//										Servo(1);
//										delay_ms(500);
//										Set_Pwm(-200,-200, -200, -200);
//										delay_ms(300);
//										Set_Pwm(200,200, 200, 200);
//										delay_ms(300);
//										Set_Pwm(0, 0, 0, 0);
//										delay_ms(500); 
//										
//										
//										run_state = FIND_TARGET;
//										location_data[0] = location_data[1] = 0;
//								}
//								Last_location_data = location_data[1];
//								if(location_data[1] == 0)
//								{
//										Set_Pwm(0, 0,  0, 0);
//								}
//								if(location_data[1] == 900)
//								{
//										Empty(0);
//								}
//								
//								else
//								{
//										Drive_Motor();
//									
//										//速度闭环控制计算各电机PWM值，PWM代表车轮实际转速对应PWM值
//										MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
//										MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
//										MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
//										MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
//													 
//										Limit_Pwm(300); 
//										
//										Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm); 
//								}
//						}
//				break;
//				case FIND_TARGET:
//						if(location_data[1] == 500)
//						{
//								run_state = FIND_BALL;
//								//HAL_UART_Transmit_IT(&huart4, data, sizeof(data));
//								break;
//						}
//						else if(location_data[1] == 550)
//						{
//								run_state = FIND_TARGET;
//						}
//						
//						if((location_data[1] != 500) && (location_data[1] != 550))
//						{
//								Get_Motor_Speed();
//					
//								if(location_data[1] == 650)
//								{
//										Set_Pwm(0, 0, 0, 0);
//									
//									
//										Set_Pwm(0, 0, 0, 0);
//										delay_ms(100);
//										Set_Pwm(-100,-100, -100, -100);
//										delay_ms(600);
//										Set_Pwm(0, 0, 0, 0);
//										delay_ms(500);
//										Set_Pwm(350,350, 350, 350);
//										delay_ms(400);
//										Set_Pwm(0, 0, 0, 0);
//										delay_ms(100);
//										Servo(2);
//										Set_Pwm(0, 0, 0, 0);
//										delay_ms(500);
//										Set_Pwm(-300, -300, -300, -300);
//										delay_ms(400);
//										Servo(1);
//										Set_Pwm(400, 400, -400, -400);  
//										delay_ms(1000);
//										Set_Pwm(0, 0, 0, 0);
//									
//										
//										HAL_UART_Transmit_IT(&huart4, data, sizeof(data));
//										delay_ms(500);
//										Set_Pwm(0, 0, 0, 0);
//									
//									
//										run_state = FIND_BALL;
//										location_data[0] = location_data[1] = 0;
//								}
//								if(location_data[1] == 0)
//								{
//										Set_Pwm(0, 0,  0, 0);
//								}
//								if(location_data[1] == 900)
//								{
//										Empty(1);
//								}
//								
//								else
//								{
//										location_data[1] = 80;
//										Drive_Motor();
//										
//										//速度闭环控制计算各电机PWM值，PWM代表车轮实际转速对应PWM值
//										MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
//										MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
//										MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
//										MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
//													 
//										Limit_Pwm(300); 
//										
//										Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm); 
//								}
//						}
//				break;
//			}
			osDelay(1);
  }
  /* USER CODE END Start_test */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

