#ifndef __KEY_H__
#define __KEY_H__
 
#include "main.h"


#define KEY0        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)  //KEY0����PH3
#define KEY1        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)  //KEY1����PH2

#define KEY0_PRES 	1  	//KEY0���º󷵻�ֵ
#define KEY1_PRES	2	//KEY1���º󷵻�ֵ


u8 KEY_Scan(u8 mode); //����ɨ�躯��


#endif
