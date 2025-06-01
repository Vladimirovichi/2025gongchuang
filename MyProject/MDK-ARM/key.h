#ifndef __KEY_H__
#define __KEY_H__
 
#include "main.h"


#define KEY0        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)  //KEY0按键PH3
#define KEY1        HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)  //KEY1按键PH2

#define KEY0_PRES 	1  	//KEY0按下后返回值
#define KEY1_PRES	2	//KEY1按下后返回值


u8 KEY_Scan(u8 mode); //按键扫描函数


#endif
