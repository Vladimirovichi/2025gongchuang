#include "encoder.h"


int Get_Encoder_Data(uint8_t TIMx)
{
	int cnt;
	switch(TIMx)
	{
		case 2:
				cnt = (short)TIM2 -> CNT;   TIM2 -> CNT=0;
		break;
		case 3:
				cnt = (short)TIM3 -> CNT;   TIM3 -> CNT=0;
		break;
		case 4:
				cnt = -(short)TIM4 -> CNT;   TIM4 -> CNT=0;
		break;
		case 5:
				cnt = -(short)TIM5 -> CNT;   TIM5 -> CNT=0;
		break;
	}
	return cnt;
}
