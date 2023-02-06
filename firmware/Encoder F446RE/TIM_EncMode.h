#ifndef TIM_EncMode
#define TIM_EncMode

#include "stm32f446xx.h"


typedef enum {
  TIM_EncMode_OK = 0,
  TIM_EncMode_ERR1 = 1,
	TIM_EncMode_ERR2 = 2,  
}TIM_EncMode_Error;

typedef enum {
	false = 0,
	FALSE = 0,
	true = 1,
	TRUE = 1,
}bool;

int encoderInit32bit(TIM_TypeDef *TIM, int pullUp);
int32_t encoderCount32bit(TIM_TypeDef *TIM);
int encoderInit16bit(TIM_TypeDef *TIM, int pullUp);
int32_t encoderCount16bit(TIM_TypeDef *TIM);


#endif
