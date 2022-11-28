#ifndef TIM_EncMode
#define TIM_EncMode

#include "stm32f401xe.h"


typedef enum {
  TIM_EncMode_OK = 0,
  TIM_EncMode_ERR1 = 1,
	TIM_EncMode_ERR2 = 2,  
}TIM_EncMode_Error;

int encoderInit32bit(TIM_TypeDef *TIM, uint32_t arr);
int32_t encoderCount32bit(TIM_TypeDef *TIM);
int encoderInit16bit(TIM_TypeDef *TIM_M, TIM_TypeDef *TIM_S, uint32_t arr);
int32_t encoderCount16bit(TIM_TypeDef *TIM_M, TIM_TypeDef *TIM_S);


#endif
