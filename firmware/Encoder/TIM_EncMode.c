#include "TIM_EncMode.h"

int encoderInit32bit(TIM_TypeDef *TIM,  uint32_t arr)
{
	if(TIM != TIM2 && TIM != TIM5) return TIM_EncMode_ERR1;
	
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  //GPIO A  ENABLE	
		
	if(TIM == TIM2)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //ENABLE CLOCK FOR TIMER 2
	}
	else
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; //ENABLE CLOCK FOR TIMER 5
	}
	
	GPIOA->MODER &= ~GPIO_MODER_MODER0_Msk;  //GPIO A PIN 0 ALTERNATE FUNCTION MODE
	GPIOA->MODER |= GPIO_MODER_MODER0_1;

	GPIOA->MODER &= ~GPIO_MODER_MODER1_Msk;  //GPIO A PIN 1 ALTERNATE FUNCTION MODE
	GPIOA->MODER |= GPIO_MODER_MODER1_1;
	if(TIM == TIM2)
	{
		GPIOA->AFR[0] |= 0x01 << 0; //GPIO ALTERNATE FUNCTION REGISTER PIN 0 SET 1
		GPIOA->AFR[0] |= 0x01 << 4; //GPIO ALTERNATE FUNCTION REGISTER PIN 1 SET 1
	}
	else
	{
		GPIOA->AFR[0] |= 0x02 << 0; //GPIO ALTERNATE FUNCTION REGISTER PIN 0 SET 2
		GPIOA->AFR[0] |= 0x02 << 4; //GPIO ALTERNATE FUNCTION REGISTER PIN 1 SET 2
	}
	
	TIM->CCMR1 &= ~TIM_CCMR1_CC1S_Msk; //MAP TI1FP1 ON T1
	TIM->CCMR1 |= TIM_CCMR1_CC1S_0;

	TIM->CCMR1 &= ~TIM_CCMR1_CC2S_Msk;  //MAP TI2FP2 ON T2
	TIM->CCMR1 |= TIM_CCMR1_CC2S_0;
	
	TIM->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC2P | TIM_CCER_CC2NP); //TI1FP1 AND TI2FP2 NONINVERTED/RISSING EDGE
	
	TIM->CCMR1 &= ~(TIM_CCMR1_IC1F_Msk | TIM_CCMR1_IC2F_Msk); //INPUT FILTERS OFF

	TIM->SMCR &= ~TIM_SMCR_SMS_Msk;
	TIM->SMCR |= (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);  //SMS = 011: BOTH INPUTS ARE ACTIVE ON BOTH EDGES

	TIM->ARR = arr;

	TIM->CR1 |= TIM_CR1_CEN; //TIMER COUNTER ENABLE
	
	
	return TIM_EncMode_OK;
}

int32_t encoderCount32bit(TIM_TypeDef *TIM)
{
	return (int32_t)TIM->CNT;
}

int encoderInit16bit(TIM_TypeDef *TIM_M, TIM_TypeDef *TIM_S, uint32_t arr)
{
	if(TIM_M != TIM3 && TIM_M != TIM4) return TIM_EncMode_ERR1;
	if(TIM_S != TIM3 && TIM_S != TIM4) return TIM_EncMode_ERR1;
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //ENABLE CLOCK FOR TIMER 3
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; //ENABLE CLOCK FOR TIMER 4
	
	if(TIM_M == TIM3)
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	//GPIO A ENABLE	
			
		GPIOA->MODER &= ~GPIO_MODER_MODER6_Msk;  //GPIO A PIN 6 ALTERNATE FUNCTION MODE
		GPIOA->MODER |= GPIO_MODER_MODER6_1;

		GPIOA->MODER &= ~GPIO_MODER_MODER7_Msk;  //GPIO A PIN 7 ALTERNATE FUNCTION MODE
		GPIOA->MODER |= GPIO_MODER_MODER7_1;
	
		GPIOA->AFR[0] |= 0x02 << 24; //GPIO ALTERNATE FUNCTION REGISTER PIN 6 SET 2
		GPIOA->AFR[0] |= 0x02 << 28; //GPIO ALTERNATE FUNCTION REGISTER PIN 7 SET 2
	}
	else
	{
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;	//GPIO D ENABLE	
	
		GPIOD->MODER &= ~GPIO_MODER_MODER12_Msk;  //GPIO D PIN 12 ALTERNATE FUNCTION MODE
		GPIOD->MODER |= GPIO_MODER_MODER12_1;

		GPIOD->MODER &= ~GPIO_MODER_MODER13_Msk;  //GPIO D PIN 13 ALTERNATE FUNCTION MODE
		GPIOD->MODER |= GPIO_MODER_MODER13_1;
	
		GPIOD->AFR[1] |= 0x02 << 16; //GPIO ALTERNATE FUNCTION REGISTER PIN 12 SET 2
		GPIOD->AFR[1] |= 0x02 << 20; //GPIO ALTERNATE FUNCTION REGISTER PIN 13 SET 2
	}
	TIM_M->CCMR1 &= ~TIM_CCMR1_CC1S_Msk; //MAP TI1FP1 ON T1
	TIM_M->CCMR1 |= TIM_CCMR1_CC1S_0;

	TIM_M->CCMR1 &= ~TIM_CCMR1_CC2S_Msk;  //MAP TI2FP2 ON T2
	TIM_M->CCMR1 |= TIM_CCMR1_CC2S_0;
	
	TIM_M->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC2P | TIM_CCER_CC2NP); //TI1FP1 AND TI2FP2 NONINVERTED/RISSING EDGE
	
	TIM_M->CCMR1 &= ~(TIM_CCMR1_IC1F_Msk | TIM_CCMR1_IC2F_Msk); //INPUT FILTERS OFF

	TIM_M->SMCR &= ~TIM_SMCR_SMS_Msk;
	TIM_M->SMCR |= (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);  //SMS = 011: BOTH INPUTS ARE ACTIVE ON BOTH EDGES

	TIM_M->ARR = arr & 0x0000FFFF;

	TIM_M->CR1 |= TIM_CR1_CEN; //TIMER COUNTER ENABLE
	
	
	
	return TIM_EncMode_OK;
}


int32_t encoderCount16bit(TIM_TypeDef *TIM_M, TIM_TypeDef *TIM_S)
{
	return (int16_t)TIM_M->CNT;
}


