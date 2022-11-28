#include "stm32f401xe.h"
#include "TIM_EncMode.h"
#include "Print.h"
#include "NucleoSetup.h"

void TIM2_IRQHandler(void);
void TIM2Init(void);

#define tim32bit TIM5
//#define tim16bit_m TIM3
//#define tim16bit_s TIM4
#define delay() for(int i = 0; i < 1000000;i++)

int main()
{
	nucleoLedInit();
	rccSetup();
	TIM2Init();
	
	encoderInit32bit(tim32bit, 0xFFFFFFFF);
	//encoderInit16bit(tim16bit_m, tim16bit_s, 0xFFFFFFFF);
	printInit(USART2);
	
	//GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0_Msk | GPIO_PUPDR_PUPD1_Msk);  //GPIO A PINS 0 AND 1 PULL UP
	//GPIOA->PUPDR |= (GPIO_PUPDR_PUPD0_0 | GPIO_PUPDR_PUPD1_0);
	
	while(1)
	{
		
		delay();
		
		GPIOA->ODR |= GPIO_ODR_OD5;
	
		delay();
	
		GPIOA->ODR &= ~GPIO_ODR_OD5;
	}
}


//*****************< INTERRUPT SETUP AND HANDLER >***********************

void TIM2Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	TIM2->PSC = 84 - 1; // 84MHz / 84 = 1MHz = TIM2 CLOCK
	TIM2->ARR = 200 - 1; 
	TIM2->CR1 = TIM_CR1_URS;
	TIM2->DIER = TIM_DIER_UIE;
	TIM2->EGR = TIM_EGR_UG; //TIMER 2 SETUP UPDATE INTERRUPT
	TIM2->CR1 = TIM_CR1_CEN; //TIMER 2 ENABLE
	
	NVIC_EnableIRQ(TIM2_IRQn); //TIMER 2 ENABLE INTERRUPT HANDLER
	
}

void TIM2_IRQHandler(void) //TIMER 2 INTERRUPT HANDLER
{
	TIM2->SR &= ~(TIM_SR_UIF); 
	
	print("%d\r\n", encoderCount32bit(tim32bit));
	//print("data:%d\r\n", encoderCount16bit(tim16bit_m, tim16bit_s));
}
