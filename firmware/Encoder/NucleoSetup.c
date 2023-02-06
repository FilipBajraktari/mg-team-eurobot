#include "NucleoSetup.h"
#include "stm32f401xe.h"   


void rccSetup(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_PWREN; //POWER INTERFACE CLOCK ENABLE
	
	FLASH->ACR |= FLASH_ACR_PRFTEN; //FLASH PREFETCH ENABLE
	FLASH->ACR &= ~(FLASH_ACR_LATENCY_Msk);
	FLASH->ACR |= 0x2; //FLASH LATENCY 2 WAIT STATE
	
	FLASH->ACR |= FLASH_ACR_ICEN;  //FLASH INSTRUCTION CACHE ENABLE
	FLASH->ACR |= FLASH_ACR_DCEN;	//FLASH DATA CACHE ENABLE
	
	RCC->PLLCFGR |= (RCC_PLLCFGR_PLLM_4 | RCC_PLLCFGR_PLLN_4 | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_8 
	| RCC_PLLCFGR_PLLP_0 | RCC_PLLCFGR_PLLQ_0 | RCC_PLLCFGR_PLLQ_1 | RCC_PLLCFGR_PLLQ_2); 
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN_7); //PLL CONFIGURATION
	
	RCC->CFGR |= RCC_CFGR_PPRE1_2; //APB1 PRESCALER = /2
	RCC->CFGR |= RCC_CFGR_SW_1; //SYSTEM CLOCK SWITCH
	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; //SYSTEM CONFIG CONTROLLER CLOCK ENABLE
	
	RCC->CR |= RCC_CR_PLLON; //PLL ON
	
}

void nucleoLedInit(void)
{
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  //GPIO A  ENABLE
	
	GPIOA->MODER &= ~GPIO_MODER_MODER5_Msk; //GPIO A PIN 5 OUTPUT
	GPIOA->MODER |= GPIO_MODER_MODER5_0;

}

