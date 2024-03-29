#include "NucleoSetup.h"
#include "stm32f446xx.h"   


void rccSetup(void)
{
	FLASH->ACR |= FLASH_ACR_PRFTEN; //FLASH PREFETCH ENABLE
	FLASH->ACR &= ~(FLASH_ACR_LATENCY_Msk);
	FLASH->ACR |= 0x5; //FLASH LATENCY 2 WAIT STATE
	
	FLASH->ACR |= FLASH_ACR_ICEN;  //FLASH INSTRUCTION CACHE ENABLE
	FLASH->ACR |= FLASH_ACR_DCEN;	//FLASH DATA CACHE ENABLE
	
	RCC->APB1ENR |= RCC_APB1ENR_PWREN; //POWER INTERFACE CLOCK ENABLE
	
	RCC->PLLCFGR |= (RCC_PLLCFGR_PLLQ_1 | RCC_PLLCFGR_PLLN_4 | RCC_PLLCFGR_PLLN_5 | RCC_PLLCFGR_PLLN_2 
	| RCC_PLLCFGR_PLLM_3); 
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLQ_2 | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLM_4); //PLL CONFIGURATION
	
	RCC->CFGR |= RCC_CFGR_PPRE2_2; //APB1 PRESCALER
	RCC->CFGR |= (RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_0); //APB1 PRESCALER
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

