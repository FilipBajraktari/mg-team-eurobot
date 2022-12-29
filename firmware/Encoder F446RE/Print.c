#include "Print.h"

static uint32_t baudrateLUT[] = {9600, 19200, 57600, 115200, 230400, 500000, 1000000, 1500000, 2000000};
static USART_TypeDef *P_UART;

int printInit(USART_TypeDef *UART, uint32_t baudrate)
{
	if(UART != USART2) return Print_ERR2;
	
	int p = 0;
	for(uint32_t i = 0; i < (sizeof(baudrateLUT) / sizeof(uint32_t)); i++)
	{
		if(baudrate == baudrateLUT[i]) p = 1;
	}
	if(p == 0) return Print_ERR1;
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //GPIO A  ENABLE
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; //UART2 ENABLE
	
	GPIOA->MODER &= ~GPIO_MODER_MODER2_Msk;  //GPIO A PIN 2 ALTERNATE FUNCTION MODE
	GPIOA->MODER |= GPIO_MODER_MODER2_1;
	
	GPIOA->MODER &= ~GPIO_MODER_MODER3_Msk;  //GPIO A PIN 3 ALTERNATE FUNCTION MODE
	GPIOA->MODER |= GPIO_MODER_MODER3_1;
	
	GPIOA->AFR[0] |= (GPIO_AFRL_AFRL2_0 | GPIO_AFRL_AFRL2_1 | GPIO_AFRL_AFRL2_2); //GPIO ALTERNATE FUNCTION REGISTER PINS 2 AND 3 SET 7
	GPIOA->AFR[0] |= (GPIO_AFRL_AFRL3_0 | GPIO_AFRL_AFRL3_1  | GPIO_AFRL_AFRL3_2);

	UART->CR1 |= USART_CR1_TE; //USART2 TRANSMITTER ENABLE
	UART->CR1 |= USART_CR1_RE; //USART2 RECIEVER ENABLE
	
	switch (baudrate)
	{
		//case(9600): UART->BRR = 0x1117; break;
		//case(19200): UART->BRR = 0x88B; break;
		//case(57600): UART->BRR = 0x2D9; break;
		case(115200): UART->BRR = 0x187; break;
		//case(230400): UART->BRR = 0xB6; break;
		//case(500000): UART->BRR = 0x54; break;
		//case(1000000): UART->BRR = 0x2A; break;
		//case(1500000): UART->BRR = 0x1C; break;
		//case(2000000): UART->BRR = 0x15; break;
	}
	
	UART->CR1 |= USART_CR1_UE; //USART2 ENABLE
	
	P_UART = UART;
	
	return Print_OK;
}

void print(char *msg, ...)
{
	char buff[80];
	
	va_list args;
	va_start(args, msg);
	vsprintf(buff, msg, args);
	
	for(uint8_t i = 0; i < strlen(buff); i++)
	{
		while(!(P_UART->SR & USART_SR_TXE));
		P_UART->DR = buff[i];
	}
	
}

void printSupportedBaudrates(void)
{
	print("Suported baudrates:\r\n");
	
	for(uint32_t i = 0; i < (sizeof(baudrateLUT) / sizeof(uint32_t)); i++)
	{
		print("%d\r\n", baudrateLUT[i]);
	}
}

