#ifndef Print
#define Print

#include "stm32f401xe.h"
#include <stdint.h>
#include "string.h"
#include "stdlib.h"
#include "stdarg.h"

static USART_TypeDef *P_UART;

void printInit(USART_TypeDef *UART);	
void print(char *msg, ...);

#endif
