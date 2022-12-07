#ifndef Print
#define Print

#include "stm32f401xe.h"
#include <stdint.h>
#include "string.h"
#include "stdlib.h"
#include "stdarg.h"

typedef enum {
  Print_OK = 0,
  Print_ERR1 = 1,
	Print_ERR2 = 2,
}Print_Error;

int printInit(USART_TypeDef *UART, uint32_t baudrate);	
void print(char *msg, ...);
void printSupportedBaudrates(void);

#endif
