#ifndef DEBUG_H
#define DEBUG_H

#include "usart.h"
#include <stdio.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart1;

/* Printf qua USART1 */
void debug_printf(const char *fmt, ...);

#endif
