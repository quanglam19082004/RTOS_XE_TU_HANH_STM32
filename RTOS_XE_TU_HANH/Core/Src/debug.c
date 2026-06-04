#include "debug.h"
#include <stdio.h>
#include <string.h>

static char debug_buffer[256];

void debug_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(debug_buffer, sizeof(debug_buffer), fmt, args);
    va_end(args);
    
    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t*)debug_buffer, len, 100);
    }
}
