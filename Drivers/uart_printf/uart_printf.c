#include "uart_printf.h"

#include "stm32f1xx_hal.h"

#include <stdarg.h>  // va_list va_start va_end
#include <stdio.h>  // sprintf
#include <string.h>  // strlen

extern UART_HandleTypeDef huart1;

void UART_Printf(const char* fmt, ...) {
    char buff[64];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);
    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
    va_end(args);
}
