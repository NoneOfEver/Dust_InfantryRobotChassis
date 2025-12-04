/**
 * @file bsp_log.c
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "bsp_log.h"

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include <stdio.h>


void log_init()
{
    SEGGER_RTT_Init();
}

int print_log(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int n = SEGGER_RTT_vprintf(BUFFER_INDEX, fmt, &args); // 一次可以开启多个buffer(多个终端),我们只用一个
    va_end(args);
    return n;
}

void float2str(char *str, float va)
{
    int flag = va < 0;
    int head = (int)va;
    int point = (int)((va - head) * 1000);
    head = abs(head);
    point = abs(point);
    if (flag)
        sprintf(str, "-%d.%d", head, point);
    else
        sprintf(str, "%d.%d", head, point);
}

