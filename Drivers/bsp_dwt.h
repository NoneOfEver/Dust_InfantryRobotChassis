/**
 * @file bsp_dwt.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-02
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef BSP_BSP_DWT_H_
#define BSP_BSP_DWT_H_

#include "main.h"
#include "stdint.h"
#include "bsp_log.h"

struct DwtTime
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
};

/**
 * @brief 该宏用于计算代码段执行时间,单位为秒/s,返回值为float类型
 *        首先需要创建一个float类型的变量,用于存储时间间隔
 *        计算得到的时间间隔同时还会通过RTT打印到日志终端,你也可以将你的dt变量添加到查看
 */
#define TIME_ELAPSE(dt, code)                    \
    do                                           \
    {                                            \
        float tstart = dwt_get_timeline_s();      \
        code;                                    \
        dt = dwt_get_timeline_s() - tstart;       \
        LOGINFO("[DWT] " #dt " = %f s\r\n", dt); \
    } while (0)

/**
 * @brief 初始化DWT,传入参数为CPU频率,单位MHz
 *
 * @param cpu_frequncy_mhz c板为168MHz,A板为180MHz
 */
void dwt_init(uint32_t cpu_frequncy_mhz);

/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s
 *
 * @param cnt_last 上一次调用的时间戳
 * @return float 时间间隔,单位为秒/s
 */
float dwt_get_delta_t(uint32_t *cnt_last);

/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s,高精度
 *
 * @param cnt_last 上一次调用的时间戳
 * @return double 时间间隔,单位为秒/s
 */
double dwt_get_delta_t_64(uint32_t *cnt_last);

/**
 * @brief 获取当前时间,单位为秒/s,即初始化后的时间
 *
 * @return float 时间轴
 */
float dwt_get_timeline_s(void);

/**
 * @brief 获取当前时间,单位为毫秒/ms,即初始化后的时间
 *
 * @return float
 */
float dwt_get_timeline_ms(void);

/**
 * @brief 获取当前时间,单位为微秒/us,即初始化后的时间
 *
 * @return uint64_t
 */
uint64_t dwt_get_timeline_us(void);

/**
 * @brief DWT延时函数,单位为秒/s
 * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
 * @note 禁止在__disable_irq()和__enable_irq()之间使用HAL_Delay()函数,应使用本函数
 *
 * @param Delay 延时时间,单位为秒/s
 */
void dwt_delay(float delay);

/**
 * @brief DWT更新时间轴函数,会被三个timeline函数调用
 * @attention 如果长时间不调用timeline函数,则需要手动调用该函数更新时间轴,否则CYCCNT溢出后定时和时间轴不准确
 */
void dwt_systime_update(void);

#endif /* BSP_BSP_DWT_H_ */
/************************ COPYRIGHT(C) HNUST-DUST **************************/
