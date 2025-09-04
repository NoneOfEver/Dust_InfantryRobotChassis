/**
 * @file bsp_dwt.cpp
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-02
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "bsp_dwt.h"
#include "cmsis_os.h"

static DwtTime  systime;
static uint32_t cpu_frequncy_hz, cpu_frequncy_hz_ms, cpu_frequncy_hz_us;
static uint32_t cyccnt_rount_count;
static uint32_t cyccnt_last;
static uint64_t cyccnt64;

/**
 * @brief 私有函数,用于检查DWT CYCCNT寄存器是否溢出,并更新cyccnt_rount_count
 * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出
 *
 * @note 更好的方案是为dwt的时间更新单独设置一个任务?
 *       不过,使用dwt的初衷是定时不被中断/任务等因素影响,因此该实现仍然有其存在的意义
 *
 */
static void dwt_cnt_update(void)
{
    /**
     *bit_locker
     *一个位锁，防止中断中调用DWTGetDeltaT或DWTGetTimeline函数更新DWT维护的时间时打断任务中
     *的相同函数导致计数被重复更新，或引起错误的DWT溢出检测。
     */
    static volatile uint8_t bit_locker = 0;
    if (!bit_locker) {
        bit_locker                = 1;
        volatile uint32_t cnt_now = DWT->CYCCNT;
        if (cnt_now < cyccnt_last)
            cyccnt_rount_count++;

        cyccnt_last = DWT->CYCCNT;
        bit_locker  = 0;
    }
}

void dwt_init(uint32_t cpu_frequncy_mhz)
{
    /* 使能DWT外设 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* DWT CYCCNT寄存器计数清0 */
    DWT->CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    cpu_frequncy_hz       = cpu_frequncy_mhz * 1000000;
    cpu_frequncy_hz_ms    = cpu_frequncy_hz / 1000;
    cpu_frequncy_hz_us    = cpu_frequncy_hz / 1000000;
    cyccnt_rount_count = 0;

    dwt_cnt_update();
}

float dwt_get_delta_t(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    float dt                  = ((float)(cnt_now - *cnt_last)) / ((float)(cpu_frequncy_hz));
    *cnt_last                 = cnt_now;

    dwt_cnt_update();

    return dt;
}

double dwt_get_delta_t64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    double dt                 = ((double)(cnt_now - *cnt_last)) / ((double)(cpu_frequncy_hz));
    *cnt_last                 = cnt_now;

    dwt_cnt_update();

    return dt;
}

void dwt_systime_update(void)
{
    volatile uint32_t cnt_now = DWT->CYCCNT;
    uint64_t cnt_temp1, cnt_temp2, cnt_temp3;

    dwt_cnt_update();

    cyccnt64   = (uint64_t)cyccnt_rount_count * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    cnt_temp1 = cyccnt64 / cpu_frequncy_hz;
    cnt_temp2 = cyccnt64 - cnt_temp1 * cpu_frequncy_hz;
    systime.s = (uint32_t)cnt_temp1;
    systime.ms = (uint16_t)(cnt_temp2 / cpu_frequncy_hz_ms);
    cnt_temp3 = cnt_temp2 - systime.ms * cpu_frequncy_hz_ms;
    systime.us = (uint16_t)(cnt_temp3 / cpu_frequncy_hz_us);
}

float dwt_get_timeline_s(void)
{
    dwt_systime_update();

    float dwt_timeline_f32 = (float)systime.s + systime.ms * 0.001f + systime.us * 0.000001f;

    return dwt_timeline_f32;
}

float dwt_get_timeline_ms(void)
{
    dwt_systime_update();

    float dwt_timeline_f32 = (float)systime.s * 1000.0f + systime.ms + systime.us * 0.001f;

    return dwt_timeline_f32;
}

uint64_t dwt_get_timeline_us(void)
{
    dwt_systime_update();

    uint64_t dwt_timeline_f32 = systime.s * 1000000 + systime.ms * 1000 + systime.us;

    return dwt_timeline_f32;
}

void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait         = Delay;

    while ((float)(DWT->CYCCNT - tickstart) < wait * (float)cpu_frequncy_hz)
        ;
}

/************************ COPYRIGHT(C) HNUST-DUST **************************/
