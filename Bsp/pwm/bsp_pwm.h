/**
 * @file Bsp_PWM.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-01-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "tim.h"
#include "stdint.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h723xx.h"
#define PWM_DEVICE_CNT 16 // 最大支持的PWM实例数量

/* pwm实例结构体 */
typedef struct pwm_ins_temp
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    uint32_t channel;                        // 通道
    uint32_t tclk;                           // 时钟频率
    float period;                         // 周期
    float dutyratio;                      // 占空比
    void (*callback)(struct pwm_ins_temp *); // DMA传输完成回调函数
    void *id;                                // 实例ID
} PWMInstance;

typedef struct 
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    uint32_t channel;                        // 通道
    float period;                         // 周期
    float dutyratio;                      // 占空比
    void (*callback)(PWMInstance*); // DMA传输完成回调函数
    void *id;                                // 实例ID
} PWM_Init_Config_s;

/**
 * @brief 注册一个pwm实例
 *
 * @param config 初始化配置
 * @return PWMInstance*
 */
PWMInstance *PWMRegister(PWM_Init_Config_s *config);

/**
 * @brief 启动pwm
 *
 * @param pwm pwm实例
 */
void PWMStart(PWMInstance *pwm);
/**
 * @brief 设置pwm占空比
 *
 * @param pwm pwm实例
 * @param dutyratio 占空比 0~1
 */

void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio);
/**
 * @brief 停止pwm
 *
 * @param pwm pwm实例
 */
void PWMStop(PWMInstance *pwm);

/**
 * @brief 设置pwm周期
 *
 * @param pwm pwm实例
 * @param period 周期 单位 s
 */
void PWMSetPeriod(PWMInstance *pwm, float period);

/**
 * @brief 启动pwm dma传输
 *
 * @param pwm pwm实例
 * @param pData 数据首地址指针,注意数据的位数必须和CubeMX配置的DMA传输位数(字长)一致
 * @param Size 数据长度
 * @note 如果使用此函数,则需要在CubeMX中配置DMA传输位数为对应位数
 *       例如:使用16位数据,则需要配置DMA传输位数为16位(half word),配置错误会导致指针越界或数据错误
 */
void PWMStartDMA(PWMInstance *pwm, uint32_t *pData, uint32_t Size);



// /**
//   * @brief  Set the TIM Capture Compare Register value on runtime without calling another time ConfigChannel function.
//   * @param  __HANDLE__ TIM handle.
//   * @param  __CHANNEL__ TIM Channels to be configured.
//   *          This parameter can be one of the following values:
//   *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
//   *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
//   *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
//   *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
//   * @param  __COMPARE__ specifies the Capture Compare register new value.
//   * @retval None
//   */
// #define __HAL_TIM_SET_COMPARE(__HANDLE__, __CHANNEL__, __COMPARE__) \
//   (((__CHANNEL__) == TIM_CHANNEL_1) ? ((__HANDLE__)->Instance->CCR1 = (__COMPARE__)) :\
//    ((__CHANNEL__) == TIM_CHANNEL_2) ? ((__HANDLE__)->Instance->CCR2 = (__COMPARE__)) :\
//    ((__CHANNEL__) == TIM_CHANNEL_3) ? ((__HANDLE__)->Instance->CCR3 = (__COMPARE__)) :\
//    ((__HANDLE__)->Instance->CCR4 = (__COMPARE__)))
//
// #define GTIM_TIMX_INT                       TIM3
// #define GTIM_TIMX_INT_IRQn                  TIM3_IRQn
// #define GTIM_TIMX_INT_CLK_ENABLE()          do{ __HAL_RCC_TIM3_CLK_ENABLE(); }while(0)   /* TIM3时钟使能 */



#endif // BSP_PWM_H