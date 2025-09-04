/**
 * @file bsp_gpio.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef BSP_BSP_GPIO_H_
#define BSP_BSP_GPIO_H_
#include "gpio.h"
#include "stdint.h"

#define GPIO_MX_DEVICE_NUM 10

/**
 * @brief 用于判断中断来源,注意和CUBEMX中配置一致
 *
 */
enum GpioExtiMode
{
    GPIO_EXTI_MODE_RISING,
    GPIO_EXTI_MODE_FALLING,
    GPIO_EXTI_MODE_RISING_FALLING,
    GPIO_EXTI_MODE_NONE,
};

/**
 * @brief GPIO实例结构体定义
 *
 */
struct GpioInstance
{
    GPIO_TypeDef *gpiox;
    GPIO_PinState pin_state;
    enum GpioExtiMode exti_mode;
    uint16_t gpio_pin;
    void (*gpio_model_callback)(struct GpioInstance *); // 回调函数
    void *id;
};
/**
 * @brief GPIO初始化配置结构体定义
 *
 */
struct GpioInitConfig
{
    GPIO_TypeDef *gpiox;        // GPIOA,GPIOB,GPIOC...
    GPIO_PinState pin_state;    // 引脚状态,Set,Reset not frequently used
    enum GpioExtiMode exti_mode; // 外部中断模式 not frequently used
    uint16_t gpio_pin;          // 引脚号,@note 这里的引脚号是gpio_pin_0,gpio_pin_1...
    // 这些引脚是stm32f4xx_hal_gpio.h中定义的宏!!! 一定要注意

    void (*gpio_model_callback)(struct GpioInstance *); // exti中断回调函数
    void *id;                                    // 区分不同的GPIO实例

};

/**
 * @brief 注册GPIO实例
 *
 * @param gpio_config
 * @return GpioInstance*
 */
struct GpioInstance *gpio_register(struct GpioInitConfig *gpio_config);

/**
 * @brief GPIO API,切换GPIO电平
 *
 * @param _instance
 */
void gpio_toggle(struct GpioInstance *_instance);

/**
 * @brief 设置GPIO电平
 *
 * @param _instance
 */
void gpio_set(struct GpioInstance *_instance);

/**
 * @brief 复位GPIO电平
 *
 * @param _instance
 */
void gpio_reset(struct GpioInstance *_instance);

/**
 * @brief 读取GPIO电平
 *
 * @param _instance
 * @return GPIO_PinState
 */
GPIO_PinState GPIORead(struct GpioInstance *_instance);
#endif //BSP_BSP_GPIO_H_
/************************ COPYRIGHT(C) HNUST-DUST **************************/