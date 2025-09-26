/**
 * @file bsp_gpio.cpp
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "bsp_gpio.h"
#include "memory.h"
#include "stdlib.h"

static uint8_t idx;
static struct GpioInstance *gpio_instance[GPIO_MX_DEVICE_NUM] = {NULL};

/**
 * @brief EXTI中断回调函数,根据GPIO_Pin找到对应的GpioInstance,并调用模块回调函数(如果有)
 * @note 如何判断具体是哪一个GPIO的引脚连接到这个EXTI中断线上?
 *       一个EXTI中断线只能连接一个GPIO引脚,因此可以通过GPIO_Pin来判断,PinX对应EXTIX
 *       一个Pin号只会对应一个EXTI,详情见gpio.md
 * @param GPIO_Pin 发生中断的GPIO_Pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 如有必要,可以根据pinstate和HAL_GPIO_ReadPin来判断是上升沿还是下降沿/rise&fall等
    struct GpioInstance *gpio;
    for (size_t i = 0; i < idx; i++)
    {
        gpio = gpio_instance[i];
        if (gpio->gpio_pin == GPIO_Pin && gpio->gpio_model_callback != NULL)
        {
            gpio->gpio_model_callback(gpio);
            return;
        }
    }
}

struct GpioInstance *GPIORegister(struct GpioInitConfig *gpio_config)
{
    struct GpioInstance *ins = (struct GpioInstance *)malloc(sizeof(GpioInstance));
    memset(ins, 0, sizeof(GpioInstance));

    ins->gpiox = gpio_config->gpiox;
    ins->gpio_pin = gpio_config->gpio_pin;
    ins->pin_state = gpio_config->pin_state;
    ins->exti_mode = gpio_config->exti_mode;
    ins->id = gpio_config->id;
    ins->gpio_model_callback = gpio_config->gpio_model_callback;
    gpio_instance[idx++] = ins;
    return ins;
}

// ----------------- GPIO API -----------------
// 都是对HAL的形式上的封装,后续考虑增加GPIO state变量,可以直接读取state

void gpio_toggle(struct GpioInstance *_instance)
{
    HAL_GPIO_TogglePin(_instance->gpiox, _instance->gpio_pin);
}

void gpio_set(struct GpioInstance *_instance)
{
    HAL_GPIO_WritePin(_instance->gpiox, _instance->gpio_pin, GPIO_PIN_SET);
}

void gpio_reset(struct GpioInstance *_instance)
{
    HAL_GPIO_WritePin(_instance->gpiox, _instance->gpio_pin, GPIO_PIN_RESET);
}

GPIO_PinState gpio_read(struct GpioInstance *_instance)
{
    return HAL_GPIO_ReadPin(_instance->gpiox, _instance->gpio_pin);
}
/************************ COPYRIGHT(C) HNUST-DUST **************************/
