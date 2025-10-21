/**
 * @file bsp_usb.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef BSP_BSP_USB_H_
#define BSP_BSP_USB_H_
#ifdef __cplusplus
extern "C"{
#endif
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"
struct UsbInitConfig
{
    UsbCallback tx_cbk;
    UsbCallback rx_cbk;
};

/* @note 虚拟串口的波特率/校验位/数据位等动态可变,取决于上位机的设定 */
/* 使用时不需要关心这些设置(作为从机) */

uint8_t *usb_init(struct UsbInitConfig usb_conf); // bsp初始化时调用会重新枚举设备

void usb_transmit(uint8_t *buffer, uint16_t len); // 通过usb发送数据

void usb_refresh(); // 重新枚举USB设备
#ifdef __cplusplus
}
#endif
#endif /* BSP_BSP_USB_H_ */