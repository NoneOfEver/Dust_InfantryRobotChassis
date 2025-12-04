//
// Created by noe on 25-8-3.
//

#include "dvc_pc_comm.h"
#include "bsp_usb.h"

uint8_t* g_usb_rx_buffer;
uint8_t g_usb_tx_cmplt_flag = 0;

/**
 * @bief USB接收完成回调函数
 *
 * @param len 接收到的数据长度
 */
void test_usb_rx_callback(uint16_t len)
{
    usb_transmit(g_usb_rx_buffer,len);
}

/**
 * @bief USB发送完成回调函数
 *
 * @param len 发送的数据长度
 */
void test_usb_tx_callback(uint16_t len)
{

}

void PcComm::Init()
{
    // USB初始化
    UsbInitConfig usb_init_config = {
        .tx_cbk = test_usb_tx_callback,
        .rx_cbk = test_usb_rx_callback,
    };
    g_usb_rx_buffer = usb_init(usb_init_config);
}
