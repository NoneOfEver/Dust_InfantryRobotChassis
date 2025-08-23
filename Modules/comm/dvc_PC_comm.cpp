//
// Created by noe on 25-8-3.
//

#include "dvc_PC_comm.h"

uint8_t* USB_RxBuf;
uint8_t usb_tx_cmplt_flag = 0;
static float Kp,Kd,Angle,Omega,Torque;
const char* param_list[] = {"Omega","Angle","Torque" "Kp", "Kd"};
#define PARAM_COUNT (sizeof(param_list) / sizeof(param_list[0]))

/**
 * @bief USB接收完成回调函数
 *
 * @param len 接收到的数据长度
 */
void test_usb_rx_callback(uint16_t len)
{
    USBTransmit(USB_RxBuf,len);
}

/**
 * @bief USB发送完成回调函数
 *
 * @param len 发送的数据长度
 */
void test_usb_tx_callback(uint16_t len)
{

}

// void parse_parameter(uint8_t* input)
// {
//     // 把 uint8_t* 转成 C 字符串
//     char* str = (char*)input;
//
//     // 尝试匹配 "Omega:"
//     if (strncmp(str, "Omega:", 6) == 0)
//     {
//         Omega = atof(str + 6);
//         Motor_Yaw.Set_Control_Omega(Omega);
//         Motor_Pitch.Set_Control_Omega(Omega);
//         return;
//     }
//
//     // 尝试匹配 "Angle:"
//     if (strncmp(str, "Angle:", 6) == 0)
//     {
//         Angle = atof(str + 6);
//         Motor_Yaw.Set_Control_Angle(Angle);
//         Motor_Pitch.Set_Control_Angle(Angle);
//         return;
//     }
//
//     // 尝试匹配 "Torque:"
//     if (strncmp(str, "Torque:", 7) == 0)
//     {
//         Torque = atof(str + 7);
//         Motor_Yaw.Set_Control_Torque(Torque);
//         Motor_Pitch.Set_Control_Torque(Torque);
//         return;
//     }
//
//     // 尝试匹配 "Kp:"
//     if (strncmp(str, "Kp:", 3) == 0)
//     {
//         Kp = atof(str + 3);
//         Motor_Yaw.Set_K_P(Kp);
//         Motor_Pitch.Set_K_P(Kp);
//         return;
//     }
//
//     // 尝试匹配 "Kd:"
//     if (strncmp(str, "Kd:", 3) == 0)
//     {
//         Kd = atof(str + 3);
//         Motor_Yaw.Set_K_D(Kd);
//         Motor_Pitch.Set_K_D(Kd);
//         return;
//     }
// }
void Class_PC_comm::Init()
{
    // USB初始化
    USB_Init_Config_s USB_Init_Config = {
        .tx_cbk = test_usb_tx_callback,
        .rx_cbk = test_usb_rx_callback,
    };
    USB_RxBuf = USBInit(USB_Init_Config);
}
