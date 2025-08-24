//
// Created by noe on 25-8-3.
//

#include "Init.h"
#include "Robot.h"

Class_Robot Robot;

/**
 * @brief CAN1回调函数
 *
 * @param CAN_RxMessage CAN1收到的消息
 */
void Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
        case (0x201):
        {
            Robot.Chassis.Motor_Chassis_1.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        case (0x202):
        {
            Robot.Chassis.Motor_Chassis_2.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        case (0x203):
        {
            Robot.Chassis.Motor_Chassis_3.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        case (0x204):
        {
            Robot.Chassis.Motor_Chassis_4.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        default:
        break;
    }
}
/**
 * @brief CAN2回调函数
 *
 * @param CAN_RxMessage CAN2收到的消息
 */
void Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
        case (0x01):
        {
            Robot.MCU_Comm.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        default:
            break;
    }
}
/**
 * @brief CAN3回调函数
 *
 * @param CAN_RxMessage CAN3收到的消息
 */
void Device_CAN3_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.Identifier)
    {
        case (0x12)://01
        {
            Robot.Gimbal.Motor_Yaw.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        case (0x11)://02
        {
            Robot.Gimbal.Motor_Pitch.CAN_RxCpltCallback(CAN_RxMessage->Data);
            break;
        }
        default:
            break;
    }
}

void Init()
{
    // CAN1 初始化，控制底盘
    CAN_Init(&hfdcan1,Device_CAN1_Callback);
    // CAN2 初始化，与上板通讯
    CAN_Init(&hfdcan2,Device_CAN2_Callback);
    // CAN3 初始化，控制云台
    CAN_Init(&hfdcan3,Device_CAN3_Callback);

    HAL_Delay(1000);
    Robot.Init();
}