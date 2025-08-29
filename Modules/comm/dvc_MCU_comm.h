#ifndef COMM_H
#define COMM_H
#include "bsp_can.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"



struct Struct_MCU_Comm_Data
{
    uint8_t Start_Of_Frame;     // 帧头
    uint8_t Yaw;                // yaw
    uint8_t Pitch_Angle;        // 俯仰角度
    uint8_t Chassis_Speed_X;    // 平移方向：前、后、左、右
    uint8_t Chassis_Speed_Y;    // 底盘移动总速度
    uint8_t Chassis_Rotation;   // 自转：不转、顺时针转、逆时针转
    uint8_t Chassis_Spin;       // 小陀螺：不转、顺时针转、逆时针转
    uint8_t Booster;            // 发射机构：准备、发射
};

struct Struct_MCU_Send_Data
{
    uint8_t Start_Of_Frame = 0xAB;
    uint8_t Armor;
    float Yaw;   // 4字节浮点数
    float Pitch; // 4字节浮点数
};

struct Struct_MCU_AutoAim_Data
{
    uint8_t Start_Of_Yaw_Frame;
    uint8_t Start_Of_Pitch_Frame;
    uint8_t Yaw[4];
    float Yaw_f;
    uint8_t Pitch[4];
    float Pitch_f;
};

class Class_MCU_Comm
{
public:

    volatile Struct_MCU_Comm_Data MCU_Comm_Data;
    Struct_MCU_Send_Data MCU_Send_Data;

    Struct_MCU_AutoAim_Data MCU_AutoAim_Data = {
        0xAC,
        0xAD,
        {0},
        {0},
    };
    void Init(FDCAN_HandleTypeDef *hcan,
              uint8_t __CAN_Rx_ID,
              uint8_t __CAN_Tx_ID
              );

    void CAN_RxCpltCallback(uint8_t *Rx_Data);

    void CAN_Send_Command();

    void Task();


protected:
    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;
    // 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致
    uint16_t CAN_Rx_ID;
    // 发数据绑定的CAN ID, 是上位机驱动参数CAN_ID加上控制模式的偏移量
    uint16_t CAN_Tx_ID;
    // 发送缓冲区
    uint8_t Tx_Data[8];
    // 内部函数
    void Data_Process();
    // FreeRTOS 入口，静态函数
    static void TaskEntry(void *param);
};

#endif //COMM_H
