#include "dvc_MCU_comm.h"

#include "dvc_motor_dm.hpp"

void Class_MCU_Comm::Init(
     FDCAN_HandleTypeDef* hcan,
     uint8_t __CAN_Rx_ID,
     uint8_t __CAN_Tx_ID)
{
     if (hcan->Instance == FDCAN1)
     {
          CAN_Manage_Object = &CAN1_Manage_Object;
     }
     else if (hcan->Instance == FDCAN2)
     {
          CAN_Manage_Object = &CAN2_Manage_Object;
     }

     CAN_Rx_ID = __CAN_Rx_ID;
     CAN_Tx_ID = __CAN_Tx_ID;

     static const osThreadAttr_t MCU_Comm_TaskAttr = {
          .name = "MCU_Comm_Task",
          .stack_size = 512,
          .priority = (osPriority_t) osPriorityNormal
     };
     // 启动任务，将 this 传入
     osThreadNew(Class_MCU_Comm::TaskEntry, this, &MCU_Comm_TaskAttr);
}

// 任务入口（静态函数）—— osThreadNew 需要这个原型
void Class_MCU_Comm::TaskEntry(void *argument) {
     Class_MCU_Comm *self = static_cast<Class_MCU_Comm *>(argument);  // 还原 this 指针
     self->Task();  // 调用成员函数
}

// 实际任务逻辑
void Class_MCU_Comm::Task()
{
     Struct_MCU_Comm_Data MCU_Comm_Data_Local;
     for (;;)
     {    // 用临界区一次性复制，避免撕裂
          // __disable_irq();
          // MCU_Comm_Data_Local = *const_cast<const Struct_MCU_Comm_Data*>(&(MCU_Comm_Data));
          // __enable_irq();
          // osDelay(pdMS_TO_TICKS(10));
     }
}

void Class_MCU_Comm::CAN_Send_Command()
{
     // static uint8_t CAN_Tx_Frame[8];
     // CAN_Tx_Frame[0] = MCU_Comm_Data.Start_Of_Frame;
     // CAN_Tx_Frame[1] = MCU_Comm_Data.Yaw_Angle;
     // CAN_Tx_Frame[2] = MCU_Comm_Data.Pitch_Angle;
     // CAN_Tx_Frame[3] = MCU_Comm_Data.Chassis_Direction;
     // CAN_Tx_Frame[4] = MCU_Comm_Data.Chassis_Speed;
     // CAN_Tx_Frame[5] = MCU_Comm_Data.Chassis_Rotation;
     // CAN_Tx_Frame[6] = MCU_Comm_Data.Chassis_Spin;
     // CAN_Tx_Frame[7] = MCU_Comm_Data.Booster;
     //
     // CAN_Send_Data(CAN_Manage_Object->CAN_Handler, CAN_Tx_ID, CAN_Tx_Frame, 8);
}


void Class_MCU_Comm::CAN_RxCpltCallback(uint8_t* Rx_Data)
{
     // 判断在线

     // 处理数据 , 解包
     MCU_Comm_Data.Start_Of_Frame       = Rx_Data[0];
     MCU_Comm_Data.Yaw                  = Rx_Data[1];
     MCU_Comm_Data.Pitch_Angle          = Rx_Data[2];
     MCU_Comm_Data.Chassis_Speed_X      = Rx_Data[3];
     MCU_Comm_Data.Chassis_Speed_Y      = Rx_Data[4];
     MCU_Comm_Data.Chassis_Rotation     = Rx_Data[5];
     MCU_Comm_Data.Chassis_Spin         = Rx_Data[6];
     MCU_Comm_Data.Booster              = Rx_Data[7];
}
