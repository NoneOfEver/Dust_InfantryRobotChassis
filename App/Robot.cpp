#include "Robot.h"

// app
#include "app_chassis.h"
// module
#include "dvc_MCU_comm.h"
// bsp
#include "bsp_usb.h"
#include "cmsis_os2.h"
#include "bsp_can.h"

void Class_Robot::Init()
{
    // 上下板通讯组件初始化
    MCU_Comm.Init(&hfdcan2, 0x01, 0x00);
    // 云台初始化
    Gimbal.Init();
    // 底盘初始化
    Chassis.Init();

    HAL_Delay(3000);
    static const osThreadAttr_t RobotTaskAttr = {
        .name = "RobotTask",
        .stack_size = 768,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Class_Robot::TaskEntry, this, &RobotTaskAttr);
}

// 任务入口（静态函数）—— osThreadNew 需要这个原型
void Class_Robot::TaskEntry(void *argument)
{
    Class_Robot *self = static_cast<Class_Robot *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

void Class_Robot::Task()
{
    Struct_MCU_Comm_Data MCU_Comm_Data_Local;
    MCU_Comm_Data_Local.Yaw                 = 127;
    MCU_Comm_Data_Local.Pitch_Angle         = 127;
    MCU_Comm_Data_Local.Chassis_Speed_X     = 127;
    MCU_Comm_Data_Local.Chassis_Speed_Y     = 127;
    MCU_Comm_Data_Local.Chassis_Rotation    = 127;
    MCU_Comm_Data_Local.Chassis_Spin        = 0;
    MCU_Comm_Data_Local.Booster             = 0;
    for (;;)
    {
        // 用临界区一次性复制，避免撕裂
        __disable_irq();
        MCU_Comm_Data_Local = *const_cast<const Struct_MCU_Comm_Data*>(&(MCU_Comm.MCU_Comm_Data));
        __enable_irq();

        Chassis.Set_Target_Velocity_X((MCU_Comm_Data_Local.Chassis_Speed_X - 127.0f) * 9.0f / 128.0f);
        Chassis.Set_Target_Velocity_Y((MCU_Comm_Data_Local.Chassis_Speed_Y - 127.0f) * 9.0f / 128.0f);
        Chassis.Set_Target_Velocity_Rotation((MCU_Comm_Data_Local.Chassis_Rotation - 127.0f) * 6.0f / 128.0f);

        // Gimbal.Motor_Yaw.Set_Control_Omega((MCU_Comm_Data_Local.Yaw - 127.0f) * 3.0f / 128.0f);
        // Gimbal.Motor_Pitch.Set_Control_Angle((MCU_Comm_Data_Local.Pitch_Angle - 127.0f) * 0.009375f);
        Gimbal.Set_Target_Yaw_Omega((MCU_Comm_Data_Local.Yaw - 127.0f) * 3.0f / 128.0f);
        Gimbal.Set_Target_Pitch_Angle((MCU_Comm_Data_Local.Pitch_Angle - 127.0f) * 0.009375f);

        MCU_Comm.MCU_Send_Data.Armor = 0x00;
        MCU_Comm.MCU_Send_Data.Yaw = Gimbal.Get_Now_Yaw_Angle();
        MCU_Comm.MCU_Send_Data.Pitch = Gimbal.Get_Now_Pitch_Angle();
        MCU_Comm.CAN_Send_Command();
        osDelay(pdMS_TO_TICKS(10));
    }
}




