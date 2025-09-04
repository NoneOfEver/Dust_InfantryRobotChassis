/**
 * @file Robot.cpp
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "Robot.h"

// app
#include "app_chassis.h"
// module
#include "dvc_MCU_comm.h"
// bsp
#include "bsp_usb.h"
#include "cmsis_os2.h"
#include "bsp_can.h"

void Robot::Init()
{
    // 上下板通讯组件初始化
    mcu_comm_.Init(&hfdcan2, 0x01, 0x00);
    // 云台初始化
    gimbal_.Init();
    // 底盘初始化
    chassis_.Init();

    HAL_Delay(3000);
    static const osThreadAttr_t kRobotTaskAttr = {
        .name = "robot_task",
        .stack_size = 768,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Robot::TaskEntry, this, &kRobotTaskAttr);
}

// 任务入口（静态函数）—— osThreadNew 需要这个原型
void Robot::TaskEntry(void *argument)
{
    Robot *self = static_cast<Robot *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

void Robot::Task()
{
    McuCommData mcu_comm_data_Local;
    mcu_comm_data_Local.yaw                 = 127;
    mcu_comm_data_Local.pitch_angle         = 127;
    mcu_comm_data_Local.chassis_speed_x     = 127;
    mcu_comm_data_Local.chassis_speed_y     = 127;
    mcu_comm_data_Local.chassis_rotation    = 127;
    mcu_comm_data_Local.chassis_spin        = 0;
    mcu_comm_data_Local.booster             = 0;


    for (;;)
    {
        // 用临界区一次性复制，避免撕裂
        __disable_irq();
        mcu_comm_data_Local = *const_cast<const McuCommData*>(&(mcu_comm_.mcu_comm_data_));
        __enable_irq();

        chassis_.SetTargetVelocityX((mcu_comm_data_Local.chassis_speed_x - 127.0f) * 9.0f / 128.0f);
        chassis_.SetTargetVelocityY((mcu_comm_data_Local.chassis_speed_y - 127.0f) * 9.0f / 128.0f);
        chassis_.SetTargetVelocityRotation((mcu_comm_data_Local.chassis_rotation - 127.0f) * 6.0f / 128.0f);

        // // 遥控模式
        // Gimbal.Set_Target_Yaw_Omega((mcu_comm_data__Local.Yaw - 127.0f) * 3.0f / 128.0f);
        // Gimbal.Set_Target_Pitch_Angle((mcu_comm_data__Local.Pitch_Angle - 127.0f) * 0.009375f);

        // 自瞄模式
        memcpy(&mcu_comm_.mcu_autoaim_data_.pitch_f,mcu_comm_.mcu_autoaim_data_.pitch,sizeof(float));
        gimbal_.SetTargetPitchAngle(gimbal_.GetNowPitchAngle() + mcu_comm_.mcu_autoaim_data_.pitch_f);
        // memcpy(&mcu_comm_.mcu_auto_aim_data_.Yaw_f,mcu_comm_.mcu_auto_aim_data_.Yaw,sizeof(float));
        // Gimbal.Yaw_Angle_PID.Set_Target(Gimbal.Get_Now_Yaw_Angle() + mcu_comm_.mcu_auto_aim_data_.Yaw_f);

        mcu_comm_.mcu_send_data_.armor = 0x00;
        mcu_comm_.mcu_send_data_.yaw = gimbal_.GetNowYawAngle();
        mcu_comm_.mcu_send_data_.pitch = gimbal_.GetNowPitchAngle();
        mcu_comm_.CanSendCommand();
        osDelay(pdMS_TO_TICKS(10));
    }
}




