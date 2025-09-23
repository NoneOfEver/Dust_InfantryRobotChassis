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
#include "alg_math.h"
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
    // 底盘跟随控制PID初始化
    chassis_follow_pid_.Init(1.0f, 0.0f, 0.0f);
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
    McuCommData mcu_comm_data_local;
    mcu_comm_data_local.yaw                 = 127;
    mcu_comm_data_local.pitch_angle         = 127;
    mcu_comm_data_local.chassis_speed_x     = 127;
    mcu_comm_data_local.chassis_speed_y     = 127;
    mcu_comm_data_local.chassis_rotation    = 127;
    mcu_comm_data_local.chassis_spin        = CHASSIS_SPIN_DISABLE;
    mcu_comm_data_local.booster             = 0;

    for (;;)
    {
        // 用临界区一次性复制，避免撕裂
        __disable_irq();
        mcu_comm_data_local = *const_cast<const McuCommData*>(&(mcu_comm_.mcu_comm_data_));
        __enable_irq();

        chassis_.SetTargetVelocityX((mcu_comm_data_local.chassis_speed_x - 127.0f) * 10.0f / 128.0f); //9
        chassis_.SetTargetVelocityY((mcu_comm_data_local.chassis_speed_y - 127.0f) * 10.0f / 128.0f); //9
        chassis_.SetTargetVelocityRotation((mcu_comm_data_local.chassis_rotation - 127.0f) * 9.0f / 128.0f);

        // 遥控模式
        gimbal_.SetTargetYawOmega((mcu_comm_data_local.yaw - 127.0f) * 3.0f / 128.0f);
        gimbal_.SetTargetPitchAngle((mcu_comm_data_local.pitch_angle - 127.0f) * 0.009375f);

        // // 自瞄模式
        // memcpy(&mcu_comm_.mcu_autoaim_data_.pitch_f,mcu_comm_.mcu_autoaim_data_.pitch,sizeof(float));
        // gimbal_.SetTargetPitchAngle(gimbal_.GetNowPitchAngle() + mcu_comm_.mcu_autoaim_data_.pitch_f);
        // memcpy(&mcu_comm_.mcu_auto_aim_data_.Yaw_f,mcu_comm_.mcu_auto_aim_data_.Yaw,sizeof(float));
        // Gimbal.Yaw_Angle_PID.Set_Target(Gimbal.Get_Now_Yaw_Angle() + mcu_comm_.mcu_auto_aim_data_.Yaw_f);

        // // 回传云台电机角度数据
        // mcu_comm_.mcu_send_data_.armor = 0x00;
        // mcu_comm_.mcu_send_data_.yaw = gimbal_.GetNowYawAngle();
        // mcu_comm_.mcu_send_data_.pitch = gimbal_.GetNowPitchAngle();
        // mcu_comm_.CanSendCommand();

        // // 底盘跟随模式
        // if(chassis_follow_mode_status_ == true && chassis_gyroscope_mode_status_ == ROBOT_GYROSCOPE_TYPE_DISABLE){
        //     chassis_follow_pid_.SetTarget(0.0f);
        //     float temp_now_yaw_angle = gimbal_.GetNowYawAngle();
        //     chassis_follow_pid_.SetNow(0.0f - math_modulus_normalization(gimbal_.GetNowYawAngle(), 2.0f * PI));
        //     chassis_follow_pid_.CalculatePeriodElapsedCallback();
        //     // TODO 这样解算会忽略掉遥控器上的底盘旋转指令，后期改进
        //     chassis_.SetTargetVelocityRotation(chassis_follow_pid_.GetOut());
        // }

        // 底盘小陀螺模式
        switch(mcu_comm_data_local.chassis_spin)
        {
            case CHASSIS_SPIN_CLOCKWISE:
            chassis_.SetTargetVelocityRotation(10.0f);
            gimbal_.SetTargetYawOmega(-2.0f);
            break;
            case CHASSIS_SPIN_DISABLE:
            // do nothing
            break;
            case CHASSIS_SPIN_COUNTER_CLOCK_WISE:
            chassis_.SetTargetVelocityRotation(-10.0f);
            gimbal_.SetTargetYawOmega(2.0f);
            break;
            default:
            // do nothing
            break; 
        };

        osDelay(pdMS_TO_TICKS(10));
    }
}
