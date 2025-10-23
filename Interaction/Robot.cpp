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
#include "dvc_mcu_comm.h"
#include "imu.hpp"
#include "stm32h7xx_hal_uart.h"
#include "user_lib.h"
#include "BMI088driver.h"
// bsp
#include "cmsis_os2.h"
#include "bsp_dwt.h"
#include "usart.h"
#include <cstdint>

void Robot::Init()
{
    dwt_init(480);
    // 上下板通讯组件初始化
    mcu_comm_.Init(&hfdcan2, 0x01, 0x00);
    // 陀螺仪初始化
    imu_.Init();
    osDelay(pdMS_TO_TICKS(10000));// 10s时间等待陀螺仪收敛
    // 底盘跟随控制PID初始化
    chassis_follow_pid_.Init(1.5f,0.0f,0.0f,0.0f,0.0f,9.0f,0.001f);
    // yaw轴角度环PID初始化
    yaw_angle_pid_.Init(0.25f,0.00018f,0.00024f,0.0f,0.0f,0.0f,0.001f);//kp_osc 0.035 0.07 0.00015 0.000020
    // 云台初始化
    gimbal_.Init();
    // 底盘初始化
    chassis_.Init();
    // 超级电容初始化
    supercap_.Init(&hfdcan3, 0x100, 0x003);

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
    mcu_comm_data_local.supercap            = SUPERCAP_STATUS_CHARGE;


    // static uint8_t virtual_angle_debug[4];
    // static uint8_t imu_angle_debug[4];
    // static uint8_t tail[4] = {0x00,0x00,0x80,0x7f};
    // static uint8_t tx_buf[12];
    memcpy(&mcu_comm_.mcu_imu_data_.yaw_total_angle_f,mcu_comm_.mcu_imu_data_.yaw_total_angle,sizeof(float));
    virtual_angle_ = mcu_comm_.mcu_imu_data_.yaw_total_angle_f;
    for (;;)
    {
        // 用临界区一次性复制，避免撕裂
        __disable_irq();
        mcu_comm_data_local = *const_cast<const McuCommData*>(&(mcu_comm_.mcu_comm_data_));
        __enable_irq();

        virtual_angle_ += ( 127.0f - mcu_comm_data_local.yaw)*YAW_SENSITIVITY;
        yaw_angle_pid_.SetTarget(virtual_angle_);
        
        ////陀螺仪调试
        memcpy(&mcu_comm_.mcu_imu_data_.yaw_total_angle_f,mcu_comm_.mcu_imu_data_.yaw_total_angle,sizeof(float));
        // HAL_UART_Transmit(&huart7,mcu_comm_.mcu_imu_data_.yaw_total_angle, 4*sizeof(uint8_t), HAL_MAX_DELAY);
        // uint8_t tail[4] = {0x00,0x00,0x80,0x7f};
        // HAL_UART_Transmit(&huart7,tail, 4*sizeof(uint8_t), HAL_MAX_DELAY);

        yaw_angle_pid_.SetNow(mcu_comm_.mcu_imu_data_.yaw_total_angle_f);
        // 云台yaw轴调试
        // memcpy(&tx_buf[0], virtual_angle_debug, 4);
        // memcpy(&tx_buf[4], imu_angle_debug, 4);
        // memcpy(&tx_buf[8], tail, 4);
        // HAL_UART_Transmit(&huart7, tx_buf, sizeof(tx_buf), HAL_MAX_DELAY);
        // yaw_angle_pid_.SetNow(0);
        yaw_angle_pid_.CalculatePeriodElapsedCallback();

        chassis_follow_pid_.SetTarget(0.0f);
        chassis_follow_pid_.SetNow(gimbal_.GetNowYawAngle());
        chassis_follow_pid_.CalculatePeriodElapsedCallback();

        chassis_.SetTargetVelocityX((mcu_comm_data_local.chassis_speed_x - 127.0f) * 10.0f / 128.0f); //9
        chassis_.SetTargetVelocityY((mcu_comm_data_local.chassis_speed_y - 127.0f) * 10.0f / 128.0f); //9
        //chassis_.SetTargetVelocityRotation(((mcu_comm_data_local.chassis_rotation - 127.0f) * 9.0f / 128.0f)-chassis_follow_pid_.GetOut());
        
        // 遥控模式
        //gimbal_.SetTargetYawOmega((mcu_comm_data_local.yaw - 127.0f) * 3.0f / 128.0f);
        gimbal_.SetTargetYawOmega(-(yaw_angle_pid_.GetOut()));
        gimbal_.SetTargetPitchAngle((mcu_comm_data_local.pitch_angle - 127.0f) * (0.3f/128.0f));

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
        //     chassis_follow_pid_.SetTarget(mcu_comm_.mcu_imu_data_.yaw_f);
        //     chassis_follow_pid_.SetNow(chassis_.imu_.GetYawAngle());
        //     chassis_follow_pid_.CalculatePeriodElapsedCallback();
        //     // TODO 这样解算会忽略掉遥控器上的底盘旋转指令，后期改进
        //     chassis_.SetTargetVelocityRotation(chassis_follow_pid_.GetOut());
        // }

        // 超级电容充放电
        if(mcu_comm_data_local.supercap == 0){
            supercap_.SetChargeStatus(SUPERCAP_STATUS_CHARGE);
        }else if(mcu_comm_data_local.supercap == 1){
            supercap_.SetChargeStatus(SUPERCAP_STATUS_DISCHARGE);
        }else{
            supercap_.SetChargeStatus(SUPERCAP_STATUS_CHARGE);
        }
        supercap_.SetPowerLimitMax(100);
        supercap_.SetChargePower(50);
        
        // 底盘小陀螺模式
        switch(mcu_comm_data_local.chassis_spin)
        {
            case CHASSIS_SPIN_CLOCKWISE:
                chassis_.SetTargetVelocityRotation(10.0f);
                //gimbal_.SetTargetYawOmega(-2.0f);
            break;
            case CHASSIS_SPIN_DISABLE:
            // do nothing
            break;
            case CHASSIS_SPIN_COUNTER_CLOCK_WISE:
                chassis_.SetTargetVelocityRotation(-10.0f);
                //gimbal_.SetTargetYawOmega(2.0f);
            break;
            default:
            // do nothing
            break; 
        };
        osDelay(pdMS_TO_TICKS(1));// 1000hz
    }
}
