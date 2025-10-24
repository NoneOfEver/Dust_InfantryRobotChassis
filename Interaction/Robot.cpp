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
#include "slidingmodec.h"
#include "general_def.h"
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
    // 底盘跟随控制PID初始化  17.0f,0.0f,0.0f,5.0f,0.0f,6.0f,0.001f,0.0f,0.0f,0.0f,0.0f
    chassis_follow_pid_.Init(17.0f,0.0f,0.0f,5.0f,0.0f,6.0f,0.001f,0.0f,0.0f,0.0f,0.0f);
    // yaw轴角度环PID初始化 0.045f,0.050800f,0.001500f,0.1f,0.0f,45.0f,0.001f
    //                     0.047f,0.050800f,0.002000f,0.1f,25.0f,45.0f,0.001f
    //                     0.048f,0.050800f,0.002000f,0.1f,25.0f,45.0f,0.001f,0.0f,0.0f,0.0f,180
    yaw_angle_pid_.Init(0.080f,0.050800f,0.002000f,10.0f,25.0f,45.0f,0.001f,0.0f,0.0f,0.0f,15);//kp_osc 0.035 0.07 0.00015 0.000020
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

// 返回值范围: -π ~ π
float get_relative_angle_pm_pi(float now_angle_cum, float zero_angle)
{
    float rel = fmodf(now_angle_cum - zero_angle, 2.0f * M_PI); // [-2π, 2π)
    if (rel > M_PI)
        rel -= 2.0f * M_PI;
    else if (rel < -M_PI)
        rel += 2.0f * M_PI;
    return rel;
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
    
    // // 5, 10, 0, 2.5, 45, 0.8, 1.0
    // Smc YawSMC(15, 8, 0, 2.5, 45, 0.8, 1.0); //这是一个yaw轴参考电机参数

    uint8_t virtual_angle_debug[4];
    uint8_t gimbal_encoder_angle_debug[4];
    float gimbal_angle_f;
    float relative_yaw_angle;
    // static uint8_t imu_angle_debug[4];
    // static uint8_t tail[4] = {0x00,0x00,0x80,0x7f};
    // static uint8_t tx_buf[12];
    memcpy(&mcu_comm_.mcu_imu_data_.yaw_total_angle_f,mcu_comm_.mcu_imu_data_.yaw_total_angle,sizeof(float));
    virtual_angle_ = mcu_comm_.mcu_imu_data_.yaw_total_angle_f;
    float zero_yaw_angle = gimbal_.motor_yaw_.GetNowAngle();
    for (;;)
    {
        // 用临界区一次性复制，避免撕裂
        __disable_irq();
        mcu_comm_data_local = *const_cast<const McuCommData*>(&(mcu_comm_.mcu_comm_data_));
        __enable_irq();

        virtual_angle_ += ( 127.0f - mcu_comm_data_local.yaw)*YAW_SENSITIVITY;
        // virtual_angle_ = (127.0f - mcu_comm_data_local.yaw); //绝对值映射
        yaw_angle_pid_.SetTarget(virtual_angle_);
        
        // 陀螺仪调试
        memcpy(&mcu_comm_.mcu_imu_data_.yaw_total_angle_f,mcu_comm_.mcu_imu_data_.yaw_total_angle,sizeof(float));
        // HAL_UART_Transmit(&huart7,mcu_comm_.mcu_imu_data_.yaw_total_angle, 4*sizeof(uint8_t), HAL_MAX_DELAY);
        // memcpy(virtual_angle_debug,&virtual_angle_,sizeof(float));
        // HAL_UART_Transmit(&huart7,virtual_angle_debug, 4*sizeof(uint8_t), HAL_MAX_DELAY);
        gimbal_angle_f = fabs(gimbal_.motor_yaw_.GetNowAngle());
        memcpy(gimbal_encoder_angle_debug,&gimbal_angle_f,sizeof(float));
        HAL_UART_Transmit(&huart7,gimbal_encoder_angle_debug, 4*sizeof(uint8_t), HAL_MAX_DELAY);
        uint8_t tail[4] = {0x00,0x00,0x80,0x7f};
        HAL_UART_Transmit(&huart7,tail, 4*sizeof(uint8_t), HAL_MAX_DELAY);

        yaw_angle_pid_.SetNow(mcu_comm_.mcu_imu_data_.yaw_total_angle_f);
        yaw_angle_pid_.CalculatePeriodElapsedCallback();

        // YawSMC.ref = virtual_angle_;
        // YawSMC.Smc_Tick(mcu_comm_.mcu_imu_data_.yaw_total_angle_f, RAD_2_DEGREE * gimbal_.GetNowYawOmega());

        relative_yaw_angle = get_relative_angle_pm_pi(gimbal_.GetNowYawAngle(), zero_yaw_angle);
        chassis_follow_pid_.SetTarget(PI/2);
        chassis_follow_pid_.SetNow(relative_yaw_angle);
        chassis_follow_pid_.CalculatePeriodElapsedCallback();

        chassis_.SetTargetVelocityX((mcu_comm_data_local.chassis_speed_x - 127.0f) * 10.0f / 128.0f); //9
        chassis_.SetTargetVelocityY((mcu_comm_data_local.chassis_speed_y - 127.0f) * 10.0f / 128.0f); //9
        // chassis_.SetTargetVelocityRotation(((mcu_comm_data_local.chassis_rotation - 127.0f) * 9.0f / 128.0f)-chassis_follow_pid_.GetOut());
        // gimbal_.SetYawOmegaFeedforword(-0.9f*chassis_follow_pid_.GetOut());
        // chassis_.SetTargetVelocityRotation(((mcu_comm_data_local.chassis_rotation - 127.0f) * 9.0f / 128.0f));

        // 遥控模式
        // gimbal_.SetTargetYawOmega((mcu_comm_data_local.yaw - 127.0f) * 3.0f / 128.0f);
        // gimbal_.SetTargetYawOmega(-(yaw_angle_pid_.GetOut() + gimbal_.GetYawOmegaFeedforword()));
        // gimbal_.SetTargetYawOmega((YawSMC.u));
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
                gimbal_.SetYawOmegaFeedforword(+5.0f);
            break;
            case CHASSIS_SPIN_DISABLE:
            // do nothing
            break;
            case CHASSIS_SPIN_COUNTER_CLOCK_WISE:
                chassis_.SetTargetVelocityRotation(-10.0f);
                gimbal_.SetYawOmegaFeedforword(-5.0f);
            break;
            default:
            // do nothing
            break; 
        };
        osDelay(pdMS_TO_TICKS(1));// 1khz
    }
}
