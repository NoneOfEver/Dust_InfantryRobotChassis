/**
 * @file app_gimbal.cpp
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "app_gimbal.h"
#include "cmsis_os2.h"
#include "interpolation.hpp"
void Gimbal::Init()
{
    // 6220电机初始化
    motor_yaw_.Init(&hfdcan3, 0x12, 0x01);
    motor_pitch_.Init(&hfdcan3, 0x11, 0x02);

    motor_yaw_.CanSendClearError();
    osDelay(1000);
    motor_yaw_.CanSendEnter();
    motor_pitch_.CanSendEnter();
    osDelay(1000);

    // motor_yaw_.SetKp(0);
    motor_yaw_.SetKp(0); //MIT模式kp
    // motor_pitch_.SetKp(3); // 3
    motor_pitch_.SetKp(26);//25

    motor_yaw_.SetKd(0.05); // MIT模式kd
    // motor_pitch_.SetKd(0.03);
    motor_pitch_.SetKd(0.028);//0.06

    // motor_yaw_.SetControlAngle(0);
    // motor_pitch_.SetControlAngle(-0.3);

    motor_yaw_.SetControlOmega(0);
    motor_pitch_.SetControlOmega(45);

    // motor_yaw_.SetControlTorque(0.2);
    motor_pitch_.SetControlTorque(0.1);

    motor_yaw_.Output();
    // osDelay(pdMS_TO_TICKS(10));
    motor_pitch_.Output();

    static const osThreadAttr_t kGimbalTaskAttr = {
        .name = "gimbal_task",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Gimbal::TaskEntry, this, &kGimbalTaskAttr);
}


/**
 * @brief 自身解算
 *
 */
void Gimbal::SelfResolution()
{
    now_pitch_angle_ = motor_pitch_.GetNowAngle();
    now_yaw_angle_   = motor_yaw_.GetNowAngle();
    // yaw_angle_pid_.SetNow(now_yaw_angle_);
    // yaw_angle_pid_.CalculatePeriodElapsedCallback();
    // target_yaw_omega_ = yaw_angle_pid_.GetOut();

    // // pitch轴角度归化到±PI / 2之间
    // now_pitch_angle_ = Math_Modulus_Normalization(-motor_pitch_.GetNowAngle(), 2.0f * PI);

}

/**
 * @brief 输出到电机
 *
 */
void Gimbal::Output()
{

    // // 云台位控
    // if (gimbal_control_type_ == GIMBAL_CONTROL_TYPE_MANUAL)         // 无自瞄介入
    // {
    //     // do nothing
    // }else if (gimbal_control_type_ == GIMBAL_CONTROL_TYPE_AUTOAIM){ // 有自瞄矫正
    //     MotorNearestTransposition();
    //     yaw_angle_pid_.SetTarget(now_yaw_angle_); // 加视觉的相对偏移量
    //     yaw_angle_pid_.CalculatePeriodElapsedCallback();
    //     target_yaw_omega_ = yaw_angle_pid_.GetOut();
    // }

    motor_yaw_.SetControlOmega(target_yaw_omega_);
    // motor_pitch_.SetControlAngle(target_pitch_angle_);
    motor_pitch_.SetControlAngle(pitch_angle_interpolation.Output());

    motor_yaw_.Output();
    motor_pitch_.Output();

}

/**
 * @brief 电机就近转位
 *
 */
void Gimbal::MotorNearestTransposition()
{
    // Yaw就近转位
    float tmp_delta_angle;
    tmp_delta_angle = fmod(target_yaw_angle_ - now_yaw_angle_, 2.0f * PI);
    if (tmp_delta_angle > PI)
    {
        tmp_delta_angle -= 2.0f * PI;
    }
    else if (tmp_delta_angle < -PI)
    {
        tmp_delta_angle += 2.0f * PI;
    }
    target_yaw_angle_ = motor_yaw_.GetNowAngle() + tmp_delta_angle;

    // // Pitch就近转位
    // Math_Constrain(&target_pitch_angle_, Min_Pitch_Angle, Max_Pitch_Angle);
    // tmp_delta_angle = target_pitch_angle_ - now_pitch_angle_;
    // target_pitch_angle_ = -motor_pitch_.GetNowAngle() + tmp_delta_angle;
}
// 任务入口（静态函数）—— osThreadNew 需要这个原型
void Gimbal::TaskEntry(void *argument)
{
    Gimbal *self = static_cast<Gimbal *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

// 实际任务逻辑（无限循环）
void Gimbal::Task()
{
    uint8_t first_run_flag = 0; // 用于标记是否是第一次运行

    for (;;)
    {
        if(first_run_flag == 0){ // 第一次运行到这里，pre_pitch_angle未初始化
            first_run_flag = 1;
            pre_pitch_angle_ = target_pitch_angle_;
        }
        SelfResolution();
        Output();
        osDelay(pdMS_TO_TICKS(1)); // 1khz电机控制频率
        pre_pitch_angle_ = target_pitch_angle_;
    }
}