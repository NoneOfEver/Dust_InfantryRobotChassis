/**
 * @file app_gimbal.h
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef APP_GIMBAL_H
#define APP_GIMBAL_H

#include "FreeRTOS.h"
// module
#include "dvc_motor_dm.h"
#include "interpolation.hpp"
// bsp
#include "bsp_log.h"
#include "bsp_usb.h"
#include "cmsis_os2.h"
#include "bsp_usart.h"
#include "bsp_can.h"

/**
 * @brief 云台控制类型
 *
 */
enum GimbalControlType
{
    GIMBAL_CONTROL_TYPE_MANUAL = 0,
    GIMBAL_CONTROL_TYPE_AUTOAIM,
};

class Gimbal
{
public:
    // 2个DM6220，作为云台Yaw和Pitch轴控制电机
    MotorDmNormal motor_yaw_;
    MotorDmNormal motor_pitch_;

    Interpolation pitch_angle_interpolation; // pitch角插补类实例

    void Init();

    void Task();

    inline float GetNowYawAngle();

    inline float GetNowPitchAngle();

    inline float GetNowYawOmega();

    inline float GetNowPitchOmega();

    inline float GetTargetYawAngle();

    inline float GetTargetPitchAngle();

    inline float GetTargetYawOmega();

    inline float GetTargetPitchOmega();

    inline float GetYawOmegaFeedforword()
    {
        return yaw_omega_feedforword_;
    }
    inline float GetPitchOmegaFeedforword()
    {
        return pitch_omega_feedforword_;
    }
    float GetYawRelativeZeroAngle()
    {
        return yaw_relative_zero_angle_;
    }

    float GetYawNowAngleNoncumulative()
    {
        return yaw_now_angle_noncumulative_;
    }
    float GetPitchNowAngleNoncumulative()
    {
        return pitch_now_angle_noncumulative_;
    }
    inline void SetTargetYawAngle(float target_yaw_angle);

    inline void SetTargetPitchAngle(float target_pitch_angle);

    inline void SetTargetYawOmega(float target_yaw_omega);

    inline void SetTargetPitchOmega(float target_pitch_omega);

    inline void SetYawOmegaFeedforword(float yaw_omega_feedforword)
    {
        yaw_omega_feedforword_ = yaw_omega_feedforword;
    }
    inline void SetPitchOmegaFeedforword(float pitch_omega_feedforword)
    {
        pitch_omega_feedforword_ = pitch_omega_feedforword;
    }

protected:
    // pitch轴最小值
    float min_pitch_angle_ = -0.30f;
    // pitch轴最大值
    float max_pitch_angle_ = 0.50f;

    float pre_pitch_angle_ = 0.0f; // 用于pitch角的插补算法
    // 内部变量

    // 读变量

    // yaw轴当前角度
    float now_yaw_angle_ = 0.0f;
    // pitch轴当前角度
    float now_pitch_angle_ = 0.0f;

    // yaw轴当前角速度
    float now_yaw_omega_ = 0.0f;
    // pitch轴当前角速度
    float now_pitch_omega_ = 0.0f;

    // 写变量

    // 云台状态
    GimbalControlType gimbal_control_type_ = GIMBAL_CONTROL_TYPE_MANUAL;
    // 读写变量

    // yaw轴目标角度
    float target_yaw_angle_ = 0.0f;
    // pitch轴目标角度
    float target_pitch_angle_ = 0.0f;

    // yaw轴目标角速度
    float target_yaw_omega_ = 0.0f;
    // pitch轴目标角速度
    float target_pitch_omega_ = 0.0f;

    // yaw轴角速度前馈
    float yaw_omega_feedforword_ = 0.0f;
    // pitch轴角速度前馈
    float pitch_omega_feedforword_ = 0.0f;

    // yaw轴相对于上电时的角度
    float yaw_relative_zero_angle_ = 0.0f;

    // yaw轴上电时的绝对角度
    float yaw_zero_angle_ = 0.0f;

    // yaw轴电机非累计角度 rad
    float yaw_now_angle_noncumulative_ = 0.0f;
     // pitch轴电机非累计角度 rad
    float pitch_now_angle_noncumulative_ = 0.0f;
    void SelfResolution();
    void MotorNearestTransposition();
    void Output();
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

/**
 * @brief 获取yaw轴当前角度
 *
 * @return float yaw轴当前角度
 */
inline float Gimbal::GetNowYawAngle()
{
    return (now_yaw_angle_);
}

/**
 * @brief 获取pitch轴当前角度
 *
 * @return float pitch轴当前角度
 */
inline float Gimbal::GetNowPitchAngle()
{
    return (now_pitch_angle_);
}

/**
 * @brief 获取yaw轴当前角速度
 *
 * @return float yaw轴当前角速度
 */
inline float Gimbal::GetNowYawOmega()
{
    return (now_yaw_omega_);
}

/**
 * @brief 获取pitch轴当前角速度
 *
 * @return float pitch轴当前角速度
 */
inline float Gimbal::GetNowPitchOmega()
{
    return (now_pitch_omega_);
}

/**
 * @brief 获取yaw轴目标角度
 *
 * @return float yaw轴目标角度
 */
inline float Gimbal::GetTargetYawAngle()
{
    return (target_yaw_angle_);
}

/**
 * @brief 获取pitch轴目标角度
 *
 * @return float pitch轴目标角度
 */
inline float Gimbal::GetTargetPitchAngle()
{
    return (target_pitch_angle_);
}

/**
 * @brief 获取yaw轴目标角速度
 *
 * @return float yaw轴目标角速度
 */
inline float Gimbal::GetTargetYawOmega()
{
    return (target_yaw_omega_);
}

/**
 * @brief 获取pitch轴目标角速度
 *
 * @return float pitch轴目标角速度
 */
inline float Gimbal::GetTargetPitchOmega()
{
    return (target_pitch_omega_);
}

/**
 * @brief 设定yaw轴角度
 *
 * @param target_yaw_angle yaw轴角度
 */
inline void Gimbal::SetTargetYawAngle(float target_yaw_angle)
{
    target_yaw_angle_ = target_yaw_angle;
}

/**
 * @brief 设定pitch轴角度
 *
 * @param target_pitch_angle pitch轴角度
 */
inline void Gimbal::SetTargetPitchAngle(float target_pitch_angle)
{
    target_pitch_angle_ = target_pitch_angle;
    pitch_angle_interpolation.Start(pre_pitch_angle_, target_pitch_angle_, 8);
}

/**
 * @brief 设定yaw轴角速度
 *
 * @param target_yaw_omega yaw轴角速度
 */
inline void Gimbal::SetTargetYawOmega(float target_yaw_omega)
{
    target_yaw_omega_ = target_yaw_omega;
}

/**
 * @brief 设定pitch轴角速度
 *
 * @param target_pitch_omega pitch轴角速度
 */
inline void Gimbal::SetTargetPitchOmega(float target_pitch_omega)
{
    target_pitch_omega_ = target_pitch_omega;
}


#endif // !GIMBAL_H