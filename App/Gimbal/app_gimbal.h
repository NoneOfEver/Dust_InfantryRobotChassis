#ifndef GIMBAL_H
#define GIMBAL_H

#include "FreeRTOS.h"
// module
#include "dvc_motor_dm.hpp"
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
enum Enum_Gimbal_Control_Type
{
    Gimbal_Control_Type_Manual = 0,
    Gimbal_Control_Type_AutoAim,
};

class Class_Gimbal
{
public:
    // 2个DM6220，作为云台Yaw和Pitch轴控制电机
    Class_Motor_DM_Normal Motor_Yaw;
    Class_Motor_DM_Normal Motor_Pitch;

    Class_PID Yaw_Angle_PID;


    void Init();

    void Task();

    inline float Get_Now_Yaw_Angle();

    inline float Get_Now_Pitch_Angle();

    inline float Get_Now_Yaw_Omega();

    inline float Get_Now_Pitch_Omega();

    inline float Get_Target_Yaw_Angle();

    inline float Get_Target_Pitch_Angle();

    inline float Get_Target_Yaw_Omega();

    inline float Get_Target_Pitch_Omega();

    inline void Set_Target_Yaw_Angle(float __Target_Yaw_Angle);

    inline void Set_Target_Pitch_Angle(float __Target_Pitch_Angle);

    inline void Set_Target_Yaw_Omega(float __Target_Yaw_Omega);

    inline void Set_Target_Pitch_Omega(float __Target_Pitch_Omega);

protected:
    // pitch轴最小值
    float Min_Pitch_Angle = -0.60f;
    // pitch轴最大值
    float Max_Pitch_Angle = 0.33f;

    // 内部变量

    // 读变量

    // yaw轴当前角度
    float Now_Yaw_Angle = 0.0f;
    // pitch轴当前角度
    float Now_Pitch_Angle = 0.0f;

    // yaw轴当前角速度
    float Now_Yaw_Omega = 0.0f;
    // pitch轴当前角速度
    float Now_Pitch_Omega = 0.0f;

    // 写变量

    // 云台状态
    Enum_Gimbal_Control_Type Gimbal_Control_Type = Gimbal_Control_Type_Manual;
    // 读写变量

    // yaw轴目标角度
    float Target_Yaw_Angle = 0.0f;
    // pitch轴目标角度
    float Target_Pitch_Angle = 0.0f;

    // yaw轴目标角速度
    float Target_Yaw_Omega = 0.0f;
    // pitch轴目标角速度
    float Target_Pitch_Omega = 0.0f;

    void Self_Resolution();
    void _Motor_Nearest_Transposition();
    void Output();
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

/**
 * @brief 获取yaw轴当前角度
 *
 * @return float yaw轴当前角度
 */
inline float Class_Gimbal::Get_Now_Yaw_Angle()
{
    return (Now_Yaw_Angle);
}

/**
 * @brief 获取pitch轴当前角度
 *
 * @return float pitch轴当前角度
 */
inline float Class_Gimbal::Get_Now_Pitch_Angle()
{
    return (Now_Pitch_Angle);
}

/**
 * @brief 获取yaw轴当前角速度
 *
 * @return float yaw轴当前角速度
 */
inline float Class_Gimbal::Get_Now_Yaw_Omega()
{
    return (Now_Yaw_Omega);
}

/**
 * @brief 获取pitch轴当前角速度
 *
 * @return float pitch轴当前角速度
 */
inline float Class_Gimbal::Get_Now_Pitch_Omega()
{
    return (Now_Pitch_Omega);
}

/**
 * @brief 获取yaw轴目标角度
 *
 * @return float yaw轴目标角度
 */
inline float Class_Gimbal::Get_Target_Yaw_Angle()
{
    return (Target_Yaw_Angle);
}

/**
 * @brief 获取pitch轴目标角度
 *
 * @return float pitch轴目标角度
 */
inline float Class_Gimbal::Get_Target_Pitch_Angle()
{
    return (Target_Pitch_Angle);
}

/**
 * @brief 获取yaw轴目标角速度
 *
 * @return float yaw轴目标角速度
 */
inline float Class_Gimbal::Get_Target_Yaw_Omega()
{
    return (Target_Yaw_Omega);
}

/**
 * @brief 获取pitch轴目标角速度
 *
 * @return float pitch轴目标角速度
 */
inline float Class_Gimbal::Get_Target_Pitch_Omega()
{
    return (Target_Pitch_Omega);
}

/**
 * @brief 设定yaw轴角度
 *
 * @param __Target_Yaw_Angle yaw轴角度
 */
inline void Class_Gimbal::Set_Target_Yaw_Angle(float __Target_Yaw_Angle)
{
    Target_Yaw_Angle = __Target_Yaw_Angle;
}

/**
 * @brief 设定pitch轴角度
 *
 * @param __Target_Pitch_Angle pitch轴角度
 */
inline void Class_Gimbal::Set_Target_Pitch_Angle(float __Target_Pitch_Angle)
{
    Target_Pitch_Angle = __Target_Pitch_Angle;
}

/**
 * @brief 设定yaw轴角速度
 *
 * @param __Target_Yaw_Omega yaw轴角速度
 */
inline void Class_Gimbal::Set_Target_Yaw_Omega(float __Target_Yaw_Omega)
{
    Target_Yaw_Omega = __Target_Yaw_Omega;
}

/**
 * @brief 设定pitch轴角速度
 *
 * @param __Target_Pitch_Omega pitch轴角速度
 */
inline void Class_Gimbal::Set_Target_Pitch_Omega(float __Target_Pitch_Omega)
{
    Target_Pitch_Omega = __Target_Pitch_Omega;
}


#endif // !GIMBAL_H