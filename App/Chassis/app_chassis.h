#ifndef CHASSIS_H
#define CHASSIS_H

#include "FreeRTOS.h"
// module
#include "dvc_motor_dji.h"
// bsp
#include "bsp_log.h"
#include "bsp_usb.h"
#include "cmsis_os2.h"
#include "bsp_usart.h"
#include "bsp_can.h"

class Class_Chassis
{
public:
    // 底盘4个3508， 控制全向轮
    Class_Motor_DJI_C620 Motor_Chassis_1,
                         Motor_Chassis_2,
                         Motor_Chassis_3,
                         Motor_Chassis_4;

    void Init();
    void Task();
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Velocity_Rotation(float __Target_Velocity_Rotation);
protected:
    // 目标速度X
    float Target_Velocity_X = 0.0f;
    // 目标速度Y
    float Target_Velocity_Y = 0.0f;
    // 目标速度 旋转
    float Target_Velocity_Rotation = 0.0f;
    void Kinematics_Inverse_Resolution();
    void Output_To_Motor();
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};
/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
inline void Class_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
inline void Class_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

/**
 * @brief 设定目标速度旋转
 *
 * @param __Target_Velocity_Rotation 目标速度Y
 */
inline void Class_Chassis::Set_Target_Velocity_Rotation(float __Target_Velocity_Rotation)
{
    Target_Velocity_Rotation = __Target_Velocity_Rotation;
}


#endif // !CHASSIS_H