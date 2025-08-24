#include "app_gimbal.h"

void Class_Gimbal::Init()
{
    // 6220电机初始化
    Motor_Yaw.Init(&hfdcan3, 0x12, 0x01);
    Motor_Pitch.Init(&hfdcan3, 0x11, 0x02);

    Yaw_Angle_PID.Init(1,0,0,0,0,3.14f);

    Motor_Yaw.CAN_Send_Clear_Error();
    HAL_Delay(1000);
    Motor_Yaw.CAN_Send_Enter();
    Motor_Pitch.CAN_Send_Enter();
    HAL_Delay(1000);

    //Motor_Yaw.Set_K_P(0);
    Motor_Yaw.Set_K_P(0); //MIT模式kp
    Motor_Pitch.Set_K_P(3);

    Motor_Yaw.Set_K_D(0.3); // MIT模式kd
    Motor_Pitch.Set_K_D(0.03);

    //Motor_Yaw.Set_Control_Angle(0);
    //Motor_Pitch.Set_Control_Angle(-0.3);

    Motor_Yaw.Set_Control_Omega(0);
    Motor_Pitch.Set_Control_Omega(45);

    // Motor_Yaw.Set_Control_Torque(0.2);
    // Motor_Pitch.Set_Control_Torque(10);

    Motor_Yaw.Output();
    //osDelay(pdMS_TO_TICKS(10));
    Motor_Pitch.Output();

    static const osThreadAttr_t GimbalTaskAttr = {
        .name = "GimbalTask",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Class_Gimbal::TaskEntry, this, &GimbalTaskAttr);
}


/**
 * @brief 自身解算
 *
 */
void Class_Gimbal::Self_Resolution()
{
    Now_Pitch_Angle = Motor_Pitch.Get_Now_Angle();
    Now_Yaw_Angle   = Motor_Yaw.Get_Now_Angle();
    // Yaw_Angle_PID.Set_Now(Now_Yaw_Angle);
    //
    // // pitch轴角度归化到±PI / 2之间
    // Now_Pitch_Angle = Math_Modulus_Normalization(-Motor_Pitch.Get_Now_Angle(), 2.0f * PI);

}

/**
 * @brief 输出到电机
 *
 */
void Class_Gimbal::Output()
{

    // 云台位控
    if (Gimbal_Control_Type == Gimbal_Control_Type_Manual)         // 无自瞄介入
    {
        // do nothing
    }else if (Gimbal_Control_Type == Gimbal_Control_Type_AutoAim){ // 有自瞄矫正
        _Motor_Nearest_Transposition();
        Yaw_Angle_PID.Set_Target(Now_Yaw_Angle); // 加视觉的相对偏移量
        Yaw_Angle_PID.Calculate_PeriodElapsedCallback();
        Target_Yaw_Omega = Yaw_Angle_PID.Get_Out();
    }


    Motor_Yaw.Set_Control_Omega(Target_Yaw_Omega);
    Motor_Pitch.Set_Control_Angle(Target_Pitch_Angle);

    Motor_Yaw.Output();
    Motor_Pitch.Output();

}

/**
 * @brief 电机就近转位
 *
 */
void Class_Gimbal::_Motor_Nearest_Transposition()
{
    // Yaw就近转位
    float tmp_delta_angle;
    tmp_delta_angle = fmod(Target_Yaw_Angle - Now_Yaw_Angle, 2.0f * PI);
    if (tmp_delta_angle > PI)
    {
        tmp_delta_angle -= 2.0f * PI;
    }
    else if (tmp_delta_angle < -PI)
    {
        tmp_delta_angle += 2.0f * PI;
    }
    Target_Yaw_Angle = Motor_Yaw.Get_Now_Angle() + tmp_delta_angle;

    // // Pitch就近转位
    // Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
    // tmp_delta_angle = Target_Pitch_Angle - Now_Pitch_Angle;
    // Target_Pitch_Angle = -Motor_Pitch.Get_Now_Angle() + tmp_delta_angle;
}
// 任务入口（静态函数）—— osThreadNew 需要这个原型
void Class_Gimbal::TaskEntry(void *argument)
{
    Class_Gimbal *self = static_cast<Class_Gimbal *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

// 实际任务逻辑（无限循环）
void Class_Gimbal::Task()
{
    for (;;)
    {
        Self_Resolution();
        Output();
        osDelay(pdMS_TO_TICKS(1));
    }
}