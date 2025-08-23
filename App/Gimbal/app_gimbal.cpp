#include "app_gimbal.h"

void Class_Gimbal::Init()
{
    // 6220电机初始化
    Motor_Yaw.Init(&hfdcan3, 0x12, 0x01);
    //Motor_Pitch.Init(&hfdcan3, 0x11, 0x02);

    Motor_Yaw.CAN_Send_Clear_Error();
    HAL_Delay(1000);
    Motor_Yaw.CAN_Send_Enter();
    //Motor_Pitch.CAN_Send_Enter();
    HAL_Delay(1000);

    Motor_Yaw.Set_K_P(0);
    //Motor_Yaw.Set_K_P(85); //MIT模式kp
    //Motor_Pitch.Set_K_P(60);

    Motor_Yaw.Set_K_D(0.3);
    //Motor_Yaw.Set_K_D(0.01); // MIT模式kd
    //Motor_Pitch.Set_K_D(0.3);

    //Motor_Yaw.Set_Control_Angle(0);
    //Motor_Pitch.Set_Control_Angle(-0.3);

    Motor_Yaw.Set_Control_Omega(0);
    //Motor_Pitch.Set_Control_Omega(45);

    Motor_Yaw.Set_Control_Torque(0.2);
    // Motor_Pitch.Set_Control_Torque(10);

    Motor_Yaw.Output();
    //osDelay(pdMS_TO_TICKS(10));
    //Motor_Pitch.Output();

    static const osThreadAttr_t GimbalTaskAttr = {
        .name = "GimbalTask",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Class_Gimbal::TaskEntry, this, &GimbalTaskAttr);
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
        Motor_Yaw.Output();
        //Motor_Pitch.Output();
        osDelay(pdMS_TO_TICKS(10));
    }
}