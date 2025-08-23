// app
#include "app_chassis.h"
// module
#include "dvc_MCU_comm.h"

void Class_Chassis::Init()
{
    // 3508电机初始化
    Motor_Chassis_1.PID_Omega.Init(1.0f,0.0f,0.0f);
    Motor_Chassis_2.PID_Omega.Init(1.0f,0.0f,0.0f);
    Motor_Chassis_3.PID_Omega.Init(1.0f,0.0f,0.0f);
    Motor_Chassis_4.PID_Omega.Init(1.0f,0.0f,0.0f);

    Motor_Chassis_1.Init(&hfdcan1, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_OMEGA);
    Motor_Chassis_2.Init(&hfdcan1, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_OMEGA);
    Motor_Chassis_3.Init(&hfdcan1, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_OMEGA);
    Motor_Chassis_4.Init(&hfdcan1, Motor_DJI_ID_0x204, Motor_DJI_Control_Method_OMEGA);

    Motor_Chassis_1.Set_Target_Omega(0.0f);
    Motor_Chassis_2.Set_Target_Omega(0.0f);
    Motor_Chassis_3.Set_Target_Omega(0.0f);
    Motor_Chassis_4.Set_Target_Omega(0.0f);

    static const osThreadAttr_t ChassisTaskAttr = {
        .name = "ChassisTask",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Class_Chassis::TaskEntry, this, &ChassisTaskAttr);

}
// 任务入口（静态函数）—— osThreadNew 需要这个原型
void Class_Chassis::TaskEntry(void *argument)
{
    Class_Chassis *self = static_cast<Class_Chassis *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

// 实际任务逻辑
void Class_Chassis::Task()
{
    for (;;)
    {
        // 平移速度 + 自旋速度
        Motor_Chassis_1.Set_Target_Omega( (-Target_Velocity_X - Target_Velocity_Y)
                                        + (-Target_Velocity_Rotation));
        Motor_Chassis_2.Set_Target_Omega( (-Target_Velocity_X + Target_Velocity_Y)
                                        + (-Target_Velocity_Rotation));
        Motor_Chassis_3.Set_Target_Omega( ( Target_Velocity_X + Target_Velocity_Y)
                                        + (-Target_Velocity_Rotation));
        Motor_Chassis_4.Set_Target_Omega( ( Target_Velocity_X - Target_Velocity_Y)
                                        + (-Target_Velocity_Rotation));

        Motor_Chassis_1.Calculate_PeriodElapsedCallback();
        Motor_Chassis_2.Calculate_PeriodElapsedCallback();
        Motor_Chassis_3.Calculate_PeriodElapsedCallback();
        Motor_Chassis_4.Calculate_PeriodElapsedCallback();
        // 全向轮底盘电机
        CAN_Send_Data(&hfdcan1, 0x200, CAN1_0x200_Tx_Data, 8);
        osDelay(pdMS_TO_TICKS(10));
    }
}

