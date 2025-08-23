#ifndef ROBOT_H
#define ROBOT_H
#include "FreeRTOS.h"
// app
#include "app_chassis.h"
#include "app_gimbal.h"
// module
#include "dvc_MCU_comm.h"

class Class_Robot
{
public:


    // 与上板的通讯服务
    Class_MCU_Comm MCU_Comm;
    // 底盘
    Class_Chassis Chassis;
    // 云台
    Class_Gimbal Gimbal;

    /**
     * @brief 机器人初始化
     */
    void Init();
    void Task();
protected:
    // 机器人等级
    int32_t Robot_Level = 1;
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

#endif // !ROBOT_H