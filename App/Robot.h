#ifndef APP_ROBOT_H_
#define APP_ROBOT_H_
#include "FreeRTOS.h"
// app
#include "app_chassis.h"
#include "app_gimbal.h"
// module
#include "dvc_MCU_comm.h"

class Robot
{
public:

    // 与上板的通讯服务
    McuComm mcu_comm_;
    // 底盘
    Chassis chassis_;
    // 云台
    Gimbal  gimbal_;

    /**
     * @brief 机器人初始化
     */
    void Init();
    void Task();
protected:
    // 机器人等级
    int32_t robot_level_ = 1;
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

#endif // !APP_ROBOT_H_