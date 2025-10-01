#ifndef APP_ROBOT_H_
#define APP_ROBOT_H_
#include "FreeRTOS.h"
// app
#include "app_chassis.h"
#include "app_gimbal.h"
// module
#include "dvc_MCU_comm.h"
#include "supercap.h"

/**
 * @brief 小陀螺类型
 * 
 */
enum RobotGyroscopeType
{
    ROBOT_GYROSCOPE_TYPE_DISABLE = 0,
    ROBOT_GYROSCOPE_TYPE_CLOCKWISE,
    ROBOT_GYROSCOPE_TYPE_COUNTERCLOCKWISE,
};

class Robot
{
public:

    // 与上板的通讯服务
    McuComm mcu_comm_;
    // 底盘跟随控制PID
    Pid chassis_follow_pid_;
    // 底盘
    Chassis chassis_;
    // 云台
    Gimbal  gimbal_;
    // 超级电容模组
    Supercap supercap_;
    /**
     * @brief 机器人初始化
     */
    void Init();
    void Task();
protected:
    // 小陀螺功能状态
    RobotGyroscopeType chassis_gyroscope_mode_status_ = ROBOT_GYROSCOPE_TYPE_DISABLE;
    // 底盘跟随模式是否使能
    bool chassis_follow_mode_status_ = true;
    // 机器人等级
    int32_t robot_level_ = 1;
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

#endif // !APP_ROBOT_H_