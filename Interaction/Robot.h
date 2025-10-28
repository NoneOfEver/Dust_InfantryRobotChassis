#ifndef APP_ROBOT_H_
#define APP_ROBOT_H_
// app
#include "app_chassis.h"
#include "app_gimbal.h"
#include "user_lib.h"
// module
#include "dvc_mcu_comm.h"
#include "supercap.h"
#include "debug_tools.h"

#define YAW_SENSITIVITY     0.001F

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
    // 调试工具
    DebugTools debug_tools_;
    // 与上板的通讯服务
    McuComm mcu_comm_;
    // 底盘跟随控制PID
    Pid chassis_follow_pid_;
    // 底盘
    Chassis chassis_;
    // 底盘小陀螺斜坡规划器
    float ramp_temp = 0.0f;
    ramp_function_source_t chassis_spin_ramp_source;

    // 云台
    Gimbal  gimbal_;
    // 超级电容模组
    Supercap supercap_;
    // 底盘陀螺仪
    Imu imu_;
    // 云台yaw轴角度环pid
    Pid yaw_angle_pid_;
    // 云台pitch轴角度环pid
    Pid pitch_angle_pid_;

    /**
     * @brief 机器人初始化
     */
    void Init();
    void Task();
protected:
    // 操作手控制的的虚拟角度
    float virtual_yaw_angle_ = 0;
    float virtual_pitch_angle_ = 0;
    
    // 小陀螺功能状态
    RobotGyroscopeType chassis_gyroscope_mode_status_ = ROBOT_GYROSCOPE_TYPE_DISABLE;
    // 底盘跟随模式是否使能
    bool chassis_follow_mode_status_ = true;
    // 机器人等级
    int32_t robot_level_ = 1;
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};

#endif // !APP_ROBOT_H_