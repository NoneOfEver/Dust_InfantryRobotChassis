/**
 * @file Robot.cpp
 * @author noe (noneofever@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-04
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "Robot.h"
#include "cmsis_os2.h"

// app
#include "app_chassis.h"
#include "math.h"
#include "user_lib.h"
#include "alg_math.h"
// module
#include "dvc_mcu_comm.h"
#include "imu.hpp"
#include "debug_tools.h"

// bsp
#include "bsp_dwt.h"

void Robot::Init()
{
    dwt_init(480);
    // 上下板通讯组件初始化
    mcu_comm_.Init(&hfdcan2, 0x01, 0x00);
    // 陀螺仪初始化
    imu_.Init();
    osDelay(pdMS_TO_TICKS(10000));// 10s时间等待陀螺仪收敛

    // 底盘跟随控制PID初始化  17.0f,0.0f,0.0f,5.0f,0.0f,6.0f,0.001f,0.0f,0.0f,0.0f,0.0f
    chassis_follow_pid_.Init(
        17.0f,
        0.0f,
        0.0f,
        5.0f,
        0.0f,
        6.0f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    );
    // yaw轴角度环PID初始化 0.045f,0.050800f,0.001500f,0.1f,0.0f,45.0f,0.001f
    //                     0.047f,0.050800f,0.002000f,0.1f,25.0f,45.0f,0.001f
    //                     0.048f,0.050800f,0.002000f,0.1f,25.0f,45.0f,0.001f,0.0f,0.0f,0.0f,180
    // yaw_angle_pid_.Init(
    //     0.080f,
    //     0.050800f,
    //     0.002000f,
    //     10.0f,
    //     25.0f,
    //     44.0f,
    //     0.001f,
    //     0.0f,
    //     0.0f,
    //     0.0f,
    //     15.0f
    // );
    yaw_angle_pid_.Init(
        250.0f,
        10.0f,
        3.0f,
        4.0f,
        0.0f,
        44.0f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f    
    );

    //pitch轴角度环PID初始化
    pitch_angle_pid_.Init(
        350.0f,
        160.0f,
        2.0f,
        44.0f,
        0.0f,
        44.0f,
        0.001f,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    );
    // 云台初始化
    gimbal_.Init();
    // 底盘初始化
    chassis_.Init();
    ramp_init(&chassis_spin_ramp_source, 0.0005f, 10.0f, -10.0f);
    // 超级电容初始化
    supercap_.Init(&hfdcan3, 0x100, 0x003);

    // 初始化虚拟角度
    // virtual_yaw_angle_ = mcu_comm_.mcu_imu_data_.yaw_total_angle_f;
    virtual_yaw_angle_ = 0.0f;
    virtual_pitch_angle_ = 0.0f;
    static const osThreadAttr_t kRobotTaskAttr = {
        .name = "robot_task",
        .stack_size = 768,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Robot::TaskEntry, this, &kRobotTaskAttr);
}

// 任务入口
void Robot::TaskEntry(void *argument)
{
    Robot *self = static_cast<Robot *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

void Robot::Task()
{
    McuCommData mcu_comm_data_local;
    McuCommData mcu_comm_data_local_pre;
    uint8_t first_flag = 0;
    float error = 0.0f;
    float adjusted_now = 0.0f;
    for (;;)
    {
        __disable_irq();
        mcu_comm_data_local = *const_cast<const McuCommData*>(&(mcu_comm_.mcu_comm_data_));
        __enable_irq();
        if(first_flag == 0){
            first_flag = 1;
            mcu_comm_data_local_pre = mcu_comm_data_local;
        }

        /********************** 云台 ***********************/   
        // virtual_yaw_angle_ += ( 127.0f - mcu_comm_data_local.yaw)*YAW_SENSITIVITY;
        // virtual_pitch_angle_ += (mcu_comm_data_local.pitch_angle - 127.0f)*(0.3f/128.0f)*YAW_SENSITIVITY;
        // virtual_yaw_angle_ = (mcu_comm_data_local.yaw - 127.0f)*(M_PI/128.0f);
        // virtual_pitch_angle_ = (mcu_comm_data_local.pitch_angle - 127.0f)*(0.3f/128.0f);
        if (virtual_pitch_angle_ >= 0.3f){
            virtual_pitch_angle_ = 0.3f;
        }else if(virtual_pitch_angle_ <= -0.3f){
            virtual_pitch_angle_ = -0.3f;
        }
        // virtual_angle_ = (127.0f - mcu_comm_data_local.yaw); //绝对值映射
        // yaw_angle_pid_.SetNow(mcu_comm_.mcu_imu_data_.yaw_total_angle_f);
        // yaw_angle_pid_.SetNow(gimbal_.GetYawNowAngleNoncumulative());
        yaw_angle_pid_.SetTarget(0);
        float yaw_err = CalcYawError(virtual_yaw_angle_, gimbal_.GetYawNowAngleNoncumulative());
        yaw_angle_pid_.SetNow(yaw_err);
        yaw_angle_pid_.CalculatePeriodElapsedCallback();
        // yaw_angle_pid_.SetTarget(virtual_yaw_angle_);
        // yaw_angle_pid_.SetNow(gimbal_.GetYawNowAngleNoncumulative());
        // yaw_angle_pid_.CalculatePeriodElapsedCallback();

        pitch_angle_pid_.SetTarget(virtual_pitch_angle_);
        pitch_angle_pid_.SetNow(gimbal_.GetPitchNowAngleNoncumulative());
        pitch_angle_pid_.CalculatePeriodElapsedCallback();

        // 遥控模式
        gimbal_.SetTargetYawOmega(-(yaw_angle_pid_.GetOut() + gimbal_.GetYawOmegaFeedforword())); //补偿速度可能符号错了
        gimbal_.SetTargetPitchOmega(pitch_angle_pid_.GetOut());
        // gimbal_.SetTargetYawOmega((mcu_comm_data_local.yaw - 127.0f) * 3.0f / 128.0f);
        // gimbal_.SetTargetPitchAngle((mcu_comm_data_local.pitch_angle - 127.0f) * (0.3f/128.0f));
        /********************** 底盘 ***********************/ 
        // if((mcu_comm_data_local.chassis_spin == CHASSIS_SPIN_DISABLE) && 
        //    (chassis_spin_ramp_source.is_completed == 1)) 
        // {
            error = normalize_angle_diff(0.0f, gimbal_.GetYawNowAngleNoncumulative());
            adjusted_now = 0.0f - error;
            chassis_follow_pid_.SetTarget(0.0f); // 相对于底盘弧度 0rad
            chassis_follow_pid_.SetNow(adjusted_now);
            chassis_follow_pid_.CalculatePeriodElapsedCallback();

            chassis_.SetTargetVelocityX((mcu_comm_data_local.chassis_speed_x - 127.0f) * 10.0f / 128.0f); //9
            chassis_.SetTargetVelocityY((mcu_comm_data_local.chassis_speed_y - 127.0f) * 10.0f / 128.0f); //9
            // chassis_.SetTargetVelocityRotation(((mcu_comm_data_local.chassis_rotation - 127.0f) * 9.0f / 128.0f)-chassis_follow_pid_.GetOut());
            // gimbal_.SetYawOmegaFeedforword(-0.9f*chassis_follow_pid_.GetOut());
            // chassis_.SetTargetVelocityRotation(((mcu_comm_data_local.chassis_rotation - 127.0f) * 9.0f / 128.0f));
        // } 


        /********************** 小陀螺 ***********************/   
        switch(mcu_comm_data_local.chassis_spin)
        {
            case CHASSIS_SPIN_CLOCKWISE:
                ramp_temp = ramp_calc(&chassis_spin_ramp_source,10.0f);
                chassis_.SetTargetVelocityRotation(ramp_temp);
                gimbal_.SetYawOmegaFeedforword(0.4f * ramp_temp);
            break;
            case CHASSIS_SPIN_DISABLE:
                chassis_spin_ramp_source.out = 0.0f; // 清零
                if(mcu_comm_data_local_pre.chassis_spin == CHASSIS_SPIN_CLOCKWISE){
                    ramp_temp = ramp_calc(&chassis_spin_ramp_source,-10.0f) + chassis_spin_ramp_source.max_value;
                    chassis_.SetTargetVelocityRotation(ramp_temp);
                    gimbal_.SetYawOmegaFeedforword(0.4f * ramp_temp);
                }else if(mcu_comm_data_local_pre.chassis_spin == CHASSIS_SPIN_COUNTER_CLOCK_WISE){
                    ramp_temp = ramp_calc(&chassis_spin_ramp_source,10.0f) - chassis_spin_ramp_source.max_value;
                    chassis_.SetTargetVelocityRotation(ramp_temp);
                    gimbal_.SetYawOmegaFeedforword(0.4f * ramp_temp);
                }
            break;
            case CHASSIS_SPIN_COUNTER_CLOCK_WISE:
                ramp_temp = ramp_calc(&chassis_spin_ramp_source,-10.0f);
                chassis_.SetTargetVelocityRotation(ramp_temp);
                gimbal_.SetYawOmegaFeedforword(0.4f * ramp_temp);
            break;
            default:
            // do nothing
            break; 
        };


        /********************** 调试信息 ***********************/   
        // debug_tools_.VofaSendFloat(mcu_comm_.mcu_imu_data_.yaw_total_angle_f);
        // debug_tools_.VofaSendFloat(virtual_pitch_angle_);
        // debug_tools_.VofaSendFloat(gimbal_.motor_yaw_.GetNowAngle());
        debug_tools_.VofaSendFloat(gimbal_.GetPitchNowAngleNoncumulative());
        
        // debug_tools_.VofaSendFloat(gimbal_.GetNowPitchOmega());
        // debug_tools_.VofaSendFloat((mcu_comm_data_local.pitch_angle - 127.0f) * (0.3f/128.0f));
        // debug_tools_.VofaSendFloat(gimbal_.GetNowPitchAngle());
        
        debug_tools_.VofaSendFloat(virtual_pitch_angle_);

        debug_tools_.VofaSendFloat(yaw_angle_pid_.GetOut());

        debug_tools_.VofaSendFloat(mcu_comm_.mcu_autoaim_data_.pitch_f);
        // 调试帧尾部
        debug_tools_.VofaSendTail();


        /********************** mini PC ***********************/   
        memcpy(&mcu_comm_.mcu_autoaim_data_.pitch_f,mcu_comm_.mcu_autoaim_data_.pitch,sizeof(float));
        if((fabs(mcu_comm_.mcu_autoaim_data_.pitch_f) > 0.00f) && (fabs(mcu_comm_.mcu_autoaim_data_.pitch_f) <0.3f)){
            virtual_pitch_angle_ -= mcu_comm_.mcu_autoaim_data_.pitch_f;
        }
        // memcpy(&mcu_comm_.mcu_autoaim_data_.yaw_f,mcu_comm_.mcu_autoaim_data_.yaw,sizeof(float));
        // virtual_yaw_angle_ -= mcu_comm_.mcu_autoaim_data_.yaw_f;
        // 回传云台电机角度数据
        mcu_comm_.mcu_send_data_.armor = 0x00;
        mcu_comm_.mcu_send_data_.yaw = -yaw_err;
        mcu_comm_.mcu_send_data_.pitch = gimbal_.GetPitchNowAngleNoncumulative();
        mcu_comm_.CanSendCommand();

        
        /********************** 超级电容 ***********************/   
        if(mcu_comm_data_local.supercap == 0){
            supercap_.SetChargeStatus(SUPERCAP_STATUS_CHARGE);
        }else if(mcu_comm_data_local.supercap == 1){
            supercap_.SetChargeStatus(SUPERCAP_STATUS_DISCHARGE);
        }else{
            supercap_.SetChargeStatus(SUPERCAP_STATUS_CHARGE);
        }
        supercap_.SetPowerLimitMax(100);
        supercap_.SetChargePower(50);
        
        mcu_comm_data_local_pre = mcu_comm_data_local;

        osDelay(pdMS_TO_TICKS(1));// 1khz
    }
}
