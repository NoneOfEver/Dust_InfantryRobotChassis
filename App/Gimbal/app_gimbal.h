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

class Class_Gimbal
{
public:
    // 2个DM6220，作为云台Yaw和Pitch轴控制电机
    Class_Motor_DM_Normal Motor_Yaw , Motor_Pitch;

    void Init();
    void Task();

protected:
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数
};


#endif // !GIMBAL_H