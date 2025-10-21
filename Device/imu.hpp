#ifndef IMU_H_
#define IMU_H_

#include "BMI088driver.h"
#include "spi.h"
#include "ins_task.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"


class Imu {
private:
    // FreeRTOS 入口，静态函数
    static void TaskEntry(void *argument)
    {
        Imu *self = static_cast<Imu *>(argument);  // 还原 this 指针
        self->Task();  // 调用成员函数

    }
public:
    void Task()
    {
        for(;;)
        {
            INS_Task();
            osDelay(pdMS_TO_TICKS(1)); // 1kHz
        }
        
    }
    void Init()
    {
        // 陀螺仪初始化
        BMI088_Init(&hspi2, 0);// 启用校准模式    
        INS_Init(); // 逆时针为+ ，-180 ~ 180
        static const osThreadAttr_t ImuTaskAttr = {
            .name = "ImuTask",
            .stack_size = 512,
            .priority = (osPriority_t) osPriorityNormal
        };
        // 启动任务，将 this 传入
        osThreadNew(TaskEntry, this, &ImuTaskAttr);
    }

    inline float GetYawAngle()
    {
        return INS.Yaw;
    }
};

#endif /* IMU_H_ */