#ifndef MODULES_COMM_DVC_PC_COMM_H
#define MODULES_COMM_DVC_PC_COMM_H

#include "bsp_usb.h"

class PcComm
{
public:

    void Init();
    void Task();

protected:
    // FreeRTOS 入口，静态函数
    static void TaskEntry(void *param);
};

#endif //MODULES_COMM_DVC_PC_COMM_H
