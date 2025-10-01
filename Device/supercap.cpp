#include "supercap.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include <cstdint>
/**
 * @brief 超级电容初始化
 * 
 * @param hcan 
 * @param can_rx_id 
 * @param can_tx_id 
 */
void Supercap::Init(
    FDCAN_HandleTypeDef *hcan,
    uint16_t can_rx_id,
    uint16_t can_tx_id
)
{
    if (hcan->Instance == FDCAN1)
    {
        can_manage_object_ = &g_can1_manage_object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        can_manage_object_ = &g_can2_manage_object;
    }
    else if (hcan->Instance == FDCAN3)
    {
        can_manage_object_ = &g_can3_manage_object;
    }
    can_rx_id_ = can_rx_id;
    can_tx_id_ = can_tx_id;

    power_limit_max_ = 55;
    power_compensate_max_ = 50;
    supercap_enable_status_ = SUPERCAP_STATUS_ENABLE;

     static const osThreadAttr_t kSupercapTaskAttr = {
        .name = "supercap_task",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(Supercap::TaskEntry, this, &kSupercapTaskAttr);

}

// 任务入口（静态函数）—— osThreadNew 需要这个原型
void Supercap::TaskEntry(void *argument)
{
    Supercap *self = static_cast<Supercap *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

/**
 * @brief 超级电容CAN通讯接收回调函数
 * 
 * @param rx_data 
 */
void Supercap::CanRxCpltCallback(uint8_t *rx_data)
{
    // 滑动窗口，判断超级电容在线状态
    flag_ += 1;
    DataProcess();
}

void Supercap::AlivePeriodElapsedCallback()
{
    // TODO:待实现
}

/**
 * @brief 超级电容数据处理
 * 
 */
void Supercap::DataProcess()
{
    SupercapRecivedData *temp_buffer = (SupercapRecivedData *)can_manage_object_->rx_buffer.data;

    recived_data_.supercap_work_status = temp_buffer->supercap_work_status;
    recived_data_.supercap_status_code = temp_buffer->supercap_status_code;
    recived_data_.supercap_energy_percent = temp_buffer->supercap_energy_percent;
    recived_data_.chassis_compensate_power = temp_buffer->chassis_compensate_power;
    recived_data_.battery_voltage = temp_buffer->battery_voltage;
}

/**
 * @brief 超级电容CAN通讯发送回调函数
 * 
 */
void Supercap::SendPeriodElapsedCallback()
{
    uint8_t temp_buffer[8];
    temp_buffer[0] = supercap_enable_status_;
    temp_buffer[1] = supercap_charge_status_;
    temp_buffer[2] = power_limit_max_;
    temp_buffer[3] = charge_power_;
    temp_buffer[4] = 0;
    temp_buffer[5] = 0;
    temp_buffer[6] = 0;
    temp_buffer[7] = 0;
    can_send_data(can_manage_object_->can_handler, can_tx_id_, temp_buffer, 8);
}

void Supercap::Task()
{
    for (;;)
    {
        AlivePeriodElapsedCallback();
        SendPeriodElapsedCallback();
        osDelay(pdMS_TO_TICKS(10));
    }
}
/************************ COPYRIGHT(C) HNUST-DUST **************************/
