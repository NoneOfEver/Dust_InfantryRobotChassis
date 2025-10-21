#ifndef DEVICE_SUPERCAP_H_
#define SUPERCAP_H_

#include "bsp_can.h"
#include <cstdint>

/**
 * @brief 超级电容状态
 * @note  包含工作状态和充放电状态
 */
enum SupercapStatus
{
    SUPERCAP_STATUS_DISABLE = 0,
    SUPERCAP_STATUS_ENABLE = 1,
    SUPERCAP_STATUS_CHARGE = 1,
    SUPERCAP_STATUS_DISCHARGE = 0,
};

/**
 * @brief 超级电容状态码
 * 
 */
enum SupercapStatusCode
{
    DISCHARGE = 0,              // 放电
    CHARGE = 1,                 // 充电
    WAIT = 2,                   // 待机 
    SOFRSTART_PROTECTION = 3,   // 软启动保护
    OCP_PROTECTION = 4,         // 过流保护
    OVP_BAT_PROTECTION = 5,     // 电池过压保护
    UVP_BAT_PROTECTION = 6,     // 电池欠压保护
    UVP_CAP_PROTECTION = 7,     // 电容欠压保护
    OTP_PROTECTION = 8          // 过温保护
};

/**
 * @brief 超级电容接收数据
 * @note 200Hz频率，可调
 */
struct SupercapRecivedData
{
    SupercapStatus supercap_work_status;        // 超级电容可用状态
    SupercapStatusCode supercap_status_code;    // 超级电容充放电状态
    uint8_t supercap_energy_percent;            // 超级电容剩余能量百分比 0~100% 0%的时候会自动关闭
    uint8_t chassis_compensate_power;           // 超级电容当前功率消耗 W 范围：0~255
    uint8_t battery_voltage;                    // 电池电压 V，放大了10倍，
};

/**
 * @brief 超级电容发送数据
 * 
 */
struct SupercapSendData
{
    SupercapStatus supercap_enable_status;        // 超级电容工作状态
    SupercapStatus supercap_charge_status;        // 超级电容充放电状态
    uint8_t power_limit_max;                      // 底盘功率限制 W
    uint8_t charge_power;                         // 充电功率 W
};

class Supercap
{
public:
    void Init(
        FDCAN_HandleTypeDef *hcan,
        uint16_t can_rx_id = 0x100,
        uint16_t can_tx_id = 0x001
    );

    inline SupercapStatus GetWorkStatus();
    
    inline SupercapStatusCode GetStatusCode();
    
    inline uint8_t GetEnergyPercent();

    inline uint8_t GetChassisCompensatePower();

    inline uint8_t GetBatteryVoltage();

    inline void SetEnableStatus(SupercapStatus supercap_enable_status);

    inline void SetChargeStatus(SupercapStatus supercap_charge_status);

    inline void SetPowerLimitMax(uint8_t power_limit_max);

    inline void SetChargePower(uint8_t charge_power);

    void CanRxCpltCallback(uint8_t *rx_data);

    void AlivePeriodElapsedCallback();

    void SendPeriodElapsedCallback();

    void Task();
protected:
    CanManageObject *can_manage_object_ = nullptr;
    uint16_t can_rx_id_ = 0x100;
    uint16_t can_tx_id_ = 0x003;
    SupercapRecivedData recived_data_ = {};
    uint32_t flag_ = 0;
    uint32_t pre_flag_ = 0;

    // 发送给超级电容的使能/失能状态
    SupercapStatus supercap_enable_status_ = SUPERCAP_STATUS_DISABLE;
    
    // 接收到的超级电容真实的工作状态
    SupercapStatus supercap_work_status_ = SUPERCAP_STATUS_DISABLE;

    // 超级电容充放电状态
    SupercapStatus supercap_charge_status_ = SUPERCAP_STATUS_CHARGE;
    
    // 超级电容状态码
    SupercapStatusCode supercap_status_code_ = {};

    // 底盘功率上限
    uint8_t power_limit_max_ = 0;
    
    // 充电功率
    uint8_t charge_power_ = 0;
    
    // 超级电容剩余能量百分比
    uint8_t supercap_energy_percent_ = 0;

    // 底盘消耗功率
    uint8_t chassis_power_ = 0;

    // 电池电压
    uint8_t battery_voltage_ = 0;

    // 超级电容可以补偿的最大功率值
    uint8_t power_compensate_max_ = 150.0f;

    void DataProcess();
    void Output();
    static void TaskEntry(void *param);  // FreeRTOS 入口，静态函数

};

/**
 * @brief 获取超级电容在线状态
 *
 * @return uint8_t 超级电容在线状态
 */
inline SupercapStatus Supercap::GetWorkStatus()
{
    return (recived_data_.supercap_work_status);
}

/**
 * @brief 获取超级电容状态码
 * 
 * @return SupercapStatusCode 
 */
inline SupercapStatusCode Supercap::GetStatusCode()
{
    return (recived_data_.supercap_status_code);
}

/**
 * @brief 获取超级电容剩余能量百分比
 * 
 * @return uint8_t 
 */
inline uint8_t Supercap::GetEnergyPercent()
{
    return (recived_data_.supercap_energy_percent);
}

/**
 * @brief 获取底盘消耗功率
 * 
 * @return uint8_t 
 */
inline uint8_t Supercap::GetChassisCompensatePower()
{
    return (recived_data_.chassis_compensate_power);
}

/**
 * @brief 获取电池电压
 * 
 * @return uint8_t 
 */
inline uint8_t Supercap::GetBatteryVoltage()
{
    return (recived_data_.battery_voltage);
}

/**
 * @brief 设置超级电容使能状态
 * 
 * @param supercap_enable_status 
 */
inline void Supercap::SetEnableStatus(SupercapStatus supercap_enable_status)
{
    supercap_enable_status_ = supercap_enable_status;
}

/**
 * @brief 设置超级电容充放电状态
 * 
 * @param supercap_charge_status 
 */
inline void Supercap::SetChargeStatus(SupercapStatus supercap_charge_status)
{
    supercap_charge_status_ = supercap_charge_status;
}

/**
 * @brief 设置底盘功率上限
 * 
 * @param power_limit_max 
 */
inline void Supercap::SetPowerLimitMax(uint8_t power_limit_max)
{
    power_limit_max_ = power_limit_max;
}

/**
 * @brief 设置充电功率
 * 
 * @param charge_power 
 */
inline void Supercap::SetChargePower(uint8_t charge_power)
{
    charge_power_ = charge_power;
}

#endif // !DEVICE_SUPERCAP_H_
