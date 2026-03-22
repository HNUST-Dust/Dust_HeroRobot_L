/**
 * @file supercap.h
 * @author qingyu
 * @brief 超级电容模块头文件
 * @version 0.1
 * @date 2025-11-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

/* Includes ------------------------------------------------------------------*/

#include "bsp_can.h"
#include "cmsis_os2.h"
#include "Timer.hpp"
#include "stdio.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Supercap使能状态枚举
 * 
 */
enum SupercapEnableStatus : uint8_t
{
    SUPERCAP_STATUS_DISABLE = 0,
    SUPERCAP_STATUS_ENABLE,
};

/**
 * @brief Supercap充放电状态枚举
 * 
 */
enum SupercapChargeStatus : uint8_t
{
    SUPERCAP_STATUS_DISCHARGE = 0,
    SUPERCAP_STATUS_CHARGE
};

/**
 * @brief Supercap就绪状态枚举
 * 
 */
enum SuperCapReadyStatus : uint8_t
{
    SUPERCAP_STATUS_UNREADY = 0,
    SUPERCAP_STATUS_READY
};

/**
 * @brief Supercap工作状态枚举
 * 
 */
enum SupercapStatus : uint8_t
{
    DISCHARGE = 0,
    CHARGE = 1,
    WAIT = 2,
    SOFRSTART_PROTECTION = 3,
    OCP_PROTECTION = 4,
    OVP_BAT_PROTECTION = 5,
    UVP_BAT_PROTECTION = 6,
    UVP_CAP_PROTECTION = 7,
    OTP_PROTECTION = 8
};

/**
 * @brief Supercap接收数据结构体
 * 
 */
struct SupercapRecvData
{
    uint8_t supercap_energy;
    uint8_t chassis_power;
    SuperCapReadyStatus ready_status;
    SupercapStatus supercap_status;
    float bat_voltage;
    uint8_t bat_power;
    int8_t supercap_power;
};

/**
 * @brief Supercap发送数据结构体
 * 
 */
struct SupercapSendData
{
    SupercapChargeStatus supercap_charge;
    SupercapEnableStatus supercap_enable;
    uint8_t power_limit;
    uint8_t charge_power;
};

/**
 * @brief Supercap类
 * 
 */
class Supercap
{
public:
    Supercap() = default;

    ~Supercap() = default;

    void Init(CAN_HandleTypeDef* hcan, uint16_t can_rx_id, uint16_t can_tx_id);

    void CanRxCpltCallback(uint8_t *rx_data);

    inline void SetSupcapChargeStatus(SupercapChargeStatus supercap_charge) {
        tx_data_.supercap_charge = supercap_charge;
    }

    inline void SetSupcapEnableStatus(SupercapEnableStatus supercap_enable) {
        tx_data_.supercap_enable = supercap_enable;
    }

    inline void SetSupcapPowerLimit(uint8_t power_limit) {
        tx_data_.power_limit = power_limit;
    }

    inline void SetSupcapChargePower(uint8_t charge_power) {
        tx_data_.charge_power = charge_power;
    }

protected:
    CanManageObject *can_manage_object_ = nullptr;

    uint32_t flag_ = 0;

    uint32_t pre_flag_ = 0;

    uint16_t can_rx_id_ = 0x100;

    uint16_t can_tx_id_ = 0x001;

    SupercapSendData tx_data_{};

    SupercapRecvData rx_data_{};

    void CanTxMessage();

    void DataProcess();

    void Task();

    static void TaskEntry(void *param);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

