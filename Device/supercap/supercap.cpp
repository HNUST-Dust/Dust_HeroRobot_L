/**
 * @file supercap.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-11-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "supercap.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief Supercap初始化
 * 
 * @param hcan 
 * @param can_rx_id 
 * @param can_tx_id 
 */
void Supercap::Init(CAN_HandleTypeDef* hcan, uint16_t can_rx_id, uint16_t can_tx_id)
{
    if (hcan->Instance == CAN1)
    {
        can_manage_object_ = &g_can1_manage_object;
    }
    else if (hcan->Instance == CAN2)
    {
        can_manage_object_ = &g_can2_manage_object;
    }
    
    can_rx_id_ = can_rx_id;

    can_tx_id_ = can_tx_id;

    tx_data_.charge_power = 20;
    tx_data_.power_limit = 100;
    tx_data_.supercap_charge = SUPERCAP_STATUS_CHARGE;
    tx_data_.supercap_enable = SUPERCAP_STATUS_ENABLE;

    static const osThreadAttr_t kSupercapTaskAttr {
        .name = "supercap_task",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    // osThreadNew(Supercap::TaskEntry, this, &kSupercapTaskAttr);
}

/**
 * @brief 任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void Supercap::TaskEntry(void *argument)
{
    Supercap *self = static_cast<Supercap *>(argument);  // 还原 this 指针
    self->Task();  // 调用成员函数
}

/**
 * @brief SupercapCan发送信息函数
 * 
 */
void Supercap::CanTxMessage()
{
    static uint8_t can_tx_frame[8]{};

    can_tx_frame[0] = static_cast<uint8_t>(tx_data_.supercap_enable);
    can_tx_frame[1] = static_cast<uint8_t>(tx_data_.supercap_charge);
    can_tx_frame[2] = tx_data_.power_limit;
    can_tx_frame[3] = tx_data_.charge_power;

    can_send_data(can_manage_object_->can_handler, can_tx_id_, can_tx_frame, 8);
}

/**
 * @brief Supercap任务函数
 * 
 */
void Supercap::Task()
{
    Timer print(10);

    for (;;)
    {
        CanTxMessage();

        print.Tick([&]()
        {
            printf("%d, %d\n", rx_data_.chassis_power, rx_data_.supercap_power);
        });

        osDelay(10);
    }
}

/**
 * @brief SupercapCan通讯接收回调函数
 * 
 * @param rx_data 
 */
void Supercap::CanRxCpltCallback(uint8_t *rx_data)
{
    flag_ += 1;

    DataProcess();
}

/**
 * @brief Supercap数据处理函数
 * 
 */
void Supercap::DataProcess()
{
    uint8_t* temp_buffer = can_manage_object_->rx_buffer.data;

    rx_data_.ready_status       = static_cast<SuperCapReadyStatus>(temp_buffer[0]);
    rx_data_.supercap_status    = static_cast<SupercapStatus>(temp_buffer[1]);
    rx_data_.supercap_energy    = temp_buffer[2];
    rx_data_.chassis_power      = temp_buffer[3];
    rx_data_.bat_voltage        = static_cast<float>(temp_buffer[4] / 10.f);
    rx_data_.bat_power          = temp_buffer[5];
    rx_data_.supercap_power     = temp_buffer[6];
}


/************************ COPYRIGHT(C) HNUST-DUST **************************/
