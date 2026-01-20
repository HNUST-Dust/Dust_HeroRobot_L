/**
 * @file dvc_PC_comm.h
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef PC_COMM_H
#define PC_COMM_H

/* Includes ------------------------------------------------------------------*/

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "ins_task.h"
#include "bsp_dwt.h"
#include "bsp_usb.h"
#include "crc.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief PcComm转换联合体
 * 
 */
union PcConv
{
    uint8_t b[4];
    float f;
};

/**
 * @brief PcComm自瞄模式
 * 
 */
enum PcAutoAimMode
{
    PC_AUTOAIM_MODE_IDIE = 0,
    PC_AUTOAIM_MODE_FOLLOW,
    PC_AUTOAIM_MODE_FIRE,
};

/**
 * @brief PcComm存活状态枚举
 * 
 */
enum PcAliveState
{
    PC_ALIVE_STATE_ENABLE = 0,
    PC_ALIVE_STATE_DISABLE,
};

#pragma pack(1)

/**
 * @brief PcComm自瞄发送结构体
 * 
 */
struct PCSendAutoAimData
{
    uint8_t head[2] = {'S','P'};

    uint8_t mode = 0;               // 0-空闲 1-自瞄不开火 2-自瞄开火

    float q[4];                     // 四元数姿态[w,x,y,z]

    struct
    {
        float ang;                  // yaw轴角度
        float vel;                  // yaw轴角速度
    } yaw;
    
    struct
    {
        float ang;                  // pitch轴角度
        float vel;                  // pitch轴角速度
    } pitch;
    
    struct
    {
        float speed;                // 子弹速度
        uint16_t count;             // 子弹累计发送次数
    } bullet;
    
    uint16_t crc16;                 // 校验位
};

/**
 * @brief PcComm自瞄接收结构体
 * 
 */
struct PCRecvAutoAimData
{
    uint8_t head[2] = {'S','P'};
    
    uint8_t mode = 0;           // 0-空闲 1-自瞄不开火 2-自瞄开火

    struct
    {
        float yaw_ang;          // yaw轴角度
        float yaw_vel;          // yaw轴角速度
        float yaw_acc;          // yaw轴角加速度
    } yaw;
    
    struct
    {
        float pitch_ang;        // pitch轴角度
        float pitch_vel;        // pitch轴角速度
        float pitch_acc;        // pitch轴角加速度
    } pitch;

    uint16_t crc16;             // 校验位
};

#pragma pack()

/**
 * @brief PcComm类
 * 
 */
class PcComm
{
public:
    // 发送自瞄数据
    PCSendAutoAimData send_autoaim_data = 
    {
        {'S','P'},
        0,
        {1,0,0,0},
        {0,0},
        {0,0},
        {0,0},
        0,
    };
    // 接收自瞄数据
    PCRecvAutoAimData recv_autoaim_data = 
    {
        {'S','P'},
        0,
        {0,0,0},
        {0,0,0},
        0,
    };
    void Init();

    void Task();

    void Send_Message();

    void RxCpltCallback();

    void UpdataAutoaimData();

private:
    uint32_t flag_ = 0;

    uint32_t pre_flag_ = 0;

    uint32_t alive_count_ = 0;

    PcAliveState pc_alive_state = PC_ALIVE_STATE_DISABLE;

    void ClearData();

    void DataProcess();

    void AlivePeriodElapsedCallback();

    // FreeRTOS 入口，静态函数
    static void TaskEntry(void *param);
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif
