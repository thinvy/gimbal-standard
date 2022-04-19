#include "InterruptService.h"
#include "main.h"
#include "Motor.h"
#include "Remote.h"
#include "RefereeCan.h"

#include "AimbotCan.h"
#include "CalculateThread.h"
#include "AttitudeThread.h"

#include "bsp_can.h"

#include "struct_typedef.h"
#include "CanPacket.h"

#include "Setting.h"



#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

RefereeChassisPowerShootHeat_t RefereeChassisPowerShootHeat;


void PitchMotorOfflineCounterUpdate(void);
void YawMotorOfflineCounterUpdate(void);
void RotorMotorOfflineCounterUpdate(void);
void AmmoLeftMotorMotorOfflineCounterUpdate(void);
void AmmoRightMotorMotorOfflineCounterUpdate(void);
void AimbotStateNodeOfflineCounterUpdate(void);
void AimbotDataNodeOfflineCounterUpdate(void);
void RefereePowerHeatNode0OfflineCounterUpdate(void);
void RefereePowerHeatNode1OfflineCounterUpdate(void);
void RefereeAmmoSpeedNode0OfflineCounterUpdate(void);
void RefereeAmmoSpeedNode1OfflineCounterUpdate(void);
void RefereeAmmoSpeedNode2OfflineCounterUpdate(void);
void RefereeAmmoLimitNode0OfflineCounterUpdate(void);
void RefereeAmmoLimitNode1OfflineCounterUpdate(void);
void RefereeAmmoLimitNode2OfflineCounterUpdate(void);
void RefereeSelfStateNodeOfflineCounterUpdate(void);
void RemoteOfflineCounterUpdate(void);


uint32_t RefereeInterpolationTimer = 0;

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case AIMBOT_STATE_NODE_ID:
        {
            AimbotStateNodeOfflineCounterUpdate();
            AimbotStateUpdate((AimbotStateNoraml_t *)rx_data);
            break;
        }
        case AIMBOT_DATA_NODE_ID:
        {
            AimbotDataNodeOfflineCounterUpdate();
            AimbotCommandUpdate((AimbotCommandNoraml_t *)rx_data);
            break;
        }
        
        
        case REFEREE_POWER_HEAT_NODE_0_ID:
        {
            RefereePowerHeatNode0OfflineCounterUpdate();
            RefereePowerHeatNode0InformationUpdate(rx_data);
            break;
        }
        case REFEREE_POWER_HEAT_NODE_1_ID:
        {
            RefereePowerHeatNode1OfflineCounterUpdate();
            RefereePowerHeatNode1InformationUpdate(rx_data);
            RefereeInterpolationTimer = 0;
            break;
        }
        case REFEREE_AMMO_SPEED_NODE_0_ID:
        {
            RefereeAmmoSpeedNode0OfflineCounterUpdate();
            RefereeAmmoSpeedNode0InformationUpdate(rx_data);
            break;
        }
        case REFEREE_AMMO_SPEED_NODE_1_ID:
        {
            RefereeAmmoSpeedNode1OfflineCounterUpdate();
            RefereeAmmoSpeedNode1InformationUpdate(rx_data);
            break;
        }
        case REFEREE_AMMO_SPEED_NODE_2_ID:
        {
            RefereeAmmoSpeedNode2OfflineCounterUpdate();
            RefereeAmmoSpeedNode2InformationUpdate(rx_data);
            break;
        }
        case REFEREE_AMMO_LIMIT_NODE_0_ID:
        {
            RefereeAmmoLimitNode0OfflineCounterUpdate();
            RefereeAmmoLimitNode0InformationUpdate(rx_data);
            break;
        }
        case REFEREE_AMMO_LIMIT_NODE_1_ID:
        {
            RefereeAmmoLimitNode1OfflineCounterUpdate();
            RefereeAmmoLimitNode1InformationUpdate(rx_data);
            break;
        }
        case REFEREE_AMMO_LIMIT_NODE_2_ID:
        {
            RefereeAmmoLimitNode2OfflineCounterUpdate();
            RefereeAmmoLimitNode2InformationUpdate(rx_data);
            break;
        }
        case REFEREE_SELF_STATE_NODE:
        {
            RefereeSelfStateNodeOfflineCounterUpdate();
            RefereeSelfStateNodeInformationUpdate(rx_data);
            break;
        }
        
        
        case YAW_MOTOR_ID:
        {
            YawMotorOfflineCounterUpdate();
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        }
        case PITCH_MOTOR_ID:
        {
            PitchMotorOfflineCounterUpdate();
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        }
        case ROTOR_MOTOR_ID:
        {
            RotorMotorOfflineCounterUpdate();
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        }
        case AMMO_LEFT_MOTOR_ID:
        {
            AmmoLeftMotorMotorOfflineCounterUpdate();
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        }
        case AMMO_RIGHT_MOTOR_ID:
        {
            AmmoRightMotorMotorOfflineCounterUpdate();
            MotorProcess(rx_header.StdId, hcan, rx_data);
            break;
        }

        default:
        {
            break;
        }
    }
}


//串口中断
void USART3_IRQHandler(void)
{
    if (huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if (USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(0);
                //记录数据接收时间
                RemoteOfflineCounterUpdate();
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(1);
                //记录数据接收时间
                RemoteOfflineCounterUpdate();
            }
        }
    }

}


/** 
  * @brief This function handles TIM3 global interrupt.
  */
uint32_t SystemTimer = 0;

uint32_t GetSystemTimer(void)
{
    return SystemTimer;
}


void GimbalMotorCommandSend(void);
void GimbalRequestStatePacketSend(void);
void GimbalImuPacketSend(void);

void CommuniteOfflineCounterUpdate(void);
void CommuniteOfflineStateUpdate(void);


void TimerTaskLoop1000Hz(void)
{
    SystemTimer++;
//    GimbalMotorCommandSend();
    CommuniteOfflineCounterUpdate();
    CommuniteOfflineStateUpdate();
    
    RefereeInterpolationTimer++;
    if ((RefereeInterpolationTimer % 10) == 0) {
        RefereeHeatInterpolation();
    }
    
}



void TimerTaskLoop500Hz(void)
{
    GimbalImuPacketSend();
}


void TimerTaskLoop100Hz(void)
{
    GimbalRequestStatePacketSend();
}






void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
    TimerTaskLoop1000Hz();
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
    TimerTaskLoop500Hz();
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
    TimerTaskLoop100Hz();
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}




GimbalOutput_t GimbalMotorOutput;
void GimbalMotorCommandSend(void)
{
    GetGimbalMotorOutput(&GimbalMotorOutput);
    GimbalMotorControl( GimbalMotorOutput.Yaw * YAW_MOTOR_DIRECTION,
                        GimbalMotorOutput.Pitch * PITCH_MOTOR_DIRECTION, 
                        GimbalMotorOutput.Rotor, 
                        GimbalMotorOutput.AmmoLeft, 
                        GimbalMotorOutput.AmmoRight
                        );
}

ImuPacketNormal_t ImuPacket;
void GimbalImuPacketSend(void)
{
    ImuPacket.TimeStamp = SystemTimer;
    GetCurrentQuaternion(ImuPacket.Quaternion);
    CanSendMessage(&COMMUNICATE_CANPORT, IMU_PACKET_TIME_ID, 4, (uint8_t *)&ImuPacket.TimeStamp);
    CanSendMessage(&COMMUNICATE_CANPORT, IMU_PACKET_DATA0_ID, 8, (uint8_t *)&ImuPacket.Quaternion[0]);
    CanSendMessage(&COMMUNICATE_CANPORT, IMU_PACKET_DATA1_ID, 8, (uint8_t *)&ImuPacket.Quaternion[2]);
}

GimbalRequestState_t RequestStatePacket;
void GimbalRequestStatePacketSend(void)
{
    GetGimbalRequestState(&RequestStatePacket);
    CanSendMessage(&COMMUNICATE_CANPORT, GINBAL_REQUEST_STATE_ID, 8, (uint8_t *)&RequestStatePacket);
}




OfflineCounter_t OfflineCounter;
OfflineMonitor_t OfflineMonitor;
void CommuniteOfflineCounterUpdate(void)
{
    OfflineCounter.PitchMotor++;
    OfflineCounter.YawMotor++;
    OfflineCounter.RotorMotor++;
    OfflineCounter.AmmoLeftMotor++;
    OfflineCounter.AmmoRightMotor++;
    OfflineCounter.AimbotStateNode++;
    OfflineCounter.AimbotDataNode++;
    OfflineCounter.RefereePowerHeatNode0++;
    OfflineCounter.RefereePowerHeatNode1++;
    OfflineCounter.RefereeAmmoSpeedNode0++;
    OfflineCounter.RefereeAmmoSpeedNode1++;
    OfflineCounter.RefereeAmmoSpeedNode2++;
    OfflineCounter.RefereeAmmoLimitNode0++;
    OfflineCounter.RefereeAmmoLimitNode1++;
    OfflineCounter.RefereeAmmoLimitNode2++;
    OfflineCounter.RefereeSelfStateNode++;
    OfflineCounter.Remote++;
}

void CommuniteOfflineStateUpdate(void)
{
    // Motor
    if (OfflineCounter.PitchMotor > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.PitchMotor = 1;
    }
    else{
        OfflineMonitor.PitchMotor = 0;
    }
    if (OfflineCounter.YawMotor > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.YawMotor = 1;
    }
    else{
        OfflineMonitor.YawMotor = 0;
    }
    if (OfflineCounter.RotorMotor > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.RotorMotor = 1;
    }
    else{
        OfflineMonitor.RotorMotor = 0;
    }
    if (OfflineCounter.AmmoLeftMotor > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.AmmoLeftMotor = 1;
    }
    else{
        OfflineMonitor.AmmoLeftMotor = 0;
    }
    if (OfflineCounter.AmmoRightMotor > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.AmmoRightMotor = 1;
    }
    else{
        OfflineMonitor.AmmoRightMotor = 0;
    }
    
    // CAN Bus Node
    if (OfflineCounter.AimbotStateNode > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.AimbotStateNode = 1;
    }
    else{
        OfflineMonitor.AimbotStateNode = 0;
    }
    if (OfflineCounter.AimbotDataNode > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.AimbotDataNode = 1;
    }
    else{
        OfflineMonitor.AimbotDataNode = 0;
    }
    if (OfflineCounter.RefereePowerHeatNode0 > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereePowerHeatNode0 = 1;
    }
    else{
        OfflineMonitor.RefereePowerHeatNode0 = 0;
    }
    if (OfflineCounter.RefereePowerHeatNode1 > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereePowerHeatNode1 = 1;
    }
    else{
        OfflineMonitor.RefereePowerHeatNode1 = 0;
    }
    if (OfflineCounter.RefereeAmmoSpeedNode0 > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereeAmmoSpeedNode0 = 1;
    }
    else{
        OfflineMonitor.RefereeAmmoSpeedNode0 = 0;
    }
    if (OfflineCounter.RefereeAmmoSpeedNode1 > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereeAmmoSpeedNode1 = 1;
    }
    else{
        OfflineMonitor.RefereeAmmoSpeedNode1 = 0;
    }
    if (OfflineCounter.RefereeAmmoSpeedNode2 > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereeAmmoSpeedNode2 = 1;
    }
    else{
        OfflineMonitor.RefereeAmmoSpeedNode2 = 0;
    }
    if (OfflineCounter.RefereeAmmoLimitNode0 > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereeAmmoLimitNode0 = 1;
    }
    else{
        OfflineMonitor.RefereeAmmoLimitNode0 = 0;
    }
    if (OfflineCounter.RefereeAmmoLimitNode1 > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereeAmmoLimitNode1 = 1;
    }
    else{
        OfflineMonitor.RefereeAmmoLimitNode1 = 0;
    }
    if (OfflineCounter.RefereeAmmoLimitNode2 > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereeAmmoLimitNode2 = 1;
    }
    else{
        OfflineMonitor.RefereeAmmoLimitNode2 = 0;
    }
    if (OfflineCounter.RefereeSelfStateNode > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.RefereeSelfStateNode = 1;
    }
    else{
        OfflineMonitor.RefereeSelfStateNode = 0;
    }

    // Remote
    if (OfflineCounter.Remote > MOTOR_OFFLINE_TIMEMAX){
        OfflineMonitor.Remote = 1;
    }
    else{
        OfflineMonitor.Remote = 0;
    }
}

void DeviceOfflineMonitorUpdate(OfflineMonitor_t *Monitor)
{
    memcpy(Monitor, &OfflineMonitor, sizeof(OfflineMonitor_t));
}


void PitchMotorOfflineCounterUpdate(void)
{
    OfflineCounter.PitchMotor = 0;
}

void YawMotorOfflineCounterUpdate(void)
{
    OfflineCounter.YawMotor = 0;
}

void RotorMotorOfflineCounterUpdate(void)
{
    OfflineCounter.RotorMotor = 0;
}

void AmmoLeftMotorMotorOfflineCounterUpdate(void)
{
    OfflineCounter.AmmoLeftMotor = 0;
}

void AmmoRightMotorMotorOfflineCounterUpdate(void)
{
    OfflineCounter.AmmoRightMotor = 0;
}

void AimbotStateNodeOfflineCounterUpdate(void)
{
    OfflineCounter.AimbotStateNode = 0;
}

void AimbotDataNodeOfflineCounterUpdate(void)
{
    OfflineCounter.AimbotDataNode = 0;
}

void RefereePowerHeatNode0OfflineCounterUpdate(void)
{
    OfflineCounter.RefereePowerHeatNode0 = 0;
}

void RefereePowerHeatNode1OfflineCounterUpdate(void)
{
    OfflineCounter.RefereePowerHeatNode1 = 0;
}

void RefereeAmmoSpeedNode0OfflineCounterUpdate(void)
{
    OfflineCounter.RefereeAmmoSpeedNode0 = 0;
}

void RefereeAmmoSpeedNode1OfflineCounterUpdate(void)
{
    OfflineCounter.RefereeAmmoSpeedNode1 = 0;
}

void RefereeAmmoSpeedNode2OfflineCounterUpdate(void)
{
    OfflineCounter.RefereeAmmoSpeedNode2 = 0;
}

void RefereeAmmoLimitNode0OfflineCounterUpdate(void)
{
    OfflineCounter.RefereeAmmoLimitNode0 = 0;
}

void RefereeAmmoLimitNode1OfflineCounterUpdate(void)
{
    OfflineCounter.RefereeAmmoLimitNode1 = 0;
}

void RefereeAmmoLimitNode2OfflineCounterUpdate(void)
{
    OfflineCounter.RefereeAmmoLimitNode2 = 0;
}

void RefereeSelfStateNodeOfflineCounterUpdate(void)
{
    OfflineCounter.RefereeSelfStateNode = 0;
}

void RemoteOfflineCounterUpdate(void)
{
    OfflineCounter.Remote = 0;
}
