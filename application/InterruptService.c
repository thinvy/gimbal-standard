#include "InterruptService.h"
#include "main.h"
#include "Motor.h"
#include "Remote.h"

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
extern TIM_HandleTypeDef htim7;


void GimbalCanCommand(void);
void GimbalRequestStateSend(void);
void GimbalImuPacketSend(void);

RefereeChassisPowerShootHeat_t RefereeChassisPowerShootHeat;


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
        case AIMBOT_STATE_NORMAL_ID:
        {
            AimbotStateUpdate((AimbotStateNoraml_t *)rx_data);
            break;
        }
        
        case AIMBOT_COMMAND_NORMAL_ID:
        {
            AimbotCommandUpdate((AimbotCommandNoraml_t *)rx_data);
            break;
        }
        
        
        case REFEREE_CHASSIS_POWER_SHOOT_HEAT_0_ID:
        {
            memcpy(&RefereeChassisPowerShootHeat, rx_data, 8);
            break;
        }
        
        case REFEREE_CHASSIS_POWER_SHOOT_HEAT_1_ID:
        {
            memcpy(&RefereeChassisPowerShootHeat.ChassisVoltage, rx_data, 8);
            break;
        }
        
        
        case YAW_MOTOR_ID:
        case PITCH_MOTOR_ID:
        case ROTOR_MOTOR_ID:
        case AMMO_LEFT_MOTOR_ID:
        case AMMO_RIGHT_MOTOR_ID:
        {
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
//                detect_hook(DBUS_TOE);
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
//                detect_hook(DBUS_TOE);
            }
        }
    }

}


/** 
  * @brief This function handles TIM3 global interrupt.
  */
uint32_t SystemTimer = 0;
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
    SystemTimer++;
    GimbalCanCommand();
    GimbalRequestStateSend();
    GimbalImuPacketSend();
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

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}



GimbalOutput_t GimbalOutput;
void GimbalCanCommand(void)
{
    GetGimbalMotorOutput(&GimbalOutput);

    GimbalMotorControl( GimbalOutput.Yaw * YAW_MOTOR_DIRECTION,
                        GimbalOutput.Pitch * PITCH_MOTOR_DIRECTION, 
                        GimbalOutput.Rotor, 
                        GimbalOutput.AmmoLeft, 
                        GimbalOutput.AmmoRight
                        );
}


GimbalRequestState_t GimbalRequestState;
void GimbalRequestStateSend(void)
{
    GetGimbalRequestState(&GimbalRequestState);
    CanSendMessage(&COMMUNICATE_CANPORT, GINBAL_REQUEST_STATE_ID, 8, (uint8_t *)&GimbalRequestState);
}



ImuPacketNormal_t ImuPacket;
void GimbalImuPacketSend(void)
{
    ImuPacket.TimeStamp = SystemTimer;
    GetCurrentQuaternion(ImuPacket.Quaternion);
    CanSendMessage(&COMMUNICATE_CANPORT, IMU_PACKET_TIME_ID, 8, (uint8_t *)&ImuPacket.TimeStamp);
    CanSendMessage(&COMMUNICATE_CANPORT, IMU_PACKET_DATA0_ID, 8, (uint8_t *)&ImuPacket.Quaternion[0]);
    CanSendMessage(&COMMUNICATE_CANPORT, IMU_PACKET_DATA1_ID, 8, (uint8_t *)&ImuPacket.Quaternion[2]);
}

