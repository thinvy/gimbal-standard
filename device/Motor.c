/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "Motor.h"
#include "bsp_can.h"

#include "Setting.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
static uint32_t             send_mail_box;
static CAN_TxHeaderTypeDef  can_tx_message;
static uint8_t              MotorSendBuffer[16];
static uint8_t              MotorBusPosition[8] = {0};


void GimbalMotorControl(int16_t YawMotor, int16_t PitchMotor, int16_t RotorMotor, int16_t AmmoLeftMotor, int16_t AmmoRightMotor);
void MotorProcess(uint32_t MotorID, CAN_HandleTypeDef *hcan, uint8_t* message);
void GimbalMotorMeasureUpdate(GimbalMotorMeasure_t* Gimbal);
void ShootMotorMeasureUpdate(ShootMotorMeasure_t* Shoot);


void GimbalMotorControl(int16_t YawMotor, int16_t PitchMotor, int16_t RotorMotor, int16_t AmmoLeftMotor, int16_t AmmoRightMotor)
{
    MotorSendBuffer[(YAW_MOTOR_ID - 0x201)*2]               =   YawMotor >> 8;
    MotorSendBuffer[(YAW_MOTOR_ID - 0x201)*2 + 1]           =   YawMotor;
    MotorSendBuffer[(PITCH_MOTOR_ID - 0x201)*2]             =   PitchMotor >> 8;
    MotorSendBuffer[(PITCH_MOTOR_ID - 0x201)*2 + 1]         =   PitchMotor;
    MotorSendBuffer[(ROTOR_MOTOR_ID - 0x201)*2]             =   RotorMotor >> 8;
    MotorSendBuffer[(ROTOR_MOTOR_ID - 0x201)*2 + 1]         =   RotorMotor;
    MotorSendBuffer[(AMMO_LEFT_MOTOR_ID - 0x201)*2]         =   AmmoLeftMotor >> 8;
    MotorSendBuffer[(AMMO_LEFT_MOTOR_ID - 0x201)*2 + 1]     =   AmmoLeftMotor;
    MotorSendBuffer[(AMMO_RIGHT_MOTOR_ID - 0x201)*2]        =   AmmoRightMotor >> 8;
    MotorSendBuffer[(AMMO_RIGHT_MOTOR_ID - 0x201)*2 + 1]    =   AmmoRightMotor;
#ifdef PITCH_AUX
    MotorSendBuffer[(PITCH_AUX_MOTOR_ID - 0x201)*2]         =   (-PitchMotor) >> 8;
    MotorSendBuffer[(PITCH_AUX_MOTOR_ID - 0x201)*2 + 1]     =   (-PitchMotor);
#endif
    
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    
    
    can_tx_message.StdId = 0x200;
    if ((MotorBusPosition[0]|MotorBusPosition[1]|MotorBusPosition[2]|MotorBusPosition[3])&0x01){
        HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, MotorSendBuffer, &send_mail_box);
    }
    if ((MotorBusPosition[0]|MotorBusPosition[1]|MotorBusPosition[2]|MotorBusPosition[3])&0x02){
        HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, MotorSendBuffer, &send_mail_box);
    }
    
    can_tx_message.StdId = 0x1FF;
    if ((MotorBusPosition[4]|MotorBusPosition[5]|MotorBusPosition[6]|MotorBusPosition[7])&0x01){
        HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, (MotorSendBuffer + 8), &send_mail_box);
    }
    if ((MotorBusPosition[4]|MotorBusPosition[5]|MotorBusPosition[6]|MotorBusPosition[7])&0x02){
        HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, (MotorSendBuffer + 8), &send_mail_box);
    }
}





static motor_measure_t YawMotorMeasure;
static motor_measure_t PitchMotorMeasure;
static motor_measure_t RotorMotorMeasure;
static motor_measure_t AmmoLeftMotorMeasure;
static motor_measure_t AmmoRightMotorMeasure;

//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

void MotorProcess(uint32_t MotorID, CAN_HandleTypeDef *hcan, uint8_t* message)
{
    switch (MotorID){
        case YAW_MOTOR_ID:
            get_motor_measure(&YawMotorMeasure, message);
            break;
        case PITCH_MOTOR_ID:
            get_motor_measure(&PitchMotorMeasure, message);
            break;
        case ROTOR_MOTOR_ID:
            get_motor_measure(&RotorMotorMeasure, message);
            break;
        case AMMO_LEFT_MOTOR_ID:
            get_motor_measure(&AmmoLeftMotorMeasure, message);
            break;
        case AMMO_RIGHT_MOTOR_ID:
            get_motor_measure(&AmmoRightMotorMeasure, message);
            break;
        default:
            break;
    }
    
    if (hcan == &hcan1){
        MotorBusPosition[MotorID - 0x201] = 1;
    }
    else{
        MotorBusPosition[MotorID - 0x201] = 2;
    }
}


void GimbalMotorMeasureUpdate(GimbalMotorMeasure_t* Gimbal)
{
    Gimbal->YawMotorAngle = YawMotorMeasure.ecd / 8192.0f * 360.0f - 180.0f;
    Gimbal->YawMotorSpeed = YawMotorMeasure.speed_rpm;
    Gimbal->PitchMotorAngle = PitchMotorMeasure.ecd / 8192.0f * 360.0f - 180.0f;
    Gimbal->PitchMotorSpeed = PitchMotorMeasure.speed_rpm;
}

void ShootMotorMeasureUpdate(ShootMotorMeasure_t* Shoot)
{
    Shoot->RotorMotorSpeed = RotorMotorMeasure.speed_rpm;
    Shoot->AmmoLeftMotorSpeed = AmmoLeftMotorMeasure.speed_rpm;
    Shoot->AmmoRightMotorSpeed = AmmoRightMotorMeasure.speed_rpm;
}











