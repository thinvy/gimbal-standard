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

#ifndef __MOTOR_H
#define __MOTOR_H

#include "struct_typedef.h"
#include "main.h"

//rm motor data
typedef struct{
    uint16_t    ecd;
    int16_t     speed_rpm;
    int16_t     given_current;
    uint8_t     temperate;
    int16_t     last_ecd;
} motor_measure_t;

extern void GimbalMotorControl(int16_t YawMotor, int16_t PitchMotor, int16_t RotorMotor, int16_t AmmoLeftMotor, int16_t AmmoRightMotor);
extern void MotorProcess(uint32_t MotorID, CAN_HandleTypeDef *hcan, uint8_t* message);
extern void GimbalMotorMeasureUpdate(GimbalMotorMeasure_t* Gimbal);
extern void ShootMotorMeasureUpdate(ShootMotorMeasure_t* Shoot);


#endif
