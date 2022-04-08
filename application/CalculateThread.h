#ifndef __CalculateThread_H
#define __CalculateThread_H

#include "struct_typedef.h"
#include "CanPacket.h"

typedef struct{
    fp32    Yaw;
    fp32    Pitch;
    fp32    Rotor;
    fp32    AmmoLeft;
    fp32    AmmoRight;
} GimbalCommand_t;

typedef struct{
    int16_t    Yaw;
    int16_t    Pitch;
    int16_t    Rotor;
    int16_t    AmmoLeft;
    int16_t    AmmoRight;
} GimbalOutput_t;

typedef struct{
    cascade_pid_t   Yaw;
    cascade_pid_t   Pitch;
    pid_type_def    Rotor;
    pid_type_def    AmmoLeft;
    pid_type_def    AmmoRight;
} GimbalPID_t;

typedef enum{
    GM_NO_FORCE         =   0x00,
    GM_INIT                     ,
    GM_TEST                     ,
    GM_MATCH                    ,
} GimbalStateMachine_e;

typedef enum{
    GM_NO_CONTROL       =   0x00,
    GM_RESET_POSITION           ,
    GM_MANUAL_OPERATE           ,
    GM_AIMBOT_OPERATE           ,
    GM_AIMBOT_RUNES             ,
} GimbalControlMode_e;

typedef enum{
    GM_FIRE_UNABLE      =   0x00,
    GM_FIRE_READY               ,
    GM_FIRE_BUSY                ,
    GM_FIRE_COOLING             ,
    GM_FIRE_LAGGING             ,
} GimbalFireMode_e;

typedef struct{
    ShootMotorMeasure_t     ShootMotor;
    GimbalMotorMeasure_t    GimbalMotor;
} MotorMeasure_t;

typedef struct{
    uint8_t     Camp;
    uint8_t     MaxSpeed;
    fp32        CurrentSpeed;
    uint16_t    MaxHeat;
    uint16_t    CoolingHeat;
    uint16_t    CurrentHeat;
} RefereeMeasure_t;

typedef struct{
    fp32                    YawAddress[64];
    fp32                    PitchAddress[64];
    LoopFifoFp32_t          YawLoopPointer;
    LoopFifoFp32_t          PitchLoopPointer;
} ImuBuffer_t;

typedef struct{
    EulerSystemMeasure_t    Imu;                    //  imu的数据反馈
    MotorMeasure_t          MotorMeasure;           //  电机编码器的数据反馈
    GimbalCommand_t         Command;                //  云台角度控制指令
    GimbalPID_t             Pid;                    //  云台PID结构体
    GimbalOutput_t          Output;                 //  云台电机输出
    GimbalStateMachine_e    StateMachine;           //  云台状态机
    GimbalControlMode_e     ControlMode;            //  云台控制模式
    GimbalFireMode_e        FireMode;               //  云台开火模式
    RefereeMeasure_t        Referee;                //  裁判系统数据
    ImuBuffer_t             ImuBuffer;
} Gimbal_t;


typedef enum{
    CHASSIS_NO_FORCE    =   0x00,
    CHASSIS_NO_MOVE             ,
    CHASSIS_FOLLOW              ,
    CHASSIS_ROTATE              ,
} ChassisState_e;

typedef enum{
    CHASSIS_NORMAL_SPEED=   0x00,
    CHASSIS_LOW_SPEED           ,
    CHASSIS_FAST_SPEED          ,
} ChassisSpeed_e;

typedef struct{
    ChassisState_e          ChassisState;
    ChassisSpeed_e          ChassisSpeed;
    fp32                    ChassisCommandX;
    fp32                    ChassisCommandY;
} Chassis_t;


extern void CalculateThread(void const * pvParameters);

extern void GetGimbalMotorOutput(GimbalOutput_t *out);
extern void GetGimbalRequestState(GimbalRequestState_t *RequestState);


#endif



