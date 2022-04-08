#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

#include "main.h"

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;


typedef struct
{
    pid_type_def    pid_inside;
    pid_type_def    pid_outside;
    
    fp32            s_set;
    fp32            s_fdb;
    fp32            v_set;
    fp32            v_fdb;
    fp32            out;
} cascade_pid_t;

typedef struct{
    fp32    YawMotorAngle;
    fp32    YawMotorSpeed;
    fp32    PitchMotorAngle;
    fp32    PitchMotorSpeed;
} GimbalMotorMeasure_t;

typedef struct{
    fp32    RotorMotorSpeed;
    fp32    AmmoLeftMotorSpeed;
    fp32    AmmoRightMotorSpeed;
} ShootMotorMeasure_t;

typedef struct{
    fp32    YawAngle;
    fp32    PitchAngle;
    fp32    RollAngle;
    fp32    YawSpeed;
    fp32    PitchSpeed;
    fp32    RollSpeed;
} EulerSystemMeasure_t;

typedef struct{
    fp32 *ptr;
    uint32_t offset;
    uint32_t size;
} LoopFifoFp32_t;

#endif



