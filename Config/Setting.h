#ifndef SETTING_H
#define SETTING_H


//#define HERO1
//#define INFANTRY3
#define INFANTRY4
//#define INFANTRY5
//#define SENTRY7






#ifdef INFANTRY4
// 参数配置文件
#define PARAMETER_FILE "Infantry4Parameter.h"
// imu安装方向
#define IMU_DIRECTION_zryx_XYZ
// 电机ID分配
#define YAW_MOTOR_ID                0x205
#define PITCH_MOTOR_ID              0x206
#define ROTOR_MOTOR_ID              0x203
#define AMMO_LEFT_MOTOR_ID          0x202
#define AMMO_RIGHT_MOTOR_ID         0x201
// 电机安装方向
// 云台电机正向运动方向和云台姿态（控制）坐标系同向为1，反向为-1
// 拨盘电机正向运动方向和弹丸进入枪管方向同向为1，反向为-1
// 摩擦轮电机正向运动方向和弹道同向为1，反向为-1
#define YAW_MOTOR_DIRECTION         1
#define PITCH_MOTOR_DIRECTION       1
#define ROTOR_MOTOR_DIRECTION       -1
#define AMMO_LEFT_MOTOR_DIRECTION   -1
#define AMMO_RIGHT_MOTOR_DIRECTION  1
// 云台YAW轴零点和俯仰限幅
#define YAW_ZERO_ECDANGLE           -94.9658203f
#define PITCH_MIN_ANGLE             -22.8f
#define PITCH_MAX_ANGLE             18.0f
// 默认摩擦轮速度
#define DEFAULT_AMMOL_PID           AMMO_LEFT_SPEED_15MS
#define DEFAULT_AMMOR_PID           AMMO_RIGHT_SPEED_15MS
#define DEFAULT_AMMO_SPEEDSET       AMMO_SPEEDSET_15MS
// 通信can总线位置
#define COMMUNICATE_CANPORT         hcan1
#endif


#ifdef SENTRY7
// 参数配置文件
#define PARAMETER_FILE "Sentry7Parameter.h"
// imu安装方向
#define IMU_DIRECTION_rxryz_XYZ
// 电机ID分配
#define YAW_MOTOR_ID                0x205
#define PITCH_MOTOR_ID              0x206
#define ROTOR_MOTOR_ID              0x203
#define AMMO_LEFT_MOTOR_ID          0x201
#define AMMO_RIGHT_MOTOR_ID         0x202
// 电机安装方向
// 云台电机正向运动方向和云台姿态（控制）坐标系同向为1，反向为-1
// 拨盘电机正向运动方向和弹丸进入枪管方向同向为1，反向为-1
// 摩擦轮电机正向运动方向和弹道同向为1，反向为-1
#define YAW_MOTOR_DIRECTION         -1
#define PITCH_MOTOR_DIRECTION       -1
#define ROTOR_MOTOR_DIRECTION       1
#define AMMO_LEFT_MOTOR_DIRECTION   -1
#define AMMO_RIGHT_MOTOR_DIRECTION  1
// 云台YAW轴零点和俯仰限幅
#define YAW_ZERO_ECDANGLE           0.0f
#define PITCH_MIN_ANGLE             -8.8f
#define PITCH_MAX_ANGLE             28.0f
// 默认摩擦轮速度
#define DEFAULT_AMMOL_PID           AMMO_LEFT_SPEED_15MS
#define DEFAULT_AMMOR_PID           AMMO_RIGHT_SPEED_15MS
#define DEFAULT_AMMO_SPEEDSET       AMMO_SPEEDSET_15MS
// 通信can总线位置
#define COMMUNICATE_CANPORT         hcan2
#endif




//#define IMU_DIRECTION_xyz_XYZ
//#define IMU_DIRECTION_yrxz_XYZ
//#define IMU_DIRECTION_rxryz_XYZ
//#define IMU_DIRECTION_ryxz_XYZ

//#define IMU_DIRECTION_zryx_XYZ
//#define IMU_DIRECTION_yzx_XYZ
//#define IMU_DIRECTION_rzyx_XYZ
//#define IMU_DIRECTION_ryrzx_XYZ




#endif

