#ifndef CAN_PACKET_H
#define CAN_PACKET_H

#include "struct_typedef.h"
#include "main.h"

#define IMU_PACKET_TIME_ID          0x100
#define IMU_PACKET_DATA0_ID         0x102
#define IMU_PACKET_DATA1_ID         0x103

#define AIMBOT_STATE_NODE_ID        0x106
#define AIMBOT_DATA_NODE_ID         0x108

#define GINBAL_REQUEST_STATE_ID     0x110

#define REFEREE_POWER_HEAT_NODE_0_ID                0x120
#define REFEREE_POWER_HEAT_NODE_1_ID                0x121
#define REFEREE_AMMO_SPEED_NODE_0_ID                0x122
#define REFEREE_AMMO_SPEED_NODE_1_ID                0x123
#define REFEREE_AMMO_SPEED_NODE_2_ID                0x124
#define REFEREE_AMMO_LIMIT_NODE_0_ID                0x125
#define REFEREE_AMMO_LIMIT_NODE_1_ID                0x126
#define REFEREE_AMMO_LIMIT_NODE_2_ID                0x127
#define REFEREE_SELF_STATE_NODE                     0x128

typedef __PACKED_STRUCT{
    uint8_t     AimbotRequest;
    int16_t     ChassisMoveXRequest;
    int16_t     ChassisMoveYRequest;
    uint8_t     ChassisStateRequest;
    uint8_t     GimbalState;
    uint8_t     Reserve;
} GimbalRequestState_t;

typedef __PACKED_STRUCT{
    uint16_t    ChassisVoltage;
    uint16_t    ChassisCurrent;
    fp32        ChassisPower;
    uint16_t    ChassisBufferEnergy;
    uint16_t    Shooter0Energy;
    uint16_t    Shooter1Energy;
    uint16_t    Shooter2Energy;
} RefereeChassisPowerShootHeat_t;

typedef __PACKED_STRUCT{
    uint32_t    TimeStamp;
    fp32        Quaternion[4];
} ImuPacketNormal_t;

typedef __PACKED_STRUCT{
    uint8_t     AimbotState;
    uint8_t     AimbotTarget;
} AimbotStateNoraml_t;

typedef __PACKED_STRUCT{
    int16_t     PitchRelativeAngle;
    int16_t     YawRelativeAngle;
    uint32_t    SystemTimer;
} AimbotCommandNoraml_t;





#endif
