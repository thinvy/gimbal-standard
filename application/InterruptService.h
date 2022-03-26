#ifndef INTERUPT_SERVICE_H
#define INTERUPT_SERVICE_H
#include "struct_typedef.h"

#define DEVICE_OFFLINE          0x01
#define DEVICE_ONLINE           0x00


typedef struct
{
    // Motor
    uint32_t PitchMotor;
    uint32_t YawMotor;
    uint32_t RotorMotor;
    uint32_t AmmoLeftMotor;
    uint32_t AmmoRightMotor;
    // CAN Bus Node
    uint32_t AimbotStateNode;
    uint32_t AimbotDataNode;
    uint32_t RefereePowerHeatNode0;
    uint32_t RefereePowerHeatNode1;
    uint32_t RefereeAmmoSpeedNode0;
    uint32_t RefereeAmmoSpeedNode1;
    uint32_t RefereeAmmoSpeedNode2;
    uint32_t RefereeAmmoLimitNode0;
    uint32_t RefereeAmmoLimitNode1;
    uint32_t RefereeAmmoLimitNode2;
    uint32_t RefereeSelfStateNode;
    // Remote
    uint32_t Remote;
} OfflineCounter_t;


typedef struct
{
    // Motor
    uint8_t PitchMotor;
    uint8_t YawMotor;
    uint8_t RotorMotor;
    uint8_t AmmoLeftMotor;
    uint8_t AmmoRightMotor;
    // CAN Bus Node
    uint8_t AimbotStateNode;
    uint8_t AimbotDataNode;
    uint8_t RefereePowerHeatNode0;
    uint8_t RefereePowerHeatNode1;
    uint8_t RefereeAmmoSpeedNode0;
    uint8_t RefereeAmmoSpeedNode1;
    uint8_t RefereeAmmoSpeedNode2;
    uint8_t RefereeAmmoLimitNode0;
    uint8_t RefereeAmmoLimitNode1;
    uint8_t RefereeAmmoLimitNode2;
    uint8_t RefereeSelfStateNode;
    // Remote
    uint8_t Remote;
} OfflineMonitor_t;






extern void DeviceOfflineMonitorUpdate(OfflineMonitor_t *Monitor);
extern uint32_t GetSystemTimer(void);
#endif
