#ifndef REFEREE_CAN_H
#define REFEREE_CAN_H

#include "struct_typedef.h"
#include "CanPacket.h"

typedef struct
{
    RefereeChassisPowerShootHeat_t      Realtime;
    fp32                                Ammo0Speed;
    fp32                                Ammo1Speed;
    fp32                                Ammo2Speed;
    RefereeAmmoLimit_t                  Ammo0Limit;
    RefereeAmmoLimit_t                  Ammo1Limit;
    RefereeAmmoLimit_t                  Ammo2Limit;
    RefereeSelfState_t                  RobotState;
} RefereeInformation_t;

extern void RefereePowerHeatNode0InformationUpdate(uint8_t *data);
extern void RefereePowerHeatNode1InformationUpdate(uint8_t *data);
extern void RefereeAmmoSpeedNode0InformationUpdate(uint8_t *data);
extern void RefereeAmmoSpeedNode1InformationUpdate(uint8_t *data);
extern void RefereeAmmoSpeedNode2InformationUpdate(uint8_t *data);
extern void RefereeAmmoLimitNode0InformationUpdate(uint8_t *data);
extern void RefereeAmmoLimitNode1InformationUpdate(uint8_t *data);
extern void RefereeAmmoLimitNode2InformationUpdate(uint8_t *data);
extern void RefereeSelfStateNodeInformationUpdate(uint8_t *data);

extern void GetRefereeInformation(RefereeInformation_t *Inf);

#endif
