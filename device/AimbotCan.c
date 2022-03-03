#include "AimbotCan.h"
#include <string.h>

AimbotCommand_t AimbotCommand;

void AimbotCommandUpdate(AimbotCommandNoraml_t *AimbotCommandNoraml)
{
    AimbotCommand.PitchRelativeAngle = AimbotCommandNoraml->PitchRelativeAngle / 32768.0f * 180.0f;
    AimbotCommand.YawRelativeAngle = AimbotCommandNoraml->YawRelativeAngle / 32768.0f * 180.0f;
    AimbotCommand.CommandTimer = AimbotCommandNoraml->SystemTimer;
}

void AimbotStateUpdate(AimbotStateNoraml_t *AimbotStateNoraml)
{
    AimbotCommand.State = AimbotStateNoraml->AimbotState;
    AimbotCommand.Target = AimbotStateNoraml->AimbotTarget;
}

void GetAimbotCommand(AimbotCommand_t *Aimbot)
{
    memcpy(Aimbot, &AimbotCommand, sizeof(AimbotCommand_t));
}