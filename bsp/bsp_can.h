#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern void can_filter_init(void);

extern void CanSendMessage(CAN_HandleTypeDef *hcan, uint32_t id, uint32_t dlc, uint8_t *message);

#endif