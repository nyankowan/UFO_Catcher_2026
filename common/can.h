#ifndef CAN_H
#define CAN_H

#include "can_protocol.h"
HAL_StatusTypeDef stm_can_send(
    CAN_HandleTypeDef *hcan,
    const can_command_data_t *com
);
#endif //CAN_H