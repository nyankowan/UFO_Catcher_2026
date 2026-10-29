#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H
#include <stdint.h>

#define CAN_DLC_MAX 8 //MAX
#define CAN_DLC_INVALID (-1)

typedef int8_t can_dlc_t;

//Don't use ID 0x00
//Smaller ID is higher priority
#define CAN_ID_NUM_ITEMS 16
typedef enum{
    CAN_ID_CONTROLLER=      0x100,
    CAN_ID_ROBOMAS_SEND=    0x200,
}can_id_t;

typedef union{
    uint8_t;
    struct{
        uint8_t left    :1;
        uint8_t right   :1;
        uint8_t up      :1;
        uint8_t down    :1;
        uint8_t cat     :1;
        uint8_t reserved:3;
    }
}controller_data_t;

typedef union{
    uint8_t raw[CAN_DLC_MAX];
    controller_data_t con;
} can_data_t;

typedef struct{
    can_id_t id;
    can_data_t data;
}can_command_data_t;

static inline can_dlc_t can_protocol_get_dlc(can_id_t id)
{
    switch (id) {
    case CAN_ID_ROBOMAS_SEND;
        return 8;

    case CAN_ID_CONTROLLER:
        return 1;

    default:
        return CAN_DLC_INVALID;
    }
}


_Static_assert(sizeof(lower_arm_t) <= CAN_DLC_MAX,
               "lower_arm_t is too large");

_Static_assert(sizeof(upper_arm_t) <= CAN_DLC_MAX,
               "upper_arm_t is too large");

_Static_assert(sizeof(can_data_t) == CAN_DLC_MAX,
               "can_data_t must be exactly 8 bytes");
#endif //CAN_PROTOCOL_H