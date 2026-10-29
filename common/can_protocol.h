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
    uint8_t raw;
    struct{
        uint8_t left    :1;
        uint8_t right   :1;
        uint8_t up      :1;
        uint8_t down    :1;
        uint8_t cat     :1;
        uint8_t reserved:3;
    };
}controller_data_t;

//Robomas (C610/C620) current command: motor1..4 as big-endian int16
typedef struct{
    int16_t current[4];
}robomas_send_t;

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
    case CAN_ID_ROBOMAS_SEND:
        return 8;

    case CAN_ID_CONTROLLER:
        return 1;

    default:
        return CAN_DLC_INVALID;
    }
}

static inline void can_protocol_pack_robomas(can_data_t *d, const int16_t current[4])
{
    for (int i = 0; i < 4; i++) {
        d->raw[i * 2]     = (uint8_t)((uint16_t)current[i] >> 8);
        d->raw[i * 2 + 1] = (uint8_t)((uint16_t)current[i] & 0xFF);
    }
}

_Static_assert(sizeof(controller_data_t) == 1,
               "controller_data_t must be 1 byte");

_Static_assert(sizeof(can_data_t) == CAN_DLC_MAX,
               "can_data_t must be exactly 8 bytes");

#endif //CAN_PROTOCOL_H