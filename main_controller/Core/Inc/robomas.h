#ifndef ROBOMAS_H
#define ROBOMAS_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#define ROBOMAS_NUM_MOTORS   4
#define ROBOMAS_ID_FB_BASE   0x200   // フィードバックは 0x201 + (モータID-1)
#define ROBOMAS_ANGLE_RES    8192    // 1回転あたりのエンコーダ値(ロータ軸)

typedef struct {
    uint16_t angle;       // 0..8191
    int16_t  rpm;
    int16_t  current;
    uint8_t  temp;
    int32_t  total;       // 最初のフィードバックを0とした多回転累積角度
    uint32_t last_rx_ms;
    bool     valid;
} robomas_fb_t;

extern robomas_fb_t robomas_fb[ROBOMAS_NUM_MOTORS];

void robomas_rx_init(CAN_HandleTypeDef *hcan);
void robomas_rx_poll(CAN_HandleTypeDef *hcan);
bool robomas_alive(int motor, uint32_t timeout_ms);

typedef struct {
    float kp, ki, kd;
    float i_max;     // 積分項の上限(出力単位)
    float out_max;
    float integ;
    float prev_err;
} pid_ctrl_t;

float pid_step(pid_ctrl_t *p, float err, float dt);
void  pid_reset(pid_ctrl_t *p);

#endif //ROBOMAS_H
