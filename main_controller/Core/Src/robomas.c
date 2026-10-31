#include "robomas.h"

robomas_fb_t robomas_fb[ROBOMAS_NUM_MOTORS];

void robomas_rx_init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef f = {0};
  f.FilterBank = 14;  // CAN2用(SlaveStartFilterBank=14以降)
  f.FilterMode = CAN_FILTERMODE_IDMASK;
  f.FilterScale = CAN_FILTERSCALE_32BIT;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation = ENABLE;
  f.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(hcan, &f) != HAL_OK) Error_Handler();

  // フィードバックは1kHz x 3台で来るのでFIFO(3段)はポーリングでは溢れる。受信割り込みで取る
  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();
}

void robomas_rx_poll(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef hdr;
  uint8_t d[8];
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, d) != HAL_OK) break;
    if (hdr.IDE != CAN_ID_STD || hdr.DLC < 7) continue;
    int idx = (int)hdr.StdId - ROBOMAS_ID_FB_BASE - 1;
    if (idx < 0 || idx >= ROBOMAS_NUM_MOTORS) continue;

    robomas_fb_t *m = &robomas_fb[idx];
    uint16_t angle = (uint16_t)((d[0] << 8) | d[1]);
    if (m->valid) {
      int32_t delta = (int32_t)angle - (int32_t)m->angle;
      if (delta >  ROBOMAS_ANGLE_RES / 2) delta -= ROBOMAS_ANGLE_RES;
      if (delta < -ROBOMAS_ANGLE_RES / 2) delta += ROBOMAS_ANGLE_RES;
      m->total += delta;
    } else {
      m->total = 0;
    }
    m->angle = angle;
    m->rpm = (int16_t)((d[2] << 8) | d[3]);
    m->current = (int16_t)((d[4] << 8) | d[5]);
    m->temp = d[6];
    m->last_rx_ms = HAL_GetTick();
    m->valid = true;
  }
}

// CAN2_RX0_IRQHandler(CubeMX生成)からHAL_CAN_IRQHandler経由で呼ばれる
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN2) {
    robomas_rx_poll(hcan);
  }
}

bool robomas_alive(int motor, uint32_t timeout_ms)
{
  return robomas_fb[motor].valid &&
         (HAL_GetTick() - robomas_fb[motor].last_rx_ms) <= timeout_ms;
}

static float clampf(float v, float lim)
{
  if (v >  lim) return  lim;
  if (v < -lim) return -lim;
  return v;
}

float pid_step(pid_ctrl_t *p, float err, float dt)
{
  p->integ = clampf(p->integ + p->ki * err * dt, p->i_max);
  float d = (err - p->prev_err) / dt;
  p->prev_err = err;
  return clampf(p->kp * err + p->integ + p->kd * d, p->out_max);
}

void pid_reset(pid_ctrl_t *p)
{
  p->integ = 0;
  p->prev_err = 0;
}
