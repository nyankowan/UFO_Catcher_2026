#include "main.h"
#include "servo.h"

#define SERVO_PERIOD_US 20000

// PC5にはタイマ出力がないため、TIM2のイベントでDMAがGPIOC->BSRRを書いてピンを動かす。
// CPU/割り込みを介さないので、他の割り込みの影響でパルス幅がずれない。
//   Update(周期開始)  -> DMA1 Stream1 ch3 -> BSRR = HIGH
//   CC1一致(幅経過)   -> DMA1 Stream5 ch3 -> BSRR = LOW
static const uint32_t bsrr_high = SERVO_PIN;
static const uint32_t bsrr_low  = (uint32_t)SERVO_PIN << 16;

void servo_set_pulse_us(uint16_t us)
{
  if (us < SERVO_MIN_US) us = SERVO_MIN_US;
  if (us > SERVO_MAX_US) us = SERVO_MAX_US;
  TIM2->CCR1 = us;  // プリロードにより次の周期から反映(周期の途中で変わってもパルスは壊れない)
}

static void dma_init(DMA_Stream_TypeDef *st, const uint32_t *src)
{
  st->CR &= ~DMA_SxCR_EN;
  while (st->CR & DMA_SxCR_EN) {}

  st->PAR  = (uint32_t)&SERVO_PORT->BSRR;
  st->M0AR = (uint32_t)src;
  st->NDTR = 1;
  st->CR = (3U << DMA_SxCR_CHSEL_Pos)   // ch3: TIM2
         | DMA_SxCR_PL_1                // 優先度high
         | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1  // 32bit
         | DMA_SxCR_CIRC
         | DMA_SxCR_DIR_0;              // memory -> peripheral
  st->CR |= DMA_SxCR_EN;
}

void servo_init(uint16_t us)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  HAL_GPIO_WritePin(SERVO_PORT, SERVO_PIN, GPIO_PIN_RESET);
  gpio.Pin = SERVO_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SERVO_PORT, &gpio);

  __HAL_RCC_DMA1_CLK_ENABLE();
  dma_init(DMA1_Stream1, &bsrr_high);  // TIM2_UP
  dma_init(DMA1_Stream5, &bsrr_low);   // TIM2_CH1

  // APB1タイマクロック = 60MHz -> 1us/tick (TIMのHALモジュールは未使用)
  __HAL_RCC_TIM2_CLK_ENABLE();
  TIM2->PSC = 60 - 1;
  TIM2->ARR = SERVO_PERIOD_US - 1;
  TIM2->CCMR1 = TIM_CCMR1_OC1PE;       // CCR1プリロード(出力ピンは使わず、比較一致だけ使う)
  TIM2->CCR1 = us < SERVO_MIN_US ? SERVO_MIN_US : (us > SERVO_MAX_US ? SERVO_MAX_US : us);
  TIM2->EGR = TIM_EGR_UG;              // プリロード反映(HIGHにもなる)
  TIM2->SR = 0;
  TIM2->DIER = TIM_DIER_UDE | TIM_DIER_CC1DE;
  TIM2->CR1 |= TIM_CR1_CEN;
}
