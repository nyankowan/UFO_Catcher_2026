#include "main.h"
#include "debug.h"

#define TX_BUF_SIZE 512  // 2のべき乗

static volatile uint8_t  tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head;  // 書き込み位置(main)
static volatile uint16_t tx_tail;  // 読み出し位置(割り込み)

void debug_init(void)
{
  // PA2/PA3はMX_GPIO_InitでAF7(USART2)に設定済み
  __HAL_RCC_USART2_CLK_ENABLE();
  USART2->BRR = 30000000U / 115200U;  // APB1=30MHz, OVER8=0
  USART2->CR1 = USART_CR1_UE | USART_CR1_TE;
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void debug_write(const char *buf, size_t len)
{
  for (size_t i = 0; i < len; i++) {
    uint16_t next = (tx_head + 1) & (TX_BUF_SIZE - 1);
    if (next == tx_tail) break;  // 満杯
    tx_buf[tx_head] = (uint8_t)buf[i];
    tx_head = next;
  }
  USART2->CR1 |= USART_CR1_TXEIE;
}

void USART2_IRQHandler(void)
{
  if ((USART2->CR1 & USART_CR1_TXEIE) && (USART2->SR & USART_SR_TXE)) {
    if (tx_tail == tx_head) {
      USART2->CR1 &= ~USART_CR1_TXEIE;
    } else {
      USART2->DR = tx_buf[tx_tail];
      tx_tail = (tx_tail + 1) & (TX_BUF_SIZE - 1);
    }
  }
}
