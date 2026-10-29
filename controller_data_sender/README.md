# controller_data_sender
コントローラ（上下左右，キャッチ）のデータをCANバスに送信する．
## settings
### Pin

| 用途      | STM32ピン | CubeMX設定 | Nucleo端子 |
| ------- | ------- | -------- | -------- |
| LEFT    | PB4     | TIM3_CH1 | D12      |
| RIGHT   | PB5     | TIM3_CH2 | D11      |
| UP      | PB0     | TIM3_CH3 | D3       |
| DOWN    | PB1     | TIM3_CH4 | D6       |
| CATCH   | PA8     | TIM1_CH1 | D9       |
| CAN RX  | PA11    | CAN_RX   | D10      |
| CAN TX  | PA12    | CAN_TX   | D2       |
| STATUS_LED|PB3    |GPIO_Output| D13     |

### CAN
Baud rate 100 bit/sec
```text
Prescaler           1
Time Quanta 1       7
Time Quanta 2       2
ReSynchro JW        1
```
