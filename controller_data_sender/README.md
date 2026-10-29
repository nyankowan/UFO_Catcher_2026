# controller_data_sender
コントローラ（上下左右，キャッチ）のデータをCANバスに送信する．
## settings
### Pin

| 用途      | STM32ピン | CubeMX設定 | Nucleo端子 |
| ------- | ------- | -------- | -------- |
| LEFT    | PB5     | GPIO_Input |          |
| RIGHT   | PB0     | GPIO_Input |          |
| UP      | PA8     | GPIO_Input |          |
| DOWN    | PB1     | GPIO_Input |          |
| CATCH   | PB4     | GPIO_Input | 仮割当(要確認) |
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
