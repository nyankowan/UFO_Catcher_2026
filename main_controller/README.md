# main_controller
## 基盤の名称と実際の動作の対応
| 基盤の名称|実際の動作 |
| :---      | :---      |
| UP_DEG+   | UP_LIM    |
| LOW_DEG+  | DOWN_LIM  |
| UP_DEG-   | LEFT_LIM  |
| DOWN_DEG- | RIGHT_LIM |

## 概要
`controller_data_sender` からCAN1(100kbps, ID 0x100)で入力を受け取り、CAN2(1Mbps, ID 0x200)でロボマスを制御する。
爪はサーボ(PC5)で動かす。

## ピン
| 用途 | ピン | 備考 |
| :--- | :--- | :--- |
| CAN1 RX/TX | PA11 / PA12 | コントローラ入力 (100kbps) |
| CAN2 RX/TX | PB12 / PB13 | ロボマス (1Mbps) |
| LEFT_LIM / UP_LIM | PC3 / PC4 | リミットスイッチ(プルアップ、押すとLOW) |
| RIGHT_LIM / DOWN_LIM | PC10 / PC11 | 同上 |
| 爪サーボ | PC5 | TIM1+DMA2でGPIOを叩く50Hz PWM |
| ステータスLED | PA5 | 入力があるときだけ点灯 |
| デバッグUART | PA2 / PA3 | USART2 115200bps |

## ロボマスの割り当て
モータID 1 = X、2 = Y、3 = Z(アーム昇降)。XYは回転数(速度)制御、Zは位置制御(位置P -> 回転数PI)。
Zの原点は電源投入時のアーム位置(上端)。下降は負方向。

## 動作
1. 手動: 上下左右ボタンでXYを動かす(リミットスイッチで停止)。
2. キャッチボタンで、下降 -> 爪を閉じる -> 上昇 -> 左手前(`LEFT_LIM`/`DOWN_LIM`)へ移動 -> 爪を開く -> 手動に戻る。

## 調整するパラメータ (`Core/Src/main.c` の `USER CODE PD`)
| 名前 | 内容 |
| :--- | :--- |
| `SERVO_OPEN_US` / `SERVO_CLOSE_US` | 爪サーボのパルス幅 (500〜2500us) |
| `Z_DOWN_POS` | Zの下降量 (エンコーダ値、8192=1回転) |
| `XY_MAX_RPM`, `XY_HOME_RPM` | XYの最高速、帰還速度 |
| `CUR_MAX` | 電流指令の上限 (C610は±10000) |
| `VEL_KP`, `VEL_KI`, `Z_KP_POS` | 制御ゲイン |
| `T_*` | シーケンスの待ち時間・上限 |

## デバッグ出力
USART2(115200bps)に100msごとに1行出る。
```text
clk=HSE CAN1 n= id= ... | L R U D C | st | fb= n= rpm= z= | lim= cur= c2esr= tx2fail= c2boff= c2lec=
```
- `clk=HSE` でなければ、HSEが起動していない。
- `fb=` はロボマス3台のフィードバックの有無。0があると、安全のため全モータの電流を0にする。
- `tx2fail` が増え続ける、`c2esr` の bit2(BOFF)が立つ場合は、CAN2のバスオフ。`c2lec` は左から stuff, form, ACK, ビット(劣性), ビット(優性), CRC の回数。
  ACKが多いときはロボマスの電源やビットレート、ビットが多いときはトランシーバや配線を疑う。
