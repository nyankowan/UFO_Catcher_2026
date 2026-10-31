# UFO_Catcher_2026
2026岡大学祭の岡ロボUFOキャッチャーのプログラム

## 構成
| ディレクトリ | マイコン | 役割 |
| :--- | :--- | :--- |
| `controller_data_sender` | STM32F303K8 | コントローラ(上下左右・キャッチ)の入力をCANで送信 |
| `main_controller` | STM32F446RE | 入力を受信し、ロボマス(X/Y/Zアーム)と爪サーボを制御 |
| `common` | - | 両方で使うCANプロトコル定義と送信関数 |

```text
[ボタン] -> controller_data_sender --CAN1 100kbps, ID 0x100--> main_controller --CAN2 1Mbps, ID 0x200--> ロボマス C610/C620 x3
                                                               main_controller --PC5--> 爪サーボ
```

## CANプロトコル (`common/can_protocol.h`)
| ID | 向き | DLC | 内容 |
| :--- | :--- | :--- | :--- |
| 0x100 | sender -> main | 1 | bit0 left, bit1 right, bit2 up, bit3 down, bit4 cat(キャッチ) |
| 0x200 | main -> ロボマス | 8 | モータ1..4の電流指令 (int16, ビッグエンディアン) |
| 0x201〜 | ロボマス -> main | 8 | フィードバック(角度, 回転数, 電流, 温度) |

## ビルド
各ディレクトリで、arm-none-eabi-gcc と CMake が必要です。
```sh
cd main_controller            # または controller_data_sender
cmake --preset Debug
cmake --build --preset Debug
```
生成される `.elf` を書き込みます。

## 注意
- 両基板とも、クロックは ST-LINK の MCO (8MHz, HSE bypass) を使います。HSEが起動しない場合はHSIに切り替わります(CANの許容誤差が厳しいので、通信が不安定になります)。
- CubeMXで Generate Code をすると、`SystemClock_Config` のHSIフォールバックが消えます。
- ロボマスの電源を先に入れてから、main_controller を起動してください。

各基板の詳細は、`controller_data_sender/README.md` と `main_controller/README.md` を見てください。
