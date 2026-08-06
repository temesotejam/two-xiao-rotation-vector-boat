# Two XIAO Production Boat Firmware

Seeed Studio XIAO ESP32S3を2台使用する、水中翼ボートの本番統合ファームウェアです。BNO08X内蔵の**Rotation Vector**を正規の姿勢入力として使用し、AS5600や独自ESKFは使用しません。

| フォルダ | 基板 | 担当 |
|---|---|---|
| `control/` | 制御側XIAO ESP32S3 | BNO08X、VL53L5CX、INA226、PCA9685、VESC、安全状態、手動・姿勢・ウェイポイント制御、D10リレー |
| `communication/` | 通信側XIAO ESP32S3 | GNSS、microSD、SoftAP/Web UI、制御側UART、時刻同期、ログ |

## 実装内容

- XIAO間UART: 921600 bps、COBS + CRC32、sequence/boot ID付き
- BNO08X: 加速度・ジャイロ200 Hz、Rotation Vector 50 Hz、磁気20 Hz
- GNSS: 通信側で受信し、10 Hzで制御側へ送信
- VL53L5CX: 8×8、10 Hz。中央領域から高さを算出
- INA226: 50 Hz、R002（2 mΩ）設定
- PCA9685: 50 Hz。CH0左前翼、CH1右前翼、CH2後部ヨー、CH3～15はFull OFF
- VESC: `GET_VALUES` 50 Hz、Duty指令20 Hz、ERPM・電流・電圧・温度・fault監視
- 制御側D10: VESCへ非ゼロDutyを送る直前だけHIGH。Duty 0、停止、通信断、faultではLOW
- 手動、姿勢補助、方位保持、ウェイポイント自動航行
- E-STOPラッチ、通信途絶、センサ期限切れ、低電圧、過電流、拘束、危険姿勢で全出力停止
- 1 Hzの双方向時刻同期とRTT/オフセット記録
- microSDバイナリログ。受信・送信フレームと通信側ローカル時刻を保存
- 日本語Web UI。状態、姿勢、GNSS、ToF、INA226、VESC、リレー、サーボ、ACK、ログを表示
- GitHub Actionsによる2台分のビルド、SHA-256、Web Serial Installer用ファイル生成

## 配線

### 制御側XIAO

| 用途 | 端子 |
|---|---|
| 周辺I2C SCL/SDA | D0 / D1 |
| BNO08X RESET/INT | D2 / D3 |
| BNO08X SDA/SCL | D4 / D5 |
| 通信側UART RX/TX | D6 / D7 |
| VESC UART RX/TX | D8 / D9 |
| VESC安全リレー | D10（HIGHで接続、LOWで切断） |

周辺I2CアドレスはVL53L5CX `0x29`、PCA9685 `0x40`、INA226 `0x44`です。BNO08Xは専用I2Cの`0x4A`（代替`0x4B`）です。

### 通信側XIAO

| 用途 | 端子 |
|---|---|
| GNSS RX/TX | D0 / D1 |
| 制御側UART RX/TX | D7 / D6 |
| microSD SCK/MISO/MOSI | D8 / D9 / D10 |
| microSD CS | GPIO21 |

UARTはクロス接続し、両XIAOのGNDを共通にします。

## ビルド

```bash
pio run -d control -e xiao_control
pio run -d communication -e xiao_communication
```

物理出力を禁止する制御側dry-runビルドも残しています。

```bash
pio run -d control -e xiao_control_dry_run
```

GitHub Actions成功後、`two-xiao-boat-firmware` artifactから2台分のBINとSHA-256を取得できます。mainへマージするとGitHub PagesのWeb Installerも更新されます。

## 起動と操作

1. 通信側XIAOのAP `BOAT-CONTROL`（パスワード `12345678`）へ接続します。
2. `http://192.168.4.1/`を開きます。
3. モード、使用出力、指令値を設定します。
4. `ARMして開始`で設定、ARM、STARTを順にUARTキューへ送ります。
5. 通常停止は`停止`、緊急時は`緊急停止`を使用します。

起動時は必ずDISARMED、D10 LOW、PCA9685全チャンネルFull OFFです。E-STOPは`E-STOP解除`を受けるまで解除されません。

詳細は[`docs/PRODUCTION_IMPLEMENTATION.md`](docs/PRODUCTION_IMPLEMENTATION.md)を参照してください。
