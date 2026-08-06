# Two XIAO Rotation Vector Boat

Seeed Studio XIAO ESP32S3 Sense を2台使う、ESKFなしの安全な実験用ファームウェアです。

- `control/`: BNO08Xを接続する制御側XIAO。BNO08X内蔵フュージョンの **Rotation Vector** を50 Hzで受信し、クォータニオンとRoll/Pitch/Yawを通信側へ送信します。
- `communication/`: 通信側XIAO。BNO08Xは接続・初期化しません。GNSS、microSD、SoftAP/Web UI、UART中継を担当します。
- ESKFは実装・実行しません。
- 両方とも `ACTUATOR_OUTPUT_ENABLE=0` です。PCA9685は全チャンネルOFF、VESCは受信解析のみで、アクチュエータを駆動するコードは含めません。

## 配線

### 制御側 XIAO

| 用途 | ピン |
| --- | --- |
| BNO08X I2C | SDA D4 / SCL D5 |
| BNO08X RESET / INT | D2 / D3 |
| 周辺I2C (ToF/INA226/PCA9685) | SDA D1 / SCL D0 |
| 通信側とのUART | RX D6 / TX D7, 921600 bps |
| VESC受信 | RX D8 / TX D9, 115200 bps |

### 通信側 XIAO

| 用途 | ピン |
| --- | --- |
| GNSS | RX D0 / TX D1, 115200 bps |
| 制御側とのUART | RX D7 / TX D6, 921600 bps |
| microSD SPI | CS 21 / SCK D8 / MISO D9 / MOSI D10 |

UARTはクロス接続します（制御側D7 TX → 通信側D7 RX、通信側D6 TX → 制御側D6 RX）。両XIAOのGNDを共通にします。

## ビルド

```powershell
cd control
pio run -e xiao_control_rotation_vector_shadow
cd ..\communication
pio run -e xiao_communication_shadow
```

このリポジトリでは書き込みコマンドを実行していません。

## Web UI

通信側XIAOに接続後、SoftAP `BOAT-CONTROL`（パスワード `12345678`）へ接続し、`http://192.168.4.1/` を開きます。

- `GET /` : 人間向け状態表示と50 ms更新のYawグラフ
- `GET /api/status` : JSON（Rotation Vector、GNSS、UART、SD、ログの鮮度・状態）
- `POST /api/log/start`, `POST /api/log/stop` : SDログ
- `POST /api/control/arm`, `/disarm`, `/start`, `/stop`, `/estop` : 制御側へ安全状態コマンドを送信

ログは通信側microSDの `/BOATLOG/RUNxxxx.BIN` に、制御側から受信したフレームとして保存されます。

## 制限・安全性

- Rotation Vectorは磁気センサを使うため、ヨー角は周囲の磁場やキャリブレーション状態の影響を受けます。
- 実機への書き込み、実機接続、航走試験は未実施です。
- D10リレーの駆動はこの安全構成には実装していません。