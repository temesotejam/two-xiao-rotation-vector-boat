# 本番実装仕様

## 1. 実行構成

本リポジトリは通信側・制御側ともXIAO ESP32S3です。CoreS3向けコードは含みません。通信側はGNSS、SD、Web、UART中継に限定し、BNO08Xを初期化しません。制御側だけが姿勢・周辺センサ・アクチュエータを扱います。

姿勢はBNO08Xの`SH2_ROTATION_VECTOR`を50 Hzで直接使用します。加速度・校正済みジャイロは200 Hz、磁気は20 Hzです。AS5600は使用せず、推進回転はVESC UARTのERPMで監視します。

## 2. 安全状態

状態は`BOOT / DISARMED / ARMED_IDLE / RUNNING / E_STOP / FAULT`です。

- 起動直後にD10をLOWへ設定してから、I2CやUARTを初期化します。
- DISARMED、E_STOP、FAULTではVESC Duty 0、D10 LOW、PCA9685 Full OFFです。
- E_STOP中のSTOP、DISARM、ARM、STARTはE_STOPを解除しません。`ClearEstop`だけがDISARMEDへ戻します。
- FAULTはDISARMでDISARMEDへ戻せますが、自動ARMは行いません。
- 通信側heartbeatが500 ms途絶すると停止します。
- Manualは500 ms以内の手動指令と1つ以上の出力選択を必要とします。
- Manualで推進を選ばない場合、VESC未接続でも接続済みサーボを操作できます。
- Auto WaypointはBNO、GNSS、経路、VESCテレメトリ、有効な電源電圧を必要とします。

## 3. D10リレー

D10は次の条件を満たす非ゼロDuty送信直前だけHIGHです。

- RUNNING
- 推進出力が選択または自動航行中
- VESCテレメトリが新鮮
- VESC faultが0
- 電源電圧が臨界値より高い
- E_STOP/FAULTでない
- dry-runでない

Duty 0では0指令を送ってからLOWへ戻します。VESC状態要求だけではHIGHになりません。UART write失敗、通信断、VESC期限切れ、fault、STOP、DISARM、E-STOP、FAULTではLOWです。

## 4. 制御則

```text
height  = Kp_height × (target_height - measured_height)
pitch   = -Kp_pitch × pitch - Kd_pitch × pitch_rate
roll    = -Kp_roll × roll - Kd_roll × roll_rate
yaw     = Kp_yaw × wrap(target_yaw - yaw) - Kd_yaw × yaw_rate

left front  = height + pitch + roll
right front = height + pitch - roll
rear yaw    = yaw
```

ToFが一時的に無効な場合は高さ項だけを0にし、姿勢制御は継続します。pitchが危険域へ近づくとroll、yaw、推進を連続的に縮小し、臨界角でFAULTへ移ります。

ウェイポイントでは前区間への投影点から4 m先をLOS目標とします。到達半径はWebから設定でき、既定1.5 mです。最終点到達時はDuty 0、リレーLOW、DISARMEDへ移ります。

## 5. 電源・VESC保護

電源値は新鮮なINA226を優先し、未接続時はVESC入力電圧・入力電流を使用します。

| 項目 | 制限開始 | 強制停止 |
|---|---:|---:|
| 入力電圧 | 9.5 V未満 | 8.5 V以下 |
| 電流 | 22 A超 | 28 A以上 |

Duty 25%以上、モータ電流8 A以上、ERPM 100未満が1秒続くと拘束として停止します。VESC faultが0以外の場合も停止します。

## 6. 時刻同期とログ

通信側が1 Hzで`t1`を送信し、制御側が受信時刻`t2`と返信時刻`t3`を返します。通信側受信時刻`t4`から次を計算します。

```text
control_minus_communication = ((t2 - t1) + (t3 - t4)) / 2
RTT = (t4 - t1) - (t3 - t2)
```

推定値は制御側へ返し、WebとログにRTT、オフセット、不確かさを残します。

SDファイルは`/BOATLOG/RUNxxxx.BIN`です。ファイルヘッダ`BLG2`の後に、通信側ローカル時刻、UARTフレームヘッダ、ペイロード、レコードCRCを保存します。送信フレームはheader flagsの`0x8000`で区別します。ログ停止時にはキューを破棄し、次のRUNへの混入を防止します。

## 7. Web API

- `GET /api/status`
- `POST /api/log/start`, `/api/log/stop`
- `POST /api/control/mode`
- `POST /api/control/manual`
- `POST /api/control/heading`
- `POST /api/control/run`
- `POST /api/control/arm`, `/disarm`, `/start`, `/stop`, `/estop`, `/clear-estop`
- `POST /api/waypoints`

ウェイポイント変更は制御側がDISARMED中だけ受理します。WebのHTTP 202は通信側キューへの受付を示し、最終結果は`CommandAck`の`disposition`と`reason`で確認します。

## 8. 未確定の実機パラメータ

以下はコード上で安全な初期値を設定していますが、実機形状・回転方向に合わせて`control/include/app_config.h`を調整します。

- 左右前翼・後部ヨーの反転フラグ
- 各制御ゲイン
- 目標高さ
- サーボ機械範囲
- 自動推進Duty
- 危険roll/pitch角
- 電源・電流閾値

これらは機能欠落ではなく機体固有の調整値です。安全状態、上限、リレー連動は調整値に関係なく常時有効です。
