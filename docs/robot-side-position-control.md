# ロボット側位置制御（CM4 で位置ループを閉じる構成）

## 目的

従来は crane（AI）側で位置制御ループを閉じ、その出力である速度指令を無線で
ロボットへ送っていた。この構成では **不安定で遅延が乗る無線経路が位置制御
ループの内側に入る**。

新構成では crane は `VisibilityGraphPlanner` が生成した **位置指令** を送り、
ロボット側の CM4 が位置制御ループを閉じて **速度指令** をマイコン（G474）へ
渡す。無線経路はループの外側（目標値の更新経路）に移動する。

このドキュメントは、その構成を **シミュレータ上で実機とできるだけ同じ形** で
再現するための、3 リポジトリ間の統合仕様を定める。

## 構成の対応

### 実機

```text
crane ──UDP broadcast :12345 (mode 4 位置指令)──> CM4: ai_cmd_v2.out
                                                     │ 位置制御ループ
                                                     ↓ UART /dev/serial0 72B (mode 3 速度指令)
                                                  G474 (500 Hz): 加速度制御・タイヤ速度制御
                                                     │ UART 128B feedback
                                                     ↓
                                                  CM4: forward_robot_feedback.out
                                                     ↓ multicast 224.5.20.(100+N):50100+N
                                                  crane / host ツール
```

### シミュレータ

```text
crane ──UDP :12345 (mode 4 位置指令)──> cm4_sim
                                          │ 位置制御ループ（実機と同一コード）
                                          ↓ UDP :12346 (mode 3 速度指令)
                                     simulator-cli  ← G474 + ロボット物理 を担当
                                          │ ibis feedback 128B
                                          ↓ UDP :50100+id
                                       cm4_sim
                                          ↓ multicast 224.5.20.(100+id):50100+id
                                       crane / host ツール
```

**対応関係**

| 実機 | シミュレータ | 備考 |
|---|---|---|
| crane | crane | 変更なし（送信先ポートのみ切替） |
| 無線 (WiFi) | UDP + 劣化注入 | `cm4_sim` の入力側で遅延・ジッタ・ロスを注入 |
| CM4 位置制御 | `cm4_sim` | **実機と同一の position_controller ライブラリを使う** |
| UART CM4→G474 | UDP :12346 | |
| G474 の速度・加速度制御 | `simulator-cli` の `IbisCommandAdaptor` | theta P 制御 + 加速度制限 |
| ロボット物理・タイヤ | `simulator-cli` (amun/Bullet) | |
| UART G474→CM4 feedback | ibis feedback 128B (UDP) | |

シミュレータは **G474 とロボット本体** を担当する。位置制御は行わない。

## パケット契約

### SSOT

`RobotCommandSerializedV2`（64 バイト）の正本は
`crane/crane_sender/include/crane_sender/robot_packet.h` とする。

現在のオフセット一致状況（2026-09-13 実測）:

| リポジトリ | ファイル | 状態 |
|---|---|---|
| crane | `crane_sender/include/crane_sender/robot_packet.h` | 正本 |
| G474_Orion_main | `Core/Inc/robot_packet.h` | byte 0..31 一致（32..37 は未使用のため未定義。問題なし） |
| framework | `src/simulator/ibis_protocol.h` | 一致 |
| **Orion_CM4** | `cm4/bridge/robot_packet.h` | **不一致（旧レイアウト）** |

Orion_CM4 のコピーは `ACCELERATION_LIMIT` が無いため **byte 12 以降が 2 バイト
ずれている**（`FLAGS`=20、`CONTROL_MODE`=21）。現在これが顕在化していないのは、
`forward_ai_cmd_v2.cpp` が受信バイト列を `memcpy` でそのまま UART へ転送する
**単なるバイト転送器** であり、デシリアライズ結果をデバッグ表示にしか使って
いないため。CM4 がパケットを解釈して制御ループを閉じた瞬間、これは正真正銘の
バグになる。**新構成の実装前に必ず統一すること。**

### バイトオフセット（正本）

```text
 0      HEADER
 1      CHECK_COUNTER
 2..3   VISION_GLOBAL_X            (float, range 32.767)
 4..5   VISION_GLOBAL_Y            (float, range 32.767)
 6..7   VISION_GLOBAL_THETA        (float, range PI)
 8..9   TARGET_GLOBAL_THETA        (float, range PI)
10      KICK_POWER                 (value * 20)
11      DRIBBLE_POWER              (value * 20)
12..13  ACCELERATION_LIMIT         (float, range 32.767)
14..15  LINEAR_VELOCITY_LIMIT      (float, range 32.767)
16..17  ANGULAR_VELOCITY_LIMIT     (float, range 32.767)
18..19  LATENCY_TIME_MS            (uint16)
20..21  ELAPSED_TIME_MS_SINCE_LAST_VISION (uint16)
22      FLAGS
23      CONTROL_MODE
24..31  CONTROL_MODE_ARGS          (mode により意味が変わる union)
32..33  TARGET_GLOBAL_POS_X        (float, range 32.767)
34..35  TARGET_GLOBAL_POS_Y        (float, range 32.767)
36..37  TERMINAL_VELOCITY          (float, range 32.767)
```

FLAGS: bit0 `IS_VISION_AVAILABLE` / bit1 `ENABLE_CHIP` / bit3 `STOP_EMERGENCY`

### 制御モード

| 値 | 名前 | ARGS (24..31) | 送信元 → 受信先 |
|---|---|---|---|
| 3 | `POLAR_VELOCITY_TARGET_MODE` | `target_global_velocity_r`, `target_global_velocity_theta` | CM4 → G474 / cm4_sim → simulator-cli |
| 4 | `POSITION_TARGET_WITH_TERMINAL_VELOCITY_MODE` | `terminal_velocity_x`, `terminal_velocity_y` | crane → CM4 / crane → cm4_sim |

**`CONTROL_MODE_ARGS` は union であり、`CONTROL_MODE` を見ずに復号してはならない。**
mode 4 のパケットを mode 3 として復号すると、`terminal_velocity_x/y` が
`r/theta` として読まれ、無言で暴走する。

### パケット全体

1 スロット = `robot_id` 1 バイト + コマンド 64 バイト = 65 バイト。
11 スロット固定で 715 バイト。送信元が制御しないロボットのスロットは
ゼロ埋めする（受信側は `ibisSlotIsEmpty()` 相当で明示的にスキップすること）。

## ポート割当

| 経路 | アドレス:ポート | 備考 |
|---|---|---|
| crane → cm4_sim（位置指令） | `127.0.0.1:12345` | 実機の AI 指令ポートと同じ |
| cm4_sim → simulator-cli（速度指令） | `127.0.0.1:12346` | `simulator-cli --ibis-port 12346` |
| simulator-cli → cm4_sim（feedback） | `127.0.0.1:50100+id` | `--ibis-feedback-addr 127.0.0.1`（既定） |
| cm4_sim → crane / host（feedback 再配信） | `224.5.20.(100+id):50100+id` | 実機と同じ multicast |
| simulator-cli → crane（vision） | `224.5.23.2:10020`（既定・変更不可） | 変更なし |

feedback のベースポートは **実機と同じ 50100 のまま** でよい。同一ホスト上で

- `cm4_sim` が `127.0.0.1:50100+id` を bind（simulator-cli からの unicast を受ける）
- `crane_robot_receiver` が `224.5.20.(100+i):50100+i` を bind（multicast を受ける）

という 2 つの bind が同居するが、**これらはポート番号が同じでも競合しない**ことを
実測で確認済み。unicast は unicast ソケットにのみ、multicast は multicast ソケットに
のみ配送され、取り違えも起きない。両者とも `SO_REUSEADDR` を設定すること。

simulator-cli の `--ibis-feedback-addr` の既定値が `127.0.0.1` なので、
feedback 関連のオプション指定は不要である。

vision の出力先ポート・アドレスを変えるオプションは無い（`ibis` ブランチ時点）。
`simulator-cli` は vision を **10020 番**でマルチキャストする（通常の 10006 ではなく、
大会ネットワークでの衝突回避のため）。同一ホストで複数インスタンスを走らせると
vision が混線するので、並列実行が必要ならコンテナのネットワーク分離を使うこと。

また、`simulator-cli` は SSL tracker（`TrackedFrame`）を出力しない。
crane はロボットを tracker 経路で追跡するため、**外部の auto-referee が必要**である
（`docker/scenario/docker-compose.yaml` の `autoref-tigers` がこれを担っている）。

## タイミング契約

### 各段のレート

シミュレータは実時間で動く（wall clock 駆動）。

| 段 | 実機 | シミュレータ |
|---|---|---|
| crane 指令 | 約 60 Hz | vision フレーム毎 = 62.5 Hz |
| CM4 位置制御ループ | 1 kHz（`usleep(1000)` ポーリング） | `--rate-hz`（既定 1 kHz） |
| G474 メインループ | 500 Hz (`MAIN_LOOP_CYCLE`) | 125 Hz（sim 基本ループ 8 ms） |
| vision 送出 | 約 60 Hz | 62.5 Hz（基本ループ 2 回に 1 回） |
| feedback | 約 125 Hz | 125 Hz（基本ループ毎） |

simulator-cli の基本ループは **8 ms = 125 Hz**（`Simulator::setScaling()` の
"scale default timing of 8 milliseconds (125Hz)"）で、vision はその 2 回に 1 回なので
62.5 Hz。Bullet の物理サブステップは 250 Hz（`SUB_TIMESTEP = 1/250.f`）。

CM4 制御ループ 1 kHz に対し G474 相当（シミュレータ基本ループ）が 125 Hz なので、
**実機（1 kHz : 500 Hz）より内側ループが粗い**。制御ゲインを詰める際はこの差を
意識すること。

### feedback はループ駆動である

`simulator-cli` の ibis feedback は独立したタイマーではなく
`Simulator::handleSimulatorTick()` の中（`src/amun/simulator/simulator.cpp`）で
**シミュレータ基本ループ 1 回につき 1 回** 送出される。位置は
`world::SimulatorState`（ground truth）由来なので毎回新鮮である
（`--ibis-feedback-hz` オプションは存在しない）。

新構成では **この feedback の位置が位置制御ループ内で唯一の位置信号** になる。
従来は装飾的なテレメトリだったが、これからは制御品質を直接左右する。

### check_counter の扱い（重要）

G474 の `checkConnect2AI()`（`Core/Src/ai_comm.c`）は
**`check_counter` が変化し続けること** を AI 接続生存の判定に使う。
変化が `AI_CMD_TIMEOUT(0.5) * MAIN_LOOP_CYCLE(500)` = **250 ms** 途切れると
`connected_ai = false` になる。

従来の CM4 は crane のパケットをそのまま転送していたため、`check_counter` は
crane が採番したものだった。新構成では CM4 が自分の制御周期で新しい指令を
生成するので、**CM4 が `check_counter` を自分で採番する** 必要がある。

その結果、G474 の `connected_ai` は **crane の生存を意味しなくなる**。
crane からのパケットが途絶えた場合の安全停止は、CM4 側で明示的に実装すること
（例: crane 無通信が一定時間続いたら `STOP_EMERGENCY` を立てる、または
速度指令をゼロにする）。これは実機・シミュレータ双方に共通の要件である。

### cm4_sim のペーシング

`cm4_sim` は実機の `forward_ai_cmd_v2.cpp` と同じ構造にする。

- `--rate-hz`（既定 1 kHz）の固定周期で回し、最後に受信した feedback を使う。
- **feedback 待ちでブロックしてはならない。** ソケットはノンブロッキングにし、
  届いていなければ前回値で制御する（実機の `usleep(1000)` ポーリングループと同じ）。
- 11 台分を **1 プロセス・1 データグラムに集約** して送る。台数分のプロセスを
  立てると、同一ポートへの送信が増えるだけで実機の構成に近づかない。
- crane の指令は約 60 Hz でしか来ないので、CM4 制御ループはその間
  同じ目標値に対して feedback だけを更新しながら回ることになる。これは実機と同じ。

## framework 側の実装状況（本リポジトリ・実装済み）

- `src/simulator/ibis_protocol.h`
  - `IBIS_MODE_POLAR_VELOCITY_TARGET` (3) / `IBIS_MODE_POSITION_TARGET` (4) を定義。
  - `ibisDeserialize()` が `CONTROL_MODE` を見て ARGS を復号するようになった。
    従来は mode を無視して常に polar velocity として読んでいた。
  - `target_global_pos` / `terminal_velocity` / `vision_global_theta` /
    `is_vision_available` を復号対象に追加。
  - `ibisSlotIsEmpty()` を追加。
- `src/simulator/simulator.cpp` `IbisCommandAdaptor`
  - 空スロットを明示的にスキップ。
  - **mode 3 以外を受け取ったらロボットを停止し、1 秒/台 のレート制限付きで警告**
    を出す。mode 4 が届くのは「cm4_sim が経路に入っていない」設定ミスであり、
    無言で誤解釈するより停止して理由を出すほうが安全かつデバッグしやすい。

simulator-cli 側に位置制御は **実装しない**。実装すると制御則のコピーが
4 つ目になり、`robot_packet.h` が 3 リポジトリで食い違った問題を繰り返す。

## 検証（A/B 比較）

この設計の主張は「無線経路をループ外に出すと、遅延・ジッタ・ロスに強くなる」
である。それを示すには、同じ劣化条件下で旧構成と新構成を比較する必要がある。

| | 旧構成 | 新構成 |
|---|---|---|
| crane | 位置ループを閉じ mode 3 を送る | mode 4 を送る |
| 経路 | crane → simulator-cli | crane → cm4_sim → simulator-cli |
| 劣化注入 | crane の送信経路 | cm4_sim の入力側 |

`cm4_sim` の入力側に `--rx-delay-ms` / `--rx-jitter-ms` / `--rx-loss-rate` を実装し、
両構成を同一条件で走らせて追従誤差・オーバーシュート・到達時間を比較する。

### simulator-cli 側のスモークテスト

`data/scripts/ibis-chain-smoketest.py` が、実際に `simulator-cli` を起動して
UDP 越しに以下を検証する。

```bash
python3 data/scripts/ibis-chain-smoketest.py [path/to/simulator-cli]
```

1. mode 3 でロボットが動く
2. mode 4 でロボットが停止し、レート制限付き警告が出る

`cm4_sim` を実装する前に simulator-cli 側の契約を固定するためのもの。
既定ポートとは離れたポート（12397 / 50700 / 10097 / 11097）を使うので、
動作中の試合には影響しない。

## 既知の忠実度ギャップ

- **feedback の位置が真値である。** シミュレータの feedback は ground truth を
  そのまま返すため、ノイズも vision 遅延も無い。実機の G474 は
  `vision_based_position` としてタイヤオドメトリと vision を融合した推定値を返す。
  シミュレータ上の位置制御は実機より良く見える。必要ならノイズ注入を追加する。
- **ボールセンサ・キック状態。** `kick_status` は常に 0。
- **ローカルカメラ。** `cam_server_v3` 相当は無い。`cm4_sim` はカメラ領域を
  ゼロ埋めすること（実機のカメラ未接続時と同じ扱い）。
- **バッテリ電圧・温度等。** 固定値。

## 関連ファイル

- `src/simulator/ibis_protocol.h` — プロトコル定義（framework 側）
- `data/scripts/ibis-chain-smoketest.py` — ibis コマンド経路のスモークテスト
- `src/simulator/simulator.cpp` — `IbisCommandAdaptor` / `IbisFeedbackAdaptor`
- `src/amun/simulator/simulator.cpp` — `handleSimulatorTick()`、feedback 送出
- `Orion_CM4/cm4/bridge/robot_packet.h` — CM4 側パケット定義（要統一）
- `Orion_CM4/doc/control_packet.md` — CM4 側の制御パケット仕様
- `G474_Orion_main/Core/Src/ai_comm.c` — `check_counter` による接続監視
- `crane/crane_sender/src/ibis_sender_node.cpp` — mode 4 送信
- `crane/crane_local_planner/src/visibility_graph_planner.cpp` — 位置目標の生成
