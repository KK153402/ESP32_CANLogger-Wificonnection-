# CanLoggerJig

PCとCANインタフェースを持ち込めない現場で、単独でCANログを取得するための治具。
**CAN1(500kbps) / CAN2(250kbps) の2バスを同時に**SDカードへ記録し、
PC側でBLF形式へ変換してCANalyzerで解析する。

```
測定対象のCANバス ──▶ 治具（SDカードへ記録）──▶ PCでBLF変換 ──▶ CANalyzerで解析
                            │
                            └─▶ Wi-Fiで自動アップロード（v2.6〜）
```

ファームウェア **v2.7** / ハードウェア **Rev.5**

> **v2.6以降、記録したログをWi-Fi経由でサーバーへ自動送信できる。**
> 待機中にのみ動作し、記録中は完全に停止する。将来的にLTE（SIM7600JC-H）へ置き換える前提で、
> 通信方式に依存しない部分（分割・再送・進捗管理）を先に実装・検証したもの。

---

## ドキュメント

| 目的 | ファイル |
|---|---|
| **使い方**（現場での操作手順） | [`docs/USAGE.md`](docs/USAGE.md) |
| **技術解説**（プログラムの中身、初心者向け） | [`docs/TECHNICAL.md`](docs/TECHNICAL.md) |
| **設計仕様**（試算・ファイル形式・実測結果） | [`docs/DESIGN.md`](docs/DESIGN.md) |
| **ハードウェア仕様**（BOM・ピンアサイン・査読結果） | [`docs/HARDWARE.md`](docs/HARDWARE.md) |

アップロード機能について知りたい場合は、
[`docs/DESIGN.md`](docs/DESIGN.md) §7（仕様）と
[`docs/TECHNICAL.md`](docs/TECHNICAL.md) §15（設計判断の理由）を参照。

---

## 実績

実機での測定結果。

| 項目 | 結果 |
|---|---|
| CH1（500kbps） | 2,747フレーム / **取りこぼし0** |
| CH2（250kbps） | 54,263フレーム / **取りこぼし0** |
| 2ch同時記録 | 57,010フレーム / 59.16秒 |
| BLF変換 → CANalyzer再生 | **動作確認済み** |
| Wi-Fiアップロード | 1,368,272 byte を2パートで送信 / **再送・中断再開とも動作確認済み** |
| 送信速度（1MBチャンク） | 124〜166 kB/s |

---

## ハードウェア構成

```
                    ┌──── 機能絶縁境界（CH1）────┐
CN1 (CAN1 500k) ────┤ RESD1CANY → ISO1042DWV     ├─── ESP32 内蔵TWAI
                    │  NME1S0505SC + 100Ωダミー   │
                    └─────────────────────────────┘

                    ┌──── 機能絶縁境界（CH2）────┐
CN2 (CAN2 250k) ────┤ RESD1CANY → ISO1042DWV     ├─── MCP2515 (HSPI)
                    │  NME1S0505SC + 100Ωダミー   │
                    └─────────────────────────────┘

USB-C ─ TPD2E2U06 ─ MF-FSML100 ─ TPS562201 ─ 3.3V ─ ESP32-WROOM-32E-N16
                                                     ├── microSD (VSPI)
                                                     ├── MCP7940N (I2C) + CR2032
                                                     ├── 開始/停止SW、状態LED
                                                     └── CH340E (USB-UART)
```

| 主要部品 | 型番 |
|---|---|
| MCU | ESP32-WROOM-32E-N16 |
| CANトランシーバ | ISO1042DWV（絶縁）×2 |
| CANコントローラ（CH2） | MCP2515T-I/SO |
| 絶縁DC/DC | NME1S0505SC ×2 |
| RTC | MCP7940N-I/SN |
| 降圧コンバータ | TPS562201DDCR |
| USB-UART | CH340E |

詳細は [`docs/HARDWARE.md`](docs/HARDWARE.md)。

---

## ディレクトリ構成

```
CanLoggerJig/
├── platformio.ini          ビルド設定
├── src/
│   ├── main.cpp            状態機械、受信タスク、書き込みタスク、シリアルコマンド
│   ├── config.h            ピン配置・ビットレート・バッファサイズ
│   ├── ring_buffer.h       24byte固定長レコードとSPSCリングバッファ
│   ├── mcp2515.h           MCP2515ドライバ（Listen-Only、外部ライブラリ不要）
│   ├── mcp7940.h           MCP7940N RTCドライバ
│   ├── asc_format.h        ASC形式の生成（ASCモード時のみ）
│   ├── cfgfile.h           /CONFIG.TXT のパーサ
│   └── uploader.h          Wi-Fiアップロード（分割・再送・進捗管理）
├── tools/
│   ├── bin_to_blf.py       BIN → BLF 変換。ファイル形式の仕様書も兼ねる
│   ├── bin_to_blf.py.txt   同上（拡張子制限がある環境向け）
│   ├── merge_parts.py      分割パートを1つのBINへ連結
│   └── upload_server.py    検証用のHTTP PUT受信サーバ
└── docs/
    ├── USAGE.md            操作手順書
    ├── TECHNICAL.md        技術解説
    ├── DESIGN.md           設計仕様
    └── HARDWARE.md         ハードウェア仕様
```

外部ライブラリへの依存はない（`SD` / `SPI` / `Wire` / `WiFi` / `Preferences` はフレームワーク同梱）。

---

## ビルドと書き込み

### 必要なもの

- [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/)
  または VSCode + PlatformIO IDE拡張

### 手順

```bash
cd CanLoggerJig

pio run                  # ビルド
pio run -t upload        # 書き込み
pio device monitor       # シリアルモニタ（115200）
```

初回は arduino-esp32 2.0.17 のツールチェーンを自動取得するため数分かかる。

> **書き込みは手動操作が必要。** USB-UARTにCH340E（DTR#なし）を使っているため自動リセットができない。
>
> **BOOTボタンを押しながらENボタンを押して離す → BOOTを離す → `pio run -t upload`**

### ビルド環境（env）

| env | 用途 | コマンド |
|---|---|---|
| `esp32dev` | 通常構成（CH1 + CH2 + RTC）。既定 | `pio run` |
| `ch1_only` | CH1のみ。MCP2515未実装基板の検証用 | `pio run -e ch1_only -t upload` |
| `no_rtc` | RTC無し。MCP7940N未実装基板の検証用 | `pio run -e no_rtc -t upload` |
| `asc_format` | ASCテキスト保存 | `pio run -e asc_format -t upload` |

`config.h` の主要な設定値には `#ifndef` ガードがあるので、
ソースを書き換えずに `build_flags` の `-D` で上書きできる。

```ini
build_flags = -DCH2_ENABLE=0 -DCH1_BITRATE=250000UL
```

アップロード機能を外すビルドもできる。`WiFi.h` ごとリンクされなくなり、ヒープが約40KB戻る。

```ini
build_flags = -DUPLOAD_ENABLE=0
```

### バージョン固定について

`platformio.ini` で `espressif32@6.8.1`（= arduino-esp32 2.0.17）を固定している。
本ファームは `driver/twai.h` を直接使っており、arduino-esp32 3.x では
API・ヘッダ構成が変わるためビルドが通らない。

### Arduino IDE で使う場合

`src/main.cpp` は `.ino` ではないためArduino IDEでは開けない。
使う場合は `main.cpp` を `CanLoggerJig/CanLoggerJig.ino` にリネームし、ヘッダ5つを同じフォルダへ置く。
**全ての関数を呼び出し箇所より前に定義してある**ので、どちらの形式でもビルドできる。
関数を追加する際はこの順序を崩さないこと。

---

## シリアルコマンド

```bash
pio device monitor
```

| コマンド | 内容 |
|---|---|
| `SETTIME YYYY-MM-DD HH:MM:SS` | RTCに現地時刻を設定（記録中は不可） |
| `GETTIME` | 現在のRTC時刻を表示 |
| `TRIM [-127..127]` | 発振周波数の補正値を表示／設定 |
| `STATUS` | 状態・fault・ヒープ残量・RTC診断フラグ・アップロード状態 |
| `UPLOAD <LOGxxxx\|nnnn\|SCAN>` | 指定ファイルを送る／未送信を探し直す |
| `UPSTAT` | アップロードの状態を表示 |
| `WIFI` | Wi-Fiの接続状態 |
| `RELOAD` | `/CONFIG.TXT` を読み直す |
| `HELP` | コマンド一覧 |

製作時に一度 `SETTIME` すれば、CR2032が切れるまで保持される。

```
> STATUS
state=IDLE  rtc=2026-09-01 15:01:50  fault=none  freeheap=164152
rtc flags: OSCRUN=1 PWRFAIL=0 VBATEN=1 (RTCWKDAY=0x2B)  trim=+0 (+0.0 ppm)
upload  : phase=idle  boot#=4  ok=6  fail=0  sent=1715725 byte
          last: LOG0006 uploaded (2 parts)
wifi    : your-ssid  ip=192.168.10.106  rssi=-43 dBm
policy  : A (everything auto)
dest    : http://192.168.10.104:8080/canlog/JIG001/  chunk=1024 KB
```

---

## ログの自動アップロード

待機中にSDカードを走査し、未送信のログをHTTP PUTでサーバーへ送る。
**記録中は完全に停止する**ので、ログの取りこぼしには影響しない。

### 設定

SDカードのルートに `CONFIG.TXT` を置く。

```
WIFI_SSID     = your-ssid
WIFI_PASS     = your-password
UP_ENABLE     = 1
UP_AUTO       = 1          # 1=待機中に自動送信 / 0=UPLOADコマンドのみ
UP_STATS_ONLY = 0          # 1=統計TXTだけ送る
DEVICE_ID     = JIG001
UP_HOST       = 192.168.10.104
UP_PORT       = 8080
UP_PATH       = /canlog
UP_CHUNK_KB   = 1024
UP_RETRY_MAX  = 10
```

ファイルが無い場合は既定値で起動し、アップロードは無効のまま記録だけ動く。

> Wi-Fiのパスワードは平文でSDに載る。検証用の専用SSIDを使うこと。

### 受信側

検証用のサーバーを同梱している。SORACOM Harvest Files と同じ
「PUTしたパスにそのまま置く」挙動を模しているので、将来LTEへ切り替えても治具側は変わらない。

```bash
python3 tools/upload_server.py --dir ./received --port 8080
```

| オプション | 用途 |
|---|---|
| `--fail-rate 0.3` | 30%の確率で500を返す（再送処理の検証） |
| `--drop-rate 0.2` | 20%の確率で応答せず切断する |
| `--slow 50` | 1KBごとに50ms待つ（低速回線の模擬） |

### 届くファイル

```
received/canlog/JIG001/
    20260901_150227_LOG0006.txt        統計ファイル（先に送られる）
    20260901_150227_LOG0006_001.bin    ログ本体（1MBごとに分割）
    20260901_150227_LOG0006_002.bin
```

分割されたBINは連結してから `bin_to_blf.py` にかける。

```bash
python3 tools/merge_parts.py received/canlog/JIG001
```

### 設計上の要点

| 項目 | 内容 |
|---|---|
| **記録優先** | `ST_LOGGING` / `ST_DRAINING` では完全停止。開始ボタンで送信を即中断 |
| **統計ファイルを先に送る** | 数KBで「そのログが信用できるか」を遠隔判定できる |
| **1MBずつ分割** | 中断・切断時の再送単位を小さくする。固定費の実測から1MBが最適 |
| **冪等な再送** | 同一パスへの再送は上書き。何度リトライしても結果が同じ |
| **SDから消さない** | 送信済みでもカードに残す。SDが最後の砦 |
| **通信層は3関数に限定** | LTE化時は `upWifiUp` / `upWifiDown` / `upPutRange` の差し替えのみ |

詳細は [`docs/DESIGN.md`](docs/DESIGN.md) §7。

---

## PC側変換ツール

```bash
cd tools
pip install python-can

python bin_to_blf.py LOG0001.BIN -o LOG0001.blf
```

| オプション | 内容 |
|---|---|
| `-o` | 出力先。省略時は入力名 + `.blf` |
| `--start` | 計測開始の実時刻。省略時は「ヘッダの `unix_time` → それが0なら入力ファイルの更新時刻」の順 |
| `--tz-offset` | CANalyzer表示用の補正量[時間]。省略時はPCのタイムゾーン（日本なら+9） |
| `--utc` | 補正せずUTCのまま出力 |

> **タイムゾーン**: python-canはBLFヘッダへUTCを書くが、CANalyzerはそれを現地時刻として
> 表示するため、補正しないと9時間ずれる。既定で補正している（相対時刻は不変）。

BLF内のチャネル番号は 1 = CAN1 / 2 = CAN2 になり、CANalyzerの設定とそのまま一致する。

ASCモードで取得した場合は python-can 同梱のコンバータでも変換できる。

```bash
python -m can.logconvert LOG0001.ASC LOG0001.blf
```

---

## 設計上の要点

| 項目 | 内容 |
|---|---|
| **Listen-Only** | 両chともACKを返さず、バスに一切影響を与えない |
| **受信と書き込みの分離** | 受信タスクは打刻とリングへの積み込みのみ。SD書き込みは別コア |
| **リングバッファ** | SDカードの数百msの停止を吸収（実測では1024段中12段しか使用せず） |
| **バイナリ保存** | ASCテキストの約1/3のデータ量。SD速度の余裕を確保 |
| **縮退保存** | 異常時もそこまでのログと統計を必ず残す |
| **統計ファイル** | ログの信頼性を現地で判定できるようにする |
| **アップロードは待機中のみ** | 記録中はSDもCPUも奪わない。中断は1KBごとに判定 |
| **大物バッファはヒープ確保** | 通信スタックの静的領域圧迫を回避（v2.7） |

詳細な理由は [`docs/TECHNICAL.md`](docs/TECHNICAL.md)。

---

## 未了項目

**ファームウェア**

- [ ] RTCの校正（1号機は約+30ppm進む。`TRIM` で補正。[`docs/HARDWARE.md`](docs/HARDWARE.md) §11.6）
- [ ] RTC電池での時刻保持確認（電源断→復帰で `PWRFAIL=1` になること）
- [ ] ファイル自動分割（FAT32 4GB上限対策）
- [ ] **実バスでの干渉試験**（アップロード中に `ring dropped` が0のままか）
- [ ] 長時間連続試験（72時間以上。`freeheap` が単調減少しないこと）
- [ ] LTE（SIM7600JC-H）への置き換え
- [ ] 圧縮（gzip）の実装
- [ ] 遠隔コマンド（サーバー上のコマンドファイルをポーリング）
- [ ] 起動バナーと統計ファイルのバージョン文字列を `v2.7` へ更新

**ハードウェア（次版）**

- [ ] RTC水晶を **CL=6〜7pF品** へ変更（MCP7940Nは6〜9pFに最適化）
- [ ] 3.3Vバイパスコンデンサの配置割当確定（SDソケット直近を最優先）

**運用**

- [ ] 筐体設計
- [ ] 治具ラベル（「制御系CAN専用。高電圧バスに接続しないこと」）
