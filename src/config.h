/*
 * config.h — CANログ取得治具 設定  v2.5
 *
 * ターゲット: ESP32-WROOM-32E-N16 (arduino-esp32 2.0.17 / PlatformIO)
 *
 *   CH1 : ESP32内蔵TWAI  + ISO1042DWV     … 500 kbps（CANalyzer CAN1相当）
 *   CH2 : MCP2515-I/SO (HSPI) + ISO1042DWV … 250 kbps（CANalyzer CAN2相当）
 *   RTC : MCP7940N-I/SN (I2C) + 32.768kHz水晶
 *
 * 本ファイルの主要な設定値は #ifndef ガードを付けてあるため、
 * platformio.ini の build_flags から -D で上書きできる。
 *   例) build_flags = -DCH2_ENABLE=0 -DCH1_BITRATE=250000UL
 */
#pragma once

#include <stdint.h>

// ============================================================
// 保存形式
// ============================================================
//   LOG_FORMAT_BIN : 24byte固定長バイナリ（既定・推奨）
//                    SD負荷が約1/3。PC側で BIN → BLF を直接生成する
//   LOG_FORMAT_ASC : Vector ASCテキスト（現地でテキスト確認したいときのデバッグ用）
#define LOG_FORMAT_BIN 0
#define LOG_FORMAT_ASC 1
#ifndef LOG_FORMAT
#define LOG_FORMAT LOG_FORMAT_BIN
#endif

// ============================================================
// ビットレート（CANalyzer側のCAN1/CAN2設定に合わせる）
// ============================================================
#ifndef CH1_BITRATE
#define CH1_BITRATE 500000UL // 内蔵TWAI : 500k / 250k / 125k を選択可
#endif
#ifndef CH2_BITRATE
#define CH2_BITRATE 250000UL // MCP2515  : 500k / 250k / 125k を選択可
#endif
#ifndef CH2_ENABLE
#define CH2_ENABLE 1         // 0にするとCH1のみで動作（MCP2515未実装基板の検証用）
#endif

// ============================================================
// CH1: ESP32内蔵TWAI
// ============================================================
#define CAN1_TX_GPIO GPIO_NUM_21 // ISO1042 TXD（Listen-Onlyでは駆動されない）
#define CAN1_RX_GPIO GPIO_NUM_22 // ISO1042 RXD
#define TWAI_RX_QUEUE_LEN 128    // ドライバ側RXキュー段数

// ============================================================
// CH2: MCP2515（HSPI・SDとは別バス）
// ============================================================
#define MCP_SCK_GPIO   14
#define MCP_MOSI_GPIO  13
#define MCP_MISO_GPIO  35 // 入力専用GPIO。GPIO12(ストラップピン)を避けるため
#define MCP_CS_GPIO    4
#define MCP_INT_GPIO   34 // 入力専用GPIO。外部10kΩプルアップ必須
#define MCP_RESET_GPIO 26 // /RESET。10kΩでプルアップした上でGPIOからも駆動する
#define MCP_SPI_HZ     10000000UL
#define MCP_XTAL_HZ    16000000UL // 治具基板の水晶。16MHz固定（mcp2515.hの定数表と対応）

// ============================================================
// SDカード（VSPI）
// ============================================================
#define SD_CS_GPIO   5
#define SD_SCK_GPIO  18
#define SD_MISO_GPIO 19
#define SD_MOSI_GPIO 23
#define SD_SPI_HZ    20000000UL

// ============================================================
// RTC: MCP7940N-I/SN（I2C）
// ============================================================
#ifndef RTC_ENABLE
#define RTC_ENABLE 1         // 0にするとRTCを読まない（未実装基板の検証用）
#endif
#define RTC_SDA_GPIO 16
#define RTC_SCL_GPIO 17
#define RTC_I2C_HZ   400000UL
#define RTC_I2C_ADDR 0x6F    // MCP7940N（DS3231の0x68とは別なので注意）

// RTCが保持する時刻のUTCからのオフセット[秒]。
// 現地作業者が見やすいようRTCには「日本時間」を設定する運用とし、
// BINヘッダには常にUTCのUNIX時刻を格納する（PC側で正しく表示させるため）。
#ifndef RTC_UTC_OFFSET_SEC
#define RTC_UTC_OFFSET_SEC (9 * 3600) // JST = UTC+9
#endif

// ============================================================
// 操作系
// ============================================================
#define BTN_START_GPIO 32 // 押しボタン（GND短絡、内部プルアップ使用）
#define BTN_STOP_GPIO  33
#define LED_GPIO       25 // 状態表示LED（直列抵抗1kΩ程度必須）
#define BTN_DEBOUNCE_MS 30

#define LED_BLINK_LOGGING_MS 200 // 記録中：ゆっくり点滅
#define LED_BLINK_ERROR_MS   80  // 異常  ：高速点滅

// ============================================================
// バッファリング
// ============================================================
// リングバッファ段数（2のべき乗必須）。1段=24byte
//   CH1: 2048段 = 49KB ≒ 4,000 frame/s で約0.5秒分
//   CH2: 1024段 = 25KB ≒ 2,250 frame/s で約0.45秒分
// v2.7以降はヒープ確保なので、DRAMの静的領域(dram0_0_seg)の制約は受けない。
// 増やす場合は setup() の "buffers:" 行と STATUS の freeheap で余裕を確認すること。
#ifndef RING_SIZE_CH1
#define RING_SIZE_CH1 2048
#endif
#ifndef RING_SIZE_CH2
#define RING_SIZE_CH2 1024
#endif

// 2chのタイムスタンプ順序をそろえるための待ち時間[us]。
// 片方のリングだけにフレームがある場合、この時間だけ待ってから書き出す。
#define MERGE_LAG_US 5000

#define WRITE_CHUNK_BYTES 4096   // SDへの1回の書き出しサイズ
#define FLUSH_INTERVAL_MS 2000   // FAT更新周期（電源断時の被害を限定する）

// ============================================================
// アップロード機能（P6: Wi-Fi版）
// ============================================================
// SSID・送信先などの可変パラメータはSDカードの /CONFIG.TXT から読む。
// ここにあるのはコードの構造に関わる固定値のみ。
//
// 注) 0にすると WiFi.h ごとリンクされなくなり、ヒープが約40KB戻る。
//     アップロードを使わない現場向けビルドではこちらを選ぶ。
#ifndef UPLOAD_ENABLE
#define UPLOAD_ENABLE 1
#endif

#define UP_TASK_PRIO       3     // writerTask(5) より低くする
#define UP_TASK_CORE       0     // 受信タスク(core1)と分離する
#define UP_TASK_STACK      8192
#define UP_WIFI_TIMEOUT_MS 20000 // Wi-Fi接続のタイムアウト
#define UP_HTTP_TIMEOUT_MS 15000 // 1パートのHTTP応答待ち

// 待機中にアップロード動作を示すLEDパターン（1秒周期で2回の短点滅）
#define LED_UPLOAD_PERIOD_MS 1000
#define LED_UPLOAD_PULSE_MS  60

// ============================================================
// 停止シーケンスのタイミング
// ============================================================
// CH1受信タスクが twai_receive() でブロックする時間。
#define RX_TIMEOUT_MS 20
// 停止時に受信タスクが受信待ちから抜けるのを待つ時間。
// twai_driver_uninstall() より前に必ず抜けている必要があるため、
// RX_TIMEOUT_MS より確実に長くしておく。両者を変更する際は必ず連動させること。
#define DRAIN_WAIT_MS (RX_TIMEOUT_MS * 2 + 10)
