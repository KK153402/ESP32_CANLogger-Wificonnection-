/*
 * main.cpp — CANログ取得治具 ファームウェア v2.5（2ch同時記録 + RTC）
 *
 * ビルド : PlatformIO / espressif32@6.8.1 (arduino-esp32 2.0.17)
 *            pio run            … ビルド
 *            pio run -t upload  … 書き込み
 *            pio device monitor … シリアルモニタ (115200)
 *
 * 用途 : 現場で単独でCANログを取得するための治具。
 *        PCを接続せずにSDカードへ保存し、PC側でBLFへ変換する。
 *
 * 構成 : ESP32-WROOM-32E-N16
 *          CH1 = 内蔵TWAI (Listen-Only) + ISO1042DWV     … 500 kbps
 *          CH2 = MCP2515-I/SO (HSPI, Listen-Only) + ISO1042DWV … 250 kbps
 *          SD  = VSPI / RTC = MCP7940N (I2C)
 *
 * 設計方針
 *   - 受信タスクは「タイムスタンプ付与とリングへの積み込み」だけを行う
 *   - 保存形式の生成とSD書き込みは書き込みタスク（別コア）に分離する
 *   - CH1/CH2で独立したリングを持ち、書き込み時にタイムスタンプ順にマージする
 *   - 両chともListen-Only（ACKも返さない）でバスに一切影響を与えない
 *   - 異常が起きても「そこまでのログと統計ファイルを必ず残す」（縮退保存）
 *
 * 版歴
 *   v2.0  CH1+CH2同時記録、24byte固定長バイナリ保存を既定化
 *   v2.1  コードレビュー反映
 *           - 異常時にドライバとファイルが開放されず開始ボタンで復帰できない問題を修正
 *           - 異常時も残データ書き出し・クローズ・統計出力を行う縮退保存を追加
 *           - 停止シーケンスの待ち時間を RX_TIMEOUT_MS と連動（config.h）
 *           - PlatformIO化（can_logger_jig.ino → src/main.cpp）
 *   v2.2  基板確定に伴う対応
 *           - RTC(MCP7940N) から実時刻を読み BinHeader.unix_time へ格納
 *           - シリアルコマンド（SETTIME / GETTIME / STATUS）を追加
 *           - MCP2515 の /RESET をGPIOから駆動（ハードウェアリセット）
 *           - ASCヘッダの日時をRTCの実時刻から生成
 *   v2.3  実機ログ（LOG0002）の解析で判明した取りこぼしを修正
 *           - mcpIsr() で portYIELD_FROM_ISR() を行っておらず、CH2受信タスクの
 *             起床が次のティック（最大1ms）まで遅れていた。250kbpsでは
 *             RXB0/RXB1が埋まりきる時間で、周期メッセージの1.2%が欠落していた
 *           - /INT がLowの間は通知を待たずに読み出しを継続するようにした
 *             （MCP2515の/INTはレベル出力のためエッジを取りこぼす経路があった）
 *           - RTCの発振待ちを 200ms → 5s に延長し、時刻書き込みの成否と分離した
 *             （32.768kHz水晶は起動に数百ms〜数秒かかるため誤判定していた）
 *           - 電源投入直後にSD初期化が失敗しST_ERRORになる問題を修正
 *             （周辺デバイスの立ち上がり待ち300ms + SD 5回 / MCP2515 3回のリトライ）
 *   v2.4  v2.3で作り込んだ致命的なバグを修正
 *           - rxTaskMcp() で「記録中でない」判定が /INT のチェックより後ろにあり、
 *             CH2をバスに接続した待機状態では /INT がLowに張り付いて
 *             どこでもブロックせずに回り続けていた。優先度19・コア1固定のため
 *             優先度1の loopTask が飢餓状態になり、setup()の完走・シリアル受信・
 *             ボタン・LEDがすべて停止していた
 *           - フレームを1件も読めなかった場合の保険として vTaskDelay(1) を追加
 *   v2.7  DRAM不足(dram0_0_seg overflow)の対策
 *           - リングバッファと書き込みバッファを静的確保からヒープ確保へ変更した
 *             Wi-Fi/LTEスタックの静的バッファ(約40KB)が加わると、リング(約74KB)を
 *             .bssに置いたままではリンクが通らないため
 *           - 確保失敗時は FAULT_NO_MEMORY で ST_ERROR へ落とす
 *   v2.6  ログの自動アップロード（改良仕様書 v3.3 §17.2 の P6）
 *           - Wi-Fi経由でHTTP PUTする uploader.h を追加。待機中のみ動作する
 *           - SDカードの /CONFIG.TXT から設定を読む cfgfile.h を追加
 *           - 記録開始時にアップロードを中断させ、SDとWi-Fiを手放してから始める
 *           - 起動回数カウンタ（NVS）を追加
 *           - シリアルコマンド UPLOAD / UPSTAT / WIFI / RELOAD を追加
 *         ※ 通信層以外は本番と同じ構造にしてある。P7でWi-Fi部分を
 *            SIM7600のATコマンドへ差し替える。
 *   v2.5  査読（Fable）指摘の反映
 *           - STATUS に RTC の診断フラグ（OSCRUN/PWRFAIL/VBATEN/trim）を追加
 *           - VBATEN の書き込み結果を読み戻して検証するようにした
 *           - 記録開始に失敗したときヘッダだけの空BINが残らないようにした
 *           - ドレイン中にSD書き込みが失敗した場合も統計の exit を ABNORMAL にする
 *           - TRIM コマンドを追加（水晶の周波数誤差をデジタルトリミングで補正）
 *
 * 注) 本ファイルは .ino ではないため、Arduino IDEのプロトタイプ自動生成は働かない。
 *     全ての関数は呼び出し箇所より前に定義してあるので前方宣言は不要だが、
 *     関数を追加する際は定義順に注意すること。
 *
 * 詳細は docs/DESIGN.md および docs/HARDWARE.md を参照。
 */

#include <Arduino.h>
#include <new>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "driver/twai.h"
#include "esp_timer.h"

#include "config.h"
#include "ring_buffer.h"
#include "asc_format.h"
#if CH2_ENABLE
#include "mcp2515.h"
#endif
#if RTC_ENABLE
#include "mcp7940.h"
#endif
#if UPLOAD_ENABLE
#include "cfgfile.h"
#include "uploader.h"
#endif

// ============================================================
// バイナリファイルヘッダ（32byte・リトルエンディアン）
// ============================================================
struct BinHeader {
  char     magic[8];     // +0  "CANLOG02"
  uint16_t version;      // +8  2
  uint16_t record_size;  // +10 24
  uint32_t ch1_bitrate;  // +12
  uint32_t ch2_bitrate;  // +16 0 = CH2無効
  uint32_t unix_time;    // +20 計測開始のUNIX時刻(UTC)。0 = RTC無効/未設定
  uint64_t t0_us;        // +24 計測開始時刻(esp_timer基準)
};
static_assert(sizeof(BinHeader) == 32, "BinHeader must be 32 bytes");

// ============================================================
// 状態
// ============================================================
enum LoggerState : uint8_t {
  ST_IDLE,      // 待機（LED消灯）
  ST_LOGGING,   // 記録中（LED点滅）
  ST_DRAINING,  // 停止処理中（受信停止・残りを書き出し中）
  ST_STOPPED,   // 正常停止・SD取り外し可（LED点灯）
  ST_ERROR,     // 異常（LED高速点滅）
};
static volatile LoggerState g_state = ST_IDLE;

// ============================================================
// 異常要因
// ============================================================
// 記録中に異常を検出したタスクは g_fault を立てるだけにして、
// 後始末（縮退保存）は必ず loop() 側で行う。
enum FaultCode : uint8_t {
  FAULT_NONE = 0,
  FAULT_SD_INIT,
  FAULT_SD_OPEN,
  FAULT_SD_WRITE,
  FAULT_TWAI_INIT,
  FAULT_MCP_INIT,
  FAULT_BUS_OFF,
  FAULT_NO_FILENAME,
  FAULT_NO_MEMORY,
};
static volatile uint8_t g_fault = FAULT_NONE;

static const char* faultText(uint8_t f) {
  switch (f) {
    case FAULT_NONE:        return "none";
    case FAULT_SD_INIT:     return "SD card not found / mount failed";
    case FAULT_SD_OPEN:     return "cannot create log file on SD";
    case FAULT_SD_WRITE:    return "SD write failed (card full or bad contact)";
    case FAULT_TWAI_INIT:   return "CH1 TWAI driver init failed";
    case FAULT_MCP_INIT:    return "CH2 MCP2515 not responding";
    case FAULT_BUS_OFF:     return "CH1 bus off";
    case FAULT_NO_FILENAME: return "no free file name (LOG0001..LOG9999 all used)";
    case FAULT_NO_MEMORY:   return "cannot allocate ring buffers (reduce RING_SIZE_*)";
    default:                return "unknown";
  }
}

// ============================================================
// グローバル
// ============================================================
/*
 * リングバッファと書き込みバッファはヒープに置く。
 *
 * ESP32の静的データ(.bss)は dram0_0_seg という固定サイズの領域に収める必要があり、
 * ヒープとは別枠の上限がある。Wi-Fi/LTEを有効にすると通信スタックの静的バッファが
 * 約40KB積まれるため、リング(約74KB)を静的確保したままではリンクが通らない
 *   → "region `dram0_0_seg' overflowed"
 * ヒープは dram0_0_seg の外のDRAMも使えるので、大物をヒープへ移すと制約を回避できる。
 *
 * setup() でタスク生成より前に確保する。確保に失敗したらタスクを起動しない。
 */
static SpscRing<RING_SIZE_CH1>* g_ring1 = nullptr;  // CH1 (TWAI)    約49KB
#if CH2_ENABLE
static SpscRing<RING_SIZE_CH2>* g_ring2 = nullptr;  // CH2 (MCP2515)  約25KB
static SPIClass      g_hspi(HSPI);
static Mcp2515       g_mcp;
static TaskHandle_t  g_mcpTaskHandle = nullptr;
static volatile bool g_mcpReady = false;
static volatile bool g_mcpIsrOn = false;
#endif

#if RTC_ENABLE
static Mcp7940 g_rtc;
static bool   g_rtcOk = false;
#endif

#if UPLOAD_ENABLE
static AppCfg g_cfg;
#endif

static File              g_file;
static SemaphoreHandle_t g_fileMtx  = nullptr;
static volatile bool     g_flushReq = false;
static volatile bool     g_twaiOn   = false;

static volatile uint64_t g_t0_us     = 0;
static uint32_t          g_startUnix = 0;  // 記録開始時のUNIX時刻(UTC)。0 = 不明
static char g_logPath[16];
static char g_txtPath[16];

struct ChStats {
  uint32_t frames;   // リングに積めたフレーム数
  uint32_t dropped;  // リング満杯で捨てた数
  uint32_t missed;   // コントローラ段の溢れ（CH1=フレーム数 / CH2=検出イベント回数）
};
struct Stats {
  ChStats  ch1;
  ChStats  ch2;
  uint32_t bus_error_alerts;
  uint32_t sd_write_max_us;
  uint32_t sd_write_count;
  uint64_t bytes_written;
  uint32_t duration_ms;
};
static Stats g_stats;

// ============================================================
// 時刻ユーティリティ
// ============================================================
// UNIX時刻(UTC) → 現地時刻の文字列 "YYYY-MM-DD HH:MM:SS"
static void formatLocalTime(uint32_t utc, char* out, size_t len) {
  if (utc == 0) {
    snprintf(out, len, "unknown (RTC not set)");
    return;
  }
  int64_t t = static_cast<int64_t>(utc) + RTC_UTC_OFFSET_SEC;
  const int64_t days = t / 86400;
  int64_t secs = t % 86400;
  // 民用日付へ戻す（days_from_civil の逆変換）
  int64_t z = days + 719468;
  const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const unsigned doe = static_cast<unsigned>(z - era * 146097);
  const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const int64_t  y   = static_cast<int64_t>(yoe) + era * 400;
  const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const unsigned mp  = (5 * doy + 2) / 153;
  const unsigned d   = doy - (153 * mp + 2) / 5 + 1;
  const unsigned m   = mp + (mp < 10 ? 3 : -9);
  snprintf(out, len, "%04lld-%02u-%02u %02lld:%02lld:%02lld",
           static_cast<long long>(y + (m <= 2)), m, d,
           static_cast<long long>(secs / 3600),
           static_cast<long long>((secs % 3600) / 60),
           static_cast<long long>(secs % 60));
}

#if LOG_FORMAT == LOG_FORMAT_ASC
// ASCヘッダ用 "Thu Aug 9 14:05:00.000 2026" 形式
static void formatAscDate(uint32_t utc, char* out, size_t len) {
  static const char* kWday[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
  static const char* kMon[]  = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  if (utc == 0) {
    snprintf(out, len, "Thu Jan 1 00:00:00.000 1970");
    return;
  }
  char tmp[32];
  formatLocalTime(utc, tmp, sizeof(tmp));
  int Y, M, D, h, mi, s;
  if (sscanf(tmp, "%d-%d-%d %d:%d:%d", &Y, &M, &D, &h, &mi, &s) != 6) {
    snprintf(out, len, "Thu Jan 1 00:00:00.000 1970");
    return;
  }
#if RTC_ENABLE
  const uint8_t w = Mcp7940::weekdayFromCivil(static_cast<uint16_t>(Y),
                                              static_cast<uint8_t>(M),
                                              static_cast<uint8_t>(D));
#else
  const uint8_t w = 4;
#endif
  snprintf(out, len, "%s %s %d %02d:%02d:%02d.000 %d",
           kWday[w % 7], kMon[(M - 1) % 12], D, h, mi, s, Y);
}
#endif

// ============================================================
// LED
// ============================================================
static void updateLed() {
  const uint32_t now = millis();

#if UPLOAD_ENABLE
  // 待機中にアップロードしている間は「1秒周期で2回の短点滅」で区別する。
  // 消灯（何もしていない）と見分けがつかないと、現場で
  // 「治具が生きているのか」が判断できないため。
  if (uploaderActive() && (g_state == ST_IDLE || g_state == ST_STOPPED)) {
    const uint32_t t = now % LED_UPLOAD_PERIOD_MS;
    const bool on = (t < LED_UPLOAD_PULSE_MS) ||
                    (t >= LED_UPLOAD_PULSE_MS * 2 && t < LED_UPLOAD_PULSE_MS * 3);
    digitalWrite(LED_GPIO, on ? HIGH : LOW);
    return;
  }
#endif

  switch (g_state) {
    case ST_IDLE:      digitalWrite(LED_GPIO, LOW); break;
    case ST_LOGGING:
    case ST_DRAINING:  digitalWrite(LED_GPIO, ((now / LED_BLINK_LOGGING_MS) & 1) ? HIGH : LOW); break;
    case ST_STOPPED:   digitalWrite(LED_GPIO, HIGH); break;
    case ST_ERROR:     digitalWrite(LED_GPIO, ((now / LED_BLINK_ERROR_MS) & 1) ? HIGH : LOW); break;
  }
}

// ============================================================
// ボタン（GND短絡・内部プルアップ・押下エッジ検出）
// ============================================================
struct Button {
  uint8_t  pin;
  bool     stable;
  bool     last;
  uint32_t lastChange;
};
static Button g_btnStart{BTN_START_GPIO, true, true, 0};
static Button g_btnStop{BTN_STOP_GPIO, true, true, 0};

static bool buttonPressed(Button& b) {
  const bool raw = (digitalRead(b.pin) != LOW);  // HIGH = 離されている
  const uint32_t now = millis();
  if (raw != b.last) {
    b.last = raw;
    b.lastChange = now;
    return false;
  }
  if (now - b.lastChange < BTN_DEBOUNCE_MS) return false;
  if (raw == b.stable) return false;
  b.stable = raw;
  return (raw == false);  // 押された瞬間のみtrue
}

// ============================================================
// CH1 受信タスク（内蔵TWAI / core1）
// ============================================================
static void rxTaskTwai(void*) {
  for (;;) {
    if (g_state != ST_LOGGING || !g_twaiOn) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(RX_TIMEOUT_MS)) != ESP_OK) continue;

    CanRec rec;
    rec.ts_us    = static_cast<uint64_t>(esp_timer_get_time());
    rec.id       = msg.identifier;
    rec.dlc      = (msg.data_length_code > 8) ? 8 : msg.data_length_code;
    rec.flags    = 0;
    rec.channel  = 1;
    rec.reserved = 0;
    if (msg.extd) rec.flags |= CANREC_FLAG_EXTENDED;
    if (msg.rtr)  rec.flags |= CANREC_FLAG_RTR;
    memcpy(rec.data, msg.data, 8);

    if (g_ring1->push(rec)) g_stats.ch1.frames++;
  }
}

// ============================================================
// CH2 受信タスク（MCP2515 / core1・INT駆動）
// ============================================================
#if CH2_ENABLE
/*
 * MCP2515 の /INT ハンドラ。
 *
 * 重要: vTaskNotifyGiveFromISR() の第2引数に nullptr を渡してはいけない。
 *       渡さないとISRから抜けるときにタスク切り替えが要求されず、
 *       通知したタスクが次のティック（1ms）まで走らない。
 *       250kbpsでは8byteフレームが約444us間隔で来るため、1ms待つと
 *       RXB0/RXB1の2段がちょうど埋まり、3フレーム目で取りこぼす。
 *       （v2.2実測: 2件まとめ読み1311回・3件44回、周期メッセージの1.2%が欠落）
 */
static void IRAM_ATTR mcpIsr() {
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  if (g_mcpTaskHandle) {
    vTaskNotifyGiveFromISR(g_mcpTaskHandle, &higherPriorityTaskWoken);
  }
  if (higherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

static void rxTaskMcp(void*) {
  for (;;) {
    // 記録中でなければ必ず眠る。
    // このタスクは優先度19でコア1に固定されており、ここでブロックしないと
    // 優先度1の loopTask（setup/loop）を飢餓状態にしてしまう。
    // v2.3初版はこの判定が /INT のチェックより後ろにあったため、
    // 「CH2をバスに接続した状態で待機中」だと /INT がLowに張り付き、
    // どこでもブロックせずに回り続けてシリアルもボタンもLEDも止まった。
    if (g_state != ST_LOGGING || !g_mcpReady) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // /INT がLowの間は受信バッファに未読フレームが残っている。
    // MCP2515の/INTはレベル出力なので、ドレイン中に届いたフレームでは
    // 新しい立ち下がりエッジが出ない。眠る前にピンの状態を直接見て、
    // まだLowなら通知を待たずにそのまま読み出しを続ける。
    if (digitalRead(MCP_INT_GPIO) == HIGH) {
      // 取りこぼし保険として5msでタイムアウトして必ず1回覗く
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
    }

    // 受信バッファが空になるまで読み切る
    uint32_t got = 0;
    for (;;) {
      CanRec rec;
      rec.ts_us    = static_cast<uint64_t>(esp_timer_get_time());
      rec.channel  = 2;
      rec.reserved = 0;
      const Mcp2515::Result r = g_mcp.readFrame(rec.id, rec.dlc, rec.flags, rec.data);
      if (r != Mcp2515::OK) break;
      if (g_ring2->push(rec)) g_stats.ch2.frames++;
      got++;
    }

    // /INT がLowのままなのにフレームが読めない異常時に
    // ビジーループへ落ちないようにする保険
    if (got == 0) vTaskDelay(1);
  }
}
#endif

// ============================================================
// SD書き込みタスク（core0）
// ============================================================
static char*    g_wbuf = nullptr;  // WRITE_CHUNK_BYTES + ASC_LINE_MAX（setup()で確保）
static size_t   g_wlen = 0;
static uint32_t g_lastFlushMs = 0;

static void flushWriteBuffer() {
  if (g_wlen == 0) return;

  // SD書き込みが一度失敗している状態では以降の書き込みを試さない。
  if (g_fault == FAULT_SD_WRITE) {
    g_wlen = 0;
    return;
  }

  if (xSemaphoreTake(g_fileMtx, pdMS_TO_TICKS(2000)) != pdTRUE) return;
  if (g_file) {
    const uint32_t t0 = micros();
    const size_t wrote = g_file.write(reinterpret_cast<uint8_t*>(g_wbuf), g_wlen);
    const uint32_t dt = micros() - t0;
    if (dt > g_stats.sd_write_max_us) g_stats.sd_write_max_us = dt;
    g_stats.sd_write_count++;
    g_stats.bytes_written += wrote;

    // 異常はここで確定させず要因だけ立てる。後始末は loop() が行う
    if (wrote != g_wlen) g_fault = FAULT_SD_WRITE;

    if (millis() - g_lastFlushMs >= FLUSH_INTERVAL_MS) {
      g_file.flush();
      g_lastFlushMs = millis();
    }
  }
  xSemaphoreGive(g_fileMtx);
  g_wlen = 0;
}

/*
 * CH1/CH2のリングからタイムスタンプの古い方を取り出す。
 * 片方のリングだけにフレームがある場合、あとから他chの古いフレームが
 * 積まれる可能性があるため MERGE_LAG_US だけ待ってから確定させる。
 * flushAll=true（停止処理中）のときは待たずに全部吐き出す。
 */
static bool popMerged(CanRec& out, bool flushAll) {
  uint64_t t1 = 0;
  const bool h1 = g_ring1->peekTs(t1);

#if CH2_ENABLE
  uint64_t t2 = 0;
  const bool h2 = g_ring2->peekTs(t2);
  if (!h1 && !h2) return false;
  if (h1 && h2) return (t1 <= t2) ? g_ring1->pop(out) : g_ring2->pop(out);

  const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
  if (h1) {
    if (!flushAll && (now - t1) < MERGE_LAG_US) return false;
    return g_ring1->pop(out);
  }
  if (!flushAll && (now - t2) < MERGE_LAG_US) return false;
  return g_ring2->pop(out);
#else
  (void)flushAll;
  if (!h1) return false;
  return g_ring1->pop(out);
#endif
}

static void writerTask(void*) {
  for (;;) {
    const bool draining = (g_state == ST_DRAINING);
    const bool active   = (g_state == ST_LOGGING || draining);
    if (!active) {
      if (g_flushReq) { flushWriteBuffer(); g_flushReq = false; }
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    CanRec rec;
    uint32_t popped = 0;
    while (g_wlen < WRITE_CHUNK_BYTES && popMerged(rec, draining)) {
#if LOG_FORMAT == LOG_FORMAT_ASC
      g_wlen += formatAscLine(&g_wbuf[g_wlen], rec, g_t0_us);
#else
      memcpy(&g_wbuf[g_wlen], &rec, sizeof(CanRec));
      g_wlen += sizeof(CanRec);
#endif
      popped++;
    }

    if (g_wlen >= WRITE_CHUNK_BYTES) {
      flushWriteBuffer();
    } else if (g_flushReq) {
      flushWriteBuffer();
      g_flushReq = false;
    } else if (popped == 0) {
      vTaskDelay(1);  // 到着待ち
    }
  }
}

// ============================================================
// 後始末（異常時・起動時の防御用）
//   どの状態から呼ばれても安全に「何も掴んでいない状態」へ戻す。
// ============================================================
static void hardCleanup() {
#if CH2_ENABLE
  if (g_mcpIsrOn) {
    detachInterrupt(digitalPinToInterrupt(MCP_INT_GPIO));
    g_mcpIsrOn = false;
  }
  g_mcpReady = false;
#endif

  if (g_twaiOn) {
    twai_stop();
    twai_driver_uninstall();
    g_twaiOn = false;
  }

  if (xSemaphoreTake(g_fileMtx, pdMS_TO_TICKS(3000)) == pdTRUE) {
    if (g_file) {
      // 記録が始まる前に失敗した場合、ヘッダだけの空ファイルが残って
      // 番号が欠番になる。中身が無いものは消しておく。
      g_file.flush();
      const size_t written = g_file.size();
      g_file.close();
      if (written <= sizeof(BinHeader) && g_logPath[0] != '\0') {
        SD.remove(g_logPath);
        SD.remove(g_txtPath);
      }
    }
    xSemaphoreGive(g_fileMtx);
  }
  g_wlen = 0;
}

// ============================================================
// ファイル名の採番
// ============================================================
static bool buildNextFileNames() {
#if LOG_FORMAT == LOG_FORMAT_ASC
  const char* ext = "ASC";
#else
  const char* ext = "BIN";
#endif
  for (uint16_t i = 1; i <= 9999; i++) {
    snprintf(g_logPath, sizeof(g_logPath), "/LOG%04u.%s", i, ext);
    snprintf(g_txtPath, sizeof(g_txtPath), "/LOG%04u.TXT", i);
    // 統計ファイルだけ残っているケースを踏まないよう両方を確認する
    if (!SD.exists(g_logPath) && !SD.exists(g_txtPath)) return true;
  }
  return false;
}

// ============================================================
// 記録開始
// ============================================================
static bool startLogging() {
  // バッファ確保に失敗している場合は何もしない（setup()でタスクも起動していない）
  if (!g_ring1 || !g_wbuf
#if CH2_ENABLE
      || !g_ring2
#endif
  ) {
    g_fault = FAULT_NO_MEMORY;
    return false;
  }

#if UPLOAD_ENABLE
  // アップロードが走っているとSDハンドルを掴んだままなので、
  // SD.end() / SD.begin() より前に必ず手放させる。
  // Wi-Fiも落とす。記録中はCPUもSDもCAN受信に使い切る。
  uploaderRequestStop();
  if (!uploaderWaitIdle(3000)) {
    Serial.println("[WARN] uploader did not stop in time. proceeding anyway.");
  }
  uploaderWifiOff();
#endif

  // 直前が異常終了でも確実に開始できるよう、まず掴んでいるものを全て開放する
  hardCleanup();
  g_fault = FAULT_NONE;

  // --- 実時刻の取得（失敗しても記録は続行する） ---
  g_startUnix = 0;
#if RTC_ENABLE
  if (g_rtcOk) {
    g_startUnix = g_rtc.readUnixTimeUtc();
    if (g_startUnix == 0) {
      Serial.println("[WARN] RTC time is not set. use SETTIME command.");
    }
  }
#endif

  // --- SD ---
  SD.end();
  bool sdOk = false;
  for (uint8_t i = 0; i < 3; i++) {
    if (SD.begin(SD_CS_GPIO, SPI, SD_SPI_HZ)) { sdOk = true; break; }
    SD.end();
    delay(100);
  }
  if (!sdOk) {
    Serial.println("[ERR] SD.begin failed");
    g_fault = FAULT_SD_INIT;
    return false;
  }
  if (!buildNextFileNames()) {
    Serial.println("[ERR] no free file name");
    g_fault = FAULT_NO_FILENAME;
    return false;
  }
  g_file = SD.open(g_logPath, FILE_WRITE);
  if (!g_file) {
    Serial.printf("[ERR] cannot open %s\n", g_logPath);
    g_fault = FAULT_SD_OPEN;
    return false;
  }

  // --- CH1: TWAI (Listen-Only) ---
  twai_general_config_t gcfg =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN1_TX_GPIO, CAN1_RX_GPIO, TWAI_MODE_LISTEN_ONLY);
  gcfg.rx_queue_len   = TWAI_RX_QUEUE_LEN;
  gcfg.tx_queue_len   = 0;
  gcfg.alerts_enabled = TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_BUS_ERROR |
                        TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF |
                        TWAI_ALERT_RX_FIFO_OVERRUN;

  // 三項演算子で複合リテラルを使うとビルドが通らない環境があるためif/elseで代入
  twai_timing_config_t tcfg;
  if (CH1_BITRATE == 250000UL) {
    twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
    tcfg = t;
  } else if (CH1_BITRATE == 125000UL) {
    twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS();
    tcfg = t;
  } else {
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    tcfg = t;
  }
  twai_filter_config_t fcfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&gcfg, &tcfg, &fcfg) != ESP_OK) {
    Serial.println("[ERR] twai_driver_install failed");
    g_fault = FAULT_TWAI_INIT;
    hardCleanup();
    return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("[ERR] twai_start failed");
    g_fault = FAULT_TWAI_INIT;
    twai_driver_uninstall();
    hardCleanup();
    return false;
  }
  g_twaiOn = true;

  // --- CH2: MCP2515 (Listen-Only) ---
#if CH2_ENABLE
  if (!g_mcp.begin(&g_hspi, MCP_CS_GPIO, CH2_BITRATE, MCP_SPI_HZ, MCP_RESET_GPIO)) {
    Serial.println("[ERR] MCP2515 init failed (CH2)");
    g_fault = FAULT_MCP_INIT;
    hardCleanup();
    return false;
  }
  g_mcpReady = true;
  attachInterrupt(digitalPinToInterrupt(MCP_INT_GPIO), mcpIsr, FALLING);
  g_mcpIsrOn = true;
#endif

  // --- 初期化 ---
  memset(&g_stats, 0, sizeof(g_stats));
  g_ring1->reset();
#if CH2_ENABLE
  g_ring2->reset();
#endif
  g_wlen        = 0;
  g_lastFlushMs = millis();
  g_t0_us       = static_cast<uint64_t>(esp_timer_get_time());

#if LOG_FORMAT == LOG_FORMAT_ASC
  char ascDate[40];
  formatAscDate(g_startUnix, ascDate, sizeof(ascDate));
  g_file.printf("date %s\n", ascDate);
  g_file.print("base hex  timestamps absolute\n");
  g_file.print("internal events logged\n");
  g_file.printf("Begin Triggerblock %s\n", ascDate);
  g_file.print("   0.000000 Start of measurement\n");
#else
  BinHeader h;
  memset(&h, 0, sizeof(h));
  memcpy(h.magic, "CANLOG02", 8);
  h.version     = 2;
  h.record_size = sizeof(CanRec);
  h.ch1_bitrate = CH1_BITRATE;
  h.ch2_bitrate = CH2_ENABLE ? CH2_BITRATE : 0UL;
  h.unix_time   = g_startUnix;  // 0ならPC側がファイル更新時刻へフォールバックする
  h.t0_us       = g_t0_us;
  g_file.write(reinterpret_cast<uint8_t*>(&h), sizeof(h));
#endif
  g_file.flush();

  g_state = ST_LOGGING;

  char ts[40];
  formatLocalTime(g_startUnix, ts, sizeof(ts));
  Serial.printf("[OK] logging started: %s  start=%s  (CH1=%lu bps, CH2=%lu bps)\n",
                g_logPath, ts, (unsigned long)CH1_BITRATE,
                (unsigned long)(CH2_ENABLE ? CH2_BITRATE : 0UL));
  return true;
}

// ============================================================
// 統計ファイル
// ============================================================
static void writeStatsFile(bool faulted) {
  File f = SD.open(g_txtPath, FILE_WRITE);
  if (!f) {
    Serial.println("[ERR] cannot write stats file");
    return;
  }
  char ts[40];
  formatLocalTime(g_startUnix, ts, sizeof(ts));

  f.printf("CAN Logger Jig v2.5\n");
  f.printf("log file        : %s\n", g_logPath);
#if LOG_FORMAT == LOG_FORMAT_ASC
  f.printf("format          : Vector ASC (text)\n");
#else
  f.printf("format          : CANLOG02 binary (32B header + 24B records)\n");
#endif
  f.printf("start time      : %s (local)\n", ts);
  f.printf("CH1 bitrate     : %lu bps\n", (unsigned long)CH1_BITRATE);
  f.printf("CH2 bitrate     : %lu bps\n", (unsigned long)(CH2_ENABLE ? CH2_BITRATE : 0UL));
  f.printf("duration        : %lu ms\n", (unsigned long)g_stats.duration_ms);
  f.printf("exit            : %s\n", faulted ? "ABNORMAL" : "normal (stop button)");
  if (faulted) f.printf("fault           : %s\n", faultText(g_fault));

  f.printf("\n-- CH1 (internal TWAI) --\n");
  f.printf("frames logged   : %lu\n", (unsigned long)g_stats.ch1.frames);
  f.printf("ring dropped    : %lu   (frames)\n", (unsigned long)g_stats.ch1.dropped);
  f.printf("ctrl missed     : %lu   (frames)\n", (unsigned long)g_stats.ch1.missed);
  f.printf("ring max used   : %lu / %lu\n", (unsigned long)g_ring1->maxUsed(),
           (unsigned long)g_ring1->capacity());
#if CH2_ENABLE
  f.printf("\n-- CH2 (MCP2515) --\n");
  f.printf("frames logged   : %lu\n", (unsigned long)g_stats.ch2.frames);
  f.printf("ring dropped    : %lu   (frames)\n", (unsigned long)g_stats.ch2.dropped);
  // EFLGの200msポーリングで検出した回数。落ちたフレーム数ではない
  f.printf("ctrl ovf events : %lu   (events, not frames)\n", (unsigned long)g_stats.ch2.missed);
  f.printf("ring max used   : %lu / %lu\n", (unsigned long)g_ring2->maxUsed(),
           (unsigned long)g_ring2->capacity());
#endif
  f.printf("\n-- SD --\n");
  f.printf("bus err alerts  : %lu\n", (unsigned long)g_stats.bus_error_alerts);
  f.printf("sd write count  : %lu\n", (unsigned long)g_stats.sd_write_count);
  f.printf("sd write max    : %lu us\n", (unsigned long)g_stats.sd_write_max_us);
  f.printf("bytes written   : %llu\n", (unsigned long long)g_stats.bytes_written);

  const bool loss = g_stats.ch1.dropped || g_stats.ch1.missed ||
                    g_stats.ch2.dropped || g_stats.ch2.missed;
  f.printf("\n");
  if (faulted) {
    f.printf("*** WARNING: abnormal exit. the log may be truncated. ***\n");
  }
  if (loss) {
    f.printf("*** WARNING: frame loss detected. ***\n");
  }
  if (g_stats.ch1.frames == 0) {
    f.printf("*** WARNING: CH1 received nothing. check bitrate/wiring. ***\n");
  }
#if CH2_ENABLE
  if (g_stats.ch2.frames == 0) {
    f.printf("*** WARNING: CH2 received nothing. check bitrate/wiring. ***\n");
  }
#endif
  if (g_startUnix == 0) {
    f.printf("*** NOTE: RTC time unavailable. use --start option on PC side. ***\n");
  }
  if (!faulted && !loss && g_stats.ch1.frames > 0
#if CH2_ENABLE
      && g_stats.ch2.frames > 0
#endif
  ) {
    f.printf("result          : OK\n");
  }
  f.close();
}

// ============================================================
// 記録の終了処理（正常停止／異常時の縮退保存の共通経路）
//   必ず loop() から呼ぶこと（writerTaskの完了を待つため）
// ============================================================
static void finalizeLogging(bool faulted) {
  const uint64_t tEnd = static_cast<uint64_t>(esp_timer_get_time());
  g_stats.duration_ms = static_cast<uint32_t>((tEnd - g_t0_us) / 1000ULL);

  // SD書き込みが壊れている場合は書き出し待ちをしても無駄なのでスキップする
  const bool sdBroken = (g_fault == FAULT_SD_WRITE);

  // 1) 受信を止める（writerTaskは書き出しを継続）
  g_state = ST_DRAINING;
  vTaskDelay(pdMS_TO_TICKS(DRAIN_WAIT_MS));

  // 2) コントローラ停止
#if CH2_ENABLE
  if (g_mcpIsrOn) {
    detachInterrupt(digitalPinToInterrupt(MCP_INT_GPIO));
    g_mcpIsrOn = false;
  }
  g_mcpReady = false;
#endif
  if (g_twaiOn) {
    twai_stop();
    twai_driver_uninstall();
    g_twaiOn = false;
  }

  if (!sdBroken) {
    // 3) リングが空になるまで待つ
    const uint32_t tw = millis();
    for (;;) {
#if CH2_ENABLE
      if (g_ring1->empty() && g_ring2->empty()) break;
#else
      if (g_ring1->empty()) break;
#endif
      if (millis() - tw >= 5000) break;
      updateLed();
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 4) 書き込みバッファを吐き出させる
    g_flushReq = true;
    const uint32_t tf = millis();
    while (g_flushReq && (millis() - tf) < 3000) {
      updateLed();
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }

  // 5) フッタとクローズ
  if (xSemaphoreTake(g_fileMtx, pdMS_TO_TICKS(3000)) == pdTRUE) {
    if (g_file) {
#if LOG_FORMAT == LOG_FORMAT_ASC
      if (!sdBroken) g_file.print("End TriggerBlock\n");
#endif
      g_file.flush();
      g_file.close();
    }
    xSemaphoreGive(g_fileMtx);
  }

  // 6) 統計ファイル（異常時こそ「何が起きたか」を残すことが重要）
  g_stats.ch1.dropped = g_ring1->dropped();
#if CH2_ENABLE
  g_stats.ch2.dropped = g_ring2->dropped();
#endif
  // ドレイン中にSD書き込みが失敗した場合も異常終了として記録する
  writeStatsFile(faulted || g_fault != FAULT_NONE);

  // 7) 状態確定。正常終了ならここで初めてLED点灯（＝SD取り外し可）にする
  g_state = (faulted || g_fault != FAULT_NONE) ? ST_ERROR : ST_STOPPED;

  Serial.printf("[%s] finished. CH1 %lu(drop %lu) / CH2 %lu(drop %lu)\n",
                faulted ? "ERR" : "OK",
                (unsigned long)g_stats.ch1.frames, (unsigned long)g_stats.ch1.dropped,
                (unsigned long)g_stats.ch2.frames, (unsigned long)g_stats.ch2.dropped);
  if (faulted) Serial.printf("      fault: %s\n", faultText(g_fault));

#if UPLOAD_ENABLE
  // 今書き終えたファイルを拾わせる。
  // 走査カーソルを戻すのは、異常終了で欠番が生じた場合に取りこぼさないため。
  uploaderResume();
  uploaderRequestScan();
#endif
}

// ============================================================
// コントローラの異常監視
// ============================================================
static void pollControllerStatus() {
  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
    if (alerts & (TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_RX_FIFO_OVERRUN)) {
      twai_status_info_t st;
      if (twai_get_status_info(&st) == ESP_OK) g_stats.ch1.missed = st.rx_missed_count;
    }
    if (alerts & TWAI_ALERT_BUS_ERROR) g_stats.bus_error_alerts++;
    if (alerts & TWAI_ALERT_BUS_OFF) {
      Serial.println("[ERR] CH1 bus off");
      g_fault = FAULT_BUS_OFF;  // 後始末は loop() が行う
    }
  }
#if CH2_ENABLE
  static uint32_t lastEflgMs = 0;
  if (g_mcpReady && (millis() - lastEflgMs >= 200)) {
    lastEflgMs = millis();
    if (g_mcp.checkAndClearOverflow()) g_stats.ch2.missed++;
  }
#endif
}

// ============================================================
// シリアルコマンド
//   SETTIME YYYY-MM-DD HH:MM:SS   RTCに現地時刻を設定する
//   GETTIME                       現在のRTC時刻を表示する
//   STATUS                        状態を表示する
//   HELP                          コマンド一覧
// ============================================================
static void printStatus() {
  char ts[40];
#if RTC_ENABLE
  formatLocalTime(g_rtcOk ? g_rtc.readUnixTimeUtc() : 0, ts, sizeof(ts));
#else
  formatLocalTime(0, ts, sizeof(ts));
#endif
  const char* st = "unknown";
  switch (g_state) {
    case ST_IDLE:     st = "IDLE";     break;
    case ST_LOGGING:  st = "LOGGING";  break;
    case ST_DRAINING: st = "DRAINING"; break;
    case ST_STOPPED:  st = "STOPPED";  break;
    case ST_ERROR:    st = "ERROR";    break;
  }
  Serial.printf("state=%s  rtc=%s  fault=%s  freeheap=%lu\n",
                st, ts, faultText(g_fault), (unsigned long)ESP.getFreeHeap());

#if RTC_ENABLE
  // RTCの診断フラグ。電源断→復帰テストで失敗したときの切り分け用。
  //   OSCRUN=0 → 発振していない（SETTIME未実行か水晶の問題）
  //   VBATEN=0 → バックアップ無効。電源断で時刻が消える
  //   PWRFAIL=1 → 電源断からの復帰履歴あり（正常な表示）
  if (g_rtcOk) {
    const uint8_t wk = g_rtc.rawWkday();
    int8_t trim = 0;
    g_rtc.readTrim(trim);
    Serial.printf("rtc flags: OSCRUN=%d PWRFAIL=%d VBATEN=%d (RTCWKDAY=0x%02X)  trim=%+d (%+.1f ppm)%s\n",
                  (wk >> 5) & 1, (wk >> 4) & 1, (wk >> 3) & 1, wk,
                  static_cast<int>(trim), trim * 1.017,
                  g_rtc.vbatenFailed() ? "  [VBATEN write failed]" : "");
  } else {
    Serial.println("rtc flags: not available (RTC not responding)");
  }
#endif

#if UPLOAD_ENABLE
  uploaderPrintStatus();
#endif
}

static void handleCommand(char* line) {
  // 前後の空白を落とす
  while (*line == ' ') line++;
  size_t n = strlen(line);
  while (n > 0 && (line[n - 1] == ' ' || line[n - 1] == '\r')) line[--n] = '\0';
  if (n == 0) return;

  if (strncasecmp(line, "SETTIME", 7) == 0) {
#if RTC_ENABLE
    if (!g_rtcOk) { Serial.println("[ERR] RTC not present"); return; }
    if (g_state == ST_LOGGING) { Serial.println("[ERR] stop logging first"); return; }
    int Y, M, D, h, mi, s;
    if (sscanf(line + 7, "%d-%d-%d %d:%d:%d", &Y, &M, &D, &h, &mi, &s) != 6) {
      Serial.println("[ERR] usage: SETTIME YYYY-MM-DD HH:MM:SS  (local time)");
      return;
    }
    if (Y < 2000 || Y > 2099 || M < 1 || M > 12 || D < 1 || D > 31 ||
        h > 23 || mi > 59 || s > 59) {
      Serial.println("[ERR] value out of range");
      return;
    }
    Mcp7940::DateTime dt{static_cast<uint16_t>(Y), static_cast<uint8_t>(M),
                         static_cast<uint8_t>(D), static_cast<uint8_t>(h),
                         static_cast<uint8_t>(mi), static_cast<uint8_t>(s)};
    if (!g_rtc.setDateTime(dt)) {
      // ここに来るのはI2C書き込み自体が失敗した場合（配線・プルアップ・はんだ）
      Serial.println("[ERR] RTC write failed (I2C)");
      return;
    }

    // 32.768kHz水晶は起動が遅い。最大5秒待つ
    Serial.print("waiting for oscillator");
    const bool osc = g_rtc.waitOscillatorStart(5000);
    Serial.println();

    if (!osc) {
      Serial.println("[ERR] oscillator did not start within 5 s.");
      Serial.println("      check the 32.768kHz crystal and its load caps (C27/C28).");
      Serial.println("      MCP7940N is optimized for CL=6-9pF crystals.");
      Serial.println("      check the crystal, load caps and flux residue around it.");
      return;
    }
    char ts[40];
    formatLocalTime(g_rtc.readUnixTimeUtc(), ts, sizeof(ts));
    Serial.printf("[OK] RTC set to %s (local)\n", ts);
#else
    Serial.println("[ERR] built without RTC support");
#endif
    return;
  }

  if (strcasecmp(line, "GETTIME") == 0) {
    char ts[40];
#if RTC_ENABLE
    formatLocalTime(g_rtcOk ? g_rtc.readUnixTimeUtc() : 0, ts, sizeof(ts));
#else
    formatLocalTime(0, ts, sizeof(ts));
#endif
    Serial.printf("RTC: %s (local, UTC%+d)\n", ts, RTC_UTC_OFFSET_SEC / 3600);
    return;
  }

  if (strncasecmp(line, "TRIM", 4) == 0) {
#if RTC_ENABLE
    if (!g_rtcOk) { Serial.println("[ERR] RTC not present"); return; }
    int v;
    if (sscanf(line + 4, "%d", &v) != 1) {
      int8_t cur = 0;
      g_rtc.readTrim(cur);
      Serial.printf("current trim = %+d (%+.1f ppm)\n", static_cast<int>(cur), cur * 1.017);
      Serial.println("usage: TRIM <-127..127>   (negative = slow the clock down)");
      Serial.println("  step = 1.017 ppm, range +-129 ppm");
      Serial.println("  if the clock gains N sec/day: TRIM -(N/86400*1e6/1.017)");
      return;
    }
    if (v < -127 || v > 127) { Serial.println("[ERR] range is -127..127"); return; }
    if (g_rtc.setTrim(static_cast<int8_t>(v))) {
      Serial.printf("[OK] trim = %+d (%+.1f ppm)\n", v, v * 1.017);
    } else {
      Serial.println("[ERR] trim write failed");
    }
#else
    Serial.println("[ERR] built without RTC support");
#endif
    return;
  }

  if (strcasecmp(line, "STATUS") == 0) { printStatus(); return; }

#if UPLOAD_ENABLE
  if (strncasecmp(line, "UPLOAD", 6) == 0) {
    if (g_state == ST_LOGGING || g_state == ST_DRAINING) {
      Serial.println("[ERR] stop logging first");
      return;
    }
    char arg[16] = {0};
    if (sscanf(line + 6, "%15s", arg) != 1) {
      Serial.println("usage: UPLOAD <LOGxxxx | nnnn | SCAN>");
      return;
    }
    if (strcasecmp(arg, "SCAN") == 0) {
      uploaderResume();
      uploaderRequestScan();
      Serial.println("[OK] rescan requested");
      return;
    }
    // "LOG0007" でも "7" でも受ける
    const char* p = arg;
    if (strncasecmp(p, "LOG", 3) == 0) p += 3;
    const long v = strtol(p, nullptr, 10);
    if (v < 1 || v > 9999) {
      Serial.println("[ERR] index must be 1..9999");
      return;
    }
    uploaderResume();
    uploaderRequestFile(static_cast<uint16_t>(v));
    Serial.printf("[OK] upload requested: LOG%04ld\n", v);
    return;
  }

  if (strcasecmp(line, "UPSTAT") == 0) { uploaderPrintStatus(); return; }

  if (strcasecmp(line, "WIFI") == 0) {
    Serial.printf("ssid=%s  status=%d  ", 
                  g_cfg.wifi_ssid[0] ? g_cfg.wifi_ssid : "<unset>",
                  static_cast<int>(WiFi.status()));
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("ip=%s rssi=%d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    } else {
      Serial.println("(not connected)");
    }
    return;
  }

  if (strcasecmp(line, "RELOAD") == 0) {
    if (g_state == ST_LOGGING || g_state == ST_DRAINING) {
      Serial.println("[ERR] stop logging first");
      return;
    }
    uploaderRequestStop();
    uploaderWaitIdle(3000);
    uploaderWifiOff();
    cfgLoad(g_cfg);
    uploaderResume();
    uploaderRequestScan();
    Serial.println("[OK] config reloaded");
    return;
  }
#endif

  if (strcasecmp(line, "HELP") == 0) {
    Serial.println("commands:");
    Serial.println("  SETTIME YYYY-MM-DD HH:MM:SS   set RTC (local time)");
    Serial.println("  GETTIME                       show RTC time");
    Serial.println("  TRIM [-127..127]              show/set oscillator trim");
    Serial.println("  STATUS                        show logger state and rtc flags");
#if UPLOAD_ENABLE
    Serial.println("  UPLOAD <LOGxxxx|nnnn|SCAN>    upload a log / rescan for new logs");
    Serial.println("  UPSTAT                        show upload status");
    Serial.println("  WIFI                          show wifi status");
    Serial.println("  RELOAD                        reload /CONFIG.TXT from SD");
#endif
    return;
  }

  Serial.printf("[ERR] unknown command: %s  (try HELP)\n", line);
}

static void pollSerial() {
  static char buf[64];
  static size_t len = 0;
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      buf[len] = '\0';
      handleCommand(buf);
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;  // 長すぎる行は捨てる
    }
  }
}

// ============================================================
// アップローダ用のフック
// ============================================================
#if UPLOAD_ENABLE
/*
 * アップロードしてよい状態かを返す。
 *
 * ST_LOGGING / ST_DRAINING では false を返すことが最重要。
 * 記録中にSDとCPUを奪うと「取りこぼしゼロ」という前提が崩れる。
 * ST_ERROR を許可しているのは、SD書き込み失敗以外の異常（CH1 bus off など）なら
 * そこまでのログを回収できるため。
 */
static bool upCanRun() {
  if (g_state == ST_LOGGING || g_state == ST_DRAINING) return false;
  if (g_fault == FAULT_SD_INIT || g_fault == FAULT_SD_WRITE) return false;
  return true;
}
#endif

// ============================================================
// setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== CAN Logger Jig v2.5 (2ch + RTC) ===");
  Serial.printf("free heap: %lu byte\n", (unsigned long)ESP.getFreeHeap());

  pinMode(LED_GPIO, OUTPUT);
  digitalWrite(LED_GPIO, LOW);
  pinMode(BTN_START_GPIO, INPUT_PULLUP);
  pinMode(BTN_STOP_GPIO, INPUT_PULLUP);
#if CH2_ENABLE
  pinMode(MCP_INT_GPIO, INPUT);  // GPIO34は内部プルアップ無し。外部10kΩを実装する
  pinMode(MCP_RESET_GPIO, OUTPUT);
  digitalWrite(MCP_RESET_GPIO, HIGH);
#endif

  SPI.begin(SD_SCK_GPIO, SD_MISO_GPIO, SD_MOSI_GPIO, SD_CS_GPIO);
#if CH2_ENABLE
  g_hspi.begin(MCP_SCK_GPIO, MCP_MISO_GPIO, MCP_MOSI_GPIO, MCP_CS_GPIO);
#endif

  g_fileMtx = xSemaphoreCreateMutex();

  // --- 大物バッファのヒープ確保（タスク生成より前に行う） ---
  bool memOk = true;
  g_ring1 = new (std::nothrow) SpscRing<RING_SIZE_CH1>();
  if (!g_ring1) memOk = false;
#if CH2_ENABLE
  g_ring2 = new (std::nothrow) SpscRing<RING_SIZE_CH2>();
  if (!g_ring2) memOk = false;
#endif
  g_wbuf = static_cast<char*>(malloc(WRITE_CHUNK_BYTES + ASC_LINE_MAX));
  if (!g_wbuf) memOk = false;

  if (!memOk) {
    // ここで落ちる場合は RING_SIZE_CH1 / RING_SIZE_CH2 を減らすしかない
    Serial.println("[ERR] buffer allocation failed. reduce RING_SIZE_CH1/CH2.");
    Serial.printf("      need: ring1=%u ring2=%u wbuf=%u byte / free=%lu byte\n",
                  (unsigned)(RING_SIZE_CH1 * sizeof(CanRec)),
                  (unsigned)(CH2_ENABLE ? RING_SIZE_CH2 * sizeof(CanRec) : 0),
                  (unsigned)(WRITE_CHUNK_BYTES + ASC_LINE_MAX),
                  (unsigned long)ESP.getFreeHeap());
    g_fault = FAULT_NO_MEMORY;
    g_state = ST_ERROR;
    // LEDだけは動かしたいので loop() には入る。タスクは起動しない
    return;
  }
  Serial.printf("buffers: ring1=%u ring2=%u wbuf=%u byte  (heap)\n",
                (unsigned)(RING_SIZE_CH1 * sizeof(CanRec)),
                (unsigned)(CH2_ENABLE ? RING_SIZE_CH2 * sizeof(CanRec) : 0),
                (unsigned)(WRITE_CHUNK_BYTES + ASC_LINE_MAX));

  // 電源投入直後は周辺デバイスがまだ立ち上がっていない。
  // ESP32は電源投入から約300msで初期化に入るが、SDカードは規格上
  // 最大1秒は応答しなくてよいため、そのまま初期化すると失敗して
  // ST_ERROR（高速点滅）になる。ENリセットでは直るのに電源投入では
  // 失敗する、という症状はこれが原因。
  delay(300);

  // --- RTC（失敗しても記録自体は可能なのでERROR状態にはしない） ---
#if RTC_ENABLE
  Wire.begin(RTC_SDA_GPIO, RTC_SCL_GPIO, RTC_I2C_HZ);
  g_rtcOk = g_rtc.begin(&Wire);
  if (g_rtcOk) {
    char ts[40];
    const uint32_t u = g_rtc.readUnixTimeUtc();
    formatLocalTime(u, ts, sizeof(ts));
    if (!g_rtc.oscillatorRunning()) {
      // 一度も SETTIME していなければ ST ビットが0なので、これが正常な初期状態。
      // SETTIME しても発振しない場合は水晶か負荷容量を疑う。
      Serial.println("[INFO] MCP7940N: oscillator stopped. run SETTIME to start it.");
    } else if (u == 0) {
      Serial.println("[WARN] MCP7940N: time not set. use SETTIME command.");
    } else {
      Serial.printf("MCP7940N: %s (local)%s\n", ts,
                    g_rtc.powerFailed() ? "  [power fail logged]" : "");
    }
  } else {
    Serial.println("[WARN] MCP7940N not responding. timestamps will be unavailable.");
  }
#endif

  // --- SD（カードの立ち上がり待ちのためリトライする） ---
  bool sdOk = false;
  for (uint8_t i = 0; i < 5; i++) {
    if (SD.begin(SD_CS_GPIO, SPI, SD_SPI_HZ)) { sdOk = true; break; }
    SD.end();
    delay(200);
  }
  if (!sdOk) {
    Serial.println("[ERR] SD card not found");
    g_fault = FAULT_SD_INIT;
    g_state = ST_ERROR;
  } else {
    Serial.printf("SD size: %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));
    g_state = ST_IDLE;
  }

  // --- MCP2515疎通確認（記録開始時に再初期化するのでここでは確認のみ） ---
#if CH2_ENABLE
  bool mcpOk = false;
  for (uint8_t i = 0; i < 3; i++) {
    if (g_mcp.begin(&g_hspi, MCP_CS_GPIO, CH2_BITRATE, MCP_SPI_HZ, MCP_RESET_GPIO)) {
      mcpOk = true;
      break;
    }
    delay(50);
  }
  if (mcpOk) {
    Serial.println("MCP2515 (CH2): detected");
  } else {
    Serial.println("[ERR] MCP2515 (CH2) not responding");
    g_fault = FAULT_MCP_INIT;
    g_state = ST_ERROR;
  }
#endif

  // --- アップロード（SDが無いと設定も進捗も読めないのでSD初期化の後に置く） ---
#if UPLOAD_ENABLE
  if (sdOk) {
    cfgLoad(g_cfg);
    UploaderPort port;
    port.canRun  = upCanRun;
    port.fileMtx = g_fileMtx;
    uploaderInit(port, &g_cfg);
    Serial.printf("boot count: %u\n", uploaderBootCount());
    if (g_cfg.wifi_ssid[0] == '\0') {
      Serial.println("[INFO] upload disabled: set WIFI_SSID in " CFG_PATH);
    } else if (g_cfg.up_host[0] == '\0') {
      Serial.println("[INFO] upload disabled: set UP_HOST in " CFG_PATH);
    }
  } else {
    // SDが無い場合でも既定値で初期化しておく（コマンドは受け付けられる）
    cfgSetDefaults(g_cfg);
    Serial.println("[WARN] no SD card. upload feature is unavailable.");
  }
#endif

  xTaskCreatePinnedToCore(rxTaskTwai, "rxTwai", 4096, nullptr, 20, nullptr, 1);
#if CH2_ENABLE
  xTaskCreatePinnedToCore(rxTaskMcp, "rxMcp", 4096, nullptr, 19, &g_mcpTaskHandle, 1);
#endif
  xTaskCreatePinnedToCore(writerTask, "writer", 8192, nullptr, 5, nullptr, 0);
#if UPLOAD_ENABLE
  if (sdOk) {
    xTaskCreatePinnedToCore(uploaderTask, "upload", UP_TASK_STACK, nullptr,
                            UP_TASK_PRIO, nullptr, UP_TASK_CORE);
  }
#endif

  Serial.printf("free heap after init: %lu byte\n", (unsigned long)ESP.getFreeHeap());
  Serial.println("type HELP for serial commands.");
}

void loop() {
  updateLed();
  pollSerial();

  if (g_state == ST_LOGGING) {
    pollControllerStatus();
    // 記録中に異常を検出したら、そこまでのログと統計を残して縮退終了する
    if (g_fault != FAULT_NONE) finalizeLogging(true);
  }

  if (buttonPressed(g_btnStart)) {
    if (g_state == ST_IDLE || g_state == ST_STOPPED || g_state == ST_ERROR) {
      if (!startLogging()) {
        hardCleanup();
        g_state = ST_ERROR;
        Serial.printf("[ERR] start failed: %s\n", faultText(g_fault));
#if UPLOAD_ENABLE
        // 記録が始まらなかったので、アップロードを再開させる。
        // ここで解除しないと中断要求が残ったままになる
        uploaderResume();
#endif
      }
    }
  }
  if (buttonPressed(g_btnStop)) {
    if (g_state == ST_LOGGING) finalizeLogging(false);
  }

  delay(5);
}
