/*
 * uploader.h — ログファイルの自動アップロード（P6: Wi-Fi版）
 *
 * 位置づけ
 *   改良仕様書 v3.3 §17.2 の P6。
 *   「通信層以外」を本番と同じ構造で作り、Rev.5基板のまま検証するための実装。
 *   P7 で Wi-Fi 部分だけを SIM7600 の AT コマンドへ差し替える。
 *   そのため HTTP の組み立て・分割・再送・進捗管理は通信方式に依存しない形にしてある。
 *
 * 絶対に守る原則
 *   1. アップロードが失敗しても記録機能は一切止めない
 *   2. 記録中（ST_LOGGING / ST_DRAINING）は SD にも CPU にも触らない
 *   3. 送信できたファイルも SD からは消さない（SDが最後の砦）
 *
 * 動作タイミング
 *   ST_IDLE / ST_STOPPED / ST_ERROR のときだけ動く。
 *   STARTボタンで記録を始める際は main.cpp が uploaderRequestStop() で
 *   中断させ、SD と Wi-Fi を手放したことを確認してから記録を開始する。
 *
 * 送信の単位
 *   part 0        … 統計ファイル LOGxxxx.TXT（数KB）
 *   part 1..N     … ログ本体を UP_CHUNK_BYTES ごとに分割したもの
 *   分割する理由は、切断・中断時の再送単位を小さくするため。
 *   SORACOM Harvest Files はセルラー経由で 1ファイル 100MiB 上限、
 *   かつ同一パスへの再送は上書きになるため、パート単位の再送は冪等になる。
 *
 * 送信先のパス
 *   <UP_PATH>/<DEVICE_ID>/<stamp>_LOG0007.txt
 *   <UP_PATH>/<DEVICE_ID>/<stamp>_LOG0007_001.bin
 *     stamp = BINヘッダの unix_time から作った "YYYYMMDD_HHMMSS"
 *             RTCが無効なら "nortc"
 *   LOG番号を必ず含めるので、RTCが壊れていてもSDカード内で一意になる。
 *
 * 進捗の記録 (/UPLOADED.TXT)
 *   追記のみ。1行1イベント。
 *     0007,P,000        … part 0（統計TXT）完了
 *     0007,P,001        … part 1 完了
 *     0007,D,003,262144 … 全パート完了（総パート数, 総バイト数）
 *   再開時は該当LOG番号の最大 P を探し、その次のパートから送る。
 */
#pragma once

#include <Arduino.h>
#include <stdarg.h>
#include <SD.h>
#include <WiFi.h>
#include <Preferences.h>

#include "config.h"
#include "cfgfile.h"

// ============================================================
// 定数
// ============================================================
#define UP_STATE_PATH   "/UPLOADED.TXT"
#define UP_IO_BUF       1024   // SD→ソケットの中継バッファ
#define UP_MAX_LOG_IDX  9999
#define UP_SCAN_LOOKAHEAD 5    // 欠番をまたいで探す数

enum UpPhase : uint8_t {
  UPP_DISABLED = 0,  // 設定で無効 / Wi-Fi未設定
  UPP_BLOCKED,       // 記録中などで動けない
  UPP_IDLE,          // 送るものが無い
  UPP_WIFI,          // Wi-Fi接続中
  UPP_SENDING,       // 送信中
  UPP_RETRY,         // 失敗して待機中
};

// main.cpp から渡すフック（インクルード順の依存を作らないため）
struct UploaderPort {
  bool (*canRun)();          // ST_IDLE / ST_STOPPED / ST_ERROR なら true
  SemaphoreHandle_t fileMtx; // SDアクセスの排他（main.cpp と共用）
};

// ============================================================
// 内部状態
// ============================================================
static UploaderPort g_upPort  = {nullptr, nullptr};
static AppCfg*      g_upCfg   = nullptr;

static volatile UpPhase g_upPhase = UPP_DISABLED;
static volatile bool    g_upStop  = false;  // 中断要求
static volatile bool    g_upAborted = false; // 直近の送信が「中断」で終わったか（失敗とは区別する）
static volatile bool    g_upBusy  = false;  // SD/Wi-Fiを掴んでいる
static volatile bool    g_upWifiWanted = false;

static uint16_t g_upBootCount = 0;

// 完了済みLOG番号のビットマップ（1..9999 → 1250byte）
static uint8_t  g_upDone[(UP_MAX_LOG_IDX / 8) + 1];
static volatile uint16_t g_upScanCursor = 1;

/*
 * 送信要求。
 *
 * 「特定ファイルの指定」と「全走査のやり直し」は別々に保持する。
 * 同じ変数に入れると、記録終了時の finalizeLogging() が出す走査要求が
 * 中断されて積み直された指定要求を上書きしてしまい、指示が黙って消える。
 */
static volatile uint16_t g_upReq     = 0;      // 0 = なし / 1..9999 = 指定ファイル
static volatile bool     g_upScanReq = false;  // 走査のやり直し要求

// 統計
static uint32_t g_upOkFiles   = 0;
static uint32_t g_upFailCount = 0;
static uint64_t g_upBytesSent = 0;
static uint16_t g_upCurIdx    = 0;
static uint16_t g_upCurPart   = 0;
static uint16_t g_upTotParts  = 0;
static char     g_upLastMsg[72] = "not started";

static uint8_t  g_upBuf[UP_IO_BUF];

// ============================================================
// 小物
// ============================================================
static inline bool upDoneGet(uint16_t idx) {
  if (idx == 0 || idx > UP_MAX_LOG_IDX) return true;
  return (g_upDone[idx >> 3] >> (idx & 7)) & 1;
}
static inline void upDoneSet(uint16_t idx) {
  if (idx == 0 || idx > UP_MAX_LOG_IDX) return;
  g_upDone[idx >> 3] |= static_cast<uint8_t>(1u << (idx & 7));
}

static inline void upMsg(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(g_upLastMsg, sizeof(g_upLastMsg), fmt, ap);
  va_end(ap);
}

static inline bool upLock(uint32_t ms) {
  if (!g_upPort.fileMtx) return true;
  return xSemaphoreTake(g_upPort.fileMtx, pdMS_TO_TICKS(ms)) == pdTRUE;
}
static inline void upUnlock() {
  if (g_upPort.fileMtx) xSemaphoreGive(g_upPort.fileMtx);
}

// 中断すべきか
static inline bool upShouldAbort() {
  if (g_upStop) return true;
  if (g_upPort.canRun && !g_upPort.canRun()) return true;
  return false;
}

/*
 * UNIX時刻(UTC) → 現地時刻の "YYYYMMDD_HHMMSS"
 * main.cpp の formatLocalTime と同じ暦計算だが、
 * uploader.h を単独で差し替えられるようにあえて内包している。
 */
static void upEpochToStamp(uint32_t utc, char* out, size_t len) {
  if (utc == 0) {
    snprintf(out, len, "nortc");
    return;
  }
  int64_t t = static_cast<int64_t>(utc) + RTC_UTC_OFFSET_SEC;
  const int64_t days = t / 86400;
  int64_t secs = t % 86400;
  int64_t z = days + 719468;
  const int64_t  era = (z >= 0 ? z : z - 146096) / 146097;
  const unsigned doe = static_cast<unsigned>(z - era * 146097);
  const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const int64_t  y   = static_cast<int64_t>(yoe) + era * 400;
  const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const unsigned mp  = (5 * doy + 2) / 153;
  const unsigned d   = doy - (153 * mp + 2) / 5 + 1;
  const unsigned m   = mp + (mp < 10 ? 3 : -9);
  snprintf(out, len, "%04lld%02u%02u_%02lld%02lld%02lld",
           static_cast<long long>(y + (m <= 2)), m, d,
           static_cast<long long>(secs / 3600),
           static_cast<long long>((secs % 3600) / 60),
           static_cast<long long>(secs % 60));
}

// ============================================================
// /UPLOADED.TXT
// ============================================================
/*
 * 起動時に全体を読み、完了ビットマップを作る。
 * 未完了ファイルの進捗は upFindProgress() で都度読み直す
 *  （同時に扱うファイルは1つだけなので、常駐させる意味がない）。
 */
static void upLoadDoneBits() {
  memset(g_upDone, 0, sizeof(g_upDone));

  File f = SD.open(UP_STATE_PATH, FILE_READ);
  if (!f) return;

  char   line[64];
  size_t n = 0;
  uint32_t lines = 0;
  for (;;) {
    const int ci = f.read();
    const bool eof = (ci < 0);
    const char ch = eof ? '\n' : static_cast<char>(ci);
    if (ch != '\n') {
      if (n < sizeof(line) - 1) line[n++] = ch;
      if (!eof) continue;
    }
    line[n] = '\0';
    n = 0;
    lines++;
    // "0007,D,003,262144"
    if (line[0] >= '0' && line[0] <= '9') {
      const uint16_t idx = static_cast<uint16_t>(strtol(line, nullptr, 10));
      const char* c1 = strchr(line, ',');
      if (c1 && c1[1] == 'D') upDoneSet(idx);
    }
    if (eof) break;
  }
  f.close();
  Serial.printf("[OK] %s loaded (%lu lines)\n", UP_STATE_PATH, (unsigned long)lines);
}

/*
 * 指定LOG番号について、完了している最大パート番号を返す。未着手なら -1。
 * 現在のチャンクサイズと一致する記録だけを採用する（§upMarkPart のコメント参照）。
 */
static int32_t upFindProgress(uint16_t idx) {
  const uint32_t curKb = g_upCfg->up_chunk_bytes / 1024;
  if (!upLock(2000)) return -1;
  File f = SD.open(UP_STATE_PATH, FILE_READ);
  if (!f) { upUnlock(); return -1; }

  char    line[64];
  size_t  n = 0;
  int32_t maxPart = -1;
  for (;;) {
    const int ci = f.read();
    const bool eof = (ci < 0);
    const char ch = eof ? '\n' : static_cast<char>(ci);
    if (ch != '\n') {
      if (n < sizeof(line) - 1) line[n++] = ch;
      if (!eof) continue;
    }
    line[n] = '\0';
    n = 0;
    if (line[0] >= '0' && line[0] <= '9') {
      const uint16_t i = static_cast<uint16_t>(strtol(line, nullptr, 10));
      const char* c1 = strchr(line, ',');
      if (i == idx && c1 && c1[1] == 'P') {
        const char* c2 = strchr(c1 + 1, ',');
        if (c2) {
          const int32_t p = strtol(c2 + 1, nullptr, 10);
          // 4番目のフィールド = 記録時のチャンクサイズ[KB]
          const char* c3 = strchr(c2 + 1, ',');
          const uint32_t kb = c3 ? strtoul(c3 + 1, nullptr, 10) : 0;
          if (kb == curKb && p > maxPart) maxPart = p;
        }
      }
    }
    if (eof) break;
  }
  f.close();
  upUnlock();
  return maxPart;
}

static void upAppendState(const char* line) {
  if (!upLock(2000)) {
    Serial.println("[WARN] sd busy: cannot append " UP_STATE_PATH);
    return;
  }
  File f = SD.open(UP_STATE_PATH, FILE_APPEND);
  if (!f) {
    upUnlock();
    Serial.println("[WARN] cannot append " UP_STATE_PATH);
    return;
  }
  f.print(line);
  f.flush();
  f.close();
  upUnlock();
}

/*
 * パート完了を記録する。
 *
 * ★チャンクサイズ(KB)も一緒に書く。
 *   パート番号はチャンクサイズを前提にした値なので、設定を変えると
 *   同じ番号でも指すオフセットが変わってしまう。
 *   読み出し側(upFindProgress)で照合し、違えば進捗を無効として先頭から送り直す。
 */
static void upMarkPart(uint16_t idx, uint16_t part) {
  char l[40];
  snprintf(l, sizeof(l), "%04u,P,%03u,%lu\n", idx, part,
           (unsigned long)(g_upCfg->up_chunk_bytes / 1024));
  upAppendState(l);
}

static void upMarkDone(uint16_t idx, uint16_t totalParts, uint64_t bytes) {
  char l[48];
  snprintf(l, sizeof(l), "%04u,D,%03u,%llu\n", idx, totalParts,
           static_cast<unsigned long long>(bytes));
  upAppendState(l);
  upDoneSet(idx);
}

// ============================================================
// Wi-Fi
// ============================================================
static bool upWifiUp(uint32_t timeoutMs) {
  if (WiFi.status() == WL_CONNECTED) return true;
  if (g_upCfg->wifi_ssid[0] == '\0') {
    upMsg("WIFI_SSID not configured");
    return false;
  }

  g_upPhase = UPP_WIFI;
  g_upWifiWanted = true;
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);  // 送信していない間の消費を抑える
  WiFi.begin(g_upCfg->wifi_ssid, g_upCfg->wifi_pass);

  const uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 >= timeoutMs) {
      upMsg("wifi connect timeout");
      return false;
    }
    if (upShouldAbort()) {
      upMsg("wifi connect aborted");
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  Serial.printf("[OK] wifi connected: %s  rssi=%d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
  return true;
}

static void upWifiDown() {
  g_upWifiWanted = false;
  if (WiFi.getMode() != WIFI_OFF) {
    WiFi.disconnect(true, false);
    WiFi.mode(WIFI_OFF);
  }
}

// ============================================================
// HTTP PUT（1パート分）
// ============================================================
// レスポンスのステータス行だけを読む。取れなければ -1
static int upReadHttpStatus(WiFiClient& cli, uint32_t timeoutMs) {
  const uint32_t t0 = millis();
  char   line[80];
  size_t n = 0;
  bool   got = false;

  while (millis() - t0 < timeoutMs && !got) {
    while (cli.available()) {
      const char c = static_cast<char>(cli.read());
      if (c == '\n') { got = true; break; }
      if (c != '\r' && n < sizeof(line) - 1) line[n++] = c;
    }
    if (got) break;
    if (!cli.connected() && !cli.available()) break;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  if (!got) return -1;

  line[n] = '\0';
  const char* sp = strchr(line, ' ');
  if (!sp) return -1;
  return atoi(sp + 1);
}

/*
 * ローカルファイルの [off, off+len) を remotePath へ PUT する。
 * f は呼び出し側で開いておく（SDアクセスはこの関数内で都度ロックする）。
 * 戻り値: true = 2xx を受け取った
 */
static bool upPutRange(const char* remotePath, File& f, uint32_t off, uint32_t len) {
  WiFiClient cli;
  cli.setTimeout(UP_HTTP_TIMEOUT_MS / 1000);

  if (!cli.connect(g_upCfg->up_host, g_upCfg->up_port)) {
    upMsg("connect failed: %s:%u", g_upCfg->up_host, g_upCfg->up_port);
    return false;
  }

  char hdr[352];
  const int hn = snprintf(hdr, sizeof(hdr),
      "PUT %s HTTP/1.1\r\n"
      "Host: %s:%u\r\n"
      "Content-Type: application/octet-stream\r\n"
      "Content-Length: %lu\r\n"
      "Connection: close\r\n"
      "\r\n",
      remotePath, g_upCfg->up_host, static_cast<unsigned>(g_upCfg->up_port),
      static_cast<unsigned long>(len));
  if (hn <= 0 || cli.write(reinterpret_cast<const uint8_t*>(hdr),
                           static_cast<size_t>(hn)) != static_cast<size_t>(hn)) {
    upMsg("header write failed");
    cli.stop();
    return false;
  }

  // --- 本体 ---
  uint32_t remain = len;
  {
    if (!upLock(2000)) { upMsg("sd busy"); cli.stop(); return false; }
    const bool sk = f.seek(off);
    upUnlock();
    if (!sk) { upMsg("seek failed"); cli.stop(); return false; }
  }

  while (remain > 0) {
    if (upShouldAbort()) {
      upMsg("aborted during send");
      cli.stop();
      return false;
    }
    const size_t want = (remain > sizeof(g_upBuf)) ? sizeof(g_upBuf) : remain;

    size_t got = 0;
    if (!upLock(2000)) { upMsg("sd busy"); cli.stop(); return false; }
    got = f.read(g_upBuf, want);
    upUnlock();

    if (got == 0) { upMsg("sd read returned 0"); cli.stop(); return false; }
    if (cli.write(g_upBuf, got) != got) { upMsg("socket write failed"); cli.stop(); return false; }

    remain -= got;
    g_upBytesSent += got;
    // 送信タスクは優先度3。ここで必ず譲る
    vTaskDelay(1);
  }

  const int code = upReadHttpStatus(cli, UP_HTTP_TIMEOUT_MS);
  cli.stop();

  if (code < 200 || code >= 300) {
    upMsg("http status %d", code);
    return false;
  }
  return true;
}

// 再試行つき
static bool upPutRangeRetry(const char* remotePath, File& f, uint32_t off, uint32_t len) {
  const int maxAttempt = static_cast<int>(g_upCfg->up_retry_max);
  for (int attempt = 0; attempt <= maxAttempt; attempt++) {
    if (upShouldAbort()) return false;
    if (attempt > 0) {
      g_upPhase = UPP_RETRY;
      Serial.printf("[WARN] upload retry %d/%d (%s)\n",
                    attempt, maxAttempt, g_upLastMsg);
      // 1,2,4,8... 秒（上限8秒）でバックオフ
      const uint32_t waitMs = 1000UL << ((attempt > 3) ? 3 : (attempt - 1));
      const uint32_t t0 = millis();
      while (millis() - t0 < waitMs) {
        if (upShouldAbort()) return false;
        vTaskDelay(pdMS_TO_TICKS(50));
      }
      if (!upWifiUp(UP_WIFI_TIMEOUT_MS)) continue;
      g_upPhase = UPP_SENDING;
    }
    if (upPutRange(remotePath, f, off, len)) return true;

    // 中断（記録開始・モード変更）は通信の失敗ではないので統計に数えない。
    // ここを分けておかないと fail= が「回線が不調か」の指標として使えなくなる。
    if (upShouldAbort()) {
      g_upAborted = true;
      return false;
    }
    g_upFailCount++;
  }
  return false;
}

// ============================================================
// 1ファイルのアップロード
// ============================================================
static void upBuildLocalNames(uint16_t idx, char* binPath, size_t bn,
                              char* txtPath, size_t tn) {
#if LOG_FORMAT == LOG_FORMAT_ASC
  snprintf(binPath, bn, "/LOG%04u.ASC", idx);
#else
  snprintf(binPath, bn, "/LOG%04u.BIN", idx);
#endif
  snprintf(txtPath, tn, "/LOG%04u.TXT", idx);
}

// BINヘッダの unix_time を読む。ASC形式や読めない場合は0
static uint32_t upReadStartUnix(const char* binPath) {
#if LOG_FORMAT == LOG_FORMAT_ASC
  (void)binPath;
  return 0;
#else
  uint32_t u = 0;
  if (!upLock(2000)) return 0;
  File f = SD.open(binPath, FILE_READ);
  if (f) {
    uint8_t h[32];
    if (f.read(h, 32) == 32 && memcmp(h, "CANLOG02", 8) == 0) {
      memcpy(&u, &h[20], 4);  // BinHeader.unix_time (+20)
    }
    f.close();
  }
  upUnlock();
  return u;
#endif
}

/*
 * 1ファイルを送る。
 *
 * forced = true は「UPLOADコマンドで明示的に指定された」ことを示す。
 *   このとき UP_STATS_ONLY は無視してログ本体まで送る。
 *   方針B（普段は統計だけ／必要なものだけ取りに行く）を成立させるために必要。
 *
 * 戻り値: true = このファイルは完了した（または送るものが無かった）
 *         false = 失敗・中断（次回に再開する）
 */
static bool upSendOneFile(uint16_t idx, bool forced) {
  g_upAborted = false;

  char binPath[20], txtPath[20];
  upBuildLocalNames(idx, binPath, sizeof(binPath), txtPath, sizeof(txtPath));

  // --- サイズとパート数を決める ---
  uint32_t binSize = 0;
  bool     hasBin  = false;
  if (!upLock(2000)) return false;
  {
    File f = SD.open(binPath, FILE_READ);
    if (f) { hasBin = true; binSize = f.size(); f.close(); }
  }
  upUnlock();

  // UPLOADコマンドで指定された場合は UP_STATS_ONLY を無視する
  const bool statsOnly = (g_upCfg->up_stats_only != 0) && !forced;

  const uint32_t chunk = g_upCfg->up_chunk_bytes;
  uint16_t binParts = 0;
  if (hasBin && binSize > 0) {
    binParts = static_cast<uint16_t>((binSize + chunk - 1) / chunk);
  }
  const uint16_t totalParts = statsOnly ? 0 : binParts;

  g_upCurIdx   = idx;
  g_upTotParts = totalParts;

  const uint32_t startUnix = upReadStartUnix(binPath);
  char stamp[24];
  upEpochToStamp(startUnix, stamp, sizeof(stamp));

  const int32_t progress = upFindProgress(idx);  // 完了済みの最大パート
  uint16_t nextPart = (progress < 0) ? 0 : static_cast<uint16_t>(progress + 1);

  // チャンクサイズが変わっていると upFindProgress が -1 を返すため、
  // 記録が残っていても先頭から送り直すことになる（欠けたファイルを作らないため）
  if (progress < 0 && upDoneGet(idx)) {
    Serial.printf("[UP] LOG%04u : progress reset (chunk size changed). resend from part 0\n", idx);
  }

  Serial.printf("[UP] LOG%04u  stamp=%s  bin=%lu byte  parts=%u  resume from %u%s%s\n",
                idx, stamp, (unsigned long)binSize, totalParts, nextPart,
                statsOnly ? "  (stats only)" : "",
                forced ? "  [manual]" : "");

  char remote[160];

  // --- part 0 : 統計ファイル ---
  if (nextPart == 0) {
    g_upCurPart = 0;
    bool txtOk = false;
    uint32_t txtSize = 0;

    if (!upLock(2000)) return false;
    File tf = SD.open(txtPath, FILE_READ);
    const bool hasTxt = (bool)tf;
    if (hasTxt) txtSize = tf.size();
    upUnlock();

    if (!hasTxt) {
      // 統計ファイルが無いのは異常終了の痕跡。ログ本体だけ送る
      Serial.printf("[WARN] %s not found. skip stats part.\n", txtPath);
      txtOk = true;
    } else {
      snprintf(remote, sizeof(remote), "%s/%s/%s_LOG%04u.txt",
               g_upCfg->up_path, g_upCfg->device_id, stamp, idx);
      g_upPhase = UPP_SENDING;
      txtOk = upPutRangeRetry(remote, tf, 0, txtSize);
      if (!upLock(2000)) { tf.close(); return false; }
      tf.close();
      upUnlock();
    }

    if (!txtOk) return false;
    upMarkPart(idx, 0);
    nextPart = 1;
    Serial.printf("[UP] LOG%04u part 0 (stats) done\n", idx);
  }

  // --- 統計のみモードならここで完了 ---
  if (statsOnly) {
    upMarkDone(idx, 0, 0);
    g_upOkFiles++;
    upMsg("LOG%04u stats uploaded", idx);
    Serial.printf("[UP] LOG%04u complete (stats only)\n", idx);
    return true;
  }

  // --- part 1..N : ログ本体 ---
  if (!hasBin || binSize == 0) {
    Serial.printf("[WARN] %s missing or empty. mark done.\n", binPath);
    upMarkDone(idx, 0, 0);
    g_upOkFiles++;
    return true;
  }

  if (!upLock(2000)) return false;
  File bf = SD.open(binPath, FILE_READ);
  upUnlock();
  if (!bf) { upMsg("cannot open %s", binPath); return false; }

  for (uint16_t part = nextPart; part <= totalParts; part++) {
    if (upShouldAbort()) {
      g_upAborted = true;
      if (upLock(2000)) { bf.close(); upUnlock(); }
      upMsg("aborted at LOG%04u part %u", idx, part);
      Serial.printf("[UP] aborted at LOG%04u part %u/%u\n", idx, part, totalParts);
      return false;
    }

    const uint32_t off = static_cast<uint32_t>(part - 1) * chunk;
    uint32_t len = chunk;
    if (off + len > binSize) len = binSize - off;

    snprintf(remote, sizeof(remote), "%s/%s/%s_LOG%04u_%03u.bin",
             g_upCfg->up_path, g_upCfg->device_id, stamp, idx, part);

    g_upCurPart = part;
    g_upPhase   = UPP_SENDING;

    const uint32_t t0 = millis();
    if (!upPutRangeRetry(remote, bf, off, len)) {
      if (upLock(2000)) { bf.close(); upUnlock(); }
      Serial.printf("[ERR] LOG%04u part %u/%u failed: %s\n",
                    idx, part, totalParts, g_upLastMsg);
      return false;
    }
    const uint32_t dt = millis() - t0;

    upMarkPart(idx, part);
    Serial.printf("[UP] LOG%04u part %u/%u done  %lu byte  %lu ms  (%lu kB/s)\n",
                  idx, part, totalParts, (unsigned long)len, (unsigned long)dt,
                  (unsigned long)(dt ? (len / dt) : 0));
  }

  if (upLock(2000)) { bf.close(); upUnlock(); }

  upMarkDone(idx, totalParts, binSize);
  g_upOkFiles++;
  upMsg("LOG%04u uploaded (%u parts)", idx, totalParts);
  Serial.printf("[UP] LOG%04u complete (%u parts, %lu byte)\n",
                idx, totalParts, (unsigned long)binSize);
  return true;
}

// ============================================================
// 次に送るファイルを探す
// ============================================================
static bool upFindNext(uint16_t& out) {
  uint16_t miss = 0;
  for (uint16_t i = g_upScanCursor; i <= UP_MAX_LOG_IDX; i++) {
    char binPath[20], txtPath[20];
    upBuildLocalNames(i, binPath, sizeof(binPath), txtPath, sizeof(txtPath));

    if (!upLock(2000)) return false;
    const bool exists = SD.exists(binPath) || SD.exists(txtPath);
    upUnlock();

    if (!exists) {
      // 採番は1から連番なので、続けて存在しなければ末尾とみなす。
      // 手動削除による欠番を数個またげるよう余裕を持たせる
      if (++miss >= UP_SCAN_LOOKAHEAD) return false;
      continue;
    }
    miss = 0;

    if (upDoneGet(i)) {
      // 完了済みが続く間はカーソルを進めて次回の走査を軽くする
      if (i == g_upScanCursor) g_upScanCursor = i + 1;
      continue;
    }
    out = i;
    return true;
  }
  return false;
}

// ============================================================
// タスク
// ============================================================
static void uploaderTask(void*) {
  for (;;) {
    // --- 動いてよい状態か ---
    if (!g_upPort.canRun || !g_upPort.canRun()) {
      if (g_upBusy) g_upBusy = false;
      if (g_upWifiWanted) upWifiDown();
      g_upStop  = false;   // 記録側へ移ったので中断要求は用済み
      g_upPhase = UPP_BLOCKED;
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    if (g_upStop) {
      g_upBusy  = false;
      g_upPhase = UPP_BLOCKED;
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    if (!g_upCfg || !g_upCfg->up_enable || g_upCfg->up_host[0] == '\0' ||
        g_upCfg->wifi_ssid[0] == '\0') {
      g_upBusy  = false;
      g_upPhase = UPP_DISABLED;
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // --- 送る対象を決める ---
    // 指定要求を最優先にする。走査要求はそれが片付いてから処理する。
    uint16_t idx = 0;
    const uint16_t req = g_upReq;
    const bool forced = (req != 0);  // UPLOADコマンドによる指定
    if (forced) {
      idx = req;
      g_upReq = 0;
    } else {
      const bool scanReq = g_upScanReq;
      g_upScanReq = false;
      if (!g_upCfg->up_auto && !scanReq) {
        g_upBusy  = false;
        g_upPhase = UPP_IDLE;
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
      g_upBusy = true;
      if (!upFindNext(idx)) {
        g_upBusy  = false;
        g_upPhase = UPP_IDLE;
        if (g_upWifiWanted) upWifiDown();  // 送るものが無ければWi-Fiを落とす
        vTaskDelay(pdMS_TO_TICKS(2000));
        continue;
      }
    }

    // --- 実行 ---
    g_upBusy = true;
    if (!upWifiUp(UP_WIFI_TIMEOUT_MS)) {
      g_upBusy  = false;
      g_upPhase = UPP_RETRY;
      Serial.printf("[WARN] wifi unavailable: %s\n", g_upLastMsg);
      vTaskDelay(pdMS_TO_TICKS(10000));
      continue;
    }

    const bool ok = upSendOneFile(idx, forced);
    g_upBusy = false;

    if (!ok) {
      /*
       * UPLOADコマンドによる指定が「中断」で終わった場合は要求を積み直す。
       *
       * 完了済み(D)のファイルを再送している最中に記録が始まると、
       * 自動走査は完了済みとして飛ばしてしまうため、ここで覚えておかないと
       * 要求そのものが消える。遠隔コマンドで特定のBINを取りに行く運用
       * （改良仕様書 方針B）では、これが無いと指示が黙って失われる。
       *
       * 通信失敗(g_upAborted=false)の場合は積み直さない。
       * 到達不能なホストを無限に叩き続けることになるため。
       */
      if (forced && g_upAborted) {
        g_upReq = idx;
        upMsg("LOG%04u re-queued after abort", idx);
        Serial.printf("[UP] LOG%04u re-queued (will resume when idle)\n", idx);
      }
      g_upPhase = UPP_RETRY;
      vTaskDelay(pdMS_TO_TICKS(5000));
    } else {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

// ============================================================
// main.cpp から呼ぶAPI
// ============================================================
static void uploaderInit(const UploaderPort& port, AppCfg* cfg) {
  g_upPort = port;
  g_upCfg  = cfg;

  // 起動回数カウンタ（NVS）。RTCが死んでいる場合の識別に使う
  Preferences prefs;
  if (prefs.begin("canlog", false)) {
    g_upBootCount = static_cast<uint16_t>(prefs.getUInt("boot", 0) + 1);
    prefs.putUInt("boot", g_upBootCount);
    prefs.end();
  }

  upLoadDoneBits();
  g_upPhase = UPP_IDLE;
  upMsg("ready");
}

static uint16_t uploaderBootCount() { return g_upBootCount; }

// 記録を始める前に呼ぶ。中断を要求する
static void uploaderRequestStop() { g_upStop = true; }

// 中断要求を解除する（記録開始に失敗したとき／記録終了後）
static void uploaderResume() { g_upStop = false; }

// SD・Wi-Fiを手放すまで待つ。戻り値 false = タイムアウト
static bool uploaderWaitIdle(uint32_t timeoutMs) {
  const uint32_t t0 = millis();
  while (g_upBusy) {
    if (millis() - t0 >= timeoutMs) return false;
    delay(10);
  }
  return true;
}

static void uploaderWifiOff() { upWifiDown(); }

// LED表示用
static bool uploaderActive() {
  return (g_upPhase == UPP_SENDING || g_upPhase == UPP_WIFI);
}

static void uploaderRequestFile(uint16_t idx) { g_upReq = idx; }

// 走査をやり直させる。指定要求が残っていても壊さない
static void uploaderRequestScan() {
  g_upScanCursor = 1;
  g_upScanReq    = true;
}

static const char* uploaderPhaseText() {
  switch (g_upPhase) {
    case UPP_DISABLED: return "disabled";
    case UPP_BLOCKED:  return "blocked";
    case UPP_IDLE:     return "idle";
    case UPP_WIFI:     return "wifi";
    case UPP_SENDING:  return "sending";
    case UPP_RETRY:    return "retry";
    default:           return "?";
  }
}

static void uploaderPrintStatus() {
  Serial.printf("upload  : phase=%s  boot#=%u  ok=%lu  fail=%lu  sent=%llu byte\n",
                uploaderPhaseText(), g_upBootCount,
                (unsigned long)g_upOkFiles, (unsigned long)g_upFailCount,
                (unsigned long long)g_upBytesSent);
  if (g_upPhase == UPP_SENDING) {
    Serial.printf("          LOG%04u part %u/%u\n", g_upCurIdx, g_upCurPart, g_upTotParts);
  }
  Serial.printf("          last: %s\n", g_upLastMsg);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("wifi    : %s  ip=%s  rssi=%d dBm\n",
                  g_upCfg ? g_upCfg->wifi_ssid : "",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.printf("wifi    : not connected  (ssid=%s)\n",
                  (g_upCfg && g_upCfg->wifi_ssid[0]) ? g_upCfg->wifi_ssid : "<unset>");
  }
  if (g_upCfg) {
    const char* policy = !g_upCfg->up_enable ? "OFF"
                       : (!g_upCfg->up_auto   ? "C (manual only)"
                       : (g_upCfg->up_stats_only ? "B (stats auto / bin on demand)"
                                                 : "A (everything auto)"));
    Serial.printf("policy  : %s\n", policy);
    Serial.printf("dest    : http://%s:%u%s/%s/  chunk=%lu KB\n",
                  g_upCfg->up_host, g_upCfg->up_port, g_upCfg->up_path,
                  g_upCfg->device_id,
                  (unsigned long)(g_upCfg->up_chunk_bytes / 1024));
  }
}
