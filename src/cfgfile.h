/*
 * cfgfile.h — SDカード上の /CONFIG.TXT を読む最小パーサ
 *
 * 目的
 *   Wi-Fi の SSID/パスワードやアップロード先をソースに直書きすると、
 *   変更のたびに再ビルド＋書き込み（CH340E は BOOT+EN の手動操作）が必要になる。
 *   現場・実験室でパラメータだけ差し替えられるようにする。
 *
 * 方針（既存コードと同じ）
 *   - 外部ライブラリを使わない
 *   - 動的メモリ確保を行わない（すべて固定長バッファ）
 *   - 不正な行は読み飛ばして既定値で動き続ける。起動は絶対に止めない
 *
 * 書式
 *   KEY = VALUE      … 前後の空白は無視
 *   # で始まる行と、行中の # 以降はコメント
 *   1行の最大長は CFG_LINE_MAX。超えた分は捨てて警告する
 *
 * 注) Wi-Fi パスワードは平文で SD に載る。持ち出し前提の治具では
 *     専用の SSID を切るなど運用側で担保すること。
 */
#pragma once

#include <Arduino.h>
#include <SD.h>

#define CFG_PATH     "/CONFIG.TXT"
#define CFG_LINE_MAX 200

struct AppCfg {
  // ---- Wi-Fi（P6: 有線LAN相当の実験用。P7でLTEへ置き換える）----
  char wifi_ssid[33];
  char wifi_pass[65];

  // ---- アップロード ----
  uint8_t  up_enable;      // 0 = アップロード機能を完全に停止
  uint8_t  up_auto;        // 1 = 待機中に自動でアップロード / 0 = UPLOADコマンドのみ
  uint8_t  up_stats_only;  // 1 = 統計TXTのみ送る（改良仕様書 §10.2 方針B相当）
  char     device_id[17];  // 送信先パスに入る治具の識別子
  char     up_host[65];
  uint16_t up_port;
  char     up_path[49];    // 先頭に / を付ける。末尾の / は不要
  uint32_t up_chunk_bytes; // 1パートのサイズ
  uint8_t  up_retry_max;   // 1パートあたりの再試行回数
};

// 既定値。CONFIG.TXT が無い/読めない場合はこの値で動く
static inline void cfgSetDefaults(AppCfg& c) {
  c.wifi_ssid[0]   = '\0';
  c.wifi_pass[0]   = '\0';
  c.up_enable      = 1;
  c.up_auto        = 1;
  c.up_stats_only  = 0;
  snprintf(c.device_id, sizeof(c.device_id), "JIG001");
  c.up_host[0]     = '\0';
  c.up_port        = 8080;
  snprintf(c.up_path, sizeof(c.up_path), "/canlog");
  c.up_chunk_bytes = 1024UL * 1024UL;
  c.up_retry_max   = 10;
}

namespace cfgfile {

static inline char* trim(char* s) {
  while (*s == ' ' || *s == '\t') s++;
  size_t n = strlen(s);
  while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t' ||
                   s[n - 1] == '\r' || s[n - 1] == '\n')) {
    s[--n] = '\0';
  }
  return s;
}

static inline void copyStr(char* dst, size_t cap, const char* src) {
  snprintf(dst, cap, "%s", src);
}

// "1" / "0" / "true" / "false" / "yes" / "no" を受ける
static inline uint8_t toBool(const char* v, uint8_t defval) {
  if (v[0] == '\0') return defval;
  if (!strcasecmp(v, "1") || !strcasecmp(v, "true") || !strcasecmp(v, "yes")) return 1;
  if (!strcasecmp(v, "0") || !strcasecmp(v, "false") || !strcasecmp(v, "no")) return 0;
  return defval;
}

static inline void applyPair(AppCfg& c, const char* key, const char* val, uint16_t lineNo) {
  if      (!strcasecmp(key, "WIFI_SSID"))     copyStr(c.wifi_ssid, sizeof(c.wifi_ssid), val);
  else if (!strcasecmp(key, "WIFI_PASS"))     copyStr(c.wifi_pass, sizeof(c.wifi_pass), val);
  else if (!strcasecmp(key, "UP_ENABLE"))     c.up_enable     = toBool(val, c.up_enable);
  else if (!strcasecmp(key, "UP_AUTO"))       c.up_auto       = toBool(val, c.up_auto);
  else if (!strcasecmp(key, "UP_STATS_ONLY")) c.up_stats_only = toBool(val, c.up_stats_only);
  else if (!strcasecmp(key, "DEVICE_ID"))     copyStr(c.device_id, sizeof(c.device_id), val);
  else if (!strcasecmp(key, "UP_HOST"))       copyStr(c.up_host, sizeof(c.up_host), val);
  else if (!strcasecmp(key, "UP_PATH"))       copyStr(c.up_path, sizeof(c.up_path), val);
  else if (!strcasecmp(key, "UP_PORT")) {
    const long v = strtol(val, nullptr, 10);
    if (v > 0 && v < 65536) c.up_port = static_cast<uint16_t>(v);
    else Serial.printf("[WARN] CONFIG.TXT line %u: UP_PORT out of range\n", lineNo);
  } else if (!strcasecmp(key, "UP_CHUNK_KB")) {
    const long v = strtol(val, nullptr, 10);
    // 上限は Harvest Files のセルラー経由上限 100MiB を意識して 65536KB(64MiB) に抑える
    if (v >= 4 && v <= 65536) c.up_chunk_bytes = static_cast<uint32_t>(v) * 1024UL;
    else Serial.printf("[WARN] CONFIG.TXT line %u: UP_CHUNK_KB out of range (4..65536)\n", lineNo);
  } else if (!strcasecmp(key, "UP_RETRY_MAX")) {
    const long v = strtol(val, nullptr, 10);
    if (v >= 0 && v <= 255) c.up_retry_max = static_cast<uint8_t>(v);
    else Serial.printf("[WARN] CONFIG.TXT line %u: UP_RETRY_MAX out of range\n", lineNo);
  } else {
    Serial.printf("[WARN] CONFIG.TXT line %u: unknown key '%s' (ignored)\n", lineNo, key);
  }
}

}  // namespace cfgfile

/*
 * /CONFIG.TXT を読み込む。
 * 戻り値: true = ファイルがあって読めた / false = 無い（既定値のまま動く）
 * どちらの場合も c は必ず有効な値で埋まる。
 */
static inline bool cfgLoad(AppCfg& c) {
  cfgSetDefaults(c);

  File f = SD.open(CFG_PATH, FILE_READ);
  if (!f) {
    Serial.println("[INFO] " CFG_PATH " not found. using defaults.");
    return false;
  }

  char     line[CFG_LINE_MAX];
  size_t   len     = 0;
  uint16_t lineNo  = 1;
  bool     tooLong = false;

  for (;;) {
    const int ci = f.read();
    const bool eof = (ci < 0);
    const char ch  = eof ? '\n' : static_cast<char>(ci);

    if (ch != '\n') {
      if (len < sizeof(line) - 1) {
        line[len++] = ch;
      } else {
        tooLong = true;  // あふれた分は捨てる
      }
      if (!eof) continue;
    }

    line[len] = '\0';
    if (tooLong) {
      Serial.printf("[WARN] CONFIG.TXT line %u: too long, truncated\n", lineNo);
      tooLong = false;
    }

    // 行中の # 以降はコメント
    char* hash = strchr(line, '#');
    if (hash) *hash = '\0';

    char* s = cfgfile::trim(line);
    if (*s != '\0') {
      char* eq = strchr(s, '=');
      if (!eq) {
        Serial.printf("[WARN] CONFIG.TXT line %u: no '=' (ignored)\n", lineNo);
      } else {
        *eq = '\0';
        char* key = cfgfile::trim(s);
        char* val = cfgfile::trim(eq + 1);
        if (*key != '\0') cfgfile::applyPair(c, key, val, lineNo);
      }
    }

    len = 0;
    lineNo++;
    if (eof) break;
  }

  f.close();

  // 整合性チェック（起動は止めない）
  if (c.up_path[0] != '/') {
    Serial.println("[WARN] CONFIG.TXT: UP_PATH must start with '/'. using /canlog");
    snprintf(c.up_path, sizeof(c.up_path), "/canlog");
  }
  size_t pn = strlen(c.up_path);
  while (pn > 1 && c.up_path[pn - 1] == '/') c.up_path[--pn] = '\0';

  Serial.printf("[OK] " CFG_PATH " loaded (%u lines)\n", static_cast<unsigned>(lineNo - 1));
  return true;
}
