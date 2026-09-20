/*
 * mcp7940.h — MCP7940N-I/SN 用ミニマルドライバ（外部ライブラリ不要）
 *
 * この治具では「記録開始時刻をBINヘッダに残す」ためだけに使うので、
 * アラーム・矩形波出力・SRAM・パワーフェイルタイムスタンプは実装していない。
 *
 * MCP7940N を使ううえでの必須事項（どれも忘れると動かない）
 *   1. 発振器は RTCSEC の bit7 = ST を 1 にしないと起動しない
 *   2. バッテリバックアップは RTCWKDAY の bit3 = VBATEN を 1 にしないと働かない
 *      （デフォルト無効。忘れると電源断で時刻が消える）
 *   3. 外部32.768kHz水晶が必要。
 *      MCP7940Nのオシレータは **CL = 6〜9pF の水晶に最適化**されている
 *      （データシート冒頭 "Oscillator for 32.768 kHz Crystals: Optimized for 6-9 pF crystals"）。
 *      負荷容量は CL = CX/2 + Cstray, CX = 外付け容量 + COSC(3pF) から逆算する。
 *      CL=6〜7pF の水晶なら外付けは 6〜8pF 程度。
 *      CL=12.5pF品を使うと実効CLを合わせるために負荷容量を重くする必要があり、
 *      発振余裕を削るので推奨しない（周波数誤差はデジタルトリミングで補正可）。
 *
 * 時刻の扱い
 *   RTCには「現地時刻（日本時間）」を設定する運用とし、
 *   本ドライバがUTCのUNIX時刻へ変換して返す（config.h の RTC_UTC_OFFSET_SEC）。
 *
 * レジスタマップ（0x00〜0x09の抜粋）
 *   0x00 RTCSEC    bit7=ST      bit6:0=秒(BCD)
 *   0x01 RTCMIN                 bit6:0=分(BCD)
 *   0x02 RTCHOUR   bit6=12/24   bit5:0=時(BCD, 24h形式)
 *   0x03 RTCWKDAY  bit5=OSCRUN  bit4=PWRFAIL  bit3=VBATEN  bit2:0=曜日(1..7)
 *   0x04 RTCDATE                bit5:0=日(BCD)
 *   0x05 RTCMTH    bit5=LPYR    bit4:0=月(BCD)
 *   0x06 RTCYEAR                年(BCD, 00..99)
 *   0x07 CONTROL
 *   0x08 OSCTRIM   デジタルトリミング
 */
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

class Mcp7940 {
 public:
  struct DateTime {
    uint16_t year;   // 2000..2099
    uint8_t  month;  // 1..12
    uint8_t  day;    // 1..31
    uint8_t  hour;   // 0..23
    uint8_t  minute; // 0..59
    uint8_t  second; // 0..59
  };

  bool begin(TwoWire* wire) {
    wire_ = wire;
    uint8_t v;
    if (!readReg(REG_RTCWKDAY, v)) return false;  // 疎通確認
    present_ = true;

    // バッテリバックアップを有効化する（デフォルト無効のため必須）。
    // 時刻レジスタは触らないので、既に動いている時計は止めない。
    // 書き込み結果は読み戻して検証する。ここが失敗していると
    // 「電源断で時刻が消える」症状になるが、記録自体は続行できるため
    // 戻り値はtrueのままにして、STATUSコマンドで確認できるようにする。
    if (!(v & BIT_VBATEN)) {
      writeReg(REG_RTCWKDAY, static_cast<uint8_t>(v | BIT_VBATEN));
      uint8_t chk;
      if (readReg(REG_RTCWKDAY, chk) && !(chk & BIT_VBATEN)) {
        vbatenFailed_ = true;
      }
    }
    return true;
  }

  bool present() const { return present_; }
  bool vbatenFailed() const { return vbatenFailed_; }

  // 発振器が回っているか（OSCRUN）。false = 時刻が進んでいない
  bool oscillatorRunning() {
    uint8_t v;
    if (!readReg(REG_RTCWKDAY, v)) return false;
    return (v & BIT_OSCRUN) != 0;
  }

  // 電源断が発生したか（PWRFAIL）。true = バックアップに落ちた履歴がある
  bool powerFailed() {
    uint8_t v;
    if (!readReg(REG_RTCWKDAY, v)) return false;
    return (v & BIT_PWRFAIL) != 0;
  }

  // バッテリバックアップが有効か（VBATEN）。
  // begin() で立てているが、I2C書き込みが失敗していると無効のままになる。
  bool batteryEnabled() {
    uint8_t v;
    if (!readReg(REG_RTCWKDAY, v)) return false;
    return (v & BIT_VBATEN) != 0;
  }

  // 診断用: RTCWKDAY の生値（bit5=OSCRUN, bit4=PWRFAIL, bit3=VBATEN）
  uint8_t rawWkday() {
    uint8_t v = 0;
    readReg(REG_RTCWKDAY, v);
    return v;
  }

  /*
   * デジタルトリミング（OSCTRIM, 0x08）
   *
   * 水晶の周波数誤差をソフトで補正する。補正範囲 ±129ppm、分解能 1.017ppm/step。
   * CRSTRIM=0（既定）では1分ごとに 2×TRIMVAL クロックパルスを加減算する。
   * バックアップ動作中も補正が効く。
   *
   * steps: 正 = クロックを加算（時計を速める） / 負 = 減算（時計を遅らせる）
   *        0 でトリミング無効。範囲 -127..+127。
   *
   * 使い方: 既知の正確な時刻と比べて1日あたりの進み/遅れを測り、
   *         ppm = 秒差/日 ÷ 86400 × 1e6 を求めて steps = -ppm/1.017 を書く。
   *         （時計が進んでいるなら負の値を入れる）
   *         符号の向きは実測で必ず確認すること。
   */
  bool setTrim(int8_t steps) {
    uint8_t v;
    if (steps >= 0) {
      v = static_cast<uint8_t>(0x80 | (steps & 0x7F));  // SIGN=1: 加算
    } else {
      v = static_cast<uint8_t>((-steps) & 0x7F);        // SIGN=0: 減算
    }
    return writeReg(REG_OSCTRIM, v);
  }

  bool readTrim(int8_t& steps) {
    uint8_t v;
    if (!readReg(REG_OSCTRIM, v)) return false;
    const int8_t mag = static_cast<int8_t>(v & 0x7F);
    steps = (v & 0x80) ? mag : static_cast<int8_t>(-mag);
    return true;
  }

  bool readDateTime(DateTime& dt) {
    if (!present_) return false;
    uint8_t b[7];
    if (!readRegs(REG_RTCSEC, b, 7)) return false;
    dt.second = bcd2bin(b[0] & 0x7F);          // bit7 = ST を除く
    dt.minute = bcd2bin(b[1] & 0x7F);
    dt.hour   = bcd2bin(b[2] & 0x3F);          // 24時間表記のみを扱う
    dt.day    = bcd2bin(b[4] & 0x3F);
    dt.month  = bcd2bin(b[5] & 0x1F);          // bit5 = LPYR を除く
    dt.year   = 2000 + bcd2bin(b[6]);
    if (dt.month < 1 || dt.month > 12 || dt.day < 1 || dt.day > 31) return false;
    if (dt.hour > 23 || dt.minute > 59 || dt.second > 59) return false;
    return true;
  }

  bool setDateTime(const DateTime& dt) {
    if (!present_) return false;

    // 書き込み中にカウントアップしないよう、いったん発振を止める
    if (!writeReg(REG_RTCSEC, 0x00)) return false;  // ST=0
    delay(5);

    const uint8_t dow = static_cast<uint8_t>(weekdayFromCivil(dt.year, dt.month, dt.day) + 1);

    uint8_t wk;
    if (!readReg(REG_RTCWKDAY, wk)) return false;
    // VBATEN は維持し、PWRFAIL はクリア（書き込みで自動的に0になる）
    const uint8_t wkNew = static_cast<uint8_t>((wk & BIT_VBATEN) | (dow & 0x07));

    uint8_t b[7];
    b[0] = bin2bcd(dt.second);                 // STはまだ立てない
    b[1] = bin2bcd(dt.minute);
    b[2] = bin2bcd(dt.hour);                   // bit6=0 で24時間表記
    b[3] = wkNew;
    b[4] = bin2bcd(dt.day);
    b[5] = bin2bcd(dt.month);
    b[6] = bin2bcd(static_cast<uint8_t>(dt.year - 2000));
    if (!writeRegs(REG_RTCSEC, b, 7)) return false;

    // 最後に ST を立てて発振開始を指示する。
    // 実際に発振が立ち上がるまでは待たない（waitOscillatorStart を別途呼ぶこと）。
    return writeReg(REG_RTCSEC, static_cast<uint8_t>(bin2bcd(dt.second) | BIT_ST));
  }

  /*
   * 発振が立ち上がるまで待つ。
   *
   * 32.768kHzの音叉型水晶は起動が遅く、発振開始まで数百ms〜数秒かかる。
   * 水晶やC27/C28の定数が正常でも1秒以上かかることがあるため、
   * 短いタイムアウトで打ち切ると「水晶不良」と誤判定する。
   * 既定を5秒としているのはこのため。
   */
  bool waitOscillatorStart(uint32_t timeoutMs = 5000) {
    const uint32_t t0 = millis();
    for (;;) {
      if (oscillatorRunning()) return true;
      if (millis() - t0 >= timeoutMs) return false;
      delay(50);
    }
  }

  // UTCのUNIX時刻を返す。時刻が無効なら0を返す。
  uint32_t readUnixTimeUtc() {
    if (!oscillatorRunning()) return 0;
    DateTime dt;
    if (!readDateTime(dt)) return 0;
    const int64_t local = toEpoch(dt);
    const int64_t utc   = local - static_cast<int64_t>(RTC_UTC_OFFSET_SEC);
    if (utc <= 0) return 0;
    return static_cast<uint32_t>(utc);
  }

  // ---- 日付ユーティリティ（他モジュールからも使う） ----

  // 1970-01-01 からの経過秒（引数はそのままの暦時刻として扱う）
  static int64_t toEpoch(const DateTime& dt) {
    return static_cast<int64_t>(daysFromCivil(dt.year, dt.month, dt.day)) * 86400LL +
           dt.hour * 3600LL + dt.minute * 60LL + dt.second;
  }

  // 0=日曜 .. 6=土曜
  static uint8_t weekdayFromCivil(uint16_t y, uint8_t m, uint8_t d) {
    const int64_t days = daysFromCivil(y, m, d);
    int64_t w = (days + 4) % 7;  // 1970-01-01 は木曜(=4)
    if (w < 0) w += 7;
    return static_cast<uint8_t>(w);
  }

 private:
  static const uint8_t REG_RTCSEC   = 0x00;
  static const uint8_t REG_RTCWKDAY = 0x03;
  static const uint8_t REG_CONTROL  = 0x07;
  static const uint8_t REG_OSCTRIM  = 0x08;

  static const uint8_t BIT_ST      = 0x80;  // RTCSEC   bit7 発振器起動
  static const uint8_t BIT_OSCRUN  = 0x20;  // RTCWKDAY bit5 発振中
  static const uint8_t BIT_PWRFAIL = 0x10;  // RTCWKDAY bit4 電源断履歴
  static const uint8_t BIT_VBATEN  = 0x08;  // RTCWKDAY bit3 バックアップ有効

  // Howard Hinnant の days_from_civil アルゴリズム
  static int64_t daysFromCivil(int64_t y, unsigned m, unsigned d) {
    y -= (m <= 2);
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);              // 0..399
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;    // 0..365
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;             // 0..146096
    return era * 146097 + static_cast<int64_t>(doe) - 719468;
  }

  static uint8_t bcd2bin(uint8_t v) { return static_cast<uint8_t>((v >> 4) * 10 + (v & 0x0F)); }
  static uint8_t bin2bcd(uint8_t v) { return static_cast<uint8_t>(((v / 10) << 4) | (v % 10)); }

  bool readReg(uint8_t addr, uint8_t& out) { return readRegs(addr, &out, 1); }

  bool readRegs(uint8_t addr, uint8_t* buf, uint8_t len) {
    wire_->beginTransmission(RTC_I2C_ADDR);
    wire_->write(addr);
    if (wire_->endTransmission(false) != 0) return false;
    if (wire_->requestFrom(static_cast<uint8_t>(RTC_I2C_ADDR), len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = static_cast<uint8_t>(wire_->read());
    return true;
  }

  bool writeReg(uint8_t addr, uint8_t val) { return writeRegs(addr, &val, 1); }

  bool writeRegs(uint8_t addr, const uint8_t* buf, uint8_t len) {
    wire_->beginTransmission(RTC_I2C_ADDR);
    wire_->write(addr);
    for (uint8_t i = 0; i < len; i++) wire_->write(buf[i]);
    return wire_->endTransmission() == 0;
  }

  TwoWire* wire_ = nullptr;
  bool     present_ = false;
  bool     vbatenFailed_ = false;
};
