/*
 * mcp2515.h — MCP2515 受信専用ミニマルドライバ（外部ライブラリ不要）
 *
 * この治具ではCH2の受信にのみ使うため、送信系は一切実装していない。
 *   - Listen-Onlyモードで動作（ACKも返さないためバスに影響しない）
 *   - フィルタ/マスクは無効化（全フレーム受信）
 *   - RXB0 → RXB1 のロールオーバ(BUKT)を有効化し、2段分をバッファとして使う
 *
 * 注意: MCP2515はVDD 2.7〜5.5Vで動作するため、治具基板では3.3V単一電源で使う。
 *       市販の5V(TJA1050)モジュールをESP32へ直結してはいけない。
 */
#pragma once

#include <Arduino.h>
#include <SPI.h>

class Mcp2515 {
 public:
  // ---- 戻り値 ----
  enum Result : uint8_t { OK = 0, NO_FRAME = 1, ERR = 2 };

  // resetPin に GPIO を渡すと、初期化前にハードウェアリセットを行う。
  // -1 を渡すとソフトウェアリセット命令のみを使う。
  bool begin(SPIClass* spi, int8_t csPin, uint32_t bitrate, uint32_t spiHz,
             int8_t resetPin = -1) {
    spi_   = spi;
    cs_    = csPin;
    spiHz_ = spiHz;

    pinMode(cs_, OUTPUT);
    digitalWrite(cs_, HIGH);

    // ハードウェアリセット（/RESETをLowに落とす）
    if (resetPin >= 0) {
      pinMode(resetPin, OUTPUT);
      digitalWrite(resetPin, LOW);
      delayMicroseconds(10);   // データシートの最小パルス幅より十分長く
      digitalWrite(resetPin, HIGH);
      delay(10);
    }

    reset();
    delay(10);
    // リセット直後はコンフィグレーションモード(OPMOD=100b)になっているはず
    if ((readReg(REG_CANSTAT) & 0xE0) != 0x80) return false;

    uint8_t cnf1, cnf2, cnf3;
    if (!bitTiming(bitrate, cnf1, cnf2, cnf3)) return false;
    writeReg(REG_CNF1, cnf1);
    writeReg(REG_CNF2, cnf2);
    writeReg(REG_CNF3, cnf3);

    // RXM=11b → マスク/フィルタを使わず全フレーム受信、BUKT=1 → RXB0満杯時にRXB1へ流す
    writeReg(REG_RXB0CTRL, 0x64);
    writeReg(REG_RXB1CTRL, 0x60);

    writeReg(REG_CANINTE, 0x03);  // RX0IE | RX1IE
    writeReg(REG_CANINTF, 0x00);
    writeReg(REG_EFLG, 0x00);

    // Listen-Onlyモードへ（REQOP=011b）
    writeReg(REG_CANCTRL, 0x60);
    delay(10);
    if ((readReg(REG_CANSTAT) & 0xE0) != 0x60) return false;

    return true;
  }

  // 受信バッファに溜まっているフレームを1つ取り出す。
  // フレームが無ければ NO_FRAME を返す（連続で呼んで空になるまで読む使い方）。
  Result readFrame(uint32_t& id, uint8_t& dlc, uint8_t& flags, uint8_t* data8) {
    const uint8_t status = readStatus();
    uint8_t cmd;
    if (status & 0x01) {
      cmd = CMD_READ_RX0;  // RXB0
    } else if (status & 0x02) {
      cmd = CMD_READ_RX1;  // RXB1
    } else {
      return NO_FRAME;
    }

    uint8_t b[13];
    spi_->beginTransaction(SPISettings(spiHz_, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_, LOW);
    spi_->transfer(cmd);  // READ RX BUFFER命令。読み終わりで該当RXnIFが自動クリアされる
    for (uint8_t i = 0; i < 13; i++) b[i] = spi_->transfer(0x00);
    digitalWrite(cs_, HIGH);
    spi_->endTransaction();

    flags = 0;
    const bool ide = (b[1] & 0x08) != 0;
    if (ide) {
      id = ((uint32_t)b[0] << 21) | ((uint32_t)(b[1] >> 5) << 18) |
           ((uint32_t)(b[1] & 0x03) << 16) | ((uint32_t)b[2] << 8) | (uint32_t)b[3];
      flags |= 0x01;                       // extended
      if (b[4] & 0x40) flags |= 0x02;      // RTR (extended時はDLCレジスタのRTRビット)
    } else {
      id = ((uint32_t)b[0] << 3) | ((uint32_t)b[1] >> 5);
      if (b[1] & 0x10) flags |= 0x02;      // SRR (standard時のリモートフレーム)
    }
    dlc = b[4] & 0x0F;
    if (dlc > 8) dlc = 8;
    memcpy(data8, &b[5], 8);
    return OK;
  }

  // EFLGのオーバフロービットを読み、立っていればクリアして true を返す
  bool checkAndClearOverflow() {
    const uint8_t eflg = readReg(REG_EFLG);
    if (eflg & 0xC0) {  // RX1OVR | RX0OVR
      bitModify(REG_EFLG, 0xC0, 0x00);
      return true;
    }
    return false;
  }

  uint8_t readReg(uint8_t addr) {
    spi_->beginTransaction(SPISettings(spiHz_, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_, LOW);
    spi_->transfer(CMD_READ);
    spi_->transfer(addr);
    const uint8_t v = spi_->transfer(0x00);
    digitalWrite(cs_, HIGH);
    spi_->endTransaction();
    return v;
  }

 private:
  // ---- 命令 ----
  static const uint8_t CMD_RESET     = 0xC0;
  static const uint8_t CMD_READ      = 0x03;
  static const uint8_t CMD_WRITE     = 0x02;
  static const uint8_t CMD_STATUS    = 0xA0;
  static const uint8_t CMD_BITMODIFY = 0x05;
  static const uint8_t CMD_READ_RX0  = 0x90; // RXB0SIDHから読み出し
  static const uint8_t CMD_READ_RX1  = 0x94; // RXB1SIDHから読み出し

  // ---- レジスタ ----
  static const uint8_t REG_CANSTAT  = 0x0E;
  static const uint8_t REG_CANCTRL  = 0x0F;
  static const uint8_t REG_CNF3     = 0x28;
  static const uint8_t REG_CNF2     = 0x29;
  static const uint8_t REG_CNF1     = 0x2A;
  static const uint8_t REG_CANINTE  = 0x2B;
  static const uint8_t REG_CANINTF  = 0x2C;
  static const uint8_t REG_EFLG     = 0x2D;
  static const uint8_t REG_RXB0CTRL = 0x60;
  static const uint8_t REG_RXB1CTRL = 0x70;

  void reset() {
    spi_->beginTransaction(SPISettings(spiHz_, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_, LOW);
    spi_->transfer(CMD_RESET);
    digitalWrite(cs_, HIGH);
    spi_->endTransaction();
  }

  void writeReg(uint8_t addr, uint8_t val) {
    spi_->beginTransaction(SPISettings(spiHz_, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_, LOW);
    spi_->transfer(CMD_WRITE);
    spi_->transfer(addr);
    spi_->transfer(val);
    digitalWrite(cs_, HIGH);
    spi_->endTransaction();
  }

  void bitModify(uint8_t addr, uint8_t mask, uint8_t val) {
    spi_->beginTransaction(SPISettings(spiHz_, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_, LOW);
    spi_->transfer(CMD_BITMODIFY);
    spi_->transfer(addr);
    spi_->transfer(mask);
    spi_->transfer(val);
    digitalWrite(cs_, HIGH);
    spi_->endTransaction();
  }

  uint8_t readStatus() {
    spi_->beginTransaction(SPISettings(spiHz_, MSBFIRST, SPI_MODE0));
    digitalWrite(cs_, LOW);
    spi_->transfer(CMD_STATUS);
    const uint8_t v = spi_->transfer(0x00);
    digitalWrite(cs_, HIGH);
    spi_->endTransaction();
    return v;  // bit0=RX0IF, bit1=RX1IF
  }

  /*
   * ビットタイミング定数（水晶16MHz前提）
   *
   * 3レートすべて 16TQ／サンプルポイント 87.5%（14TQ目でサンプル）に統一している。
   *   Sync 1TQ + PropSeg 6TQ + PhaseSeg1 7TQ + PhaseSeg2 2TQ = 16TQ
   *   → CNF2 = 0xB5 (BTLMODE=1, SAM=0, PHSEG1=6d, PRSEG=5d)
   *   → CNF3 = 0x81 (SOF=1, WAKFIL=0, PHSEG2=1d)
   *   CNF1 でTQ長とSJWを決める（SJW=1TQ固定）
   *     500k : BRP=0 → TQ = 2*(0+1)/16MHz = 125ns、16TQ = 2us
   *     250k : BRP=1 → TQ = 250ns、16TQ = 4us
   *     125k : BRP=3 → TQ = 500ns、16TQ = 8us
   *
   * MCP2515データシートの制約チェック
   *   PhaseSeg2(2TQ) > SJW(1TQ)                 … OK
   *   PropSeg + PhaseSeg1 (13TQ) >= PhaseSeg2   … OK
   *   PhaseSeg2 >= 2TQ                          … OK
   *
   * 注) autowp系ライブラリの一般的な定数表（500k: 0x00/0xF0/0x86 等）は
   *     サンプルポイントが 9/16 = 56.25% と低く、バスの立ち上がりリンギングに弱い。
   *     CiA推奨の87.5%に合わせるため独自に算出している。
   *     水晶を8MHzに変更する場合はこの表を再計算すること。
   */
  static bool bitTiming(uint32_t bitrate, uint8_t& cnf1, uint8_t& cnf2, uint8_t& cnf3) {
    cnf2 = 0xB5;
    cnf3 = 0x81;
    switch (bitrate) {
      case 500000UL: cnf1 = 0x00; return true; // SJW=1TQ, BRP=0
      case 250000UL: cnf1 = 0x01; return true; // SJW=1TQ, BRP=1
      case 125000UL: cnf1 = 0x03; return true; // SJW=1TQ, BRP=3
      default: return false;
    }
  }

  SPIClass* spi_  = nullptr;
  int8_t    cs_   = -1;
  uint32_t  spiHz_ = 10000000UL;
};
