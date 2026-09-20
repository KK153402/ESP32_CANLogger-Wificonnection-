/*
 * asc_format.h — CanRec → Vector ASC 1行 への変換（LOG_FORMAT_ASC時のみ使用）
 *
 * snprintf("%f") は重いので整数演算のみで組み立てる。
 * 出力書式は python-can の ASCWriter に合わせてあるため、
 * PC側は can.ASCReader → can.BLFWriter で素通しできる。
 *
 *   例)    0.010000 1  101             Rx   d 8 00 11 22 33 44 55 66 77
 *          [ts]     [ch] [ID(+x)]      Rx   [d/r] [DLC] [data...]
 */
#pragma once

#include <stdint.h>
#include "ring_buffer.h"

#define ASC_LINE_MAX 80  // 1行の最大長（余裕込み）

namespace ascfmt {

static const char kHex[] = "0123456789ABCDEF";

// 符号なし整数を10進で書く。戻り値は書いた文字数
static inline size_t putDec(char* p, uint32_t v) {
  char tmp[10];
  size_t n = 0;
  do {
    tmp[n++] = static_cast<char>('0' + (v % 10));
    v /= 10;
  } while (v);
  for (size_t i = 0; i < n; i++) p[i] = tmp[n - 1 - i];
  return n;
}

// 符号なし整数を10進・固定桁でゼロ埋めして書く
static inline void putDecFixed(char* p, uint32_t v, size_t width) {
  for (size_t i = width; i > 0; i--) {
    p[i - 1] = static_cast<char>('0' + (v % 10));
    v /= 10;
  }
}

// 符号なし整数を大文字16進で書く（前ゼロなし）。戻り値は書いた文字数
static inline size_t putHex(char* p, uint32_t v) {
  char tmp[8];
  size_t n = 0;
  do {
    tmp[n++] = kHex[v & 0x0F];
    v >>= 4;
  } while (v);
  for (size_t i = 0; i < n; i++) p[i] = tmp[n - 1 - i];
  return n;
}

}  // namespace ascfmt

/*
 * ASC 1行を out に生成する（末尾に '\n' を含む）。戻り値は書いたバイト数。
 * out は ASC_LINE_MAX 以上の容量が必要。
 * t0_us は計測開始時刻（"Start of measurement" の 0.000000 に対応）。
 * チャネル番号は rec.channel をそのまま使う（1=CAN1 / 2=CAN2）。
 */
static inline size_t formatAscLine(char* out, const CanRec& rec, uint64_t t0_us) {
  char* p = out;

  // ---- タイムスタンプ: 右寄せ幅11の "sec.usususu" ----
  const uint64_t d   = (rec.ts_us >= t0_us) ? (rec.ts_us - t0_us) : 0;
  const uint32_t sec = static_cast<uint32_t>(d / 1000000ULL);
  const uint32_t us  = static_cast<uint32_t>(d % 1000000ULL);

  char secbuf[12];
  const size_t seclen = ascfmt::putDec(secbuf, sec);
  const size_t tslen  = seclen + 1 + 6;            // "sec" + "." + 6桁
  for (size_t i = tslen; i < 11; i++) *p++ = ' ';  // 幅11に右寄せ
  for (size_t i = 0; i < seclen; i++) *p++ = secbuf[i];
  *p++ = '.';
  ascfmt::putDecFixed(p, us, 6);
  p += 6;

  // ---- チャネル ----
  *p++ = ' ';
  p += ascfmt::putDec(p, rec.channel);
  *p++ = ' ';
  *p++ = ' ';

  // ---- ID（大文字16進、29bitは末尾に 'x'）を幅15で左寄せ ----
  char* const idStart = p;
  p += ascfmt::putHex(p, rec.id);
  if (rec.flags & CANREC_FLAG_EXTENDED) *p++ = 'x';
  while (static_cast<size_t>(p - idStart) < 15) *p++ = ' ';

  // ---- 方向（治具はListen-Onlyなので常にRx） ----
  *p++ = ' ';
  *p++ = 'R';
  *p++ = 'x';
  *p++ = ' ';
  *p++ = ' ';
  *p++ = ' ';

  // ---- データ/リモート + DLC ----
  const bool rtr = (rec.flags & CANREC_FLAG_RTR) != 0;
  *p++ = rtr ? 'r' : 'd';
  *p++ = ' ';
  p += ascfmt::putDec(p, rec.dlc);

  // ---- データバイト ----
  if (!rtr) {
    const uint8_t n = (rec.dlc > 8) ? 8 : rec.dlc;
    for (uint8_t i = 0; i < n; i++) {
      *p++ = ' ';
      *p++ = ascfmt::kHex[rec.data[i] >> 4];
      *p++ = ascfmt::kHex[rec.data[i] & 0x0F];
    }
  }

  *p++ = '\n';
  return static_cast<size_t>(p - out);
}
