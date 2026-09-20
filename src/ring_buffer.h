/*
 * ring_buffer.h — 単一生産者/単一消費者リングバッファ
 *
 * 生産者: CH1=rxTaskTwai / CH2=rxTaskMcp（それぞれ専用のリングを持つ）
 * 消費者: writerTask（両リングからタイムスタンプ順に取り出す）
 *
 * head は生産者のみ、tail は消費者のみが書くのでロック不要。
 * ESP32はデュアルコアなのでメモリバリアを明示している。
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// ------------------------------------------------------------
// CAN 1フレーム分のレコード（24byte固定・リトルエンディアン）
// SDへ保存されるバイナリと完全に同じレイアウト。
// PC側の変換スクリプトは struct '<QIBBBB8s' で読める。
// ------------------------------------------------------------
struct CanRec {
  uint64_t ts_us;    // +0  esp_timer_get_time() の絶対値（起動からのus）
  uint32_t id;       // +8  11bit または 29bit
  uint8_t  dlc;      // +12 0..8
  uint8_t  flags;    // +13 下記ビット定義
  uint8_t  channel;  // +14 1 = CH1(TWAI) / 2 = CH2(MCP2515)
  uint8_t  reserved; // +15 0固定
  uint8_t  data[8];  // +16
};

enum : uint8_t {
  CANREC_FLAG_EXTENDED = 0x01, // 29bit ID
  CANREC_FLAG_RTR      = 0x02, // リモートフレーム
};

static_assert(sizeof(CanRec) == 24, "CanRec must be exactly 24 bytes");
static_assert(offsetof(CanRec, id) == 8, "unexpected layout");
static_assert(offsetof(CanRec, dlc) == 12, "unexpected layout");
static_assert(offsetof(CanRec, data) == 16, "unexpected layout");

// ------------------------------------------------------------
template <uint32_t N>
class SpscRing {
  static_assert((N & (N - 1)) == 0, "ring size must be a power of two");

 public:
  // --- 生産者側からのみ呼ぶ ---
  bool push(const CanRec& rec) {
    const uint32_t h    = head_;
    const uint32_t next = (h + 1) & (N - 1);
    if (next == tail_) {  // 満杯
      dropped_++;
      return false;
    }
    buf_[h] = rec;
    __sync_synchronize();  // データ書き込みがhead更新より先に見えることを保証
    head_ = next;
    const uint32_t used = (next - tail_) & (N - 1);
    if (used > max_used_) max_used_ = used;
    return true;
  }

  // --- 消費者側からのみ呼ぶ ---
  // 先頭フレームのタイムスタンプだけを覗く（取り出さない）
  bool peekTs(uint64_t& ts) const {
    const uint32_t t = tail_;
    if (t == head_) return false;
    ts = buf_[t].ts_us;
    return true;
  }

  bool pop(CanRec& out) {
    const uint32_t t = tail_;
    if (t == head_) return false;  // 空
    out = buf_[t];
    __sync_synchronize();
    tail_ = (t + 1) & (N - 1);
    return true;
  }

  bool empty() const { return head_ == tail_; }
  uint32_t dropped() const { return dropped_; }
  uint32_t maxUsed() const { return max_used_; }
  uint32_t capacity() const { return N; }

  void reset() {
    head_ = tail_ = 0;
    dropped_ = max_used_ = 0;
  }

 private:
  CanRec buf_[N];
  volatile uint32_t head_     = 0;
  volatile uint32_t tail_     = 0;
  volatile uint32_t dropped_  = 0;  // 生産者のみ更新
  volatile uint32_t max_used_ = 0;  // 生産者のみ更新
};
