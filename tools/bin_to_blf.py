#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bin_to_blf.py — CANログ取得治具の CANLOG02 バイナリを Vector BLF へ変換する

使い方:
    pip install python-can
    python bin_to_blf.py LOG0001.BIN -o LOG0001.blf
    python bin_to_blf.py LOG0001.BIN -o LOG0001.blf --start "2026-07-30 14:05:00"

ファイル形式（リトルエンディアン）
    ヘッダ 32byte
        +0  char[8]  magic        "CANLOG02"
        +8  uint16   version      2
        +10 uint16   record_size  24
        +12 uint32   ch1_bitrate  [bps]
        +16 uint32   ch2_bitrate  [bps] 0ならCH2無効
        +20 uint32   unix_time    計測開始のUNIX時刻(UTC)。0ならRTC無効/未設定
        +24 uint64   t0_us        計測開始時刻（治具のesp_timer基準）
    レコード 24byte × N
        +0  uint64   ts_us        受信時刻（esp_timer基準の絶対値）
        +8  uint32   id           11bit または 29bit
        +12 uint8    dlc          0..8
        +13 uint8    flags        bit0: 29bit ID / bit1: リモートフレーム
        +14 uint8    channel      1 = CH1 / 2 = CH2
        +15 uint8    reserved     0
        +16 uint8[8] data

備考
    - レコードはチャネル内では時刻順だが、チャネル間の順序は保証しない。
      本スクリプトは念のため ts_us で安定ソートしてから書き出す。
    - BLFのチャネル番号は 1 = CAN1 / 2 = CAN2 になる（CANalyzerの設定と一致）。

タイムゾーンの扱い（重要）
    治具は unix_time に UTC を格納する。一方 python-can は BLF ヘッダの
    計測開始時刻（SYSTEMTIME）へ UTC をそのまま書き込むが、
    CANalyzer はこのフィールドを「現地時刻」として表示する。
    そのため何も補正しないと CANalyzer 上で UTC がそのまま出てしまう
    （日本なら9時間前にずれる）。

    本スクリプトは既定で全タイムスタンプに現地のUTCオフセットを加算し、
    CANalyzer に現地時刻が表示されるようにしている。
    全フレームを同じ量だけずらすので、フレーム間の相対時刻は変わらない。
    UTC のまま書き出したい場合は --utc を指定する。
"""

import argparse
import datetime
import os
import struct
import sys

import can

HDR = struct.Struct("<8sHHIIIQ")
REC = struct.Struct("<QIBBBB8s")

MAGIC = b"CANLOG02"

FLAG_EXTENDED = 0x01
FLAG_RTR = 0x02


def parse_args():
    p = argparse.ArgumentParser(description="CANLOG02 binary -> Vector BLF")
    p.add_argument("input", help="治具が出力した .BIN ファイル")
    p.add_argument("-o", "--output", help="出力する .blf（既定: 入力名 + .blf）")
    p.add_argument(
        "--start",
        help='計測開始の実時刻 "YYYY-MM-DD HH:MM:SS"。'
        "省略時はヘッダのunix_time(UTC)、それも0なら入力ファイルの更新時刻を使う",
    )
    p.add_argument(
        "--utc",
        action="store_true",
        help="BLFへUTCのまま書き出す（既定は現地時刻に補正してCANalyzerの表示を合わせる）",
    )
    p.add_argument(
        "--tz-offset",
        type=float,
        help="現地時刻への補正量[時間]。省略時はこのPCのタイムゾーンを使う（日本なら9）",
    )
    return p.parse_args()


def resolve_tz_offset(args):
    """BLFへ書き込む際に加算する秒数を返す。"""
    if args.utc:
        return 0.0, "UTCのまま出力"
    if args.tz_offset is not None:
        return args.tz_offset * 3600.0, f"指定 UTC{args.tz_offset:+g}"
    off = datetime.datetime.now().astimezone().utcoffset()
    sec = off.total_seconds() if off else 0.0
    return sec, f"このPCのタイムゾーン UTC{sec / 3600:+g}"


def resolve_start_epoch(args, unix_time):
    if args.start:
        dt = datetime.datetime.strptime(args.start, "%Y-%m-%d %H:%M:%S")
        return dt.timestamp(), "コマンドライン指定"
    if unix_time:
        return float(unix_time), "治具のRTC"
    return os.path.getmtime(args.input), "入力ファイルの更新時刻（推定）"


def main():
    args = parse_args()
    out_path = args.output or (os.path.splitext(args.input)[0] + ".blf")

    with open(args.input, "rb") as f:
        raw = f.read()

    if len(raw) < HDR.size:
        sys.exit("エラー: ファイルが短すぎます")

    magic, version, rec_size, ch1_bps, ch2_bps, unix_time, t0_us = HDR.unpack_from(raw, 0)
    if magic != MAGIC:
        sys.exit(f"エラー: magicが一致しません ({magic!r})")
    if rec_size != REC.size:
        sys.exit(f"エラー: record_size={rec_size} は未対応です")

    body = raw[HDR.size :]
    n_full, remainder = divmod(len(body), REC.size)
    if remainder:
        print(
            f"警告: 末尾 {remainder} byte が不完全です（記録中に電源が切れた可能性）。破棄します",
            file=sys.stderr,
        )

    start_epoch, start_src = resolve_start_epoch(args, unix_time)
    tz_shift, tz_src = resolve_tz_offset(args)

    records = []
    for i in range(n_full):
        ts_us, can_id, dlc, flags, channel, _rsv, data = REC.unpack_from(body, i * REC.size)
        records.append((ts_us, can_id, dlc, flags, channel, data))

    # チャネル間の順序を保証していないので時刻順に並べ替える
    records.sort(key=lambda r: r[0])

    per_ch = {}
    with can.BLFWriter(out_path) as writer:
        for ts_us, can_id, dlc, flags, channel, data in records:
            rel = (ts_us - t0_us) / 1_000_000.0
            # CANalyzer は BLF ヘッダの時刻を現地時刻として表示するため、
            # 全フレームを一律にずらして辻褄を合わせる（相対時刻は不変）
            msg = can.Message(
                timestamp=start_epoch + rel + tz_shift,
                arbitration_id=can_id,
                is_extended_id=bool(flags & FLAG_EXTENDED),
                is_remote_frame=bool(flags & FLAG_RTR),
                dlc=dlc,
                data=b"" if (flags & FLAG_RTR) else data[:dlc],
                # BLFWriterが +1 するので、CH1 -> BLF channel 1 になる
                channel=channel - 1,
                is_rx=True,
            )
            writer.on_message_received(msg)
            per_ch[channel] = per_ch.get(channel, 0) + 1

    duration = (records[-1][0] - records[0][0]) / 1_000_000.0 if records else 0.0
    print(f"入力            : {args.input}")
    print(f"出力            : {out_path}")
    print(f"version         : {version}")
    print(f"CH1 / CH2       : {ch1_bps} bps / {ch2_bps} bps")
    print(f"開始実時刻      : {datetime.datetime.fromtimestamp(start_epoch)} ({start_src})")
    print(f"BLFへの時刻補正 : {tz_shift / 3600:+g} 時間 ({tz_src})")
    print(f"フレーム数      : {len(records)}  " + "  ".join(f"CH{k}={v}" for k, v in sorted(per_ch.items())))
    print(f"記録長          : {duration:.3f} s")


if __name__ == "__main__":
    main()
