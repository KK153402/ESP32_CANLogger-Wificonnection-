#!/usr/bin/env python3
"""
merge_parts.py — アップロードされた分割パートを1つのBINへ連結する

治具は 1MB ごとに分割して送るため、受信側には
    20260916_150227_LOG0007_001.bin
    20260916_150227_LOG0007_002.bin
    ...
という並びで置かれる。これを連結して bin_to_blf.py にかけられる形に戻す。

使い方
    python3 tools/merge_parts.py received/canlog/JIG001
        → 同じディレクトリに 20260916_150227_LOG0007.bin を作る

    python3 tools/merge_parts.py received/canlog/JIG001 -o ./merged

チェック内容
    - パート番号が 001 から連番で揃っているか（欠けていれば中断する）
    - 連結後の先頭が "CANLOG02" で始まるか
    - 連結後のサイズが「32 + 24 × N」になっているか（BIN形式の整合性）
"""

import argparse
import os
import re
import struct
import sys
from collections import defaultdict

PART_RE = re.compile(r"^(?P<stem>.+_LOG\d{4})_(?P<part>\d{3})\.bin$", re.IGNORECASE)

HEADER_SIZE = 32
RECORD_SIZE = 24
MAGIC = b"CANLOG02"


def collect(src):
    """{stem: {part_no: path}} を作る"""
    groups = defaultdict(dict)
    for name in os.listdir(src):
        m = PART_RE.match(name)
        if not m:
            continue
        groups[m.group("stem")][int(m.group("part"))] = os.path.join(src, name)
    return groups


def verify_header(path):
    with open(path, "rb") as f:
        head = f.read(HEADER_SIZE)
    if len(head) < HEADER_SIZE or head[:8] != MAGIC:
        return None
    (_magic, version, rec_size, ch1, ch2, unix_time, t0) = struct.unpack(
        "<8sHHIIIQ", head)
    return {
        "version": version,
        "record_size": rec_size,
        "ch1_bitrate": ch1,
        "ch2_bitrate": ch2,
        "unix_time": unix_time,
        "t0_us": t0,
    }


def merge_one(stem, parts, outdir):
    nums = sorted(parts.keys())
    if nums[0] != 1:
        print(f"[SKIP] {stem}: part 001 が見つかりません", file=sys.stderr)
        return False
    expected = list(range(1, nums[-1] + 1))
    missing = [n for n in expected if n not in parts]
    if missing:
        print(f"[SKIP] {stem}: パート欠落 {missing}", file=sys.stderr)
        print("       治具側で UPLOAD LOG%s を再実行してください" % stem[-4:],
              file=sys.stderr)
        return False

    info = verify_header(parts[1])
    if info is None:
        print(f"[SKIP] {stem}: part 001 が CANLOG02 ヘッダで始まっていません",
              file=sys.stderr)
        return False

    os.makedirs(outdir, exist_ok=True)
    outpath = os.path.join(outdir, f"{stem}.bin")
    total = 0
    with open(outpath, "wb") as out:
        for n in expected:
            with open(parts[n], "rb") as f:
                data = f.read()
            out.write(data)
            total += len(data)

    body = total - HEADER_SIZE
    ok = (body >= 0 and body % RECORD_SIZE == 0)
    frames = body // RECORD_SIZE if ok else -1

    status = "OK" if ok else "**サイズ不整合**"
    print(f"[{status}] {outpath}")
    print(f"    parts={len(expected)}  bytes={total}  frames={frames}")
    print(f"    CH1={info['ch1_bitrate']} bps  CH2={info['ch2_bitrate']} bps  "
          f"unix_time={info['unix_time']}")
    if not ok:
        print("    連結後のサイズが 32 + 24xN になっていません。"
              "パートの順序か内容を確認してください", file=sys.stderr)
    return ok


def main():
    ap = argparse.ArgumentParser(description="分割アップロードされたBINを連結する")
    ap.add_argument("src", help="パートが置かれたディレクトリ")
    ap.add_argument("-o", "--out", default=None, help="出力先（既定: srcと同じ）")
    args = ap.parse_args()

    if not os.path.isdir(args.src):
        print(f"ディレクトリがありません: {args.src}", file=sys.stderr)
        return 1

    outdir = args.out or args.src
    groups = collect(args.src)
    if not groups:
        print("連結対象のパートが見つかりませんでした", file=sys.stderr)
        return 1

    ng = 0
    for stem in sorted(groups):
        if not merge_one(stem, groups[stem], outdir):
            ng += 1
    return 1 if ng else 0


if __name__ == "__main__":
    sys.exit(main())
