#!/usr/bin/env python3
"""
upload_server.py — P6（Wi-Fiアップロード）検証用の受信サーバ

治具が投げる HTTP PUT を受けてローカルに保存するだけの最小サーバ。
SORACOM Harvest Files のエントリポイントと同じ「PUT したパスにそのまま置く」
振る舞いを模しているので、P7 で送信先を Harvest Files に切り替えても
治具側のコードは変えなくてよい。

使い方
    python3 tools/upload_server.py --dir ./received --port 8080

    # 治具側 /CONFIG.TXT
    UP_HOST = <PCのIPアドレス>
    UP_PORT = 8080

動作確認用のオプション
    --fail-rate 0.3     30%の確率で500を返す（再送処理の検証）
    --drop-rate 0.2     20%の確率で応答せず切断する（切断時の再送検証）
    --slow 50           1KBごとに50ms待つ（低速回線の模擬）

注) 検証専用。認証もアクセス制限も無いので、閉じたLANでのみ使うこと。
"""

import argparse
import os
import random
import sys
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ARGS = None


def log(msg):
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}", flush=True)


def safe_join(base, urlpath):
    """パストラバーサルを防ぎつつ、URLパスをローカルパスへ変換する"""
    rel = urlpath.lstrip("/")
    rel = rel.split("?", 1)[0]
    full = os.path.normpath(os.path.join(base, rel))
    if not os.path.abspath(full).startswith(os.path.abspath(base)):
        return None
    return full


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *a):
        pass  # 既定のアクセスログは抑制する

    def _recv_body(self, length):
        buf = bytearray()
        remain = length
        while remain > 0:
            chunk = self.rfile.read(min(4096, remain))
            if not chunk:
                break
            buf += chunk
            remain -= len(chunk)
            if ARGS.slow:
                time.sleep(ARGS.slow / 1000.0 * (len(chunk) / 1024.0))
        return bytes(buf)

    def do_PUT(self):
        length = int(self.headers.get("Content-Length", 0))
        path = safe_join(ARGS.dir, self.path)
        if path is None:
            self.send_response(400)
            self.send_header("Content-Length", "0")
            self.end_headers()
            log(f"REJECT  {self.path}")
            return

        t0 = time.time()
        body = self._recv_body(length)
        dt = time.time() - t0

        if ARGS.drop_rate and random.random() < ARGS.drop_rate:
            log(f"DROP    {self.path}  ({len(body)} byte received, no response)")
            self.close_connection = True
            return

        if ARGS.fail_rate and random.random() < ARGS.fail_rate:
            self.send_response(500)
            self.send_header("Content-Length", "0")
            self.end_headers()
            log(f"FAIL500 {self.path}")
            return

        if len(body) != length:
            self.send_response(400)
            self.send_header("Content-Length", "0")
            self.end_headers()
            log(f"SHORT   {self.path}  {len(body)}/{length} byte")
            return

        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "wb") as f:
            f.write(body)

        kbps = (len(body) / 1024.0 / dt) if dt > 0 else 0
        self.send_response(200)
        self.send_header("Content-Length", "0")
        self.end_headers()
        log(f"PUT  OK {self.path}  {len(body)} byte  {dt:.1f}s  {kbps:.0f} kB/s")

    def do_GET(self):
        """遠隔コマンドファイルの取得（P9の先取り。無ければ404）"""
        path = safe_join(ARGS.dir, self.path)
        if path is None or not os.path.isfile(path):
            self.send_response(404)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        with open(path, "rb") as f:
            data = f.read()
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)
        log(f"GET  OK {self.path}  {len(data)} byte")


def main():
    global ARGS
    ap = argparse.ArgumentParser(description="CANロガー治具のアップロード検証用サーバ")
    ap.add_argument("--dir", default="./received", help="保存先ディレクトリ")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--fail-rate", type=float, default=0.0,
                    help="この確率で500を返す（再送検証用, 0.0-1.0）")
    ap.add_argument("--drop-rate", type=float, default=0.0,
                    help="この確率で応答せず切断する（0.0-1.0）")
    ap.add_argument("--slow", type=int, default=0,
                    help="1KBあたりの追加待ち時間[ms]")
    ARGS = ap.parse_args()

    os.makedirs(ARGS.dir, exist_ok=True)
    srv = ThreadingHTTPServer((ARGS.bind, ARGS.port), Handler)

    log(f"listening on {ARGS.bind}:{ARGS.port}  ->  {os.path.abspath(ARGS.dir)}")
    if ARGS.fail_rate or ARGS.drop_rate or ARGS.slow:
        log(f"fault injection: fail={ARGS.fail_rate} drop={ARGS.drop_rate} slow={ARGS.slow}ms/KB")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        log("bye")
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
