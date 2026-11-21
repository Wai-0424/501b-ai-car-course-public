"""車一 BMduino 本機 USB 測試共用函式：CMD/TUNE 封包、STA 解析（含 checksum 驗證）。

STA 格式與車二相同（ackseq/S1/rpmL/rpmR/vL/vR/f 在前，車一專屬欄位 dutyL/dutyR/posL/posR/unit/upms 在後）。
左輪 = M2（bwl）、右輪 = M1（bwr）。recovery v3 的 USB STA 10 Hz、dbg 2 Hz；舊版頻率較高。
"""
import time

import serial


def checksum(payload: str) -> int:
    cs = 0
    for b in payload.encode('ascii'):
        cs ^= b
    return cs


def frame(payload: str) -> bytes:
    return f"<{payload},cs {checksum(payload):02X}>\n".encode()


def parse_sta(line: str):
    """checksum 通過回傳 dict（值為字串），否則 None。"""
    line = line.strip()
    if not (line.startswith('<STA,') and line.endswith('>')):
        return None
    parts = line[1:-1].split(',')
    kv, cs = [], None
    for part in parts[1:]:
        key, _, val = part.partition(' ')
        if key == 'cs':
            try:
                cs = int(val, 16)
            except ValueError:
                return None
        else:
            kv.append((key, val))
    if cs is None:
        return None
    payload = 'STA,' + ','.join(f'{k} {v}' for k, v in kv)
    if checksum(payload) != cs:
        return None
    return dict(kv)


class Car1:
    def __init__(self, port='COM25', baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.seq = int(time.time()) % 50000 + 1000
        self.bad = 0
        self.dbg = None
        self._rx = bytearray()
        time.sleep(0.3)  # 開啟 USB 序列埠可能讓 BMduino 重開機
        self.ser.reset_input_buffer()

    def close(self):
        try:
            self.cmd(0, 0)
        finally:
            self.ser.close()

    def cmd(self, left, right, fwa=0, brk=None):
        """brk=None 不帶 brk 欄位（韌體維持上一個值）；0/1 則明確送出。"""
        self.seq = (self.seq + 1) % 65536
        extra = '' if brk is None else f",brk {brk}"
        self.ser.write(frame(f"CMD,seq {self.seq},fwa {fwa},bwl {left},bwr {right}{extra}"))

    def tune(self, **kw):
        body = ','.join(f'{k} {v}' for k, v in kw.items())
        self.ser.write(frame(f"TUNE,{body}"))

    def poll(self, budget=0.02):
        """限時讀取已到達的資料；跨 poll 保留半行，回傳 STA 列表與最新 dbg。

        韌體 USB 輸出幾乎佔滿頻寬時 readline 會一直有資料可讀；沒有時間上限的話呼叫端的 CMD 會被餓死、
        超過 500 ms 韌體就當作失聯把速度歸零（2026-11-03 測煞車時踩到）。"""
        out = []
        deadline = time.monotonic() + budget
        while time.monotonic() < deadline:
            count = min(self.ser.in_waiting, 512)
            if count <= 0:
                break
            # 只讀 in_waiting 已有的 bytes，不等待 Serial.timeout；避免 41 ms
            # 長的 STA 被舊 20 ms readline timeout 切半後丟棄。
            raw = self.ser.read(count)
            if not raw:
                break
            self._rx.extend(raw)
            while b'\n' in self._rx:
                raw, _, remaining = self._rx.partition(b'\n')
                self._rx = bytearray(remaining)
                try:
                    text = raw.decode('ascii').strip()
                except UnicodeDecodeError:
                    self.bad += 1
                    continue
                if text.startswith('<STA'):
                    sta = parse_sta(text)
                    if sta is None:
                        self.bad += 1
                    else:
                        out.append(sta)
                elif text.startswith('dbg,'):
                    self.dbg = text.split(',')[1:]
            if len(self._rx) > 4096:  # 雜訊／缺少換行時也限制記憶體
                self._rx.clear()
                self.bad += 1
        return out

    def run(self, left, right, seconds, period=0.05, fwa=0):
        """每 period 送一次 CMD，持續 seconds；回傳 [(t, sta)]。"""
        rows = []
        t0 = time.monotonic()
        nxt = 0.0
        while True:
            t = time.monotonic() - t0
            if t >= seconds:
                break
            if t >= nxt:
                self.cmd(left, right, fwa)
                nxt += period
            for sta in self.poll():
                rows.append((time.monotonic() - t0, sta))
        return rows
