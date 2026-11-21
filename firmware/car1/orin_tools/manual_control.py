"""Bounded operator control; independent of ROS and UART for verification."""
import math


class ManualControl:
    """Control state for an operator-controlled session."""
    MAX_SPEED = 100.0
    MAX_STEER = 20.0
    COMMAND_TTL = 0.35

    def __init__(self):
        self.armed = False
        self.drive = False
        self.fwa = self.speed = self.used = self.output_speed = 0.0
        self.drive_limit = 0.0
        self.go = False
        self.last_command = self.last_tick = None
        self.seq = -1
        self.reason = '等待開始'

    def configure(self, _drive_limit=None):
        """Compatibility hook: the operator console has no drive-time cutoff."""
        self.drive_limit = 0.0

    def stop(self, reason='已停止'):
        self.armed = self.go = False
        self.fwa = self.speed = self.output_speed = 0.0
        self.reason = reason

    def arm(self, now, *, ready, drive, connected, fault, stopped):
        if ready is not True or not isinstance(drive, bool):
            raise ValueError('請確認現場已準備好，並選擇控制模式')
        if self.armed:
            raise ValueError('請先停止目前工作階段')
        if not connected or fault is None or fault != 0 or not stopped:
            raise ValueError('需要雙向 UART 正常、無故障且兩輪已停穩')
        self.stop()
        self.armed, self.drive = True, drive
        self.used = 0.0
        self.seq = -1
        self.last_command = self.last_tick = now
        self.reason = '只轉向' if not drive else '已就緒，按住才前進'

    def command(self, now, *, fwa, speed, go, seq):
        if not self.armed:
            raise ValueError('請先開始控制')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
               for v in (fwa, speed)) or not isinstance(go, bool) or type(seq) is not int:
            self.stop('指令格式錯誤，已停止')
            raise ValueError('轉向、速度必須是有限數字；按住狀態及序號格式錯誤')
        if seq <= self.seq:
            raise ValueError('已忽略過時指令')
        self.seq = seq
        self.fwa = max(-self.MAX_STEER, min(self.MAX_STEER, float(fwa)))
        self.speed = max(0.0, min(self.MAX_SPEED, float(speed)))
        self.go, self.last_command = go, now

    def tick(self, now, *, connected, fault):
        dt = 0.0 if self.last_tick is None else max(0.0, now - self.last_tick)
        self.last_tick = now
        if self.output_speed > 0:
            self.used += dt
        if self.armed:
            if not connected:
                self.stop('BM 通訊中斷，已停止')
            elif fault is None or fault != 0:
                self.stop('BM 故障，已停止')
            elif now - self.last_command > self.COMMAND_TTL:
                self.stop('操作頁失聯，已停止；需重新開始')
        if not self.armed:
            return 0.0, 0.0
        wanted = self.speed if self.drive and self.go else 0.0
        self.output_speed = min(wanted, self.output_speed + 8.0 * min(dt, 0.1)) if wanted > self.output_speed else wanted
        self.reason = '前進中' if self.output_speed > 0 else ('只轉向' if not self.drive else '已就緒，按住才前進')
        return self.fwa, self.output_speed
