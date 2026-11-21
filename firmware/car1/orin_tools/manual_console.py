"""Car 1 manual/automatic console reusing the deployed UART bridge and steering law."""
import argparse
import csv
import json
import math
import os
import secrets
import signal
import subprocess
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32, Float32MultiArray

from car1_bringup.uart_bridge_node import UartBridgeNode
from car1_bringup.uart_protocol import build_cmd
from manual_control import ManualControl
from auto_control import automatic_fwa, validate_auto_gains


class ManualConsole(UartBridgeNode):
    def __init__(self, host, port):
        self.lock = threading.RLock()
        self.operator = ManualControl()
        self.metrics = None
        self.metrics_t = 0.0
        self.stopped_since = None
        self.session = None
        self.mode = 'manual'
        self.kp_lateral, self.kp_heading = 150.0, 20.0
        self.auto_fwa, self.auto_t = 0.0, None
        self.host, self.port = host, port
        self.token = secrets.token_urlsafe(24)
        super().__init__()
        self._ser.exclusive = True
        # The bridge subscribes only to a private topic. /cmd_vel here reports
        # this owner's actual bounded output to the existing viewer.
        self.display_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(Float32MultiArray, '/line_follow/bev_metrics', self.on_metrics, 1)
        self.run_dir = Path.home() / 'car1_ws/run' / time.strftime('manual_console_%Y%m%d_%H%M%S')
        self.run_dir.mkdir(parents=True)
        self.log_file = (self.run_dir / 'commands_feedback.csv').open('w', newline='')
        self.log = csv.writer(self.log_file)
        self.log.writerow(['unix_s', 'armed', 'drive', 'go', 'request_fwa_deg', 'request_speed_pct',
                           'sent_fwa_deg', 'sent_speed_pct', 'vL_rpm', 'vR_rpm', 'rpmL', 'rpmR',
                           'posL', 'posR', 'dutyL', 'dutyR', 'fault', 'ackseq', 'reason'])
        self.last_flush = 0.0
        self.output = (0.0, 0.0)
        self.http = ThreadingHTTPServer((host, port), handler_for(self))
        self.http.daemon_threads = True
        threading.Thread(target=self.http.serve_forever, daemon=True).start()

    def on_metrics(self, msg):
        with self.lock:
            self.metrics, self.metrics_t = list(msg.data), time.monotonic()

    def _receive(self):
        with self.lock:
            super()._receive()

    def feedback(self, now):
        sta = self._core.sta or {}
        fresh = self._core._last_sta is not None and now - self._core._last_sta < 0.5
        ack = sta.get('ackseq', -1)
        ack_ok = ack >= 0 and ((self._core.seq - ack) & 0xFFFF) <= self._core.ack_lag_max
        connected = bool(fresh and ack_ok)
        fault = sta.get('f') if fresh else None
        signed = (sta.get('vL'), sta.get('vR'))
        stationary = fresh and all(v is not None and abs(v) <= 3 for v in signed)
        if stationary:
            if self.stopped_since is None:
                self.stopped_since = now
        else:
            self.stopped_since = None
        stopped = self.stopped_since is not None and now - self.stopped_since >= 0.3
        return connected, fault, stopped, sta

    def start(self, data):
        with self.lock:
            now = time.monotonic()
            mode = data.get('mode', 'manual')
            if mode not in ('manual', 'auto'):
                raise ValueError('控制模式錯誤')
            gains = validate_auto_gains(data.get('kp_lateral', 150), data.get('kp_heading', 20))
            self.operator.configure(data.get('drive_limit_sec', 30))
            if mode == 'auto':
                automatic_fwa(self.metrics, now - self.metrics_t, *gains)
            connected, fault, stopped, _ = self.feedback(now)
            if self._core.fault_gate.latched or self._core.fault_gate.blocking(now):
                raise ValueError('BM 故障停車閘門尚未放行')
            if self.other_controller():
                raise ValueError('另一個控制器正在發布命令，請先停止它')
            self.operator.arm(now, ready=data.get('ready'), drive=data.get('drive'),
                              connected=connected, fault=fault, stopped=stopped)
            self.mode, (self.kp_lateral, self.kp_heading) = mode, gains
            self.auto_fwa, self.auto_t = 0.0, now
            # Re-arm only after fresh signed wheel feedback proves standstill.
            # Preserve sequence, STA and the persistent fault latch.
            self._core.drive_used_sec = 0.0
            self._core._limit_hit = False
            self._core._last_tick = None
            self._core.cap = self.operator.MAX_SPEED
            self._core.max_drive_sec = 0.0
            self.session = secrets.token_urlsafe(16)
            return {'ok': True, 'session': self.session}

    def command(self, data):
        with self.lock:
            if not self.session or data.get('session') != self.session:
                raise ValueError('工作階段已失效，請重新開始')
            self.operator.command(time.monotonic(), fwa=data.get('fwa'), speed=data.get('speed'),
                                  go=True if self.mode == 'auto' else data.get('go'), seq=data.get('seq'))
            return {'ok': True}

    def stop(self):
        with self.lock:
            self.operator.stop('操作員停止')
            self.session = None

    def other_controller(self):
        return any(p.node_name != self.get_name() for p in self.get_publishers_info_by_topic('/cmd_vel'))

    def _send(self):
        with self.lock:
            now = time.monotonic()
            connected, fault, _, sta = self.feedback(now)
            if self.operator.armed and self.other_controller():
                self.operator.stop('另一個控制器已啟動，手動已停止')
            if self._core.fault_gate.latched:
                self.operator.stop('重複故障鎖定，需重新啟動控制頁服務')
            if self.operator.armed and self.mode == 'auto':
                try:
                    target = automatic_fwa(self.metrics, now - self.metrics_t, self.kp_lateral, self.kp_heading)
                    dt = max(0, now - self.auto_t)
                    self.auto_t = now
                    self.auto_fwa += (1 - math.exp(-dt / 0.08)) * (target - self.auto_fwa)
                    self.operator.fwa = self.auto_fwa
                except ValueError as exc:
                    self.operator.stop(str(exc))
            fwa, speed = self.operator.tick(now, connected=connected, fault=fault)
            self._core.on_twist(speed / 100.0, -fwa / 50.0, now)
            seq, fwa, left, right, reason = self._core.next_cmd(now)
            try:
                self._ser.write(build_cmd(seq, fwa, left, right))
            except Exception:
                self.operator.stop('UART 寫入失敗')
                raise
            self.output = (fwa, left)
            self._pub_fwa.publish(Float32(data=float(fwa)))
            msg = Twist()
            msg.linear.x, msg.angular.z = left / 100.0, -fwa / 50.0
            self.display_pub.publish(msg)
            if reason == 'time_limit':
                self.operator.stop(f'已達橋接 {self.operator.drive_limit:g} 秒上限；需重新開始')
            self.log.writerow([f'{time.time():.3f}', int(self.operator.armed), int(self.operator.drive),
                               int(self.operator.go), self.operator.fwa, self.operator.speed, fwa, left,
                               *(sta.get(k, '') for k in ('vL', 'vR', 'rpmL', 'rpmR', 'posL', 'posR',
                                                         'dutyL', 'dutyR', 'f', 'ackseq')),
                               reason or self.operator.reason])
            if now - self.last_flush >= 1:
                self.log_file.flush()
                self.last_flush = now

    def status(self):
        with self.lock:
            now = time.monotonic()
            connected, fault, stopped, sta = self.feedback(now)
            m = self.metrics if now - self.metrics_t < 0.5 else None
            return dict(armed=self.operator.armed, drive=self.operator.drive, reason=self.operator.reason, mode=self.mode,
                        kp_lateral=self.kp_lateral, kp_heading=self.kp_heading,
                        connected=connected, stopped=stopped, fault=fault,
                        sent_fwa=self.output[0], sent_speed=self.output[1],
                        rpm_l=sta.get('vL') if connected else None,
                        rpm_r=sta.get('vR') if connected else None,
                        near_cm=m[4] * 100 if m and len(m) >= 9 and m[6] >= 0.5 else None,
                        far_cm=m[5] * 100 if m and len(m) >= 9 and m[7] >= 0.5 else None,
                        speed_max=self.operator.MAX_SPEED, drive_limit_sec=self.operator.drive_limit,
                        remaining_s=max(0, self.operator.drive_limit - max(self.operator.used, self._core.drive_used_sec)),
                        run_dir=str(self.run_dir), telemetry=sta if connected else None)


def handler_for(app):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def authorized(self, query=False):
            token = parse_qs(urlparse(self.path).query).get('token', [''])[0] if query else self.headers.get('X-Control-Token', '')
            return secrets.compare_digest(token, app.token)

        def reply(self, code, data):
            body = json.dumps(data, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Referrer-Policy', 'no-referrer')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlparse(self.path).path
            if path == '/' and self.authorized(query=True):
                body = Path(__file__).with_name('manual_console.html').read_bytes()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Referrer-Policy', 'no-referrer')
                self.end_headers()
                self.wfile.write(body)
            elif path == '/api/status' and self.authorized():
                self.reply(200, app.status())
            elif path in ('/overlay', '/bev') and self.authorized(query=True):
                port = 8082 if path == '/overlay' else 8083
                try:
                    with urllib.request.urlopen(f'http://127.0.0.1:{port}/stream', timeout=5) as stream:
                        self.send_response(200)
                        self.send_header('Content-Type', stream.headers['Content-Type'])
                        self.send_header('Cache-Control', 'no-store')
                        self.end_headers()
                        while True:
                            chunk = stream.read1(32768)
                            if not chunk:
                                break
                            self.wfile.write(chunk)
                            self.wfile.flush()
                except (OSError, TimeoutError):
                    pass
            else:
                self.reply(403, {'error': '請使用本次控制頁的完整連結'})

        def do_POST(self):
            origin = self.headers.get('Origin')
            if not self.authorized() or (origin is not None and origin != f'http://{app.host}:{app.port}'):
                self.reply(403, {'error': '控制連結或來源不符'})
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 2048:
                    raise ValueError('請求大小錯誤')
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError('請求必須是物件')
                path = urlparse(self.path).path
                if path == '/api/start':
                    result = app.start(data)
                elif path == '/api/cmd':
                    result = app.command(data)
                elif path == '/api/stop':
                    app.stop()
                    result = {'ok': True}
                else:
                    self.reply(404, {'error': '找不到端點'})
                    return
                self.reply(200, result)
            except (ValueError, TypeError) as exc:
                self.reply(400, {'error': str(exc)})
    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default=os.environ.get('CAR1_CONSOLE_HOST', '127.0.0.1'))
    parser.add_argument('--port', type=int, default=8092)
    args = parser.parse_args()
    if subprocess.run(['fuser', '/dev/ttyTHS1'], capture_output=True).returncode == 0:
        raise SystemExit('UART 正被其他程式使用；請先停止自動控制')
    rclpy.init(args=['--ros-args', '-r', '__node:=car1_manual_uart',
                    '-p', 'cmd_vel_topic:=/car1/manual_unused/cmd_vel',
                    '-p', 'max_wheel_speed:=100.0', '-p', 'max_steer_deg:=20.0', '-p', 'max_drive_sec:=0.0'])
    app = None
    try:
        app = ManualConsole(args.host, args.port)
        link = f'http://{args.host}:{args.port}/?token={app.token}'
        Path('/tmp/car1_manual_console_link.txt').write_text(link + '\n')
        print('MANUAL_PAGE_READY', flush=True)
        signal.signal(signal.SIGTERM, lambda *_: rclpy.try_shutdown())
        rclpy.spin(app)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if app:
            app.stop()
            app.close()
            app.http.shutdown()
            app.log_file.close()
            app.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
