#!/usr/bin/env python3
"""Orin 端：以 20 Hz 對 /cmd_vel 發布一段受控的速度/轉向曲線，供方法 B（穩定性測試）使用。

要搭配 uart_bridge_node 一起跑（linear_scale=100、angular_scale=50 的預設值）：
  輪速：0 -> 2/3 -> 1 -> 2/3 -> 0（乘以 --pct-max），每 1.5 秒換一階，只往前轉，不反轉。
  轉向：fwa 在 +-8 度之間每 0.5 秒切換。
安全：--pct-max 硬上限 60；結束或被中斷時一定發布零速。需要輪子離地。
"""
import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

CAP = 60.0
PROFILE = [0.0, 2 / 3, 1.0, 2 / 3, 0.0]
STEP_S = 1.5
FLIP_S = 0.5
AMP_DEG = 8.0
LINEAR_SCALE = 100.0     # uart_bridge_node 預設：speed% = linear.x * 100
ANGULAR_SCALE = 50.0     # uart_bridge_node 預設：fwa = -angular.z * 50


class Profile(Node):
    def __init__(self, pct_max: float, seconds: float):
        super().__init__('method_b_profile')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pct_max = max(0.0, min(CAP, pct_max))
        self.seconds = seconds
        self.t0 = time.monotonic()
        self.finished = False
        self.published = 0
        self.create_timer(0.05, self.tick)

    def tick(self):
        rel = time.monotonic() - self.t0
        msg = Twist()
        if rel < self.seconds:
            pct = self.pct_max * PROFILE[int(rel / STEP_S) % len(PROFILE)]
            sign = 1.0 if int(rel / FLIP_S) % 2 == 0 else -1.0
            msg.linear.x = pct / LINEAR_SCALE
            msg.angular.z = -sign * AMP_DEG / ANGULAR_SCALE
        else:
            self.finished = True
        self.pub.publish(msg)
        self.published += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pct-max', type=float, required=True)
    ap.add_argument('--seconds', type=float, required=True)
    args = ap.parse_args()

    rclpy.init()
    node = Profile(args.pct_max, args.seconds)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
        end = time.monotonic() + 1.0          # 結束後再送 1 秒零速
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            for _ in range(5):
                node.pub.publish(Twist())
                time.sleep(0.02)
        except Exception:     # noqa: BLE001
            pass
        print(f'profile done: published {node.published} Twist, pct_max={node.pct_max}', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
