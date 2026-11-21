"""Automatic preview controller using the deployed car1 steering law."""
import math
from car1_bringup.steering_law import lane_errors, chord_heading_rad, steering_fwa_deg


def automatic_fwa(metrics, age, kp_lateral, kp_heading):
    if metrics is None or age > 0.3:
        raise ValueError('感知資料逾時，自動循線已停止')
    near, heading = lane_errors(metrics)
    if near is None:
        raise ValueError('近點白線無效，自動循線已停止')
    if heading is None:
        heading = chord_heading_rad(metrics, 0.61, 1.10)
    return steering_fwa_deg(near, heading if heading is not None else 0.0,
                            kp_lateral, kp_heading, 20.0)


def validate_auto_gains(lateral, heading):
    for value, limit in ((lateral, 200), (heading, 30)):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= limit:
            raise ValueError('近點增益須為 0..200，航向增益須為 0..30 的有限數字')
    return float(lateral), float(heading)
