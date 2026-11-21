"""Per-wheel motor + feedback verification for car2 (direct UART, no ROS nodes).

usage (stdin):  python3 - precheck
                python3 - motion [speed_pct=25] [pulse_s=2.0]

precheck: no wheel motion (passive listen, then zero-speed commands).
motion:   one wheel at a time (L fwd, R fwd, L rev, R rev), then both, then a
          command-silence (watchdog) test. Always ends with a confirmed zero.
fwa is always 0 (steering is not touched by this test).
"""
import json, math, signal, statistics as st, subprocess, sys, time
from pathlib import Path

sys.path.insert(0, str(Path.home() / 'car2_ws/src/car2_bringup'))
import serial
from car2_bringup.uart_protocol import build_cmd, parse_sta, send_chunked

mode = sys.argv[1] if len(sys.argv) > 1 else 'precheck'
SPEED = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0
PULSE = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
ONLY = sys.argv[4].split(',') if len(sys.argv) > 4 else None  # e.g. L_fwd,R_fwd,L_rev,R_rev,both_fwd,watchdog
if SPEED > 30 or (PULSE > 3 and mode != 'handspin') or PULSE > 30:
    raise SystemExit('refusing: speed >30 % or pulse >3 s (handspin: >30 s)')
def expected_rpm(pct):
    """L298N profile of the car firmware: 0 -> 0, otherwise effective% = 5 + 0.95*pct of 105 ticks/20 ms (2.3275 rpm per tick)."""
    pct = abs(pct)
    return 0.0 if pct <= 0 else (5.0 + 0.95 * pct) / 100.0 * 105.0 * 2.3275


FULL_RPM = {'L': 344.89, 'R': 344.16}  # (legacy XY-160D numbers, no longer used)

root = Path.home() / 'car2_ws/run' / time.strftime('claude_motor_verify_%Y%m%d_%H%M%S')
root.mkdir(parents=True)
trace = (root / 'trace.jsonl').open('w')
report = {'mode': mode, 'speed_pct': SPEED, 'pulse_s': PULSE, 'phases': [], 'error': None, 'final_zero_confirmed': None}

if subprocess.run(['fuser', '/dev/ttyTHS1'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
    raise SystemExit('UART already in use; cancelled')
port = serial.Serial('/dev/ttyTHS1', 460800, timeout=0.02, write_timeout=0.2, exclusive=True)

seq = 3000
closing = False
last_sta_t = None
last_ack = None
last_ack_t = None
latest = None
last_rpm = {'L': None, 'R': None}
last_f = 0
LAST_VSER = []


def interrupted(_s, _f):
    if not closing:
        raise KeyboardInterrupt()


signal.signal(signal.SIGINT, interrupted)
signal.signal(signal.SIGTERM, interrupted)


def phase(name, bwl, bwr, dur, tx=True, guard=True):
    """Run one phase; returns dict incl. raw series (t, rpmL, rpmR)."""
    global seq, last_sta_t, last_ack, last_ack_t, latest, last_f, LAST_VSER
    t0 = time.monotonic()
    next_tx = t0
    sent, acks, series = [], set(), []
    vser, fset = [], set()
    bad = 0
    moving = bool(bwl or bwr)
    while True:
        now = time.monotonic()
        if now - t0 >= dur:
            break
        if tx and now >= next_tx:
            pkt = build_cmd(seq, 0, bwl, bwr)
            trace.write(json.dumps({'t': time.time(), 'phase': name, 'tx': pkt.decode().strip()}) + '\n')
            send_chunked(port, pkt)
            sent.append(seq)
            seq = (seq + 1) % 65536
            next_tx = now + 0.1
        line = port.readline()
        if line:
            p = parse_sta(line)
            if p:
                tm = time.monotonic()
                try:
                    rl, rr, ack = float(p['rpmL']), float(p['rpmR']), int(p['ackseq'])
                    # firmware 2026-10-02+: always-on signed speed (forward +) and fault flags
                    vl, vr, fl = float(p.get('vL', 'nan')), float(p.get('vR', 'nan')), int(p.get('f', '0'))
                except (KeyError, ValueError):
                    bad += 1
                    continue
                last_f = fl
                fset.add(fl)
                vser.append((tm - t0, vl, vr))
                latest = p
                last_sta_t = tm
                if ack != last_ack:
                    last_ack, last_ack_t = ack, tm
                if ack in sent:
                    acks.add(ack)
                last_rpm['L'], last_rpm['R'] = rl, rr
                series.append((tm - t0, rl, rr))
                trace.write(json.dumps({'t': time.time(), 'phase': name, 'rx': p}) + '\n')
            elif line.endswith(b'\n') and line.strip():
                bad += 1
        if guard and tx and moving:
            if last_sta_t is None or now - last_sta_t > 0.5 or last_ack_t is None or now - last_ack_t > 0.5:
                raise RuntimeError(f'UART feedback lost during {name}')
            if last_f != 0:
                raise RuntimeError(f'FIRMWARE FAULT f={last_f} (bit0 L stall, bit1 R stall, bit2 PCA9685/dir I2C) in {name}')
            # --- per-wheel response guards: abort at once (final_stop sends zero) ---
            el = now - t0
            for side, cmdv in (('L', bwl), ('R', bwr)):
                v = last_rpm[side]
                if v is None:
                    continue
                target = expected_rpm(cmdv)
                if cmdv:
                    if el >= 0.6 and abs(v) < 5:
                        raise RuntimeError(f'NO RESPONSE on {side} wheel ({v} rpm after {el:.2f}s of command) in {name}')
                    if el >= 0.3 and abs(v) > 1.8 * target:
                        raise RuntimeError(f'OVERSPEED on {side} wheel ({v} rpm vs target {target:.0f}) in {name}')
                elif abs(v) > 15:
                    raise RuntimeError(f'UNEXPECTED MOTION on idle {side} wheel ({v} rpm) in {name}')
    res = {'phase': name, 'bwl': bwl, 'bwr': bwr, 'dur': dur, 'sent': len(sent), 'acked': len(acks),
           'n_sta': len(series), 'sta_hz': round(len(series) / dur, 2), 'bad_lines': bad,
           'f_seen': sorted(fset)}
    LAST_VSER = vser
    return res, series


def stats(series, lo, hi, idx):
    v = [s[idx] for s in series if lo <= s[0] <= hi]
    if not v:
        return None
    return {'mean': round(st.mean(v), 1), 'std': round(st.pstdev(v), 1), 'min': round(min(v), 1),
            'max': round(max(v), 1), 'n': len(v)}


def stop_time(series):
    for t, rl, rr in series:
        if abs(rl) < 5 and abs(rr) < 5:
            return round(t, 2)
    return None


def run_precheck():
    r, s = phase('passive_listen', 0, 0, 2.0, tx=False, guard=False)
    r['rest_rpm'] = {'L': stats(s, 0, 9, 1), 'R': stats(s, 0, 9, 2)}
    report['phases'].append(r)
    r, s = phase('zero_cmd', 0, 0, 3.0)
    r['rest_rpm'] = {'L': stats(s, 0, 9, 1), 'R': stats(s, 0, 9, 2)}
    report['phases'].append(r)


def run_motion():
    plan = [('L_fwd', SPEED, 0, 'L', 1), ('R_fwd', 0, SPEED, 'R', 2),
            ('L_rev', -SPEED, 0, 'L', 1), ('R_rev', 0, -SPEED, 'R', 2)]
    rest0, s0 = phase('rest_before', 0, 0, 1.0)
    report['phases'].append(rest0)
    for name, bwl, bwr, side, idx in plan:
        if ONLY and name not in ONLY:
            continue
        other = 2 if idx == 1 else 1
        r, s = phase(name, bwl, bwr, PULSE)
        target = expected_rpm(SPEED)
        steady = stats(s, PULSE - 1.0, PULSE, idx)
        oth = stats(s, PULSE - 1.0, PULSE, other)
        spin = next((round(t, 2) for t, a, b in s if (a if idx == 1 else b) >= 0.5 * target), None)
        vst = {'L': stats(LAST_VSER, PULSE - 1.0, PULSE, 1), 'R': stats(LAST_VSER, PULSE - 1.0, PULSE, 2)}
        r.update({'wheel': side, 'target_rpm': round(target, 1), 'steady': steady, 'other_wheel_steady': oth,
                  'spinup_s': spin, 'v_steady_signed_rpm': vst})
        rest, rs = phase('rest_after_' + name, 0, 0, 1.2)
        r['stop_s'] = stop_time(LAST_VSER)  # from vL/vR (real coast-down), not the forced-0 rpm
        r['rest_stats'] = {'max_abs_rpm_end': max((max(abs(a), abs(b)) for t, a, b in rs if t > 0.8), default=None)}
        ok = []
        if steady:
            ok.append(('dir_positive', steady['mean'] > 0))
            ok.append(('mag_within_25pct', 0.75 * target <= steady['mean'] <= 1.25 * target))
            ok.append(('steady_std<=15%', steady['std'] <= 0.15 * max(1.0, abs(steady['mean']))))
        if oth:
            ok.append(('other_wheel_idle', abs(oth['mean']) < 5 and max(abs(oth['min']), abs(oth['max'])) < 15))
        vd = vst[side]
        if vd:
            want_sign = 1 if bwl + bwr > 0 else -1
            ok.append(('v_sign_matches_command', vd['mean'] * want_sign > 0))
            ok.append(('v_magnitude_matches_rpm', abs(abs(vd['mean']) - abs(steady['mean'])) <= max(5.0, 0.15 * abs(steady['mean']))) if steady else ('v_magnitude_matches_rpm', False))
        r['checks'] = {k: bool(v) for k, v in ok}
        report['phases'].append(r)
        report['phases'].append(rest)
    # both wheels forward
    if not ONLY or 'both_fwd' in ONLY:
        r, s = phase('both_fwd', SPEED, SPEED, PULSE)
        r['steady'] = {'L': stats(s, PULSE - 1.0, PULSE, 1), 'R': stats(s, PULSE - 1.0, PULSE, 2)}
        report['phases'].append(r)
    # command silence (BMduino 500 ms watchdog): keep both spinning, then stop sending
    if not ONLY or 'watchdog' in ONLY:
        r, s = phase('spin_before_silence', SPEED, SPEED, 1.0)
        report['phases'].append(r)
        r, s = phase('command_silence', 0, 0, 2.0, tx=False, guard=False)
        r['stop_s_after_last_cmd'] = stop_time(LAST_VSER)
        report['phases'].append(r)


def run_handspin():
    """MOTOR SUPPLY MUST BE OFF. Holds a small forward command on both channels so the firmware
    computes encoder rpm (at target 0 it forces reported rpm to 0), while you turn wheels BY HAND
    in the forward direction. Guards are off on purpose: no motor power -> no rpm unless you turn it."""
    r, s = phase('handspin_cmd', SPEED, SPEED, PULSE, guard=False)
    per_sec = []
    for k in range(int(PULSE)):
        L = [abs(a) for t, a, b in s if k <= t < k + 1]
        R = [abs(b) for t, a, b in s if k <= t < k + 1]
        per_sec.append({'sec': k, 'maxL': max(L, default=None), 'maxR': max(R, default=None)})
    r['per_second_max_abs_rpm'] = per_sec
    r['signed_extremes'] = {'L': [min((a for t, a, b in s), default=None), max((a for t, a, b in s), default=None)],
                            'R': [min((b for t, a, b in s), default=None), max((b for t, a, b in s), default=None)]}
    report['phases'].append(r)


try:
    if mode == 'handspin':
        pre, _ = phase('uart_alive_zero', 0, 0, 1.0)
        report['phases'].append(pre)
        if pre['acked'] < 2 or pre['n_sta'] < 2:
            raise RuntimeError('no acks/STA at zero speed; refusing')
        run_handspin()
    elif mode == 'precheck':
        run_precheck()
    elif mode == 'motion':
        pre, _ = phase('uart_alive_zero', 0, 0, 1.0)
        report['phases'].append(pre)
        if pre['acked'] < 2 or pre['n_sta'] < 2:
            raise RuntimeError('no acks/STA at zero speed; refusing to move wheels')
        run_motion()
    else:
        raise RuntimeError('unknown mode')
except (Exception, KeyboardInterrupt) as exc:
    report['error'] = str(exc) or 'interrupted'
finally:
    closing = True
    if mode in ('motion', 'handspin'):
        try:
            r, s = phase('final_stop', 0, 0, 2.0, guard=False)
            report['phases'].append(r)
            report['final_zero_confirmed'] = bool(latest and last_sta_t and time.monotonic() - last_sta_t < 0.5 and
                                                  abs(float(latest['rpmL'])) <= 1 and abs(float(latest['rpmR'])) <= 1)
        except Exception as exc:  # noqa
            report['stop_error'] = str(exc)
    port.close()
    trace.close()
    (root / 'report.json').write_text(json.dumps(report, indent=1))
    print(json.dumps(report, indent=1))
    print('\nrun dir:', root)
