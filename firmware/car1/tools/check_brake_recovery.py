"""Run actual firmware PID code on the host with simulated encoders/I2C; no serial IO.

Pass a native C++ compiler with --cxx. This checks transitions and protection,
not motor torque, current or the real vehicle's braking performance.
"""
import argparse
import importlib.util
import os
from pathlib import Path
import re
import subprocess


def block(source, start):
    begin = source.index(start)
    opened = source.index('{', begin)
    depth = 1
    end = opened + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[begin:end]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cxx', required=True)
    ap.add_argument('--output', required=True)
    args = ap.parse_args()
    repo = Path(__file__).resolve().parents[3]
    ino = repo / 'firmware/car1/bmduino_car_control_l298n/bmduino_car_control_l298n.ino'
    src = ino.read_text(encoding='utf-8-sig')
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    constants = src[src.index('const uint8_t FF_LUT_N'):src.index('// ---- 型別定義')]
    state = block(src, 'struct PidState') + ';'
    functions = '\n'.join(block(src, name) for name in (
        'bool driveDirectionCached(', 'float ffDutyForTicks(',
        'void clearBrakeState(', 'void updateMotorPID('))
    harness = r'''
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <string>
using std::abs;
#define constrain(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
const uint32_t PID_INTERVAL_MS = 20;
bool pcaReady = true, failDirection = false;
uint32_t fakeMs = 1000;
uint32_t millis() { return fakeMs; }
void noInterrupts() {}
void interrupts() {}
int hardwareDirection = -1;
int hardwareDuty = 100;
bool liveBrakeToDrive = false;
void analogWrite(uint8_t, int value) { hardwareDuty = value; }
bool driveDirection(uint8_t, uint8_t, int dir) {
  if (failDirection) { pcaReady = false; return false; }
  if (hardwareDirection == 0 && dir != 0 && hardwareDuty != 0) liveBrakeToDrive = true;
  hardwareDirection = dir; return true;
}
''' + constants + state + functions + r'''
int checks = 0;
void check(bool ok, const char* message) {
  if (!ok) { fprintf(stderr, "FAIL: %s\n", message); exit(1); }
  ++checks;
}
PidState pid;
volatile int32_t pos;
uint8_t duty;
float actual;
void reset() {
  pid = {}; pos = 0; duty = 100; actual = 0;
  pid.lastDir = -1; pid.lastNonZeroDir = -1;
  pid.integral = 40;
  pcaReady = true; failDirection = false; brakeEnabled = true;
  hardwareDirection = -1;
  hardwareDuty = 100; liveBrakeToDrive = false;
}
void step(int ticks, int target = -1) {
  // Forward right-wheel encoder decreases. Set EMA to the current raw sample
  // to isolate state transitions from the independently preserved filter.
  pos -= ticks;
  pid.filteredSpeed = ticks;
  fakeMs += 20;
  updateMotorPID(target, &pos, &pid, &duty, 1, 2, &actual,
                 FF_LUT_TICKS_M1, MAX_TICKS_PER_INTERVAL_M1, 1, 8);
  hardwareDuty = duty;  // End-of-cycle updateMotorPWM().
}
void brake() { reset(); step(60); }
int main() {
  reset(); step(28);
  check(!pid.braking && duty > 0 && pid.lastDir == -1, "ordinary forward drive");
  brake();
  check(pid.braking && duty == 128 && pid.lastDir == 0, "overspeed activates capped short brake");
  check(pid.integral == 40 && pid.brakeCycles == 1, "brake freezes integral and counts successful cycles");
  step(180);
  check(pid.braking && duty == 128, "sustained extreme overspeed remains capped");
  fakeMs += 100; step(32);
  check(pid.braking && duty == 30, "hysteresis survives a delayed loop");
  step(30);
  check(!pid.braking && pid.brakeRecovering && duty == 0 && pid.lastDir == 0,
        "brake exit above target coasts instead of feedforward drive");
  check(pid.integral == 0, "old integral cleared on brake exit");
  step(29);
  check(pid.brakeRecovering && duty == 0, "recovery keeps coasting above target");
  step(28);
  float seed = ffDutyForTicks(FF_LUT_TICKS_M1, 28);
  check(!pid.brakeRecovering && duty > 8 && duty <= seed + 8 && pid.lastDir == -1,
        "resume from current-speed feedforward only once below target");
  int resumedDuty = duty;
  step(28);
  check(duty <= resumedDuty + 8, "second resume cycle also obeys slew limit");
  brake(); step(30); step(40);
  check(pid.braking && !pid.brakeRecovering && duty > 0 && pid.lastDir == 0,
        "renewed overspeed during coasting re-enters brake");
  brake(); step(60, 0);
  check(duty == 0 && !pid.braking && !pid.brakeRecovering && pid.lastDir == 0,
        "zero/watchdog stop clears brake state and coasts");
  brake(); pcaReady = false; step(60);
  check(duty == 0 && !pid.braking && !pid.brakeRecovering, "PCA offline clears active state");
  reset(); failDirection = true; step(60);
  check(duty == 0 && !pid.braking && pid.brakeCycles == 0,
        "failed brake direction write cannot report successful braking");
  brake(); pid.fault = true; step(60);
  check(duty == 0 && !pid.braking && !pid.brakeRecovering, "fault priority over brake");
  brake(); pid.noMotionMs = 580; step(0);
  check(pid.fault && duty == 0 && !pid.braking, "stall cutoff priority over brake");
  brake(); step(60, 1);
  check(duty == 0 && !pid.braking && !pid.brakeRecovering,
        "reverse waiting never applies drive or brake PWM");
  brake(); brakeEnabled = false; step(30);
  check(duty <= ffDutyForTicks(FF_LUT_TICKS_M1, MIN_TICKS_PER_INTERVAL) + 8 &&
        !pid.braking && !pid.brakeRecovering, "USB brake disable resumes from bounded feedforward");
  brake(); step(30); step(28, -100);
  check(duty <= seed + 8 && pid.lastDir == -1,
        "higher target during recovery seeds current speed instead of higher target feedforward");
  brake(); step(28);
  check(duty <= seed + 8 && !liveBrakeToDrive,
        "direct brake-to-drive exit disables physical Enable before direction switch");
  brake(); step(30);
  pos -= 31; pid.filteredSpeed = 27;
  updateMotorPID(-1, &pos, &pid, &duty, 1, 2, &actual,
                 FF_LUT_TICKS_M1, MAX_TICKS_PER_INTERVAL_M1, 1, 8);
  check(actual < MIN_TICKS_PER_INTERVAL && pid.brakeRecovering && duty == 0,
        "rising raw speed above target cannot get feedforward while EMA lags below target");
  brake(); step(0);
  check(duty <= 8 && !pid.brakeRecovering, "stopped wheel restarts from zero instead of minimum lookup duty");
  reset(); pos = 60; pid.filteredSpeed = 60;
  updateMotorPID(-1, &pos, &pid, &duty, 3, 4, &actual,
                 FF_LUT_TICKS_M2, MAX_TICKS_PER_INTERVAL_M2, -1, 9);
  check(pid.braking && duty == 128 && actual == 60 && pid.lastDir == 0,
        "left wheel encoder sign and its own feedforward table also brake correctly");
  // Run actual sendStatus/startSend/pumpSend under a 1-ms virtual loop.
  // Includes diagnostic bursts; checking buffer sizes alone misses congestion.
  fakeMs = 0; lastUsbStatusMs = 0; usbDbgDiv = 0;
  staSkipBM = staSkipUSB = 0;
  bmSender = {}; usbSender = {};
  SerialBM.bytes.clear(); Serial.bytes.clear();
  for (fakeMs = 1; fakeMs <= 2100; ++fakeMs) {
    pumpSend(bmSender, SerialBM); pumpSend(usbSender, Serial);
    if (fakeMs <= 2000 && fakeMs % 50 == 0) sendStatus();
  }
  check(staSkipBM == 0 && staSkipUSB == 0, "STA plus diagnostic bursts fit their independent transmission slots");
  check(countPrefix(SerialBM.bytes, "<STA,") == 40, "Orin UART retains 20-Hz STA");
  check(countPrefix(Serial.bytes, "<STA,") == 20, "USB independently uses 10-Hz STA");
  check(countPrefix(Serial.bytes, "dbg,") == 4, "USB diagnostics use 2 Hz without starving STA");
  printf("PASS: %d actual-PID transition/protection checks\n", checks);
}
'''
    telemetry = r'''
struct Stream {
  std::string bytes;
  void write(const uint8_t* data, uint16_t n) { bytes.append((const char*)data, n); }
};
Stream SerialBM, Serial;
const int32_t TICKS_PER_REV_M1 = 1560, TICKS_PER_REV_M2 = 1560;
const bool L298N_PID_CALIBRATED = true;
const uint8_t FAULT_CALIBRATION_REQUIRED = 8, FAULT_LEFT_STALL = 1,
              FAULT_RIGHT_STALL = 2, FAULT_PCA_OFFLINE = 4;
PidState pidM1State = {}, pidM2State = {};
float lastActualM1 = 200, lastActualM2 = 200;
int32_t posM1 = -2147483647, posM2 = -2147483647;
uint8_t m1DriverDuty = 128, m2DriverDuty = 128;
uint16_t unit = 65535, lastAcceptedSeq = 65535;
bool hasAcceptedSeq = true;
uint32_t lastUsbStatusMs = 0, staSkipBM = 0, staSkipUSB = 0;
uint8_t usbDbgDiv = 0;
int leftSpeed = 100, rightSpeed = 100;
uint32_t i2cSteerCallCount = UINT32_MAX, i2cSteerErrorCount = UINT32_MAX,
         dirI2cCallCount = UINT32_MAX, dirI2cErrorCount = UINT32_MAX,
         maxLoopGapUs = UINT32_MAX, cmdChecksumFailCount = UINT32_MAX;
int readSensorMM(uint8_t) { return -1; }
uint8_t ledEffectiveMode(uint32_t) { return 3; }
int countPrefix(const std::string& data, const char* prefix) {
  int n = 0; size_t at = 0;
  while ((at = data.find(prefix, at)) != std::string::npos) { ++n; ++at; }
  return n;
}
'''
    telemetry += '\n'.join(re.findall(
        r'^const (?:uint32_t USB_STATUS_PERIOD_MS|uint8_t USB_DBG_DIV)\s*=.*?;', src, re.M))
    telemetry += '\n' + block(src, 'struct ChunkedSender') + ';\n'
    telemetry += 'ChunkedSender bmSender = {}, usbSender = {};\n'
    telemetry += '\n'.join(block(src, name) for name in (
        'void startSend(', 'void pumpSend(', 'uint8_t xorChecksum(',
        'float ticksPerIntervalToRPM(', 'int roundToInt(', 'void sendStatus('))
    harness = harness.replace('int checks = 0;', telemetry + '\nint checks = 0;')
    cpp, exe = out / 'pid_recovery_host.cpp', out / 'pid_recovery_host.exe'
    cpp.write_text(harness, encoding='utf-8')
    env = dict(os.environ)
    env['PATH'] = str(Path(args.cxx).parent) + os.pathsep + env.get('PATH', '')
    if Path(args.cxx).name.lower() == 'cl.exe':
        vcvars = Path(args.cxx).parents[6] / 'Auxiliary/Build/vcvars64.bat'
        setup = subprocess.run(f'cmd.exe /d /s /c ""{vcvars}" >nul && set"',
                               check=True, capture_output=True, text=True, errors='replace')
        for entry in setup.stdout.splitlines():
            key, sep, value = entry.partition('=')
            if sep and key:
                env[key] = value
        command = [args.cxx, '/nologo', '/EHsc', '/utf-8', '/W3', cpp.name, f'/Fe:{exe.name}']
    else:
        command = [args.cxx, '-std=c++11', '-Wall', '-Wextra', cpp.name, '-o', exe.name]
    subprocess.run(command, check=True, env=env, cwd=out)
    subprocess.run([str(exe)], check=True, env=env)

    # Exercise the actual STA format, buffers and deployed Orin parser, including
    # full-range counters and the expanded USB debug diagnostic line.
    start = src.index('"STA,ackseq %d')
    stop = src.index(',\n', start)
    fmt = ''.join(re.findall(r'"([^"\n]*)"', src[start:stop]))
    fmt = fmt.replace('%lu', '%d').replace('%ld', '%d').replace('%u', '%d')
    vals = (65535, -1, -999999.9, -999999.9, -2147483648, -2147483648,
            15, 3, 255, 255, -2147483648, -2147483648, 4294967295,
            4294967295, 31, 255)
    payload = fmt % vals
    payload_size = int(re.search(r'char payload\[(\d+)\]', src).group(1))
    line_size = int(re.search(r'char line\[(\d+)\]', src).group(1))
    sender_size = int(re.search(r'struct ChunkedSender.*?char buf\[(\d+)\]', src, re.S).group(1))
    assert len(payload) + 1 <= payload_size
    protocol_path = repo / '書面資料/實驗報告/car1_brake_governor_20261103/candidate/car1_bringup/uart_protocol.py'
    spec = importlib.util.spec_from_file_location('car1_uart', protocol_path)
    protocol = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(protocol)
    line = f'<{payload},cs {protocol.checksum(payload):02X}>\n'
    assert len(line) + 1 <= line_size < sender_size
    assembler = protocol.StaAssembler()
    decoded = []
    for i in range(0, len(line), 5):
        decoded += assembler.feed(line[i:i+5].encode('ascii'))
    assert len(decoded) == 1 and decoded[0]['brk'] == 31 and decoded[0]['bmax'] == 255
    assert assembler.bad_packets == 0
    # Every 4th USB message appends dbg. At maximum 32-bit counters it must
    # still terminate both the STA checksum and the full diagnostic line.
    dbg = src[src.index('"dbg,%d'):].split('\n', 1)[0]
    dbg = dbg[1:dbg.rfind('"')].replace('\\n', '\n')
    dbg = dbg.replace('%lu', '%d').replace('%u', '%d')
    dbg_values = (100, 100, 255, 255, *([4294967295]*4), 1,
                  *([4294967295]*4), 5., .5, 1000., 40., 1, 1,
                  4294967295, 4294967295, 1, 40., 40., 30., 255.)
    combined = line + (dbg % dbg_values)
    assert len(combined) + 1 <= sender_size
    print(f'PASS: STA parser/checksum/5-byte assembly; worst STA={len(line)} B, USB={len(combined)} B')
    # 5 bytes/ms, excluding physical loop jitter. Does not certify real link timing.
    print(f'STA nominal chunking budget: {(len(line)+4)//5} ms / 50 ms publication period')


if __name__ == '__main__':
    main()
