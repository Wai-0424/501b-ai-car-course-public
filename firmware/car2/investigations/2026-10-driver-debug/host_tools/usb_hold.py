"""Hold a fixed duty on ONE wheel (diagnostic firmware) so you can probe it with a meter/scope.

usage: py -3 usb_hold.py <command like HL50 | HR30 | Hl20 | Hr20> <seconds>
Sends 'x' (stop) when the time is up, on errors and on Ctrl-C.
"""
import sys, time
import serial
from serial.tools import list_ports

cmd, secs = sys.argv[1], float(sys.argv[2])
port = next((p.device for p in list_ports.comports() if p.vid == 0x04D9 and p.pid == 0x802F), None)
if port is None:
    sys.exit('no Holtek 04D9:802F COM port found')
ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = port, 115200, 0.1
ser.dtr = False
ser.rts = False
ser.open()
try:
    time.sleep(0.6)
    ser.reset_input_buffer()
    ser.write((cmd + '\n').encode())
    print('sent:', cmd, f'(holding for {secs:.0f} s; x is sent automatically at the end)')
    t0 = time.time()
    while time.time() - t0 < secs:
        line = ser.readline().decode('ascii', errors='replace').strip()
        if line:
            print(f'  [{time.time() - t0:5.1f}s] {line}')
            if line.startswith(('err', 'abort')):
                break
finally:
    ser.write(b'x\n')
    time.sleep(0.3)
    tail = ser.read(200).decode('ascii', errors='replace').strip()
    print('stopped:', tail.splitlines()[-1] if tail else '(no reply)')
    ser.close()
