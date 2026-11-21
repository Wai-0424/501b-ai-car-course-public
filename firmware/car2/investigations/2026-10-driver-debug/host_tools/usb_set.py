"""Send ONE command to the diagnostic firmware and LEAVE (no automatic stop).

usage: py -3 usb_set.py <command like Hl50 | HL100 | x>
The firmware keeps its state until 'x' (usb_set.py x) or its own time cap.
"""
import sys, time
import serial
from serial.tools import list_ports

cmd = sys.argv[1]
port = next((p.device for p in list_ports.comports() if p.vid == 0x04D9 and p.pid == 0x802F), None)
if port is None:
    sys.exit('no Holtek 04D9:802F COM port found')
ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = port, 115200, 0.1
ser.dtr = False
ser.rts = False
ser.open()
time.sleep(0.6)
ser.reset_input_buffer()
ser.write((cmd + '\n').encode())
t0 = time.time()
while time.time() - t0 < 2.5:
    line = ser.readline().decode('ascii', errors='replace').strip()
    if line:
        print('  ', line)
ser.close()
print('left running (send "x" with usb_set.py to stop)' if cmd.lower() != 'x' else 'stop sent')
