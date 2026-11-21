"""USB receive tests using fake bytes/time only. Never opens a serial port."""
import sys
import types
import unittest
from unittest.mock import patch

try:
    import serial
except ImportError:
    serial = types.ModuleType('serial')
    sys.modules['serial'] = serial

from car1_bm import Car1, frame


class FakePort:
    def __init__(self):
        self.data = bytearray()
        self.writes = []
        self.read_calls = 0

    @property
    def in_waiting(self):
        return len(self.data)

    def read(self, count):
        assert count <= len(self.data), 'read must not wait for future bytes'
        self.read_calls += 1
        chunk = bytes(self.data[:count])
        del self.data[:count]
        return chunk

    def write(self, data):
        self.writes.append(data)


class FakeClock:
    def __init__(self):
        self.now = 0.

    def __call__(self):
        self.now += .001
        return self.now


class PollTests(unittest.TestCase):
    def setUp(self):
        self.car = Car1.__new__(Car1)
        self.car.ser = FakePort()
        self.car.bad, self.car.dbg, self.car.seq = 0, None, 0
        self.car._rx = bytearray()
        self.sample = frame('STA,ackseq 7,f 0,rpmL 72.7,rpmR 72.7,brk 19,bmax 128')

    def test_fragment_survives_between_polls(self):
        self.car.ser.data.extend(self.sample[:23])
        self.assertEqual(self.car.poll(), [])
        self.car.ser.data.extend(self.sample[23:])
        result = self.car.poll()
        self.assertEqual(result[0]['brk'], '19')
        self.assertEqual(self.car.bad, 0)

    def test_multiple_sta_and_debug_lines(self):
        self.car.ser.data.extend(self.sample + b'dbg,1,2,3\n' + self.sample)
        self.assertEqual(len(self.car.poll()), 2)
        self.assertEqual(self.car.dbg, ['1', '2', '3'])

    def test_flood_returns_to_command_sender(self):
        self.car.ser.data.extend(self.sample * 10000)
        clock = FakeClock()
        with patch('car1_bm.time.monotonic', clock):
            self.car.poll(budget=.02)
        self.assertLessEqual(clock.now, .022)
        self.assertTrue(self.car.ser.in_waiting > 0)
        self.car.cmd(15, 15)
        self.assertEqual(len(self.car.ser.writes), 1)

    def test_empty_port_never_enters_blocking_read(self):
        self.assertEqual(self.car.poll(), [])
        self.assertEqual(self.car.ser.read_calls, 0)

    def test_corrupt_checksum_is_counted_not_exception(self):
        self.car.ser.data.extend(b'<STA,ackseq 1,cs ZZ>\n' + self.sample)
        self.assertEqual(len(self.car.poll()), 1)
        self.assertEqual(self.car.bad, 1)

    def test_missing_newline_buffer_is_bounded(self):
        self.car.ser.data.extend(b'x' * 10000)
        self.car.poll()
        self.assertLessEqual(len(self.car._rx), 4096)
        self.assertGreater(self.car.bad, 0)


if __name__ == '__main__':
    unittest.main()
