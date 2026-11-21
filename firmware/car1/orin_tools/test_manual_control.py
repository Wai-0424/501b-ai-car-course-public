import unittest
from manual_control import ManualControl


class ManualControlTests(unittest.TestCase):
    def setUp(self):
        self.c = ManualControl()

    def arm(self, drive=True):
        self.c.arm(0.0, ready=True, drive=drive, connected=True, fault=0, stopped=True)

    def cmd(self, t=0.0, seq=0, go=True, fwa=5.0, speed=15.0):
        self.c.command(t, fwa=fwa, speed=speed, go=go, seq=seq)

    def tick(self, t, connected=True, fault=0):
        return self.c.tick(t, connected=connected, fault=fault)

    def test_open_page_does_not_drive(self):
        self.assertEqual(self.tick(0.1), (0, 0))

    def test_requires_ready_feedback_and_stationary_wheels(self):
        for override in ({'ready': False}, {'connected': False}, {'fault': None}, {'fault': 1}, {'stopped': False}):
            args = dict(ready=True, drive=True, connected=True, fault=0, stopped=True)
            args.update(override)
            with self.assertRaises(ValueError):
                self.c.arm(0, **args)
            self.assertFalse(self.c.armed)

    def test_dry_run_steers_but_never_drives(self):
        self.arm(False)
        self.cmd()
        self.assertEqual(self.tick(0.1), (5, 0))

    def test_release_cuts_throttle_immediately(self):
        self.arm()
        self.cmd()
        self.assertGreater(self.tick(0.1)[1], 0)
        self.cmd(0.11, 1, go=False)
        self.assertEqual(self.tick(0.12)[1], 0)

    def test_stale_page_disarms_and_does_not_resume(self):
        self.arm()
        self.cmd()
        self.assertEqual(self.tick(0.36), (0, 0))
        with self.assertRaises(ValueError):
            self.cmd(0.37, 1)

    def test_fault_and_link_loss_stop(self):
        for condition in ({'fault': 1}, {'connected': False}):
            self.setUp()
            self.arm()
            self.cmd()
            self.assertEqual(self.tick(0.1, **condition), (0, 0))
            self.assertFalse(self.c.armed)

    def test_operator_session_has_no_time_cutoff(self):
        self.arm()
        for i in range(901):
            t = i * 0.1
            self.cmd(t, i, go=(i != 50))
            self.tick(t)
        self.assertTrue(self.c.armed)
        self.assertGreaterEqual(self.c.used, 89)
        self.assertGreater(self.c.output_speed, 0)

    def test_commands_are_bounded(self):
        self.arm()
        self.cmd(fwa=100, speed=150)
        self.assertEqual(self.c.fwa, 20)
        self.assertEqual(self.c.speed, 100)
        self.cmd(0.1, 1, fwa=-100, speed=-100)
        self.assertEqual(self.c.fwa, -20)
        self.assertEqual(self.c.speed, 0)

    def test_reordered_requests_do_not_restart_throttle(self):
        self.arm()
        self.cmd(seq=1)
        self.cmd(0.1, 2, go=False)
        with self.assertRaises(ValueError):
            self.cmd(0.11, 1, go=True)
        self.assertFalse(self.c.go)

    def test_nan_or_boolean_speed_disarms(self):
        for value in (float('nan'), float('inf'), True):
            self.setUp()
            self.arm()
            with self.assertRaises(ValueError):
                self.cmd(speed=value)
            self.assertFalse(self.c.armed)

    def test_stop_clears_steering_and_requires_new_arm(self):
        self.arm()
        self.cmd()
        self.c.stop()
        self.assertEqual(self.tick(0.1), (0, 0))
        self.arm()
        self.assertFalse(self.c.go)


if __name__ == '__main__':
    unittest.main()
