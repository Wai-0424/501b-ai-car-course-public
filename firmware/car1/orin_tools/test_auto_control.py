import math
import unittest
from auto_control import automatic_fwa, validate_auto_gains


class AutoControlTests(unittest.TestCase):
    def metrics(self, near=0.02, far=0.02, near_valid=1, far_valid=1, pose=0, heading=0):
        return [pose, 0, heading, 0, near, far, near_valid, far_valid, 0]

    def test_stale_or_missing_perception_refuses_motion(self):
        for metrics, age in ((None, 0), (self.metrics(), 0.31), ([], 0)):
            with self.assertRaises(ValueError):
                automatic_fwa(metrics, age, 150, 20)

    def test_invalid_near_refuses_motion_even_with_pose(self):
        for m in (self.metrics(near_valid=0, pose=1), self.metrics(near=float('nan'))):
            with self.assertRaises(ValueError):
                automatic_fwa(m, 0, 150, 20)

    def test_right_offset_steers_right(self):
        self.assertAlmostEqual(automatic_fwa(self.metrics(), 0, 150, 20), 3)

    def test_missing_far_uses_near_without_stale_heading(self):
        self.assertAlmostEqual(automatic_fwa(self.metrics(far=1, far_valid=0), 0, 150, 20), 3)

    def test_chord_and_pose_sign_are_preserved(self):
        self.assertGreater(automatic_fwa(self.metrics(far=0.1), 0, 150, 20), 3)
        self.assertAlmostEqual(automatic_fwa(self.metrics(pose=1, heading=0.1), 0, 150, 20), 1)

    def test_maximum_steering_is_twenty_degrees(self):
        self.assertEqual(automatic_fwa(self.metrics(near=1, far=1), 0, 150, 20), 20)
        self.assertEqual(automatic_fwa(self.metrics(near=-1, far=-1), 0, 150, 20), -20)

    def test_bad_gain_settings_are_rejected(self):
        for a, b in ((201, 20), (150, 31), (-1, 20), (True, 20), (math.nan, 20)):
            with self.assertRaises(ValueError):
                validate_auto_gains(a, b)


if __name__ == '__main__':
    unittest.main()
