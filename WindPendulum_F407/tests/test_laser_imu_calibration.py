import importlib.util
import pathlib
import sys
import unittest

import numpy as np


MODULE_PATH = pathlib.Path(__file__).resolve().parents[1] / 'Tools/laser_imu_calibration.py'
sys.path.insert(0, str(MODULE_PATH.parent))
SPEC = importlib.util.spec_from_file_location('laser_imu_calibration', MODULE_PATH)
calibration = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(calibration)


class LaserCalibrationTests(unittest.TestCase):
    def test_capture_flag_counts_only_active_calibration_rows(self):
        rows = [
            {'mode': 9, 'q1_state': 0, 'time_ms': 0},
            {'mode': 9, 'q1_state': 1, 'q1_switches': 0, 'time_ms': 20},
            {'mode': 9, 'q1_state': 0, 'q1_switches': 1, 'time_ms': 40},
            {'mode': 9, 'q1_state': 1, 'q1_switches': 1, 'time_ms': 60},
            {'mode': 9, 'q1_state': 0, 'q1_switches': 2, 'time_ms': 80},
            {'mode': 9, 'q1_state': 1, 'q1_switches': 2, 'time_ms': 100},
            {'mode': 9, 'q1_state': 0, 'q1_switches': 3, 'time_ms': 120},
            {'mode': 2, 'q1_state': 1, 'q1_switches': 0, 'time_ms': 140},
        ]
        report = calibration.summarize_capture(rows)
        self.assertEqual(report['valid_capture_rows'], 3)
        self.assertEqual(report['outside_capture_rows'], 5)
        self.assertEqual(report['completed_points'], 3)
        self.assertTrue(report['all_three_points_complete'])

    def test_laser_ground_geometry(self):
        gravity = calibration.body_gravity_from_spot(0.0, 0.0, 99.0)
        np.testing.assert_allclose(gravity, [0, 0, 1], atol=1e-12)
        right = calibration.body_gravity_from_spot(20.0, 0.0, 99.0)
        left = calibration.body_gravity_from_spot(-20.0, 0.0, 99.0)
        up = calibration.body_gravity_from_spot(0.0, 20.0, 99.0)
        down = calibration.body_gravity_from_spot(0.0, -20.0, 99.0)
        self.assertGreater(right[0], 0)
        self.assertLess(left[0], 0)
        self.assertGreater(up[1], 0)
        self.assertLess(down[1], 0)
        self.assertAlmostEqual(np.degrees(np.arcsin(right[0])), 11.42, delta=0.02)

    def test_fit_recovers_rotation_used_by_firmware(self):
        old = np.array([
            [0.64941527, -0.75831454, 0.05673503],
            [-0.76034698, -0.64865960, 0.03336445],
            [0.01150097, -0.06480569, -0.99783162]])
        points = [(0, 0), (20, 0), (-20, 0), (0, 20), (0, -20)]
        body = np.array([calibration.body_gravity_from_spot(x, y, 99) for x, y in points])
        sensor = body @ old
        fitted, errors, singular = calibration.fit_rotation(sensor, body)
        np.testing.assert_allclose(fitted, old, atol=1e-10)
        self.assertGreater(np.linalg.det(fitted), 0.999999)
        self.assertLess(float(np.max(errors)), 0.01)
        self.assertGreater(float(singular[-1]), 0)

    def test_rejects_unobservable_or_bad_data(self):
        with self.assertRaises(ValueError):
            calibration.fit_rotation([[0, 0, 1]] * 5, [[0, 0, 1]] * 5)
        with self.assertRaises(ValueError):
            calibration.body_gravity_from_spot(0, 0, 0)

    def test_center_zero_and_single_laser_point_check(self):
        rotation=np.array([[.64941527,-.75831454,.05673503],
                           [-.76034698,-.64865960,.03336445],
                           [.01150097,-.06480569,-.99783162]])
        center=calibration.body_gravity_from_spot(0,0,99)@rotation
        right=calibration.body_gravity_from_spot(20,0,99)@rotation
        result=calibration.zero_and_check(center,right,20,0,99,rotation)
        self.assertLess(abs(result['roll_zero_deg']),1e-4)
        self.assertLess(abs(result['pitch_zero_deg']),1e-4)
        self.assertTrue(result['within_manual_point_tolerance'])
        self.assertAlmostEqual(result['reference_corrected_pitch_deg'],
                               result['reference_laser_pitch_deg'],delta=1e-6)

    def test_manual_hold_tolerates_small_hand_motion_and_rejects_swing(self):
        rng=np.random.default_rng(7)
        accel=np.tile([200.0,-300.0,-2000.0],(200,1))+rng.normal(0,2,(200,3))
        gyro=np.tile([5.0,-2.0,7.0],(200,1))+rng.normal(0,8,(200,3))
        self.assertTrue(calibration.evaluate_pose_stability(accel,gyro)['stable_pose'])
        swinging=accel.copy()
        swinging[:,0]+=np.linspace(-200,200,200)
        self.assertFalse(calibration.evaluate_pose_stability(swinging,gyro)['stable_pose'])


if __name__ == '__main__':
    unittest.main()
