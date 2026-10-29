import unittest

from real_robot.startup_scope import classify_start_pose, START_ABS_LIMITS_RAD


class InitialPoseScope(unittest.TestCase):
    def test_current_start_not_matched_to_historical_snapshots(self):
        for j2 in (-.09,.01049,.01659,.032616,.055,.09):
            q = [.007,j2,.01545,.05894,.12417,.00706,-.0288]
            self.assertEqual(classify_start_pose(q),'measured_start_region_v1')
        self.assertEqual(classify_start_pose([0.]*7),'measured_start_region_v1')

    def test_each_axis_has_positive_and_negative_start_boundaries(self):
        for axis,limit in enumerate(START_ABS_LIMITS_RAD):
            for sign in (-1,1):
                q = [0.]*7
                q[axis] = sign*limit
                classify_start_pose(q)
                q[axis] = sign*(limit+.00001)
                with self.assertRaisesRegex(ValueError,f'J{axis+1}'):
                    classify_start_pose(q)

    def test_malformed_and_nonfinite_rejected(self):
        for q in ([0.]*6, [0.]*8, [float('nan')]*7, [float('inf')]*7):
            with self.assertRaises(ValueError):
                classify_start_pose(q)
