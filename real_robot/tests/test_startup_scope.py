import unittest

from real_robot.startup_scope import classify_start_pose, SCREENED_20261029


class InitialPoseScope(unittest.TestCase):
    def test_old_near_zero_and_new_measured_pose(self):
        self.assertEqual(classify_start_pose([0.]*7), 'near_zero_0.1rad')
        self.assertEqual(classify_start_pose(SCREENED_20261029),
                         'screened_pose_20261029_153744')

    def test_new_pose_does_not_expand_arbitrary_joint_limits(self):
        for axis in range(7):
            q = list(SCREENED_20261029)
            q[axis] += .0021
            with self.assertRaises(ValueError):
                classify_start_pose(q)
        q = [0.]*7
        q[4] = SCREENED_20261029[4]
        with self.assertRaises(ValueError):
            classify_start_pose(q)

    def test_malformed_and_nonfinite_rejected(self):
        for q in ([0.]*6, [0.]*8, [float('nan')]*7, [float('inf')]*7):
            with self.assertRaises(ValueError):
                classify_start_pose(q)
