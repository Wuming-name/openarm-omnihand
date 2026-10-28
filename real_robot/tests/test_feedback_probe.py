import unittest

from real_robot.feedback_probe import summarize
from real_robot.settings import load


class FeedbackSummary(unittest.TestCase):
    def test_nonzero_velocity_is_retained_when_positions_are_identical(self):
        motors = load()['motors']
        samples = {m['tx']: [dict(q=.1, velocity=-.11, status=0),
                              dict(q=.1, velocity=.12, status=0)] for m in motors}
        result = summarize(samples, motors)
        self.assertEqual(result[0]['position_span_rad'], 0.)
        self.assertEqual(result[0]['max_abs_velocity_rad_s'], .12)
        self.assertAlmostEqual(result[0]['velocity_code_step_rad_s'], 90/4095)
        self.assertTrue(result[0]['all_samples_disabled'])

    def test_missing_joint_cannot_produce_successful_summary(self):
        motors = load()['motors']
        samples = {m['tx']: [dict(q=0., velocity=0., status=0)] for m in motors}
        samples[7] = []
        with self.assertRaises(RuntimeError):
            summarize(samples, motors)
