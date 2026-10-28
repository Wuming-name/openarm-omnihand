import unittest

from real_robot.plan_startup import EscapeContactGuard


class EscapeGuard(unittest.TestCase):
    def item(self, depth=.018, part='mcp'):
        return {'a': 'collision_openarm_body_link0_0',
                'b': f'collision_omni_R_thumb_{part}_0', 'distance_m': -depth}

    def test_reduction_then_clearance_allowed(self):
        guard = EscapeContactGuard([self.item()])
        guard.check([self.item(.012)])
        guard.check([self.item(.002)])
        guard.check([])

    def test_reentry_after_clearing_rejected(self):
        guard = EscapeContactGuard([self.item()])
        guard.check([])
        with self.assertRaises(RuntimeError):
            guard.check([self.item(.002)])

    def test_increase_from_best_depth_rejected_even_below_initial(self):
        guard = EscapeContactGuard([self.item()])
        guard.check([self.item(.010)])
        with self.assertRaises(RuntimeError):
            guard.check([self.item(.011)])

    def test_new_contact_rejected(self):
        guard = EscapeContactGuard([self.item()])
        with self.assertRaises(RuntimeError):
            guard.check([self.item(.002, 'pip')])
