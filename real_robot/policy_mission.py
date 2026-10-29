"""Startup -> measured-palm A acceptance -> PPO -> continuous B hold. No CAN."""
import numpy as np

from .commission_hold import StationaryOverlapGuard
from .plan_startup import model_contacts
from rl_ppo.model import contacts, rotation_vector


def a_metrics(bundle,q,velocity):
    bundle.update_state(bundle.q_a,np.zeros(7))
    position,rotation = bundle.env.position.copy(),bundle.env.rotation.copy()
    bundle.update_state(q,velocity)
    return {'a_distance_m':float(np.linalg.norm(bundle.env.position-position)),
            'a_orientation_error_rad':float(np.linalg.norm(rotation_vector(rotation @ bundle.env.rotation.T)))}


def a_accepted(metrics,velocity):
    return (metrics['a_distance_m'] <= .015 and metrics['a_orientation_error_rad'] <= .10
            and np.max(np.abs(velocity)) <= .05)


class PolicyMission:
    def __init__(self,bundle,start,segments,enable_policy=False):
        self.bundle,self.start,self.segments = bundle,np.asarray(start),segments
        self.enable_policy = enable_policy
        self.guard = StationaryOverlapGuard(model_contacts(bundle,self.start))
        self.duration = 1.+sum(s['duration_s'] for s in segments)
        self.begin = None
        self.phase = 'HOLD_INITIAL'
        self.a_stable = self.b_stable = None
        self.policy_started = None
        self.a_record = None
        self.hold_b = None
        self.b_lost_since = None

    def tick(self,q,velocity,now):
        from .move_to_a import reference_at
        b = self.bundle
        elapsed = 0. if self.begin is None else now-self.begin
        metrics = a_metrics(b,q,velocity)
        if self.phase not in ('POLICY','HOLD_B'):
            command,v,phase = reference_at(self.start,self.segments,elapsed)
        else:
            phase = self.phase
        found = contacts(b.env.model,b.env.data,b.env.physics['contact_tolerance_m'])
        if np.any(np.asarray(q) < b.hard_low) or np.any(np.asarray(q) > b.hard_high):
            raise RuntimeError('Measured joint outside hard limits')
        if phase in ('HOLD_INITIAL','shoulder_retreat','wrist_retreat'):
            self.guard.check(found)
            if np.max(np.abs(np.asarray(q)-self.start)) > .125:
                raise RuntimeError('Measured escape leaves startup neighborhood')
        elif found:
            raise RuntimeError('Model collision after startup escape: '+str(found))
        if phase == 'HOLD_A' and self.enable_policy:
            if a_accepted(metrics,velocity):
                self.a_stable = now if self.a_stable is None else self.a_stable
            else:
                self.a_stable = None
            if self.a_stable is not None and now-self.a_stable >= .5:
                b.check_state()
                # Keep last A command and preload zero velocity/action history.
                # State observations remain the measured q/dq, not ideal q_A.
                b.start_policy(command=command)
                self.a_record = {'q_rad':list(q),'velocity_rad_s':list(velocity),**metrics}
                self.policy_started = now
                phase = 'POLICY'
            elif elapsed > self.duration+15.:
                raise RuntimeError('Palm did not settle at A within 15 seconds')
        elif phase == 'POLICY':
            b.check_state()
            command,v,_ = b.action_command()
            if b.goal_reached():
                self.b_stable = now if self.b_stable is None else self.b_stable
            else:
                self.b_stable = None
            if self.b_stable is not None and now-self.b_stable >= b.env.settings['success_hold_seconds']:
                self.hold_b = command.copy()
                phase = 'HOLD_B'
                v = np.zeros(7)
            elif now-self.policy_started > b.env.settings['episode_seconds']:
                raise RuntimeError('PPO did not reach B within trained episode duration')
        elif phase == 'HOLD_B':
            b.check_state()
            command,v = self.hold_b.copy(),np.zeros(7)
            if b.goal_reached():
                self.b_lost_since = None
            else:
                self.b_lost_since = now if self.b_lost_since is None else self.b_lost_since
                if now-self.b_lost_since > .5:
                    raise RuntimeError('B hold left endpoint acceptance region for 0.5 seconds')
        self.phase = phase
        return {'command':command.tolist(),'velocity_command':v.tolist(),
                'ff':b.feedforward().tolist(),'phase':phase,
                'tcp_base_m':b.env.position.tolist(),**metrics,
                'b_distance_m':float(np.linalg.norm(b.env.position-b.env.goal)),
                'b_goal_reached':bool(b.goal_reached()),'accepted_A':self.a_record,
                'elapsed_s':elapsed,'computed':now}
