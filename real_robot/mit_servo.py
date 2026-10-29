"""Native MIT PD; host computes model feedforward, never an extra PD command.

Official OpenArm V1.0 default gains checked 2026-10-29:
https://github.com/enactic/openarm_description/blob/main/assets/robot/openarm_v1.0/config/arm/control_gains.yaml
Software torque estimates are NOT firmware current/torque limits.
"""
import math

from .servo import Servo

OFFICIAL_KP = (70.,70.,70.,60.,10.,10.,10.)
OFFICIAL_KD = (2.75,2.5,2.,2.,.7,.6,.5)
PROFILE = 'openarm_v1_official_native_mit'


class NativeMITServo(Servo):
    def __init__(self,hardware,physics,low,high):
        native = dict(physics)
        native['kp'],native['kd'] = list(OFFICIAL_KP),list(OFFICIAL_KD)
        super().__init__(hardware,native,low,high)
        self.last_estimated_torque = [0.]*7
        self.max_estimated_torque = [0.]*7
        self.max_feedforward = [0.]*7
        self.feedback_over_since = [None]*7

    def tick(self,q,velocity,now,feedback_torque=None):
        for values in (q,velocity):
            if len(values) != 7 or not all(math.isfinite(float(x)) for x in values):
                raise RuntimeError('Invalid measured MIT state')
        if self.target is None:
            return None
        if self.last_tick is not None and now-self.last_tick > self.hardware['max_timer_gap_s']:
            self.stop('servo_deadline',q)
        self.last_tick = now
        if self.received is None or now-self.received > self.hardware['command_timeout_s']:
            self.stop('command_timeout',q)
        if max(abs(x) for x in velocity) > self.hardware['max_joint_speed_rad_s']:
            raise RuntimeError('Measured MIT overspeed')
        if feedback_torque is not None:
            if len(feedback_torque) != 7 or not all(math.isfinite(float(x)) for x in feedback_torque):
                raise RuntimeError('Invalid measured torque')
        result = []
        for i,motor in enumerate(self.hardware['motors']):
            if not self.low[i] <= q[i] <= self.high[i]:
                raise RuntimeError(f'J{i+1}: measured hard limit exceeded')
            kp,kd = OFFICIAL_KP[i],OFFICIAL_KD[i]
            position,desired_velocity = self.target[0][i],self.target[1][i]
            pd = kp*(position-q[i])+kd*(desired_velocity-velocity[i])
            estimate = pd+self.ff[i]
            peak = min(self.physics['torque_limits'][i],motor['mechanical_peak_nm'])
            # Conservative component budget: cancellation cannot hide large PD.
            budget = abs(kp*(position-q[i]))+abs(kd*(desired_velocity-velocity[i]))+abs(self.ff[i])
            if budget > peak-.1:
                raise RuntimeError(f'J{i+1}: native MIT torque budget {budget:.3f}Nm exceeds {peak-.1:.3f}Nm')
            measured = abs(feedback_torque[i]) if feedback_torque is not None else 0.
            if measured > peak:
                raise RuntimeError(f'J{i+1}: measured torque exceeds mechanical peak')
            if max(abs(estimate),measured) > motor['continuous_nm']:
                if self.feedback_over_since[i] is None:
                    self.feedback_over_since[i] = now
                if now-self.feedback_over_since[i] > self.hardware['over_continuous_torque_seconds']:
                    raise RuntimeError(f'J{i+1}: sustained torque exceeds continuous rating')
            else:
                self.feedback_over_since[i] = None
            self.last_estimated_torque[i] = estimate
            self.max_estimated_torque[i] = max(self.max_estimated_torque[i],abs(estimate))
            self.max_feedforward[i] = max(self.max_feedforward[i],abs(self.ff[i]))
            result.append({'q':position,'velocity':desired_velocity,'kp':kp,'kd':kd,'torque':self.ff[i]})
        return result
