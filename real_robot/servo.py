"""Low-level PD and watchdog math independent of ROS and CAN."""
import math


class Servo:
    def __init__(self, hardware, physics, low, high):
        self.hardware, self.physics = hardware, physics
        self.low, self.high = list(low), list(high)
        self.target = None
        self.received = None
        self.fault = None
        self.last_tick = None
        self.ff = [0.]*7
        self.over_since = [None]*7

    def accept(self, q, velocity, ff, q_actual, now):
        for values in (q, velocity, ff):
            if len(values) != 7 or not all(math.isfinite(float(x)) for x in values):
                raise ValueError("Invalid command array")
        for i in range(7):
            if not self.low[i] <= q[i] <= self.high[i]:
                raise ValueError("Command outside trained soft limits")
            if abs(velocity[i]) > self.physics["vmax"][i]+1e-5:
                raise ValueError("Velocity reference exceeds trained limit")
            if abs(q[i]-q_actual[i]) > self.physics["max_target_lead_rad"]+.03:
                raise ValueError("Command too far from measured position")
        if self.fault:
            return
        self.target = (list(q), list(velocity))
        self.ff = list(ff)
        self.received = now

    def stop(self, reason, q):
        if not self.fault:
            self.fault = reason
            self.target = (list(q), [0.]*7)

    def tick(self, q, velocity, now):
        if self.target is None:
            return None
        if self.last_tick is not None and now-self.last_tick > self.hardware["max_timer_gap_s"]:
            self.stop("servo_deadline", q)
        self.last_tick = now
        if self.received is None or now-self.received > self.hardware["command_timeout_s"]:
            self.stop("command_timeout", q)
        if any(not math.isfinite(x) for x in list(q)+list(velocity)):
            raise RuntimeError("Nonfinite measured state")
        if max(abs(x) for x in velocity) > self.hardware["max_joint_speed_rad_s"]:
            raise RuntimeError("Measured overspeed; control stopped")
        output = []
        for i, motor in enumerate(self.hardware["motors"]):
            if not self.low[i]-.03 <= q[i] <= self.high[i]+.03:
                raise RuntimeError("Measured joint outside deployment limits")
            tau = (self.physics["kp"][i]*(self.target[0][i]-q[i])
                   + self.physics["kd"][i]*(self.target[1][i]-velocity[i]) + self.ff[i])
            limit = min(self.physics["torque_limits"][i], motor["mechanical_peak_nm"])
            tau = max(-limit, min(limit, tau))
            if abs(tau) > motor["continuous_nm"]:
                if self.over_since[i] is None:
                    self.over_since[i] = now
                if now-self.over_since[i] > self.hardware["over_continuous_torque_seconds"]:
                    raise RuntimeError(f"Joint {i+1} exceeded continuous torque rating too long")
            else:
                self.over_since[i] = None
            output.append(tau)
        return output
