"""Shared deployment math; no CAN/ROS side effects. Preserves the trained 37-D contract."""
from pathlib import Path
import numpy as np
import mujoco

from rl_ppo.common import read_json, sha256
from rl_ppo.env import PalmReachEnv
from rl_ppo.model import tcp_pose, tcp_velocity, contacts, within_workspace, rotation_vector


class RunBundle:
    def __init__(self, run, load_policy=True):
        self.run = Path(run).resolve()
        contract = read_json(self.run / "policy_contract.json")
        if sha256(self.run / "task/task.json") != contract["task_sha256"]:
            raise ValueError("Trained A/B task was modified")
        self.env = PalmReachEnv(self.run / "task/task.json", randomize=False, reset_noise=0.)
        if (contract["model_sha256"] != self.env.task["model_sha256"]
                or contract["config_sha256"] != self.env.task["config_sha256"]
                or contract["observation_size"] != 37 or contract["action_size"] != 7):
            raise ValueError("Model/observation contract mismatch")
        self.env.reset(seed=0)
        self.policy = None
        if load_policy:
            import torch
            torch.set_num_threads(1)
            from stable_baselines3 import PPO
            result = read_json(self.run / "result.json")
            if result.get("failed"):
                raise ValueError("Training run is marked failed")
            self.policy = PPO.load(self.run / result["model"], device="cpu")
            if self.policy.observation_space.shape != (37,) or self.policy.action_space.shape != (7,):
                raise ValueError("Policy dimensions do not match deployment")
        self.q_a, self.q_b = np.array(self.env.task["q_A"]), np.array(self.env.task["q_B"])
        self.hard_low = self.env.model.jnt_range[self.env.indices["joints"],0].copy()
        self.hard_high = self.env.model.jnt_range[self.env.indices["joints"],1].copy()

    def update_state(self, q, velocity):
        q, velocity = np.asarray(q, dtype=float), np.asarray(velocity, dtype=float)
        if q.shape != (7,) or velocity.shape != (7,) or not np.isfinite(q).all() or not np.isfinite(velocity).all():
            raise ValueError("Invalid seven-joint state")
        env = self.env
        env.data.qpos[:] = q
        env.data.qvel[:] = velocity
        mujoco.mj_forward(env.model, env.data)
        env.position, env.rotation = tcp_pose(env.data, env.indices)

    def check_state(self, startup=False):
        env = self.env
        low, high = (self.hard_low, self.hard_high) if startup else (env.low, env.high)
        if np.any(env.data.qpos < low-.01) or np.any(env.data.qpos > high+.01):
            raise RuntimeError("Measured joint outside startup hard limits / policy soft limits")
        if not startup and not within_workspace(env.position, env.config):
            raise RuntimeError("Measured palm outside trained workspace")
        collision = contacts(env.model, env.data, env.physics["contact_tolerance_m"])
        if collision:
            raise RuntimeError(f"Collision model rejects current state: {collision}")

    def observation(self):
        return self.env._observation()

    def start_policy(self, command=None):
        self.env.command = self.env.data.qpos.copy() if command is None else np.asarray(command).copy()
        self.env.velocity_command[:] = 0.
        self.env.last_action[:] = 0.

    def feedforward(self):
        return self.env.physics["gravity_compensation"]*self.env.data.qfrc_bias.copy()

    def action_command(self, action=None):
        env = self.env
        if action is None:
            action, _ = self.policy.predict(self.observation(), deterministic=True)
        action = np.asarray(action, dtype=float)
        if action.shape != (7,) or not np.isfinite(action).all():
            raise ValueError("Policy returned invalid action")
        env.velocity_command += np.clip(np.clip(action, -1, 1)*env.vmax-env.velocity_command,
                                         -env.amax*env.dt, env.amax*env.dt)
        env.command = np.clip(env.command+env.velocity_command*env.dt,
                              env.data.qpos-env.physics["max_target_lead_rad"],
                              env.data.qpos+env.physics["max_target_lead_rad"])
        env.command = np.clip(env.command, env.low, env.high)
        env.last_action = env.velocity_command/env.vmax
        return env.command.copy(), env.velocity_command.copy(), self.feedforward()

    def goal_reached(self):
        env = self.env
        return (np.linalg.norm(env.goal-env.position) < env.settings["success_distance_m"]
                and np.linalg.norm(tcp_velocity(env.model, env.data, env.indices)) < env.settings["success_speed_m_s"]
                and np.linalg.norm(rotation_vector(env.goal_rotation @ env.rotation.T)) < env.settings["success_rotation_rad"])

    def path_to_a(self, q_start, hardware):
        delta = self.q_a - q_start
        if np.max(np.abs(delta)) > hardware["max_step_to_A_rad"]:
            raise RuntimeError("Start is too far from A for this conservative startup path")
        for t in np.linspace(0, 1, 80):
            self.update_state(q_start+t*delta, np.zeros(7))
            self.check_state(startup=True)
        duration = max(hardware["to_A_seconds"], float(np.max(1.875*np.abs(delta)/self.env.vmax)),
                       float(np.max(np.sqrt(5.8*np.abs(delta)/self.env.amax))))
        self.update_state(q_start, np.zeros(7))
        return float(duration)


def minimum_jerk(start, target, elapsed, duration):
    u = float(np.clip(elapsed/duration, 0., 1.))
    blend = 10*u**3-15*u**4+6*u**5
    derivative = (30*u**2-60*u**3+30*u**4)/duration
    delta = np.asarray(target)-start
    return np.asarray(start)+blend*delta, derivative*delta


class Mission:
    def __init__(self, bundle, hardware):
        self.bundle, self.hardware = bundle, hardware
        self.phase = "WAIT_STATE"
        self.phase_started = None
        self.success_duration = 0.
        self.last_tick = None

    def transition(self, phase, now):
        self.phase, self.phase_started = phase, now

    def tick(self, q, velocity, hand_ready, hand_healthy, now):
        b = self.bundle
        b.update_state(q, velocity)
        b.check_state(startup=self.phase in ("WAIT_STATE", "HOLD_INITIAL", "FISTING", "MOVE_A"))
        if np.max(np.abs(velocity)) > self.hardware["max_joint_speed_rad_s"]:
            raise RuntimeError("Joint overspeed")
        if self.last_tick is not None and now-self.last_tick > .10:
            raise RuntimeError("Supervisor timer deadline missed")
        dt = b.env.dt
        self.last_tick = now
        if self.phase == "WAIT_STATE":
            if np.max(np.abs(velocity)) > .05:
                raise RuntimeError("Arm must be nearly stationary before takeover")
            self.start = np.clip(np.asarray(q), b.hard_low, b.hard_high)
            self.duration = b.path_to_a(self.start, self.hardware)
            self.transition("HOLD_INITIAL", now)
        if self.phase == "HOLD_INITIAL":
            command, speed = self.start, np.zeros(7)
            if now-self.phase_started >= self.hardware["initial_hold_seconds"]:
                self.transition("FISTING", now)
        elif self.phase == "FISTING":
            command, speed = self.start, np.zeros(7)
            if not hand_healthy:
                raise RuntimeError("Hand worker failed")
            if hand_ready:
                self.transition("MOVE_A", now)
            elif now-self.phase_started > 90:
                raise RuntimeError("Hand closure deadline exceeded")
        elif self.phase == "MOVE_A":
            if not hand_healthy or not hand_ready:
                raise RuntimeError("Hand is not holding its approved pose")
            command, speed = minimum_jerk(self.start, b.q_a, now-self.phase_started, self.duration)
            if now-self.phase_started >= self.duration:
                if np.max(np.abs(np.asarray(q)-b.q_a)) < .01 and np.max(np.abs(velocity)) < .03:
                    b.start_policy(command=command)
                    self.transition("POLICY", now)
                elif now-self.phase_started > self.duration+5:
                    raise RuntimeError("Failed to settle at A")
        elif self.phase == "POLICY":
            if not hand_ready or not hand_healthy:
                raise RuntimeError("Hand hold lost during policy control")
            command, speed, ff = b.action_command()
            self.success_duration = self.success_duration+dt if b.goal_reached() else 0.
            if self.success_duration >= b.env.settings["success_hold_seconds"]:
                self.hold_target = command.copy()
                self.transition("HOLD_B", now)
            elif now-self.phase_started > b.env.settings["episode_seconds"]:
                raise RuntimeError("Policy deadline expired before stable B arrival")
            return command, speed, ff
        elif self.phase in ("HOLD_B", "DONE"):
            if not hand_ready or not hand_healthy:
                raise RuntimeError("Hand hold lost after arrival")
            command, speed = self.hold_target, np.zeros(7)
            if self.phase == "HOLD_B" and now-self.phase_started >= self.hardware["hold_B_seconds"]:
                self.transition("DONE", now)
        else:
            raise RuntimeError("Unknown mission phase")
        if np.max(np.abs(command-np.asarray(q))) > b.env.physics["max_target_lead_rad"] + .03:
            raise RuntimeError("Joint reference tracking error exceeded")
        return np.array(command), np.array(speed), b.feedforward()
