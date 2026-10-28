"""37-observation / 7-action Gymnasium environment, with torque-limited MIT-form servo."""
from __future__ import annotations

import copy
from pathlib import Path
import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np

from .common import config_hash, load_config, read_json, sha256
from .model import ids, tcp_pose, tcp_velocity, contacts, within_workspace, rotation_vector


class PalmReachEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(self, task_path, randomize=None, reset_noise=None, render_mode=None):
        super().__init__()
        self.task_path = Path(task_path).resolve()
        self.task = read_json(self.task_path)
        self.config = load_config(self.task_path.with_name("config.json"))
        if config_hash(self.config) != self.task["config_sha256"]:
            raise ValueError("Task configuration fingerprint mismatch; regenerate the task")
        model_path = self.task_path.parent / self.task["model_file"]
        if sha256(model_path) != self.task["model_sha256"] or mujoco.__version__ != self.task["mujoco_version"]:
            raise ValueError("Task model/version mismatch; regenerate using the pinned MuJoCo version")
        self.model = mujoco.MjModel.from_binary_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.nominal = copy.copy(self.model)
        self.nominal_data = mujoco.MjData(self.nominal)
        self.indices = ids(self.model, self.config)
        self.physics, self.settings = self.config["physics"], self.config["task"]
        self.dt = self.physics["control_dt"]
        self.substeps = round(self.dt / self.model.opt.timestep)
        self.vmax = np.asarray(self.physics["vmax"])
        self.amax = np.asarray(self.physics["amax"])
        self.limits = np.asarray(self.physics["torque_limits"])
        self.base_kp = np.asarray(self.physics["kp"])
        self.base_kd = np.asarray(self.physics["kd"])
        ranges = self.model.jnt_range[self.indices["joints"]]
        self.low = ranges[:, 0] + self.physics["joint_margin_rad"]
        self.high = ranges[:, 1] - self.physics["joint_margin_rad"]
        self.randomize = self.config["randomization"]["enabled"] if randomize is None else randomize
        self.reset_noise = self.settings["reset_noise_rad"] if reset_noise is None else reset_noise
        self.curriculum = 1.
        self.viewer = None
        self.render_mode = render_mode
        self.action_space = spaces.Box(-1., 1., (7,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (37,), dtype=np.float32)
        self.ended = True

    def set_curriculum(self, fraction):
        if not np.isfinite(fraction) or not 0 < fraction <= 1:
            raise ValueError("Curriculum fraction must be in (0,1]")
        self.curriculum = float(fraction)  # applied only on the next reset

    def _randomize(self):
        r = self.config["randomization"]
        self.model.body_mass[:] = self.nominal.body_mass
        self.model.body_inertia[:] = self.nominal.body_inertia
        self.model.dof_damping[:] = self.nominal.dof_damping
        self.kp, self.kd = self.base_kp.copy(), self.base_kd.copy()
        if self.randomize:
            scale = self.np_random.uniform(1-r["mass_fraction"], 1+r["mass_fraction"], self.model.nbody)
            self.model.body_mass[:] *= scale
            self.model.body_inertia[:] *= scale[:, None]
            self.model.dof_damping[:] *= self.np_random.uniform(1-r["damping_fraction"], 1+r["damping_fraction"], 7)
            self.kp *= self.np_random.uniform(1-r["gain_fraction"], 1+r["gain_fraction"], 7)
            self.kd *= self.np_random.uniform(1-r["gain_fraction"], 1+r["gain_fraction"], 7)
        mujoco.mj_setConst(self.model, self.data)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._randomize()
        mujoco.mj_resetData(self.model, self.data)
        goal_q = (1-self.curriculum)*np.asarray(self.task["q_A"]) + self.curriculum*np.asarray(self.task["q_B"])
        self.data.qpos[:] = goal_q
        mujoco.mj_forward(self.model, self.data)
        self.goal, self.goal_rotation = tcp_pose(self.data, self.indices)
        for _ in range(100):
            self.data.qpos[:] = np.clip(np.asarray(self.task["q_A"]) + self.np_random.uniform(-self.reset_noise, self.reset_noise, 7), self.low, self.high)
            self.data.qvel[:] = 0
            mujoco.mj_forward(self.model, self.data)
            self.position, self.rotation = tcp_pose(self.data, self.indices)
            if not contacts(self.model, self.data, self.physics["contact_tolerance_m"]) and within_workspace(self.position, self.config):
                break
        else:
            raise RuntimeError("Unable to sample collision-free start near A")
        self.command = self.data.qpos.copy()
        self.velocity_command = np.zeros(7)
        self.last_action = np.zeros(7)
        self.elapsed = 0
        self.success_steps = 0
        self.last_distance = float(np.linalg.norm(self.goal-self.position))
        self.ended = False
        marker = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "goal_marker")
        rb = self.data.site_xmat[self.indices["base"]].reshape(3, 3)
        self.model.site_pos[marker] = self.data.site_xpos[self.indices["base"]] + rb @ self.goal
        mujoco.mj_forward(self.model, self.data)
        return self._observation(), {"goal_base_m": self.goal.tolist(), "curriculum": self.curriculum}

    def _observation(self):
        q, qd = self.data.qpos.copy(), self.data.qvel.copy()
        if self.randomize:
            r = self.config["randomization"]
            q += self.np_random.normal(0., r["position_noise_std_rad"], 7)
            qd += self.np_random.normal(0., r["velocity_noise_std_rad_s"], 7)
        # FK uses the same noisy encoder sample as q, not privileged true TCP.
        self.nominal_data.qpos[:] = q
        self.nominal_data.qvel[:] = qd
        mujoco.mj_forward(self.nominal, self.nominal_data)
        p, rot = tcp_pose(self.nominal_data, self.indices)
        observation = np.concatenate([(2*q-self.low-self.high)/(self.high-self.low), qd/self.vmax,
            (self.command-q)/self.physics["max_target_lead_rad"], p/self.settings["position_scale_m"],
            (self.goal-p)/self.settings["goal_scale_m"], rotation_vector(self.goal_rotation @ rot.T)/np.pi,
            self.last_action])
        if not np.isfinite(observation).all():
            raise FloatingPointError("Nonfinite observation")
        return observation.astype(np.float32)

    def step(self, action):
        if self.ended:
            raise RuntimeError("Episode ended; call reset before step")
        action = np.asarray(action, dtype=float)
        if action.shape != (7,) or not np.isfinite(action).all():
            raise ValueError("Action must contain seven finite values")
        action = np.clip(action, -1., 1.)
        previous_action = self.last_action.copy()
        desired_velocity = action*self.vmax
        self.velocity_command += np.clip(desired_velocity-self.velocity_command, -self.amax*self.dt, self.amax*self.dt)
        self.command = np.clip(self.command + self.velocity_command*self.dt,
                               self.data.qpos-self.physics["max_target_lead_rad"],
                               self.data.qpos+self.physics["max_target_lead_rad"])
        self.command = np.clip(self.command, self.low, self.high)
        self.last_action = self.velocity_command/self.vmax
        # Feedforward uses the NOMINAL model, not the randomized true mass.
        self.nominal_data.qpos[:] = self.data.qpos
        self.nominal_data.qvel[:] = self.data.qvel
        mujoco.mj_forward(self.nominal, self.nominal_data)
        feedforward = self.physics["gravity_compensation"]*self.nominal_data.qfrc_bias.copy()
        failure, collision = None, []
        effort = 0.
        for _ in range(self.substeps):
            torque = self.kp*(self.command-self.data.qpos) + self.kd*(self.velocity_command-self.data.qvel) + feedforward
            self.data.ctrl[:] = np.clip(torque, -self.limits, self.limits)
            effort += float(np.mean((self.data.ctrl/self.limits)**2))/self.substeps
            before_time = self.data.time
            mujoco.mj_step(self.model, self.data)
            mujoco.mj_forward(self.model, self.data)
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                raise FloatingPointError("MuJoCo produced nonfinite state; training stopped")
            if self.data.time < before_time + .5*self.model.opt.timestep:
                raise FloatingPointError("MuJoCo reset an unstable simulation; training stopped")
            if np.any(np.abs(self.data.qvel) > 3.):
                failure = "joint_overspeed"
                break
            if np.any(self.data.qpos < self.low-0.03) or np.any(self.data.qpos > self.high+0.03):
                failure = "joint_limit"
                break
            collision = contacts(self.model, self.data, self.physics["contact_tolerance_m"])
            if collision:
                failure = "self_collision"
                break
        self.elapsed += 1
        self.position, self.rotation = tcp_pose(self.data, self.indices)
        distance = float(np.linalg.norm(self.goal-self.position))
        angle = float(np.linalg.norm(rotation_vector(self.goal_rotation @ self.rotation.T)))
        speed = float(np.linalg.norm(tcp_velocity(self.model, self.data, self.indices)))
        if not within_workspace(self.position, self.config):
            failure = failure or "workspace"
        if np.max(np.abs(self.command-self.data.qpos)) > self.physics["max_target_lead_rad"] + .03:
            failure = failure or "tracking_error"
        good = distance < self.settings["success_distance_m"] and speed < self.settings["success_speed_m_s"] and angle < self.settings["success_rotation_rad"]
        self.success_steps = self.success_steps + 1 if good and not failure else 0
        success = self.success_steps >= round(self.settings["success_hold_seconds"]/self.dt)
        w = self.config["reward"]
        reward = w["progress"]*(self.last_distance-distance) - self.dt*(w["distance"]*distance + w["orientation"]*angle**2
            + w["velocity"]*float(np.mean((self.data.qvel/self.vmax)**2))
            + w["smoothness"]*float(np.mean((self.last_action-previous_action)**2)) + w["effort"]*effort + w["time"])
        reward += w["success_bonus"] if success else (-w["failure_penalty"] if failure else 0.)
        self.last_distance = distance
        terminated = bool(success or failure)
        truncated = bool(not terminated and self.elapsed >= round(self.settings["episode_seconds"]/self.dt))
        self.ended = terminated or truncated
        info = {"is_success": bool(success), "failure": failure, "distance_m": distance, "orientation_error_rad": angle,
                "tcp_speed_m_s": speed, "tcp_base_m": self.position.tolist(), "contacts": collision,
                "simulation_seconds": float(self.data.time)}
        if self.render_mode == "human":
            self.render()
        return self._observation(), float(reward), terminated, truncated, info

    def render(self):
        if self.viewer is None:
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        if self.viewer.is_running():
            self.viewer.sync()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
