"""ROS 2 seven-axis servo. Default mock mode; CAN torque output requires --backend socketcan --execute."""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data

from .settings import DEFAULT, NAMES, load
from .control import RunBundle
from .servo import Servo


class Bridge(Node):
    def __init__(self, args):
        super().__init__("openarm_rl_servo")
        live = args.backend == "socketcan"
        if live and not args.execute:
            raise ValueError("CAN torque output requires explicit --execute; use real_robot.probe for read-only checks")
        self.hardware = load(args.config, live=live)
        self.bundle = RunBundle(args.run, load_policy=False)
        self.env = self.bundle.env
        self.servo = Servo(self.hardware, self.env.physics, self.bundle.hard_low, self.bundle.hard_high)
        self.backend, self.bus = args.backend, None
        self.phase, self.reason = "WAIT_COMMAND", ""
        self.last_refresh, self.last_publish, self.last_status = 0., 0., 0.
        self.can_send = True
        self.enable_motors = args.enable_motors
        self.enabled_sent = False
        self.enable_started = None
        self.last_command_stamp = -1
        if live:
            from .can_bus import MotorBus
            self.bus = MotorBus(self.hardware, allow_control=True)
            self.bus.inspect()
            self.bus.validate_parameters(live=True)
        else:
            import mujoco
            self.env.data.qpos[:] = self.bundle.q_a
            self.env.data.qpos[0] += .03
            self.env.data.qpos[1] -= .02
            mujoco.mj_forward(self.env.model, self.env.data)
        self.state_pub = self.create_publisher(JointState, "/openarm_rl/joint_states", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, "/openarm_rl/bridge_status", 1)
        self.create_subscription(JointState, "/openarm_rl/joint_command", self.command, 1)
        self.create_service(Trigger, "/openarm_rl/stop", self.stop_request)
        self.timer = self.create_timer(1/self.hardware["servo_hz"], self.tick)
        self.get_logger().info(f"Backend={self.backend}; waiting for fresh matching-run commands")

    def measured(self):
        if self.bus is None:
            return self.env.data.qpos.tolist(), self.env.data.qvel.tolist(), self.env.data.ctrl.tolist()
        now = time.monotonic()
        states = [self.bus.states.get(m["tx"]) for m in self.hardware["motors"]]
        if any(s is None or now-s["received_monotonic"] > self.hardware["feedback_timeout_s"] for s in states):
            raise RuntimeError("Motor feedback missing or stale; torque transmission stopped")
        for state in states:
            if state["status"] not in (0, 1):
                raise RuntimeError(f"Motor fault/status {state['status']}; torque transmission stopped")
            if max(state["mos_c"], state["rotor_c"]) >= self.hardware["max_temperature_c"]:
                raise RuntimeError("Application temperature threshold exceeded")
        return [s["q"] for s in states], [s["velocity"] for s in states], [s["torque"] for s in states]

    def command(self, msg):
        if self.phase.startswith("FAULT"):
            return
        try:
            if msg.name != NAMES or self.count_publishers("/openarm_rl/joint_command") != 1:
                raise ValueError("Wrong joint names or multiple command publishers")
            stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds-stamp)*1e-9
            if not -.01 <= age <= self.hardware["command_timeout_s"] or stamp <= self.last_command_stamp:
                raise ValueError("Old or out-of-order command")
            q, velocity, _ = self.measured()
            if self.phase == "WAIT_COMMAND":
                if len(msg.position) != 7 or max(abs(v) for v in velocity) > .05 or max(abs(a-b) for a, b in zip(msg.position, q)) > .015:
                    raise ValueError("First command must hold the stationary measured pose")
            self.servo.accept(list(msg.position), list(msg.velocity), list(msg.effort), q, time.monotonic())
            self.last_command_stamp = stamp
            self.phase = "ACTIVE"
        except (ValueError, RuntimeError) as exc:
            self.latch(str(exc), fresh=True)

    def latch(self, reason, fresh):
        if not self.can_send:
            self.phase = "FAULT_NO_CONTROL"
            return
        if not self.phase.startswith("FAULT"):
            self.get_logger().error(reason)
            self.reason = reason
        if self.servo.target is None:
            self.phase, self.can_send = "FAULT_NO_CONTROL", False
            return
        if fresh:
            try:
                q, _, _ = self.measured()
                self.servo.stop(reason, q)
                self.phase = "FAULT_HOLD"
                return
            except RuntimeError:
                pass
        self.phase, self.can_send = "FAULT_NO_CONTROL", False

    def stop_request(self, request, response):
        self.latch("operator_requested_stop", fresh=True)
        response.success, response.message = True, self.phase
        return response

    def tick(self):
        now = time.monotonic()
        try:
            if self.bus:
                self.bus.drain()
                if self.phase == "WAIT_COMMAND" and now-self.last_refresh > .02:
                    self.bus.refresh()
                    self.last_refresh = now
            try:
                q, velocity, torque = self.measured()
            except RuntimeError:
                if self.phase == "WAIT_COMMAND":
                    return
                raise
            if self.phase in ("ACTIVE", "FAULT_HOLD") and self.can_send:
                if self.bus:
                    disabled = [s for s in self.bus.states.values() if s["status"] == 0]
                    if disabled:
                        if self.enable_motors and not self.enabled_sent:
                            self.bus.torques([0.]*7)
                            self.bus.enable()
                            self.enabled_sent = True
                            self.enable_started = now
                            return
                        if self.enable_started is not None and now-self.enable_started < .1:
                            return
                        raise RuntimeError("Motor disabled; no automatic fault clear or re-enable")
                torque = self.servo.tick(q, velocity, now)
                if self.servo.fault:
                    self.phase, self.reason = "FAULT_HOLD", self.servo.fault
                if torque is not None:
                    if self.bus:
                        self.bus.torques(torque)
                    else:
                        import mujoco
                        self.env.data.ctrl[:] = torque
                        mujoco.mj_step(self.env.model, self.env.data)
                        mujoco.mj_forward(self.env.model, self.env.data)
                        q, velocity, torque = self.measured()
            if now-self.last_publish >= 1/self.hardware["state_hz"]:
                message = JointState()
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = self.bundle.env.config["base_site"]
                message.name, message.position, message.velocity, message.effort = NAMES, q, velocity, torque
                self.state_pub.publish(message)
                self.last_publish = now
        except (RuntimeError, ValueError, OSError) as exc:
            self.latch(str(exc), fresh=False)
        if now-self.last_status > .1:
            message = String()
            message.data = json.dumps({"backend": self.backend, "phase": self.phase, "reason": self.reason,
                "model_sha256": self.env.task["model_sha256"], "config_sha256": self.env.task["config_sha256"]})
            self.status_pub.publish(message)
            self.last_status = now

    def close(self):
        # No automatic disable/zeroing on exit. The reviewed motor watchdog is the
        # last resort; it cannot mechanically support a backdrivable arm.
        self.timer.cancel()
        if self.bus:
            self.bus.close()
        self.bundle.env.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    parser.add_argument("--config", default=str(DEFAULT))
    parser.add_argument("--backend", choices=["mock", "socketcan"], default="mock")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--enable-motors", action="store_true", help="Permit one initial enable, never re-enable a faulted motor")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = None
    try:
        node = Bridge(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
