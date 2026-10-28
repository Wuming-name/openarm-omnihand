"""ROS 2: hold initial arm pose -> fist -> A -> PPO -> B hold. Default hand is mock."""
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
from .control import RunBundle, Mission
from .hand_worker import HandWorker


class Supervisor(Node):
    def __init__(self, args):
        super().__init__("openarm_rl_mission")
        self.hardware = load(args.config, live=args.hand == "canfd")
        if args.hand == "canfd" and not args.execute:
            raise ValueError("Real hand motion requires --hand canfd --execute")
        self.bundle = RunBundle(args.run)
        self.mission = Mission(self.bundle, self.hardware)
        self.hand = HandWorker(self.hardware, real=args.hand == "canfd")
        self.state, self.bridge = None, None
        self.state_time, self.bridge_time = 0., 0.
        self.failed = False
        self.previous_phase = ""
        self.publisher = self.create_publisher(JointState, "/openarm_rl/joint_command", 1)
        self.status_pub = self.create_publisher(String, "/openarm_rl/mission_status", 1)
        self.create_subscription(JointState, "/openarm_rl/joint_states", self.receive, qos_profile_sensor_data)
        self.create_subscription(String, "/openarm_rl/bridge_status", self.receive_bridge, 1)
        self.stop_client = self.create_client(Trigger, "/openarm_rl/stop")
        self.timer = self.create_timer(self.bundle.env.dt, self.tick)

    def receive(self, msg):
        if msg.name != NAMES:
            return
        age = (self.get_clock().now().nanoseconds - (msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec))*1e-9
        if not -.01 <= age <= self.hardware["feedback_timeout_s"]:
            return
        self.state, self.state_time = msg, time.monotonic()

    def receive_bridge(self, msg):
        self.bridge = json.loads(msg.data)
        self.bridge_time = time.monotonic()

    def request_stop(self):
        if self.stop_client.service_is_ready():
            self.stop_client.call_async(Trigger.Request())

    def tick(self):
        if self.failed or self.state is None or self.bridge is None:
            return
        now = time.monotonic()
        try:
            if now-self.state_time > self.hardware["feedback_timeout_s"] or now-self.bridge_time > .5:
                raise RuntimeError("ROS feedback/status timeout")
            if self.bridge["phase"].startswith("FAULT"):
                raise RuntimeError("Bridge latched fault: " + self.bridge.get("reason", ""))
            if (self.bridge["model_sha256"] != self.bundle.env.task["model_sha256"]
                    or self.bridge["config_sha256"] != self.bundle.env.task["config_sha256"]):
                raise RuntimeError("Bridge and policy use different trained-run configuration")
            if (self.bridge["backend"] == "socketcan") != self.hand.real:
                raise RuntimeError("Mock and live arm/hand backends cannot be mixed")
            ready, healthy = self.hand.status()
            q, velocity, ff = self.mission.tick(self.state.position, self.state.velocity, ready, healthy, now)
            if self.mission.phase == "FISTING":
                if self.bridge["phase"] != "ACTIVE":
                    raise RuntimeError("Arm servo is not holding before hand closure")
                self.hand.start()
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.bundle.env.config["base_site"]
            message.name = NAMES
            message.position, message.velocity, message.effort = q.tolist(), velocity.tolist(), ff.tolist()
            self.publisher.publish(message)
            if self.previous_phase != self.mission.phase:
                self.get_logger().info("STATE: " + self.mission.phase)
                self.previous_phase = self.mission.phase
            status = String()
            status.data = json.dumps({"phase": self.mission.phase, "hand_ready": ready, "hand_error": self.hand.error,
                                      "tcp_base_m": self.bundle.env.position.tolist()})
            self.status_pub.publish(status)
        except (ValueError, RuntimeError, OSError) as exc:
            self.failed = True
            reason = str(exc) + ("; hand: " + self.hand.error if self.hand.error else "")
            self.get_logger().error(reason)
            status = String()
            status.data = json.dumps({"phase": "FAULT", "reason": reason})
            self.status_pub.publish(status)
            self.request_stop()

    def close(self):
        self.timer.cancel()
        self.request_stop()
        self.hand.close()
        self.bundle.env.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    parser.add_argument("--config", default=str(DEFAULT))
    parser.add_argument("--hand", choices=["mock", "canfd"], default="mock")
    parser.add_argument("--execute", action="store_true")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = None
    try:
        node = Supervisor(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            rclpy.spin_once(node, timeout_sec=.1)
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
