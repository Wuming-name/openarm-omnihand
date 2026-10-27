#!/usr/bin/env python3
"""Visualization JointState source, never a controller. Linear reversible pose transitions."""
import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
class Player(Node):
 def __init__(self):
  super().__init__('model_pose_player');p=Path(get_package_share_directory('openarm_omnihand_description'))
  self.poses=json.loads((p/'config/poses.json').read_text());self.q=self.poses['closed_hand'].copy();self.start=self.q.copy();self.target=self.q.copy();self.t=time.monotonic();self.publisher=self.create_publisher(JointState,'joint_states',10);self.create_subscription(String,'model_pose',self.select,10);self.create_timer(.033,self.tick)
 def select(self,msg):
  if msg.data not in self.poses:self.get_logger().error('Unknown model pose');return
  self.start=self.q.copy();self.target=self.poses[msg.data];self.t=time.monotonic();self.get_logger().info('SYNTHETIC_MODEL_POSE '+msg.data)
 def tick(self):
  a=min(1.,(time.monotonic()-self.t)/8.);self.q={n:(1-a)*v+a*self.target[n] for n,v in self.start.items()};m=JointState();m.header.stamp=self.get_clock().now().to_msg();m.name=list(self.q);m.position=list(self.q.values());self.publisher.publish(m)
def main():
 rclpy.init();node=Player()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:
  node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
