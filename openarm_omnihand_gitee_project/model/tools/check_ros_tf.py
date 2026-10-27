import rclpy,time,json,sys
from pathlib import Path
from sensor_msgs.msg import JointState
from tf2_ros import Buffer,TransformListener
import numpy as np
from scipy.spatial.transform import Rotation
from model_math import Robot
P=Path(__file__).resolve().parents[1];rclpy.init();n=rclpy.create_node('portable_tf_check');b=Buffer();listener=TransformListener(b,n);msgs=[];sub=n.create_subscription(JointState,'joint_states',lambda m:msgs.append(m),10);robot=Robot(P/'ros2_description/urdf/openarm_omnihand.urdf');end=time.monotonic()+8
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
assert msgs;m=msgs[-1];q=dict(zip(m.name,m.position));fk=robot.fk(q)[0];errors={}
for name,T in fk.items():
 if name=='world':continue
 t=b.lookup_transform('world',name,rclpy.time.Time()).transform;v=t.rotation;actual=np.eye(4);actual[:3,:3]=Rotation.from_quat([v.x,v.y,v.z,v.w]).as_matrix();actual[:3,3]=[t.translation.x,t.translation.y,t.translation.z];errors[name]=float(np.max(np.abs(actual-T)))
assert max(errors.values())<1e-8
out={'joint_states_source':[(x.node_name,x.node_namespace) for x in n.get_publishers_info_by_topic('/joint_states')],'q':q,'stamp':{'sec':m.header.stamp.sec,'nanosec':m.header.stamp.nanosec},'TF_max_element_error':max(errors.values()),'TF_count':len(errors),'errors':errors};(P/'reports'/sys.argv[1]).write_text(json.dumps(out,indent=2));print({k:v for k,v in out.items() if k not in ['q','errors']});n.destroy_node();rclpy.shutdown()
