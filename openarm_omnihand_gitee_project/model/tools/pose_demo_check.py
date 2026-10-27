import json,time,subprocess,os,sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
P=Path(__file__).resolve().parents[1];rclpy.init();n=Node('pose_demo_check');pub=n.create_publisher(String,'model_pose',10);msgs=[];n.create_subscription(JointState,'joint_states',lambda m:msgs.append(m),10);poses=json.loads((P/'ros2_description/config/poses.json').read_text());out=[]
for label in ['closed_hand','intermediate_hand','open_hand','closed_hand']:
 msg=String();msg.data=label
 for i in range(20):rclpy.spin_once(n,timeout_sec=.1)
 pub.publish(msg);end=time.monotonic()+9
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
 assert msgs;actual=dict(zip(msgs[-1].name,msgs[-1].position));assert set(actual)==set(poses[label]);err=max(abs(actual[j]-q) for j,q in poses[label].items());assert err<1e-8
 subprocess.run([sys.executable,str(P/'tools/activate_window.py'),' - RViz'],check=True)
 from PIL import ImageGrab
 ImageGrab.grab(xdisplay=os.environ.get('DISPLAY',':1')).save(P/'screenshots'/('rviz_'+label+'_'+sys.argv[1]+'.png'))
 out.append({'pose':label,'max_error':err,'joint_count':len(actual),'q':actual})
(P/'reports'/('rviz_pose_demo_'+sys.argv[1]+'.json')).write_text(json.dumps(out,indent=2));n.destroy_node();rclpy.shutdown()
