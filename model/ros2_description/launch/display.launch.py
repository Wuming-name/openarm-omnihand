from pathlib import Path
import json
import xml.etree.ElementTree as E
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition,UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
 p=Path(get_package_share_directory('openarm_omnihand_description'));q=json.loads((p/'config/poses.json').read_text())['closed_hand'];manifest=json.loads((p/'config/joint_manifest.json').read_text());gui=LaunchConfiguration('gui')
 gui_root=E.fromstring((p/'urdf/openarm_omnihand.urdf').read_text());bounds=json.loads((p/'config/effective_limits.json').read_text())
 for j in gui_root.findall('joint'):
  if j.get('name') in bounds and j.get('name').startswith('omni_'):
   b=bounds[j.get('name')];E.SubElement(j,'safety_controller',soft_lower_limit=str(b['lower']),soft_upper_limit=str(b['upper']),k_position='0',k_velocity='0')
 gui_parameters={'zeros.'+n:q[n] for n in manifest['independent']};gui_parameters['robot_description']=E.tostring(gui_root,encoding='unicode')
 return LaunchDescription([DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('view',default_value=str(p/'rviz/whole.rviz')),Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':(p/'urdf/openarm_omnihand.urdf').read_text()}]),Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui',parameters=[gui_parameters],condition=IfCondition(gui)),Node(package='openarm_omnihand_description',executable='pose_player.py',condition=UnlessCondition(gui)),Node(package='rviz2',executable='rviz2',arguments=['-d',LaunchConfiguration('view')])])
