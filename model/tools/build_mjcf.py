"""MJCF adapter: same rigid tree, source inertia, explicit mimic equalities. Mesh contacts use MuJoCo convex hulls."""
from pathlib import Path
import json,xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation as R
P=Path(__file__).resolve().parents[1];A=P/'simulation/mjcf';(A/'meshes').mkdir(exist_ok=True);m=json.loads((P/'simulation/usd/model.json').read_text());poses=json.loads((P/'ros2_description/config/poses.json').read_text());root=E.Element('mujoco',model='openarm_omnihand');E.SubElement(root,'compiler',angle='radian',autolimits='true',meshdir='meshes',inertiafromgeom='false');E.SubElement(root,'option',timestep='0.002',gravity='0 0 0',integrator='implicitfast');asset=E.SubElement(root,'asset');world=E.SubElement(root,'worldbody');E.SubElement(world,'light',pos='-1 -1 2',dir='1 1 -1',diffuse='.9 .9 .9');eq=E.SubElement(root,'equality');act=E.SubElement(root,'actuator');contact=E.SubElement(root,'contact')
def fmt(v):return ' '.join(format(float(x),'.12g') for x in np.asarray(v).flat)
def pose(T):
 T=np.array(T);q=R.from_matrix(T[:3,:3]).as_quat();return {'pos':fmt(T[:3,3]),'quat':fmt([q[3],*q[:3]])}
bychild={j['child']:j for j in m['joints']};nodes={};order=[]
for n,b in m['bodies'].items():
 j=bychild.get(n);parent=world if j is None else nodes[j['parent']];T=b['rest_world'] if j is None else j['origin'];el=E.SubElement(parent,'body',name=n,**pose(T));nodes[n]=el
 # MuJoCo diagonal inertia expressed in principal frame, copied from accepted URDF aggregation.
 q=b['principal_xyzw'];E.SubElement(el,'inertial',mass=str(b['mass']),pos=fmt(b['com']),quat=fmt([q[3],*q[:3]]),diaginertia=fmt(b['diagonal']))
 if j:
  order.append(j['name']);E.SubElement(el,'joint',name=j['name'],type='hinge',axis=fmt(j['axis']),range=fmt([j['limit']['lower'],j['limit']['upper']]),limited='true')
  if j['mimic']:E.SubElement(eq,'joint',joint1=j['name'],joint2=j['mimic']['joint'],polycoef=fmt([0,float(j['mimic']['multiplier']),0,0,0]))
  else:
   i=m['joint_names'].index(j['name']);kp=([4000,4000,2000,2000,800,800,800]+[2.]*16)[i];kd=([160,160,80,80,30,30,30]+[.08]*16)[i];E.SubElement(act,'position',name=j['name'],joint=j['name'],kp=str(kp),kv=str(kd),forcerange=fmt([-j['limit']['effort'],j['limit']['effort']]),ctrlrange=fmt([j['limit']['lower'],j['limit']['upper']]))
for n,f in m['frames'].items():E.SubElement(nodes[f['body']],'site',name=n,**pose(f['local']),size='.001',rgba='0 0 0 0')
seen=set()
for g in m['geometry']:
 visual=g['kind']=='visual';attrs={'name':g['kind']+'_'+g['link']+'_'+str(g['index']),**pose(g['local']),'group':'2' if visual else '3','contype':'0' if visual else '1','conaffinity':'0' if visual else '1','rgba':'.65 .68 .72 1' if visual else '.7 .2 .2 0'};typ=g['type'];geom=g['geometry']
 if typ=='mesh':
  key=g['mesh'][:-4]
  if key not in seen:
   a=np.load(P/'simulation/usd'/g['mesh']);v=a['vertices'];f=a['faces'];(A/'meshes'/(key+'.obj')).write_text(''.join('v '+fmt(x)+'\n' for x in v)+''.join('f '+' '.join(str(int(x)+1) for x in t)+'\n' for t in f));E.SubElement(asset,'mesh',name=key,file=key+'.obj');seen.add(key)
  attrs.update(type='mesh',mesh=key)
 elif typ=='box':attrs.update(type='box',size=fmt(np.fromstring(geom['size'],sep=' ')/2))
 elif typ=='cylinder':attrs.update(type='cylinder',size=fmt([float(geom['radius']),float(geom['length'])/2]))
 elif typ=='sphere':attrs.update(type='sphere',size=geom['radius'])
 E.SubElement(nodes[g['body']],'geom',**attrs)
# Existing SRDF pairs only; aggregation turns fixed connected parts into same body.
seen=set()
for x in m['reviewed_pairs']:
 a,b=[m['frames'][n]['body'] for n in x['links']];key=tuple(sorted([a,b]))
 if a!=b and key not in seen:E.SubElement(contact,'exclude',body1=a,body2=b);seen.add(key)
keys=E.SubElement(root,'keyframe');ind=[a.get('joint') for a in act]
for name,q in poses.items():E.SubElement(keys,'key',name=name,qpos=fmt([q[n] for n in order]),ctrl=fmt([q[n] for n in ind]))
E.indent(root);E.ElementTree(root).write(A/'openarm_omnihand.xml',encoding='unicode',xml_declaration=True);(A/'mapping.json').write_text(json.dumps({'joint_names':order,'actuator_names':ind,'collision_policy':'MuJoCo mesh convex hull; original SRDF excludes mapped to bodies; native parent filtering; not exact SDF envelope'},indent=2))
print('MJCF generated',len(order),len(ind))
