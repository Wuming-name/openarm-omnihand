"""Generate portable rigid-body metadata from delivered URDF and verified mesh caches."""
from pathlib import Path
import json,hashlib
import numpy as np
from scipy.spatial.transform import Rotation as R
from model_math import Robot,origin,points,vec,aggregate
P=Path(__file__).resolve().parents[1];A=P/'simulation/usd';robot=Robot(P/'ros2_description/urdf/openarm_omnihand.urdf');rest=robot.fk()[0];parents={j.find('child').get('link'):j.find('parent').get('link') for j in robot.joints};active=[j for j in robot.joints if j.get('type')!='fixed'];reps=['openarm_right_link0']+[j.find('child').get('link') for j in active]
def owner(n):return n if n in reps else owner(parents[n]) if n in parents else reps[0]
owners={n:owner(n) for n in robot.links};m={'source_sha256':hashlib.sha256((P/'ros2_description/urdf/openarm_omnihand.urdf').read_bytes()).hexdigest(),'joint_names':robot.joint_names,'root_body':reps[0],'bodies':{},'frames':{},'joints':[],'geometry':[]}
for rep in reps:
 bodies=[];members=[]
 for n,(mass,c,I) in robot.bodies.items():
  if owners[n]!=rep:continue
  T=np.linalg.inv(rest[rep])@rest[n];m['frames'][n]={'body':rep,'local':T.tolist()}
  if mass:bodies.append((mass,points(T,c),T[:3,:3]@I@T[:3,:3].T));members.append(n)
 mass,c,I=aggregate(bodies);eig,V=np.linalg.eigh(I)
 if np.linalg.det(V)<0:V[:,0]*=-1
 assert min(eig)>0
 m['bodies'][rep]={'mass':mass,'com':c.tolist(),'inertia':I.tolist(),'diagonal':eig.tolist(),'principal_xyzw':R.from_matrix(V).as_quat().tolist(),'rest_world':rest[rep].tolist(),'members':members}
for j in active:m['joints'].append({'name':j.get('name'),'parent':owners[j.find('parent').get('link')],'child':j.find('child').get('link'),'origin':(np.linalg.inv(rest[owners[j.find('parent').get('link')]])@rest[j.find('parent').get('link')]@origin(j)).tolist(),'axis':vec(j.find('axis').get('xyz')).tolist(),'limit':{k:float(v) for k,v in j.find('limit').attrib.items()},'mimic':j.find('mimic').attrib if j.find('mimic') is not None else None})
for n,link in robot.links.items():
 for kind in ['visual','collision']:
  for idx,el in enumerate(link.findall(kind)):
   g=el.find('geometry')[0];row={'link':n,'body':owners[n],'kind':kind,'index':idx,'type':g.tag,'local':(np.array(m['frames'][n]['local'])@origin(el)).tolist(),'geometry':dict(g.attrib)}
   if g.tag=='mesh':
    src=P/'ros2_description'/g.get('filename').split('/',3)[3];digest=hashlib.sha256(src.read_bytes()).hexdigest();key=f'{digest[:16]}_{hashlib.sha256(g.get("scale","1 1 1").encode()).hexdigest()[:8]}';target=A/(key+'.npz');assert target.is_file(),target;row.update(mesh=target.name,source_sha256=digest)
   m['geometry'].append(row)
import xml.etree.ElementTree as E
m['reviewed_pairs']=[{'links':[x.get('link1'),x.get('link2')]} for x in E.parse(P/'ros2_description/config/accepted_arm.srdf').findall('disable_collisions')]
(A/'model.json').write_text(json.dumps(m,indent=2));print('bodies',len(reps),'joints',len(active))
