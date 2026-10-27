"""Actual MuJoCo load, forward kinematics, rendering and finite dynamics; no hardware dependencies."""
from pathlib import Path
import json,time
import numpy as np
import mujoco
from PIL import Image
P=Path(__file__).resolve().parents[1];path=P/'simulation/mjcf/openarm_omnihand.xml';model=mujoco.MjModel.from_xml_path(str(path));data=mujoco.MjData(model);poses=json.loads((P/'ros2_description/config/poses.json').read_text());refs=json.loads((P/'reports/structural_validation.json').read_text())['FK'];names=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,i) for i in range(model.njnt)];result={'version':mujoco.__version__,'nq':model.nq,'nu':model.nu,'neq':model.neq,'joint_names':names,'source':'SYNTHETIC_MODEL_POSE','kinematics':[],'physics_pass':False}
model.vis.global_.offwidth=960;model.vis.global_.offheight=720;renderer=mujoco.Renderer(model,720,960);camera=mujoco.MjvCamera();camera.lookat[:]=[.12,-.22,.22];camera.distance=.65;camera.azimuth=210;camera.elevation=-12
for label,q in poses.items():
 for n,v in q.items():data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,n)]]=v
 mujoco.mj_forward(model,data);errors={}
 for name,T in refs[label].items():
  i=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,name);actual=np.eye(4);actual[:3,:3]=data.site_xmat[i].reshape(3,3);actual[:3,3]=data.site_xpos[i];errors[name]=float(np.max(abs(actual-np.array(T))))
 assert max(errors.values())<1e-8;contacts=[{'a':mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,c.geom1),'b':mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,c.geom2),'distance':float(c.dist)} for c in data.contact];result['kinematics'].append({'pose':label,'max_error':max(errors.values()),'contacts':contacts});renderer.update_scene(data,camera);Image.fromarray(renderer.render()).save(P/'screenshots'/('mujoco_'+label+'.png'))
# Do not integrate a configuration with unexpected contacts. This is an adapter limit, not license to remove colliders.
mujoco.mj_resetDataKeyframe(model,data,0);mujoco.mj_forward(model,data);samples=[]
if data.ncon:result['physics_limitation']='Convex mesh collision envelope reports contacts at initial pose; integration intentionally not attempted.'
else:
 start=data.qpos.copy();qgoal=poses['open_hand'];tnames=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_ACTUATOR,i) for i in range(model.nu)]
 for k in range(4000):
  a=min(1.,k/3500)
  for i,n in enumerate(tnames):j=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,n);data.ctrl[i]=(1-a)*start[model.jnt_qposadr[j]]+a*qgoal[n]
  mujoco.mj_step(model,data);samples.append({'time':float(data.time),'q':data.qpos.tolist(),'v':data.qvel.tolist(),'contacts':data.ncon})
  if not np.isfinite(data.qpos).all() or data.ncon:result['physics_limitation']='Nonfinite state or contact encountered; stopped';break
 result['physics_pass']=len(samples)==4000 and 'physics_limitation' not in result
(P/'reports/mujoco_result.json').write_text(json.dumps(result,indent=2));(P/'reports/mujoco_steps.json').write_text(json.dumps(samples));renderer.close();print(json.dumps(result))
