"""Synthetic articulated model demo; Physics targets and mimic constraints, never ROS/CAN."""
from pathlib import Path
import argparse,json,time
import numpy as np
from scipy.spatial.transform import Rotation as R
P=Path(__file__).resolve().parents[1];pa=argparse.ArgumentParser();pa.add_argument('--gui',action='store_true');args=pa.parse_args()
from isaacsim import SimulationApp
app=SimulationApp({'headless':not args.gui,'width':1280,'height':900,'extra_args':['--/app/settings/persistent=false','--/app/window/title=Portable Articulated OmniHand - SYNTHETIC']})
import omni.usd,omni.physx
from pxr import UsdGeom,UsdPhysics,UsdLux,Gf,PhysicsSchemaTools
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager as SM
m=json.loads((P/'simulation/usd/model.json').read_text());poses=json.loads((P/'ros2_description/config/poses.json').read_text());mapping=json.loads((P/'simulation/usd/usd_mapping_fresh.json').read_text());ctx=omni.usd.get_context();assert ctx.open_stage(str(P/'simulation/usd/openarm_omnihand.usda'));app.update();s=ctx.get_stage();s.SetEditTarget(s.GetSessionLayer());UsdPhysics.Scene(s.GetPrimAtPath('/PhysicsScene')).CreateGravityMagnitudeAttr(0.);art=Articulation('/Robot');result={'source':'SYNTHETIC_MODEL_POSE','gui':args.gui,'gravity':0.,'physics_dt':.005,'physics_pass':False};rows=[]
try:
 SM.setup_simulation(dt=.005,device='cpu');SM.initialize_physics();names=art.dof_names;assert set(names)==set(m['joint_names']);result['dof_names']=names;result['links']=art.link_names;view=SM.get_physics_simulation_view();cv=view.create_rigid_contact_view('/Robot/bodies/*',filter_patterns=list(mapping['body_paths'].values()),max_contact_data_count=4096)
 def contacts():
  f,p,n,d,c,i=cv.get_contact_data(.005);return int(c.numpy().sum())
 # Independent engine FK for all three poses before dynamics; set positions is explicitly initialization/kinematic check.
 ref=json.loads((P/'reports/structural_validation.json').read_text())['FK'];fk=[]
 for pose,q in poses.items():
  vals=[q[n] for n in names];art.set_dof_positions([vals]);art.set_dof_velocities([[0.]*len(names)]);view.update_articulations_kinematic();raw=art._physics_articulation_view.get_link_transforms().numpy()[0];bodies={}
  for idx,n in enumerate(art.link_names):
   T=np.eye(4);T[:3,:3]=R.from_quat(raw[idx,3:]).as_matrix();T[:3,3]=raw[idx,:3];bodies[n]=T
  err={n:float(np.max(np.abs(bodies[f['body']]@np.array(f['local'])-np.array(ref[pose][n])))) for n,f in m['frames'].items()};fk.append({'pose':pose,'max_error':max(err.values()),'errors':err})
 result['FK']=fk;assert max(x['max_error'] for x in fk)<1e-5
 # Fixed-base and zero gravity demonstration, no parameter claims for hardware.
 q0=np.array([poses['closed_hand'][n] for n in names]);art.set_dof_positions([q0]);art.set_dof_velocities([[0.]*len(names)]);view.update_articulations_kinematic()
 independent=[i for i,n in enumerate(names) if not next(j for j in m['joints'] if j['name']==n)['mimic']]
 kp=np.array([4000,4000,2000,2000,800,800,800]+[2.]*16);kd=np.array([160,160,80,80,30,30,30]+[.08]*16);byname=dict(zip(m['joint_names'],zip(kp,kd)));art.set_dof_gains([[byname[n][0] if i in independent else 0. for i,n in enumerate(names)]],[[byname[n][1] if i in independent else 0. for i,n in enumerate(names)]])
 from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
 cam=UsdGeom.Camera.Define(s,'/ReviewCamera');cam.CreateFocalLengthAttr(25.);cam.CreateClippingRangeAttr(Gf.Vec2f(.01,100.));mat=Gf.Matrix4d(1);mat.SetLookAt(Gf.Vec3d(-.40,-.65,.42),Gf.Vec3d(.12,-.22,.20),Gf.Vec3d(0,0,1));UsdGeom.Xformable(cam).AddTransformOp().Set(mat.GetInverse());vp=get_active_viewport();vp.camera_path='/ReviewCamera';UsdLux.DomeLight.Define(s,'/Light').CreateIntensityAttr(1000.)
 def capture(label):
  if not args.gui:return
  for _ in range(15):app.update()
  capture_viewport_to_file(vp,str(P/'screenshots'/('isaac_'+label+'.png')))
  for _ in range(15):app.update()
 capture('closed_hand')
 previous=q0.copy();st=time.monotonic();maxerr=0.;maxmimic=0.
 for label in ['intermediate_hand','open_hand','closed_hand']:
  goal=np.array([poses[label][n] for n in names])
  for k in range(1600):
   a=min(1.,k/1400);target=(1-a)*previous+a*goal;art.set_dof_position_targets([target]);SM.step();actual=art.get_dof_positions().numpy()[0];vel=art.get_dof_velocities().numpy()[0];count=contacts();error=float(max(abs(actual-target)));maxerr=max(maxerr,error);mimic=max(abs(actual[names.index(j['name'])]-float(j['mimic']['multiplier'])*actual[names.index(j['mimic']['joint'])]) for j in m['joints'] if j['mimic']);maxmimic=max(maxmimic,float(mimic));rows.append({'pose':label,'step':k,'time':len(rows)*.005,'q':actual.tolist(),'v':vel.tolist(),'target':target.tolist(),'max_error':error,'mimic_error':float(mimic),'contacts':count})
   if not np.isfinite(actual).all() or not np.isfinite(vel).all() or error>.1 or count:raise RuntimeError(f'Physics guard: {label}/{k} error={error} contacts={count}')
   if args.gui and k%20==0:app.update()
  capture(label);previous=goal
 result.update(physics_pass=True,steps=len(rows),wall_seconds=time.monotonic()-st,max_tracking_error_rad=maxerr,max_mimic_error_rad=maxmimic)
except Exception as e:
 result['failure']=repr(e);print('ISAAC_FAILED',repr(e),flush=True)
finally:
 (P/'reports/isaac_result.json').write_text(json.dumps(result,indent=2));(P/'reports/isaac_steps.json').write_text(json.dumps(rows));print('ISAAC_RESULT',json.dumps({k:v for k,v in result.items() if k!='FK'}),flush=True)
 try:SM.invalidate_physics()
 finally:app.close()
