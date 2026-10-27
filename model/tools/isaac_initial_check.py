from pathlib import Path
import json,numpy as np
P=Path(__file__).resolve().parents[1]
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'extra_args':['--/app/settings/persistent=false']})
import omni.usd
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager as SM
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema
try:
 ctx=omni.usd.get_context();assert ctx.open_stage(str(P/'simulation/usd/openarm_omnihand.usda'));app.update();s=ctx.get_stage();s.SetEditTarget(s.GetSessionLayer());UsdPhysics.Scene(s.GetPrimAtPath('/PhysicsScene')).CreateGravityMagnitudeAttr(0.)
 art=Articulation('/Robot');SM.setup_simulation(dt=.005,device='cpu');SM.initialize_physics();q=json.loads((P/'ros2_description/config/poses.json').read_text())['closed_hand'];a=art.get_dof_positions().numpy()[0];err=max(abs(float(a[i])-q[n]) for i,n in enumerate(art.dof_names));print('INITIAL',err,a.tolist(),flush=True);assert err<1e-5
 deps=[str(l.identifier) for l in s.GetUsedLayers() if not l.anonymous];assert len(deps)==1,deps
 mimics=[str(p.GetPath()) for p in s.Traverse() if 'PhysxMimicJointAPI:rotX' in p.GetAppliedSchemas()];assert len(mimics)==6
 (P/'reports/isaac_final_asset_check.json').write_text(json.dumps({'passed':True,'default_pose':'closed_hand','initial_position_error_rad':err,'dof_count':len(art.dof_names),'mimic_constraints':mimics,'layers':deps,'position_reset_calls':0},indent=2));print('FINAL_ASSET_INITIAL_PASS',err,flush=True)
except Exception as e:
 import traceback;traceback.print_exc();(P/'reports/isaac_final_asset_check.json').write_text(json.dumps({'passed':False,'failure':repr(e)},indent=2))
finally:
 try:SM.invalidate_physics()
 finally:app.close()
