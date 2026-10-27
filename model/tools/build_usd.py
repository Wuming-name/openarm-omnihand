from pathlib import Path
import json,numpy as np,math
from scipy.spatial.transform import Rotation as R
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':640,'height':480,'extra_args':['--/app/settings/persistent=0','--/crashreporter/skipOldDumpUpload=1']})
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf,Sdf,Vt
P=Path(__file__).resolve().parents[1];m=json.loads((P/'simulation/usd/model.json').read_text());out=P/'simulation/usd/openarm_omnihand.usda'
if out.exists():raise RuntimeError('Preserve previous USD')
closed=json.loads((P/'ros2_description/config/poses.json').read_text())['closed_hand'];closed_fk=json.loads((P/'reports/structural_validation.json').read_text())['FK']['closed_hand']
s=Usd.Stage.CreateNew(str(out));UsdGeom.SetStageUpAxis(s,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(s,1.);UsdPhysics.SetStageKilogramsPerUnit(s,1.)
root=UsdGeom.Xform.Define(s,'/Robot');s.SetDefaultPrim(root.GetPrim())
root.GetPrim().SetCustomDataByKey('model:sourceURDFSHA256',m['source_sha256']);root.GetPrim().SetCustomDataByKey('model:importPolicy','articulated SDK 0.8.0; accepted source triangles; SDF 521/6; 6 mimic constraints; default closed pose')
def quat(matrix):
 q=R.from_matrix(np.array(matrix)).as_quat();return Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3]))
def setx(prim,T):
 T=np.array(T);x=UsdGeom.Xformable(prim);x.ClearXformOpOrder();x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*T[:3,3]));q=R.from_matrix(T[:3,:3]).as_quat();x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]),Gf.Vec3d(*q[:3])))
bodypaths={n:'/Robot/bodies/'+n for n in m['bodies']};framepaths={}
for n,b in m['bodies'].items():
 body=UsdGeom.Xform.Define(s,bodypaths[n]);setx(body,closed_fk[n]);rb=UsdPhysics.RigidBodyAPI.Apply(body.GetPrim());rb.CreateRigidBodyEnabledAttr(True)
 mass=UsdPhysics.MassAPI.Apply(body.GetPrim());mass.CreateMassAttr(b['mass']);mass.CreateCenterOfMassAttr(Gf.Vec3f(*b['com']));mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*b['diagonal']));q=b['principal_xyzw'];mass.CreatePrincipalAxesAttr(Gf.Quatf(q[3],Gf.Vec3f(*q[:3])))
 PhysxSchema.PhysxContactReportAPI.Apply(body.GetPrim()).CreateThresholdAttr(0.)
for n,f in m['frames'].items():
 path=bodypaths[f['body']]+'/frames/'+n;frame=UsdGeom.Xform.Define(s,path);setx(frame,f['local']);framepaths[n]=path;frame.GetPrim().SetCustomDataByKey('model:sourceLink',n)
# Fixed root and source revolute joints (7 arm + 16 hand). Align USD X-axis to the URDF axis.
base=m['root_body'];fix=UsdPhysics.FixedJoint.Define(s,'/Robot/joints/fixed_base');fix.CreateBody1Rel().SetTargets([bodypaths[base]])
T=np.array(m['bodies'][base]['rest_world']);fix.CreateLocalPos0Attr(Gf.Vec3f(*T[:3,3]));fix.CreateLocalRot0Attr(quat(T[:3,:3]));fix.CreateLocalPos1Attr(Gf.Vec3f(0));fix.CreateLocalRot1Attr(Gf.Quatf(1))
UsdPhysics.ArticulationRootAPI.Apply(fix.GetPrim());ar=PhysxSchema.PhysxArticulationAPI.Apply(fix.GetPrim());ar.CreateEnabledSelfCollisionsAttr(True);ar.CreateSolverPositionIterationCountAttr(32);ar.CreateSolverVelocityIterationCountAttr(4);ar.CreateSleepThresholdAttr(0.);ar.CreateStabilizationThresholdAttr(0.)
kp=[4000,4000,2000,2000,800,800,800]+[2.]*16;kd=[160,160,80,80,30,30,30]+[.08]*16
for idx,j in enumerate(m['joints']):
 axis=np.array(j['axis'],float);axis/=np.linalg.norm(axis)
 A=R.from_rotvec([0,0,math.pi]).as_matrix() if axis[0]<-.99 else (np.eye(3) if axis[0]>.99 else R.align_vectors([axis],[[1,0,0]])[0].as_matrix())
 J=np.array(j['origin']);joint=UsdPhysics.RevoluteJoint.Define(s,'/Robot/joints/'+j['name']);joint.CreateBody0Rel().SetTargets([bodypaths[j['parent']]]);joint.CreateBody1Rel().SetTargets([bodypaths[j['child']]])
 joint.CreateAxisAttr('X');joint.CreateLocalPos0Attr(Gf.Vec3f(*J[:3,3]));joint.CreateLocalRot0Attr(quat(J[:3,:3]@A));joint.CreateLocalPos1Attr(Gf.Vec3f(0));joint.CreateLocalRot1Attr(quat(A));joint.CreateLowerLimitAttr(math.degrees(j['limit']['lower']));joint.CreateUpperLimitAttr(math.degrees(j['limit']['upper']));joint.CreateCollisionEnabledAttr(True)
 drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'angular');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(kp[idx]*math.pi/180);drive.CreateDampingAttr(kd[idx]*math.pi/180);drive.CreateMaxForceAttr(j['limit']['effort']);drive.CreateTargetPositionAttr(math.degrees(closed[j['name']]));drive.CreateTargetVelocityAttr(0.)
colliders={};geomap=[]
for g in m['geometry']:
 gp=bodypaths[g['body']]+f"/{g['kind']}/{g['link']}_{g['index']}";typ=g['type']
 if typ=='mesh':
  a=np.load(P/'simulation/usd'/g['mesh']);v=a['vertices'];f=a['faces'][::-1];geom=UsdGeom.Mesh.Define(s,gp);geom.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(v.astype('float32')));geom.CreateFaceVertexCountsAttr([3]*len(f));geom.CreateFaceVertexIndicesAttr(f.reshape(-1).tolist());geom.CreateSubdivisionSchemeAttr('none');geom.CreateDoubleSidedAttr(True)
 elif typ=='cylinder':
  geom=UsdGeom.Cylinder.Define(s,gp);geom.CreateRadiusAttr(float(g['geometry']['radius']));geom.CreateHeightAttr(float(g['geometry']['length']));geom.CreateAxisAttr('Z')
 elif typ=='sphere':geom=UsdGeom.Sphere.Define(s,gp);geom.CreateRadiusAttr(float(g['geometry']['radius']))
 elif typ=='box':
  geom=UsdGeom.Cube.Define(s,gp);geom.CreateSizeAttr(1.)
 else:raise RuntimeError(typ)
 setx(geom,g['local'])
 if typ=='box':UsdGeom.Xformable(geom).AddScaleOp().Set(Gf.Vec3f(*map(float,g['geometry']['size'].split())))
 if g['kind']=='visual':geom.CreateDisplayColorAttr([Gf.Vec3f(.55,.58,.62)] if 'omni_' not in g['link'] else [Gf.Vec3f(.78,.75,.7)])
 else:
  geom.CreateVisibilityAttr('invisible');UsdPhysics.CollisionAPI.Apply(geom.GetPrim()).CreateCollisionEnabledAttr(True)
  pc=PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim());pc.CreateContactOffsetAttr(.0001);pc.CreateRestOffsetAttr(0.)
  if typ=='mesh':
   # Preserve current accepted collision-model triangles; sparse SDF avoids lossy 256-vertex convex cooking.
   UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr('sdf');sdf=PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(geom.GetPrim());sdf.CreateSdfResolutionAttr(521);sdf.CreateSdfSubgridResolutionAttr(6)
  colliders.setdefault(g['link'],[]).append(gp)
 geomap.append({**g,'usd_path':gp})
filtered=[]
for pair in m['reviewed_pairs']:
 a,b=pair['links']
 for ca in colliders.get(a,[]):
  for cb in colliders.get(b,[]):
   UsdPhysics.FilteredPairsAPI.Apply(s.GetPrimAtPath(ca)).CreateFilteredPairsRel().AddTarget(cb);filtered.append([ca,cb])
scene=UsdPhysics.Scene.Define(s,'/PhysicsScene');scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));scene.CreateGravityMagnitudeAttr(9.81);ps=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim());ps.CreateTimeStepsPerSecondAttr(200);ps.CreateSolverTypeAttr('PGS');ps.CreateEnableGPUDynamicsAttr(False);ps.CreateBroadphaseTypeAttr('MBP')
s.GetRootLayer().Save()
(P/'simulation/usd/usd_mapping_fresh.json').write_text(json.dumps({'body_paths':bodypaths,'frame_paths':framepaths,'geometry':geomap,'exact_filtered_collider_pairs':filtered,'gains_SI':{'stiffness':kp,'damping':kd},'sdf_resolution':521},indent=2))

# Explicit initial joint state: body poses alone do not initialize reduced-coordinate articulation q.
for j in m['joints']:
 prim=s.GetPrimAtPath('/Robot/joints/'+j['name']);prim.AddAppliedSchema('PhysicsJointStateAPI:angular');prim.CreateAttribute('state:angular:physics:position',Sdf.ValueTypeNames.Float).Set(math.degrees(closed[j['name']]));prim.CreateAttribute('state:angular:physics:velocity',Sdf.ValueTypeNames.Float).Set(0.)
# Six actual mechanical mimic constraints, not independent hand actuators.
for j in m['joints']:
 if j['mimic']:
  prim=s.GetPrimAtPath('/Robot/joints/'+j['name']);api=PhysxSchema.PhysxMimicJointAPI.Apply(prim,'rotX');api.CreateReferenceJointRel().SetTargets(['/Robot/joints/'+j['mimic']['joint']]);api.CreateReferenceJointAxisAttr('rotX');api.CreateGearingAttr(-float(j['mimic']['multiplier']));api.CreateOffsetAttr(0.);prim.RemoveAPI(UsdPhysics.DriveAPI,'angular')
s.GetRootLayer().Save()
print('ARTICULATED_USD_DONE',len(m['joints']),flush=True)
app.close()
